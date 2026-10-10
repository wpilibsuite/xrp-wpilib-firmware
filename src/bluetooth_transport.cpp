#include "debug_log.h"

#include "bluetooth_transport.h"

#include <Arduino.h>
#include <BTstackLib.h>
#include <BluetoothLock.h>
#include <stdint.h>
#include <string.h>

extern "C" {
#include "ble/att_db_util.h"
#include "ble/att_server.h"
#include "bluetooth.h"
#include "bluetooth_data_types.h"
#include "btstack_event.h"
#include "btstack_util.h"
#include "gap.h"
#include "hci.h"
#include "l2cap.h"
}

namespace bluetooth_transport {
namespace {

constexpr uint8_t RX_QUEUE_DEPTH = 16;
constexpr uint16_t INITIAL_CREDITS = L2CAP_LE_AUTOMATIC_CREDITS;
constexpr uint16_t DEFAULT_GATT_PAYLOAD_MTU = 20;
constexpr uint16_t CONNECTION_SUPERVISION_TIMEOUT = 200;  // 2 seconds.

enum class Transport : uint8_t {
  TRANSPORT_NONE,
  TRANSPORT_L2CAP,
  TRANSPORT_GATT,
};

struct PacketSlot {
  uint16_t size = 0;
  uint8_t data[MAX_PACKET_SIZE];
};

PacketSlot rxQueue[RX_QUEUE_DEPTH];
uint8_t rxWriteIndex = 0;
uint8_t rxReadIndex = 0;
bool rxOverflow = false;
uint32_t rxPacketsQueued = 0;
uint32_t rxPacketsDropped = 0;
uint8_t rxMaxQueueUsed = 0;

uint8_t l2capReceiveBuffer[MAX_PACKET_SIZE];
hci_con_handle_t leConnectionHandle = HCI_CON_HANDLE_INVALID;
uint16_t l2capChannelId = 0;
uint16_t pendingL2capChannelId = 0;
uint32_t connectionGeneration = 0;
hci_con_handle_t l2capConnectionHandle = HCI_CON_HANDLE_INVALID;
uint16_t l2capRemoteMtu = MAX_PACKET_SIZE;

hci_con_handle_t gattConnectionHandle = HCI_CON_HANDLE_INVALID;
uint16_t gattControlValueHandle = 0;
uint16_t gattStatusValueHandle = 0;
uint16_t gattStatusCccHandle = 0;
uint16_t gattPayloadMtu = DEFAULT_GATT_PAYLOAD_MTU;
bool gattNotificationsEnabled = false;

// BTstack borrows an L2CAP SDU until it finishes sending it. Keep that
// storage separate from the pending status, which may be coalesced at any time.
uint8_t l2capTxBuffer[MAX_PACKET_SIZE];
uint8_t txBuffer[MAX_PACKET_SIZE];
uint16_t txSize = 0;
uint32_t txPendingSinceMicros = 0;
bool txPending = false;
bool l2capCanSendRequested = false;
bool gattCanSendRequested = false;
Transport txTransport = Transport::TRANSPORT_NONE;
Transport activeTransport = Transport::TRANSPORT_NONE;

btstack_packet_callback_registration_t hciEventRegistration;
btstack_packet_callback_registration_t l2capEventRegistration;
att_service_handler_t gattServiceHandler{};
btstack_context_callback_registration_t gattNotifyRegistration;
AdvertisementDiagnostics advertisementInfo;
ConnectionDiagnostics connectionInfo;
char advertisedDeviceName[32];
// BTstack keeps pointers to advertisement data instead of copying it.
uint8_t advertisingDataBuffer[31];
uint8_t scanResponseDataBuffer[31];
uint8_t preferredConnectionParameters[8];

uint8_t nextQueueIndex(uint8_t index) {
  return static_cast<uint8_t>((index + 1) % RX_QUEUE_DEPTH);
}

uint8_t rxQueueUsed() {
  if (rxWriteIndex >= rxReadIndex) {
    return rxWriteIndex - rxReadIndex;
  }
  return static_cast<uint8_t>(RX_QUEUE_DEPTH - rxReadIndex + rxWriteIndex);
}

uint16_t gattNotificationConfiguration() {
  return gattNotificationsEnabled
             ? GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
             : 0;
}

void clearRxQueue() {
  rxWriteIndex = 0;
  rxReadIndex = 0;
  rxOverflow = false;
  rxMaxQueueUsed = 0;
}

hci_con_handle_t effectiveGattHandle() {
  return gattConnectionHandle != HCI_CON_HANDLE_INVALID ? gattConnectionHandle
                                                        : leConnectionHandle;
}

uint16_t currentGattPayloadMtu() {
  hci_con_handle_t handle = effectiveGattHandle();
  if (handle != HCI_CON_HANDLE_INVALID) {
    uint16_t attMtu = att_server_get_mtu(handle);
    if (attMtu > 3) {
      gattPayloadMtu = attMtu - 3;
    } else if (attMtu > 0) {
      gattPayloadMtu = 0;
    }
  }
  return gattPayloadMtu;
}

void recordTxPendingDuration() {
  if (!txPending) {
    return;
  }

  uint32_t pendingDurationUs =
      static_cast<uint32_t>(micros() - txPendingSinceMicros);
  connectionInfo.lastTxPendingDurationUs = pendingDurationUs;
  if (pendingDurationUs > connectionInfo.maxTxPendingDurationUs) {
    connectionInfo.maxTxPendingDurationUs = pendingDurationUs;
  }
}

bool txCanSendRequested() {
  return (txTransport == Transport::TRANSPORT_L2CAP && l2capCanSendRequested) ||
         (txTransport == Transport::TRANSPORT_GATT && gattCanSendRequested);
}

void finishPendingTx() {
  recordTxPendingDuration();
  txPending = false;
  txTransport = Transport::TRANSPORT_NONE;
  txSize = 0;
  txPendingSinceMicros = 0;
  // Outstanding BTstack callbacks belong to their channel/connection, even
  // when this particular packet is discarded or another transport is selected.
}

void clearPendingTxFor(Transport transport) {
  if (txPending && txTransport == transport) {
    finishPendingTx();
  }
}

void recordConnectionParameters(uint16_t interval, uint16_t latency,
                                uint16_t supervisionTimeout) {
  connectionInfo.connectionInterval = interval;
  connectionInfo.connectionLatency = latency;
  connectionInfo.connectionSupervisionTimeout = supervisionTimeout;
}

bool queueIncomingPacket(const uint8_t* packet, uint16_t size,
                         Transport transport) {
  if (size == 0 || size > MAX_PACKET_SIZE) {
    ++rxPacketsDropped;
    return false;
  }

  uint8_t nextWrite = nextQueueIndex(rxWriteIndex);
  if (nextWrite == rxReadIndex) {
    rxOverflow = true;
    ++rxPacketsDropped;
    return false;
  }

  PacketSlot& slot = rxQueue[rxWriteIndex];
  memcpy(slot.data, packet, size);
  slot.size = size;
  rxWriteIndex = nextWrite;
  ++rxPacketsQueued;
  uint8_t used = rxQueueUsed();
  if (used > rxMaxQueueUsed) {
    rxMaxQueueUsed = used;
  }
  activeTransport = transport;

  return true;
}

void clearL2capConnection() {
  l2capChannelId = 0;
  pendingL2capChannelId = 0;
  l2capCanSendRequested = false;
  l2capConnectionHandle = HCI_CON_HANDLE_INVALID;
  l2capRemoteMtu = MAX_PACKET_SIZE;
  clearPendingTxFor(Transport::TRANSPORT_L2CAP);
  if (activeTransport == Transport::TRANSPORT_L2CAP) {
    activeTransport = Transport::TRANSPORT_NONE;
  }
}

void clearGattConnection() {
  gattConnectionHandle = HCI_CON_HANDLE_INVALID;
  gattCanSendRequested = false;
  gattNotificationsEnabled = false;
  gattPayloadMtu = DEFAULT_GATT_PAYLOAD_MTU;
  clearPendingTxFor(Transport::TRANSPORT_GATT);
  if (activeTransport == Transport::TRANSPORT_GATT) {
    activeTransport = Transport::TRANSPORT_NONE;
  }
}

bool canSendL2cap(size_t packetSize) {
  return l2capChannelId != 0 && packetSize <= l2capRemoteMtu;
}

bool canSendGatt(size_t packetSize) {
  hci_con_handle_t handle = effectiveGattHandle();
  return handle != HCI_CON_HANDLE_INVALID && gattNotificationsEnabled &&
         packetSize <= currentGattPayloadMtu();
}

bool packetFitsTransport(Transport transport, size_t packetSize) {
  switch (transport) {
    case Transport::TRANSPORT_L2CAP:
      return canSendL2cap(packetSize);
    case Transport::TRANSPORT_GATT:
      return canSendGatt(packetSize);
    default:
      return false;
  }
}

Transport selectTxTransport(size_t packetSize) {
  if (activeTransport == Transport::TRANSPORT_L2CAP) {
    if (canSendL2cap(packetSize)) {
      return Transport::TRANSPORT_L2CAP;
    }
    return canSendGatt(packetSize) ? Transport::TRANSPORT_GATT
                                   : Transport::TRANSPORT_NONE;
  }

  if (activeTransport == Transport::TRANSPORT_GATT) {
    if (canSendGatt(packetSize)) {
      return Transport::TRANSPORT_GATT;
    }
    return canSendL2cap(packetSize) ? Transport::TRANSPORT_L2CAP
                                    : Transport::TRANSPORT_NONE;
  }

  if (canSendL2cap(packetSize)) {
    return Transport::TRANSPORT_L2CAP;
  }

  if (canSendGatt(packetSize)) {
    return Transport::TRANSPORT_GATT;
  }

  return Transport::TRANSPORT_NONE;
}

void addGapService(const char* deviceName) {
  att_db_util_add_service_uuid16(GAP_SERVICE_UUID);
  auto deviceNameBytes =
      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(deviceName));
  att_db_util_add_characteristic_uuid16(
      GAP_DEVICE_NAME_UUID, ATT_PROPERTY_READ, ATT_SECURITY_NONE,
      ATT_SECURITY_NONE, deviceNameBytes, strlen(deviceName));

  little_endian_store_16(preferredConnectionParameters, 0,
                         PREFERRED_CONNECTION_INTERVAL_MIN);
  little_endian_store_16(preferredConnectionParameters, 2,
                         PREFERRED_CONNECTION_INTERVAL_MAX);
  little_endian_store_16(preferredConnectionParameters, 4,
                         PREFERRED_SLAVE_LATENCY);
  little_endian_store_16(preferredConnectionParameters, 6,
                         CONNECTION_SUPERVISION_TIMEOUT);
  att_db_util_add_characteristic_uuid16(
      GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS_UUID, ATT_PROPERTY_READ,
      ATT_SECURITY_NONE, ATT_SECURITY_NONE, preferredConnectionParameters,
      sizeof(preferredConnectionParameters));
}

void updateGattPayloadMtu(uint16_t attMtu) {
  if (attMtu > 3) {
    gattPayloadMtu = attMtu - 3;
  } else {
    gattPayloadMtu = 0;
  }
}

void sendPendingL2capPacket() {
  if (!txPending || txTransport != Transport::TRANSPORT_L2CAP) {
    return;
  }

  if (l2capChannelId == 0) {
    return;
  }

  memcpy(l2capTxBuffer, txBuffer, txSize);
  uint8_t result = l2cap_send(l2capChannelId, l2capTxBuffer, txSize);
  connectionInfo.lastL2capSendResult = result;
  if (result == ERROR_CODE_SUCCESS) {
    ++connectionInfo.l2capPacketsSent;
  } else {
    ++connectionInfo.l2capSendDrops;
    debug_log::logLimited(debug_log::Error::L2CAP_SEND,
                          "[BT] L2CAP send failed: 0x%02x\n", result);
  }
  finishPendingTx();
}

void handleL2capCanSendNow(uint16_t eventChannelId) {
  ++connectionInfo.l2capCanSendCallbacks;

  if (eventChannelId != l2capChannelId) {
    return;
  }

  l2capCanSendRequested = false;
  sendPendingL2capPacket();
}

void sendPendingGattNotification() {
  if (!txPending || txTransport != Transport::TRANSPORT_GATT) {
    return;
  }

  hci_con_handle_t handle = effectiveGattHandle();
  if (handle == HCI_CON_HANDLE_INVALID || !gattNotificationsEnabled ||
      txSize > currentGattPayloadMtu()) {
    ++connectionInfo.gattNotificationDrops;
    finishPendingTx();
    return;
  }

  uint8_t result =
      att_server_notify(handle, gattStatusValueHandle, txBuffer, txSize);
  connectionInfo.lastGattNotifyResult = result;
  if (result == ERROR_CODE_SUCCESS) {
    ++connectionInfo.gattNotificationsSent;
  } else {
    ++connectionInfo.gattNotificationDrops;
    debug_log::logLimited(debug_log::Error::GATT_SEND,
                          "[BT] GATT notification send failed: 0x%02x\n", result);
  }
  finishPendingTx();
}

void handleGattCanSendNow(void* context) {
  ++connectionInfo.gattNotificationCallbacks;
  auto handle =
      static_cast<hci_con_handle_t>(reinterpret_cast<uintptr_t>(context));
  if (handle != effectiveGattHandle()) {
    return;
  }
  gattCanSendRequested = false;
  sendPendingGattNotification();
}

void requestCanSend() {
  if (!txPending || txCanSendRequested()) {
    return;
  }

  if (txTransport == Transport::TRANSPORT_L2CAP && l2capChannelId != 0) {
    if (l2cap_can_send_packet_now(l2capChannelId)) {
      ++connectionInfo.l2capImmediateSends;
      sendPendingL2capPacket();
      return;
    }

    // Both request APIs may invoke their callback before returning.
    l2capCanSendRequested = true;
    ++connectionInfo.l2capCanSendRequests;
    uint8_t result = l2cap_request_can_send_now_event(l2capChannelId);
    if (result != ERROR_CODE_SUCCESS) {
      l2capCanSendRequested = false;
      connectionInfo.lastL2capSendResult = result;
      ++connectionInfo.l2capSendDrops;
      finishPendingTx();
    }
    return;
  }

  hci_con_handle_t handle = effectiveGattHandle();
  if (txTransport == Transport::TRANSPORT_GATT &&
      handle != HCI_CON_HANDLE_INVALID && gattNotificationsEnabled) {
    if (att_server_can_send_packet_now(handle)) {
      ++connectionInfo.gattNotificationImmediateSends;
      sendPendingGattNotification();
      return;
    }

    gattNotifyRegistration.callback = &handleGattCanSendNow;
    gattNotifyRegistration.context =
        reinterpret_cast<void*>(static_cast<uintptr_t>(handle));
    gattCanSendRequested = true;
    ++connectionInfo.gattNotificationRequests;
    uint8_t result = att_server_request_to_send_notification(
        &gattNotifyRegistration, handle);
    connectionInfo.lastGattNotifyRequestResult = result;
    if (result != ERROR_CODE_SUCCESS &&
        result != ERROR_CODE_COMMAND_DISALLOWED) {
      gattCanSendRequested = false;
      debug_log::logLimited(debug_log::Error::GATT_REQUEST,
                            "[BT] GATT notification request failed: 0x%02x\n",
                            result);
      ++connectionInfo.gattNotificationDrops;
      finishPendingTx();
    }
  }
}

bool appendAdField(uint8_t* data, uint8_t& pos, uint8_t type,
                   const uint8_t* value, uint8_t valueSize) {
  if (pos + valueSize + 2 > 31) {
    return false;
  }

  data[pos++] = valueSize + 1;
  data[pos++] = type;
  memcpy(&data[pos], value, valueSize);
  pos += valueSize;

  return true;
}

void configureAdvertisement(const char* deviceName, bool appliedAfterHciWorking) {
  AdvertisementDiagnostics diagnostics;
  diagnostics.appliedAfterHciWorking = appliedAfterHciWorking;
  size_t deviceNameLength = strlen(deviceName);

  memset(advertisingDataBuffer, 0, sizeof(advertisingDataBuffer));
  uint8_t advPos = 0;

  uint8_t flags = 0x06;  // LE General Discoverable, BR/EDR not supported.
  diagnostics.advertisingDataOverflow |=
      !appendAdField(advertisingDataBuffer, advPos, BLUETOOTH_DATA_TYPE_FLAGS,
                     &flags, sizeof(flags));

  uint8_t primaryNameMaxLength = static_cast<uint8_t>(31 - advPos - 2);
  uint8_t primaryNameLength =
      static_cast<uint8_t>(deviceNameLength > primaryNameMaxLength
                               ? primaryNameMaxLength
                               : deviceNameLength);
  uint8_t primaryNameType = primaryNameLength == deviceNameLength
                                ? BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME
                                : BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME;
  diagnostics.advertisingDataOverflow |= !appendAdField(
      advertisingDataBuffer, advPos, primaryNameType,
      reinterpret_cast<const uint8_t*>(deviceName), primaryNameLength);

  BTstack.setAdvData(advPos, advertisingDataBuffer);
  diagnostics.advertisingDataLength = advPos;
  memcpy(diagnostics.advertisingData, advertisingDataBuffer, advPos);

  memset(scanResponseDataBuffer, 0, sizeof(scanResponseDataBuffer));
  uint8_t scanPos = 0;

  UUID serviceUuid(GATT_SERVICE_UUID);
  uint8_t serviceUuidLe[16];
  reverse_128(serviceUuid.getUuid(), serviceUuidLe);
  diagnostics.scanResponseDataOverflow |= !appendAdField(
      scanResponseDataBuffer, scanPos,
      BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
      serviceUuidLe, sizeof(serviceUuidLe));

  uint8_t intervalRange[4];
  little_endian_store_16(intervalRange, 0, PREFERRED_CONNECTION_INTERVAL_MIN);
  little_endian_store_16(intervalRange, 2, PREFERRED_CONNECTION_INTERVAL_MAX);
  diagnostics.scanResponseDataOverflow |= !appendAdField(
      scanResponseDataBuffer, scanPos,
      BLUETOOTH_DATA_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE, intervalRange,
      sizeof(intervalRange));

  BTstack.setScanData(scanPos, scanResponseDataBuffer);
  diagnostics.scanResponseDataLength = scanPos;
  memcpy(diagnostics.scanResponseData, scanResponseDataBuffer, scanPos);

  advertisementInfo = diagnostics;
}

void startAdvertising(const char* reason) {
  configureAdvertisement(advertisedDeviceName,
                         hci_get_state() == HCI_STATE_WORKING);
  BTstack.startAdvertising();
  debug_log::log("[BT] Advertising started (%s)\n", reason);
}

void restartAdvertisingIfIdle(const char* reason) {
  if (hci_get_state() != HCI_STATE_WORKING ||
      leConnectionHandle != HCI_CON_HANDLE_INVALID ||
      l2capChannelId != 0 || gattConnectionHandle != HCI_CON_HANDLE_INVALID) {
    return;
  }

  startAdvertising(reason);
}

void disconnectHandle(hci_con_handle_t handle, const char* reason) {
  if (handle == HCI_CON_HANDLE_INVALID) {
    return;
  }

  debug_log::log("[BT] Disconnecting LE link handle=0x%04x (%s)\n",
                handle, reason);
  uint8_t result = gap_disconnect(handle);
  if (result != ERROR_CODE_SUCCESS && result != ERROR_CODE_COMMAND_DISALLOWED) {
    debug_log::log("[BT] LE disconnect request failed: 0x%02x\n", result);
  }
}

void disconnectLeConnection(const char* reason) {
  if (leConnectionHandle == HCI_CON_HANDLE_INVALID) {
    restartAdvertisingIfIdle(reason);
    return;
  }

  disconnectHandle(leConnectionHandle, reason);
}

bool isForeignLeHandle(hci_con_handle_t handle) {
  return leConnectionHandle != HCI_CON_HANDLE_INVALID &&
         handle != leConnectionHandle;
}

bool isForeignGattHandle(hci_con_handle_t handle) {
  return gattConnectionHandle != HCI_CON_HANDLE_INVALID &&
         handle != gattConnectionHandle;
}

void handleLeConnected(hci_con_handle_t handle, uint16_t interval,
                       uint16_t latency, uint16_t supervisionTimeout,
                       const char* eventName) {
  if (isForeignLeHandle(handle)) {
    ++connectionInfo.rejectedLeConnections;
    debug_log::log(
        "[BT] Rejecting extra LE connection handle=0x%04x "
        "active=0x%04x\n",
        handle, leConnectionHandle);
    disconnectHandle(handle, "busy");
    return;
  }

  if (handle == leConnectionHandle) {
    recordConnectionParameters(interval, latency, supervisionTimeout);
    debug_log::log("[BT] Duplicate LE connected event handle=0x%04x (%s)\n",
                  handle, eventName);
    return;
  }

  leConnectionHandle = handle;
  ++connectionGeneration;
  recordConnectionParameters(interval, latency, supervisionTimeout);
  clearL2capConnection();
  clearGattConnection();
  clearRxQueue();
  debug_log::log("[BT] LE connected handle=0x%04x (%s)\n", handle, eventName);
}

uint16_t handleGattRead(hci_con_handle_t handle, uint16_t attributeHandle,
                        uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
  if (handle != effectiveGattHandle() ||
      attributeHandle != gattStatusCccHandle) {
    return 0;
  }

  uint8_t configuration[2];
  little_endian_store_16(configuration, 0, gattNotificationConfiguration());
  return att_read_callback_handle_blob(configuration, sizeof(configuration),
                                       offset, buffer, bufferSize);
}

int handleGattWrite(hci_con_handle_t handle, uint16_t attributeHandle,
                    uint16_t transactionMode, uint16_t offset, uint8_t* buffer,
                    uint16_t bufferSize) {
  if (handle == HCI_CON_HANDLE_INVALID || handle != effectiveGattHandle()) {
    return ATT_ERROR_WRITE_NOT_PERMITTED;
  }
  // Each control write is a complete packet. Prepared/offset writes would
  // otherwise apply fragments before a transaction was committed.
  if (transactionMode != ATT_TRANSACTION_MODE_NONE) {
    return ATT_ERROR_REQUEST_NOT_SUPPORTED;
  }
  if (offset != 0) {
    return ATT_ERROR_INVALID_OFFSET;
  }

  if (attributeHandle == gattControlValueHandle) {
    ++connectionInfo.gattControlPacketsReceived;
    if (bufferSize == 0 || bufferSize > MAX_PACKET_SIZE) {
      return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    }
    // Preserve the existing control-traffic notification fallback.
    if (!gattNotificationsEnabled) {
      gattNotificationsEnabled = true;
      debug_log::println("[BT] GATT status notifications enabled by control traffic");
    }
    queueIncomingPacket(buffer, bufferSize, Transport::TRANSPORT_GATT);
    requestCanSend();
    return 0;
  }

  if (attributeHandle == gattStatusCccHandle) {
    if (bufferSize != 2) {
      return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    }
    uint16_t configuration = little_endian_read_16(buffer, 0);
    if (configuration != 0 &&
        configuration !=
            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION) {
      return ATT_ERROR_VALUE_NOT_ALLOWED;
    }
    ++connectionInfo.gattCccdWrites;
    gattNotificationsEnabled = configuration != 0;
    debug_log::log("[BT] GATT status notifications %s\n",
                  gattNotificationsEnabled ? "enabled" : "disabled");
    if (!gattNotificationsEnabled) {
      clearPendingTxFor(Transport::TRANSPORT_GATT);
    } else {
      requestCanSend();
    }
    return 0;
  }

  return ATT_ERROR_WRITE_NOT_PERMITTED;
}

void addGattService() {
  static UUID serviceUuid(GATT_SERVICE_UUID);
  static UUID controlUuid(GATT_CONTROL_CHARACTERISTIC_UUID);
  static UUID statusUuid(GATT_STATUS_CHARACTERISTIC_UUID);

  BTstack.addGATTService(&serviceUuid);
  gattControlValueHandle = BTstack.addGATTCharacteristicDynamic(
      &controlUuid, ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, 0);
  gattStatusValueHandle =
      BTstack.addGATTCharacteristicDynamic(&statusUuid, ATT_PROPERTY_NOTIFY, 0);
  gattStatusCccHandle = gattStatusValueHandle + 1;

  // The Arduino wrapper discards connection handles, offsets, transaction
  // modes, and write errors. Register the native ATT callbacks after setup().
  gattServiceHandler.start_handle = gattControlValueHandle;
  gattServiceHandler.end_handle = gattStatusCccHandle;
  gattServiceHandler.read_callback = &handleGattRead;
  gattServiceHandler.write_callback = &handleGattWrite;
}

void packetHandler(uint8_t packetType, uint16_t channel, uint8_t* packet,
                   uint16_t size) {
  switch (packetType) {
    case HCI_EVENT_PACKET:
      switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
          if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            startAdvertising("HCI working");
          }
          break;

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
          hci_con_handle_t handle =
              hci_event_disconnection_complete_get_connection_handle(packet);
          uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
          debug_log::log("[BT] LE disconnected handle=0x%04x reason=0x%02x\n",
                        handle, reason);
          if (handle == leConnectionHandle) {
            connectionInfo.lastDisconnectReason = reason;
            ++connectionGeneration;
            leConnectionHandle = HCI_CON_HANDLE_INVALID;
            recordConnectionParameters(0, 0, 0);
            clearL2capConnection();
            clearGattConnection();
            clearRxQueue();
          } else {
            if (handle == l2capConnectionHandle) {
              clearL2capConnection();
            }
            if (handle == gattConnectionHandle) {
              clearGattConnection();
            }
          }
          restartAdvertisingIfIdle("disconnect");
          break;
        }

        case HCI_EVENT_LE_META:
          switch (hci_event_le_meta_get_subevent_code(packet)) {
            case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
              if (hci_subevent_le_connection_complete_get_status(packet) ==
                  ERROR_CODE_SUCCESS) {
                handleLeConnected(
                    hci_subevent_le_connection_complete_get_connection_handle(
                        packet),
                    hci_subevent_le_connection_complete_get_conn_interval(
                        packet),
                    hci_subevent_le_connection_complete_get_conn_latency(
                        packet),
                    hci_subevent_le_connection_complete_get_supervision_timeout(
                        packet),
                    "LE connection complete");
              }
              break;

            case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE_V1:
              if (hci_subevent_le_enhanced_connection_complete_v1_get_status(
                      packet) == ERROR_CODE_SUCCESS) {
                handleLeConnected(
                    hci_subevent_le_enhanced_connection_complete_v1_get_connection_handle(
                        packet),
                    hci_subevent_le_enhanced_connection_complete_v1_get_conn_interval(
                        packet),
                    hci_subevent_le_enhanced_connection_complete_v1_get_conn_latency(
                        packet),
                    hci_subevent_le_enhanced_connection_complete_v1_get_supervision_timeout(
                        packet),
                    "LE enhanced connection complete v1");
              }
              break;

            case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE_V2:
              if (hci_subevent_le_enhanced_connection_complete_v2_get_status(
                      packet) == ERROR_CODE_SUCCESS) {
                handleLeConnected(
                    hci_subevent_le_enhanced_connection_complete_v2_get_connection_handle(
                        packet),
                    hci_subevent_le_enhanced_connection_complete_v2_get_conn_interval(
                        packet),
                    hci_subevent_le_enhanced_connection_complete_v2_get_conn_latency(
                        packet),
                    hci_subevent_le_enhanced_connection_complete_v2_get_supervision_timeout(
                        packet),
                    "LE enhanced connection complete v2");
              }
              break;

            case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
              if (hci_subevent_le_connection_update_complete_get_status(
                      packet) == ERROR_CODE_SUCCESS &&
                  hci_subevent_le_connection_update_complete_get_connection_handle(
                      packet) == leConnectionHandle) {
                recordConnectionParameters(
                    hci_subevent_le_connection_update_complete_get_conn_interval(
                        packet),
                    hci_subevent_le_connection_update_complete_get_conn_latency(
                        packet),
                    hci_subevent_le_connection_update_complete_get_supervision_timeout(
                        packet));
                ++connectionInfo.connectionUpdates;
                debug_log::log(
                    "[BT] Connection interval now %u units, latency %u\n",
                    connectionInfo.connectionInterval,
                    connectionInfo.connectionLatency);
              }
              break;

            default:
              break;
          }
          break;

        case ATT_EVENT_CONNECTED: {
          hci_con_handle_t handle = att_event_connected_get_handle(packet);
          if (isForeignLeHandle(handle) || isForeignGattHandle(handle)) {
            ++connectionInfo.rejectedGattConnections;
            debug_log::log(
                "[BT] Rejecting extra GATT connection handle=0x%04x "
                "active_le=0x%04x active_gatt=0x%04x\n",
                handle, leConnectionHandle, gattConnectionHandle);
            disconnectHandle(handle, "busy GATT");
            break;
          }

          if (leConnectionHandle == HCI_CON_HANDLE_INVALID) {
            handleLeConnected(handle, 0, 0, 0, "ATT connected");
          }
          gattConnectionHandle = handle;
          updateGattPayloadMtu(att_server_get_mtu(gattConnectionHandle));
          debug_log::log("[BT] GATT connected handle=0x%04x mtu_payload=%u\n",
                        gattConnectionHandle, currentGattPayloadMtu());
          break;
        }

        case ATT_EVENT_DISCONNECTED: {
          hci_con_handle_t handle = att_event_disconnected_get_handle(packet);
          if (handle == gattConnectionHandle) {
            debug_log::println("[BT] GATT disconnected");
            clearGattConnection();
          }
          restartAdvertisingIfIdle("GATT disconnect");
          break;
        }

        case ATT_EVENT_MTU_EXCHANGE_COMPLETE: {
          hci_con_handle_t handle =
              att_event_mtu_exchange_complete_get_handle(packet);
          if (handle == gattConnectionHandle) {
            updateGattPayloadMtu(
                att_event_mtu_exchange_complete_get_MTU(packet));
            debug_log::log("[BT] GATT MTU payload=%u\n",
                          currentGattPayloadMtu());
          }
          break;
        }

        case L2CAP_EVENT_CBM_INCOMING_CONNECTION: {
          hci_con_handle_t handle =
              l2cap_event_cbm_incoming_connection_get_handle(packet);
          uint16_t psm = l2cap_event_cbm_incoming_connection_get_psm(packet);
          uint16_t localCid =
              l2cap_event_cbm_incoming_connection_get_local_cid(packet);
          if (psm != LE_PSM) {
            break;
          }

          if (isForeignLeHandle(handle)) {
            ++connectionInfo.rejectedL2capConnections;
            debug_log::log(
                "[BT] Declining L2CAP CBM connection from extra LE "
                "handle=0x%04x active=0x%04x cid=0x%04x\n",
                handle, leConnectionHandle, localCid);
            l2cap_cbm_decline_connection(
                localCid, L2CAP_CBM_CONNECTION_RESULT_NO_RESOURCES_AVAILABLE);
            disconnectHandle(handle, "busy L2CAP");
            break;
          }

          if (l2capChannelId != 0 || pendingL2capChannelId != 0) {
            ++connectionInfo.rejectedL2capConnections;
            debug_log::log(
                "[BT] Declining extra L2CAP CBM connection cid=0x%04x "
                "active=0x%04x\n",
                localCid, l2capChannelId);
            l2cap_cbm_decline_connection(
                localCid, L2CAP_CBM_CONNECTION_RESULT_NO_RESOURCES_AVAILABLE);
            break;
          }

          debug_log::log(
              "[BT] Accepting L2CAP CBM connection cid=0x%04x psm=0x%04x\n",
              localCid, psm);
          // Reserve our single receive buffer before accept: opening may be
          // deferred, or may complete synchronously inside this call.
          pendingL2capChannelId = localCid;
          uint8_t result = l2cap_cbm_accept_connection(
              localCid, l2capReceiveBuffer, sizeof(l2capReceiveBuffer),
              INITIAL_CREDITS);
          if (result != ERROR_CODE_SUCCESS) {
            pendingL2capChannelId = 0;
            debug_log::log("[BT] L2CAP accept failed: 0x%02x\n", result);
          }
          break;
        }

        case L2CAP_EVENT_CBM_CHANNEL_OPENED: {
          uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
          hci_con_handle_t handle =
              l2cap_event_cbm_channel_opened_get_handle(packet);
          uint16_t localCid =
              l2cap_event_cbm_channel_opened_get_local_cid(packet);
          if (localCid == pendingL2capChannelId) {
            pendingL2capChannelId = 0;
          }
          if (status != ERROR_CODE_SUCCESS) {
            debug_log::log("[BT] L2CAP CBM open failed: 0x%02x\n", status);
            if (localCid == l2capChannelId) {
              clearL2capConnection();
            }
            break;
          }

          if (isForeignLeHandle(handle)) {
            ++connectionInfo.rejectedL2capConnections;
            debug_log::log(
                "[BT] Closing L2CAP channel from extra LE handle=0x%04x "
                "active=0x%04x cid=0x%04x\n",
                handle, leConnectionHandle, localCid);
            l2cap_disconnect(localCid);
            disconnectHandle(handle, "busy L2CAP open");
            break;
          }

          if (l2capChannelId != 0 && localCid != l2capChannelId) {
            ++connectionInfo.rejectedL2capConnections;
            debug_log::log(
                "[BT] Closing extra L2CAP channel cid=0x%04x "
                "active=0x%04x\n",
                localCid, l2capChannelId);
            l2cap_disconnect(localCid);
            break;
          }

          if (leConnectionHandle == HCI_CON_HANDLE_INVALID) {
            handleLeConnected(handle, 0, 0, 0, "L2CAP opened");
          }
          l2capConnectionHandle = handle;
          l2capChannelId = localCid;
          l2capRemoteMtu =
              l2cap_event_cbm_channel_opened_get_remote_mtu(packet);

          debug_log::log(
              "[BT] L2CAP CBM channel open cid=0x%04x remote_mtu=%u\n",
              l2capChannelId, l2capRemoteMtu);
          requestCanSend();
          break;
        }

        case L2CAP_EVENT_CHANNEL_CLOSED: {
          uint16_t closedCid = l2cap_event_channel_closed_get_local_cid(packet);
          if (closedCid == pendingL2capChannelId) {
            pendingL2capChannelId = 0;
          }
          bool wasPacketChannel = closedCid == l2capChannelId;
          debug_log::log("[BT] L2CAP channel closed cid=0x%04x\n", closedCid);
          if (wasPacketChannel) {
            ++connectionGeneration;
            clearRxQueue();
            clearL2capConnection();
            if (!gattNotificationsEnabled) {
              disconnectLeConnection("L2CAP close");
            } else {
              restartAdvertisingIfIdle("L2CAP close");
            }
          }
          break;
        }

        case L2CAP_EVENT_CAN_SEND_NOW:
          handleL2capCanSendNow(l2cap_event_can_send_now_get_local_cid(packet));
          break;

        default:
          break;
      }
      break;

    case L2CAP_DATA_PACKET:
      if (channel == l2capChannelId) {
        queueIncomingPacket(packet, size, Transport::TRANSPORT_L2CAP);
      }
      break;

    default:
      break;
  }
}

}  // namespace

void begin(const char* deviceName) {
  BluetoothLock lock;
  leConnectionHandle = HCI_CON_HANDLE_INVALID;
  clearL2capConnection();
  clearGattConnection();
  activeTransport = Transport::TRANSPORT_NONE;
  finishPendingTx();
  clearRxQueue();
  strncpy(advertisedDeviceName, deviceName, sizeof(advertisedDeviceName) - 1);
  advertisedDeviceName[sizeof(advertisedDeviceName) - 1] = '\0';

  addGapService(deviceName);
  addGattService();
  BTstack.setup(deviceName);
  att_server_register_service_handler(&gattServiceHandler);

  hciEventRegistration.callback = &packetHandler;
  hci_add_event_handler(&hciEventRegistration);

  att_server_register_packet_handler(&packetHandler);

  l2capEventRegistration.callback = &packetHandler;
  l2cap_add_event_handler(&l2capEventRegistration);

  uint8_t result = l2cap_cbm_register_service(&packetHandler, LE_PSM, LEVEL_0);
  if (result != ERROR_CODE_SUCCESS) {
    debug_log::log("[BT] Failed to register L2CAP CBM service: 0x%02x\n",
                  result);
  }

  startAdvertising("startup");

  debug_log::log("[BT] Advertising %s, GATT %s, L2CAP LE PSM 0x%04x\n",
                deviceName, GATT_SERVICE_UUID, LE_PSM);
}

AdvertisementDiagnostics advertisementDiagnostics() {
  BluetoothLock lock;
  return advertisementInfo;
}

ConnectionDiagnostics connectionDiagnostics() {
  BluetoothLock lock;
  hci_con_handle_t gattHandle = effectiveGattHandle();
  connectionInfo.leConnected = leConnectionHandle != HCI_CON_HANDLE_INVALID;
  connectionInfo.l2capConnected = l2capChannelId != 0;
  connectionInfo.gattConnected = gattHandle != HCI_CON_HANDLE_INVALID &&
                                 (gattNotificationsEnabled ||
                                  activeTransport == Transport::TRANSPORT_GATT);
  connectionInfo.gattNotificationsEnabled = gattNotificationsEnabled;
  connectionInfo.txPending = txPending;
  connectionInfo.txCanSendRequested = txCanSendRequested();
  connectionInfo.rxOverflow = rxOverflow;
  connectionInfo.rxQueueDepth = RX_QUEUE_DEPTH - 1;
  connectionInfo.rxQueueUsed = rxQueueUsed();
  connectionInfo.rxQueueMaxUsed = rxMaxQueueUsed;
  connectionInfo.activeTransport = static_cast<uint8_t>(activeTransport);
  connectionInfo.txTransport = static_cast<uint8_t>(txTransport);
  connectionInfo.txPendingAgeUs =
      txPending ? static_cast<uint32_t>(micros() - txPendingSinceMicros) : 0;
  connectionInfo.leConnectionHandle = leConnectionHandle;
  connectionInfo.l2capChannelId = l2capChannelId;
  connectionInfo.l2capRemoteMtu = l2capRemoteMtu;
  connectionInfo.l2capPeerCredits =
      l2capChannelId != 0 ? l2cap_cbm_available_credits(l2capChannelId) : 0;
  connectionInfo.l2capCanSendNow =
      l2capChannelId != 0 && l2cap_can_send_packet_now(l2capChannelId);
  connectionInfo.gattConnectionHandle = gattHandle;
  connectionInfo.gattPayloadMtu = currentGattPayloadMtu();
  connectionInfo.gattControlValueHandle = gattControlValueHandle;
  connectionInfo.gattStatusValueHandle = gattStatusValueHandle;
  connectionInfo.gattStatusCccHandle = gattStatusCccHandle;
  connectionInfo.rxPacketsQueued = rxPacketsQueued;
  connectionInfo.rxPacketsDropped = rxPacketsDropped;
  return connectionInfo;
}

bool connected() {
  BluetoothLock lock;
  return leConnectionHandle != HCI_CON_HANDLE_INVALID;
}

uint32_t connectionSession() {
  BluetoothLock lock;
  return connectionGeneration;
}

unsigned hciState() {
  BluetoothLock lock;
  return static_cast<unsigned>(hci_get_state());
}

const char* hciStateName() {
  switch (hciState()) {
    case HCI_STATE_OFF:
      return "OFF";
    case HCI_STATE_INITIALIZING:
      return "INITIALIZING";
    case HCI_STATE_WORKING:
      return "WORKING";
    case HCI_STATE_HALTING:
      return "HALTING";
    case HCI_STATE_SLEEPING:
      return "SLEEPING";
    case HCI_STATE_FALLING_ASLEEP:
      return "FALLING_ASLEEP";
    default:
      return "UNKNOWN";
  }
}

void localAddress(char* buffer, size_t bufferSize) {
  BluetoothLock lock;
  if (bufferSize == 0) {
    return;
  }

  if (hci_get_state() != HCI_STATE_WORKING) {
    snprintf(buffer, bufferSize, "unavailable");
    return;
  }

  bd_addr_t addr;
  gap_local_bd_addr(addr);
  snprintf(buffer, bufferSize, "%s", bd_addr_to_str(addr));
}

bool readPacket(char* buffer, size_t bufferSize, size_t* packetSize,
                uint32_t* session) {
  BluetoothLock lock;
  *packetSize = 0;
  if (session != nullptr) {
    *session = connectionGeneration;
  }
  if (rxReadIndex == rxWriteIndex) {
    return false;
  }

  PacketSlot& slot = rxQueue[rxReadIndex];
  if (slot.size > bufferSize) {
    rxReadIndex = nextQueueIndex(rxReadIndex);
    ++rxPacketsDropped;
    return false;
  }

  memcpy(buffer, slot.data, slot.size);
  *packetSize = slot.size;
  rxReadIndex = nextQueueIndex(rxReadIndex);

  return true;
}

bool sendPacket(const char* buffer, size_t packetSize) {
  BluetoothLock lock;
  ++connectionInfo.statusSendAttempts;
  connectionInfo.lastStatusPacketSize = packetSize;

  if (buffer == nullptr || packetSize == 0 || packetSize > MAX_PACKET_SIZE) {
    ++connectionInfo.statusSendInvalidSizeDrops;
    return false;
  }

  Transport transport = selectTxTransport(packetSize);
  if (txPending && transport != Transport::TRANSPORT_NONE &&
      transport != txTransport) {
    // A client can switch transports without closing the old bearer. Replace
    // the queued status so a credit-starved L2CAP channel cannot block GATT.
    // Keep the old callback registration and any SDU borrowed by BTstack alive.
    finishPendingTx();
  }

  if (txPending) {
    if (packetFitsTransport(txTransport, packetSize)) {
      memcpy(txBuffer, buffer, packetSize);
      txSize = packetSize;
      ++connectionInfo.statusSendCoalesced;
      requestCanSend();
      return true;
    }

    ++connectionInfo.statusSendBusyDrops;
    requestCanSend();
    return false;
  }

  if (transport == Transport::TRANSPORT_NONE) {
    hci_con_handle_t gattHandle = effectiveGattHandle();
    if (gattHandle != HCI_CON_HANDLE_INVALID || gattNotificationsEnabled ||
        activeTransport == Transport::TRANSPORT_GATT) {
      connectionInfo.lastGattStatusPacketSize = packetSize;
      if (!gattNotificationsEnabled) {
        ++connectionInfo.gattStatusPacketsBlockedNotifications;
      } else if (packetSize > currentGattPayloadMtu()) {
        ++connectionInfo.gattStatusPacketsBlockedMtu;
      }
    }
    ++connectionInfo.statusSendNoTransportDrops;
    return false;
  }

  memcpy(txBuffer, buffer, packetSize);
  txSize = packetSize;
  txPendingSinceMicros = micros();
  txTransport = transport;
  txPending = true;
  if (transport == Transport::TRANSPORT_GATT) {
    connectionInfo.lastGattStatusPacketSize = packetSize;
    ++connectionInfo.gattStatusPacketsQueued;
  } else if (transport == Transport::TRANSPORT_L2CAP) {
    ++connectionInfo.l2capStatusPacketsQueued;
  }
  requestCanSend();

  return true;
}

}  // namespace bluetooth_transport
