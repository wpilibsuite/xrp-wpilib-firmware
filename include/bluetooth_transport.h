#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bluetooth_transport {

struct AdvertisementDiagnostics {
  uint8_t advertisingData[31]{};
  uint8_t advertisingDataLength = 0;
  bool advertisingDataOverflow = false;
  uint8_t scanResponseData[31]{};
  uint8_t scanResponseDataLength = 0;
  bool scanResponseDataOverflow = false;
  bool appliedAfterHciWorking = false;
};

struct ConnectionDiagnostics {
  bool leConnected = false;
  bool l2capConnected = false;
  bool gattConnected = false;
  bool gattNotificationsEnabled = false;
  bool txPending = false;
  bool txCanSendRequested = false;
  bool rxOverflow = false;
  uint8_t rxQueueDepth = 0;
  uint8_t rxQueueUsed = 0;
  uint8_t rxQueueMaxUsed = 0;
  uint8_t activeTransport = 0;
  uint8_t txTransport = 0;
  uint16_t leConnectionHandle = 0xffff;
  uint16_t connectionInterval = 0;
  uint16_t connectionLatency = 0;
  uint16_t connectionSupervisionTimeout = 0;
  uint16_t l2capChannelId = 0;
  uint16_t l2capRemoteMtu = 0;
  uint16_t l2capPeerCredits = 0;
  bool l2capCanSendNow = false;
  uint16_t gattConnectionHandle = 0xffff;
  uint16_t gattPayloadMtu = 0;
  uint16_t gattControlValueHandle = 0;
  uint16_t gattStatusValueHandle = 0;
  uint16_t gattStatusCccHandle = 0;
  uint16_t lastStatusPacketSize = 0;
  uint16_t lastGattStatusPacketSize = 0;
  uint32_t statusSendAttempts = 0;
  uint32_t statusSendCoalesced = 0;
  uint32_t statusSendBusyDrops = 0;
  uint32_t statusSendNoTransportDrops = 0;
  uint32_t statusSendInvalidSizeDrops = 0;
  uint32_t txPendingAgeUs = 0;
  uint32_t lastTxPendingDurationUs = 0;
  uint32_t maxTxPendingDurationUs = 0;
  uint32_t rxPacketsQueued = 0;
  uint32_t rxPacketsDropped = 0;
  uint32_t rejectedLeConnections = 0;
  uint32_t rejectedGattConnections = 0;
  uint32_t rejectedL2capConnections = 0;
  uint32_t connectionUpdates = 0;
  uint8_t lastDisconnectReason = 0;
  uint32_t l2capStatusPacketsQueued = 0;
  uint32_t l2capCanSendRequests = 0;
  uint32_t l2capCanSendCallbacks = 0;
  uint32_t l2capImmediateSends = 0;
  uint32_t l2capPacketsSent = 0;
  uint32_t l2capSendDrops = 0;
  uint8_t lastL2capSendResult = 0;
  uint32_t gattControlPacketsReceived = 0;
  uint32_t gattCccdWrites = 0;
  uint32_t gattStatusPacketsQueued = 0;
  uint32_t gattStatusPacketsBlockedNotifications = 0;
  uint32_t gattStatusPacketsBlockedMtu = 0;
  uint32_t gattNotificationRequests = 0;
  uint32_t gattNotificationCallbacks = 0;
  uint32_t gattNotificationImmediateSends = 0;
  uint32_t gattNotificationsSent = 0;
  uint32_t gattNotificationDrops = 0;
  uint8_t lastGattNotifyRequestResult = 0;
  uint8_t lastGattNotifyResult = 0;
};

constexpr size_t MAX_PACKET_SIZE = 512;
constexpr unsigned LE_PSM = 0x0081;
constexpr const char* GATT_SERVICE_UUID = "7d2ea28a-f7bd-485d-9d6a-2c3f0b214a3f";
constexpr const char* GATT_CONTROL_CHARACTERISTIC_UUID =
    "7d2ea28b-f7bd-485d-9d6a-2c3f0b214a3f";
constexpr const char* GATT_STATUS_CHARACTERISTIC_UUID =
    "7d2ea28c-f7bd-485d-9d6a-2c3f0b214a3f";
constexpr unsigned PREFERRED_CONNECTION_INTERVAL_MIN = 6;   // 7.5 ms
constexpr unsigned PREFERRED_CONNECTION_INTERVAL_MAX = 12;  // 15 ms
constexpr unsigned PREFERRED_SLAVE_LATENCY = 0;

void begin(const char* deviceName);
AdvertisementDiagnostics advertisementDiagnostics();
ConnectionDiagnostics connectionDiagnostics();
bool connected();
uint32_t connectionSession();
unsigned hciState();
const char* hciStateName();
void localAddress(char* buffer, size_t bufferSize);
// Return the session even when there is no packet, under the receive lock.
// Callers can reset protocol state across disconnect/reconnect between polls.
bool readPacket(char* buffer, size_t bufferSize, size_t* packetSize,
                uint32_t* session = nullptr);
bool sendPacket(const char* buffer, size_t packetSize);

}  // namespace bluetooth_transport
