// Use the installed BTstack declarations/event accessors, with a fake radio
// that can retain SDU buffers and deliver callbacks either inline or later.
#include "Arduino.h"
#include "BluetoothLock.h"
#include "bluetooth_transport.h"
#include "byteutils.h"
#include "config.h"
#include "debug_log.h"
#include "imu.h"
#include "LittleFS.h"
#include "robot.h"
#include "wpilib_protocol.h"
#include "tusb.h"

void loop();
void sendData();
bool shouldSendCommandAck(unsigned long now);
void updateStatusFile();
uint8_t handleBluetoothDeviceNameRequest(const char*, size_t);
extern char DEFAULT_BLUETOOTH_NAME_SUFFIX[20];
extern char CHIP_ID[20];
extern char BLUETOOTH_DEVICE_NAME[32];
extern bool _restartRequested;
extern unsigned long _restartAtMs;
extern "C" const unsigned char* GetResource_VERSION(size_t* len) {
  *len = 4;
  return reinterpret_cast<const unsigned char*>("test");
}
extern "C" {
#include "ble/att_db_util.h"
#include "ble/att_server.h"
#include "btstack_event.h"
#include "gap.h"
#include "hci.h"
#include "l2cap.h"
}
#include <algorithm>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>

namespace {
btstack_packet_handler_t hciHandler;
btstack_packet_handler_t attHandler;
btstack_packet_handler_t channelHandler;
att_service_handler_t* service;
btstack_context_callback_registration_t* notifyCallback = nullptr;
constexpr uint16_t connection = 0x40;
constexpr uint16_t cid = 0x50;
uint16_t attMtu = 92;
const uint8_t* borrowedSdu = nullptr;
uint16_t borrowedSize = 0;
std::vector<std::vector<uint8_t>> sentL2cap;
std::vector<std::vector<uint8_t>> sentGatt;
bool l2capRequested = false;
bool inlineL2cap = false;
bool inlineGatt = false;
bool gattReady = true;
uint8_t l2capRequestResult = ERROR_CODE_SUCCESS;
uint8_t gattRequestResult = ERROR_CODE_SUCCESS;
unsigned accepts = 0;
unsigned declines = 0;
uint16_t disconnectedHandle = 0xffff;

void emit(btstack_packet_handler_t handler, std::vector<uint8_t> event) {
  BluetoothLock lock;  // Radio callbacks execute under the async context lock.
  event[1] = event.size() - 2;
  handler(HCI_EVENT_PACKET, 0, event.data(), event.size());
}
void canSend(uint16_t channelId) {
  emit(channelHandler,
       {L2CAP_EVENT_CAN_SEND_NOW, 0, static_cast<uint8_t>(channelId),
        static_cast<uint8_t>(channelId >> 8)});
}
void completeL2cap() {
  assert(borrowedSdu);
  sentL2cap.emplace_back(borrowedSdu, borrowedSdu + borrowedSize);
  borrowedSdu = nullptr;
  if (l2capRequested) {
    l2capRequested = false;
    canSend(cid);
  }
}
void completeGattRequest() {
  assert(notifyCallback);
  auto* callback = notifyCallback;
  notifyCallback = nullptr;
  gattReady = true;
  BluetoothLock lock;
  callback->callback(callback->context);
}
void connectLe() {
  std::vector<uint8_t> event(21);
  event[0] = HCI_EVENT_LE_META;
  event[2] = HCI_SUBEVENT_LE_CONNECTION_COMPLETE;
  little_endian_store_16(event.data(), 4, connection);
  little_endian_store_16(event.data(), 14, 6);
  emit(hciHandler, event);
}
void connectGatt(uint16_t handle = connection) {
  std::vector<uint8_t> event(11);
  event[0] = ATT_EVENT_CONNECTED;
  little_endian_store_16(event.data(), 9, handle);
  emit(attHandler, event);
}
void updateConnection(uint16_t handle, uint16_t interval) {
  std::vector<uint8_t> event(12);
  event[0] = HCI_EVENT_LE_META;
  event[2] = HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE;
  little_endian_store_16(event.data(), 4, handle);
  little_endian_store_16(event.data(), 6, interval);
  emit(hciHandler, event);
}
void incomingL2cap(uint16_t channelId = cid) {
  std::vector<uint8_t> event(19);
  event[0] = L2CAP_EVENT_CBM_INCOMING_CONNECTION;
  little_endian_store_16(event.data(), 9, connection);
  little_endian_store_16(event.data(), 11, bluetooth_transport::LE_PSM);
  little_endian_store_16(event.data(), 13, channelId);
  emit(channelHandler, event);
}
void openL2cap(uint16_t channelId = cid, uint16_t mtu = 512) {
  std::vector<uint8_t> event(23);
  event[0] = L2CAP_EVENT_CBM_CHANNEL_OPENED;
  little_endian_store_16(event.data(), 10, connection);
  little_endian_store_16(event.data(), 15, channelId);
  little_endian_store_16(event.data(), 21, mtu);
  emit(channelHandler, event);
}
int write(uint16_t attribute, std::vector<uint8_t> value,
          uint16_t handle = connection,
          uint16_t transaction = ATT_TRANSACTION_MODE_NONE,
          uint16_t offset = 0) {
  BluetoothLock lock;
  return service->write_callback(handle, attribute, transaction, offset,
                                 value.data(), value.size());
}
void disconnect(uint16_t handle = connection) {
  if (handle == connection) {
    // BTstack releases per-connection callback lists and SDUs on disconnect.
    borrowedSdu = nullptr;
    notifyCallback = nullptr;
    l2capRequested = false;
  }
  emit(hciHandler,
       {HCI_EVENT_DISCONNECTION_COMPLETE, 0, 0, static_cast<uint8_t>(handle),
        static_cast<uint8_t>(handle >> 8), 0x13});
}
void receive(std::initializer_list<uint8_t> bytes) {
  std::vector<uint8_t> value(bytes);
  BluetoothLock lock;
  channelHandler(L2CAP_DATA_PACKET, cid, value.data(), value.size());
}
void control(uint16_t seq, uint8_t ctrl = 1) {
  receive({static_cast<uint8_t>(seq >> 8), static_cast<uint8_t>(seq), ctrl, 0, 0});
}
void rename(uint16_t seq, const std::string& name) {
  std::vector<uint8_t> value(wpilib_protocol::PACKET_HEADER_SIZE + 1 + name.size());
  uint16ToNetwork(seq, reinterpret_cast<char*>(value.data()));
  uint16ToNetwork(wpilib_protocol::CONTROL_DEVICE_NAME,
                  reinterpret_cast<char*>(value.data()), 3);
  value[5] = name.size();
  std::copy(name.begin(), name.end(), value.begin() + 6);
  BluetoothLock lock;
  channelHandler(L2CAP_DATA_PACKET, cid, value.data(), value.size());
}
}  // namespace

// Fake implementations keep the actual library signatures checked by C++.
extern "C" {
void little_endian_store_16(uint8_t* buffer, uint16_t pos, uint16_t value) {
  buffer[pos] = value;
  buffer[pos + 1] = value >> 8;
}
uint16_t little_endian_read_16(const uint8_t* buffer, int pos) {
  return buffer[pos] | (static_cast<uint16_t>(buffer[pos + 1]) << 8);
}
void reverse_128(const uint8_t* source, uint8_t* dest) {
  std::reverse_copy(source, source + 16, dest);
}
HCI_STATE hci_get_state() {
  requireBluetoothLock();
  return HCI_STATE_WORKING;
}
void hci_add_event_handler(
    btstack_packet_callback_registration_t* registration) {
  requireBluetoothLock();
  hciHandler = registration->callback;
}
void l2cap_add_event_handler(btstack_packet_callback_registration_t*) {
  requireBluetoothLock();
}
void att_server_register_packet_handler(btstack_packet_handler_t handler) {
  requireBluetoothLock();
  attHandler = handler;
}
void att_server_register_service_handler(att_service_handler_t* handler) {
  requireBluetoothLock();
  service = handler;
}
uint16_t att_db_util_add_service_uuid16(uint16_t) {
  requireBluetoothLock();
  return 1;
}
uint16_t att_db_util_add_characteristic_uuid16(uint16_t, uint16_t, uint8_t,
                                               uint8_t, uint8_t*, uint16_t) {
  requireBluetoothLock();
  return 2;
}
uint16_t att_read_callback_handle_blob(const uint8_t* blob, uint16_t size,
                                       uint16_t offset, uint8_t* buffer,
                                       uint16_t bufferSize) {
  if (!buffer) return size;
  if (offset > size) return 0;
  uint16_t count = std::min<uint16_t>(size - offset, bufferSize);
  std::memcpy(buffer, blob + offset, count);
  return count;
}
uint16_t att_server_get_mtu(hci_con_handle_t) {
  requireBluetoothLock();
  return attMtu;
}
int att_server_can_send_packet_now(hci_con_handle_t) {
  requireBluetoothLock();
  return gattReady;
}
uint8_t att_server_request_to_send_notification(
    btstack_context_callback_registration_t* callback, hci_con_handle_t) {
  requireBluetoothLock();
  if (gattRequestResult) return gattRequestResult;
  assert(!notifyCallback);  // A registration must never be inserted twice.
  notifyCallback = callback;
  if (inlineGatt) completeGattRequest();
  return ERROR_CODE_SUCCESS;
}
uint8_t att_server_notify(hci_con_handle_t, uint16_t, const uint8_t* data,
                          uint16_t size) {
  requireBluetoothLock();
  assert(gattReady);
  sentGatt.emplace_back(data, data + size);
  return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_cbm_register_service(btstack_packet_handler_t handler, uint16_t,
                                   gap_security_level_t) {
  requireBluetoothLock();
  channelHandler = handler;
  return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_cbm_accept_connection(uint16_t, uint8_t*, uint16_t, uint16_t) {
  requireBluetoothLock();
  ++accepts;
  return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_cbm_decline_connection(uint16_t, uint16_t) {
  requireBluetoothLock();
  ++declines;
  return ERROR_CODE_SUCCESS;
}
bool l2cap_can_send_packet_now(uint16_t) {
  requireBluetoothLock();
  return !borrowedSdu;
}
uint8_t l2cap_request_can_send_now_event(uint16_t) {
  requireBluetoothLock();
  if (l2capRequestResult) return l2capRequestResult;
  l2capRequested = true;
  if (inlineL2cap) {
    assert(borrowedSdu);
    completeL2cap();
  }
  return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_send(uint16_t, const uint8_t* data, uint16_t size) {
  requireBluetoothLock();
  assert(!borrowedSdu);
  borrowedSdu = data;
  borrowedSize = size;
  return ERROR_CODE_SUCCESS;
}
uint8_t l2cap_disconnect(uint16_t) {
  requireBluetoothLock();
  return ERROR_CODE_SUCCESS;
}
uint16_t l2cap_cbm_available_credits(uint16_t) {
  requireBluetoothLock();
  return 0;
}
uint8_t gap_disconnect(hci_con_handle_t handle) {
  requireBluetoothLock();
  disconnectedHandle = handle;
  return ERROR_CODE_SUCCESS;
}
void gap_local_bd_addr(bd_addr_t address) {
  requireBluetoothLock();
  std::memset(address, 0, 6);
}
char* bd_addr_to_str(const bd_addr_t) {
  static char address[] = "00:00:00:00:00:00";
  return address;
}
}

int main() {
  using namespace bluetooth_transport;

  // Identification is refreshed at boot only when the saved contents change.
  std::strcpy(CHIP_ID, "AAAA-BBBB");
  std::strcpy(BLUETOOTH_DEVICE_NAME, "WPIXRP-Bot");
  updateStatusFile();
  assert(LittleFS.writeOpens == 1);
  const auto identification = LittleFS.files.at("/status.txt");
  assert(identification.find("Version: test\n") != std::string::npos);
  assert(identification.find("Chip ID: AAAA-BBBB\n") != std::string::npos);
  assert(identification.find("Bluetooth Name: WPIXRP-Bot\n") !=
         std::string::npos);
  assert(identification.find("USB Serial at 115200 baud") != std::string::npos);
  assert(identification.find("Uptime") == std::string::npos);
  assert(identification.find("Packet Counters") == std::string::npos);
  auto writes = LittleFS.writeOpens;
  updateStatusFile();
  assert(LittleFS.writeOpens == writes);

  // Replace files with extra data, truncated contents, or a different version.
  auto oldVersion = identification;
  oldVersion.replace(oldVersion.find("test"), 4, "old!");
  for (const auto& stale : {identification + "extra data\n",
                            identification.substr(0, 20), oldVersion}) {
    LittleFS.files["/status.txt"] = stale;
    updateStatusFile();
    assert(LittleFS.writeOpens == ++writes);
    assert(LittleFS.files.at("/status.txt") == identification);
  }
  std::strcpy(BLUETOOTH_DEVICE_NAME, "WPIXRP-Renamed");
  updateStatusFile();
  assert(LittleFS.writeOpens == ++writes);
  assert(LittleFS.files.at("/status.txt").find("WPIXRP-Renamed\n") !=
         std::string::npos);
  updateStatusFile();
  assert(LittleFS.writeOpens == writes);

  begin("WPIXRP-1234567890123456789");
  const auto ad = advertisementDiagnostics();
  assert(ad.advertisingDataLength == 31 && !ad.advertisingDataOverflow);
  assert(ad.scanResponseDataLength == 24 && !ad.scanResponseDataOverflow);
  assert(std::memcmp(ad.advertisingData + 5, "WPIXRP-1234567890123456789",
                     26) == 0);
  assert(ad.scanResponseData[2] == 0x3f && ad.scanResponseData[17] == 0x7d);
  assert(!connected());
  // Runtime diagnostics must not write to the filesystem.
  for (int i = 0; i < 3; ++i) {
    testMicros += 11000000;
    loop();
    assert(LittleFS.writeOpens == writes);
  }
  connectGatt();
  assert(connected());
  const auto handles = connectionDiagnostics();
  const auto session = connectionSession();
  const auto controlHandle = handles.gattControlValueHandle;
  const auto cccHandle = handles.gattStatusCccHandle;

  // Foreign peers and invalid writes cannot alter subscription or receive
  // state.
  connectGatt(connection + 1);
  assert(disconnectedHandle == connection + 1);
  assert(write(controlHandle, {1}, connection + 1) ==
         ATT_ERROR_WRITE_NOT_PERMITTED);
  assert(write(cccHandle, {1, 0}, connection + 1) ==
         ATT_ERROR_WRITE_NOT_PERMITTED);
  assert(write(cccHandle, {1}) == ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);
  assert(write(cccHandle, {1, 0, 0}) ==
         ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);
  assert(write(cccHandle, {2, 0}) == ATT_ERROR_VALUE_NOT_ALLOWED);
  assert(write(controlHandle, {1}, connection, ATT_TRANSACTION_MODE_ACTIVE) ==
         ATT_ERROR_REQUEST_NOT_SUPPORTED);
  assert(write(controlHandle, {1}, connection, ATT_TRANSACTION_MODE_NONE, 1) ==
         ATT_ERROR_INVALID_OFFSET);
  assert(write(controlHandle, {1}) == 0);
  // Preserve the existing control-traffic subscription fallback.
  assert(connectionDiagnostics().gattNotificationsEnabled);
  assert(sendPacket("abc", 3));
  assert(write(cccHandle, {1, 0}) == 0);
  updateConnection(connection, 6);
  updateConnection(connection + 1, 12);
  assert(connectionDiagnostics().connectionInterval == 6);
  {
    BluetoothLock lock;
    uint8_t value = 255;
    assert(service->read_callback(connection, cccHandle, 1, &value, 1) == 1);
    assert(value == 0);
  }
  assert(sendPacket("abc", 3) &&
         sentGatt.back() == std::vector<uint8_t>({'a', 'b', 'c'}));
  assert(!sendPacket(std::string(90, 'x').data(),
                     90));  // ATT MTU 92 -> payload 89.

  // Inline callbacks must not leave a phantom outstanding request.
  gattReady = false;
  inlineGatt = true;
  assert(sendPacket("inline", 6));
  assert(!connectionDiagnostics().txPending &&
         !connectionDiagnostics().txCanSendRequested);
  inlineGatt = false;
  gattReady = false;
  gattRequestResult = ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER;
  assert(sendPacket("error", 5));
  assert(!connectionDiagnostics().txPending &&
         !connectionDiagnostics().txCanSendRequested);
  gattRequestResult = ERROR_CODE_SUCCESS;
  assert(sendPacket("old", 3));
  canSend(cid + 1);
  assert(connectionDiagnostics().txCanSendRequested);
  assert(write(cccHandle, {0, 0}) == 0);
  assert(write(controlHandle, {2}) == 0);
  assert(connectionDiagnostics().gattNotificationsEnabled);
  assert(write(cccHandle, {1, 0}) == 0);
  assert(sendPacket("new",
                    3));  // Reuses the still-outstanding notification callback.
  completeGattRequest();
  assert(sentGatt.back() == std::vector<uint8_t>({'n', 'e', 'w'}));

  // Reserve the one receive buffer while an accepted channel is still opening.
  incomingL2cap();
  incomingL2cap(cid + 1);
  assert(accepts == 1 && declines == 1);
  openL2cap();
  char rx[512];
  size_t size;
  while (readPacket(rx, sizeof(rx), &size)) {
  }
  receive({0x12, 0x34});  // Select L2CAP for status replies.
  assert(readPacket(rx, sizeof(rx), &size) && size == 2);

  // BTstack borrows A across calls; coalescing B/C must not corrupt A.
  assert(sendPacket("AAAAA", 5));
  assert(sendPacket("BB", 2));
  assert(sendPacket("CCC", 3));
  assert(std::memcmp(borrowedSdu, "AAAAA", 5) == 0);
  canSend(cid + 1);  // An unrelated callback cannot clear the active request.
  assert(connectionDiagnostics().txCanSendRequested);
  completeL2cap();
  assert(sentL2cap.back() == std::vector<uint8_t>({'A', 'A', 'A', 'A', 'A'}));
  assert(borrowedSize == 3 && std::memcmp(borrowedSdu, "CCC", 3) == 0);
  inlineL2cap = true;
  assert(sendPacket("inline", 6));
  assert(!connectionDiagnostics().txPending &&
         !connectionDiagnostics().txCanSendRequested);
  inlineL2cap = false;
  l2capRequestResult = L2CAP_LOCAL_CID_DOES_NOT_EXIST;
  assert(sendPacket("error", 5));
  assert(!connectionDiagnostics().txPending &&
         !connectionDiagnostics().txCanSendRequested);
  l2capRequestResult = ERROR_CODE_SUCCESS;
  completeL2cap();

  // Reproduce the Linux capture: an L2CAP SDU is retained with no credits,
  // another status is pending, and the client falls back to GATT on the same
  // LE link without closing L2CAP. New replies must follow GATT controls.
  assert(sendPacket("L2 in flight", 12));
  assert(sendPacket("L2 pending", 10));
  assert(write(controlHandle, {0, 1, 0, 0, 0}) == 0);
  auto gattSentBeforeFallback = sentGatt.size();
  assert(sendPacket("GATT reply", 10));
  assert(sentGatt.size() == gattSentBeforeFallback + 1);
  assert(sentGatt.back() ==
         std::vector<uint8_t>({'G', 'A', 'T', 'T', ' ', 'r', 'e', 'p', 'l', 'y'}));
  assert(borrowedSize == 12 &&
         std::memcmp(borrowedSdu, "L2 in flight", 12) == 0);

  // A delayed callback from the abandoned transport must neither send the
  // wrong packet nor clear the new transport's outstanding callback.
  gattReady = false;
  assert(sendPacket("GATT pending", 12));
  completeL2cap();
  assert(connectionDiagnostics().txPending &&
         connectionDiagnostics().txCanSendRequested);
  assert(sentGatt.size() == gattSentBeforeFallback + 1);
  completeGattRequest();
  assert(sentGatt.back() == std::vector<uint8_t>(
         {'G', 'A', 'T', 'T', ' ', 'p', 'e', 'n', 'd', 'i', 'n', 'g'}));

  // Also support returning to L2CAP while a GATT callback is still queued.
  gattReady = false;
  assert(sendPacket("old GATT", 8));
  receive({0, 2, 0, 0, 0});
  assert(sendPacket("new L2", 6));
  assert(borrowedSize == 6 && std::memcmp(borrowedSdu, "new L2", 6) == 0);
  assert(sendPacket("L2 pending", 10));
  auto gattSentBeforeSwitchBack = sentGatt.size();
  completeGattRequest();
  assert(sentGatt.size() == gattSentBeforeSwitchBack);
  assert(connectionDiagnostics().txPending &&
         connectionDiagnostics().txCanSendRequested);
  completeL2cap();
  assert(borrowedSize == 10 &&
         std::memcmp(borrowedSdu, "L2 pending", 10) == 0);
  completeL2cap();
  while (readPacket(rx, sizeof(rx), &size)) {}

  for (unsigned i = 0; i < 20; ++i) receive({static_cast<uint8_t>(i)});
  auto full = connectionDiagnostics();
  assert(full.rxQueueUsed == full.rxQueueDepth && full.rxPacketsDropped == 5);
  for (unsigned i = 0; i < full.rxQueueDepth; ++i) {
    assert(readPacket(rx, sizeof(rx), &size) && size == 1 &&
           static_cast<unsigned char>(rx[0]) == i);
  }
  assert(!readPacket(rx, sizeof(rx), &size) && size == 0);
  receive({3, 4});
  disconnect(connection + 1);
  assert(connectionSession() == session && connected());
  disconnect();
  connectGatt();
  assert(connectionSession() != session && connected());
  assert(!readPacket(rx, sizeof(rx), &size));
  assert(!connectionDiagnostics().gattNotificationsEnabled);
  assert(!sendPacket("old subscription", 16));

  uint32_t readSession = 0;
  assert(!readPacket(rx, sizeof(rx), &size, &readSession));
  assert(readSession == connectionSession());

  // Closing the packet channel flushes queued controls and advances the
  // session even while the GATT bearer remains connected.
  incomingL2cap();
  openL2cap();
  receive({5});
  assert(readPacket(rx, sizeof(rx), &size));
  assert(write(cccHandle, {1, 0}) == 0);
  assert(sendPacket("in flight", 9));
  assert(sendPacket("pending", 7));
  receive({6});
  auto closingSession = connectionSession();
  borrowedSdu = nullptr;
  l2capRequested = false;
  emit(channelHandler, {L2CAP_EVENT_CHANNEL_CLOSED, 0, cid, 0});
  assert(connectionSession() != closingSession);
  assert(!readPacket(rx, sizeof(rx), &size));
  assert(!connectionDiagnostics().txPending);
  assert(sendPacket("GATT fallback", 13));
  assert(sentGatt.back() ==
         std::vector<uint8_t>({'G', 'A', 'T', 'T', ' ', 'f', 'a', 'l', 'l', 'b',
                               'a', 'c', 'k'}));
  // Exercise the real firmware loop through rapid L2CAP reconnects, with
  // reused handles/CIDs and without waiting for the 500 ms watchdog timeout.
  // Alternate LE-before-ATT and ATT-before-LE event ordering.
  for (uint16_t oldSeq : {100, 40000, 65535}) {
    disconnect();
    connectLe();
    connectGatt();
    incomingL2cap();
    openL2cap();
    control(oldSeq);
    loop();
    assert(xrp::testRobotEnabled && wpilib_protocol::dsWatchdogActive());
    auto oldSession = connectionSession();
    control(static_cast<uint16_t>(oldSeq + 1));  // Still queued at disconnect.
    assert(sendPacket("in flight", 9));
    assert(sendPacket("pending", 7));
    disconnect();
    connectGatt();
    connectLe();
    incomingL2cap();
    openL2cap();
    assert(connectionSession() != oldSession);
    assert(!connectionDiagnostics().txPending);
    assert(!readPacket(rx, sizeof(rx), &size));
    control(0, 0);
    loop();
    assert(!xrp::testRobotEnabled && wpilib_protocol::dsWatchdogActive());
    control(1);
    loop();
    assert(xrp::testRobotEnabled);
    sendData();
    completeL2cap();
    assert(sentL2cap.back()[2] == 1);
    disconnect();
    loop();
    assert(!xrp::testRobotEnabled && !wpilib_protocol::dsWatchdogActive());
  }

  connectLe();
  incomingL2cap();
  openL2cap();
  control(100);
  loop();
  assert(xrp::testRobotEnabled);
  assert(write(cccHandle, {1, 0}) == 0);
  auto channelSession = connectionSession();
  control(101);  // Must be flushed when just the L2CAP channel closes.
  emit(channelHandler, {L2CAP_EVENT_CHANNEL_CLOSED, 0, cid, 0});
  incomingL2cap();
  openL2cap();
  assert(connectionSession() != channelSession);
  control(0, 0);
  loop();
  assert(!xrp::testRobotEnabled && wpilib_protocol::dsWatchdogActive());

  // A stalled USB reader and a full log queue cannot prevent motor commands
  // from being processed or prevent the watchdog from disabling outputs.
  testUsbConnected = true;
  testUsbSpace = 0;
  for (size_t i = 0; i < debug_log::QUEUE_CAPACITY; ++i) {
    debug_log::print("x");
  }
  auto droppedLogs = debug_log::counters().dropped;
  debug_log::println("stalled reader");
  assert(debug_log::counters().dropped == droppedLogs + 1);
  auto usbWrites = testUsbWrites;
  receive({0, 1, 1, 0, 1, 0, 127});  // Enable and set motor 0 to 127.
  loop();
  assert(xrp::testRobotEnabled && xrp::testPwm[0] == 127.0 / 255.0);
  assert(wpilib_protocol::dsWatchdogActive());
  assert(testUsbWrites == usbWrites);
  testMicros += 600000;
  loop();
  assert(!xrp::testRobotEnabled && !xrp::testImuEnabled);
  assert(!wpilib_protocol::dsWatchdogActive() && testUsbWrites == usbWrites);

  // USB drain happens after watchdog shutdown, even when the sink recovers.
  testUsbSpace = 256;
  testDuringUsbWrite = [] {
    assert(!xrp::testRobotEnabled && !xrp::testImuEnabled);
  };
  loop();
  testDuringUsbWrite = nullptr;
  assert(testUsbWrites == usbWrites + 1);
  testUsbConnected = false;

  // Test the status encoder path in main.cpp, including signed wrap and the
  // invalid period sentinel before left-encoder direction reversal.
  xrp::testEncoderCount = INT32_MIN;
  testAnalogValue = 2048;
  sendData();
  completeL2cap();
  auto* status = reinterpret_cast<char*>(sentL2cap.back().data());
  assert(networkToInt32(status, 5) == INT32_MIN);
  assert(networkToUInt32(status, 9) == UINT32_MAX);
  assert((networkToUInt16(status, 3) & wpilib_protocol::STATUS_INPUT_VOLTAGE) != 0);
  // No external analog sensors: VIN follows encoders, DIO, gyro and accel.
  assert(networkToUInt16(status, 5 + 32 + 2 + 24 + 12) == 6652);
  assert(sentL2cap.back().size() == 81);
  assert(LittleFS.writeOpens == writes);

  // All sensors fit in one packet, with voltage immediately before timing.
  xrp::testReflectanceInitialized = true;
  xrp::testRangefinderInitialized = true;
  sendData();
  completeL2cap();
  status = reinterpret_cast<char*>(sentL2cap.back().data());
  assert(sentL2cap.back().size() == 87);
  assert(networkToUInt16(status, 3) == 0x0fff);
  assert(networkToUInt16(status, 81) == 6652);
  assert(networkToUInt16(status, 83) == 0);
  assert(networkToUInt16(status, 85) == wpilib_protocol::INVALID_CONTROL_RX_AGE_10_US);

  // Identify works while disabled and its ACK does not interrupt telemetry.
  auto normalStatusSize = sentL2cap.back().size();
  auto identifyCalls = xrp::testIdentifyCalls;
  receive({0x12, 0x34, 1, 0x40, 0});
  loop();
  assert(xrp::testIdentifyCalls == identifyCalls + 1);
  assert(!xrp::testRobotEnabled && !wpilib_protocol::dsWatchdogActive());
  assert(!_restartRequested && LittleFS.writeOpens == writes);
  sendData();
  completeL2cap();
  assert(sentL2cap.back().size() == normalStatusSize + 5);
  assert(sentL2cap.back().size() == 92);
  status = reinterpret_cast<char*>(sentL2cap.back().data());
  auto statusMask = networkToUInt16(status, 3);
  assert((statusMask & wpilib_protocol::STATUS_COMMAND_ACK) != 0);
  assert((statusMask & wpilib_protocol::STATUS_ENCODER_0) != 0);
  assert((statusMask & wpilib_protocol::STATUS_TIMING) != 0);
  auto ackOffset = sentL2cap.back().size() - 5;
  assert(networkToUInt16(status, ackOffset) == 0x1234);
  assert(networkToUInt16(status, ackOffset + 2) == wpilib_protocol::CONTROL_IDENTIFY);
  assert(status[ackOffset + 4] == wpilib_protocol::COMMAND_ACK_SUCCESS);
  testMicros += 500000;
  sendData();
  completeL2cap();
  assert(sentL2cap.back().size() == normalStatusSize);
  assert(!wpilib_protocol::commandAckPending());

  // Each new ACK gets the full repeat interval, including a new command near
  // expiry of a previous NACK. Failed saves must leave outputs disabled.
  std::strcpy(DEFAULT_BLUETOOTH_NAME_SUFFIX, "AAAA-BBBB");
  wpilib_protocol::setDeviceNameHandler(handleBluetoothDeviceNameRequest);
  control(1);
  loop();
  LittleFS.failOpen = true;
  rename(2, "FailedSave");
  loop();
  assert(!xrp::testRobotEnabled && !_restartRequested);
  assert(shouldSendCommandAck(millis()));
  testMicros += 490000;
  LittleFS.failOpen = false;
  control(3);
  loop();
  assert(xrp::testRobotEnabled);
  auto enableCalls = xrp::testEnableCalls;
  testBeforeFileWrite = [] {
    assert(!xrp::testRobotEnabled && !xrp::testImuEnabled);
    assert(!testInterruptsEnabled);
  };
  // A suffix beginning with WPIXRP- must not lose a second prefix.
  rename(4, "WPIXRP-WPIXRP-Bot");
  control(5);  // A queued enable must not override rename's output shutdown.
  rename(6, "Ignored");
  loop();
  testBeforeFileWrite = nullptr;
  assert(_restartRequested && !xrp::testRobotEnabled);
  assert(xrp::testEnableCalls == enableCalls && testInterruptsEnabled);
  assert(loadConfiguration("AAAA-BBBB").bluetoothConfig.deviceNameSuffix ==
         "WPIXRP-Bot");
  assert(shouldSendCommandAck(millis()));
  testMicros += 20000;
  assert(shouldSendCommandAck(millis()));  // Past the original NACK deadline.
  sendData();
  completeL2cap();
  status = reinterpret_cast<char*>(sentL2cap.back().data());
  assert(networkToUInt16(status, 3) == wpilib_protocol::STATUS_COMMAND_ACK);
  assert(networkToUInt16(status, 5) == 4);
  assert(status[9] == wpilib_protocol::COMMAND_ACK_SUCCESS);
  testMicros = _restartAtMs * 1000;
  loop();
  assert(rp2040.restarts == 1 && !xrp::testRobotEnabled);
  assert(!shouldSendCommandAck(millis()));
  std::puts("transport and firmware loop tests passed");
}
