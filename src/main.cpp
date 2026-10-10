#include "debug_log.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <SingleFileDrive.h>
#include <Wire.h>

#include <string>
#include <string.h>

#include "bluetooth_transport.h"
#include "byteutils.h"
#include "config.h"
#include "encoder.h"
#include "imu.h"
#include "input_voltage.h"
#include "robot.h"
#include "wpilib_protocol.h"

// Resource strings
extern "C" {
const unsigned char* GetResource_VERSION(size_t* len);
}

// Beta board uses Wire where Production board uses Wire1
#ifdef ARDUINO_SPARKFUN_XRP_CONTROLLER_BETA
  #define MYWIRE Wire
#else
  #define MYWIRE Wire1
#endif

char BLUETOOTH_DEVICE_NAME[32];
char CHIP_ID[20];
char DEFAULT_BLUETOOTH_NAME_SUFFIX[20];

char transportPacketBuf[bluetooth_transport::MAX_PACKET_SIZE];

// Serial diagnostics
unsigned long _lastMessageStatusPrint = 0;
constexpr unsigned long COMMAND_ACK_REPEAT_MS = 500;

unsigned long _avgLoopTimeUs = 0;
unsigned long _loopTimeMeasurementCount = 0;
bool _restartRequested = false;
unsigned long _restartAtMs = 0;
unsigned long _commandAckUntilMs = 0;
uint32_t _lastCommandAckVersion = 0;

uint16_t seq = 0;

// Called only during setup, before Bluetooth and the USB status drive start.
// Write only when identification changes to limit flash wear.
void updateStatusFile() {
  size_t versionLength = 0;
  const auto* version = GetResource_VERSION(&versionLength);
  std::string contents = "Version: ";
  contents.append(reinterpret_cast<const char*>(version), versionLength);
  if (contents.back() != '\n') {
    contents += '\n';
  }
  contents += "Chip ID: ";
  contents += CHIP_ID;
  contents += "\nBluetooth Name: ";
  contents += BLUETOOTH_DEVICE_NAME;
  contents += "\nConfig Version: " + std::to_string(XRP_CONFIG_VERSION);
  contents += "\nConfig File: ";
  contents += XRP_CONFIG_PATH;
  contents += "\nTransport: Bluetooth LE GATT + L2CAP Credit-Based Mode\n";
  contents += "\nLive diagnostics: USB Serial at 115200 baud.\n";

  File existing = LittleFS.open("/status.txt", "r");
  size_t matched = 0;
  while (existing && matched < contents.size() && existing.available() &&
         existing.read() == static_cast<unsigned char>(contents[matched])) {
    matched++;
  }
  bool unchanged = existing && matched == contents.size() && !existing.available();
  existing.close();
  if (unchanged) {
    return;
  }

  File file = LittleFS.open("/status.txt", "w");
  if (!file) {
    debug_log::println("[STATUS] Failed to open status file for writing");
    return;
  }
  size_t written = file.print(contents.c_str());
  file.close();
  if (written != contents.size()) {
    debug_log::println("[STATUS] Failed to write complete status file");
  }
}

uint8_t handleBluetoothDeviceNameRequest(const char* deviceName,
                                         size_t length) {
  if (_restartRequested) {
    return wpilib_protocol::COMMAND_ACK_REJECTED;
  }
  std::string requestedDeviceName(deviceName, length);
  std::string deviceNameSuffix =
      normalizeBluetoothDeviceNameSuffix(requestedDeviceName);
  if (!isValidBluetoothDeviceNameSuffix(deviceNameSuffix)) {
    debug_log::println("[CONFIG] Rejected Bluetooth rename request");
    return wpilib_protocol::COMMAND_ACK_REJECTED;
  }

  // Disable outputs before flash stalls, and serialize LittleFS access with
  // USB status-file reads in the interrupt handler. Normalize the name once.
  xrp::robotSetEnabled(false);
  xrp::imuSetEnabled(false);
  noInterrupts();
  bool saved = saveBluetoothDeviceName(requestedDeviceName,
                                       DEFAULT_BLUETOOTH_NAME_SUFFIX);
  interrupts();
  if (!saved) {
    debug_log::println("[CONFIG] Failed to save Bluetooth rename request");
    return wpilib_protocol::COMMAND_ACK_REJECTED;
  }

  std::string bluetoothDeviceName =
      buildBluetoothDeviceName(deviceNameSuffix);
  strncpy(BLUETOOTH_DEVICE_NAME, bluetoothDeviceName.c_str(),
          sizeof(BLUETOOTH_DEVICE_NAME) - 1);
  BLUETOOTH_DEVICE_NAME[sizeof(BLUETOOTH_DEVICE_NAME) - 1] = '\0';
  debug_log::log("[CONFIG] Bluetooth name changed to %s; rebooting\n",
                BLUETOOTH_DEVICE_NAME);

  _restartRequested = true;
  _restartAtMs = millis() + 500;
  return wpilib_protocol::COMMAND_ACK_SUCCESS;
}

// ==================================================
// Bluetooth Transport Functions
// ==================================================

bool shouldSendCommandAck(unsigned long now) {
  if (!wpilib_protocol::commandAckPending()) {
    _commandAckUntilMs = 0;
    return false;
  }

  uint32_t version = wpilib_protocol::commandAckVersion();
  if (_commandAckUntilMs == 0 || version != _lastCommandAckVersion) {
    _lastCommandAckVersion = version;
    _commandAckUntilMs = now + COMMAND_ACK_REPEAT_MS;
  }

  if (static_cast<long>(now - _commandAckUntilMs) >= 0) {
    wpilib_protocol::clearCommandAck();
    _commandAckUntilMs = 0;
    return false;
  }

  return true;
}

void sendStatusPacket(char* buffer, int size) {
  if (bluetooth_transport::sendPacket(buffer, size)) {
    seq++;
  }
}

void sendData() {
  unsigned long statusBuildMs = millis();
  int size = 0;
  char buffer[512];
  int ptr = 0;
  uint16_t fieldMask = 0;
  bool sendCommandAck = shouldSendCommandAck(statusBuildMs);

  uint16ToNetwork(seq, buffer);
  buffer[2] = wpilib_protocol::lastControlByteReceived();
  ptr = wpilib_protocol::PACKET_HEADER_SIZE;

  if (sendCommandAck &&
      wpilib_protocol::commandAckFieldMask() == wpilib_protocol::CONTROL_DEVICE_NAME) {
    fieldMask |= wpilib_protocol::STATUS_COMMAND_ACK;
    ptr += wpilib_protocol::writeCommandAckData(buffer, ptr);
    uint16ToNetwork(fieldMask, buffer, 3);
    size = ptr;
    sendStatusPacket(buffer, size);
    return;
  }

  // Encoders
  static constexpr uint divisor = xrp::Encoder::getDivisor();
  for (int i = 0; i < 4; i++) {
    int encoderValue = xrp::readEncoderRaw(i);
    uint encoderPeriod = xrp::readEncoderPeriod(i);

    // We want to flip the encoder 0 value (left motor encoder) so that this returns
    // positive values when moving forward.
    if (i == 0) {
      // Use unsigned arithmetic at the signed counter's wrap point.
      encoderValue =
          static_cast<int32_t>(0u - static_cast<uint32_t>(encoderValue));
      if (encoderPeriod != UINT32_MAX) {
        encoderPeriod ^= 1;  // Preserve the invalid-period sentinel.
      }
    }

    fieldMask |= wpilib_protocol::STATUS_ENCODER_0 << i;
    ptr += wpilib_protocol::writeEncoderData(encoderValue, encoderPeriod, divisor,
                                       buffer, ptr);
  }

  // DIO (currently just the button)
  fieldMask |= wpilib_protocol::STATUS_DIO;
  ptr += wpilib_protocol::writeDIOData(0x01, xrp::isUserButtonPressed() ? 0x01 : 0x00,
                                 buffer, ptr);

  // Gyro and accel data
  float gyroRates[3] = {
      xrp::imuGetGyroRateX(),
      xrp::imuGetGyroRateY(),
      xrp::imuGetGyroRateZ(),
  };

  float gyroAngles[3] = {
      xrp::imuGetRoll(),
      xrp::imuGetPitch(),
      xrp::imuGetYaw(),
  };

  float accels[3] = {
      xrp::imuGetAccelX(),
      xrp::imuGetAccelY(),
      xrp::imuGetAccelZ(),
  };

  fieldMask |= wpilib_protocol::STATUS_GYRO;
  ptr += wpilib_protocol::writeGyroData(gyroRates, gyroAngles, buffer, ptr);
  fieldMask |= wpilib_protocol::STATUS_ACCEL;
  ptr += wpilib_protocol::writeAccelData(accels, buffer, ptr);

  if (xrp::reflectanceInitialized()) {
    fieldMask |= wpilib_protocol::STATUS_ANALOG_0;
    ptr += wpilib_protocol::writeAnalogData(xrp::getReflectanceLeft5V(), buffer, ptr);
    fieldMask |= wpilib_protocol::STATUS_ANALOG_1;
    ptr += wpilib_protocol::writeAnalogData(xrp::getReflectanceRight5V(), buffer, ptr);
  }

  if (xrp::rangefinderInitialized()) {
    fieldMask |= wpilib_protocol::STATUS_ANALOG_2;
    ptr += wpilib_protocol::writeAnalogData(xrp::getRangefinderDistance5V(), buffer, ptr);
  }

  fieldMask |= wpilib_protocol::STATUS_INPUT_VOLTAGE;
  ptr += wpilib_protocol::writeInputVoltageData(xrp::getInputVoltage(), buffer, ptr);

  fieldMask |= wpilib_protocol::STATUS_TIMING;
  ptr += wpilib_protocol::writeTimingData(buffer, ptr);
  if (sendCommandAck) {
    fieldMask |= wpilib_protocol::STATUS_COMMAND_ACK;
    ptr += wpilib_protocol::writeCommandAckData(buffer, ptr);
  }
  uint16ToNetwork(fieldMask, buffer, 3);

  // ptr should now point to 1 past the last byte
  size = ptr;

  sendStatusPacket(buffer, size);
}

void checkPrintStatus() {
  if (millis() - _lastMessageStatusPrint > 5000) {
    int usedHeap = rp2040.getUsedHeap();
    const auto& btDiag = bluetooth_transport::connectionDiagnostics();
    const auto logCounts = debug_log::counters();
    debug_log::log("t(ms):%lu h:%d bt:%d lt(us):%lu gatt:%d notify:%d "
                  "mtu:%u size:%u ctrl:%lu cccd:%lu q:%lu bN:%lu bM:%lu "
                  "req:%lu cb:%lu imm:%lu sent:%lu drop:%lu rx:%u/%u "
                  "rxmax:%u rxdrop:%lu l2q:%lu l2i:%lu l2s:%lu l2d:%lu "
                  "rej:%lu/%lu/%lu last:%02x/%02x/%02x "
                  "active:%u tx:%u pending:%d requested:%d pending_us:%lu "
                  "l2cid:%04x credits:%u log_drop:%lu log_supp:%lu log_trunc:%lu\n",
                  static_cast<unsigned long>(millis()),
                  usedHeap,
                  bluetooth_transport::connected() ? 1 : 0,
                  _avgLoopTimeUs,
                  btDiag.gattConnected ? 1 : 0,
                  btDiag.gattNotificationsEnabled ? 1 : 0,
                  btDiag.gattPayloadMtu,
                  btDiag.lastGattStatusPacketSize,
                  static_cast<unsigned long>(btDiag.gattControlPacketsReceived),
                  static_cast<unsigned long>(btDiag.gattCccdWrites),
                  static_cast<unsigned long>(btDiag.gattStatusPacketsQueued),
                  static_cast<unsigned long>(
                      btDiag.gattStatusPacketsBlockedNotifications),
                  static_cast<unsigned long>(
                      btDiag.gattStatusPacketsBlockedMtu),
                  static_cast<unsigned long>(btDiag.gattNotificationRequests),
                  static_cast<unsigned long>(btDiag.gattNotificationCallbacks),
                  static_cast<unsigned long>(
                      btDiag.gattNotificationImmediateSends),
                  static_cast<unsigned long>(btDiag.gattNotificationsSent),
                  static_cast<unsigned long>(btDiag.gattNotificationDrops),
                  btDiag.rxQueueUsed,
                  btDiag.rxQueueDepth,
                  btDiag.rxQueueMaxUsed,
                  static_cast<unsigned long>(btDiag.rxPacketsDropped),
                  static_cast<unsigned long>(btDiag.l2capStatusPacketsQueued),
                  static_cast<unsigned long>(btDiag.l2capImmediateSends),
                  static_cast<unsigned long>(btDiag.l2capPacketsSent),
                  static_cast<unsigned long>(btDiag.l2capSendDrops),
                  static_cast<unsigned long>(btDiag.rejectedLeConnections),
                  static_cast<unsigned long>(btDiag.rejectedGattConnections),
                  static_cast<unsigned long>(btDiag.rejectedL2capConnections),
                  btDiag.lastL2capSendResult,
                  btDiag.lastGattNotifyRequestResult,
                  btDiag.lastGattNotifyResult,
                  btDiag.activeTransport,
                  btDiag.txTransport,
                  btDiag.txPending ? 1 : 0,
                  btDiag.txCanSendRequested ? 1 : 0,
                  static_cast<unsigned long>(btDiag.txPendingAgeUs),
                  btDiag.l2capChannelId,
                  btDiag.l2capPeerCredits,
                  static_cast<unsigned long>(logCounts.dropped),
                  static_cast<unsigned long>(logCounts.suppressed),
                  static_cast<unsigned long>(logCounts.truncated));
    _lastMessageStatusPrint = millis();
  }
}

void updateLoopTime(unsigned long loopStart) {
  unsigned long loopTime = micros() - loopStart;
  unsigned long totalTime = _avgLoopTimeUs * _loopTimeMeasurementCount;
  _loopTimeMeasurementCount++;

  _avgLoopTimeUs = (totalTime + loopTime) / _loopTimeMeasurementCount;
}

void setupBluetoothTransport() {
  bluetooth_transport::begin(BLUETOOTH_DEVICE_NAME);

  debug_log::println("[BT] Bluetooth transport ready");
}

void setup() {
  // Start Serial port for logging
  Serial.begin(115200);

  // Start LittleFS for read/write from disk
  LittleFS.begin();

  // Set up the I2C pins
  MYWIRE.setSCL(I2C_SCL_1);
  MYWIRE.setSDA(I2C_SDA_1);
  MYWIRE.begin();

  // Give a few seconds if attaching a Serial port listener
  delay(2000);

  // Generate the default Bluetooth name suffix using the flash ID
  pico_unique_board_id_t id_out;
  pico_get_unique_board_id(&id_out);
  snprintf(CHIP_ID, sizeof(CHIP_ID), "%02x%02x-%02x%02x", id_out.id[4],
           id_out.id[5], id_out.id[6], id_out.id[7]);
  snprintf(DEFAULT_BLUETOOTH_NAME_SUFFIX,
           sizeof(DEFAULT_BLUETOOTH_NAME_SUFFIX), "%s", CHIP_ID);

  XRPConfiguration config = loadConfiguration(DEFAULT_BLUETOOTH_NAME_SUFFIX);
  std::string bluetoothDeviceName =
      buildBluetoothDeviceName(config.bluetoothConfig.deviceNameSuffix);
  strncpy(BLUETOOTH_DEVICE_NAME, bluetoothDeviceName.c_str(),
          sizeof(BLUETOOTH_DEVICE_NAME) - 1);
  BLUETOOTH_DEVICE_NAME[sizeof(BLUETOOTH_DEVICE_NAME) - 1] = '\0';
  wpilib_protocol::setDeviceNameHandler(handleBluetoothDeviceNameRequest);

  // MUST BE BEFORE imuCalibrate (has digitalWrites) and Bluetooth startup
  xrp::robotInit();

  // Initialize IMU
  debug_log::println("[IMU] Initializing IMU");
  xrp::imuInit(IMU_I2C_ADDR, &MYWIRE);

  debug_log::println("[IMU] Beginning IMU calibration");
  xrp::imuCalibrate(5000);

  // Update identification before Bluetooth callbacks or USB file reads can run.
  updateStatusFile();

  // Setup Bluetooth transport
  setupBluetoothTransport();

  // NOTE: For now, we'll force init the reflectance sensor
  // TODO Enable this via configuration
  xrp::reflectanceInit();

  // NOTE: For now we'll force init the rangefinder
  // TODO enable this via configuration
  xrp::rangefinderInit();

  _lastMessageStatusPrint = millis();
  // Emulates a FAT-formatted USB stick
  // to allow txt file to be read if USB connected
  singleFileDrive.begin("status.txt", "XRP-Status.txt");
}

void loop() {
  unsigned long loopStartTime = micros();

  // A disconnect/reconnect may occur entirely between loop iterations. Read
  // the session and packet atomically so the new peer can start at any sequence.
  static uint32_t previousSession = 0;
  // Bound work so sustained control traffic cannot starve outputs or reboot.
  constexpr unsigned MAX_CONTROL_PACKETS_PER_LOOP = 16;
  for (unsigned i = 0; i < MAX_CONTROL_PACKETS_PER_LOOP; ++i) {
    size_t packetSize = 0;
    uint32_t session;
    bool havePacket = bluetooth_transport::readPacket(
        transportPacketBuf, sizeof(transportPacketBuf), &packetSize, &session);
    if (session != previousSession) {
      previousSession = session;
      wpilib_protocol::resetState();
      wpilib_protocol::clearCommandAck();
      _commandAckUntilMs = 0;
      xrp::robotSetEnabled(false);
      xrp::imuSetEnabled(false);
    }
    if (!havePacket) {
      break;
    }
    if (!_restartRequested) {
      wpilib_protocol::processPacket(transportPacketBuf, static_cast<int>(packetSize));
    }
  }

  if (_restartRequested &&
      static_cast<long>(millis() - _restartAtMs) >= 0) {
    rp2040.restart();
  }

  xrp::imuPeriodic();
  xrp::rangefinderPollForData();

  // Disable the robot when the driver station watchdog times out.
  // Also reset the max sequence number so we can handle reconnects.
  if (!wpilib_protocol::dsWatchdogActive()) {
    wpilib_protocol::resetState();
    xrp::robotSetEnabled(false);
    xrp::imuSetEnabled(false);
  }

  if (xrp::robotPeriodic()) {
    // Package up and send all the data to the Bluetooth client.
    sendData();
  }

  checkPrintStatus();
  debug_log::drain();
  updateLoopTime(loopStartTime);
}

void loop1() {
  if (xrp::rangefinderInitialized()) {
    xrp::rangefinderPeriodic();
  }

  delay(50);
}
