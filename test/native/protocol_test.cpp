#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#include "Arduino.h"
#include "XRPServo.h"
#include "byteutils.h"
#include "robot.h"
#include "wpilib_protocol.h"

using namespace wpilib_protocol;
std::vector<char> packet(uint16_t seq, uint16_t mask = 0,
                         std::initializer_list<uint8_t> payload = {},
                         uint8_t ctrl = 1) {
  std::vector<char> result(PACKET_HEADER_SIZE + payload.size());
  uint16ToNetwork(seq, result.data());
  result[2] = ctrl;
  uint16ToNetwork(mask, result.data(), 3);
  std::copy(payload.begin(), payload.end(),
            result.begin() + PACKET_HEADER_SIZE);
  return result;
}
bool process(std::vector<char> value) {
  return processPacket(value.data(), value.size());
}
bool renameSucceeds = false;
unsigned renameCalls = 0;
uint8_t renameHandler(const char *, size_t) {
  ++renameCalls;
  return renameSucceeds ? COMMAND_ACK_SUCCESS : COMMAND_ACK_REJECTED;
}
int main() {
  testMicros = 0;
  resetState();
  clearCommandAck();
  assert(!dsWatchdogActive());
  assert(process(packet(0)));
  assert(dsWatchdogActive() && xrp::testRobotEnabled);
  assert(!process(packet(0)));
  assert(process(packet(1)));
  assert(!process(packet(0)));
  assert(!process(packet(65535)));
  assert(!process(packet(0x8001)));

  resetState();
  assert(process(packet(65534)));
  assert(process(packet(65535)));
  assert(!process(packet(65535)));
  assert(process(packet(0)));
  assert(process(packet(1)));
  assert(!process(packet(65534)));
  assert(!process(packet(65535)));

  // Malformed packets have no partial effects and do not consume sequence IDs.
  assert(!process(packet(2, CONTROL_MOTOR_0, {0})));
  assert(!process(packet(2, 1u << 13)));
  assert(!process(packet(2, CONTROL_DEVICE_NAME | CONTROL_MOTOR_0)));
  assert(!processPacket(nullptr, 5));
  assert(process(packet(
      2, CONTROL_MOTOR_0 | CONTROL_MOTOR_1 | CONTROL_SERVO_4 | CONTROL_DIO,
      {0x7f, 0xff, 0x80, 0, 255, 2, 2})));
  assert(xrp::testPwm[0] == 1 && xrp::testPwm[1] == -1);
  assert(xrp::testPwm[4] == 1 && xrp::testDio[1]);
  assert(process(packet(3, 0, {}, 0)));
  assert(!xrp::testRobotEnabled);

  // Rename failures queue a NACK and consume the understood command sequence.
  setDeviceNameHandler(renameHandler);
  assert(!process(packet(4, CONTROL_DEVICE_NAME, {1, 'A'})));
  assert(commandAckPending());
  char ack[5];
  assert(writeCommandAckData(ack) == 5);
  assert(networkToUInt16(ack) == 4);
  assert(networkToUInt16(ack, 2) == CONTROL_DEVICE_NAME);
  assert(static_cast<uint8_t>(ack[4]) == COMMAND_ACK_REJECTED);
  assert(!process(packet(4)));
  assert(process(packet(5)));
  auto ackVersion = commandAckVersion();
  renameSucceeds = true;
  assert(process(packet(6, CONTROL_DEVICE_NAME, {1, 'A'})));
  assert(commandAckVersion() != ackVersion);
  assert(commandAckPending());
  assert(writeCommandAckData(ack) == 5);
  assert(networkToUInt16(ack) == 6);
  assert(networkToUInt16(ack, 2) == CONTROL_DEVICE_NAME);
  assert(static_cast<uint8_t>(ack[4]) == COMMAND_ACK_SUCCESS);
  clearCommandAck();
  assert(!commandAckPending());
  assert(!process(packet(6, CONTROL_DEVICE_NAME, {1, 'A'})));
  assert(renameCalls == 2);
  char timing[4];
  testMicros += 120;
  assert(writeTimingData(timing) == 4);
  // Rename advances the command sequence, but not the control timestamp.
  assert(networkToUInt16(timing) == 5 && networkToUInt16(timing, 2) == 12);
  testMicros += 500000;
  assert(!dsWatchdogActive());
  resetState();
  assert(process(packet(0)));

  // Reset must invalidate even a freshly fed watchdog and accept an arbitrary
  // first sequence, including a fast reconnect to zero from either half-space.
  for (uint16_t previous : {100, 40000, 65535}) {
    resetState();
    assert(!dsWatchdogActive());
    assert(process(packet(previous)));
    assert(dsWatchdogActive());
    resetState();
    assert(!dsWatchdogActive());
    assert(process(packet(0)));
  }

  // Identify is a sequenced command, not a motor enable or watchdog feed.
  resetState();
  clearCommandAck();
  xrp::robotSetEnabled(false);
  auto identifyCalls = xrp::testIdentifyCalls;
  assert(!process(packet(100, CONTROL_IDENTIFY, {0})));
  assert(!process(packet(100, CONTROL_IDENTIFY | CONTROL_MOTOR_0, {0, 127})));
  assert(!process(packet(100, CONTROL_IDENTIFY | CONTROL_DEVICE_NAME, {1, 'A'})));
  assert(xrp::testIdentifyCalls == identifyCalls);
  assert(process(packet(100, CONTROL_IDENTIFY)));
  assert(xrp::testIdentifyCalls == identifyCalls + 1);
  assert(!xrp::testRobotEnabled && !dsWatchdogActive());
  assert(commandAckFieldMask() == CONTROL_IDENTIFY);
  assert(writeCommandAckData(ack) == 5);
  assert(networkToUInt16(ack) == 100);
  assert(networkToUInt16(ack, 2) == CONTROL_IDENTIFY);
  assert(static_cast<uint8_t>(ack[4]) == COMMAND_ACK_SUCCESS);
  assert(!process(packet(100, CONTROL_IDENTIFY)));
  assert(!process(packet(99, CONTROL_IDENTIFY)));
  assert(xrp::testIdentifyCalls == identifyCalls + 1);
  assert(process(packet(101, CONTROL_MOTOR_0, {0, 127})));
  testMicros += 400000;
  assert(process(packet(102, CONTROL_IDENTIFY, {}, 0)));
  assert(xrp::testRobotEnabled && xrp::testPwm[0] == 127.0 / 255.0);
  writeTimingData(timing);
  assert(networkToUInt16(timing) == 101);
  assert(networkToUInt16(timing, 2) == 40000);
  testMicros += 100000;
  assert(!dsWatchdogActive());
  clearCommandAck();

  // Check actual big-endian wire bytes and normalized sensor boundaries.
  char encoded[8];
  assert(writeEncoderData(-2, 21, 2000000, encoded) == 8);
  const unsigned char expected[] = {255, 255, 255, 254, 0, 0, 0, 11};
  assert(
      std::equal(encoded, encoded + 8, expected, [](char a, unsigned char b) {
        return static_cast<unsigned char>(a) == b;
      }));
  writeEncoderData(0, UINT32_MAX, 1000000, encoded);
  assert(networkToUInt32(encoded, 4) == UINT32_MAX);
  writeEncoderData(0, 12, 0, encoded);
  assert(networkToUInt32(encoded, 4) == UINT32_MAX);
  writeAnalogData(std::numeric_limits<float>::quiet_NaN(), encoded);
  assert(networkToUInt16(encoded) == 0);
  writeAnalogData(2.5f, encoded);
  assert(networkToUInt16(encoded) == 32768);
  writeAnalogData(6.0f, encoded);
  assert(networkToUInt16(encoded) == 65535);
  for (float voltage : {0.0f, -1.0f, -std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::quiet_NaN()}) {
    assert(writeInputVoltageData(voltage, encoded) == 2);
    assert(networkToUInt16(encoded) == 0);
  }
  std::fill(encoded, encoded + 8, 0x55);
  assert(writeInputVoltageData(7.2506f, encoded, 2) == 2);
  assert(static_cast<uint8_t>(encoded[2]) == 0x1c);
  assert(static_cast<uint8_t>(encoded[3]) == 0x53);
  assert(encoded[1] == 0x55 && encoded[4] == 0x55);
  for (float voltage : {65.535f, 100.0f, std::numeric_limits<float>::infinity()}) {
    writeInputVoltageData(voltage, encoded);
    assert(networkToUInt16(encoded) == UINT16_MAX);
  }
  floatToNetwork(-1.25f, encoded);
  assert(networkToFloat(encoded) == -1.25f);

  resetState();
  testMicros = UINT32_MAX - 49;
  assert(process(packet(0)));
  testMicros = 50;
  writeTimingData(timing);
  assert(networkToUInt16(timing, 2) == 10);
  testMicros += 700000;
  writeTimingData(timing);
  assert(networkToUInt16(timing, 2) == INVALID_CONTROL_RX_AGE_10_US);

  // Exercise the complete degrees -> normalized PWM -> Servo.write path.
  XRPServo servo;
  assert(servo.init(1));
  resetState();
  for (unsigned degrees = 0; degrees <= 180; ++degrees) {
    assert(process(
        packet(degrees, CONTROL_SERVO_4, {static_cast<uint8_t>(degrees)})));
    servo.setValue(xrp::testPwm[4]);
    assert(testServoAngle == static_cast<int>(degrees));
  }

  // Exhaust all fixed-field masks and every truncated length.
  for (uint16_t mask = 0; mask < 512; ++mask) {
    unsigned size = PACKET_HEADER_SIZE;
    for (unsigned bit = 0; bit < 9; ++bit) {
      if (mask & (1u << bit))
        size += bit < 4 || bit == 8 ? 2 : 1;
    }
    auto value = packet(0, mask);
    value.resize(size);
    resetState();
    for (unsigned length = 0; length < size; ++length) {
      assert(!processPacket(value.data(), length));
    }
    assert(processPacket(value.data(), size));
  }
  std::puts("protocol tests passed");
}
