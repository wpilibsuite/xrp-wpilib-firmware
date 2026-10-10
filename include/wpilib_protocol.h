#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wpilib_protocol {

constexpr int PACKET_HEADER_SIZE = 5;
constexpr uint32_t ENCODER_PERIOD_DENOMINATOR = 1000000;
constexpr uint32_t CONTROL_RX_AGE_UNIT_US = 10;
constexpr uint16_t INVALID_CONTROL_RX_AGE_10_US = UINT16_MAX;
constexpr int16_t MOTOR_MAX_PWM = 255;
constexpr uint8_t SERVO_MAX_DEGREES = 180;
constexpr uint16_t ANALOG_MAX_VALUE = UINT16_MAX;
constexpr float ANALOG_MAX_VOLTAGE = 5.0f;
constexpr uint16_t INPUT_VOLTAGE_MILLIVOLTS_PER_VOLT = 1000;

constexpr uint16_t CONTROL_MOTOR_0 = 1u << 0;
constexpr uint16_t CONTROL_MOTOR_1 = 1u << 1;
constexpr uint16_t CONTROL_MOTOR_2 = 1u << 2;
constexpr uint16_t CONTROL_MOTOR_3 = 1u << 3;
constexpr uint16_t CONTROL_SERVO_4 = 1u << 4;
constexpr uint16_t CONTROL_SERVO_5 = 1u << 5;
constexpr uint16_t CONTROL_SERVO_6 = 1u << 6;
constexpr uint16_t CONTROL_SERVO_7 = 1u << 7;
constexpr uint16_t CONTROL_DIO = 1u << 8;
constexpr uint16_t CONTROL_IDENTIFY = 1u << 14;
constexpr uint16_t CONTROL_DEVICE_NAME = 1u << 15;
constexpr uint8_t CONTROL_DEVICE_NAME_MAX_LENGTH = 26;
constexpr uint16_t CONTROL_ALL_FIELDS =
    CONTROL_MOTOR_0 | CONTROL_MOTOR_1 | CONTROL_MOTOR_2 | CONTROL_MOTOR_3 |
    CONTROL_SERVO_4 | CONTROL_SERVO_5 | CONTROL_SERVO_6 | CONTROL_SERVO_7 |
    CONTROL_DIO | CONTROL_IDENTIFY | CONTROL_DEVICE_NAME;

constexpr uint16_t STATUS_ENCODER_0 = 1u << 0;
constexpr uint16_t STATUS_ENCODER_1 = 1u << 1;
constexpr uint16_t STATUS_ENCODER_2 = 1u << 2;
constexpr uint16_t STATUS_ENCODER_3 = 1u << 3;
constexpr uint16_t STATUS_DIO = 1u << 4;
constexpr uint16_t STATUS_GYRO = 1u << 5;
constexpr uint16_t STATUS_ACCEL = 1u << 6;
constexpr uint16_t STATUS_ANALOG_0 = 1u << 7;
constexpr uint16_t STATUS_ANALOG_1 = 1u << 8;
constexpr uint16_t STATUS_ANALOG_2 = 1u << 9;
constexpr uint16_t STATUS_INPUT_VOLTAGE = 1u << 10;
constexpr uint16_t STATUS_TIMING = 1u << 11;
constexpr uint16_t STATUS_COMMAND_ACK = 1u << 12;
constexpr uint16_t STATUS_ALL_FIELDS =
    STATUS_ENCODER_0 | STATUS_ENCODER_1 | STATUS_ENCODER_2 | STATUS_ENCODER_3 |
    STATUS_DIO | STATUS_GYRO | STATUS_ACCEL | STATUS_ANALOG_0 | STATUS_ANALOG_1 |
    STATUS_ANALOG_2 | STATUS_INPUT_VOLTAGE | STATUS_TIMING | STATUS_COMMAND_ACK;
constexpr uint8_t COMMAND_ACK_SUCCESS = 0;
constexpr uint8_t COMMAND_ACK_REJECTED = 1;

bool dsWatchdogActive();

using DeviceNameHandler = uint8_t (*)(const char* deviceName, size_t length);

void setDeviceNameHandler(DeviceNameHandler handler);
bool processPacket(char* buffer, int size);
void resetState();
uint8_t lastControlByteReceived();
bool commandAckPending();
uint16_t commandAckFieldMask();
// Changes for each queued ACK, even when it replaces an unexpired ACK.
uint32_t commandAckVersion();
void clearCommandAck();

int writeEncoderData(int count, unsigned period, unsigned divisor, char* buffer,
                     int offset = 0);
int writeDIOData(uint8_t presentMask, uint8_t valueMask, char* buffer,
                 int offset = 0);
int writeGyroData(float rates[3], float angles[3], char* buffer,
                  int offset = 0);
int writeAccelData(float accels[3], char* buffer, int offset = 0);
int writeAnalogData(float voltage, char* buffer, int offset = 0);
int writeInputVoltageData(float voltage, char* buffer, int offset = 0);
int writeTimingData(char* buffer, int offset = 0);
int writeCommandAckData(char* buffer, int offset = 0);
} // namespace wpilib_protocol
