#pragma once

#define XRP_TAG_MOTOR 0x12
#define XRP_TAG_SERVO 0x13
#define XRP_TAG_DIO 0x14
#define XRP_TAG_ANALOG 0x15
#define XRP_TAG_GYRO 0x16
#define XRP_TAG_ACCEL 0x17
#define XRP_TAG_ENCODER 0x18

namespace wpilibudp {

bool dsWatchdogActive();

bool processPacket(char* buffer, int size);
void resetState();

int writeEncoderData(int deviceId, int count, unsigned period, unsigned divisor, char* buffer, int offset = 0);
int writeDIOData(int deviceId, bool value, char* buffer, int offset = 0);
int writeGyroData(float rates[3], float angles[3], char* buffer, int offset = 0);
int writeAccelData(float accels[3], char* buffer, int offset = 0);
int writeAnalogData(int deviceId, float voltage, char* buffer, int offset = 0);
} // namespace wpilibudp