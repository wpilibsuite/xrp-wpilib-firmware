#pragma once

#define XRP_TAG_MOTOR 0x12
#define XRP_TAG_SERVO 0x13
#define XRP_TAG_DIO 0x14
#define XRP_TAG_ANALOG 0x15
#define XRP_TAG_GYRO 0x16

namespace wpilibudp {

bool dsWatchdogActive();

bool processPacket(char* buffer, int size);



} // namespace wpilibudp