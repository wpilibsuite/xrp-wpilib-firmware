#pragma once
#include "Wire.h"
constexpr int IMU_I2C_ADDR = 0x6b;
namespace xrp {
inline bool testImuEnabled = false;
inline void imuSetEnabled(bool enabled) { testImuEnabled = enabled; }
inline void imuInit(int, TwoWire*) {}
inline void imuCalibrate(unsigned long) {}
inline void imuPeriodic() {}
inline float imuGetGyroRateX() { return 0; }
inline float imuGetGyroRateY() { return 0; }
inline float imuGetGyroRateZ() { return 0; }
inline float imuGetRoll() { return 0; }
inline float imuGetPitch() { return 0; }
inline float imuGetYaw() { return 0; }
inline float imuGetAccelX() { return 0; }
inline float imuGetAccelY() { return 0; }
inline float imuGetAccelZ() { return 0; }
}
