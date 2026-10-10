#pragma once
#include <cstdint>
#include <limits>
namespace xrp {
inline bool testRobotEnabled = false;
inline unsigned testIdentifyCalls = 0;
inline void identifyRobot() { ++testIdentifyCalls; }
inline double testPwm[8]{};
inline bool testDio[8]{};
inline unsigned testEnableCalls = 0;
inline unsigned testPeriodicCalls = 0;
inline int32_t testEncoderCount = 0;
inline uint32_t testEncoderPeriod = UINT32_MAX;
inline void robotSetEnabled(bool enabled) {
  if (enabled) ++testEnableCalls;
  testRobotEnabled = enabled;
}
inline bool robotEnabled() { return testRobotEnabled; }
inline void robotInit() {}
inline uint8_t robotPeriodic() { ++testPeriodicCalls; return 0; }
inline int readEncoderRaw(int) { return testEncoderCount; }
inline unsigned readEncoderPeriod(int) { return testEncoderPeriod; }
inline bool isUserButtonPressed() { return false; }
inline bool testReflectanceInitialized = false;
inline bool testRangefinderInitialized = false;
inline bool reflectanceInitialized() { return testReflectanceInitialized; }
inline bool rangefinderInitialized() { return testRangefinderInitialized; }
inline void reflectanceInit() {}
inline void rangefinderInit() {}
inline void rangefinderPollForData() {}
inline void rangefinderPeriodic() {}
inline float getReflectanceLeft5V() { return 0; }
inline float getReflectanceRight5V() { return 0; }
inline float getRangefinderDistance5V() { return 0; }
inline void setPwmValue(int channel, double value) { testPwm[channel] = value; }
inline void setDigitalOutput(int channel, bool value) {
  testDio[channel] = value;
}
}  // namespace xrp
