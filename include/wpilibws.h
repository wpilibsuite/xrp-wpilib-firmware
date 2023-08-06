#pragma once

#include <ArduinoJson.h>

namespace wpilibws {

enum ws_gyro_axis_t {
  AXIS_X,
  AXIS_Y,
  AXIS_Z
};

bool dsWatchdogActive();

void processWSMessage(JsonDocument& jsonMsg);

// Message Encoders
std::string makeEncoderMessage(int deviceId, int count);
std::string makeDIOMessage(int deviceId, bool value);
std::string makeGyroCombinedMessage(float rates[3], float angles[3]);
std::string makeGyroSingleMessage(ws_gyro_axis_t axis, float rate, float angle);

} // namespace wpilibws
