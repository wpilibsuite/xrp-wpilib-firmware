#pragma once

#include <ArduinoJson.h>

namespace wpilibws {

bool dsWatchdogActive();

void processWSMessage(JsonDocument& jsonMsg);

// Message Encoders
std::string makeEncoderMessage(int deviceId, int count);

} // namespace wpilibws
