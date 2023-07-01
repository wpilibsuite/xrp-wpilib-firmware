#pragma once

#include <ArduinoJson.h>

namespace wpilibws {

bool dsWatchdogActive();

void processWSMessage(JsonDocument& jsonMsg);

} // namespace wpilibws