#include "watchdog.h"

#include <Arduino.h>

namespace xrp {

void Watchdog::feed() {
  if (!_lastSatisfiedState) {
    // transitioning from T -> F
    Serial.printf("[WD:%s] F -> T\n", _name.c_str());
  }
  _lastSatisfiedState = true;
  _lastFeedTime = millis();
}

bool Watchdog::satisfied() {
  if (_wdTimeout == 0) {
    if (!_lastSatisfiedState) {
      // Transitioned from false to true
      Serial.printf("[WD:%s] F -> T\n", _name.c_str());
    }
    _lastSatisfiedState = true;
    return true;
  }

  if (millis() - _lastFeedTime < _wdTimeout) {
    if (!_lastSatisfiedState) {
      Serial.printf("[WD:%s] F -> T\n", _name.c_str());
    }
    _lastSatisfiedState = true;
    return true;
  }

  if (_lastSatisfiedState) {
    Serial.printf("[WD:%s] T -> F\n", _name.c_str());
  }
  _lastSatisfiedState = false;
  return false;
}

void Watchdog::setTimeout(unsigned long timeout) {
  _wdTimeout = timeout;
}

} // namespace xrp