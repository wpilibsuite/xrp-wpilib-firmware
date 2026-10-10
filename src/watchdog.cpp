#include "debug_log.h"

#include "watchdog.h"

#include <Arduino.h>

namespace xrp {

void Watchdog::feed() {
  bool wasSatisfied = _lastSatisfiedState;
  _lastSatisfiedState = true;
  _lastFeedTime = millis();
  if (!wasSatisfied) {
    debug_log::log("[WD:%s] F -> T\n", _name.c_str());
  }
}

bool Watchdog::satisfied() {
  bool wasSatisfied = _lastSatisfiedState;
  _lastSatisfiedState =
      _wdTimeout == 0 || millis() - _lastFeedTime < _wdTimeout;
  if (wasSatisfied != _lastSatisfiedState) {
    debug_log::log("[WD:%s] %s\n", _name.c_str(),
                   _lastSatisfiedState ? "F -> T" : "T -> F");
  }
  return _lastSatisfiedState;
}

void Watchdog::setTimeout(unsigned long timeout) {
  _wdTimeout = timeout;
}

} // namespace xrp