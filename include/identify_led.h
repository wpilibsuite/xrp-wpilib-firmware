#pragma once

#include <stdint.h>

namespace xrp {

// A temporary identify pattern overrides the latest requested DIO state.
class IdentifyLed {
 public:
  static constexpr uint32_t DURATION_MS = 5000;
  static constexpr uint32_t HALF_PERIOD_MS = 250;

  void set(bool value) { requested = value; }
  void start(uint32_t now) {
    startedAt = now;
    identifying = true;
  }
  bool value(uint32_t now) {
    uint32_t elapsed = now - startedAt;
    if (identifying && elapsed >= DURATION_MS) identifying = false;
    return identifying ? (elapsed / HALF_PERIOD_MS) % 2 == 0 : requested;
  }

 private:
  uint32_t startedAt = 0;
  bool identifying = false;
  bool requested = false;
};

}  // namespace xrp
