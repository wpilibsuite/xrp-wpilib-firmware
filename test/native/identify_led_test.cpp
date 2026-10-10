#include <cassert>
#include <cstdio>

#include "identify_led.h"

int main() {
  xrp::IdentifyLed led;
  assert(!led.value(0));
  led.start(100);
  assert(led.value(100));
  for (uint32_t elapsed = 0; elapsed < 5000; ++elapsed) {
    assert(led.value(100 + elapsed) == ((elapsed / 250) % 2 == 0));
  }
  assert(!led.value(5100));
  assert(!led.value(6000));

  // Changes from the robot program are remembered throughout the override.
  led.set(true);
  led.start(6000);
  assert(!led.value(6250));
  led.set(false);
  assert(led.value(6500));
  led.set(true);
  assert(led.value(11000));
  led.set(false);
  assert(!led.value(11001));

  // A new request restarts the pattern; no periodic network input is needed.
  led.start(12000);
  led.start(16000);
  assert(led.value(17000));
  assert(!led.value(21000));

  // Unsigned elapsed time preserves both phase and expiry across millis wrap.
  uint32_t start = UINT32_MAX - 100;
  led.start(start);
  assert(led.value(start));
  assert(!led.value(start + 250));
  assert(led.value(start + 500));
  assert(!led.value(start + 5000));
  std::puts("identify LED tests passed");
}
