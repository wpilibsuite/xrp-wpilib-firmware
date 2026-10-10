#include "input_voltage.h"

#include <Arduino.h>

namespace xrp {

void inputVoltageInit() {
  analogReadResolution(12);
}

float getInputVoltage() {
  // Both boards divide VIN through 100 kohm / 33 kohm resistors.
  constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
  constexpr float VIN_DIVIDER_RATIO = (100.0f + 33.0f) / 33.0f;
  constexpr float ADC_MAX_VALUE = 4095.0f;
  return analogRead(BOARD_VIN_MEASURE) *
         (ADC_REFERENCE_VOLTAGE * VIN_DIVIDER_RATIO / ADC_MAX_VALUE);
}

} // namespace xrp
