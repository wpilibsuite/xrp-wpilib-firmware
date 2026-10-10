#include <Arduino.h>
#include <cassert>
#include <cmath>
#include <cstdio>

#include "input_voltage.h"

int main() {
  // Voltage measurement initializes the ADC without any external sensors.
  xrp::inputVoltageInit();
  assert(testAnalogResolution == 12);
  testAnalogValue = 0;
  assert(xrp::getInputVoltage() == 0.0f);
  assert(testAnalogPin == BOARD_VIN_MEASURE);
  testAnalogValue = 2048;
  assert(std::abs(xrp::getInputVoltage() - 6.651624f) < 0.00001f);
  testAnalogValue = 4095;
  assert(std::abs(xrp::getInputVoltage() - 13.3f) < 0.00001f);
  std::puts("input voltage tests passed");
}
