#pragma once

namespace xrp {

// Configure the ADC for board input voltage measurements.
void inputVoltageInit();
// Read board VIN in volts using the nominal 3.3 V ADC reference.
float getInputVoltage();

} // namespace xrp
