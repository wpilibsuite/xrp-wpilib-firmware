#pragma once

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

#define IMU_I2C_ADDR 0x6B

namespace xrp {

bool imuReady();
void imuInit(uint8_t addr, TwoWire *theWire);

} // namespace xrp