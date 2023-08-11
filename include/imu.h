#pragma once

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

#define IMU_I2C_ADDR 0x6B
#define IMU_UPDATE_RATE_HZ 20

namespace xrp {

bool imuIsReady();

void imuSetEnabled(bool enabled);
bool imuIsEnabled();

void imuInit(uint8_t addr, TwoWire *theWire);

bool imuPeriodic();

float gyroGetAngleX();
float gyroGetRateX();

float gyroGetAngleY();
float gyroGetRateY();

float gyroGetAngleZ();
float gyroGetRateZ();

void gyroReset();

} // namespace xrp