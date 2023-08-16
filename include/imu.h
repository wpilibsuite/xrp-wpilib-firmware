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
void imuCalibrate(unsigned long calibrationTime);

void imuPeriodic();
bool imuDataReady();

float imuGetAccelX();
float imuGetAccelY();
float imuGetAccelZ();

float imuGetGyroRateX();
float imuGetGyroRateY();
float imuGetGyroRateZ();

float imuGetRoll();
float imuGetPitch();
float imuGetYaw();

void imuResetRoll();
void imuResetPitch();
void imuResetYaw();

void gyroReset();

} // namespace xrp