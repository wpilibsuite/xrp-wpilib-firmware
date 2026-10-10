#pragma once

#include <Arduino.h>
#include <vector>
#include <pins.h>

namespace xrp {

void robotInit();
bool robotInitialized();
uint8_t robotPeriodic();

// Robot control
void robotSetEnabled(bool enabled);
bool robotEnabled();

// Encoder Related
void configureEncoder(int deviceId, int chA, int chB);
int readEncoderRaw(int rawDeviceId);
uint readEncoderPeriod(int rawDeviceId);

// PWM Related
void setPwmValue(int wpilibChannel, double value);

// DIO Related
bool isUserButtonPressed();
void setDigitalOutput(int channel, bool value);
// Flash the onboard LED for five seconds without changing control state.
void identifyRobot();

// Line/Reflectance Sensing Related
void reflectanceInit();
bool reflectanceInitialized();
float getReflectanceLeft5V();
float getReflectanceRight5V();

// Rangefinder
void rangefinderInit();
bool rangefinderInitialized();
float getRangefinderDistance5V();
void rangefinderPollForData();
void rangefinderPeriodic();

} // namespace xrp
