#pragma once

#include <Arduino.h>
#include <vector>

// Hardware Pin defs

// XRP Pin Functions
#define XRP_LEFT_MOTOR_EN 7
#define XRP_LEFT_MOTOR_PH 6
#define XRP_RIGHT_MOTOR_EN 15
#define XRP_RIGHT_MOTOR_PH 14
#define XRP_MOTOR_3_EN 3
#define XRP_MOTOR_3_PH 2
#define XRP_MOTOR_4_EN 11
#define XRP_MOTOR_4_PH 10
#define XRP_SERVO_1_PIN 16
#define XRP_SERVO_2_PIN 17

// WPILib Channel mappings
#define WPILIB_ENCODER_L_CH_A 4
#define WPILIB_ENCODER_L_CH_B 5
#define WPILIB_ENCODER_R_CH_A 6
#define WPILIB_ENCODER_R_CH_B 7
#define WPILIB_ENCODER_3_CH_A 8
#define WPILIB_ENCODER_3_CH_B 9
#define WPILIB_ENCODER_4_CH_A 10
#define WPILIB_ENCODER_4_CH_B 11

// Encoder State Machine indices
#define ENC_SM_IDX_MOTOR_L 0
#define ENC_SM_IDX_MOTOR_R 1
#define ENC_SM_IDX_MOTOR_3 2
#define ENC_SM_IDX_MOTOR_4 3

#define XRP_BUILTIN_LED LED_BUILTIN
#define XRP_BUILTIN_BUTTON 22

#define WPILIB_CH_PWM_MOTOR_L 0
#define WPILIB_CH_PWM_MOTOR_R 1
#define WPILIB_CH_PWM_MOTOR_3 2
#define WPILIB_CH_PWM_MOTOR_4 3
#define WPILIB_CH_PWM_SERVO_1 4
#define WPILIB_CH_PWM_SERVO_2 5

#define XRP_SERVO_MIN_PULSE_US 500
#define XRP_SERVO_MAX_PULSE_US 2500

#define XRP_DATA_ENCODER 0x01
#define XRP_DATA_DIO 0x02
#define XRP_DATA_AIO 0x04
#define XRP_DATA_GENERAL 0x08

#define XRP_VIN_MEAS 28

namespace xrp {

void robotInit();
bool robotInitialized();
uint8_t robotPeriodic();
float getVinMeasured();

// Robot control
void robotSetEnabled(bool enabled);

// Encoder Related
void configureEncoder(int deviceId, int chA, int chB);
int readEncoderRaw(int rawDeviceId);
uint readEncoderPeriod(int rawDeviceId);

// PWM Related
void setPwmValue(int wpilibChannel, double value);

// DIO Related
bool isUserButtonPressed();
void setDigitalOutput(int channel, bool value);

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
