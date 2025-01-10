#pragma once

#include <Arduino.h>
#include <vector>

/************************************************
 *********** START Hardware Pin defs ************
*************************************************/
#define __XRP_PIN_UNDEF 127

// LED_BUILTIN is defined in the board to 
// the correct pin 
#define XRP_BUILTIN_LED LED_BUILTIN

// Wpilib Motor mappings
#define WPILIB_CH_PWM_MOTOR_L 0
#define WPILIB_CH_PWM_MOTOR_R 1
#define WPILIB_CH_PWM_MOTOR_3 2
#define WPILIB_CH_PWM_MOTOR_4 3
#define WPILIB_CH_PWM_SERVO_1 4
#define WPILIB_CH_PWM_SERVO_2 5

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

#define XRP_DATA_ENCODER 0x01
#define XRP_DATA_DIO 0x02
#define XRP_DATA_AIO 0x04
#define XRP_DATA_GENERAL 0x08

#ifdef PICO_RP2350

// Non-Beta board
#define PIN_LAYOUT_IDENT "Non-Beta"
#define __XRP_NONBETA

// Motor and Servo Outputs
#define XRP_MOTOR_L_EN  39
#define XRP_MOTOR_L_PH  38
#define XRP_MOTOR_R_EN  31
#define XRP_MOTOR_R_PH  30
#define XRP_MOTOR_3_EN  23
#define XRP_MOTOR_3_PH  22
#define XRP_MOTOR_4_EN  11
#define XRP_MOTOR_4_PH  10
#define XRP_SERVO_1     25
#define XRP_SERVO_2     7
#define XRP_SERVO_3     24
#define XRP_SERVO_4     6

#define NUM_OF_SERVOS 4

// Encoders
#define XRP_ENC_L_A     36
#define XRP_ENC_L_B     37
#define XRP_ENC_R_A     28
#define XRP_ENC_R_B     29
#define XRP_ENC_3_A     20
#define XRP_ENC_3_B     21
#define XRP_ENC_4_A     8
#define XRP_ENC_4_B     9

// LED Is this needed?
//#define XRP_BUILTIN_LED 32

// I2C
#define XRP_SCL 27
#define XRP_SDA 26

// User Button
#define XRP_USER_BUTTON 2

#else
// Beta
#define PIN_LAYOUT_IDENT "Beta"

// XRP Pin Functions
#define XRP_MOTOR_L_EN 7
#define XRP_MOTOR_L_PH 6
#define XRP_MOTOR_R_EN 15
#define XRP_MOTOR_R_PH 14
#define XRP_MOTOR_3_EN 3
#define XRP_MOTOR_3_PH 2
#define XRP_MOTOR_4_EN 11
#define XRP_MOTOR_4_PH 10
#define XRP_SERVO_1    16
#define XRP_SERVO_2    17
#define XRP_SERVO_3    __XRP_PIN_UNDEF
#define XRP_SERVO_4    __XRP_PIN_UNDEF

#define NUM_OF_SERVOS 2

// I2C
#define XRP_SCL 19
#define XRP_SDA 18

// User Button
#define XRP_USER_BUTTON 22

#endif

/************************************************
 ************ END Hardware Pin defs *************
*************************************************/

namespace xrp {

void robotInit();
bool robotInitialized();
uint8_t robotPeriodic();

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
