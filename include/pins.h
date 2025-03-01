/* This file is used for mapping the GPIO to the software constants based on the board being built. */

#pragma once

#define __XRP_PIN_UNDEF 127

// Ultra Sonic max pulse
#define ULTRASONIC_MAX_PULSE_WIDTH 23200

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
#define WPILIB_CH_PWM_SERVO_3 6
#define WPILIB_CH_PWM_SERVO_4 7

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

#define NUM_OF_ENCODERS 4
#define NUM_OF_SERVOS 4

#define XRP_DATA_ENCODER 0x01
#define XRP_DATA_DIO 0x02
#define XRP_DATA_AIO 0x04
#define XRP_DATA_GENERAL 0x08

#ifdef PICO_RP2350

// Non-Beta board
#define PIN_LAYOUT_IDENT "Non-Beta"
#define __XRP_NONBETA

/*
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

// Encoders
#define XRP_ENC_L_A     36
#define XRP_ENC_L_B     37
#define XRP_ENC_R_A     28
#define XRP_ENC_R_B     29
#define XRP_ENC_3_A     20
#define XRP_ENC_3_B     21
#define XRP_ENC_4_A     8
#define XRP_ENC_4_B     9

// Line Reflectors
#define REFLECT_LEFT_PIN 44
#define REFLECT_RIGHT_PIN 45

// Range Finder
#define ULTRASONIC_TRIG_PIN 0
#define ULTRASONIC_ECHO_PIN 1

// I2C IMU
#define XRP_SCL 27
#define XRP_SDA 26

// User Button
#define XRP_USER_BUTTON 2
*/

// Motors
#define MOTOR_L_IN_1 34
#define MOTOR_L_IN_2 35 
#define MOTOR_R_IN_1 32
#define MOTOR_R_IN_2 33
#define MOTOR_3_IN_1 20
#define MOTOR_3_IN_2 21
#define MOTOR_4_IN_1 10
#define MOTOR_4_IN_2 11

// Encoders
#define MOTOR_L_ENCODER_A     30
#define MOTOR_L_ENCODER_B     31
#define MOTOR_R_ENCODER_A     24
#define MOTOR_R_ENCODER_B     25
#define MOTOR_3_ENCODER_A     22
#define MOTOR_3_ENCODER_B     23
#define MOTOR_4_ENCODER_A     2
#define MOTOR_4_ENCODER_B     3

// SERVOS
#define SERVO_1    6
#define SERVO_2    9
#define SERVO_3    7
#define SERVO_4    8

// Line Reflectors
#define LINE_L 44
#define LINE_R 45

// Range Finder
#define DISTANCE_TRIGGER 0
#define DISTANCE_ECHO    1

// I2C IMU
#define I2C_SCL_1 39
#define I2C_SDA_1 38

// User Button
#define BOARD_USER_BUTTON 36

/*
// Motor and Servo Outputs
#define XRP_MOTOR_L_EN  35
#define XRP_MOTOR_L_PH  34
#define XRP_MOTOR_R_EN  33
#define XRP_MOTOR_R_PH  32
#define XRP_MOTOR_3_EN  21
#define XRP_MOTOR_3_PH  20
#define XRP_MOTOR_4_EN  11
#define XRP_MOTOR_4_PH  10
#define XRP_SERVO_1     6
#define XRP_SERVO_2     9
#define XRP_SERVO_3     7
#define XRP_SERVO_4     8

// Encoders
#define XRP_ENC_L_A     30
#define XRP_ENC_L_B     31
#define XRP_ENC_R_A     24
#define XRP_ENC_R_B     25
#define XRP_ENC_3_A     22
#define XRP_ENC_3_B     23
#define XRP_ENC_4_A     2
#define XRP_ENC_4_B     3

// Line Reflectors
#define REFLECT_LEFT_PIN 44
#define REFLECT_RIGHT_PIN 45

// Range Finder
#define ULTRASONIC_TRIG_PIN 0
#define ULTRASONIC_ECHO_PIN 1

// I2C IMU
#define XRP_SCL 39
#define XRP_SDA 38

// User Button
#define XRP_USER_BUTTON 36
*/

#else
// Beta
#define PIN_LAYOUT_IDENT "Beta"

// XRP Pin Functions

// Motors
#define MOTOR_L_IN_1 6
#define MOTOR_L_IN_2 7 
#define MOTOR_R_IN_1 14
#define MOTOR_R_IN_2 15
#define MOTOR_3_IN_1 2
#define MOTOR_3_IN_2 3
#define MOTOR_4_IN_1 10
#define MOTOR_4_IN_2 11

// Encoders
#define MOTOR_L_ENCODER_A     4
#define MOTOR_L_ENCODER_B     5
#define MOTOR_R_ENCODER_A     12
#define MOTOR_R_ENCODER_B     13
#define MOTOR_3_ENCODER_A     0
#define MOTOR_3_ENCODER_B     1
#define MOTOR_4_ENCODER_A     8
#define MOTOR_4_ENCODER_B     9

// SERVOS
#define SERVO_1    16
#define SERVO_2    17
#define SERVO_3    __XRP_PIN_UNDEF
#define SERVO_4    __XRP_PIN_UNDEF

// Line Reflectors
#define LINE_L 26
#define LINE_R 27

// Range Finder
#define DISTANCE_TRIGGER 20
#define DISTANCE_ECHO    21

// I2C IMU
#define I2C_SCL_1 19
#define I2C_SDA_1 18

// User Button
#define BOARD_USER_BUTTON 22

/*
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

// Encoders
#define XRP_ENC_L_A     4
#define XRP_ENC_L_B     5
#define XRP_ENC_R_A     12
#define XRP_ENC_R_B     13
#define XRP_ENC_3_A     0
#define XRP_ENC_3_B     1
#define XRP_ENC_4_A     8
#define XRP_ENC_4_B     9


// Line Reflectors
#define REFLECT_LEFT_PIN 26
#define REFLECT_RIGHT_PIN 27

// Range Finder
#define ULTRASONIC_TRIG_PIN 20
#define ULTRASONIC_ECHO_PIN 21

// I2C IMU
#define XRP_SCL 19
#define XRP_SDA 18

// User Button
#define XRP_USER_BUTTON 22
*/

#endif