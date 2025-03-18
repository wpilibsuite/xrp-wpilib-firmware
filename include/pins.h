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

#else
// Beta
#define PIN_LAYOUT_IDENT "Beta"

// XRP default pin names
 #define MOTOR_L_IN_1 (6u)
 #define MOTOR_L_IN_2 (7u)
 #define MOTOR_R_IN_1 (14u)
 #define MOTOR_R_IN_2 (15u)
 #define MOTOR_3_IN_1 (2u)
 #define MOTOR_3_IN_2 (3u)
 #define MOTOR_4_IN_1 (10u)
 #define MOTOR_4_IN_2 (11u)
 #define MOTOR_L_ENCODER_A (4u)
 #define MOTOR_L_ENCODER_B (5u)
 #define MOTOR_R_ENCODER_A (12u)
 #define MOTOR_R_ENCODER_B (13u)
 #define MOTOR_3_ENCODER_A (0u)
 #define MOTOR_3_ENCODER_B (1u)
 #define MOTOR_4_ENCODER_A (8u)
 #define MOTOR_4_ENCODER_B (9u)
 #define SERVO_1 (16u)
 #define SERVO_2 (17u)
 #define I2C_SDA_1 (18u)
 #define I2C_SCL_1 (19u)
 #define DISTANCE_TRIGGER (20u)
 #define DISTANCE_ECHO (21u)
 #define LINE_L (26u)
 #define LINE_R (27u)
 #define BOARD_VIN_MEASURE (28u)
 #define BOARD_USER_BUTTON (22u)
 #define BOARD_LED (PIN_LED)

#define SERVO_3    __XRP_PIN_UNDEF
#define SERVO_4    __XRP_PIN_UNDEF

#endif