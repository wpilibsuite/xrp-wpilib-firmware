#pragma once

#include <Servo.h>

// Servo pulses
#define XRP_SERVO_MIN_PULSE_US 500
#define XRP_SERVO_MAX_PULSE_US 2500

class XRPServo {
    public:
        boolean init(int pin);
        void setValue(double value);
    private:
        int _pin;
        int _value;
        Servo _servo;
};