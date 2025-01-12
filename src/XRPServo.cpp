#include <Servo.h>
#include "XRPServo.h"

boolean XRPServo::init(int pin) {
    _pin = pin;
    _value = 0;
    boolean success = true;
    if(_servo.attach(pin, XRP_SERVO_MIN_PULSE_US, XRP_SERVO_MAX_PULSE_US) == -1) {
        Serial.println("[ERR] Failed to attach servo1");
        success = false;
    }
    return success;
}

void XRPServo::setValue(double value) {
  int val = ((value + 1.0) / 2.0) * 180;

  if(val != _value) {
    Serial.printf("Servo[%d] %d\n", _pin, _value);
    Serial.println("\n");
    _value = val;
  }

  if ( _servo.attached() ) {
    _servo.write(val);
  }
}