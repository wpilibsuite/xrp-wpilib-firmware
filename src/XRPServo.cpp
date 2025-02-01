#include <Servo.h>
#include "XRPServo.h"
#include "pins.h"

// Initialize the servo values
boolean XRPServo::init(int pin) {
    _pin = pin;
    _value = 0;
    boolean success = true;

    // Only attach to a servo if it is valid.
    if(!isValid(pin)) {
      Serial.printf("Pin [%d] unavailable on this boaord", pin);
      Serial.println("\n");
      return true;
    }

    if(_servo.attach(pin, XRP_SERVO_MIN_PULSE_US, XRP_SERVO_MAX_PULSE_US) == -1) {
        Serial.println("[ERR] Failed to attach servo1");
        success = false;
    }
    return success;
}

// Set the new servo position
void XRPServo::setValue(double value) {
  int val = ((value + 1.0) / 2.0) * 180;

  if(val != _value) {
    Serial.printf("Servo[%d] %d\n", _pin, _value);
    Serial.println("\n");
    _value = val;
  }

  if ( isValid() && _servo.attached() ) {
    _servo.write(val);
  }
}

bool XRPServo::isValid(int pin) {
  return pin != __XRP_PIN_UNDEF;
}

// Check if a valid pin
bool XRPServo::isValid() {
  return isValid(_pin);
}