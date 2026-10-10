#pragma once
#include "Arduino.h"
inline int testServoAngle = -1;
class Servo {
 public:
  int attach(int, int, int) { return 1; }
  bool attached() const { return true; }
  void write(int degrees) { testServoAngle = degrees; }
};
