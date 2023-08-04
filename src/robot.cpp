#include "robot.h"
#include "quadrature.pio.h"
#include "wpilibws.h"

#include <map>
#include <vector>

#include <Servo.h>

namespace xrp {

bool _robotInitialized = false;
bool _robotEnabled = false;
unsigned long _lastRobotPeriodicCall = 0;

// Servo Outputs
Servo servo1;
Servo servo2;

// Encoder PIO
PIO _encoderPio = nullptr;
uint _encoderPgmOffset = 0;
PIOProgram _encoderPgm(&quadrature_program);

// Encoders
std::vector<std::pair<int, int> > _encoderPins = {
  {4, 5},
  {12, 13},
  {0, 1},
  {8, 9}
};
int _encoderValues[4] = {0, 0, 0, 0};
int _encoderStateMachineIdx[4] = {-1, -1, -1, -1};
PIO _encoderPioInstance[4] = {nullptr, nullptr, nullptr, nullptr};
std::map<int, int> _encoderWPILibChannelToNativeMap;

// Internal helper functions
bool _initEncoders() {
  // TODO Use the PIOProgram wrapper here
  for (int i = 0; i < 4; i++) {
    int _pgmOffset = -1;
    int _smIdx = -1;
    PIO _pio;
    if (!_encoderPgm.prepare(&_pio, &_smIdx, &_pgmOffset)) {
      Serial.printf("[ENC-%u] Failed to set up program\n", i);
      return false;
    }

    // Save values
    _encoderPgmOffset = _pgmOffset;
    _encoderPioInstance[i] = _pio;
    _encoderStateMachineIdx[i] = _smIdx;

    // Init the program
    auto pins = _encoderPins.at(i);
    quadrature_program_init(_pio, _smIdx, _pgmOffset, pins.first, pins.second);
  }

  return true;
  // uint offset0 = pio_add_program(_encoderPio, &quadrature_program);
  
  // quadrature_program_init(_encoderPio, ENC_SM_IDX_MOTOR_L, offset0, 4, 5);
  // quadrature_program_init(_encoderPio, ENC_SM_IDX_MOTOR_R, offset0, 12, 13);
  // quadrature_program_init(_encoderPio, ENC_SM_IDX_MOTOR_3, offset0, 0, 1);
  // quadrature_program_init(_encoderPio, ENC_SM_IDX_MOTOR_4, offset0, 8, 9);
}

unsigned long _readEncodersInternal() {
  unsigned long _start = millis();
  for (int i = 0; i < 4; i++) {
    PIO _pio = _encoderPioInstance[i];
    uint _smIdx = _encoderStateMachineIdx[i];

    if (_pio != nullptr) {
      pio_sm_exec_wait_blocking(_pio, _smIdx, pio_encode_in(pio_x, 32));
      _encoderValues[i] = pio_sm_get_blocking(_pio, _smIdx);
    }
  }
  return millis() - _start;
}

void _initMotors() {
  // Left
  pinMode(XRP_LEFT_MOTOR_EN, OUTPUT);
  pinMode(XRP_LEFT_MOTOR_PH, OUTPUT);

  // RIGHT
  pinMode(XRP_RIGHT_MOTOR_EN, OUTPUT);
  pinMode(XRP_RIGHT_MOTOR_PH, OUTPUT);

  // Motor 3
  pinMode(XRP_MOTOR_3_EN, OUTPUT);
  pinMode(XRP_MOTOR_3_PH, OUTPUT);

  // Motor 4
  pinMode(XRP_MOTOR_4_EN, OUTPUT);
  pinMode(XRP_MOTOR_4_PH, OUTPUT);

  // Servos
  // pinMode(XRP_SERVO_1_PIN, OUTPUT);
  // pinMode(XRP_SERVO_2_PIN, OUTPUT);
}

bool _initServos() {
  bool success = true;
  if(servo1.attach(XRP_SERVO_1_PIN) == -1) {
    Serial.println("[ERR] Failed to attach servo1");
    success = false;
  }

  if (servo2.attach(XRP_SERVO_2_PIN) == -1) {
    Serial.println("[ERR] Failed to attach servo2");
    success = false;
  }

  return success;
}

void _setMotorPwmValueInternal(int en, int ph, double value) {
  PinStatus phValue = (value < 0.0) ? LOW : HIGH;
  int enValue = (abs(value) * 255);

  digitalWrite(ph, phValue);
  analogWrite(en, enValue);
}

void _setServoPwmValueInternal(int servoIdx, double value) {
  int val = ((value + 1.0) / 2.0) * 180;
  
  if (servoIdx == 0 && servo1.attached()) {
    servo1.write(val);
  }
  else if (servoIdx == 1 && servo2.attached()) {
    servo2.write(val);
  }
}

void _setPwmValueInternal(int channel, double value, bool override) {
  if (!_robotEnabled && !override) return;

  if (!wpilibws::dsWatchdogActive() && !override) {
    return;
  }

  // Hard coded channel list
  switch (channel) {
    case WPILIB_CH_PWM_MOTOR_L:
      _setMotorPwmValueInternal(XRP_LEFT_MOTOR_EN, XRP_LEFT_MOTOR_PH, value);
      break;
    case WPILIB_CH_PWM_MOTOR_R:
      _setMotorPwmValueInternal(XRP_RIGHT_MOTOR_EN, XRP_RIGHT_MOTOR_PH, value);
      break;
    case WPILIB_CH_PWM_MOTOR_3:
      _setMotorPwmValueInternal(XRP_MOTOR_3_EN, XRP_MOTOR_3_PH, value);
      break;
    case WPILIB_CH_PWM_MOTOR_4:
      _setMotorPwmValueInternal(XRP_MOTOR_4_EN, XRP_MOTOR_4_PH, value);
      break;
    case WPILIB_CH_PWM_SERVO_1:
      _setServoPwmValueInternal(0, value);
      break;
    case WPILIB_CH_PWM_SERVO_2:
      _setServoPwmValueInternal(1, value);
      break;
  }
}

void _pwmShutoff() {
  _setPwmValueInternal(0, 0, true);
  _setPwmValueInternal(1, 0, true);
  _setPwmValueInternal(2, 0, true);
  _setPwmValueInternal(3, 0, true);
  _setPwmValueInternal(4, 0, true);
  _setPwmValueInternal(5, 0, true);
}

void robotInit() {
  Serial.println("[XRP] Initializing XRP Onboards");

  // Set up the encoder state machines
  Serial.println("[XRP] Initializing Encoders");
  if (!_initEncoders()) {
    Serial.println("  - ERROR");
  }

  // Set up the motors
  Serial.println("[XRP] Initializing Motors");
  _initMotors();

  // Set up servos
  Serial.println("[XRP] Initializing Servos");
  if (!_initServos()) {
    Serial.println("  - ERROR");
  }

  // Set up on-board hardware
  pinMode(XRP_BUILTIN_LED, OUTPUT);
  pinMode(XRP_BUILTIN_BUTTON, INPUT_PULLUP);

  _robotInitialized = true;
}

bool robotInitialized() {
  return _robotInitialized;
}

// Return true if this actually ran
bool robotPeriodic() {
  // Kill PWM if the watchdog is dead
  // We want this to run as quickly as possible
  if (!wpilibws::dsWatchdogActive()) {
    _pwmShutoff();
  }

  if (millis() - _lastRobotPeriodicCall < 50) return false;

  unsigned long encoderReadTime = _readEncodersInternal();
  _lastRobotPeriodicCall = millis();
  return true;
}

void setEnabled(bool enabled) {
  // Prevent motors from starting with arbitrary values when enabling
  if (!_robotEnabled && enabled) {
    _pwmShutoff();
  }

  bool prevEnabledValue = _robotEnabled;
  _robotEnabled = enabled;

  if (prevEnabledValue && !enabled) {
    Serial.println("[XRP] Disabling");
    _pwmShutoff();
  }
  else if (!prevEnabledValue && enabled) {
    Serial.println("[XRP] Enabling");
  }
}

void configureEncoder(int deviceId, int chA, int chB) {
  if (chA == WPILIB_ENCODER_L_CH_A && chB == WPILIB_ENCODER_L_CH_B) {
    _encoderWPILibChannelToNativeMap[deviceId] = ENC_SM_IDX_MOTOR_L;
  }
  else if (chA == WPILIB_ENCODER_R_CH_A && chB == WPILIB_ENCODER_R_CH_B) {
    _encoderWPILibChannelToNativeMap[deviceId] = ENC_SM_IDX_MOTOR_R;
  }
  else if (chA == WPILIB_ENCODER_3_CH_A && chB == WPILIB_ENCODER_3_CH_B) {
    _encoderWPILibChannelToNativeMap[deviceId] = ENC_SM_IDX_MOTOR_3;
  }
  else if (chA == WPILIB_ENCODER_4_CH_A && chB == WPILIB_ENCODER_4_CH_B) {
    _encoderWPILibChannelToNativeMap[deviceId] = ENC_SM_IDX_MOTOR_4;
  }
  else {
    Serial.printf("[ERR] Invalid encoder pin mapping %d,%d\n", chA, chB);
  }
}

int readEncoder(int deviceId) {
  if (_encoderWPILibChannelToNativeMap.count(deviceId) > 0) {
    return _encoderValues[_encoderWPILibChannelToNativeMap[deviceId]];
  }
  return 0;
}

void resetEncoder(int deviceId) {
  if (_encoderWPILibChannelToNativeMap.count(deviceId) > 0) {
    int idx = _encoderWPILibChannelToNativeMap[deviceId];
    PIO _pio = _encoderPioInstance[idx];
    uint _smIdx = _encoderStateMachineIdx[idx];
    pio_sm_exec(_pio, _smIdx, pio_encode_set(pio_x, 0));
  }
}

std::vector<std::pair<int,int> > getActiveEncoderValues() {
  std::vector<std::pair<int,int> > ret;
  for (auto encData : _encoderWPILibChannelToNativeMap) {
    int wpilibDevice = encData.first;
    int nativeChannel = encData.second;
    ret.push_back(std::make_pair(wpilibDevice, _encoderValues[nativeChannel]));
  }
  return ret;
}

void setPwmValue(int wpilibChannel, double value) {
  _setPwmValueInternal(wpilibChannel, value, false);
}


} // namespace xrp
