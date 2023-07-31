#include "robot.h"
#include "quadrature.pio.h"

#include <map>

namespace xrp {

bool _robotInitialized = false;
bool _robotEnabled = false;
unsigned long _lastRobotPeriodicCall = 0;

// PWM outputs

// Encoders
int _encoderValues[4] = {0, 0, 0, 0};
std::map<int, int> _encoderWPILibChannelToNativeMap;


// Internal helper functions
void _initEncoders() {
  uint offset0 = pio_add_program(pio0, &quadrature_program);
  quadrature_program_init(pio0, ENC_SM_IDX_MOTOR_L, offset0, 4, 5);
  quadrature_program_init(pio0, ENC_SM_IDX_MOTOR_R, offset0, 12, 13);
  quadrature_program_init(pio0, ENC_SM_IDX_MOTOR_3, offset0, 0, 1);
  quadrature_program_init(pio0, ENC_SM_IDX_MOTOR_4, offset0, 8, 9);
}

unsigned long _readEncodersInternal() {
  unsigned long _start = millis();
  for (int i = 0; i < 4; i++) {
    pio_sm_exec_wait_blocking(pio0, i, pio_encode_in(pio_x, 32));
    _encoderValues[i] = pio_sm_get_blocking(pio0, i);
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
  pinMode(XRP_SERVO_1_PIN, OUTPUT);
  pinMode(XRP_SERVO_2_PIN, OUTPUT);
}

void _setMotorPwmValueInternal(int en, int ph, double value) {
  PinStatus phValue = (value < 0.0) ? LOW : HIGH;
  int enValue = (abs(value) * 255);

  digitalWrite(ph, phValue);
  analogWrite(en, enValue);
}

void _setServoPwmValueInternal(int pin, double value) {
  int val = ((value + 1.0) / 2.0) * 255;
  analogWrite(pin, val);
}

void _setPwmValueInternal(int channel, double value, bool override) {
  if (!_robotEnabled && !override) return;

  // TODO Watchdog

  Serial.printf("[PWM-%u] v: %f\n", channel, value);

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
      _setServoPwmValueInternal(XRP_SERVO_1_PIN, value);
      break;
    case WPILIB_CH_PWM_SERVO_2:
      _setServoPwmValueInternal(XRP_SERVO_2_PIN, value);
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
  _initEncoders();

  // Set up the motors and servo outputs
  Serial.println("[XRP] Initializing PWM");
  _initMotors();

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
    pio_sm_exec(pio0, _encoderWPILibChannelToNativeMap[deviceId], pio_encode_set(pio_x, 0));
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