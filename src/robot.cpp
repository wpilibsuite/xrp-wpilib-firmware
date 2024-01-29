#include "robot.h"
#include "encoder.pio.h"
#include "encoder_period.pio.h"
#include "wpilibudp.h"

#include <map>
#include <vector>

#include <Servo.h>

#define REFLECT_LEFT_PIN 26
#define REFLECT_RIGHT_PIN 27

#define ULTRASONIC_TRIG_PIN 20
#define ULTRASONIC_ECHO_PIN 21
#define ULTRASONIC_MAX_PULSE_WIDTH 23200

namespace xrp {

bool _robotInitialized = false;
bool _robotEnabled = false;
unsigned long _lastRobotPeriodicCall = 0;

// Digital IO
bool _lastUserButtonState = false;

// Servo Outputs
Servo servo1;
Servo servo2;

// Encoder PIO
PIO _encoderPio = nullptr;
PIOProgram _encoderPgm(&encoder_program);

// Encoders
std::vector<std::pair<int, int> > _encoderPins = {
  {4, 5},
  {12, 13},
  {0, 1},
  {8, 9}
};

int _encoderValuesLast[4] = {0, 0, 0, 0};
int _encoderValues[4] = {0, 0, 0, 0};
int _encoderStateMachineIdx[4] = {-1, -1, -1, -1};
PIO _encoderPioInstance[4] = {nullptr, nullptr, nullptr, nullptr};
std::map<int, int> _encoderWPILibChannelToNativeMap;

//Encoder Period PIO (needed for getRate())
PIOProgram _encoder_periodPgm(&encoder_period_program);
int _encoder_periodValuesLast[4] = {0, 0, 0, 0};
int _encoder_periodValues[4] = {0, 0, 0, 0};
int _encoder_periodStateMachineIdx[4] = {-1, -1, -1, -1};
PIO _encoder_periodPioInstance[4] = {nullptr, nullptr, nullptr, nullptr};

// Reflectance
bool _reflectanceInitialized = false;

// Rangefinder
bool _rangefinderInitialized = false;
float _rangefinderDistMetres = 0.0f;
const float RANGEFINDER_MAX_DIST_M = 4.0f;

typedef void(*PIO_Program_Init_Fn)(PIO,uint,uint,uint);

//Internal helper functions
bool _initEncoders(PIO pioInstances[], 
                   int stateMachineIndexes[],
                   PIO_Program_Init_Fn pgmInit) {

  for (int i = 0; i < 4; i++) {
    int _pgmOffset = -1;
    int _smIdx = -1;
    PIO _pio;
    if (!_encoderPgm.prepare(&_pio, &_smIdx, &_pgmOffset)) {
      Serial.printf("[ENC-%u] Failed to set up program\n", i);
      return false;
    }

    // Save values
    pioInstances[i] = _pio;
    stateMachineIndexes[i] = _smIdx;

    // Init the program
    auto pins = _encoderPins.at(i);
    pgmInit(_pio, _smIdx, _pgmOffset, pins.first);
  }
}

bool _initEncoders() {
  return _initEncoders(_encoderPioInstance, _encoderStateMachineIdx, encoder_program_init) &&  //Init encoder counter program
         _initEncoders(_encoder_periodPioInstance, _encoder_periodStateMachineIdx, encoder_period_program_init); //Init encoder period program
}

int _readEncoderInternal(PIO _pio, uint _smIdx) {
  int count;

  // Read 5 times to get past buffer
  count = pio_sm_get_blocking(_pio, _smIdx);
  count = pio_sm_get_blocking(_pio, _smIdx);
  count = pio_sm_get_blocking(_pio, _smIdx);
  count = pio_sm_get_blocking(_pio, _smIdx);
  count = pio_sm_get_blocking(_pio, _smIdx);

  return count;
}

bool _readEncodersInternal() {
  unsigned long _start = millis();
  bool hasChange = false;
  for (int i = 0; i < 4; i++) {
    PIO _pio = _encoderPioInstance[i];
    uint _smIdx = _encoderStateMachineIdx[i];

    if (_pio != nullptr) {
      _encoderValues[i] = _readEncoderInternal(_pio, _smIdx);

      if (_encoderValues[i] != _encoderValuesLast[i]) {
        hasChange = true;
      }

      _encoderValuesLast[i] = _encoderValues[i];
    }
  }

  return hasChange;
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
  if(servo1.attach(XRP_SERVO_1_PIN, XRP_SERVO_MIN_PULSE_US, XRP_SERVO_MAX_PULSE_US) == -1) {
    Serial.println("[ERR] Failed to attach servo1");
    success = false;
  }

  if (servo2.attach(XRP_SERVO_2_PIN, XRP_SERVO_MIN_PULSE_US, XRP_SERVO_MAX_PULSE_US) == -1) {
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

  if (!wpilibudp::dsWatchdogActive() && !override) {
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
  pinMode(XRP_BUILTIN_LED, OUTPUT);
  pinMode(XRP_BUILTIN_BUTTON, INPUT_PULLUP);

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

  _robotInitialized = true;
}

bool robotInitialized() {
  return _robotInitialized;
}

// Return true if this actually ran
uint8_t robotPeriodic() {
  uint8_t ret = 0;

  // Kill PWM if the watchdog is dead
  // We want this to run as quickly as possible
  if (!wpilibudp::dsWatchdogActive()) {
    _pwmShutoff();
  }

  if (millis() - _lastRobotPeriodicCall < 50) return ret;

  // Just set the flag if we made it past the time check
  ret |= XRP_DATA_GENERAL;

  // Check for encoder updates
  bool hasEncUpdate = _readEncodersInternal();
  if (hasEncUpdate) {
    ret |= XRP_DATA_ENCODER;
  }

  // Check for DIO (button) updates
  bool currButtonState = isUserButtonPressed();
  if (currButtonState != _lastUserButtonState) {
    ret |= XRP_DATA_DIO;
    _lastUserButtonState = currButtonState;
  }

  _lastRobotPeriodicCall = millis();
  return ret;
}

bool isUserButtonPressed() {
  // This is a pull up circuit, so when pressed, the pin is low
  return digitalRead(XRP_BUILTIN_BUTTON) == LOW;
}

void robotSetEnabled(bool enabled) {
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

// TODO this can get removed at some point since we'll ALWAYS report all values
int readEncoder(int deviceId) {
  if (_encoderWPILibChannelToNativeMap.count(deviceId) > 0) {
    return _encoderValues[_encoderWPILibChannelToNativeMap[deviceId]];
  }
  return 0;
}

int readEncoderRaw(int rawDeviceId) {
  return _encoderValues[rawDeviceId];
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

void setDigitalOutput(int channel, bool value) {
  if (channel == 1) {
    // LED
    digitalWrite(XRP_BUILTIN_LED, value ? HIGH : LOW);
  }
}

void reflectanceInit() {
  analogReadResolution(12);

  _reflectanceInitialized = true;
}

bool reflectanceInitialized() {
  return _reflectanceInitialized;
}

/**
 * Return a scaled voltage (0 to 1) based off analog pin reading
 */
float _readAnalogPinScaled(uint8_t pin) {
  float scaled = (float)analogRead(pin) / 4095.0f;
  return scaled;
}

float getReflectanceLeft5V() {
  if (!_reflectanceInitialized) {
    return -1.0f;
  }

  return _readAnalogPinScaled(REFLECT_LEFT_PIN) * 5.0f;
}

float getReflectanceRight5V() {
  if (!_reflectanceInitialized) {
    return -1.0f;
  }

  return _readAnalogPinScaled(REFLECT_RIGHT_PIN) * 5.0f;
}

void rangefinderInit() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Trigger Pin
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  pinMode(ULTRASONIC_ECHO_PIN, INPUT); // Echo pin
  _rangefinderInitialized = true;
}

bool rangefinderInitialized() {
  return _rangefinderInitialized;
}

float getRangefinderDistance5V() {
  return (_rangefinderDistMetres / RANGEFINDER_MAX_DIST_M) * 5.0f;
}

void rangefinderPollForData() {
  uint32_t bits = 0;
  if (rp2040.fifo.pop_nb(&bits)) {
    memcpy(&_rangefinderDistMetres, &bits, sizeof(_rangefinderDistMetres));
  }
}

void rangefinderPeriodic() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulseWidth;
  float distCM;
  float distMetres = RANGEFINDER_MAX_DIST_M;

  // Stabilize the sensor
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);

  // Send a 10us pulse
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // wait for pulse on the echo pin
  while (digitalRead(ULTRASONIC_ECHO_PIN) == 0);

  t1 = micros();
  while (digitalRead(ULTRASONIC_ECHO_PIN) == 1) {
    if (micros() - t1 > ULTRASONIC_MAX_PULSE_WIDTH) {
      break;
    }
  }
  t2 = micros();
  pulseWidth = t2 - t1;

  distCM = pulseWidth / 58.0;

  if (pulseWidth > ULTRASONIC_MAX_PULSE_WIDTH) {
    distMetres = RANGEFINDER_MAX_DIST_M;
  }
  else {
    distMetres = distCM / 100.0f;
  }

  // convert to a uint32_t so that we can push it onto the FIFO
  uint32_t bits = 0;
  memcpy(&bits, &distMetres, sizeof(bits));
  bool result = rp2040.fifo.push_nb(bits);
}

} // namespace xrp
