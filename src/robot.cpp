#include "robot.h"
#include "wpilibudp.h"
#include "encoder.h"

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

// Encoders
std::vector<std::pair<int, int> > _encoderPins = {
  {4, 5},
  {12, 13},
  {0, 1},
  {8, 9}
};

std::map<int, int> _encoderWPILibChannelToNativeMap;

//Encoder PIO
Encoder encoders[4];


// Reflectance
bool _reflectanceInitialized = false;

// Rangefinder
bool _rangefinderInitialized = false;
float _rangefinderDistMetres = 0.0f;
const float RANGEFINDER_MAX_DIST_M = 4.0f;

bool _initEncoders() {
  for(int i=0; i < 4; ++i) {
    int pin = _encoderPins[i].first;

    if(!encoders[i].init(pin)) {
      Serial.printf("[ENC-%u] Failed to set up program.\n", i);
      return false;
    }
  }

  return true;
}

int _updateEncoders() {
  int count = 0;
  for(int i=0; i < 4; ++i) {
    auto& encoder = encoders[i];
    int next = encoder.update();
    if(next >= 8) {
      Serial.printf("[ENC-%u] Encoder Possible PIO RX Buffer Overrun: %d\n", i, next);
    }
    count += next;
  }
  return count;
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

  _updateEncoders();

  if (millis() - _lastRobotPeriodicCall < 50) return ret;

  // Just set the flag if we made it past the time check
  ret |= XRP_DATA_GENERAL;

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
    for(auto& encoder : encoders) {
      encoder.enable();
    }
  }

  bool prevEnabledValue = _robotEnabled;
  _robotEnabled = enabled;

  if (prevEnabledValue && !enabled) {
    Serial.println("[XRP] Disabling");
    _pwmShutoff();
    for(auto& encoder : encoders) {
      encoder.disable();
    }
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

int readEncoderRaw(int rawDeviceId) {
  return encoders[rawDeviceId].getCount();
}

uint readEncoderPeriod(int rawDeviceId) {
  return encoders[rawDeviceId].getPeriod();
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

float getVinMeasured() {
  //ADC reads are 3.3V max
  //R1 = 100k, R2 = 33k, voltage divider = R2/(R1+R2)
  //reading * Vadc_max / volt divider
  return _readAnalogPinScaled(XRP_VIN_MEAS) * 3.3f / (33.f/(100.f+33.f));
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
