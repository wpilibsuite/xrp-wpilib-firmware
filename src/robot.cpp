#include "robot.h"
#include "wpilibudp.h"
#include "encoder.h"
#include "XRPServo.h"

#include <map>
#include <vector>

#define MIN_UPDATE_TIME_MS 50

namespace xrp {

bool _robotInitialized = false;
bool _robotEnabled = false;
unsigned long _lastRobotPeriodicCall = 0;

// Digital IO
bool _lastUserButtonState = false;

// Encoders
std::vector<std::pair<int, int> > _encoderPins = {
  {MOTOR_L_ENCODER_A, MOTOR_L_ENCODER_B},
  {MOTOR_R_ENCODER_A, MOTOR_R_ENCODER_B},
  {MOTOR_3_ENCODER_A, MOTOR_3_ENCODER_B},
  {MOTOR_4_ENCODER_A, MOTOR_4_ENCODER_B}
};

std::vector<int> _servoPins = {
  SERVO_1,
  SERVO_2,
  SERVO_3,
  SERVO_4
};

std::map<int, int> _encoderWPILibChannelToNativeMap;

//Encoder PIO
Encoder encoders[NUM_OF_ENCODERS];

// Servo array
XRPServo servos[NUM_OF_SERVOS];


// Reflectance
bool _reflectanceInitialized = false;

// Rangefinder
bool _rangefinderInitialized = false;
float _rangefinderDistMetres = 0.0f;
const float RANGEFINDER_MAX_DIST_M = 4.0f;

bool _initEncoders() {
  for(int i=0; i < NUM_OF_ENCODERS; ++i) {
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
  for(int i=0; i < NUM_OF_ENCODERS; ++i) {
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
  pinMode(MOTOR_L_IN_1, OUTPUT);
  pinMode(MOTOR_L_IN_2, OUTPUT);

  // RIGHT
  pinMode(MOTOR_R_IN_1, OUTPUT);
  pinMode(MOTOR_R_IN_2, OUTPUT);

  // Motor 3
  pinMode(MOTOR_3_IN_1, OUTPUT);
  pinMode(MOTOR_3_IN_2, OUTPUT);

  // Motor 4
  pinMode(MOTOR_4_IN_1, OUTPUT);
  pinMode(MOTOR_4_IN_2, OUTPUT);
}

bool _initServos() {
  bool success = true;

  for (int i=0;i<NUM_OF_SERVOS;i++) {
    success = servos[i].init(_servoPins[i]);
  }

  return success;
}

#ifdef PICO_RP2350

void _setMotorPwmValueInternal(int in2, int in1, double value) {
  boolean is_forward = (value >= 0.0);
  int speed = (abs(value) * 255);
  
  // Direction determines which pin should be the brake
  if(is_forward) {
    digitalWrite(in2,LOW);
    analogWrite(in1, speed);
  } else {
    digitalWrite(in1,LOW);
    analogWrite(in2, speed);    
  }
}

#else

void _setMotorPwmValueInternal(int en, int ph, double value) {
  
  PinStatus phValue = (value < 0.0) ? LOW : HIGH;
  int enValue = (abs(value) * 255);

  digitalWrite(ph, phValue);
  analogWrite(en, enValue);
}

#endif

void _setPwmValueInternal(int channel, double value, bool override) {
  if (!_robotEnabled && !override) return;

  if (!wpilibudp::dsWatchdogActive() && !override) {
    return;
  }

  // Hard coded channel list
  switch (channel) {
    case WPILIB_CH_PWM_MOTOR_L:
      _setMotorPwmValueInternal(MOTOR_L_IN_2, MOTOR_L_IN_1, value);
      break;
    case WPILIB_CH_PWM_MOTOR_R:
      _setMotorPwmValueInternal(MOTOR_R_IN_2, MOTOR_R_IN_1, value);
      break;
    case WPILIB_CH_PWM_MOTOR_3:
      _setMotorPwmValueInternal(MOTOR_3_IN_2, MOTOR_3_IN_1, value);
      break;
    case WPILIB_CH_PWM_MOTOR_4:
      _setMotorPwmValueInternal(MOTOR_4_IN_2, MOTOR_4_IN_1, value);
      break;
    case WPILIB_CH_PWM_SERVO_1:
      servos[0].setValue(value);
      break;
    case WPILIB_CH_PWM_SERVO_2:
      servos[1].setValue(value);
      break;
    case WPILIB_CH_PWM_SERVO_3:
      servos[2].setValue(value);
      break;
    case WPILIB_CH_PWM_SERVO_4:
      servos[3].setValue(value);
      break;
  }
}

void _pwmShutoff() {
  _setPwmValueInternal(WPILIB_CH_PWM_MOTOR_L, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_MOTOR_R, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_MOTOR_3, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_MOTOR_4, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_SERVO_1, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_SERVO_2, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_SERVO_3, 0, true);
  _setPwmValueInternal(WPILIB_CH_PWM_SERVO_4, 0, true);
}

void robotInit() {
  Serial.println("[XRP] Initializing XRP Onboards");
  pinMode(XRP_BUILTIN_LED, OUTPUT);
  pinMode(BOARD_USER_BUTTON, INPUT_PULLUP);

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

  // Only check if user button pressed at the less frequent interval
  if (millis() - _lastRobotPeriodicCall < MIN_UPDATE_TIME_MS) return ret;

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
  return digitalRead(BOARD_USER_BUTTON) == LOW;
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

  return _readAnalogPinScaled(LINE_L) * 5.0f;
}

float getReflectanceRight5V() {
  if (!_reflectanceInitialized) {
    return -1.0f;
  }

  return _readAnalogPinScaled(LINE_R) * 5.0f;
}

float getVinMeasured() {
  //ADC reads are 3.3V max
  //R1 = 100k, R2 = 33k, voltage divider = R2/(R1+R2)
  //reading * Vadc_max / volt divider
  return _readAnalogPinScaled(XRP_VIN_MEAS) * 3.3f / (33.f/(100.f+33.f));
}

void rangefinderInit() {
  pinMode(DISTANCE_TRIGGER, OUTPUT); // Trigger Pin
  digitalWrite(DISTANCE_TRIGGER, LOW);

  pinMode(DISTANCE_ECHO, INPUT); // Echo pin
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
  digitalWrite(DISTANCE_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(DISTANCE_TRIGGER, HIGH);

  // Send a 10us pulse
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIGGER, LOW);

  // wait for pulse on the echo pin
  while (digitalRead(DISTANCE_ECHO) == 0);

  t1 = micros();
  while (digitalRead(DISTANCE_ECHO) == 1) {
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
