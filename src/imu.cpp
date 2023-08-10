#include "imu.h"

namespace xrp {

unsigned long _imuUpdatePeriod = 1000 / IMU_UPDATE_RATE_HZ;
Adafruit_LSM6DSOX _lsm6;
bool _imuReady = false;
bool _imuEnabled = false;

unsigned long _lastIMUUpdateTime = 0;
bool _imuOnePassComplete = false;

float _gyroOffsetsDegPerSec[3] = {0, 0, 0};
float _gyroRatesDegPerSec[3] = {0, 0, 0};
float _gyroAnglesDeg[3] = {0, 0, 0};

float _radToDeg(float angleRad) {
  return angleRad * 180.0 / PI;
}

bool imuIsReady() {
    return _imuReady;
}

void imuSetEnabled(bool enabled) {
  _imuEnabled = enabled;
  Serial.printf("[IMU] %s\n", enabled ? "Enabling" : "Disabling");
}

bool imuIsEnabled() {
  return _imuEnabled;
}

void imuInit(uint8_t addr, TwoWire *theWire) {
  if (!_lsm6.begin_I2C(addr, theWire, 0)) {
    Serial.println("Failed to find LSM6DSOX");
    _imuReady = false;
  }
  else {
    _imuReady = true;
    Serial.println("--- IMU ---");
    Serial.println("LSM6DSOX detected");
    Serial.print("Accel Range: ");
    switch (_lsm6.getAccelRange()) {
      case LSM6DS_ACCEL_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case LSM6DS_ACCEL_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case LSM6DS_ACCEL_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case LSM6DS_ACCEL_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    Serial.print("Gyro Range: ");
    switch(_lsm6.getGyroRange()) {
      case LSM6DS_GYRO_RANGE_125_DPS:
        Serial.println("125 DPS");
        break;
      case LSM6DS_GYRO_RANGE_250_DPS:
        Serial.println("250 DPS");
        break;
      case LSM6DS_GYRO_RANGE_500_DPS:
        Serial.println("500 DPS");
        break;
      case LSM6DS_GYRO_RANGE_1000_DPS:
        Serial.println("1000 DPS");
        break;
      case LSM6DS_GYRO_RANGE_2000_DPS:
        Serial.println("2000 DPS");
        break;
      case ISM330DHCX_GYRO_RANGE_4000_DPS:
        break;
    }
  }
}

bool imuPeriodic() {
  if (!_imuReady) return false;
  if (!_imuEnabled) return false;

  unsigned long currTime = millis();
  unsigned long dt = currTime - _lastIMUUpdateTime;

  // Send the IMU data at the specified IMU_UPDATE_RATE_HZ
  if (dt < _imuUpdatePeriod) return false;

  // Get IMU data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  _lsm6.getEvent(&accel, &gyro, &temp);

  float rateX = _radToDeg(gyro.gyro.x);
  float rateY = _radToDeg(gyro.gyro.y);
  float rateZ = _radToDeg(gyro.gyro.z);
  _gyroRatesDegPerSec[0] = rateX - _gyroOffsetsDegPerSec[0];
  _gyroRatesDegPerSec[1] = rateY - _gyroOffsetsDegPerSec[1];
  _gyroRatesDegPerSec[2] = rateZ - _gyroOffsetsDegPerSec[2];

  // We can't integrate with only a single value, so bail out
  if (!_imuOnePassComplete) {
    _imuOnePassComplete = true;
    _lastIMUUpdateTime = currTime;
    return false;
  }
  
  float dtInSeconds = dt / 1000.0;

  for (int i = 0; i < 3; i++) {
    _gyroAnglesDeg[i] = _gyroAnglesDeg[i] + (_gyroRatesDegPerSec[i] * dtInSeconds);
  }

  _lastIMUUpdateTime = currTime;
  return true;
}

float gyroGetAngleX() {
  return _gyroAnglesDeg[0];
}

float gyroGetRateX() {
  return _gyroRatesDegPerSec[0];
}

float gyroGetAngleY() {
  return _gyroAnglesDeg[1];
}

float gyroGetRateY() {
  return _gyroRatesDegPerSec[1];
}

float gyroGetAngleZ() {
  return _gyroAnglesDeg[2];
}

float gyroGetRateZ() {
  return _gyroRatesDegPerSec[2];
}

void gyroReset() {
  for (int i = 0; i < 3; i++) {
    _gyroRatesDegPerSec[i] = 0;
    _gyroAnglesDeg[i] = 0;
  }

  _imuOnePassComplete = false;
}

} // namespace xrp