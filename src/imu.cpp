#include "imu.h"

#include <MadgwickAHRS.h>

#define IMU_DEFAULT_CALIBRATION_TIME_MS 3000

#define IMU_MADGWICK_LOOP_FREQ_HZ 25

namespace xrp {

unsigned long _imuUpdatePeriod = 1000 / IMU_UPDATE_RATE_HZ;
Adafruit_LSM6DSOX _lsm6;
bool _imuReady = false;
bool _imuEnabled = false;

unsigned long _lastIMUUpdateTime = 0;
bool _imuOnePassComplete = false;

float _accelOffsetsG[3] = {0, 0, 0};
float _accelG[3] = {0, 0, 0};

float _gyroOffsetsDPS[3] = {0, 0, 0};
float _gyroRatesDPS[3] = {0, 0, 0};
float _gyroAnglesDeg[3] = {0, 0, 0};

float _ahrsOffsets[3] = {0, 0, 0};

Madgwick _ahrsFilter;
bool _filterStarted = false;
unsigned long _microsPerReading, _microsPrevious;

float _radToDeg(float angleRad) {
  return angleRad * 180.0 / PI;
}

float _accelToG(float accelMS2) {
  return accelMS2 / 9.80665;
}

bool imuIsReady() {
    return _imuReady;
}

void imuSetEnabled(bool enabled) {
  // If going from false to true, reset the gyro
  if (!_imuEnabled && enabled) {
    gyroReset();
  }

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
    
    Serial.println("Setting update rate to 208Hz");
    _lsm6.setGyroDataRate(LSM6DS_RATE_208_HZ);
    _lsm6.setAccelDataRate(LSM6DS_RATE_208_HZ);

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

void imuCalibrate(unsigned long calibrationTimeMs) {
  unsigned long loopDelayTime = 1000 / 208;

  if (calibrationTimeMs == 0) {
    calibrationTimeMs = IMU_DEFAULT_CALIBRATION_TIME_MS;
  }

  Serial.printf("[IMU] Beginning calibration. Running for %u ms\n", calibrationTimeMs);
  
  float gyroAvgValues[3] = {0, 0, 0};
  float accelAvgValues[3] = {0, 0, 0};

  int numVals = 0;

  bool ledBlinkState = true;

  unsigned long startTime = millis();
  unsigned long lastBlinkTime = startTime;

  digitalWrite(LED_BUILTIN, HIGH);
  while (millis() < startTime + calibrationTimeMs) {
    // Handle the blink (the delay at the end of this loop is much
    // smaller than what we can visually see anyway)
    if (millis() - lastBlinkTime > 100) {
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_BUILTIN, ledBlinkState ? HIGH : LOW);
      lastBlinkTime = millis();
    }

    // Get IMU data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    _lsm6.getEvent(&accel, &gyro, &temp);
    
    // Accelerometer averages
    accelAvgValues[0] += _accelToG(accel.acceleration.x);
    accelAvgValues[1] += _accelToG(accel.acceleration.y);
    accelAvgValues[2] += _accelToG(accel.acceleration.z);

    // Gyro averages
    gyroAvgValues[0] += _radToDeg(gyro.gyro.x);
    gyroAvgValues[1] += _radToDeg(gyro.gyro.y);
    gyroAvgValues[2] += _radToDeg(gyro.gyro.z);

    numVals++;
    delay(loopDelayTime);
  }

  _accelOffsetsG[0] = accelAvgValues[0] / numVals;
  _accelOffsetsG[1] = accelAvgValues[1] / numVals;
  _accelOffsetsG[2] = accelAvgValues[2] / numVals;

  _gyroOffsetsDPS[0] = gyroAvgValues[0] / numVals;
  _gyroOffsetsDPS[1] = gyroAvgValues[1] / numVals;
  _gyroOffsetsDPS[2] = gyroAvgValues[2] / numVals;

  // Remove 1G from the vertical axis (assumed to be Z)
  _accelOffsetsG[2] -= 1.0;

  Serial.printf("[IMU] Gyro Offsets(dps): X(%f) Y(%f) Z(%f), Accel Offsets(g): X(%f) Y(%f) Z(%f)\n",
      _gyroOffsetsDPS[0],
      _gyroOffsetsDPS[1],
      _gyroOffsetsDPS[2],
      _accelOffsetsG[0],
      _accelOffsetsG[1],
      _accelOffsetsG[2]);
  Serial.println("[IMU] Calibration Complete");

  digitalWrite(LED_BUILTIN, LOW);
}

unsigned long _imuLoopTime = 0;
int _imuLoopCount = 0;

void imuPeriodic() {
  // Initialize the filter if this is the first time we are running through the periodic
  if (!_filterStarted) {
    Serial.printf("[IMU] Starting Madgwick filter at %u hz\n", IMU_MADGWICK_LOOP_FREQ_HZ);
    _microsPerReading = 1000000 / IMU_MADGWICK_LOOP_FREQ_HZ;
    _microsPrevious = micros();
    _ahrsFilter.begin(IMU_MADGWICK_LOOP_FREQ_HZ);
    _filterStarted = true;
    return;
  }

  unsigned long microsNow = micros();
  if (microsNow - _microsPrevious >= _microsPerReading) {
    // Read data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    _lsm6.getEvent(&accel, &gyro, &temp);

    _gyroRatesDPS[0] = _radToDeg(gyro.gyro.x) - _gyroOffsetsDPS[0];
    _gyroRatesDPS[1] = _radToDeg(gyro.gyro.y) - _gyroOffsetsDPS[1];
    _gyroRatesDPS[2] = _radToDeg(gyro.gyro.z) - _gyroOffsetsDPS[2];

    _accelG[0] = _accelToG(accel.acceleration.x) - _accelOffsetsG[0];
    _accelG[1] = _accelToG(accel.acceleration.y) - _accelOffsetsG[1];
    _accelG[2] = _accelToG(accel.acceleration.z) - _accelOffsetsG[2];

    // Update the filter, which will compute orientation
    _ahrsFilter.updateIMU(_gyroRatesDPS[0], _gyroRatesDPS[1], _gyroRatesDPS[2], _accelG[0], _accelG[1], _accelG[2]);

    // Increment the previous time so that we keep proper pace
    _microsPrevious = _microsPrevious + _microsPerReading;

    // Stats
    _imuLoopTime += micros() - microsNow;
    _imuLoopCount++;

    if (_imuLoopCount > 100) {
      Serial.printf("[IMU] Avg AHRS Update Time: %u us\n", _imuLoopTime / _imuLoopCount);
      _imuLoopCount = 0;
      _imuLoopTime = 0;
    }
  }
}

/**
 * Determine if we have data ready to send upstream
 */
bool imuDataReady() {
  if (!_imuReady) return false;
  if (!_imuEnabled) return false;

  unsigned long currTime = millis();
  unsigned long dt = currTime - _lastIMUUpdateTime;

  if (dt < _imuUpdatePeriod) return false;

  _lastIMUUpdateTime = currTime;
  return true;
}

// bool imuPeriodic() {
//   if (!_imuReady) return false;
//   if (!_imuEnabled) return false;

//   unsigned long currTime = millis();
//   unsigned long dt = currTime - _lastIMUUpdateTime;

//   // Send the IMU data at the specified IMU_UPDATE_RATE_HZ
//   if (dt < _imuUpdatePeriod) return false;

//   // Get IMU data
//   sensors_event_t accel;
//   sensors_event_t gyro;
//   sensors_event_t temp;

//   _lsm6.getEvent(&accel, &gyro, &temp);

//   float rateX = _radToDeg(gyro.gyro.x);
//   float rateY = _radToDeg(gyro.gyro.y);
//   float rateZ = _radToDeg(gyro.gyro.z);
//   _gyroRatesDPS[0] = rateX - _gyroOffsetsDPS[0];
//   _gyroRatesDPS[1] = rateY - _gyroOffsetsDPS[1];
//   _gyroRatesDPS[2] = rateZ - _gyroOffsetsDPS[2];

//   // We can't integrate with only a single value, so bail out
//   if (!_imuOnePassComplete) {
//     _imuOnePassComplete = true;
//     _lastIMUUpdateTime = currTime;
//     return false;
//   }
  
//   float dtInSeconds = dt / 1000.0;

//   for (int i = 0; i < 3; i++) {
//     _gyroAnglesDeg[i] = _gyroAnglesDeg[i] + (_gyroRatesDPS[i] * dtInSeconds);
//   }

//   _lastIMUUpdateTime = currTime;
//   return true;
// }

// ===============================
// AHRS Values
// ===============================

/**
 * Get acceleration in the X axis
 * 
 * @return Acceleration in X (in G)
 */
float imuGetAccelX() {
  return _accelG[0];
}

/**
 * Get acceleration in the Y axis
 * 
 * @return Acceleration in Y (in G)
 */
float imuGetAccelY() {
  return _accelG[1];
}

/**
 * Get acceleration in the Z axis
 * 
 * @return Acceleration in Z (in G)
 */
float imuGetAccelZ() {
  return _accelG[2];
}

/**
 * Get gyro rate in the X axis
 * 
 * @return Gyro rate in X (in DPS)
 */
float imuGetGyroRateX() {
  return _gyroRatesDPS[0];
}

/**
 * Get gyro rate in the Y axis
 * 
 * @return Gyro rate in Y (in DPS)
 */
float imuGetGyroRateY() {
  return _gyroRatesDPS[1];
}

/**
 * Get gyro rate in the Z axis
 * 
 * @return Gyro rate in Z (in DPS)
 */
float imuGetGyroRateZ() {
  return _gyroRatesDPS[2];
}

/**
 * Get current roll angle
 * 
 * @return Current roll angle (in degrees)
 */
float imuGetRoll() {
  return _ahrsFilter.getRoll() - _ahrsOffsets[0];
}

/**
 * Get current pitch angle
 * 
 * @return Current pitch angle (in degrees)
 */
float imuGetPitch() {
  return _ahrsFilter.getPitch() - _ahrsOffsets[1];
}

/**
 * Get current yaw angle
 * 
 * @return Current yaw angle (in degrees)
 */
float imuGetYaw() {
  return _ahrsFilter.getYaw() - _ahrsOffsets[2];
}

/**
 * Reset the roll angle.
 * 
 * The AHRS filter always runs, so this basically sets an offset value
 */
void imuResetRoll() {
  _ahrsOffsets[0] = _ahrsFilter.getRoll();
}

/**
 * Reset the pitch angle.
 * 
 * The AHRS filter always runs, so this basically sets an offset value
 */
void imuResetPitch() {
  _ahrsOffsets[1] = _ahrsFilter.getPitch();
}

/**
 * Reset the yaw angle.
 * 
 * The AHRS filter always runs, so this basically sets an offset value
 */
void imuResetYaw() {
  _ahrsOffsets[2] = _ahrsFilter.getYaw();
}

void gyroReset() {
  Serial.println("[IMU] Resetting Gyro");
  imuResetRoll();
  imuResetPitch();
  imuResetYaw();
}

} // namespace xrp