#include "imu.h"

namespace xrp {

Adafruit_LSM6DSOX _lsm6;
bool _imuReady = false;

bool imuReady() {
    return _imuReady;
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

} // namespace xrp