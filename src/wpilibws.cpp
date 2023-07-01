#include "wpilibws.h"
#include "robot.h"
#include "watchdog.h"

namespace wpilibws {

xrp::Watchdog _dsWatchdog{"ds"};

void _handleDIOMessage(JsonDocument& dioMsg) {

}

void _handleDSMessage(JsonDocument& dsMsg) {
  _dsWatchdog.feed();

  auto data = dsMsg["data"];
  if (data.containsKey(">enabled")) {
    xrp::setEnabled(data[">enabled"].as<bool>());
  }
}

void _handleEncoderMessage(JsonDocument& encMsg) {
  int deviceNum = atoi(encMsg["device"].as<const char*>());
  auto data = encMsg["data"];

  if (data.containsKey("<init")) {
    bool initVal = data["<init"];
    int chA = -1;
    int chB = -1;

    if (data.containsKey("<channel_a")) {
      chA = data["<channel_a"];
    }
    if (data.containsKey("<channel_b")) {
      chB = data["<channel_b"];
    }

    if (initVal && chA != -1 && chB != -1) {
      xrp::configureEncoder(deviceNum, chA, chB);
    }
  }
}

void _handleGyroMessage(JsonDocument& gyroMsg) {

}

void _handlePWMMessage(JsonDocument& pwmMsg) {
  int channel = atoi(pwmMsg["device"].as<const char*>());
  auto data = pwmMsg["data"];

  if (data.containsKey("<speed")) {
    double value = atof(data["<speed"].as<const char*>());
    xrp::setPwmValue(channel, value);
  }
  else if (data.containsKey("<position")) {
    double value = atof(data["<position"].as<const char*>());
    value = (2.0 * value) - 1.0;
    xrp::setPwmValue(channel, value);
  }
}

// WS Message Handling
void processWSMessage(JsonDocument& jsonMsg) {
  // Ensure Validity
  if (jsonMsg.containsKey("type") && jsonMsg.containsKey("data")) {
    if (jsonMsg["type"] == "PWM") {
      _handlePWMMessage(jsonMsg);
    }
    else if (jsonMsg["type"] == "DriverStation") {
      _handleDSMessage(jsonMsg);
    }
    else if (jsonMsg["type"] == "Encoder") {
      _handleEncoderMessage(jsonMsg);
    }
    else if (jsonMsg["type"] == "DIO") {
      // TODO DIO
    }
    else if (jsonMsg["type"] == "Gyro") {
      // TODO Gyro
    }
  }
}

bool dsWatchdogActive() {
  return _dsWatchdog.satisfied();
}

} // namespace wpilibws