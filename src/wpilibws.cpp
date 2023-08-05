#include "wpilibws.h"
#include "robot.h"
#include "watchdog.h"

#include <map>

namespace wpilibws {

xrp::Watchdog _dsWatchdog{"ds"};

std::map<std::string, int> _xrpMotorMap = {
  {"motorL", 0},
  {"motorR", 1},
  {"motor3", 2},
  {"motor4", 3}
};

std::map<std::string, int> _xrpServoMap = {
  {"servo1", 4},
  {"servo2", 5}
};

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
    double value = atof(data["<speed"].as<std::string>().c_str());
    // Only use the speed value for the builtin motors
    if (channel >= 0 && channel <= 3) {
      xrp::setPwmValue(channel, value);
    }
  }
  else if (data.containsKey("<position")) {
    double value = atof(data["<position"].as<std::string>().c_str());
    value = (2.0 * value) - 1.0;
    // Use position values for other PWM
    if (channel > 3) {
      xrp::setPwmValue(channel, value);
    }
  }
}

void _handleXRPMotorMessage(JsonDocument& motorMsg) {
  std::string device = motorMsg["device"];
  auto data = motorMsg["data"];

  if (data.containsKey("<speed")) {
    double value = atof(data["<speed"].as<std::string>().c_str());

    int channel = -1;
    if (_xrpMotorMap.count(device) > 0) {
      channel = _xrpMotorMap[device];
    }

    if (channel != -1) {
      xrp::setPwmValue(channel, value);
    }
  }
}

void _handleXRPServoMessage(JsonDocument& servoMsg) {
  std::string device = servoMsg["device"];
  auto data = servoMsg["data"];

  if (data.containsKey("<position")) {
    double value = atof(data["<position"].as<std::string>().c_str());
    value = (2.0 * value) - 1.0;
    int channel = -1;
    if (_xrpServoMap.count(device) > 0) {
      channel = _xrpServoMap[device];
    }

    if (channel != -1) {
      xrp::setPwmValue(channel, value);
    }
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
    else if (jsonMsg["type"] == "XRPMotor") {
      _handleXRPMotorMessage(jsonMsg);
    }
    else if (jsonMsg["type"] == "XRPServo") {
      _handleXRPServoMessage(jsonMsg);
    }
  }
}

bool dsWatchdogActive() {
  return _dsWatchdog.satisfied();
}

// Message encoders
std::string makeEncoderMessage(int deviceId, int count) {
  StaticJsonDocument<256> msg;
  msg["type"] = "Encoder";
  msg["device"] = std::to_string(deviceId);
  msg["data"][">count"] = count;

  std::string ret;
  serializeJson(msg, ret);
  return ret;
}

std::string makeDIOMessage(int deviceId, bool value) {
  StaticJsonDocument<256> msg;
  msg["type"] = "DIO";
  msg["device"] = std::to_string(deviceId);
  msg["data"]["<>value"] = value;

  std::string ret;
  serializeJson(msg, ret);
  return ret;
}

} // namespace wpilibws
