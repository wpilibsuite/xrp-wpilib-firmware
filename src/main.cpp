#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WebSockets4WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>

#include <vector>

#include "imu.h"
#include "robot.h"
#include "wpilibws.h"

char chipID[20];
char DEFAULT_SSID[32];

// HTTP and WS Servers
WebServer webServer(3300);
WebSocketsServer wsServer(3300);

std::vector<std::string> outboundMessages;

// TEMP: Status
unsigned long _wsMessageCount = 0;
unsigned long _lastMessageStatusPrint = 0;
int _baselineUsedHeap = 0;

unsigned long _avgLoopTimeUs = 0;
unsigned long _loopTimeMeasurementCount = 0;

// void handleIndexRoute() {
//   webServer.send(200, "text/plain", "You probably want the WS interface on /wpilibws");
// }

void sendMessage(std::string msg) {
  outboundMessages.push_back(msg);
}

void checkAndSendMessages() {
  for (auto msg : outboundMessages) {
    wsServer.broadcastTXT(msg.c_str());
  }

  outboundMessages.clear();
}

void handleWSEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[NET:WS] [%u] Disconnected\n", num);
      break;
    case WStype_CONNECTED: {
        IPAddress ip = wsServer.remoteIP(num);
        Serial.printf("[NET:WS] [%u] Connection from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT: {
        StaticJsonDocument<512> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, payload);
        if (error) {
          Serial.println(error.f_str());
          break;
        }

        _wsMessageCount++;
        wpilibws::processWSMessage(jsonDoc);
      }
      break;
  }
}

void checkPrintStatus() {
  if (millis() - _lastMessageStatusPrint > 5000) {
    int numConnectedClients = wsServer.connectedClients();
    int usedHeap = rp2040.getUsedHeap();
    Serial.printf("t(ms):%u c:%d h:%d msg:%u lt(us):%u\n", millis(), numConnectedClients, usedHeap, _wsMessageCount, _avgLoopTimeUs);
    _lastMessageStatusPrint = millis();
  }
}

void updateLoopTime(unsigned long loopStart) {
  unsigned long loopTime = micros() - loopStart;
  unsigned long totalTime = _avgLoopTimeUs * _loopTimeMeasurementCount;
  _loopTimeMeasurementCount++;

  _avgLoopTimeUs = (totalTime + loopTime) / _loopTimeMeasurementCount;
}


void setup() {
  // Generate the default SSID using the flash ID
  pico_unique_board_id_t id_out;
  pico_get_unique_board_id(&id_out);
  sprintf(chipID, "%02x%02x-%02x%02x", id_out.id[4], id_out.id[5], id_out.id[6], id_out.id[7]);
  sprintf(DEFAULT_SSID, "XRP-%s", chipID);

  Serial.begin(115200);

  // Set up the I2C pins
  Wire1.setSCL(19);
  Wire1.setSDA(18);
  Wire1.begin();

  delay(2000);

  // Initialize IMU
  Serial.println("[IMU] Initializing IMU");
  xrp::imuInit(IMU_I2C_ADDR, &Wire1);

  // TODO Calibrate IMU

  // Busy-loop if there's no WiFi hardware
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("[NET] No WiFi Module");
    while (true);
  }

  // Set up WiFi AP
  WiFi.setHostname("XRP");
  bool result = WiFi.softAP(DEFAULT_SSID, "xrp-wpilib");
  if (result) {
    Serial.println("[NET] WiFi AP Ready");
  }
  else {
    Serial.println("[NET] AP Set up failed");
    while (true);
  }

  // Set up HTTP server routes
  // Serial.println("[NET] Setting up Web Server routes");
  // webServer.on("/", handleIndexRoute);

  // Set up WS routing
  // Serial.println("[NET] Setting up WebSocket routing");
  // webServer.addHook(wsServer.hookForWebserver("/wpilibws", handleWSEvent));

  // Serial.println("[NET] Starting Web Server on port 3300");
  // webServer.begin();

  Serial.println("[NET] Setting up WS Server");
  wsServer.onEvent(handleWSEvent);
  wsServer.begin();

  Serial.println("[NET] Network Ready");
  Serial.printf("[NET] SSID: %s\n", WiFi.SSID().c_str());
  Serial.printf("[NET] IP: %s\n", WiFi.localIP().toString().c_str());

  xrp::robotInit();

  _lastMessageStatusPrint = millis();
  _baselineUsedHeap = rp2040.getUsedHeap();
}

int lastCheckedNumClients = 0;
void loop() {
  unsigned long loopStartTime = micros();

  // webServer.handleClient();
  wsServer.loop();

  // Disable the robot when we no longer have a connection
  int numConnectedClients = wsServer.connectedClients();
  if (lastCheckedNumClients > 0 && numConnectedClients == 0) {
    xrp::robotSetEnabled(false);
    xrp::imuSetEnabled(false);
    outboundMessages.clear();
  }
  lastCheckedNumClients = numConnectedClients;

  if (numConnectedClients > 0) {
    // Send any messages we need to
    checkAndSendMessages();

    // Read sensor data
    auto updatedData = xrp::robotPeriodic();
    if (updatedData & XRP_DATA_ENCODER) {
      auto encValues = xrp::getActiveEncoderValues();
      for (auto encData : encValues) {
        sendMessage(wpilibws::makeEncoderMessage(encData.first, encData.second));
      }
    }

    if (updatedData & XRP_DATA_DIO) {
      // User button is on DIO 0
      sendMessage(wpilibws::makeDIOMessage(0, xrp::isUserButtonPressed()));
    }

    // Read Gyro Data
    if (xrp::imuPeriodic()) {
      float rateZ = xrp::gyroGetRateZ();
      float angleZ = xrp::gyroGetAngleZ();
      
      sendMessage(wpilibws::makeGyroSingleMessage(wpilibws::AXIS_Z, rateZ, angleZ));
    }

  }

  updateLoopTime(loopStartTime);
  checkPrintStatus();
}
