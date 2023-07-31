#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WebSockets4WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <vector>

#include "robot.h"
#include "wpilibws.h"

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
  Serial.begin(115200);

  delay(2000);

  // Busy-loop if there's no WiFi hardware
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("[NET] No WiFi Module");
    while (true);
  }

  // Set up WiFi AP
  WiFi.setHostname("XRP");
  bool result = WiFi.softAP("XRP", "0123456789");
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

void loop() {
  unsigned long loopStartTime = micros();

  // webServer.handleClient();
  wsServer.loop();

  // Disable the robot when we no longer have a connection
  if (wsServer.connectedClients() == 0) {
    xrp::setEnabled(false);
  }

  // Send any messages we need to
  checkAndSendMessages();

  // Read sensor data
  if (xrp::robotPeriodic()) {
    // TODO Got new data, send it up
    // Send Encoder data if present
    auto encValues = xrp::getActiveEncoderValues();
    for (auto encData : encValues) {
      sendMessage(wpilibws::makeEncoderMessage(encData.first, encData.second));
    }
  }

  updateLoopTime(loopStartTime);
  checkPrintStatus();
}
