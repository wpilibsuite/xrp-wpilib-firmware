#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WebSockets4WebServer.h>
#include <WiFi.h>

#include "robot.h"
#include "wpilibws.h"

// HTTP and WS Servers
WebServer webServer(3300);
WebSockets4WebServer wsServer;

// TEMP: Status
unsigned long _wsMessageCount = 0;
unsigned long _lastMessageStatusPrint = 0;
int _baselineUsedHeap = 0;

void handleIndexRoute() {
  webServer.send(200, "text/plain", "You probably want the WS interface on /wpilibws");
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
  if (millis() - _lastMessageStatusPrint > 1000) {
    int numConnectedClients = wsServer.connectedClients();
    int usedHeap = rp2040.getUsedHeap();
    Serial.printf("t:%u c:%d h:%d msg:%u\n", millis(), numConnectedClients, usedHeap, _wsMessageCount);
    _lastMessageStatusPrint = millis();
  }
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
  Serial.println("[NET] Setting up Web Server routes");
  webServer.on("/", handleIndexRoute);

  // Set up WS routing
  Serial.println("[NET] Setting up WebSocket routing");
  webServer.addHook(wsServer.hookForWebserver("/wpilibws", handleWSEvent));

  Serial.println("[NET] Starting Web Server on port 3300");
  webServer.begin();

  Serial.println("[NET] Network Ready");
  Serial.printf("[NET] SSID: %s\n", WiFi.SSID().c_str());
  Serial.printf("[NET] IP: %s\n", WiFi.localIP().toString().c_str());

  xrp::robotInit();

  _lastMessageStatusPrint = millis();
  _baselineUsedHeap = rp2040.getUsedHeap();
}

void loop() {
  webServer.handleClient();
  wsServer.loop();

  // Read sensor data
  if (xrp::robotPeriodic()) {
    // TODO Got new data, send it up
    // Send Encoder data if present
    auto encValues = xrp::getActiveEncoderValues();
    for (auto encData : encValues) {
      // TODO make message and send
    }
  }

  checkPrintStatus();
}