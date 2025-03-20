#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFiMulti.h>
#include <WiFi.h>

#include "config.h"

NetworkMode configureNetwork(XRPConfiguration config) {
  bool shouldUseAP = false;
  WiFiMulti multi;

  if (config.networkConfig.mode == NetworkMode::AP) {
    shouldUseAP = true;
  }
  else if (config.networkConfig.mode == NetworkMode::STA) {
    Serial.println("[NET] Attempting to start in STA Mode");
    Serial.println("[NET] Trying the following networks:");
    for (auto netInfo : config.networkConfig.networkList) {
      Serial.printf("* %s\n", netInfo.first.c_str());
      multi.addAP(netInfo.first.c_str(), netInfo.second.c_str());
    }

    // Attempt to connect
    if (multi.run() != WL_CONNECTED) {
      Serial.println("[NET] Failed to connect to any network on list. Falling back to AP");
      shouldUseAP = true;
    }
  }

  if (shouldUseAP) {
    Serial.println("[NET] Attempting to start in AP mode");
    bool result = WiFi.softAP(
          config.networkConfig.defaultAPName.c_str(),
          config.networkConfig.defaultAPPassword.c_str());
    
    if (result) {
      Serial.println("[NET] AP Ready");
    }
    else {
      Serial.println("[NET] AP Set up Failed");
    }
  }

  Serial.println("[NET] ### NETWORK CONFIGURED ###");
  Serial.printf("[NET] SSID: %s\n", WiFi.SSID().c_str());

  return shouldUseAP ? NetworkMode::AP : NetworkMode::STA;
}

XRPConfiguration generateDefaultConfig(std::string defaultAPName) {
  XRPConfiguration defaultConfig;

  // Set up the default AP
  defaultConfig.networkConfig.defaultAPName = defaultAPName;
  defaultConfig.networkConfig.defaultAPPassword = "xrp-wpilib";
  defaultConfig.networkConfig.mode = NetworkMode::AP;

  // Load in a test AP
  defaultConfig.networkConfig.networkList.push_back(std::make_pair("Test Network", "Test Password"));

  return defaultConfig;
}

std::string XRPConfiguration::toJsonString() {
  StaticJsonDocument<512> config;

  config["configVersion"] = XRP_CONFIG_VERSION;

  // Network
  JsonObject network = config.createNestedObject("network");
  JsonObject defaultAP = network.createNestedObject("defaultAP");

  defaultAP["ssid"] = networkConfig.defaultAPName;
  defaultAP["password"] = networkConfig.defaultAPPassword;

  JsonArray prefNetworks = network.createNestedArray("networkList");
  network["mode"] = networkConfig.mode == NetworkMode::AP ? "AP" : "STA";

  for (auto netInfo : networkConfig.networkList) {
    JsonObject networkObj = prefNetworks.createNestedObject();
    networkObj["ssid"] = netInfo.first;
    networkObj["password"] = netInfo.second;
  }

  std::string ret;
  serializeJsonPretty(config, ret);
  return ret;
}

void writeConfigToDisk(XRPConfiguration config) {
  File f = LittleFS.open("/config.json", "w");
  f.print(config.toJsonString().c_str());
  f.close();
}

XRPConfiguration loadConfiguration(std::string defaultAPName) {
  XRPConfiguration config;

  File f = LittleFS.open("/config.json", "r");
  if (!f) {
    Serial.println("[CONFIG] No config file found. Creating default");
    
    config = generateDefaultConfig(defaultAPName);
    writeConfigToDisk(config);
    return config;
  }

  // Load and verify
  StaticJsonDocument<512> configJson;
  auto jsonErr = deserializeJson(configJson, f);
  f.close();

  if (jsonErr) {
    Serial.print("[CONFIG] Deserialization failed: ");
    Serial.println(jsonErr.f_str());
    Serial.println("[CONFIG] Using default");
    config = generateDefaultConfig(defaultAPName);

    // Write the file
    writeConfigToDisk(config);

    return config;
  }

  if (configJson["configVersion"] != XRP_CONFIG_VERSION) {
    Serial.print("[CONFIG] Configuration version mismatch. Using default");
    config = generateDefaultConfig(defaultAPName);

    writeConfigToDisk(config);
    return config;
  }

  // Network Section
  if (!configJson.containsKey("network")) {
    Serial.print("[CONFIG] No network information specified. Using defaults");
    config = generateDefaultConfig(defaultAPName);

    writeConfigToDisk(config);
    return config;
  }

  auto networkInfo = configJson["network"];

  // Generate a temporary defaults object
  XRPConfiguration tempDefault = generateDefaultConfig(defaultAPName);
  bool shouldWrite = false;

  // Check if there's a default AP provided
  if (networkInfo.containsKey("defaultAP")) {
    auto defaultAPInfo = networkInfo["defaultAP"];
    if (defaultAPInfo.containsKey("ssid") && defaultAPInfo["ssid"].as<std::string>().length() != 0) {
      config.networkConfig.defaultAPName = defaultAPInfo["ssid"].as<std::string>();
    }
    else {
      Serial.println("[CONFIG] Default AP SSID missing. Using default");
      config.networkConfig.defaultAPName = tempDefault.networkConfig.defaultAPName;
      shouldWrite = true;
    }

    if (defaultAPInfo.containsKey("password")) {
      config.networkConfig.defaultAPPassword = defaultAPInfo["password"].as<std::string>();
    }
    else {
      Serial.println("[CONFIG] Default AP Password missing. Using default");
      config.networkConfig.defaultAPPassword = tempDefault.networkConfig.defaultAPPassword;
      shouldWrite = true;
    }
  }

  // Load in the preferred network list
  if (networkInfo.containsKey("networkList")) {
    auto networkList = networkInfo["networkList"];
    JsonArray networks = networkList.as<JsonArray>();
    for (auto v : networks) {
      if (v.containsKey("ssid") && v.containsKey("password")) {
        config.networkConfig.networkList.push_back(std::make_pair<std::string, std::string>(v["ssid"], v["password"]));
      }
    }
  }

  // Check if we're in STA mode. If so, we'll need at least 1 network in the list
  if (networkInfo.containsKey("mode")) {
    if (networkInfo["mode"] == "STA") {
      if (config.networkConfig.networkList.size() > 0) {
        config.networkConfig.mode = NetworkMode::STA;
      }
      else {
        Serial.println("[CONFIG] Network mode set to STA but no provided networks. Resettign to AP");
        config.networkConfig.mode = NetworkMode::AP;
        shouldWrite = true;
      }
    }
    else {
      config.networkConfig.mode = NetworkMode::AP;
    }
  }
  else {
    Serial.println("[CONFIG] Network Mode missing. Defaulting to AP");
    config.networkConfig.mode = NetworkMode::AP;
    shouldWrite = true;
  }

  if (shouldWrite) {
    writeConfigToDisk(config);
  }

  return config;
}