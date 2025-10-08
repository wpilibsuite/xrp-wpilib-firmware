#pragma once

#include <string>
#include <vector>

// This should get incremented everytime we make changes here
#define XRP_CONFIG_VERSION 1

enum NetworkMode { AP, STA, NOT_CONFIGURED };

class XRPNetConfig {
  public:
    NetworkMode mode { NetworkMode::NOT_CONFIGURED };
    std::string defaultAPName {""};
    std::string defaultAPPassword {""};
    int defaultAPChannel = 0;
    std::vector< std::pair<std::string, std::string> > networkList;
};

class XRPConfiguration {
  public:
    XRPNetConfig networkConfig;

    std::string toJsonString();
};

XRPConfiguration loadConfiguration(std::string defaultAPName);
NetworkMode configureNetwork(XRPConfiguration config);
XRPConfiguration generateDefaultConfig(std::string defaultAP);