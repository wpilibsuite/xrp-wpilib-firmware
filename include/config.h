#pragma once

#include <stddef.h>

#include <string>

// Increment this whenever the persisted config schema changes.
#define XRP_CONFIG_VERSION 2

constexpr const char XRP_CONFIG_PATH[] = "/config.ini";

constexpr const char XRP_BLUETOOTH_NAME_PREFIX[] = "WPIXRP-";
constexpr size_t XRP_BLUETOOTH_NAME_MAX_LENGTH = 26;
constexpr size_t XRP_BLUETOOTH_NAME_PREFIX_LENGTH =
    sizeof(XRP_BLUETOOTH_NAME_PREFIX) - 1;
constexpr size_t XRP_BLUETOOTH_NAME_SUFFIX_MAX_LENGTH =
    XRP_BLUETOOTH_NAME_MAX_LENGTH - XRP_BLUETOOTH_NAME_PREFIX_LENGTH;

class XRPBluetoothConfig {
 public:
  std::string deviceNameSuffix{""};
};

class XRPConfiguration {
 public:
  XRPBluetoothConfig bluetoothConfig;

  std::string toIniString(
      const std::string& defaultBluetoothNameSuffix) const;
};

std::string buildBluetoothDeviceName(const std::string& deviceNameSuffix);
std::string normalizeBluetoothDeviceNameSuffix(
    const std::string& deviceNameOrSuffix);
XRPConfiguration generateDefaultConfig(
    const std::string& defaultBluetoothNameSuffix);
XRPConfiguration loadConfiguration(
    const std::string& defaultBluetoothNameSuffix);
bool saveBluetoothDeviceName(const std::string& deviceNameOrSuffix,
                             const std::string& defaultBluetoothNameSuffix);
bool isValidBluetoothDeviceNameSuffix(const std::string& deviceNameSuffix);
