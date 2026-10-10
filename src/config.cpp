#include "debug_log.h"

#include "config.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <limits.h>

namespace {

constexpr const char* CONFIG_VERSION_KEY = "config_version";
constexpr const char* CONFIG_VERSION_CAMEL_CASE_KEY = "configversion";
constexpr const char* BLUETOOTH_SECTION = "bluetooth";
constexpr const char* BLUETOOTH_DEVICE_NAME_KEY = "devicename";

bool isAsciiWhitespace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' ||
         c == '\v';
}

std::string trimAsciiWhitespace(const std::string& value) {
  size_t start = 0;
  while (start < value.size() && isAsciiWhitespace(value[start])) {
    start++;
  }

  size_t end = value.size();
  while (end > start && isAsciiWhitespace(value[end - 1])) {
    end--;
  }

  return value.substr(start, end - start);
}

std::string toLowerAscii(std::string value) {
  for (char& c : value) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }

  return value;
}

std::string stripInlineComment(const std::string& line) {
  bool inSingleQuote = false;
  bool inDoubleQuote = false;

  for (size_t i = 0; i < line.size(); i++) {
    char c = line[i];
    if ((inSingleQuote || inDoubleQuote) && c == '\\') {
      i++;
      continue;
    }
    if (c == '"' && !inSingleQuote) {
      inDoubleQuote = !inDoubleQuote;
    } else if (c == '\'' && !inDoubleQuote) {
      inSingleQuote = !inSingleQuote;
    } else if (!inSingleQuote && !inDoubleQuote && (c == '#' || c == ';') &&
               (i == 0 || isAsciiWhitespace(line[i - 1]))) {
      return line.substr(0, i);
    }
  }

  return line;
}

bool unquoteIniValue(std::string* value, std::string* error,
                     unsigned int lineNumber) {
  if (value->empty()) {
    return true;
  }

  char quote = value->front();
  if (quote != '"' && quote != '\'') {
    return true;
  }

  if (value->size() < 2 || value->back() != quote) {
    *error = "line " + std::to_string(lineNumber) + ": unterminated quote";
    return false;
  }

  std::string unquoted;
  for (size_t i = 1; i < value->size() - 1; i++) {
    char c = (*value)[i];
    if (c == '\\') {
      i++;
      if (i >= value->size() - 1) {
        *error =
            "line " + std::to_string(lineNumber) + ": unterminated escape";
        return false;
      }
      c = (*value)[i];
    } else if (c == quote) {
      *error = "line " + std::to_string(lineNumber) +
               ": unexpected text after closing quote";
      return false;
    }
    unquoted.push_back(c);
  }

  *value = unquoted;
  return true;
}

bool parseUnsignedInteger(const std::string& value, int* parsed) {
  if (value.empty()) {
    return false;
  }

  int result = 0;
  for (char c : value) {
    if (c < '0' || c > '9') {
      return false;
    }
    int digit = c - '0';
    if (result > (INT_MAX - digit) / 10) {
      return false;
    }
    result = result * 10 + digit;
  }

  *parsed = result;
  return true;
}

std::string readFileToString(File& f) {
  std::string contents;
  while (f.available()) {
    contents.push_back(static_cast<char>(f.read()));
  }

  return contents;
}

std::string quoteIniValue(const std::string& value) {
  std::string quoted{"\""};
  for (char c : value) {
    if (c == '"' || c == '\\') {
      quoted.push_back('\\');
    }
    quoted.push_back(c);
  }
  quoted.push_back('"');
  return quoted;
}

bool writeConfigToDisk(const XRPConfiguration& config,
                       const std::string& defaultBluetoothNameSuffix) {
  // Keep the previous configuration until the full replacement is committed.
  constexpr const char* temporaryPath = "/config.ini.tmp";
  std::string contents = config.toIniString(defaultBluetoothNameSuffix);
  File f = LittleFS.open(temporaryPath, "w");
  if (!f) {
    debug_log::println("[CONFIG] Failed to open config file for writing");
    return false;
  }

  size_t written = f.print(contents.c_str());
  f.close();
  // File::close() cannot report sync errors; verify before replacing the file.
  File verification = LittleFS.open(temporaryPath, "r");
  bool complete = written == contents.size() && verification &&
                  readFileToString(verification) == contents;
  verification.close();
  if (complete && LittleFS.rename(temporaryPath, XRP_CONFIG_PATH)) {
    return true;
  }
  LittleFS.remove(temporaryPath);
  debug_log::println("[CONFIG] Failed to save config file");
  return false;
}

XRPConfiguration resetToDefaultConfig(
    const std::string& defaultBluetoothNameSuffix) {
  XRPConfiguration config = generateDefaultConfig(defaultBluetoothNameSuffix);
  writeConfigToDisk(config, defaultBluetoothNameSuffix);
  return config;
}

bool parseConfigIni(const std::string& contents,
                    const std::string& defaultBluetoothNameSuffix,
                    XRPConfiguration* config, bool* shouldWrite,
                    std::string* error) {
  *config = generateDefaultConfig(defaultBluetoothNameSuffix);
  *shouldWrite = false;

  bool foundConfigVersion = false;
  int parsedConfigVersion = 0;
  std::string currentSection;
  size_t lineStart = 0;
  unsigned int lineNumber = 1;

  while (lineStart <= contents.size()) {
    size_t lineEnd = contents.find('\n', lineStart);
    if (lineEnd == std::string::npos) {
      lineEnd = contents.size();
    }

    std::string line = contents.substr(lineStart, lineEnd - lineStart);
    line = trimAsciiWhitespace(stripInlineComment(line));

    if (!line.empty()) {
      if (line.front() == '[') {
        if (line.back() != ']') {
          *error = "line " + std::to_string(lineNumber) +
                   ": malformed section header";
          return false;
        }

        currentSection =
            toLowerAscii(trimAsciiWhitespace(line.substr(1, line.size() - 2)));
      } else {
        size_t separator = line.find('=');
        if (separator == std::string::npos) {
          *error =
              "line " + std::to_string(lineNumber) + ": expected key = value";
          return false;
        }

        std::string key =
            toLowerAscii(trimAsciiWhitespace(line.substr(0, separator)));
        std::string value = trimAsciiWhitespace(line.substr(separator + 1));
        if (!unquoteIniValue(&value, error, lineNumber)) {
          return false;
        }

        if (currentSection.empty() &&
            (key == CONFIG_VERSION_KEY ||
             key == CONFIG_VERSION_CAMEL_CASE_KEY)) {
          foundConfigVersion = true;
          if (!parseUnsignedInteger(value, &parsedConfigVersion)) {
            *error = "line " + std::to_string(lineNumber) +
                     ": config_version must be an integer";
            return false;
          }
        } else if (currentSection == BLUETOOTH_SECTION &&
                   key == BLUETOOTH_DEVICE_NAME_KEY) {
          std::string configuredDeviceNameSuffix =
              normalizeBluetoothDeviceNameSuffix(value);
          if (isValidBluetoothDeviceNameSuffix(configuredDeviceNameSuffix)) {
            config->bluetoothConfig.deviceNameSuffix =
                configuredDeviceNameSuffix;
          } else {
            debug_log::println(
                "[CONFIG] Invalid Bluetooth device name. Using default");
            config->bluetoothConfig.deviceNameSuffix =
                defaultBluetoothNameSuffix;
            *shouldWrite = true;
          }
        }
      }
    }

    if (lineEnd == contents.size()) {
      break;
    }
    lineStart = lineEnd + 1;
    lineNumber++;
  }

  if (!foundConfigVersion) {
    *error = "missing config_version";
    return false;
  }

  if (parsedConfigVersion != XRP_CONFIG_VERSION) {
    *error = "configuration version mismatch";
    return false;
  }

  return true;
}

}  // namespace

std::string buildBluetoothDeviceName(const std::string& deviceNameSuffix) {
  return std::string(XRP_BLUETOOTH_NAME_PREFIX) + deviceNameSuffix;
}

std::string normalizeBluetoothDeviceNameSuffix(
    const std::string& deviceNameOrSuffix) {
  if (deviceNameOrSuffix.rfind(XRP_BLUETOOTH_NAME_PREFIX, 0) == 0) {
    return deviceNameOrSuffix.substr(XRP_BLUETOOTH_NAME_PREFIX_LENGTH);
  }

  return deviceNameOrSuffix;
}

bool isValidBluetoothDeviceNameSuffix(const std::string& deviceNameSuffix) {
  if (deviceNameSuffix.empty() ||
      deviceNameSuffix.length() > XRP_BLUETOOTH_NAME_SUFFIX_MAX_LENGTH) {
    return false;
  }

  for (char c : deviceNameSuffix) {
    if (c < 0x20 || c > 0x7e) {
      return false;
    }
  }

  return true;
}

XRPConfiguration generateDefaultConfig(
    const std::string& defaultBluetoothNameSuffix) {
  XRPConfiguration defaultConfig;
  defaultConfig.bluetoothConfig.deviceNameSuffix = defaultBluetoothNameSuffix;
  return defaultConfig;
}

std::string XRPConfiguration::toIniString(
    const std::string& defaultBluetoothNameSuffix) const {
  std::string defaultBluetoothDeviceName =
      buildBluetoothDeviceName(defaultBluetoothNameSuffix);
  std::string bluetoothDeviceName =
      buildBluetoothDeviceName(bluetoothConfig.deviceNameSuffix);
  std::string ret;
  ret += "# XRP firmware configuration\n";
  ret += "# Edit this file with a plain text editor, then restart the XRP.\n";
  ret += "# Lines starting with # or ; are comments.\n";
  ret += "# Inline comments are allowed after whitespace.\n";
  ret += "\n";
  ret += "config_version = " + std::to_string(XRP_CONFIG_VERSION) + "\n";
  ret += "\n";
  ret += "[bluetooth]\n";
  ret += "# The firmware always advertises Bluetooth names with the ";
  ret += XRP_BLUETOOTH_NAME_PREFIX;
  ret += " prefix.\n";
  ret += "# Use either a full WPIXRP- name or just the suffix after WPIXRP-.\n";
  ret += "# Suffix length: 1-";
  ret += std::to_string(XRP_BLUETOOTH_NAME_SUFFIX_MAX_LENGTH);
  ret += " printable ASCII characters.\n";
  ret += "# Default: ";
  ret += defaultBluetoothDeviceName;
  ret += "\n";
  if (bluetoothDeviceName == defaultBluetoothDeviceName) {
    ret += "# deviceName = ";
  } else {
    ret += "deviceName = ";
  }
  ret += quoteIniValue(bluetoothDeviceName);
  ret += "\n";
  return ret;
}

XRPConfiguration loadConfiguration(
    const std::string& defaultBluetoothNameSuffix) {
  File f = LittleFS.open(XRP_CONFIG_PATH, "r");
  if (!f) {
    debug_log::println("[CONFIG] No config file found. Creating default");
    return resetToDefaultConfig(defaultBluetoothNameSuffix);
  }

  std::string contents = readFileToString(f);
  f.close();

  XRPConfiguration config;
  bool shouldWrite = false;
  std::string error;
  if (!parseConfigIni(contents, defaultBluetoothNameSuffix, &config,
                      &shouldWrite, &error)) {
    debug_log::print("[CONFIG] Invalid config file: ");
    debug_log::println(error.c_str());
    debug_log::println("[CONFIG] Using default");
    return resetToDefaultConfig(defaultBluetoothNameSuffix);
  }

  if (shouldWrite) {
    writeConfigToDisk(config, defaultBluetoothNameSuffix);
  }

  return config;
}

bool saveBluetoothDeviceName(const std::string& deviceNameOrSuffix,
                             const std::string& defaultBluetoothNameSuffix) {
  std::string deviceNameSuffix =
      normalizeBluetoothDeviceNameSuffix(deviceNameOrSuffix);
  if (!isValidBluetoothDeviceNameSuffix(deviceNameSuffix)) {
    return false;
  }

  XRPConfiguration config = generateDefaultConfig(defaultBluetoothNameSuffix);
  config.bluetoothConfig.deviceNameSuffix = deviceNameSuffix;
  return writeConfigToDisk(config, defaultBluetoothNameSuffix);
}
