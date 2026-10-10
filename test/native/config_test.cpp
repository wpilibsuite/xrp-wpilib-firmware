#include "config.h"

#include <cassert>
#include <cstdio>

#include "LittleFS.h"

const std::string defaultName = "AAAA-BBBB";
std::string loadName(const std::string& contents) {
  LittleFS.files[XRP_CONFIG_PATH] = contents;
  return loadConfiguration(defaultName).bluetoothConfig.deviceNameSuffix;
}
int main() {
  assert(loadConfiguration(defaultName).bluetoothConfig.deviceNameSuffix ==
         defaultName);
  for (const std::string name :
       {"Robot", "WPIXRP-Bob", " \"#;=\\' ", "1234567890123456789"}) {
    assert(
        saveBluetoothDeviceName(buildBluetoothDeviceName(name), defaultName));
    assert(loadConfiguration(defaultName).bluetoothConfig.deviceNameSuffix ==
           name);
  }
  assert(loadName("config_version = 2\r\n[Bluetooth]\r\ndeviceName = 'Bot # 2' "
                  "; note\r\n") == "Bot # 2");
  assert(loadName("config_version = 2\n[bluetooth]\ndeviceName = WPIXRP-A # "
                  "comment\n") == "A");
  for (const std::string version :
       {"2147483648", "4294967298", "999999999999999999999999", "-2", "3"}) {
    assert(loadName("config_version = " + version +
                    "\n[bluetooth]\ndeviceName = Custom") == defaultName);
  }
  assert(
      loadName(
          "config_version = 2\n[bluetooth]\ndeviceName = \"Bot\" junk\"\n") ==
      defaultName);
  assert(loadName("config_version = 2\n[bluetooth]\ndeviceName = \"Bot\n") ==
         defaultName);
  assert(loadName("config_version = 2\n[bluetooth]\ndeviceName = WPIXRP-\n") ==
         defaultName);
  assert(!saveBluetoothDeviceName(std::string("bad\0name", 8), defaultName));
  assert(!saveBluetoothDeviceName(std::string(20, 'x'), defaultName));
  assert(saveBluetoothDeviceName("Saved", defaultName));
  const auto original = LittleFS.files[XRP_CONFIG_PATH];
  testWriteLimit = 10;
  assert(!saveBluetoothDeviceName("ShortWrite", defaultName));
  assert(LittleFS.files[XRP_CONFIG_PATH] == original);
  testWriteLimit = std::numeric_limits<size_t>::max();
  testCorruptClose = true;
  assert(!saveBluetoothDeviceName("FailedSync", defaultName));
  assert(LittleFS.files[XRP_CONFIG_PATH] == original);
  testCorruptClose = false;
  LittleFS.failRename = true;
  assert(!saveBluetoothDeviceName("FailedRename", defaultName));
  assert(LittleFS.files[XRP_CONFIG_PATH] == original);
  LittleFS.failRename = false;
  LittleFS.failOpen = true;
  assert(!saveBluetoothDeviceName("FailedOpen", defaultName));
  assert(LittleFS.files[XRP_CONFIG_PATH] == original);
  std::puts("configuration tests passed");
}
