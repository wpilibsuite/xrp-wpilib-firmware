#pragma once
#include <cstdint>
#include <cstdlib>

#include "BluetoothLock.h"
class UUID {
 public:
  explicit UUID(const char* value) {
    for (unsigned i = 0; i < 16; ++i) {
      if (*value == '-') ++value;
      char byte[] = {value[0], value[1], 0};
      bytes[i] = std::strtoul(byte, nullptr, 16);
      value += 2;
    }
  }
  const uint8_t* getUuid() const { return bytes; }

 private:
  uint8_t bytes[16];
};
struct TestBTstack {
  uint16_t nextHandle = 10;
  void addGATTService(UUID*) { requireBluetoothLock(); }
  uint16_t addGATTCharacteristicDynamic(UUID*, uint16_t, uint16_t) {
    requireBluetoothLock();
    nextHandle += 3;
    return nextHandle;
  }
  void setup(const char*) { requireBluetoothLock(); }
  void setAdvData(uint16_t, const uint8_t*) { requireBluetoothLock(); }
  void setScanData(uint16_t, const uint8_t*) { requireBluetoothLock(); }
  void startAdvertising() { requireBluetoothLock(); }
};
inline TestBTstack BTstack;
