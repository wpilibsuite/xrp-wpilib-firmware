#pragma once
#include <cassert>
inline unsigned testBluetoothLockDepth = 0;
struct BluetoothLock {
  BluetoothLock() { ++testBluetoothLockDepth; }
  ~BluetoothLock() { --testBluetoothLockDepth; }
};
inline void requireBluetoothLock() { assert(testBluetoothLockDepth > 0); }
