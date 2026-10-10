#pragma once
#include <pico/mutex.h>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <string>

inline bool testUsbConnected = false;
inline uint32_t testUsbSpace = 0;
inline uint32_t testUsbWriteLimit = std::numeric_limits<uint32_t>::max();
inline unsigned testUsbCalls = 0;
inline unsigned testUsbWrites = 0;
inline unsigned testUsbFlushes = 0;
inline std::string testUsbOutput;
inline void (*testDuringUsbWrite)() = nullptr;

inline bool tud_cdc_connected() {
  assert(__usb_mutex.locked);
  ++testUsbCalls;
  return testUsbConnected;
}
inline uint32_t tud_cdc_write_available() {
  assert(__usb_mutex.locked);
  ++testUsbCalls;
  return testUsbSpace;
}
inline uint32_t tud_cdc_write(const void* data, uint32_t length) {
  assert(__usb_mutex.locked && testUsbConnected && length <= testUsbSpace);
  ++testUsbCalls;
  ++testUsbWrites;
  if (testDuringUsbWrite) testDuringUsbWrite();
  auto written = std::min(length, testUsbWriteLimit);
  testUsbOutput.append(static_cast<const char*>(data), written);
  testUsbSpace -= written;
  return written;
}
inline uint32_t tud_cdc_write_flush() {
  assert(__usb_mutex.locked);
  ++testUsbCalls;
  ++testUsbFlushes;
  return 0;
}
