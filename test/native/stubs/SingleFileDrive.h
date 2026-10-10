#pragma once
#include <cstdint>
struct TestSingleFileDrive {
  void onPlug(void (*)(uint32_t)) {}
  void onUnplug(void (*)(uint32_t)) {}
  void begin(const char*, const char*) {}
};
inline TestSingleFileDrive singleFileDrive;
