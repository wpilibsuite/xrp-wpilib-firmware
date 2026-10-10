#pragma once
#include <cstdint>
#include <cstdio>
inline uint32_t testMicros = 1000000;
inline uint32_t testMillisOffset = 0;
inline uint32_t millis() { return testMillisOffset + testMicros / 1000; }
