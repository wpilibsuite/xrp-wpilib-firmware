#pragma once

#include <stddef.h>
#include <stdint.h>

namespace debug_log {

constexpr size_t QUEUE_CAPACITY = 4096;
constexpr size_t MAX_MESSAGE_SIZE = 768;
constexpr size_t DRAIN_BUDGET = 64;

enum class Error {
  L2CAP_SEND,
  GATT_SEND,
  GATT_REQUEST,
  ENCODER_OVERRUN,
  COUNT
};

struct Counters {
  uint32_t dropped;
  uint32_t suppressed;
  uint32_t truncated;
};

// Queue formatted text in fixed buffers without waiting for another caller.
// A full or busy queue drops the message. Oversized messages end with "...\n".
void log(const char* format, ...) __attribute__((format(printf, 1, 2)));
void print(const char* text);
void println(const char* text);

// Emit at most one message per error category per second, counting omissions.
void logLimited(Error error, const char* format, ...)
    __attribute__((format(printf, 2, 3)));
Counters counters();

// Call from the main loop after control and watchdog work. Writes at most
// DRAIN_BUDGET bytes to USB without waiting for host progress or a USB mutex.
void drain();

}  // namespace debug_log
