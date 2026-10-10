#include "debug_log.h"

#include <Arduino.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "debug_log_usb.h"

namespace debug_log {
namespace {

std::atomic<uint32_t> busy{0};
std::atomic<uint32_t> dropped{0};
std::atomic<uint32_t> suppressed{0};
std::atomic<uint32_t> truncated{0};
char queue[QUEUE_CAPACITY];
char message[MAX_MESSAGE_SIZE];
size_t readPos = 0;
size_t used = 0;

struct RateLimit {
  uint32_t lastMs = 0;
  bool emitted = false;
};
RateLimit limits[static_cast<size_t>(Error::COUNT)];

// Interrupts or another core may interrupt a producer or the drain. Never spin
// on their queue ownership; a missed diagnostic is preferable to delaying them.
class TryLock {
 public:
  TryLock() : acquired(busy.exchange(1, std::memory_order_acquire) == 0) {}
  ~TryLock() {
    if (acquired) busy.store(0, std::memory_order_release);
  }
  explicit operator bool() const { return acquired; }

 private:
  bool acquired;
};

void enqueue(const char* format, va_list args) {
  if (used == QUEUE_CAPACITY) {
    dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  int result = vsnprintf(message, sizeof(message), format, args);
  if (result < 0) {
    dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  size_t length = static_cast<size_t>(result);
  if (length >= sizeof(message)) {
    length = sizeof(message) - 1;
    memcpy(message + length - 4, "...\n", 4);
    truncated.fetch_add(1, std::memory_order_relaxed);
  }
  if (length > QUEUE_CAPACITY - used) {
    dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  size_t writePos = (readPos + used) % QUEUE_CAPACITY;
  size_t first = std::min(length, QUEUE_CAPACITY - writePos);
  memcpy(queue + writePos, message, first);
  memcpy(queue, message + first, length - first);
  used += length;
}

}  // namespace

void log(const char* format, ...) {
  TryLock lock;
  if (!lock) {
    dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  va_list args;
  va_start(args, format);
  enqueue(format, args);
  va_end(args);
}

void print(const char* text) { log("%s", text); }
void println(const char* text) { log("%s\n", text); }

void logLimited(Error error, const char* format, ...) {
  TryLock lock;
  if (!lock) {
    dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  auto& limit = limits[static_cast<size_t>(error)];
  uint32_t now = millis();
  if (limit.emitted && static_cast<uint32_t>(now - limit.lastMs) < 1000) {
    suppressed.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  limit.emitted = true;
  limit.lastMs = now;
  va_list args;
  va_start(args, format);
  enqueue(format, args);
  va_end(args);
}

Counters counters() {
  return {dropped.load(std::memory_order_relaxed),
          suppressed.load(std::memory_order_relaxed),
          truncated.load(std::memory_order_relaxed)};
}

void drain() {
  TryLock lock;
  if (!lock || used == 0) return;
  size_t length = std::min({used, QUEUE_CAPACITY - readPos, DRAIN_BUDGET});
  size_t written = usb::tryWrite(queue + readPos, length);
  readPos = (readPos + written) % QUEUE_CAPACITY;
  used -= written;
}

}  // namespace debug_log
