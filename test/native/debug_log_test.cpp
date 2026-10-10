#include "debug_log.h"

#include <cassert>
#include <string>

#include "Arduino.h"
#include "tusb.h"

namespace {
void drainAll() {
  // Even a full queue must empty within this bound when USB accepts data.
  for (size_t i = 0; i < debug_log::QUEUE_CAPACITY; ++i) {
    testUsbSpace = 256;
    debug_log::drain();
  }
}
}  // namespace

int main() {
  using namespace debug_log;
  log("packet count: %d\n", 42);
  assert(testUsbCalls == 0);  // Producers never access USB.
  for (int i = 0; i < 100; ++i) drain();
  assert(testUsbWrites == 0 && testUsbOutput.empty());

  testUsbConnected = true;
  // A stalled reader does not consume the message or trigger a USB write.
  for (int i = 0; i < 100; ++i) drain();
  assert(testUsbWrites == 0 && testUsbOutput.empty());
  testUsbSpace = 256;
  __usb_mutex.locked = true;
  auto calls = testUsbCalls;
  drain();
  assert(testUsbCalls == calls && __usb_mutex.locked);
  __usb_mutex.locked = false;

  // A short driver write preserves the unsent suffix for a later loop.
  testUsbSpace = 3;
  testUsbWriteLimit = 0;
  drain();
  assert(testUsbOutput.empty());
  testUsbWriteLimit = 2;
  drain();
  assert(testUsbOutput == "pa");
  testUsbWriteLimit = UINT32_MAX;
  drainAll();
  assert(testUsbOutput == "packet count: 42\n");
  testUsbOutput.clear();

  // Each drain is a single, bounded write, even with many messages queued.
  std::string line(100, 'a');
  log("%s", line.c_str());
  testUsbSpace = 256;
  auto writes = testUsbWrites;
  drain();
  assert(testUsbWrites == writes + 1);
  assert(testUsbOutput.size() == DRAIN_BUDGET);
  drainAll();
  assert(testUsbOutput == line);
  testUsbOutput.clear();

  // An interrupting producer or drain drops/skips instead of waiting for the
  // foreground queue owner. Accepted messages remain ordered and intact.
  print("before\n");
  auto drops = counters().dropped;
  testDuringUsbWrite = [] {
    log("interrupted\n");
    drain();
  };
  drain();
  testDuringUsbWrite = nullptr;
  assert(counters().dropped == drops + 1);
  print("after\n");
  drainAll();
  assert(testUsbOutput == "before\nafter\n");
  testUsbOutput.clear();

  // Exercise ring wraparound and full-queue drops without partial messages.
  std::string expected;
  drops = counters().dropped;
  for (size_t i = 0; i < QUEUE_CAPACITY + 10; ++i) print("x");
  assert(counters().dropped == drops + 10);
  drainAll();
  assert(testUsbOutput == std::string(QUEUE_CAPACITY, 'x'));
  testUsbOutput.clear();
  for (size_t i = 0; i < QUEUE_CAPACITY - 3; ++i) print("x");
  drops = counters().dropped;
  print("cannot fit\n");
  assert(counters().dropped == drops + 1);
  drainAll();
  assert(testUsbOutput == std::string(QUEUE_CAPACITY - 3, 'x'));
  testUsbOutput.clear();
  for (int i = 0; i < 100; ++i) {
    std::string item = std::to_string(i) + std::string(83, 'b') + "\n";
    print(item.c_str());
    expected += item;
    for (int j = 0; j < 3; ++j) {
      testUsbSpace = 256;
      drain();
    }
  }
  drainAll();
  assert(testUsbOutput == expected);
  testUsbOutput.clear();

  std::string huge(MAX_MESSAGE_SIZE * 2, 'c');
  print(huge.c_str());
  drainAll();
  assert(testUsbOutput.size() == MAX_MESSAGE_SIZE - 1);
  assert(testUsbOutput.substr(testUsbOutput.size() - 4) == "...\n");
  assert(counters().truncated == 1);
  testUsbOutput.clear();

  // Rate limits are independent per category and handle millis() rollover.
  testMicros = 0;
  testMillisOffset = UINT32_MAX - 500;
  logLimited(Error::L2CAP_SEND, "first\n");
  for (int i = 0; i < 100; ++i) logLimited(Error::L2CAP_SEND, "repeat\n");
  logLimited(Error::GATT_SEND, "other\n");
  testMillisOffset = 200;
  logLimited(Error::L2CAP_SEND, "too soon\n");
  testMillisOffset = 500;
  logLimited(Error::L2CAP_SEND, "next\n");
  drainAll();
  assert(testUsbOutput == "first\nother\nnext\n");
  assert(counters().suppressed == 101);
  assert(testUsbFlushes == testUsbWrites);
  std::puts("nonblocking diagnostic log tests passed");
}
