#pragma once
#include <cassert>
#include <cstdint>

struct mutex_t {
  bool locked = false;
};
inline mutex_t __usb_mutex;
inline unsigned testUsbLockAttempts = 0;
inline bool mutex_try_enter(mutex_t* mutex, uint32_t*) {
  ++testUsbLockAttempts;
  if (mutex->locked) return false;
  mutex->locked = true;
  return true;
}
inline void mutex_exit(mutex_t* mutex) {
  assert(mutex->locked);
  mutex->locked = false;
}
