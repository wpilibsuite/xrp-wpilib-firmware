#include "debug_log_usb.h"

#include <pico/mutex.h>
#include <tusb.h>

#include <algorithm>

// Arduino-Pico's USB IRQ shares this mutex with all foreground USB access.
extern mutex_t __usb_mutex;

namespace debug_log::usb {

size_t tryWrite(const char* data, size_t length) {
  if (!mutex_try_enter(&__usb_mutex, nullptr)) return 0;
  size_t written = 0;
  if (tud_cdc_connected()) {
    size_t available = tud_cdc_write_available();
    size_t count = std::min(length, available);
    if (count != 0) {
      written = tud_cdc_write(data, count);
      tud_cdc_write_flush();
    }
  }
  mutex_exit(&__usb_mutex);
  // The framework's USB IRQ services transfers. Do not run tud_task here:
  // it can dispatch unrelated USB callbacks inside the control loop.
  return written;
}

}  // namespace debug_log::usb
