#pragma once

#include <stddef.h>

namespace debug_log::usb {

// Return bytes accepted by the USB FIFO, never more than length. Return zero
// if the host is absent, the FIFO is full, or the USB mutex is already held.
size_t tryWrite(const char* data, size_t length);

}  // namespace debug_log::usb
