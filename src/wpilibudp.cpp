#include "byteutils.h"
#include "wpilibudp.h"
#include "robot.h"
#include "watchdog.h"
#include "imu.h"

// Since we might (nay, will) rollover, the fudge factor lets us deal with cases like
// 65532, 65533, 0, 65534, 65535 by taking 0 as the new highest seq number
#define SEQ_FUDGE_FACTOR 5
#define SEQ_MAX 65535

namespace wpilibudp {

uint16_t currMaxSeq = 0;
xrp::Watchdog _dsWatchdog{"status"};

bool _processTaggedData(char* buffer, int start, int end) {
  // The data here is the 1 byte tag and n byte payload
  // range is [start, end) in buffer

  bool success = true;

  uint8_t tag = buffer[start];

  switch (tag) {
    case XRP_TAG_MOTOR: {
      // Verify size
      if (end - start < 6) {
        return false;
      }

      int channel = buffer[start+1];
      float value = networkToFloat(buffer, start+2);

      xrp::setPwmValue(channel, value);
    } break;
    case XRP_TAG_SERVO: {
      // Verify size
      if (end - start < 6) {
        return false;
      }

      int channel = buffer[start+1];
      float value = networkToFloat(buffer, start+2);

      // Servo position info comes as a 0 to 1 range
      // we need to convert to -1 to 1
      value = (2.0 * value) - 1.0;
      xrp::setPwmValue(channel, value);
    } break;
    case XRP_TAG_DIO: {
      if (end - start < 3) {
        return false;
      }

      int channel = buffer[start+1];
      bool value = buffer[start+2] == 1;

      xrp::setDigitalOutput(channel, value);
    } break;
    default:
      success = false;
  }

  return success;
}

bool dsWatchdogActive() {
  return _dsWatchdog.satisfied();
}

bool processPacket(char* buffer, int size) {
  if (size < 3) {
    return false;
  }

  int startIdx = 0;
  int endIdx = 0;

  // Overall packet format is
  //       2           1           n 
  // [    seq    ] [ ctrl ] [ tagged data ]

  uint16_t seq = networkToUInt16(buffer);
  uint8_t ctrl = buffer[2];

  // Check if the sequence number exceeds our latest seen seq number
  if (seq > currMaxSeq) {
    currMaxSeq = seq;
  }
  else {
    if (SEQ_MAX - seq < SEQ_FUDGE_FACTOR) {
      // Rollover
      currMaxSeq = seq;
    }
    else {
      // Not processing this
      return false;
    }
  }

  // TODO handle the enabled/disabled state

  // Feed the watchdog
  _dsWatchdog.feed();

  // Advance the start pointer
  startIdx = 3;

  // We might have multiple tags in the same packet, so we basically need to take chunks of this
  // [ size ] [ tag ] [      data       ]
  // size does NOT include the size byte itself

  while (startIdx < size) {
    // Read the size
    int msgSize = buffer[startIdx];
    endIdx = startIdx + msgSize + 1;

    // We pass in 1 past startIdx so that we only give the tag + payload
    bool result = _processTaggedData(buffer, startIdx+1, endIdx);

    // Advance the start pointer
    startIdx = endIdx;
  }

  return true;
}

} // namespace wpilibudp