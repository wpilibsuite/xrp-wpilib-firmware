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

void resetState() {
  currMaxSeq = 0;
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

  // Control byte essentially encodes the enabled/disabled state
  xrp::robotSetEnabled(ctrl == 1);

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

// ===================
// Message Encoders
// ===================

int writeEncoderData(int deviceId, int count, uint period, uint divisor, char* buffer, int offset) {
  // Encoder message is 14 bytes
  // tag(1) id(1) int(4) uint(4) uint(4)
  int i = offset;
  buffer[i++] = 2 + sizeof(int) + sizeof(uint) + sizeof(uint);
  buffer[i++] = XRP_TAG_ENCODER;
  buffer[i++] = deviceId & 0xFF;
  int32ToNetwork(count, buffer, i);
  i += sizeof(int);
  int32ToNetwork(period, buffer, i);
  i += sizeof(uint);
  int32ToNetwork(divisor, buffer, i);
  i += sizeof(uint);
  return i-offset; // +1 for the size byte
}

int writeDIOData(int deviceId, bool value, char* buffer, int offset) {
  // DIO Message is 3 bytes
  // tag(1) id(1) value(1)
  buffer[offset] = 3;
  buffer[offset+1] = XRP_TAG_DIO;
  buffer[offset+2] = deviceId & 0xFF;
  buffer[offset+3] = value ? 1 : 0;
  return 4; // +1 for the size byte
}

int writeGyroData(float rates[3], float angles[3], char* buffer, int offset) {
  // Gyro message is 25 bytes
  // tag(1) rateX(4) rateY(4) rateZ(4) angleX(4) angleY(4) angleZ(4)
  buffer[offset] = 25;
  buffer[offset+1] = XRP_TAG_GYRO;
  int ratePtr = offset + 2;
  int anglePtr = ratePtr + 12;
  for (int i = 0; i < 3; i++) {
    floatToNetwork(rates[i], buffer, ratePtr);
    floatToNetwork(angles[i], buffer, anglePtr);

    ratePtr += 4;
    anglePtr += 4;
  }
  return 26; // +1 for the size byte
}

int writeAccelData(float accels[3], char* buffer, int offset) {
  // Accel message is 13 bytes
  // tag(1) accX(4) accY(4) accZ(4)
  buffer[offset] = 13;
  buffer[offset+1] = XRP_TAG_ACCEL;
  int ptr = offset + 2;
  
  for (int i = 0; i < 3; i++) {
    floatToNetwork(accels[i], buffer, ptr);
    ptr += 4;
  }

  return 14; // +1 for the size byte
}

int writeAnalogData(int deviceId, float voltage, char* buffer, int offset) {
  // Analog message is 6 bytes
  // tag(1) id(1) value(4)
  buffer[offset] = 6;
  buffer[offset+1] = XRP_TAG_ANALOG;
  buffer[offset+2] = deviceId;
  floatToNetwork(voltage, buffer, offset+3);

  return 7; // +1 for size byte
}

} // namespace wpilibudp