#include <cstring>

#include "byteutils.h"

// NOTE: The RP2040 is Little Endian, and Network Byte Order is Big Endian
float networkToFloat(char* buf) {
  return networkToFloat(buf, 0);
}

float networkToFloat(char* buf, int offset) {
  float f;
  unsigned char b[] = {buf[offset+3], buf[offset+2], buf[offset+1], buf[offset+0]};
  memcpy(&f, &b, sizeof(f));
  return f;
}

int16_t networkToInt16(char* buf) {
  int16_t i;
  unsigned char b[] = {buf[1], buf[0]};
  memcpy(&i, &b, sizeof(i));
  return i;
}

uint16_t networkToUInt16(char* buf) {
  uint16_t u;
  unsigned char b[] = {buf[1], buf[0]};
  memcpy(&u, &b, sizeof(u));
  return u;
}

void floatToNetwork(float num, char* buf) {
  unsigned char b[4];
  memcpy(&b, &num, sizeof(num));

  buf[0] = b[3];
  buf[1] = b[2];
  buf[2] = b[1];
  buf[3] = b[0];
}

void int16ToNetwork(int16_t num, char* buf) {
  unsigned char b[2];
  memcpy(&b, &num, sizeof(num));

  buf[0] = b[1];
  buf[1] = b[0];
}

void uint16ToNetwork(uint16_t num, char* buf) {
  unsigned char b[2];
  memcpy(&b, &num, sizeof(num));

  buf[0] = b[1];
  buf[1] = b[0];
}