#include <cstring>

#include "byteutils.h"

// NOTE: The RP2040 is Little Endian, and Network Byte Order is Big Endian

float networkToFloat(char* buf, int offset) {
  float f;
  unsigned char b[] = {buf[offset+3], buf[offset+2], buf[offset+1], buf[offset+0]};
  memcpy(&f, &b, sizeof(f));
  return f;
}

int16_t networkToInt16(char* buf, int offset) {
  int16_t i;
  unsigned char b[] = {buf[offset+1], buf[offset+0]};
  memcpy(&i, &b, sizeof(i));
  return i;
}

uint16_t networkToUInt16(char* buf, int offset) {
  uint16_t u;
  unsigned char b[] = {buf[offset+1], buf[offset+0]};
  memcpy(&u, &b, sizeof(u));
  return u;
}

int32_t networkToInt32(char* buf, int offset) {
  int32_t i;
  unsigned char b[] = {buf[offset+3], buf[offset+2], buf[offset+1], buf[offset+0]};
  memcpy(&i, &b, sizeof(i));
  return i;
}

uint32_t networkToUInt32(char* buf, int offset) {
  uint32_t u;
  unsigned char b[] = {buf[offset+3], buf[offset+2], buf[offset+1], buf[offset+0]};
  memcpy(&u, &b, sizeof(u));
  return u;
}

void floatToNetwork(float num, char* buf, int offset) {
  unsigned char b[4];
  memcpy(&b, &num, sizeof(num));

  buf[offset+0] = b[3];
  buf[offset+1] = b[2];
  buf[offset+2] = b[1];
  buf[offset+3] = b[0];
}

void doubleToNetwork(double num, char* buf, int offset) {
  unsigned char* b = (unsigned char *)&num;
  for(int i=0, j=sizeof(double)-1; i < sizeof(double); ++i, --j) {
    buf[offset+i] = b[j];
  }
}

void int16ToNetwork(int16_t num, char* buf, int offset) {
  unsigned char b[2];
  memcpy(&b, &num, sizeof(num));

  buf[offset+0] = b[1];
  buf[offset+1] = b[0];
}

void uint16ToNetwork(uint16_t num, char* buf, int offset) {
  unsigned char b[2];
  memcpy(&b, &num, sizeof(num));

  buf[offset+0] = b[1];
  buf[offset+1] = b[0];
}

void int32ToNetwork(int32_t num, char* buf, int offset) {
  unsigned char b[4];
  memcpy(&b, &num, sizeof(num));

  buf[offset+0] = b[3];
  buf[offset+1] = b[2];
  buf[offset+2] = b[1];
  buf[offset+3] = b[0];
}

void uint32ToNetwork(uint32_t num, char* buf, int offset) {
  unsigned char b[4];
  memcpy(&b, &num, sizeof(num));

  buf[offset+0] = b[3];
  buf[offset+1] = b[2];
  buf[offset+2] = b[1];
  buf[offset+3] = b[0];
}