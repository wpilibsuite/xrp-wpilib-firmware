#pragma once

#include <stdint.h>
/**
 * Decode a float from a 4-byte buffer in Network Byte order
 */
float networkToFloat(char* buf);
float networkToFloat(char* buf, int offset);

/**
 * Decode an int16 from a 4-byte buffer in Network Byte order
 */
int16_t networkToInt16(char* buf);

/**
 * Decode an uint16 from a 4-byte buffer in Network Byte order
 */
uint16_t networkToUInt16(char* buf);

/**
 * Encode a float to a buffer in Network Byte Order
*/
void floatToNetwork(float num, char* buf);

/**
 * Encode an Int16 to a buffer in Network Byte Order
*/
void int16ToNetwork(int16_t num, char* buf);

/**
 * Encode a UInt16 to a buffer in Network Byte Order
*/
void uint16ToNetwork(uint16_t num, char* buf);