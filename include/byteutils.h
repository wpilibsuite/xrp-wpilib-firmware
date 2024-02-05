#pragma once

#include <stdint.h>
/**
 * Decode a float from a 4-byte buffer in Network Byte order
 */
float networkToFloat(char* buf, int offset = 0);

/**
 * Decode an int16 from a 2-byte buffer in Network Byte order
 */
int16_t networkToInt16(char* buf, int offset = 0);

/**
 * Decode an uint16 from a 2-byte buffer in Network Byte order
 */
uint16_t networkToUInt16(char* buf, int offset = 0);

/**
 * Decode an int32 from a 4-byte buffer in Network Byte order
 */
int32_t networkToInt32(char* buf, int offset = 0);

/**
 * Decode an uint32 from a 4-byte buffer in Network Byte order
 */
uint32_t networkToUInt32(char* buf, int offset = 0);

/**
 * Encode a float to a buffer in Network Byte Order
*/
void floatToNetwork(float num, char* buf, int offset = 0);

/**
 * Encode a double to a buffer in Network Byte Order
*/
void doubleToNetwork(double num, char* buf, int offset = 0);

/**
 * Encode an Int16 to a buffer in Network Byte Order
*/
void int16ToNetwork(int16_t num, char* buf, int offset = 0);

/**
 * Encode a UInt16 to a buffer in Network Byte Order
*/
void uint16ToNetwork(uint16_t num, char* buf, int offset = 0);

/**
 * Encode an Int32 to a buffer in Network Byte Order
*/
void int32ToNetwork(int32_t num, char* buf, int offset = 0);

/**
 * Encode a UInt32 to a buffer in Network Byte Order
*/
void uint32ToNetwork(uint32_t num, char* buf, int offset = 0);