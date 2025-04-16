#ifndef CRC32_UTIL_H
#define CRC32_UTIL_H

#include <stddef.h>
#include <stdint.h>

// Einfache CRC32-Funktion (Standardpolynom 0xEDB88320)
inline uint32_t computeCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (int j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

#endif // CRC32_UTIL_H
