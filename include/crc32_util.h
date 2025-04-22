#pragma once
#include <stdint.h>

/* Software‑CRC32 (Polynom 0xEDB88320) */
static inline uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++)
    crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1)));
  return crc;
}

static inline uint32_t computeCRC32(const uint8_t* buf, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFU;
  while (len--) crc = crc32_update(crc, *buf++);
  return ~crc;
}
