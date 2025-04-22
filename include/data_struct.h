#pragma once
#include <stdint.h>

/* ESP‑NOW‑Grenzen */
#define DATA_OVERHEAD  9
#define PAYLOAD_MAX    241

/* 1‑Byte‑Packing */
#ifdef __GNUC__
  #define PACKED __attribute__((packed))
#else
  #define PACKED
#endif

typedef enum : uint8_t {
  PACKET_TYPE_HANDSHAKE       = 1,
  PACKET_TYPE_HANDSHAKE_ACK   = 2,
  PACKET_TYPE_CONFIG          = 3,
  PACKET_TYPE_CONFIG_ACK      = 4,
  PACKET_TYPE_CONFIG_COMMIT   = 5,
  PACKET_TYPE_CONFIG_DONE     = 6,
  PACKET_TYPE_TEST            = 7
} PacketType;

/* Handshake & Test */
typedef struct PACKED {
  uint8_t  type;
  uint32_t id;
  uint8_t  payload[PAYLOAD_MAX];
  uint32_t crc;
} DataPacket;

/* Config, ACK, Commit, Done */
typedef struct PACKED {
  uint8_t  type;
  uint8_t  channel;
  uint8_t  txPower;
  uint16_t paySize;
  uint32_t crc;
} ConfigPacket;

/* Alias‑Typen */
typedef ConfigPacket ConfigAckPacket;
typedef ConfigPacket ConfigCommitPacket;
typedef ConfigPacket ConfigDonePacket;
