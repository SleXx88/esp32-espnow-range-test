#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

#include <stdint.h>

// Mögliche PacketTypes
enum PacketType : uint8_t {
    PACKET_TYPE_HANDSHAKE      = 1, // Master -> Broadcast
    PACKET_TYPE_HANDSHAKE_ACK  = 2, // Slave -> Master
    PACKET_TYPE_TEST           = 3, // Ping-Pong
    PACKET_TYPE_CONFIG         = 4, // Master -> Slave (Parameter ändern)
    PACKET_TYPE_CONFIG_ACK     = 5  // Slave -> Master (Bestätigung)
};

// Falls der Master mit #define PAYLOAD_SIZE X in main.cpp kommt, 
// sonst Default 200:
#ifndef PAYLOAD_SIZE
  #define PAYLOAD_SIZE 200
#endif

#pragma pack(push, 1)
typedef struct {
    PacketType type;
    uint32_t   packetId;
    uint8_t    payload[PAYLOAD_SIZE];
    uint32_t   crc;
} DataPacket;

// Master -> Slave
typedef struct {
    PacketType type;       // PACKET_TYPE_CONFIG
    uint8_t    channel;    // Neuer Kanal [1..13]
    uint8_t    txPower;    // [1..80] => 0.25 * txPower = dBm
    uint16_t   paySize;    // [1..250]
    uint32_t   crc;
} ConfigPacket;

// Slave -> Master
typedef struct {
    PacketType type;       // PACKET_TYPE_CONFIG_ACK
    uint8_t    channel;    // Bestätigter Kanal
    uint8_t    txPower;    // Bestätigte Sendeleistung
    uint16_t   paySize;    // Bestätigte Payload-Größe
    uint32_t   crc;
} ConfigAckPacket;
#pragma pack(pop)

#endif // DATA_STRUCT_H
