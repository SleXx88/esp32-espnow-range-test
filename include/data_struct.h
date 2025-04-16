#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

#include <stdint.h>

// Größe des Daten-Payloads nach Bedarf
#define PAYLOAD_SIZE 200

// Mögliche Pakettypen im Protokoll
enum PacketType : uint8_t {
    PACKET_TYPE_HANDSHAKE      = 1,
    PACKET_TYPE_HANDSHAKE_ACK  = 2,
    PACKET_TYPE_TEST           = 3
};

// Damit die Struktur "kompakt" ohne Auffüllbytes bleibt
#pragma pack(push, 1)
typedef struct {
    PacketType type;       // Art des Pakets
    uint32_t   packetId;   // z.B. fortlaufende ID
    uint8_t    payload[PAYLOAD_SIZE];
    uint32_t   crc;        // CRC über (type, packetId, payload)
} DataPacket;
#pragma pack(pop)

#endif // DATA_STRUCT_H
