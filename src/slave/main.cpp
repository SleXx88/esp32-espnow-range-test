#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

#define DEFAULT_TXPWR 40    // 10 dBm

static uint8_t  gChannel = 1;
static uint8_t  gTxPower = DEFAULT_TXPWR;
static uint16_t gPaySize = PAYLOAD_MAX;

#ifndef RGB_BUILTIN
  #define RGB_BUILTIN 48
#endif

void applyWiFi() {
  esp_err_t e1 = esp_wifi_set_channel(gChannel, WIFI_SECOND_CHAN_NONE);
  esp_err_t e2 = esp_wifi_set_max_tx_power(gTxPower);
  Serial.printf("[Slave] applyWiFi: CH=%u err=%d, TX=%u err=%d\n", gChannel, e1, gTxPower, e2);
}

void flashLED() {
  neopixelWrite(RGB_BUILTIN,255,0,0);
  delay(80);
  neopixelWrite(RGB_BUILTIN,0,0,0);
}

// Master‑MAC und Link‑Status
uint8_t masterMac[6];
bool    linkOK = false;

void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len <= 0) return;
  PacketType tp = (PacketType)data[0];

  switch (tp) {
    case PACKET_TYPE_HANDSHAKE: {
      if (len < (int)sizeof(DataPacket)) break;
      auto *p = (const DataPacket*)data;
      if (computeCRC32((uint8_t*)p, sizeof(*p)-sizeof(p->crc)) != p->crc) {
        Serial.println("[Slave] HS CRC ERR");
        break;
      }
      // Einstellungen spiegeln
      gChannel = p->payload[0];
      gTxPower = p->payload[1];
      gPaySize  = uint16_t(p->payload[2]) | (uint16_t(p->payload[3])<<8);
      applyWiFi();

      memcpy(masterMac, mac, 6);
      linkOK = true;
      Serial.printf("[Slave] HS received → CH=%u, TX=%u, SIZE=%u\n", gChannel, gTxPower, gPaySize);

      // Peer anlegen
      {
        esp_now_peer_info_t pi{};
        memcpy(pi.peer_addr, mac, 6);
        pi.channel = gChannel;
        pi.encrypt = false;
        pi.ifidx   = WIFI_IF_STA;
        esp_now_add_peer(&pi);
      }

      // ACK zurück
      DataPacket ack{PACKET_TYPE_HANDSHAKE_ACK,0};
      memcpy(ack.payload, p->payload, 4);
      ack.crc = computeCRC32((uint8_t*)&ack, sizeof(ack)-sizeof(ack.crc));
      esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
      Serial.println("[Slave] HS_ACK sent");
      break;
    }

    case PACKET_TYPE_CONFIG: {
      if (len < (int)sizeof(ConfigPacket)) break;
      auto *c = (const ConfigPacket*)data;
      if (computeCRC32((uint8_t*)c, sizeof(*c)-sizeof(c->crc)) != c->crc) {
        Serial.println("[Slave] CFG CRC ERR");
        break;
      }
      Serial.printf("[Slave] CONFIG rcv → CH=%u, TX=%u, SIZE=%u\n",
                    c->channel, c->txPower, c->paySize);
      // ACK
      ConfigAckPacket ack{PACKET_TYPE_CONFIG_ACK,
                          c->channel, c->txPower, c->paySize, 0};
      ack.crc = computeCRC32((uint8_t*)&ack, sizeof(ack)-sizeof(ack.crc));
      esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
      Serial.println("[Slave] CONFIG_ACK sent");
      break;
    }

    case PACKET_TYPE_CONFIG_COMMIT: {
      if (len < (int)sizeof(ConfigCommitPacket)) break;
      auto *c = (const ConfigCommitPacket*)data;
      if (computeCRC32((uint8_t*)c, sizeof(*c)-sizeof(c->crc)) != c->crc) {
        Serial.println("[Slave] COMMIT CRC ERR");
        break;
      }
      // Anwenden
      gChannel = c->channel;
      gTxPower = c->txPower;
      gPaySize  = c->paySize;
      Serial.printf("[Slave] COMMIT rcv → apply CH=%u, TX=%u, SIZE=%u\n",
                    gChannel, gTxPower, gPaySize);
      applyWiFi();
      // Peer neu
      esp_now_del_peer(mac);
      {
        esp_now_peer_info_t pi{};
        memcpy(pi.peer_addr, mac, 6);
        pi.channel = gChannel;
        pi.encrypt = false;
        pi.ifidx   = WIFI_IF_STA;
        esp_now_add_peer(&pi);
      }
      // DONE senden
      ConfigDonePacket done{PACKET_TYPE_CONFIG_DONE,
                            gChannel, gTxPower, gPaySize, 0};
      done.crc = computeCRC32((uint8_t*)&done, sizeof(done)-sizeof(done.crc));
      esp_err_t e = esp_now_send(mac, (uint8_t*)&done, sizeof(done));
      Serial.printf("[Slave] CONFIG_DONE sent, err=%d\n", e);
      break;
    }

    case PACKET_TYPE_TEST: {
      if (len < (int)sizeof(DataPacket)) break;
      auto *p = (const DataPacket*)data;
      if (computeCRC32((uint8_t*)p, sizeof(*p)-sizeof(p->crc)) != p->crc) {
        Serial.println("[Slave] TEST CRC ERR");
        break;
      }
      flashLED();
      DataPacket rsp = *p;
      rsp.crc = computeCRC32((uint8_t*)&rsp, sizeof(rsp)-sizeof(rsp.crc));
      esp_now_send(mac, (uint8_t*)&rsp, sizeof(rsp));
      Serial.printf("[Slave] TEST ping‑pong id=%u\n", p->id);
      break;
    }

    default:
      Serial.printf("[Slave] Unknown packet type=%u\n", tp);
      break;
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  pinMode(RGB_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN,0,0,0);
  Serial.printf("SLAVE Boot → CH=%u TX=%u SIZE=%u\n", gChannel, gTxPower, gPaySize);

  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  applyWiFi();

  // ESPNOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("[Slave] ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  // nichts weiter hier, alles passiert in onRecv()
}
