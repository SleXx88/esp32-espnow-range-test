#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

// ── Konfiguration Debug-Ausgabe ─────────────────────────
#define SLAVE_DEBUG 1    // 1=an, 0=aus

// ── Defaults ────────────────────────────────────────────
#define DEFAULT_TXPWR 40    // 10 dBm

static uint8_t  gChannel = 1;
static uint8_t  gTxPower = DEFAULT_TXPWR;
static uint16_t gPaySize = PAYLOAD_MAX;

#ifndef RGB_BUILTIN
  #define RGB_BUILTIN 48
#endif

// ── Hilfen ──────────────────────────────────────────────
#if SLAVE_DEBUG
  #define DPRINT(...)    Serial.printf(__VA_ARGS__)
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)    do{}while(0)
  #define DPRINTLN(...)  do{}while(0)
#endif

void printLine() {
  Serial.println(F("----------------------------------------"));
}

// ── WLAN/Esp-Now Setup ───────────────────────────────────
void applyWiFi() {
  esp_err_t e1 = esp_wifi_set_channel(gChannel, WIFI_SECOND_CHAN_NONE);
  esp_err_t e2 = esp_wifi_set_max_tx_power(gTxPower);
  DPRINT("[D] applyWiFi: CH=%u err=%d, TX=%u err=%d\n", gChannel, e1, gTxPower, e2);
}

// ── Neopixel ■───────────────────────────────────────────
void flashLED() {
  neopixelWrite(RGB_BUILTIN,255,0,0);
  delay(80);
  neopixelWrite(RGB_BUILTIN,0,0,0);
}

// ── State ───────────────────────────────────────────────
uint8_t masterMac[6];
bool    linkOK = false;

// ── Empfangs-Callback ───────────────────────────────────
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 1) return;
  PacketType tp = (PacketType)data[0];

  switch (tp) {

    case PACKET_TYPE_HANDSHAKE: {
      if (len < (int)sizeof(DataPacket)) break;
      auto *p = (const DataPacket*)data;
      if (computeCRC32((uint8_t*)p, offsetof(DataPacket, crc)) != p->crc) {
        Serial.println("⨯ Handshake CRC-Fehler");
        break;
      }

      // Settings übernehmen
      gChannel = p->payload[0];
      gTxPower = p->payload[1];
      gPaySize  = uint16_t(p->payload[2]) | (uint16_t(p->payload[3])<<8);
      applyWiFi();

      memcpy(masterMac, mac, 6);
      linkOK = true;

      printLine();
      Serial.println("⇆ Handshake empfangen");
      Serial.printf("   Kanal:      %u\n", gChannel);
      Serial.printf("   Tx-Power:   %u (%.2f dBm)\n", gTxPower, 0.25f*gTxPower);
      Serial.printf("   Nutzlast:   %u B\n", gPaySize);
      printLine();

      // Peer anlegen
      {
        esp_now_peer_info_t pi{};
        memcpy(pi.peer_addr, mac, 6);
        pi.channel = gChannel;
        pi.ifidx   = WIFI_IF_STA;
        esp_now_add_peer(&pi);
      }

      // ACK zurück
      DataPacket ack{PACKET_TYPE_HANDSHAKE_ACK, 0};
      memcpy(ack.payload, p->payload, 4);
      ack.crc = computeCRC32((uint8_t*)&ack, offsetof(DataPacket, crc));
      esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));

      Serial.println("✓ Handshake-ACK versendet");
      break;
    }

    case PACKET_TYPE_CONFIG: {
      if (len < (int)sizeof(ConfigPacket)) break;
      auto *c = (const ConfigPacket*)data;
      if (computeCRC32((uint8_t*)c, offsetof(ConfigPacket, crc)) != c->crc) {
        Serial.println("⨯ Config CRC-Fehler");
        break;
      }

      printLine();
      Serial.println("⇆ Config-Einstellungen empfangen");
      Serial.printf("   Kanal:      %u\n", c->channel);
      Serial.printf("   Tx-Power:   %u (%.2f dBm)\n", c->txPower, 0.25f*c->txPower);
      Serial.printf("   Nutzlast:   %u B\n", c->paySize);
      printLine();

      // ACK
      ConfigAckPacket ack{PACKET_TYPE_CONFIG_ACK,
                          c->channel, c->txPower, c->paySize, 0};
      ack.crc = computeCRC32((uint8_t*)&ack, offsetof(ConfigAckPacket, crc));
      esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));

      Serial.println("✓ Config-ACK versendet");
      break;
    }

    case PACKET_TYPE_CONFIG_COMMIT: {
      if (len < (int)sizeof(ConfigPacket)) break;
      auto *c = (const ConfigPacket*)data;
      if (computeCRC32((uint8_t*)c, offsetof(ConfigPacket, crc)) != c->crc) {
        Serial.println("⨯ Commit CRC-Fehler");
        break;
      }

      // übernehmen
      gChannel = c->channel;
      gTxPower = c->txPower;
      gPaySize  = c->paySize;
      Serial.println("⇆ Config-Commit erhalten");
      printLine();
      Serial.printf("   Übernehme → CH=%u, TX=%u, SIZE=%u\n", gChannel, gTxPower, gPaySize);
      printLine();

      applyWiFi();
      esp_now_del_peer(mac);
      {
        esp_now_peer_info_t pi{};
        memcpy(pi.peer_addr, mac, 6);
        pi.channel = gChannel;
        pi.ifidx   = WIFI_IF_STA;
        esp_now_add_peer(&pi);
      }

      // DONE
      ConfigDonePacket done{PACKET_TYPE_CONFIG_DONE,
                            gChannel, gTxPower, gPaySize, 0};
      done.crc = computeCRC32((uint8_t*)&done, offsetof(ConfigDonePacket, crc));
      esp_now_send(mac, (uint8_t*)&done, sizeof(done));

      Serial.println("✓ Config-DONE versendet");
      break;
    }

    case PACKET_TYPE_TEST: {
      if (len < (int)sizeof(DataPacket)) break;
      auto *p = (const DataPacket*)data;
      if (computeCRC32((uint8_t*)p, offsetof(DataPacket, crc)) != p->crc) {
        Serial.println("⨯ Test CRC-Fehler");
        break;
      }

      flashLED();
      DataPacket rsp = *p;
      rsp.crc = computeCRC32((uint8_t*)&rsp, offsetof(DataPacket, crc));
      esp_now_send(mac, (uint8_t*)&rsp, sizeof(rsp));

      DPRINT("[D] TEST id=%u empfangen, zurückgesendet\n", p->id);
      break;
    }

    default:
      Serial.printf("⨯ Unbekannter Paket-Typ: %u\n", tp);
      break;
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  pinMode(RGB_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 0,0,0);
  printLine();
  Serial.printf("🔌 SLAVE Start → CH=%u, TX=%u , SIZE=%u\n", gChannel, gTxPower, gPaySize);
  printLine();

  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  applyWiFi();

  if (esp_now_init() != ESP_OK) {
    Serial.println("⨯ ESP-NOW Init fehlgeschlagen");
    while(true) delay(100);
  }
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  // alles in onRecv abgehandelt
}
