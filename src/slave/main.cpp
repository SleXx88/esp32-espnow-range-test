#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

// ----------------------------
// Slave Startwerte
// ----------------------------
static uint8_t  gChannel = 1;
static uint8_t  gTxPower = 8;   // => 2 dBm
static uint16_t gPaySize = PAYLOAD_SIZE; // Default 200

// ----------------------------

static uint8_t masterMac[6];
static bool    handshakeDone = false;

void applySlaveSettings() {
  esp_wifi_set_channel(gChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(gTxPower * 4);
}

// Callback: Daten empfangen
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len)
{
  if (len <= 0) return;
  PacketType ptype = (PacketType)data[0];

  if (ptype == PACKET_TYPE_HANDSHAKE) {
    // Master -> Broadcast
    if (len < (int)sizeof(DataPacket)) return;
    DataPacket rx;
    memcpy(&rx, data, sizeof(rx));
    uint32_t c = computeCRC32((uint8_t*)&rx, sizeof(rx) - sizeof(rx.crc));
    if (c != rx.crc) {
      Serial.println("[Slave] CRC-Fehler im HANDSHAKE-Paket");
      return;
    }
    Serial.println("[Slave] Handshake-Paket erhalten!");
    handshakeDone = true;
    memcpy(masterMac, mac, 6);

    // Peer anlegen
    if (!esp_now_is_peer_exist(mac)) {
      esp_now_peer_info_t pi = {};
      memcpy(pi.peer_addr, mac, 6);
      pi.channel = gChannel;
      pi.encrypt = false;
      pi.ifidx   = WIFI_IF_STA;
      esp_now_add_peer(&pi);
    }

    // Ack
    DataPacket ack;
    memset(&ack, 0, sizeof(ack));
    ack.type     = PACKET_TYPE_HANDSHAKE_ACK;
    ack.packetId = 0;
    for (int i=0; i<PAYLOAD_SIZE; i++) {
      ack.payload[i] = 0x55;
    }
    ack.crc = computeCRC32((uint8_t*)&ack, sizeof(ack) - sizeof(ack.crc));
    esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
    Serial.println("[Slave] Handshake-ACK an Master gesendet.");
  }
  else if (ptype == PACKET_TYPE_CONFIG) {
    // Master möchte Kanal/TxPower/Payload ändern
    if (len < (int)sizeof(ConfigPacket)) return;
    ConfigPacket cfg;
    memcpy(&cfg, data, sizeof(cfg));
    uint32_t c = computeCRC32((uint8_t*)&cfg, sizeof(cfg) - sizeof(cfg.crc));
    if (c != cfg.crc) {
      Serial.println("[Slave] CRC-Fehler im CONFIG-Paket");
      return;
    }
    Serial.printf("[Slave] ConfigPacket empfangen: CH=%d, TX=%d(%.2f dBm), SIZE=%d\n",
                  cfg.channel, cfg.txPower, 0.25f*cfg.txPower, cfg.paySize);

    // Zuerst Ack senden (PACKET_TYPE_CONFIG_ACK),
    // damit der Master es noch auf dem alten Kanal empfangen kann.
    ConfigAckPacket ack;
    ack.type    = PACKET_TYPE_CONFIG_ACK;
    ack.channel = cfg.channel;
    ack.txPower = cfg.txPower;
    ack.paySize = cfg.paySize;
    ack.crc     = computeCRC32((uint8_t*)&ack, sizeof(ack) - sizeof(ack.crc));
    esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
    Serial.println("[Slave] ConfigAck an Master gesendet.");

    // Jetzt wendet der Slave die neuen Werte an
    gChannel = cfg.channel;
    gTxPower = cfg.txPower;
    gPaySize = cfg.paySize;

    // Peer updaten
    if (esp_now_is_peer_exist(mac)) {
      esp_now_peer_info_t pi = {};
      memcpy(pi.peer_addr, mac, 6);
      pi.channel = gChannel;
      pi.encrypt = false;
      pi.ifidx   = WIFI_IF_STA;
      esp_now_del_peer(mac);
      esp_now_add_peer(&pi);
    }

    applySlaveSettings();
    Serial.println("[Slave] Neue Einstellungen uebernommen!");
  }
  else if (ptype == PACKET_TYPE_TEST) {
    // Ping-Pong
    if (len < (int)sizeof(DataPacket)) return;
    DataPacket rx;
    memcpy(&rx, data, sizeof(rx));
    uint32_t c = computeCRC32((uint8_t*)&rx, sizeof(rx) - sizeof(rx.crc));
    if (c != rx.crc) {
      Serial.println("[Slave] CRC-Fehler im TEST-Paket");
      return;
    }
    // Antwort
    if (!esp_now_is_peer_exist(mac)) {
      esp_now_peer_info_t pi = {};
      memcpy(pi.peer_addr, mac, 6);
      pi.channel = gChannel;
      pi.encrypt = false;
      pi.ifidx   = WIFI_IF_STA;
      esp_now_add_peer(&pi);
    }
    DataPacket tx = rx;
    tx.crc = computeCRC32((uint8_t*)&tx, sizeof(tx) - sizeof(tx.crc));
    esp_now_send(mac, (uint8_t*)&tx, sizeof(tx));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("=== SLAVE STARTET ===");
  Serial.printf("[Slave] Startwerte: CH=%d, TX=%d => %.2f dBm, SIZE=%d\n",
                gChannel, gTxPower, 0.25f*gTxPower, gPaySize);

  WiFi.mode(WIFI_STA);
  esp_wifi_start();

  applySlaveSettings();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[Slave] ESP-NOW Init fehlgeschlagen.");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("[Slave] Warte auf Broadcast-Handshakes...");
}

void loop()
{
  // Der Slave tut sonst nichts; er lauscht auf onDataRecv.
}
