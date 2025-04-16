#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

// ------------------------------------------
// Master-Einstellungen
// ------------------------------------------

// Überschreibt in data_struct.h die PAYLOAD_SIZE (Standard=200).
#define PAYLOAD_SIZE      200

#define SERIAL_BAUDRATE   115200
#define REPORT_INTERVAL_MS 1000

// Unsere Startwerte. Können per Serial geändert werden.
static uint8_t  gChannel = 1;  // 1..13
static uint8_t  gTxPower = 8;  // => 2 dBm
static uint16_t gPaySize = PAYLOAD_SIZE;

// ------------------------------------------

static const uint8_t BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t slaveMac[6];
static bool handshakeDone = false;

// Statistik
static uint32_t sentPacketsThisInterval     = 0;
static uint32_t receivedPacketsThisInterval = 0;
static uint32_t lastReportTime              = 0;

// RSSI
static int8_t   lastRSSI  = 0;
static int32_t  rssiSum   = 0;
static uint32_t rssiCount = 0;

// ------------------------------------------
// Hilfsfunktionen
// ------------------------------------------

void applyMasterSettings() {
  esp_wifi_set_channel(gChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(gTxPower * 4);
}

void sendHandshakePacket() {
  DataPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.type = PACKET_TYPE_HANDSHAKE;
  pkt.packetId = 0;
  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    pkt.payload[i] = 0xAA;
  }
  pkt.crc = computeCRC32((uint8_t*)&pkt, sizeof(pkt) - sizeof(pkt.crc));
  esp_err_t e = esp_now_send(BROADCAST_ADDR, (uint8_t*)&pkt, sizeof(pkt));
  if (e == ESP_OK) {
    Serial.printf("[Handshake] Sende auf Kanal %d...\n", gChannel);
  } else {
    Serial.printf("[Handshake] Sendefehler=%d\n", e);
  }
}

// Multi-Channel-Scan: Kanäle 1..13 durchprobieren,
// bis handshakeDone == true
void multiChannelScan() {
  handshakeDone = false;
  for (int ch = 1; ch <= 13; ch++) {
    gChannel = (uint8_t)ch;
    applyMasterSettings();
    Serial.printf("[Scan] Teste Kanal %d...\n", ch);

    uint32_t start = millis();
    bool found = false;
    while (!handshakeDone && (millis() - start < 1500)) {
      static uint32_t lastHS = 0;
      if (millis() - lastHS > 300) {
        sendHandshakePacket();
        lastHS = millis();
      }
      delay(50);
      if (handshakeDone) {
        found = true;
        break;
      }
    }
    if (found) {
      Serial.printf("Slave gefunden auf Kanal %d\n", ch);
      break;
    }
  }
  if (!handshakeDone) {
    Serial.println("Kein Slave auf 1..13 gefunden!");
  }
}

uint32_t getDynamicSendInterval() {
  // Pro 10 Byte 1ms, min. 10ms
  uint32_t interval = gPaySize / 10;
  if (interval < 10) interval = 10;
  return interval;
}

// Serielle Eingabe: CH=, TXPWR=, SIZE=
void handleSerialInput() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // CH=x
    if (line.startsWith("CH=")) {
      int val = line.substring(3).toInt();
      if (val >= 1 && val <= 13) {
        // Schicke ConfigPacket an Slave
        ConfigPacket cfg;
        cfg.type     = PACKET_TYPE_CONFIG;
        cfg.channel  = (uint8_t)val;
        cfg.txPower  = gTxPower;  // alter TxPower
        cfg.paySize  = gPaySize;  // Payload
        cfg.crc      = computeCRC32((uint8_t*)&cfg, sizeof(cfg) - sizeof(cfg.crc));

        if (handshakeDone) {
          esp_now_send(slaveMac, (uint8_t*)&cfg, sizeof(cfg));
          Serial.printf("[Master] Sende Config (CH=%d) an Slave.\n", val);
        } else {
          Serial.println("Kein Handshake erfolgt. Multi-Channel-Scan noetig?");
        }
      } else {
        Serial.println("Ungueltiger Kanal (1..13)!");
      }
    }
    // TXPWR=x
    else if (line.startsWith("TXPWR=")) {
      int val = line.substring(6).toInt();
      if (val >= 1 && val <= 80) {
        // Schicke ConfigPacket an Slave
        ConfigPacket cfg;
        cfg.type     = PACKET_TYPE_CONFIG;
        cfg.channel  = gChannel;
        cfg.txPower  = (uint8_t)val;
        cfg.paySize  = gPaySize;
        cfg.crc      = computeCRC32((uint8_t*)&cfg, sizeof(cfg) - sizeof(cfg.crc));
        if (handshakeDone) {
          esp_now_send(slaveMac, (uint8_t*)&cfg, sizeof(cfg));
          Serial.printf("[Master] Sende Config (TxPwr=%d => %.2f dBm) an Slave.\n",
                        val, 0.25f*val);
        } else {
          Serial.println("Kein Handshake erfolgt. Multi-Channel-Scan noetig?");
        }
      } else {
        Serial.println("Ungueltiger TXPWR (1..80)!");
      }
    }
    // SIZE=x
    else if (line.startsWith("SIZE=")) {
      int val = line.substring(5).toInt();
      if (val >= 1 && val <= 250) {
        // Schicke ConfigPacket an Slave
        ConfigPacket cfg;
        cfg.type     = PACKET_TYPE_CONFIG;
        cfg.channel  = gChannel;
        cfg.txPower  = gTxPower;
        cfg.paySize  = (uint16_t)val;
        cfg.crc      = computeCRC32((uint8_t*)&cfg, sizeof(cfg) - sizeof(cfg.crc));
        if (handshakeDone) {
          esp_now_send(slaveMac, (uint8_t*)&cfg, sizeof(cfg));
          Serial.printf("[Master] Sende Config (Payload=%d) an Slave.\n", val);
        } else {
          Serial.println("Kein Handshake erfolgt. Multi-Channel-Scan noetig?");
        }
      } else {
        Serial.println("Ungueltige Payload-Groesse (1..250)!");
      }
    }
    else {
      Serial.println("Unbekannter Befehl. Beispiel: CH=6, TXPWR=8, SIZE=200");
    }
  }
}

// -------------------------------------------------------
// ESP-NOW Callback
// -------------------------------------------------------

// RSSI via Promiscuous
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_MGMT || type == WIFI_PKT_DATA) {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
    lastRSSI = pkt->rx_ctrl.rssi;
  }
}

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= 0) return;
  PacketType ptype = (PacketType)data[0];

  if (ptype == PACKET_TYPE_HANDSHAKE_ACK) {
    if (len < (int)sizeof(DataPacket)) return;
    DataPacket rx;
    memcpy(&rx, data, sizeof(rx));
    uint32_t c = computeCRC32((uint8_t*)&rx, sizeof(rx) - sizeof(rx.crc));
    if (c == rx.crc) {
      Serial.println("[Master] Handshake-Ack vom Slave empfangen.");
      handshakeDone = true;
      memcpy(slaveMac, mac, 6);

      esp_now_peer_info_t pi = {};
      memcpy(pi.peer_addr, slaveMac, 6);
      pi.channel = gChannel;
      pi.encrypt = false;
      pi.ifidx   = WIFI_IF_STA;
      esp_now_del_peer(slaveMac);
      esp_now_add_peer(&pi);
    }
  }
  else if (ptype == PACKET_TYPE_TEST) {
    if (len < (int)sizeof(DataPacket)) return;
    DataPacket rx;
    memcpy(&rx, data, sizeof(rx));
    uint32_t c = computeCRC32((uint8_t*)&rx, sizeof(rx) - sizeof(rx.crc));
    if (c == rx.crc) {
      receivedPacketsThisInterval++;
      rssiSum   += lastRSSI;
      rssiCount += 1;
    }
  }
  else if (ptype == PACKET_TYPE_CONFIG_ACK) {
    if (len < (int)sizeof(ConfigAckPacket)) return;
    ConfigAckPacket ack;
    memcpy(&ack, data, sizeof(ack));
    uint32_t c = computeCRC32((uint8_t*)&ack, sizeof(ack) - sizeof(ack.crc));
    if (c == ack.crc) {
      Serial.printf("[Master] ConfigAck vom Slave! CH=%d, TXPWR=%d(%.2f dBm), SIZE=%d\n",
                    ack.channel, ack.txPower, 0.25f*ack.txPower, ack.paySize);
      // Slave hat seine Seite aktualisiert, 
      // jetzt der Master:
      gChannel = ack.channel;
      gTxPower = ack.txPower;
      gPaySize = ack.paySize;

      // Anwenden
      applyMasterSettings();
      // Peer updaten
      esp_now_peer_info_t pi = {};
      memcpy(pi.peer_addr, slaveMac, 6);
      pi.channel = gChannel;
      pi.encrypt = false;
      pi.ifidx   = WIFI_IF_STA;
      esp_now_del_peer(slaveMac);
      esp_now_add_peer(&pi);

      Serial.println("[Master] Master hat nun ebenfalls Kanal/TxPower/PayloadSize gesetzt!");
    }
  }
}

// -------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(300);
  Serial.println("=== MASTER STARTET ===");

  WiFi.mode(WIFI_STA);
  esp_wifi_start();

  applyMasterSettings();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[Master] ESP-NOW Init fehlgeschlagen.");
    return;
  }

  // RSSI
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);

  esp_now_register_recv_cb(onDataRecv);

  // Broadcast-Peer anlegen
  {
    esp_now_peer_info_t pi = {};
    memcpy(pi.peer_addr, BROADCAST_ADDR, 6);
    pi.channel = gChannel;
    pi.encrypt = false;
    pi.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&pi);
  }

  // Multi-Channel-Handshake
  multiChannelScan();

  Serial.println("[Master] Setup beendet.");
}

void loop() {
  handleSerialInput();

  if (!handshakeDone) {
    // Alle 1 Sek. Broadcast-Handshake
    static uint32_t tLastHS = 0;
    if (millis() - tLastHS > 1000) {
      sendHandshakePacket();
      tLastHS = millis();
    }
    return;
  }

  // Test-Ping-Pong
  static uint32_t lastSend = 0;
  uint32_t now = millis();
  uint32_t si = getDynamicSendInterval();
  if (now - lastSend >= si) {
    static uint32_t packetId = 1;
    DataPacket tx;
    memset(&tx, 0, sizeof(tx));
    tx.type     = PACKET_TYPE_TEST;
    tx.packetId = packetId++;

    int used = (gPaySize <= PAYLOAD_SIZE) ? gPaySize : PAYLOAD_SIZE;
    for (int i=0; i<used; i++) {
      tx.payload[i] = (uint8_t)(tx.packetId + i);
    }
    tx.crc = computeCRC32((uint8_t*)&tx, sizeof(tx) - sizeof(tx.crc));
    esp_now_send(slaveMac, (uint8_t*)&tx, sizeof(tx));
    sentPacketsThisInterval++;
    lastSend = now;
  }

  // Statistik
  if (now - lastReportTime >= REPORT_INTERVAL_MS) {
    int32_t diff = (int32_t)sentPacketsThisInterval - (int32_t)receivedPacketsThisInterval;
    if (diff < 0) diff = 0;
    float loss = 0.0f;
    if (sentPacketsThisInterval > 0) {
      loss = 100.0f * diff / (float)sentPacketsThisInterval;
    }
    float avgRSSI = (rssiCount > 0) ? (float)rssiSum / rssiCount : 0.0f;

    Serial.println("=== Verbindungstest (MASTER) ===");
    Serial.printf("CH=%d, TX=%d(%.2f dBm), SIZE=%d\n", gChannel, gTxPower, 0.25f*gTxPower, gPaySize);
    Serial.printf("Gesendet:  %lu\n", sentPacketsThisInterval);
    Serial.printf("Empfangen: %lu\n", receivedPacketsThisInterval);
    Serial.printf("Verlust:   %.2f %%\n", loss);
    Serial.printf("RSSI(avg): %.2f dBm\n", avgRSSI);

    sentPacketsThisInterval = 0;
    receivedPacketsThisInterval = 0;
    rssiSum = 0;
    rssiCount = 0;
    lastReportTime = now;
  }
}
