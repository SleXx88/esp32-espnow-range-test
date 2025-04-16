#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ============================================================================
//                           K O N F I G   (MASTER)
// ============================================================================

// Serielle Schnittstelle (Baudrate)
#define SERIAL_BAUDRATE 115200

// Funk-Kanal (üblich: 1 bis 13; USA: 1–11)
#define CHANNEL 1

// Sendeleistung in 0.25 dBm-Schritten
// Beispiele:
//   4  => 1 dBm
//   8  => 2 dBm
//   60 => 15 dBm (typisch Maximum für viele ESP32-S3-Module)
//   80 => 20 dBm (ggf. nicht von allen Modulen unterstützt)
#define SEND_POWER 4

// Statistik-Intervall (ms) für Paketverlust / RSSI-Ausgabe
#define REPORT_INTERVAL_MS 1000

// ============================================================================
//                           E N D E   K O N F I G
// ============================================================================

#include "crc32_util.h"  // Oder direkt computeCRC32(...) hier reinkopieren
#include "data_struct.h" // Hier ist PacketType, DataPacket etc. definiert

static const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t slaveMac[6]; // Nach Handshake hier speichern
bool handshakeDone = false;

// Statistik
uint32_t sentPacketsThisInterval = 0;
uint32_t receivedPacketsThisInterval = 0;
uint32_t lastReportTime = 0;

// RSSI-Erfassung
int8_t lastRSSI = 0;
int32_t rssiSum = 0;
uint32_t rssiCount = 0;

static uint32_t getDynamicSendInterval()
{
  uint32_t interval = PAYLOAD_SIZE / 10;
  if (interval < 10)
    interval = 10;
  return interval;
}

static uint32_t getDynamicWaitTime()
{
  // Nur nötig, wenn Sie sie aufrufen
  return 3 * getDynamicSendInterval();
}

// Callback für Promiscuous-Mode
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
  if (type == WIFI_PKT_MGMT || type == WIFI_PKT_DATA)
  {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    lastRSSI = pkt->rx_ctrl.rssi;
  }
}

// Callback: Daten empfangen
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len < (int)sizeof(DataPacket))
  {
    // Unerwartete Paketgröße
    return;
  }

  DataPacket rx;
  memcpy(&rx, incomingData, sizeof(rx));

  // CRC prüfen
  uint32_t calcCrc = computeCRC32((uint8_t *)&rx, sizeof(rx) - sizeof(rx.crc));
  if (calcCrc != rx.crc)
  {
    // Paket korrupt
    Serial.println("CRC-Fehler im empfangenen Paket.");
    return;
  }

  switch (rx.type)
  {
  case PACKET_TYPE_HANDSHAKE_ACK:
    // Slave hat geantwortet, MAC übernehmen
    Serial.println("Handshake ACK empfangen!");
    // MAC aus dem Callback entnehmen
    memcpy(slaveMac, mac, 6);
    handshakeDone = true;

    // Peer mit Slave-MAC hinzufügen (Unicast)
    {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, slaveMac, 6);
      peerInfo.channel = CHANNEL;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;

      esp_now_del_peer(slaveMac); // Vorsichtshalber löschen, falls existiert
      esp_now_add_peer(&peerInfo);

      Serial.printf("Slave-MAC gesetzt: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    slaveMac[0], slaveMac[1], slaveMac[2],
                    slaveMac[3], slaveMac[4], slaveMac[5]);
    }
    break;

  case PACKET_TYPE_TEST:
    // Dies ist die Antwort vom Slave im normalen Test-Betrieb
    receivedPacketsThisInterval++;
    rssiSum += lastRSSI;
    rssiCount += 1;
    break;

  default:
    // Andere Packet-Typen hier verarbeiten
    break;
  }
}

// Setup
void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  delay(500);

  // WLAN + ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_start();

  // Kanal und Sendeleistung
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(SEND_POWER * 4);

  // RSSI via Promiscuous
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init fehlgeschlagen.");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  // Broadcast-Peer hinzufügen (für Handshake)
  {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDR, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
  }

  Serial.println("Master bereit. Versuche Handshake per Broadcast...");
}

void loop()
{
  uint32_t now = millis();

  // --- Handshake-Phase ---
  if (!handshakeDone)
  {
    // Regelmäßig ein Handshake-Paket broadcasten, bis Slave antwortet
    static uint32_t lastHandshakeSend = 0;
    if (now - lastHandshakeSend > 500)
    { // alle 500ms
      DataPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      pkt.type = PACKET_TYPE_HANDSHAKE;
      pkt.packetId = 0; // kann beliebig sein
      // Payload füllen (optional)
      for (int i = 0; i < PAYLOAD_SIZE; i++)
      {
        pkt.payload[i] = 0xAA; // Testdaten
      }
      pkt.crc = computeCRC32((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

      esp_err_t err = esp_now_send(BROADCAST_ADDR, (uint8_t *)&pkt, sizeof(pkt));
      if (err == ESP_OK)
      {
        Serial.println("Handshake-Paket gesendet (Broadcast)...");
      }
      else
      {
        Serial.printf("Fehler beim Senden des Handshake-Pakets: %d\n", err);
      }
      lastHandshakeSend = now;
    }

    // Solange Handshake nicht fertig, kein Testbetrieb
    return;
  }

  // --- Ab hier: Test-Betrieb (Ping-Pong mit dem Slave) ---

  static uint32_t lastSend = 0;
  uint32_t sendInterval = getDynamicSendInterval();
  if (now - lastSend >= sendInterval)
  {
    // Neues Test-Paket an Slave senden
    static uint32_t packetCounter = 1;

    DataPacket pkt;
    pkt.type = PACKET_TYPE_TEST;
    pkt.packetId = packetCounter++;
    // Beispiel: Payload mit irgendwelchen Daten füllen
    for (int i = 0; i < PAYLOAD_SIZE; i++)
    {
      pkt.payload[i] = (uint8_t)(pkt.packetId + i);
    }
    pkt.crc = computeCRC32((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

    esp_err_t err = esp_now_send(slaveMac, (uint8_t *)&pkt, sizeof(pkt));
    if (err == ESP_OK)
    {
      sentPacketsThisInterval++;
    }
    else
    {
      Serial.printf("Fehler beim Senden des Testpakets: %d\n", err);
    }

    lastSend = now;
  }

  // Auswertung in festen Intervallen
  if (now - lastReportTime >= REPORT_INTERVAL_MS)
  {
    // Differenz als signed int bilden
    int32_t diff = (int32_t)sentPacketsThisInterval - (int32_t)receivedPacketsThisInterval;
    if (diff < 0)
    {
      // Falls mehr "Empfangen" als "Gesendet"
      // (oder Zähler aus dem Takt geraten), auf 0 begrenzen.
      diff = 0;
    }

    float loss = 0.0f;
    if (sentPacketsThisInterval > 0)
    {
      loss = 100.0f * diff / (float)sentPacketsThisInterval;
    }

    float avgRSSI = (rssiCount > 0) ? (float)rssiSum / rssiCount : 0.0f;

    Serial.println("=== Verbindungstest ===");
    Serial.printf("Gesendet:    %lu\n", sentPacketsThisInterval);
    Serial.printf("Empfangen:   %lu\n", receivedPacketsThisInterval);
    Serial.printf("Verlust:     %.2f %%\n", loss);
    Serial.printf("Durchschn. RSSI: %.2f dBm\n", avgRSSI);

    // Reset der Zähler
    sentPacketsThisInterval = 0;
    receivedPacketsThisInterval = 0;
    rssiSum = 0;
    rssiCount = 0;
    lastReportTime = now;
  }
}
