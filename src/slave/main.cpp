#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ---------- Konfiguration -------------
#define CHANNEL 1
#define SEND_POWER 8
// --------------------------------------

#include "crc32_util.h"
#include "data_struct.h"

static uint8_t masterMac[6]; // Nach Handshake bekannt
bool handshakeDone = false;

// Callback: Daten empfangen
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len < (int)sizeof(DataPacket))
  {
    return;
  }

  DataPacket rx;
  memcpy(&rx, incomingData, sizeof(rx));

  // CRC prüfen
  uint32_t calcCrc = computeCRC32((uint8_t *)&rx, sizeof(rx) - sizeof(rx.crc));
  if (calcCrc != rx.crc)
  {
    Serial.println("CRC-Fehler im empfangenen Paket (Slave).");
    return;
  }

  switch (rx.type)
  {
  case PACKET_TYPE_HANDSHAKE:
  {
    Serial.println("Handshake-Paket vom Master empfangen.");

    // Master-MAC speichern
    memcpy(masterMac, mac, 6);
    handshakeDone = true;

    // Peer anlegen (für Unicast zurück)
    if (!esp_now_is_peer_exist(mac))
    {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, mac, 6);
      peerInfo.channel = CHANNEL;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;
      esp_now_add_peer(&peerInfo);
    }

    // Zurückschicken: HANDSHAKE_ACK
    DataPacket ack;
    memset(&ack, 0, sizeof(ack));
    ack.type = PACKET_TYPE_HANDSHAKE_ACK;
    ack.packetId = 0;
    for (int i = 0; i < PAYLOAD_SIZE; i++)
    {
      ack.payload[i] = 0x55; // Test-Daten
    }
    ack.crc = computeCRC32((uint8_t *)&ack, sizeof(ack) - sizeof(ack.crc));

    esp_now_send(mac, (uint8_t *)&ack, sizeof(ack));
    Serial.println("Handshake-ACK an Master gesendet.");
  }
  break;

  case PACKET_TYPE_TEST:
  {
    // Normales Test-Paket. Einfach zurücksenden (Ping-Pong).
    if (!esp_now_is_peer_exist(mac))
    {
      // Falls Peer noch nicht existiert, hinzufügen
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, mac, 6);
      peerInfo.channel = CHANNEL;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;
      esp_now_add_peer(&peerInfo);
    }

    // Hier könnte man den Payload untersuchen oder CRC nochmal checken
    // => Ist bereits oben geschehen.

    // Antwort mit demselben Inhalt
    DataPacket tx = rx;
    // Wir könnten hier tx.packetId oder payload abändern, um anzuzeigen,
    // dass es vom Slave kommt; typischerweise lassen wir es gleich.

    tx.crc = computeCRC32((uint8_t *)&tx, sizeof(tx) - sizeof(tx.crc));
    esp_now_send(mac, (uint8_t *)&tx, sizeof(tx));
  }
  break;

  default:
    // Andere Paket-Typen ignorieren oder verarbeiten
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  // WLAN + ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_start();

  // Kanal + Sendeleistung
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(SEND_POWER * 4);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init fehlgeschlagen (Slave).");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Slave bereit. Warte auf Broadcast-Handshakes...");
}

void loop()
{
  // Hier kann der Slave ggf. weitere Tasks ausführen.
  // Der Empfang und das Antworten auf Pakete passieren im Callback.
}
