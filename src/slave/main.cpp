#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL 1
#define SEND_POWER 8
#define RGB_BRIGHTNESS 64

uint32_t packetCounter = 0;

typedef struct __attribute__((packed)) {
  uint32_t packetId;
} DataPacket;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(DataPacket)) return;

  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Grün
  delay(100);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);  // Aus

  DataPacket rx;
  memcpy(&rx, incomingData, sizeof(rx));

  Serial.print("Empfangen von ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.printf(" | Paket #%lu\n", rx.packetId);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peerInfo);  // kein Problem, wenn schon hinzugefügt

  DataPacket response = { packetCounter++ };
  esp_err_t result = esp_now_send(mac, (uint8_t *)&response, sizeof(response));

  if (result == ESP_OK) {
    Serial.printf("Antwort #%lu gesendet\n", response.packetId);
  } else {
    Serial.printf("Antwort senden fehlgeschlagen: %d\n", result);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(RGB_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);

  WiFi.mode(WIFI_STA);
  esp_wifi_start();

  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(SEND_POWER * 4);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init fehlgeschlagen");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Slave bereit. Warte auf Broadcast...");
}

void loop() {
  delay(5);
}
