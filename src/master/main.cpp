#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

#define SERIAL_BAUD       115200
#define DEFAULT_CH        1
#define DEFAULT_TXPWR    40    // 10 dBm
#define ACK_TIMEOUT_MS  400

// ── State für Config-Wechsel ───────────────────────────
enum ConfigState { IDLE, WAIT_ACK, WAIT_COMMIT, COMMIT_SENT };
static ConfigState cfgState = IDLE;
static ConfigPacket pendingCfg;
static uint32_t    lastCfgMs = 0;

// ── Globale Settings ────────────────────────────────────
static uint8_t  gCh    = DEFAULT_CH;
static uint8_t  gTx    = DEFAULT_TXPWR;
static uint16_t gSz    = PAYLOAD_MAX;
static uint32_t gTotal = 1000;

// ── Laufzeit-Zähler ─────────────────────────────────────
static bool     linkOK     = false;
static bool     testRun    = false;
static uint32_t txCnt      = 0, rxCnt = 0;
static bool     waitingAck = false;
static int32_t  rssiSum    = 0;
static uint32_t rssiCnt    = 0;
static int8_t   lastRSSI   = 0;

// ── Peer-Addressen ──────────────────────────────────────
uint8_t slaveMac[6];
const uint8_t BC_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── Neopixel (on-board RGB) ─────────────────────────────
#ifndef RGB_BUILTIN
  #define RGB_BUILTIN 48
#endif
void ledOff() { neopixelWrite(RGB_BUILTIN, 0, 0, 0); }
void ledRedBlink(uint32_t on, uint32_t off) {
  static uint32_t t; static bool s;
  if (millis() - t > (s ? on : off)) {
    s = !s; t = millis();
    if (s) neopixelWrite(RGB_BUILTIN,255,0,0);
    else   ledOff();
  }
}
void ledGreenOnce() {
  neopixelWrite(RGB_BUILTIN,0,255,0);
  delay(150);
  ledOff();
}
void ledGreenTwice() {
  for(int i=0;i<2;i++){
    neopixelWrite(RGB_BUILTIN,0,255,0);
    delay(150);
    ledOff();
    delay(100);
  }
}

// ── Wi‑Fi Helper ────────────────────────────────────────
void applyWiFi(uint8_t ch, uint8_t tx) {
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(tx);
}

// ── Konsole ──────────────────────────────────────────────
void printLine() {
  Serial.println("----------------------------------------");
}
void printHelp() {
  printLine();
  Serial.println(" Befehle:");
  Serial.println("   -help          Hilfe anzeigen");
  Serial.println("   -scan          Slave suchen");
  Serial.println("   -run / -stop   Test starten / stoppen");
  Serial.println("   -ch N          Kanal 1–13");
  Serial.println("   -txpwr N       Tx‑Power 0–80 (¼ dBm)");
  Serial.println("   -pkgsize N     Nutzlast 1–241 B");
  Serial.println("   -sendpkg N     Pakete 1–99999");
  Serial.println("   -reset         Neustart");
  printLine();
}
void printCfg() {
  Serial.printf("  Kanal:      %2u\n", gCh);
  Serial.printf("  Tx‑Power:   %2u (%.2f dBm)\n", gTx, 0.25f*gTx);
  Serial.printf("  Nutzlast:   %3u B (+%u OH = %u B)\n", gSz, DATA_OVERHEAD, gSz+DATA_OVERHEAD);
  Serial.printf("  Pakete:     %u\n", gTotal);
  Serial.printf("  Slave:      %s\n", linkOK?"verbunden":"nicht verbunden");
  printLine();
}

// ── ESP‑NOW Sends ─────────────────────────────────────────
void sendHandshake() {
  applyWiFi(gCh, gTx);
  DataPacket p{PACKET_TYPE_HANDSHAKE, 0};
  p.payload[0] = gCh;
  p.payload[1] = gTx;
  p.payload[2] = gSz & 0xFF;
  p.payload[3] = gSz >> 8;
  p.crc = computeCRC32((uint8_t*)&p,sizeof(p)-sizeof(p.crc));
  //Serial.printf("[Master] HS send on CH %u\n", gCh);
  Serial.printf(".");
  esp_now_send((uint8_t*)BC_ADDR,(uint8_t*)&p,sizeof(p));
}

void sendConfig(PacketType type) {
  applyWiFi(gCh, gTx);
  ConfigPacket pkt = pendingCfg;
  pkt.type = (uint8_t)type;
  pkt.crc  = computeCRC32((uint8_t*)&pkt,sizeof(pkt)-sizeof(pkt.crc));
  Serial.printf("[Master] sendConfig type=%u on CH %u\n", (uint8_t)type, gCh);
  esp_now_send(slaveMac,(uint8_t*)&pkt,sizeof(pkt));
}

void doTest() {
  if (!testRun || !linkOK || waitingAck || txCnt>=gTotal) return;
  DataPacket p{PACKET_TYPE_TEST, txCnt+1};
  for(int i=0;i<gSz;i++) p.payload[i]=(uint8_t)(p.id+i);
  p.crc = computeCRC32((uint8_t*)&p,sizeof(p)-sizeof(p.crc));
  applyWiFi(gCh, gTx);
  esp_now_send(slaveMac,(uint8_t*)&p,sizeof(p));
  waitingAck = true;
  txCnt++;
}

// ── Callbacks ────────────────────────────────────────────
void promiscCB(void*buf, wifi_promiscuous_pkt_type_t t) {
  if (t==WIFI_PKT_DATA||t==WIFI_PKT_MGMT)
    lastRSSI = ((wifi_promiscuous_pkt_t*)buf)->rx_ctrl.rssi;
}

void onSend(const uint8_t* mac, esp_now_send_status_t status) {
  if (cfgState==WAIT_COMMIT && status==ESP_OK) {
    cfgState = COMMIT_SENT;
    Serial.println("-> COMMIT bestä­ti­gung, wechs­le…");
    gCh = pendingCfg.channel;
    gTx = pendingCfg.txPower;
    gSz = pendingCfg.paySize;
    applyWiFi(gCh, gTx);

    esp_now_del_peer(slaveMac);
    esp_now_peer_info_t pi{};
    memcpy(pi.peer_addr,slaveMac,6);
    pi.channel = gCh;
    pi.encrypt = false;
    pi.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&pi);

    Serial.printf("-> Master switched to CH %u\n", gCh);
    ledGreenTwice();
  }
}

void recvCB(const uint8_t* mac, const uint8_t* data, int len) {
  if (len<=0) return;
  PacketType tp = (PacketType)data[0];

  if (tp==PACKET_TYPE_HANDSHAKE_ACK) {
    const DataPacket* p = (const DataPacket*)data;
    gCh = p->payload[0];
    applyWiFi(gCh, gTx);
    memcpy(slaveMac,mac,6);
    linkOK = true;
    esp_now_peer_info_t pi{};
    memcpy(pi.peer_addr,mac,6);
    pi.channel = gCh;
    pi.encrypt = false;
    pi.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&pi);
    Serial.printf("\n");
    printLine();
    Serial.printf("[Slave] HS_ACK  CH=%u  TX=%u  SIZE=%u\n",
                  p->payload[0], p->payload[1],
                  p->payload[2]|(p->payload[3]<<8));
    // Statusanzeige *nur hier*:
    printCfg();
    ledGreenOnce();
  }
  else if (tp==PACKET_TYPE_TEST) {
    rxCnt++;
    rssiSum += lastRSSI;
    rssiCnt++;
    waitingAck = false;
  }
  else if (tp==PACKET_TYPE_CONFIG_ACK && cfgState==WAIT_ACK) {
    Serial.println("-> CONFIG_ACK empfangen, sende COMMIT");
    sendConfig(PACKET_TYPE_CONFIG_COMMIT);
    cfgState   = WAIT_COMMIT;
    lastCfgMs  = millis();
  }
  else if (tp==PACKET_TYPE_CONFIG_DONE && cfgState==COMMIT_SENT) {
    Serial.println("-> CONFIG_DONE empfangen, sync OK");
    cfgState = IDLE;
    printCfg();
    ledGreenTwice();
  }
}

// ── Scan über alle Kanäle ────────────────────────────────
bool scanSlave() {
  linkOK = false;
  uint8_t prevCh = gCh;
  for (uint8_t ch=1; ch<=13 && !linkOK; ++ch) {
    Serial.printf("\n[Master] Scanne Kanal %u ", ch);
    gCh = ch;
    applyWiFi(gCh, gTx);
    uint32_t t0 = millis();
    while (!linkOK && millis()-t0<800) {
      sendHandshake();
      delay(200);
    }
  }
  if (!linkOK) {
    gCh = prevCh;
    applyWiFi(gCh, gTx);
  }
  return linkOK;
}

// ── Kommando‑Parser ─────────────────────────────────────
void execCmd(const String& s) {
  String cmd = s; cmd.trim(); cmd.toLowerCase();
  if (cmd=="-help")        { printHelp(); printCfg(); }
  else if (cmd=="-scan")    { scanSlave(); /* kein double-printCfg hier */ }
  else if (cmd=="-run")     {
    if (!linkOK && !scanSlave()) { Serial.println("Kein Slave."); return; }
    testRun = true; txCnt=rxCnt=rssiSum=rssiCnt=0; waitingAck=false;
    Serial.println("→ Test gestartet");
  }
  else if (cmd=="-stop")    { testRun=false; Serial.println("→ Test gestoppt"); ledOff(); }
  else if (cmd=="-reset")   { ESP.restart(); }
  else if (cmd.startsWith("-ch ")) {
    int v = cmd.substring(4).toInt();
    if (v>=1&&v<=13) {
      pendingCfg.channel = v;
      pendingCfg.txPower = gTx;
      pendingCfg.paySize = gSz;
      cfgState = WAIT_ACK;
      Serial.printf("→ CFG send CH %u on CH %u\n", v, gCh);
      sendConfig(PACKET_TYPE_CONFIG);
      lastCfgMs = millis();
    }
  }
  else if (cmd.startsWith("-txpwr ")) {
    int v = cmd.substring(7).toInt();
    if (v>=0&&v<=80) {
      pendingCfg.channel = gCh;
      pendingCfg.txPower = v;
      pendingCfg.paySize = gSz;
      cfgState = WAIT_ACK;
      Serial.printf("→ CFG send TX %u on CH %u\n", v, gCh);
      sendConfig(PACKET_TYPE_CONFIG);
      lastCfgMs = millis();
    }
  }
  else if (cmd.startsWith("-pkgsize ")) {
    int v = cmd.substring(9).toInt();
    if (v>=1&&v<=PAYLOAD_MAX) {
      pendingCfg.channel = gCh;
      pendingCfg.txPower = gTx;
      pendingCfg.paySize = v;
      cfgState = WAIT_ACK;
      Serial.printf("→ CFG send SZ %u on CH %u\n", v, gCh);
      sendConfig(PACKET_TYPE_CONFIG);
      lastCfgMs = millis();
    }
  }
  else if (cmd.startsWith("-sendpkg ")) {
    int v = cmd.substring(9).toInt();
    if (v>=1&&v<=99999) { gTotal=v; Serial.printf("→ Pakete auf %u gesetzt\n",gTotal); }
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD); delay(100);
  pinMode(RGB_BUILTIN, OUTPUT); ledOff();
  printHelp();
  printCfg();              // initial, vor dem ersten Scan

  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  applyWiFi(gCh,gTx);

  esp_now_init();
  esp_now_register_send_cb(onSend);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promiscCB);
  esp_now_register_recv_cb(recvCB);

  // Broadcast‑Peer
  esp_now_peer_info_t bc{};
  memcpy(bc.peer_addr, BC_ADDR, 6);
  bc.channel = 0;
  bc.encrypt = false;
  bc.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&bc);

  Serial.println("→ Suche Slave auf allen Kanälen…");
  scanSlave();
  // **kein** zweites printCfg() hier!
}

void loop(){
  if (Serial.available()) {
    String l = Serial.readStringUntil('\n');
    if (l.length()) execCmd(l);
  }
  // Config‑Retry
  if (cfgState==WAIT_ACK && millis()-lastCfgMs>ACK_TIMEOUT_MS) {
    Serial.println("[Master] ACK‑Timeout, retry CONFIG");
    sendConfig(PACKET_TYPE_CONFIG);
    lastCfgMs = millis();
  }
  // Test‑Loop
  if (testRun) {
    ledRedBlink(400,400);
    doTest();
    if (txCnt==gTotal && !waitingAck) {
      float loss = 100.0f*(txCnt - rxCnt)/txCnt;
      float avg  = rssiCnt? float(rssiSum)/rssiCnt : 0;
      printLine();
      Serial.printf(" Result: Tx %u Rx %u Loss %.2f%% ØRSSI %.1f dBm\n",
                    txCnt,rxCnt,loss,avg);
      printLine();
      testRun=false;
      ledOff();
    }
  } else {
    ledOff();
  }
}
