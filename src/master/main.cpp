#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

#define SERIAL_BAUD     115200
#define DEFAULT_CH      1
#define DEFAULT_TXPWR   40
#define ACK_TIMEOUT_MS  400

static uint8_t  gCh     = DEFAULT_CH;
static uint8_t  gTx     = DEFAULT_TXPWR;
static uint16_t gSz     = PAYLOAD_MAX;
static uint32_t gTotal  = 1000;

enum ConfigState { IDLE, WAIT_ACK, WAIT_COMMIT };
static ConfigState cfgState = IDLE;
static ConfigPacket pendingCfg = {0,0,0,0,0};
static uint8_t slaveMac[6];
static bool    linkOK = false;
static uint32_t lastCfgMs = 0;

static bool    testRun = false;
static uint32_t txCnt=0, rxCnt=0, lastTxMs=0;
static bool    waitingTestAck = false;
static int32_t rssiSum=0; static uint32_t rssiCnt=0;
static int8_t  lastRSSI=0;

#ifndef RGB_BUILTIN
  #define RGB_BUILTIN 48
#endif

void ledOff() { neopixelWrite(RGB_BUILTIN,0,0,0); }
void ledBlink() {
  static uint32_t t=0; static bool s=false;
  if (millis()-t > 400) {
    s = !s; t = millis();
    neopixelWrite(RGB_BUILTIN, s?255:0,0,0);
  }
}

void applyWiFi(uint8_t ch, uint8_t tx) {
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(tx);
}

void printHelp() {
  Serial.println("-help -scan -run/-stop -ch N -txpwr N -pkgsize N -sendpkg N -reset");
}
void printCfg() {
  Serial.printf("CH %u  TX %u  SIZE %u  PKTS %u  Slave:%s\n",
    gCh, gTx, gSz, gTotal, linkOK?"OK":"--");
}

void sendHandshake() {
  DataPacket p{PACKET_TYPE_HANDSHAKE,0};
  p.payload[0]=gCh; p.payload[1]=gTx;
  p.payload[2]=gSz&0xFF; p.payload[3]=gSz>>8;
  p.crc=computeCRC32((uint8_t*)&p,sizeof(p)-sizeof(p.crc));
  esp_now_send((uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF",(uint8_t*)&p,sizeof(p));
}

void doTest() {
  if(!testRun||!linkOK||waitingTestAck||txCnt>=gTotal) return;
  DataPacket p{PACKET_TYPE_TEST,txCnt+1};
  for(int i=0;i<gSz;i++) p.payload[i]=p.id+i;
  p.crc=computeCRC32((uint8_t*)&p,sizeof(p)-sizeof(p.crc));
  if(esp_now_send(slaveMac,(uint8_t*)&p,sizeof(p))==ESP_OK){
    waitingTestAck=true;
    lastTxMs=millis();
    txCnt++;
  }
}

esp_err_t sendConfigRet(PacketType type) {
  ConfigPacket pkt=pendingCfg;
  pkt.type=(uint8_t)type;
  pkt.crc =computeCRC32((uint8_t*)&pkt,sizeof(pkt)-sizeof(pkt.crc));
  esp_err_t e=esp_now_send(slaveMac,(uint8_t*)&pkt,sizeof(pkt));
  Serial.printf("[Master] sendConfig type=%u err=%d\n", type, e);
  return e;
}

void promiscCB(void*buf,wifi_promiscuous_pkt_type_t t){
  if(t==WIFI_PKT_DATA||t==WIFI_PKT_MGMT)
    lastRSSI = ((wifi_promiscuous_pkt_t*)buf)->rx_ctrl.rssi;
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("[Master] onSend to %02X:%02X:%02X... status=%u\n",
    mac_addr[0], mac_addr[1], mac_addr[2], status);
}

void recvCB(const uint8_t* mac,const uint8_t* data,int len){
  if(len<=0) return;
  PacketType tp=(PacketType)data[0];

  if(tp==PACKET_TYPE_HANDSHAKE_ACK){
    memcpy(slaveMac,mac,6); linkOK=true;
    const DataPacket* p=(const DataPacket*)data;
    Serial.printf("HS_ACK  CH %u TX %u SIZE %u\n",
      p->payload[0],p->payload[1],p->payload[2]|(p->payload[3]<<8));
    esp_now_peer_info_t pi{}; memcpy(pi.peer_addr,mac,6);
    pi.channel=gCh; esp_now_add_peer(&pi);
  }
  else if(tp==PACKET_TYPE_TEST){
    rxCnt++; rssiSum+=lastRSSI; rssiCnt++;
    waitingTestAck=false;
  }
  else if(tp==PACKET_TYPE_CONFIG_ACK && cfgState==WAIT_ACK){
    Serial.println("-> ACK rcved, sending COMMIT");
    sendConfigRet(PACKET_TYPE_CONFIG_COMMIT);
    lastCfgMs=millis();
    cfgState=WAIT_COMMIT;
    // ** jetzt erst umstellen **
    delay(100);
    gCh=pendingCfg.channel; gTx=pendingCfg.txPower; gSz=pendingCfg.paySize;
    applyWiFi(gCh,gTx);
    esp_now_del_peer(slaveMac);
    { esp_now_peer_info_t pi{}; memcpy(pi.peer_addr,slaveMac,6);
      pi.channel=gCh; esp_now_add_peer(&pi); }
    Serial.printf("-> Master switched to CH %u\n", gCh);
  }
  else if(tp==PACKET_TYPE_CONFIG_DONE && cfgState==WAIT_COMMIT){
    Serial.println("-> DONE rcved, sync OK");
    cfgState=IDLE;
    printCfg();
  }
}

bool scanSlave(){
  linkOK=false;
  for(uint8_t ch=1;ch<=13&&!linkOK;ch++){
    applyWiFi(ch,gTx);
    uint32_t t0=millis();
    while(!linkOK&&millis()-t0<800){
      sendHandshake(); delay(200);
    }
  }
  if(!linkOK) applyWiFi(DEFAULT_CH,gTx);
  return linkOK;
}

void execCmd(String s){
  s.toLowerCase();
  if(s=="-help"){ printHelp(); printCfg(); }
  else if(s=="-scan"){ scanSlave(); printCfg(); }
  else if(s=="-run"){
    if(cfgState!=IDLE){
      Serial.println("Config pending – bitte warten");return;
    }
    if(!linkOK&&!scanSlave()){ Serial.println("no slave");return; }
    testRun=true; txCnt=rxCnt=rssiSum=rssiCnt=0; waitingTestAck=false;
    Serial.println("START");
  }
  else if(s=="-stop") testRun=false;
  else if(s=="-reset") ESP.restart();
  else if(s.startsWith("-ch ")){
    int v=s.substring(4).toInt();
    if(v>=1&&v<=13){
      pendingCfg.channel=v; pendingCfg.txPower=gTx;
      pendingCfg.paySize=gSz; cfgState=WAIT_ACK;
      Serial.printf("-> CFG send CH %u\n",v);
      sendConfigRet(PACKET_TYPE_CONFIG);
      lastCfgMs=millis();
    }
  }
  else if(s.startsWith("-txpwr ")){
    int v=s.substring(7).toInt();
    if(v>=0&&v<=80){
      pendingCfg.channel=gCh; pendingCfg.txPower=v;
      pendingCfg.paySize=gSz; cfgState=WAIT_ACK;
      Serial.printf("-> CFG send TX %u\n",v);
      sendConfigRet(PACKET_TYPE_CONFIG);
      lastCfgMs=millis();
    }
  }
  else if(s.startsWith("-pkgsize ")){
    int v=s.substring(9).toInt();
    if(v>=1&&v<=PAYLOAD_MAX){
      pendingCfg.channel=gCh; pendingCfg.txPower=gTx;
      pendingCfg.paySize=v; cfgState=WAIT_ACK;
      Serial.printf("-> CFG send SZ %u\n",v);
      sendConfigRet(PACKET_TYPE_CONFIG);
      lastCfgMs=millis();
    }
  }
  else if(s.startsWith("-sendpkg ")){
    int v=s.substring(9).toInt();
    if(v>=1&&v<=99999){ gTotal=v; printCfg(); }
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD); delay(100);
  pinMode(RGB_BUILTIN,OUTPUT); ledOff();
  printHelp(); printCfg();

  WiFi.mode(WIFI_STA); esp_wifi_start();
  applyWiFi(gCh,gTx);

  esp_now_init();
  esp_now_register_send_cb(onSend);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promiscCB);
  esp_now_register_recv_cb(recvCB);

  // Broadcast‑Peer
  esp_now_peer_info_t bc{};
  memcpy(bc.peer_addr,"\xFF\xFF\xFF\xFF\xFF\xFF",6);
  bc.channel=0; esp_now_add_peer(&bc);

  scanSlave();
}

void loop(){
  if(Serial.available()){
    String l=Serial.readStringUntil('\n'); l.trim();
    if(l.length()) execCmd(l);
  }
  if(cfgState==WAIT_ACK && millis()-lastCfgMs>ACK_TIMEOUT_MS){
    sendConfigRet(PACKET_TYPE_CONFIG);
    lastCfgMs=millis();
  }
  if(cfgState==WAIT_COMMIT && millis()-lastCfgMs>ACK_TIMEOUT_MS){
    sendConfigRet(PACKET_TYPE_CONFIG_COMMIT);
    lastCfgMs=millis();
  }
  if(testRun){
    ledBlink(); doTest();
    if(waitingTestAck && millis()-lastTxMs>ACK_TIMEOUT_MS) waitingTestAck=false;
    if(txCnt==gTotal && !waitingTestAck){
      float loss=100.0f*(txCnt-rxCnt)/txCnt;
      float avg=(rssiCnt?(float)rssiSum/rssiCnt:0);
      Serial.printf("\nResult Tx %u Rx %u Loss %.2f%% ØRSSI %.1f dBm\n",
                    txCnt,rxCnt,loss,avg);
      testRun=false; ledOff();
    }
  } else ledOff();
}
