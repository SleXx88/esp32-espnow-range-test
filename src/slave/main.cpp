#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "crc32_util.h"
#include "data_struct.h"

#define DEFAULT_TXPWR 40
static uint8_t  gChannel = 1;
static uint8_t  gTxPower = DEFAULT_TXPWR;
static uint16_t gPaySize = PAYLOAD_MAX;

#ifndef RGB_BUILTIN
  #define RGB_BUILTIN 48
#endif
void flashLED(){
  neopixelWrite(RGB_BUILTIN,255,0,0);
  delayMicroseconds(400);
  neopixelWrite(RGB_BUILTIN,0,0,0);
}

uint8_t masterMac[6];
bool    linkOK=false;

void applyWiFi(){
  esp_wifi_set_channel(gChannel,WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(gTxPower);
}

void onRecv(const uint8_t* mac,const uint8_t* d,int len){
  if(len<=0) return;
  PacketType tp=(PacketType)d[0];

  if(tp==PACKET_TYPE_HANDSHAKE && len==sizeof(DataPacket)){
    const DataPacket* p=(const DataPacket*)d;
    if(computeCRC32((uint8_t*)p,len-sizeof(p->crc))!=p->crc) return;
    gChannel=p->payload[0];
    gTxPower=p->payload[1];
    gPaySize =p->payload[2]|(p->payload[3]<<8);
    applyWiFi();
    memcpy(masterMac,mac,6);
    linkOK=true;
    esp_now_peer_info_t pi{}; memcpy(pi.peer_addr,mac,6);
    pi.channel=gChannel; esp_now_add_peer(&pi);
    DataPacket ack{PACKET_TYPE_HANDSHAKE_ACK,0};
    memcpy(ack.payload,p->payload,4);
    ack.crc=computeCRC32((uint8_t*)&ack,sizeof(ack)-sizeof(ack.crc));
    esp_now_send(mac,(uint8_t*)&ack,sizeof(ack));
    Serial.printf("HS OK  CH=%d TX=%d SZ=%d\n",gChannel,gTxPower,gPaySize);
  }
  else if(tp==PACKET_TYPE_CONFIG && len>=sizeof(ConfigPacket)){
    const ConfigPacket* c=(const ConfigPacket*)d;
    if(computeCRC32((uint8_t*)c,len-sizeof(c->crc))!=c->crc) return;
    ConfigAckPacket ack{PACKET_TYPE_CONFIG_ACK,c->channel,c->txPower,c->paySize,0};
    ack.crc=computeCRC32((uint8_t*)&ack,sizeof(ack)-sizeof(ack.crc));
    esp_now_send(mac,(uint8_t*)&ack,sizeof(ack));
    Serial.printf("ACK rcv  CH=%d TX=%d SZ=%d\n",c->channel,c->txPower,c->paySize);
  }
  else if(tp==PACKET_TYPE_CONFIG_COMMIT && len>=sizeof(ConfigPacket)){
    // Debug: prüfen, ob Commit ankommt
    const ConfigPacket* c=(const ConfigPacket*)d;
    Serial.printf("[Slave] COMMIT rcv type=%u ch=%u tx=%u sz=%u len=%d\n",
                  c->type,c->channel,c->txPower,c->paySize,len);
    if(computeCRC32((uint8_t*)c,len-sizeof(c->crc))!=c->crc){
      Serial.println("[Slave] COMMIT CRC ERR");
      return;
    }
    gChannel=c->channel; gTxPower=c->txPower; gPaySize=c->paySize;
    applyWiFi();
    Serial.printf("[Slave] Applying new config: CH=%d TX=%d SZ=%d\n",
                  gChannel,gTxPower,gPaySize);
    esp_now_del_peer(mac);
    esp_now_peer_info_t pi{}; memcpy(pi.peer_addr,mac,6);
    pi.channel=gChannel; esp_now_add_peer(&pi);
    ConfigDonePacket done{PACKET_TYPE_CONFIG_DONE,gChannel,gTxPower,gPaySize,0};
    done.crc=computeCRC32((uint8_t*)&done,sizeof(done)-sizeof(done.crc));
    esp_err_t e=esp_now_send(mac,(uint8_t*)&done,sizeof(done));
    Serial.printf("[Slave] CONFIG_DONE send err=%d\n",e);
  }
  else if(tp==PACKET_TYPE_TEST && len==sizeof(DataPacket)){
    const DataPacket* p=(const DataPacket*)d;
    if(computeCRC32((uint8_t*)p,len-sizeof(p->crc))!=p->crc) return;
    flashLED();
    DataPacket rsp=*p;
    rsp.crc=computeCRC32((uint8_t*)&rsp,sizeof(rsp)-sizeof(rsp.crc));
    esp_now_send(mac,(uint8_t*)&rsp,sizeof(rsp));
  }
}

void setup(){
  Serial.begin(115200);
  delay(120);
  pinMode(RGB_BUILTIN,OUTPUT); neopixelWrite(RGB_BUILTIN,0,0,0);
  Serial.printf("SLAVE Boot  CH=%d TX=%d SZ=%d\n",gChannel,gTxPower,gPaySize);
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  applyWiFi();
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
}
void loop(){}
