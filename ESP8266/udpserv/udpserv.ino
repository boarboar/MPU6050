#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "cmd.h"

void print_sys_info();
void doCycle(uint16_t);

const char* ssid = "NETGEAR";
const char* password = "boarboar";
const int udp_port = 4444;
const int BUF_SZ = 255;
const int CYCLE_TO = 10;
bool isConnected=false;
char packetBuffer[BUF_SZ];
WiFiUDP udp_rcv;
WiFiUDP udp_snd;
uint32_t last_cycle;

uint32_t cnt[4]={0,0,0,0};

CmdProc cmd;

void setup() {
  delay(2000);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }

  if(udp_rcv.begin(udp_port)) {
    isConnected=true;
    Serial.print("Ready! Listening on ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(udp_port);
  } else {
    Serial.println("Failed to init UDP socket!");
  }  
  print_sys_info();

  last_cycle=millis();
}

void loop() {
  if(isConnected) {
    uint32_t m1=millis();
    int packetSize = udp_rcv.parsePacket(); 
    if (packetSize) {
      int len = udp_rcv.read(packetBuffer, BUF_SZ);
      if(len>BUF_SZ-1) len=BUF_SZ-1;
      packetBuffer[len] = 0;    
      // echo
      udp_snd.beginPacket(udp_rcv.remoteIP(), udp_rcv.remotePort());
      udp_snd.write(packetBuffer, len);
      udp_snd.endPacket();

      // dbg out
      uint32_t m2=millis();
      Serial.print("From: "); Serial.print(udp_rcv.remoteIP());
      Serial.print(":"); Serial.print(udp_rcv.remotePort());
      Serial.print(" len: "); Serial.print(len);
      Serial.print(" T: "); Serial.print(m2-m1);
      Serial.print(" Val: "); Serial.println(packetBuffer);
      //

      cmd.doCmd(packetBuffer);

      print_sys_info();
    }
  }
  uint32_t t = millis();
  if(t-last_cycle >= CYCLE_TO) {
    doCycle(t-last_cycle);
    last_cycle = t;
  }
  //delay(10);
}

void doCycle(uint16_t dt) {
  if(dt<=CYCLE_TO) cnt[0]++;
  else if(dt<=CYCLE_TO<<1) cnt[1]++;
  else if(dt<=CYCLE_TO<<2) cnt[2]++;
  else cnt[3]++;
}
void print_sys_info() {
  Serial.print("Heap sz: "); Serial.println(ESP.getFreeHeap());
  Serial.print("TO<="); Serial.print(CYCLE_TO); Serial.print(": "); Serial.print(cnt[0]);
  Serial.print(" TO<="); Serial.print(CYCLE_TO<<1); Serial.print(": "); Serial.print(cnt[1]);
  Serial.print(" TO<="); Serial.print(CYCLE_TO<<2); Serial.print(": "); Serial.print(cnt[2]);
  Serial.print(" TO>"); Serial.print(CYCLE_TO<<2); Serial.print(": "); Serial.println(cnt[3]);
}
