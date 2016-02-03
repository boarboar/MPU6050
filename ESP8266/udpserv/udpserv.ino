#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "cmd.h"
#include "stat.h"

void print_sys_info();
void doCycle();

const char* ssid = "NETGEAR";
const char* password = "boarboar";
const int udp_port = 4444;
//const int BUF_SZ = 255;
const int CYCLE_TO = 2;
//bool isConnected=false;
//char packetBuffer[BUF_SZ];
//WiFiUDP udp_rcv;
//WiFiUDP udp_snd;
uint32_t last_cycle;

//CmdProc cmd;

void setup() {
  delay(2000);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to "); Serial.println(ssid);
    delay(10000);
    ESP.reset();
  }

  if(/*udp_rcv.begin(udp_port)*/CmdProc::Cmd.init(udp_port)) {
    //isConnected=true;
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
  if(/*isConnected*/CmdProc::Cmd.connected()) {
    uint32_t m1=millis();
    //int packetSize = udp_rcv.parsePacket(); 
    if (/*packetSize*/CmdProc::Cmd.read()) {
      //int len = udp_rcv.read(packetBuffer, BUF_SZ);
      //if(len>BUF_SZ-1) len=BUF_SZ-1;
      //packetBuffer[len] = 0;    
 
      doCycle(); yield();
      /*
      // dbg out
      uint32_t m2=millis();
      Serial.print("From: "); Serial.print(udp_rcv.remoteIP());
      Serial.print(":"); Serial.print(udp_rcv.remotePort());
      Serial.print(" len: "); Serial.print(len);
      Serial.print(" T: "); Serial.print(m2-m1);
      Serial.print(" Val: "); Serial.println(packetBuffer);
      //
      doCycle(); yield();
      */
      
      //cmd.doCmd(packetBuffer);

      CmdProc::Cmd.doCmd();

     // resp
     /*
      udp_snd.beginPacket(udp_rcv.remoteIP(), udp_rcv.remotePort());
      udp_snd.write(packetBuffer, strlen(packetBuffer));
      udp_snd.endPacket();
*/

      doCycle(); yield();

      CmdProc::Cmd.respond();
      
      print_sys_info();

      if(CmdProc::Cmd.isSysLog()) CmdProc::Cmd.sendSysLog("syslog");
    }
  }
  doCycle();
}

void doCycle() {
  uint32_t t = millis();
  uint16_t dt=t-last_cycle;
  if(dt < CYCLE_TO) return;
  last_cycle = t;
  if(dt<=CYCLE_TO) Stat::StatStore.cnt[0]++;
  else if(dt<=CYCLE_TO<<1) Stat::StatStore.cnt[1]++;
  else if(dt<=CYCLE_TO<<2) Stat::StatStore.cnt[2]++;
  else Stat::StatStore.cnt[3]++;
}

void print_sys_info() {
  Serial.print("Heap sz: "); Serial.println(ESP.getFreeHeap());
  Serial.print("TO<="); Serial.print(CYCLE_TO); Serial.print(": "); Serial.print(Stat::StatStore.cnt[0]);
  Serial.print(" TO<="); Serial.print(CYCLE_TO<<1); Serial.print(": "); Serial.print(Stat::StatStore.cnt[1]);
  Serial.print(" TO<="); Serial.print(CYCLE_TO<<2); Serial.print(": "); Serial.print(Stat::StatStore.cnt[2]);
  Serial.print(" TO>"); Serial.print(CYCLE_TO<<2); Serial.print(": "); Serial.println(Stat::StatStore.cnt[3]);
}
