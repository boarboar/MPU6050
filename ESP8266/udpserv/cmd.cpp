//#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "cmd.h"
#include "stat.h"


CmdProc CmdProc::Cmd; // singleton

typedef int16_t (*VFP)(JsonObject&,JsonObject&);

int16_t _doCmd(JsonObject&,JsonObject&);
int16_t c_info(JsonObject&,JsonObject&);
int16_t c_reset(JsonObject&,JsonObject&);
int16_t c_setsyslog(JsonObject&,JsonObject&);

VFP cmd_imp[3]={c_info, c_reset, c_setsyslog};

//const int BUF_SZ = 255; 
const char *CMDS="INFO\0RST\0SYSL\0";
enum CMDS_ID {CMD_INFO=0, CMD_RESET=1, CMD_SETSYSLOG=2, CMD_NOCMD=3};
// {"I":1,"C":"INFO"}
// {"I":1,"C":"RST"}
// {"I":1,"C":"SYSL", "ON":1, "ADDR":"192.168.1.1", "PORT":4444}


boolean CmdProc::setLogger(bool on, const char *addr, uint16_t port) {
   log_on = on;
   if(!log_on) { 
      Serial.println("Remote logging off..."); 
      return true;
   }
   if(!port || !addr || !*addr) {
      log_on = false;
      return false;
   }   
   if(!WiFi.hostByName(addr, log_addr)) { log_on = false; return false; }
   log_port=port;
   Serial.print("SET_SYSL:"); Serial.print(log_addr); Serial.print(":"); Serial.println(log_port);     
   return true;
}

int16_t CmdProc::init(uint16_t port) {
  if(udp_rcv.begin(port)) {
    isConnected=true;
    return 1;
  }
  return 0;
}   

boolean CmdProc::read() {
  int packetSize = udp_rcv.parsePacket(); 
  if (packetSize) {
    int len = udp_rcv.read(packetBuffer, BUF_SZ);
    if(len>BUF_SZ-1) len=BUF_SZ-1;
    packetBuffer[len] = 0;    
  }
 return packetSize>0; 
}

void CmdProc::respond() {
  udp_snd.beginPacket(udp_rcv.remoteIP(), udp_rcv.remotePort());
  udp_snd.write(packetBuffer, strlen(packetBuffer));
  udp_snd.endPacket();
}


int16_t CmdProc::doCmd(/*char *buf*/) {  
  //if(!buf) return 0;
  char bufout[BUF_SZ];
 {
  //StaticJsonBuffer<JSON_OBJECT_SIZE(4)> jsonBufferIn;
  //StaticJsonBuffer<JSON_OBJECT_SIZE(4)+ JSON_ARRAY_SIZE(4)> jsonBufferOut;
  StaticJsonBuffer<200> jsonBufferIn;
  StaticJsonBuffer<200> jsonBufferOut;
  //JsonObject& root = jsonBufferIn.parseObject(buf);
  JsonObject& root = jsonBufferIn.parseObject(packetBuffer);
  JsonObject& rootOut = jsonBufferOut.createObject();
  _doCmd(root, rootOut);
  rootOut.printTo(Serial);
  rootOut.printTo(bufout, BUF_SZ-1);
 }
  //strcpy(buf, bufout);     
  strcpy(packetBuffer, bufout);     
  return 0;
}

boolean CmdProc::sendSysLog(const char *buf) {
  Serial.println("sending syslog...");
  udp_snd.beginPacket(log_addr, log_port);
  udp_snd.write(buf, strlen(buf));
  udp_snd.endPacket();  
}
  
int16_t _doCmd(JsonObject& root, JsonObject& rootOut) {  
  if (!root.success()) {
    //Serial.println("parseObject() failed");
    rootOut["I"] = -1;
    rootOut["R"] = -1;
    return 0;
  }

  long id = root["I"];
  const char* cmd = root["C"];
  
  //Serial.print("Id: "); Serial.print(id);
  rootOut["I"] = id;
  if(!cmd || !*cmd) {
    rootOut["R"] = -2;
    return 0;
  }
  if(cmd) {
    //Serial.print(" Cmd:"); Serial.println(cmd);
    rootOut["C"] = cmd;
    const char *p=CMDS;
    uint8_t i=0;
    while(*p && strcmp(p, cmd)) { p+=strlen(p)+1; i++; }
    if(i>=CMD_NOCMD) { rootOut["R"] = -2; return 0;}
    rootOut["R"] = (*(cmd_imp[i]))(root, rootOut);
  }      
  return 0;
}

int16_t c_info(JsonObject& root, JsonObject& rootOut) {
  //Serial.println("INFO"); 
  rootOut["fhs"]=ESP.getFreeHeap();
  rootOut["fss"]=ESP.getFreeSketchSpace();
  JsonArray& data = rootOut.createNestedArray("to");
  data.add(Stat::StatStore.cnt[0]);
  data.add(Stat::StatStore.cnt[1]);
  data.add(Stat::StatStore.cnt[2]);
  data.add(Stat::StatStore.cnt[3]);
  return 0;
}

int16_t c_reset(JsonObject& root, JsonObject& rootOut) {
  Serial.println("Resetting..."); 
  delay(1000);
  ESP.restart();
  return 0;
}

int16_t c_setsyslog(JsonObject& root, JsonObject& rootOut) {
  
    bool on = root["ON"]!=0;
    /*
    if(!log_on) {
      Serial.println("Remote logging off..."); 
      return 0;
    }
    */
    long port = root["PORT"];
    const char* addr = root["ADDR"];
    /*
    if(port==0 || addr==NULL || !*addr) {
       log_on = false;
       return -3;      
    }

    return CmdProc::Cmd.setLogger(addr, port) ? 0 : -3;
    */
    return CmdProc::Cmd.setLogger(on, addr, port) ? 0 : -3;
}



