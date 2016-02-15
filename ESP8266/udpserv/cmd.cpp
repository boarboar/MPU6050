#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "stat.h"
#include "cfg.h"
#include "mpu.h"

#include "cmd.h"

CmdProc CmdProc::Cmd; // singleton

typedef int16_t (*VFP)(JsonObject&,JsonObject&);

int16_t _doCmd(JsonObject&,JsonObject&);
int16_t c_info(JsonObject&,JsonObject&);
int16_t c_reset(JsonObject&,JsonObject&);
int16_t c_setsyslog(JsonObject&,JsonObject&);
int16_t c_getpos(JsonObject&,JsonObject&);

VFP cmd_imp[4]={c_info, c_reset, c_setsyslog, c_getpos};

const char *CMDS="INFO\0RST\0SYSL\0POS\0";
enum CMDS_ID {CMD_INFO=0, CMD_RESET=1, CMD_SETSYSLOG=2, CMD_POS=3, CMD_NOCMD=4};
// {"I":1,"C":"INFO"}
// {"I":1,"C":"RST"}
// {"I":1,"C":"SYSL", "ON":1, "ADDR":"192.168.1.141", "PORT":4444}
// {"I":1,"C":"POS"}

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
      /*
      Serial.print("From: "); Serial.print(udp_rcv.remoteIP()); Serial.print(":"); Serial.print(udp_rcv.remotePort());
      Serial.print(" len: "); Serial.print(len);Serial.print(" Val: "); Serial.println(packetBuffer);
      */      
  }
 return packetSize>0; 
}

void CmdProc::respond() {
  udp_snd.beginPacket(udp_rcv.remoteIP(), udp_rcv.remotePort());
  udp_snd.write(packetBuffer, strlen(packetBuffer));
  udp_snd.endPacket();
}


int16_t CmdProc::doCmd() {    
  char bufout[BUF_SZ];
  //StaticJsonBuffer<JSON_OBJECT_SIZE(4)> jsonBufferIn;
  //StaticJsonBuffer<JSON_OBJECT_SIZE(4)+ JSON_ARRAY_SIZE(4)> jsonBufferOut;
  StaticJsonBuffer<200> jsonBufferIn;
  StaticJsonBuffer<400> jsonBufferOut;
  JsonObject& root = jsonBufferIn.parseObject(packetBuffer);
  JsonObject& rootOut = jsonBufferOut.createObject();
  _doCmd(root, rootOut);
  //rootOut.printTo(Serial);
  rootOut.printTo(bufout, BUF_SZ-1);
  strcpy(packetBuffer, bufout);     
  return 0;
}

boolean CmdProc::isSysLog() { return CfgDrv::Cfg.log_on;}

boolean CmdProc::sendSysLog(const char *buf) {
  if(!CfgDrv::Cfg.log_on) return false;
  StaticJsonBuffer<200> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  char bufout[BUF_SZ];
  rootOut["S"] = "I";
  rootOut["T"] = millis();
  rootOut["M"] = buf;
  rootOut.printTo(bufout, BUF_SZ-1);
  udp_snd.beginPacket(CfgDrv::Cfg.log_addr, CfgDrv::Cfg.log_port);
  udp_snd.write(bufout, strlen(bufout));
  udp_snd.endPacket();  
}

boolean CmdProc::sendSysLogStatus() {
  if(!CfgDrv::Cfg.log_on) return false;
  StaticJsonBuffer<200> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  char bufout[BUF_SZ];
  rootOut["S"] = "I";
  rootOut["T"] = millis();
  rootOut["R"] = 0; // OK, shoild be -5 if tracking is off (reserved)
  rootOut["X"] = rand()%100;
  rootOut["Y"] = rand()%100;
  rootOut.printTo(bufout, BUF_SZ-1);
  udp_snd.beginPacket(CfgDrv::Cfg.log_addr, CfgDrv::Cfg.log_port);
  udp_snd.write(bufout, strlen(bufout));
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
  rootOut["MST"]=MpuDrv::Mpu.isReady() ? 1 : 0;
  rootOut["MDR"]=MpuDrv::Mpu.isDataReady() ? 1 : 0;
  rootOut["FHS"]=ESP.getFreeHeap();
  rootOut["FSS"]=ESP.getFreeSketchSpace();
  rootOut["MDC"]=Stat::StatStore.cycle_mpu_dry_cnt;
  JsonArray& data = rootOut.createNestedArray("CYT");
  for(int i=0; i<4; i++) data.add(Stat::StatStore.cycle_delay_cnt[i]);
  return 0;
}

int16_t c_reset(JsonObject& root, JsonObject& rootOut) {
  Serial.println(F("Resetting...")); 
  delay(1000);
  ESP.restart();
  return 0;
}

int16_t c_setsyslog(JsonObject& root, JsonObject& rootOut) {
  return CfgDrv::Cfg.setSysLog(root) ? 0 : -3;
}

int16_t c_getpos(JsonObject& root, JsonObject& rootOut) {
  if(!MpuDrv::Mpu.isDataReady()) return -5;
  
  rootOut["X"] = rand()%100;
  rootOut["Y"] = rand()%100;
  JsonArray& qa = rootOut.createNestedArray("Q");
  Quaternion& q = MpuDrv::Mpu.getQuaternion(); 
  qa.add(q.w); qa.add(q.x); qa.add(q.y); qa.add(q.z);
  JsonArray& ga = rootOut.createNestedArray("G");
  VectorFloat& g = MpuDrv::Mpu.getGravity(); 
  ga.add(g.x); ga.add(g.y); ga.add(g.z);
  JsonArray& ya = rootOut.createNestedArray("YPR");
  float *ypr  = MpuDrv::Mpu.getYPR(); 
  ya.add(ypr[0] * 180/M_PI); ya.add(ypr[1] * 180/M_PI); ya.add(ypr[2] * 180/M_PI);
  JsonArray& aa = rootOut.createNestedArray("A");
  VectorInt16 a  = MpuDrv::Mpu.getWorldAccel(); 
  aa.add(a.x); aa.add(a.y); aa.add(a.z);

/*
            Serial.print("cmd quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
  */
            /*
//yield();
            Serial.print("Cmd Grav\t");
            Serial.print(g.x);
            Serial.print("\t");
            Serial.print(g.y);
            Serial.print("\t");
            Serial.println(g.z);
yield();
*/
/*
            Serial.print("YPR\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
*/

//yield();

            Serial.print("Cmd Acc\t");
            Serial.print(a.x);
            Serial.print("\t");
            Serial.print(a.y);
            Serial.print("\t");
            Serial.println(a.z);

  return 0;
}



