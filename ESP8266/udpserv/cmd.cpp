#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "stat.h"
#include "cfg.h"
#include "mpu.h"
#include "controller.h"

#include "cmd.h"

CmdProc CmdProc::Cmd; // singleton

typedef int16_t (*VFP)(JsonObject&,JsonObject&);

int16_t _doCmd(JsonObject&,JsonObject&);
int16_t c_info(JsonObject&,JsonObject&);
int16_t c_reset(JsonObject&,JsonObject&);
int16_t c_setsyslog(JsonObject&,JsonObject&);
int16_t c_getpos(JsonObject&,JsonObject&);
int16_t c_resetMPU(JsonObject&,JsonObject&);

VFP cmd_imp[5]={c_info, c_reset, c_setsyslog, c_getpos, c_resetMPU};

const char *CMDS="INFO\0RST\0SYSL\0POS\0RSTMPU\0";
enum CMDS_ID {CMD_INFO=0, CMD_RESET=1, CMD_SETSYSLOG=2, CMD_POS=3, CMD_RESET_MPU=4, CMD_NOCMD=5};
// {"I":1,"C":"INFO"}
// {"I":1,"C":"RST"}
// {"I":1,"C":"SYSL", "ON":1, "ADDR":"192.168.1.141", "PORT":4444}
// {"I":1,"C":"POS"}
// {"I":1,"C":"RSTMPU"}

int16_t CmdProc::init(uint16_t port) {
  if(udp_rcv.begin(port)) {
    isConnected=true;
    return 1;
  }
  return 0;
}   

boolean CmdProc::connected() { return isConnected; }

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

int16_t CmdProc::getSysLogLevel() { return CfgDrv::Cfg.log_on;}


boolean CmdProc::sendSysLog(const char *buf) {
  if(CfgDrv::Cfg.log_on<SL_LEVEL_MESSAGE) return false;
  StaticJsonBuffer<200> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  rootOut["C"] = "I";
  rootOut["T"] = millis();
  rootOut["M"] = buf;
  _sendToSysLog(rootOut);
}

boolean CmdProc::sendSysLogStatus() {
  //{"C": "L", "T":12345, "R":0, "YPR": [59, 12, 13], "A": [0.01, 0.02, -0.03], "V": [0.1, 0.2, -0.3]}
  if(CfgDrv::Cfg.log_on<SL_LEVEL_MESSAGE) return false;
  StaticJsonBuffer<400> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  //char bufout[BUF_SZ];
  rootOut["C"] = "L";
  rootOut["T"] = millis();
  rootOut["R"] = c_getpos(rootOut, rootOut); 
  _sendToSysLog(rootOut);
}

boolean CmdProc::sendAlarm(uint8_t alr, uint8_t param) {
  //{"C": "A", "T":12345, "R":1123, "P":1123} 
  if(CfgDrv::Cfg.log_on<SL_LEVEL_ALARM) return false;
  StaticJsonBuffer<200> jsonBufferOut;
  JsonObject& rootOut = jsonBufferOut.createObject();
  rootOut["C"] = "A";
  rootOut["T"] = millis();
  rootOut["R"] = alr;
  rootOut["P"] = param;
  _sendToSysLog(rootOut);
}

void CmdProc::_sendToSysLog(JsonObject& rootOut) {
  rootOut.printTo(packetBuffer, BUF_SZ-1);
  udp_snd.beginPacket(CfgDrv::Cfg.log_addr, CfgDrv::Cfg.log_port);
  udp_snd.write(packetBuffer, strlen(packetBuffer));
  udp_snd.endPacket();  
}

int16_t _doCmd(JsonObject& root, JsonObject& rootOut) {  
  rootOut["T"] = millis();
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
  rootOut["MST"]=MpuDrv::Mpu.getStatus();
  rootOut["MDR"]=MpuDrv::Mpu.isDataReady();
  rootOut["FHS"]=ESP.getFreeHeap();
  rootOut["FSS"]=ESP.getFreeSketchSpace();
  rootOut["MDC"]=Stat::StatStore.cycle_mpu_dry_cnt;
  rootOut["MOC"]=Stat::StatStore.mpu_owfl_cnt;
  rootOut["MGC"]=Stat::StatStore.mpu_gup_cnt;
  rootOut["MXC"]=Stat::StatStore.mpu_exc_cnt;
  rootOut["MND"]=Stat::StatStore.mpu_ndt_cnt;
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

int16_t c_resetMPU(JsonObject& root, JsonObject& rootOut) {
  const char *action=root["A"];
  if(!action || !*action) return -3;
  if(!strcmp(action, "MPU")) {
    Serial.println(F("Req to RST MPU/CTL...")); 
    //MpuDrv::Mpu.init();
    MpuDrv::Mpu.needReset();
    Controller::ControllerProc.needReset();
  } else if(!strcmp(action, "MPU_INT")) {
    Serial.println(F("Resetting MPU/CTL integrator...")); 
    MpuDrv::Mpu.resetIntegrator();
    Controller::ControllerProc.resetIntegrator();
  } else return -3;
  return 0;
}

int16_t c_setsyslog(JsonObject& root, JsonObject& rootOut) {
  return CfgDrv::Cfg.setSysLog(root) ? 0 : -3;
}

int16_t c_getpos(JsonObject& root, JsonObject& rootOut) {
  //{"C": "I", "T":12345, "R":0, "C": "POS", "YPR": [59, 12, 13], "A": [0.01, 0.02, -0.03], "V": [0.1, 0.2, -0.3], "P": [100.01, 200.44, 0.445]}
  rootOut["MST"]=MpuDrv::Mpu.getStatus();
  if(!MpuDrv::Mpu.isDataReady()) return -5;
  JsonArray& ya = rootOut.createNestedArray("YPR");
  float ypr[3], af[3], vf[3]/*, rf[3]*/;
  uint8_t i;
  MpuDrv::Mpu.getAll(ypr, af, vf); 
  for(i=0; i<3; i++) ya.add(ypr[i] * 180/M_PI);
  //JsonArray& aa = rootOut.createNestedArray("A");
  //for(i=0; i<3; i++) aa.add(af[i]);  
  JsonArray& v = rootOut.createNestedArray("V");
  for(i=0; i<3; i++) v.add(vf[i]);
  JsonArray& r = rootOut.createNestedArray("CRD");
  for(i=0; i<3; i++) r.add(0.0f);
  
  uint8_t ns=Controller::ControllerProc.getNumSensors();
  JsonArray& s = rootOut.createNestedArray("S");
  for(i=0; i<3; i++) s.add(Controller::ControllerProc.getStoredSensors()[i]);
  
  rootOut["D"]=Controller::ControllerProc.getDistance();
  return 0;
}



