#include <Arduino.h>
#include <ArduinoJson.h>

#include "cmd.h"
#include "stat.h"

typedef int16_t (*VFP)(JsonObject&,JsonObject&);

int16_t _doCmd(JsonObject&,JsonObject&);
int16_t c_info(JsonObject&,JsonObject&);
int16_t c_reset(JsonObject&,JsonObject&);
int16_t c_setsyslog(JsonObject&,JsonObject&);

VFP cmd_imp[2]={c_info, c_reset};

const int BUF_SZ = 255; 
const char *CMDS="INFO\0RST\0SYSL\0";
enum CMDS_ID {CMD_INFO=0, CMD_RESET=1, CMD_SETSYSLOG=2, CMD_NOCMD=3};
// {"I":1,"C":"INFO"}
// {"I":1,"C":"RST"}
// {"I":1,"C":"SYSL", "ADDR":"ipaddr", "PORT":port}
   
int16_t CmdProc::doCmd(char *buf) {  
  if(!buf) return 0;
  char bufout[BUF_SZ];
 {
  StaticJsonBuffer<JSON_OBJECT_SIZE(4)> jsonBufferIn;
  StaticJsonBuffer<JSON_OBJECT_SIZE(4)+ JSON_ARRAY_SIZE(4)> jsonBufferOut;
  JsonObject& root = jsonBufferIn.parseObject(buf);
  JsonObject& rootOut = jsonBufferOut.createObject();
  _doCmd(root, rootOut);
  rootOut.printTo(Serial);
  rootOut.printTo(bufout, 254);
 }
  strcpy(buf, bufout);     
  return 0;
}

int16_t _doCmd(JsonObject& root, JsonObject& rootOut) {  
  if (!root.success()) {
    Serial.println("parseObject() failed");
    rootOut["I"] = -1;
    rootOut["R"] = -1;
    return 0;
  }

  long id = root["I"];
  const char* cmd = root["C"];
  
  Serial.print("Id: "); Serial.print(id);
  rootOut["I"] = id;
  if(!cmd || !*cmd) {
    rootOut["R"] = -2;
    return 0;
  }
  if(cmd) {
    Serial.print(" Cmd:"); Serial.println(cmd);
    //rootOut["R"] = 0;
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
  Serial.println("INFO"); 
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
  Serial.println("RST"); 
  delay(1000);
  ESP.restart();
  return 0;
}

int16_t c_setsyslog(JsonObject& root, JsonObject& rootOut) {
    long port = root["PORT"];
    const char* addr = root["ADDR"];
    if(port==0 || addr==NULL || !*addr) {
       return -3;      
    }
    Serial.print("SET_SYSL:"); Serial.print(addr); Serial.print(":"); Serial.println(port); 
    return 0;
}



