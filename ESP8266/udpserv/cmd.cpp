#include <Arduino.h>
#include <ArduinoJson.h>

#include "cmd.h"

int16_t _doCmd(JsonObject&,JsonObject&);
void c_info(JsonObject&,JsonObject&);
void c_reset(JsonObject&,JsonObject&);

typedef void (*VFP)(JsonObject&,JsonObject&);

const int BUF_SZ = 255; 
const char *CMDS="I\0R\0";
enum CMDS_ID {CMD_INFO=0, CMD_RESET=1, CMD_NOCMD=2};
//void (*vf)()[2]={c_info, c_reset};

VFP cmd_imp[2]={c_info, c_reset};

// {"i":id,"c":"cmd","p1":param}
// {"i":1,"c":"I"}
   
int16_t CmdProc::doCmd(char *buf) {  
  if(!buf) return 0;
  char bufout[BUF_SZ];
 {
  StaticJsonBuffer<JSON_OBJECT_SIZE(4)> jsonBufferIn;
  StaticJsonBuffer<JSON_OBJECT_SIZE(4)> jsonBufferOut;
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
    rootOut["i"] = -1;
    rootOut["r"] = -1;
    return 0;
  }

  long id = root["i"];
  const char* cmd = root["c"];
  
  Serial.print("Id: "); Serial.print(id);
  rootOut["i"] = id;
  if(!cmd || !*cmd) {
    rootOut["r"] = -2;
    return 0;
  }
  if(cmd) {
    Serial.print(" Cmd:"); Serial.println(cmd);
    rootOut["r"] = 0;
    rootOut["c"] = cmd;
    const char *p=CMDS;
    uint8_t i=0;
    while(*p && strcmp(p, cmd)) { p+=strlen(p)+1; i++; }
    if(i>=CMD_NOCMD) { rootOut["r"] = -2; return 0;}
    (*(cmd_imp[i]))(root, rootOut);
  }      
  return 0;
}

void c_info(JsonObject& root, JsonObject& rootOut) {
  Serial.println("INFO"); 
  rootOut["fhs"]=ESP.getFreeHeap();
  rootOut["fss"]=ESP.getFreeSketchSpace();
}

void c_reset(JsonObject& root, JsonObject& rootOut) {
  Serial.println("RST"); 
  delay(1000);
  ESP.restart();
}



