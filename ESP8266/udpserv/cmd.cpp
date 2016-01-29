#include <Arduino.h>
#include <ArduinoJson.h>

#include "cmd.h"

StaticJsonBuffer<200> jsonBuffer; // size - TBD



// {"i":id,"c":"cmd","p1":param}
// {"i":1,"c":"I"}
   
int16_t CmdProc::doCmd(char *buf) {
  if(!buf) return 0;

  JsonObject& root = jsonBuffer.parseObject(buf);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return 0;
  }

  long id = root["i"];
  const char* cmd = root["c"];

  Serial.print("Id: "); Serial.print(id);
  if(cmd) Serial.print("Cmd:"); Serial.println(cmd);
      
  return 0;
}

