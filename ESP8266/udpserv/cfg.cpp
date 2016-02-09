//#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "FS.h"
#include "cfg.h"

#define MAX_CFG_LINE_SZ 80
/*
 * 
 * SPIFFS.open and dir.openFile functions return a File object. 
 * This object supports all the functions of Stream, so you can use readBytes, findUntil, parseInt, println, and all other Stream methods.
There are also some functions which are specific to File object.
 */
CfgDrv CfgDrv::Cfg; // singleton

CfgDrv::CfgDrv() : fs_ok(false), dirty(false), debug_on(0), log_on(0), log_port(0) {;}

int16_t CfgDrv::init() {
  fs_ok=SPIFFS.begin();
  if (!fs_ok) {
    Serial.println(F("Failed to mount FS"));
    return 0;
  }
  Serial.println(F("FS mounted"));
  return 1;
}

int16_t CfgDrv::load(const char* fname) {
  if (!fs_ok) return 0;
  
  char buf[MAX_CFG_LINE_SZ];
  uint32_t ms1=millis();
  File f = SPIFFS.open(fname, "r");
  if (!f) {
    Serial.println(F("Failed to open config file"));
    return 0;
  }
  size_t size = f.size();
  int c=0;
  char *p = buf;
  while(c!=-1) { // while !EOF
    while(1) { // new line
      c=f.read();
      if(c==-1 || c=='\n'  || c=='\r') break;
      if(p-buf<MAX_CFG_LINE_SZ-1) *p++=c; 
    }
    if(p>buf) { //non-empty
      *p=0; 
      Serial.println(buf);  
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& json = jsonBuffer.parseObject(buf);  
      if (json.success()) {
        const char* cmd = json["C"];
        if(!strcmp(cmd, "SYSL")) setSysLog(json);
        else {
          Serial.println(F("Bad param or TODO"));
        }
      }
    } // new line
  } // while !EOF
  f.close();
  uint16_t t=millis()-ms1;
  Serial.print(F("Cfg sz ")); Serial.print(size); Serial.print(F(", read in ")); Serial.println(t);
  return 1;
}

int16_t CfgDrv::store(const char* fname) {
  if (!fs_ok) return 0;
  uint32_t ms1=millis();
  dirty=false; // to avoid multiple writes...
  File f = SPIFFS.open(fname, "w");
  if (!f) {
    Serial.println(F("Failed to open config file (w)"));
    return 0;
  }
  {
  //{"C":"DBG", "ON":1}
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["C"]="DBG";
  json["ON"]=debug_on;
  json.printTo(f);
  }
  {
  //{"C":"SYSL", "ON":1, "ADDR":"192.168.1.141", "PORT":4444}
  String addr = log_addr.toString();
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["C"]="SYSL";
  json["ON"]=log_on;
  json["PORT"]=log_port;
  json["ADDR"]=addr;
  json.printTo(f);
  }
  {
  //{"C":"TODO", "ON":1}
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["C"]="TODO";
  json["ON"]=3;
  json.printTo(f);
  }
  f.close();
  uint16_t t=millis()-ms1;
  Serial.print(F("Cfg written in ")); Serial.println(t);
  return 1;
}

bool CfgDrv::setSysLog(JsonObject& root) {
  uint8_t on = root["ON"] ? 1 : 0;
  long port = root["PORT"];
  const char* addr = root["ADDR"];
  IPAddress newaddr;
  if(!on) { 
    if(!log_on) return true;
    Serial.println(F("SET_SYSL OFF")); 
    log_on=0;
    dirty=true; 
    return true;
  }  
  if(!port || !addr || !*addr) {
    //log_on = false;
    return false;
  }     
  if(!WiFi.hostByName(addr, newaddr)) { /*log_on = false;*/ return false; }
  if(log_addr==newaddr && log_port==port && on) return true;
  log_addr=newaddr;
  log_port=port;
  log_on=1;
  dirty=true;
  Serial.print(F("SET_SYSL ON:")); Serial.print(log_addr); Serial.print(":"); Serial.println(Cfg.log_port);     
  return true;
}

