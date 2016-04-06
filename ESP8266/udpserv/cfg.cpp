//#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "FS.h"
#include "cfg.h"

#define MAX_CFG_LINE_SZ 80

const int NCFGS=3; 
const char *CFG_NAMES[]={"DBG", "SYSL", "TODO"};
enum CFG_ID {CFG_DBG=0, CFG_SYSL=1, CFG_TODO=2};

CfgDrv CfgDrv::Cfg; // singleton

CfgDrv::CfgDrv() : fs_ok(false), dirty(false), last_chg(0), debug_on(0), log_on(0), log_port(0) {;}

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

  while(c!=-1) { // while !EOF
    char *p = buf;
    while(1) { // new line
      c=f.read();
      //Serial.println((char)c);
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
          //Serial.println(F("Bad param or TODO"));
        }
      }
    } // new line
  } // while !EOF
  f.close();
  dirty=false;
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
  for(int i=0; i<NCFGS; i++) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    Serial.print(F("Writing ")); Serial.println(CFG_NAMES[i]);
    json["C"]=CFG_NAMES[i];
    switch(i) {
      case CFG_DBG: json["ON"]=debug_on; break;
      case CFG_SYSL: {
        String addr = log_addr.toString();
        json["ON"]=log_on;
        json["PORT"]=log_port;
        json["ADDR"]=addr;
        break;
      }
      //case CFG_TODO: json["ON"]=3; break;  
      default:;    
    }
    json.printTo(f);
    f.write('\n');
    yield();
  }
  f.close();
  uint16_t t=millis()-ms1;
  Serial.print(F("Cfg written in ")); Serial.println(t);
  return 1;
}

bool CfgDrv::needToStore() {
  return dirty && (millis()-last_chg)/1000>CfgDrv::LAZY_WRITE_TIMEOUT;
}

bool CfgDrv::setSysLog(JsonObject& root) {
  //uint8_t on = root["ON"] ? 1 : 0;
  uint8_t on = root["ON"];
  long port = root["PORT"];
  const char* addr = root["ADDR"];
  IPAddress newaddr;
  /*
  if(!on) { 
    if(!log_on) return true;
    Serial.println(F("SET_SYSL OFF")); 
    log_on=0;
    dirty=true; 
    last_chg=millis();
    return true;
  } 
  */ 
  if(on && !(port && addr && *addr && WiFi.hostByName(addr, newaddr))) return false;    
  if(log_addr==newaddr && log_port==port && log_on==on) return true; // nothing to change
  log_addr=newaddr;
  log_port=port;
  log_on=on;
  dirty=true;
  last_chg=millis();
  if(log_on) {
    Serial.print(F("SET_SYSL ")); Serial.print(log_on); Serial.print(":"); Serial.print(log_addr); Serial.print(":"); Serial.println(Cfg.log_port);     
  }
  else Serial.println(F("SET_SYSL OFF")); 
  return true;
}

