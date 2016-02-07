//#include <Arduino.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "cfg.h"

/*
 * 
 * SPIFFS.open and dir.openFile functions return a File object. 
 * This object supports all the functions of Stream, so you can use readBytes, findUntil, parseInt, println, and all other Stream methods.
There are also some functions which are specific to File object.
 */
CfgDrv CfgDrv::Cfg; // singleton

CfgDrv::CfgDrv() : dirty(false), fs_ok(false) {;}

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
  char buf[256];
  File f = SPIFFS.open(fname, "r");
  if (!f) {
    Serial.println(F("Failed to open config file"));
    return 0;
  }

  size_t size = f.size();
  Serial.print(F("Cfg sz ")); Serial.println(size);

  int c;
  char *p = buf;
  while(1) {
    c=f.read();
    //Serial.print(c); 
    if(c==-1 || c=='\n'  || c=='\r') break;
    if(p-buf<254) *p++=c; 
  }
  *p=0;
  
  Serial.println(buf);  
  
  f.close();
  return 1;
}

int16_t CfgDrv::store(const char* fname) {
  dirty=false;
  return 1;
}


