#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "cmd.h"
#include "stat.h"
#include "cfg.h"
#include "mpu.h"

void doCycle();

const char* ssid = "NETGEAR";
const char* password = "boarboar";
const char* cfg_file = "/config.json";
const int udp_port = 4444;
const int CYCLE_TO = 2;
const int CYCLE_SLOW_TO = 1000;
const int MPU_SDA=0;
const int MPU_SDL=2;
const int MPU_INT=15;

uint32_t last_cycle;
uint32_t last_slow_cycle;

CmdProc& cmd = CmdProc::Cmd; 

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  if(CfgDrv::Cfg.init() && CfgDrv::Cfg.load(cfg_file)) {
    Serial.println(F("Cfg loaded"));
  } else {
    Serial.println(F("Failed to load cfg, using default!"));
  }
  
  WiFi.begin(ssid, password);
  Serial.print(F("\nConnecting to ")); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print(F("Could not connect to ")); Serial.println(ssid);
    delay(10000);
    ESP.reset();
  }

  if(cmd.init(udp_port)) {
    Serial.print(F("Ready! Listening on "));
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(udp_port);
  } else {
    Serial.println(F("Failed to init UDP socket!"));
    delay(1000);
    ESP.reset();
  } 
   
  yield();
  
  if(MpuDrv::Mpu.init(MPU_SDA, MPU_SDL, MPU_INT)) { //sda, sdl, intr
    Serial.println(F("MPU Ready!"));
  } else {
    Serial.println(F("Failed to init MPU!"));
    //delay(1000);
    //ESP.reset();
  }  
  last_cycle=last_slow_cycle=millis();
}

void loop() {
  if(cmd.connected()) {
    uint32_t m1=millis();
    if (cmd.read()) {
      doCycle(); yield();
      cmd.doCmd();
      doCycle(); yield();
      cmd.respond();      
    }
  }
  doCycle();
}

void doCycle() {
  uint32_t t = millis();
  uint16_t dt=t-last_cycle;
  if(dt < CYCLE_TO) return;
  last_cycle = t;
  MpuDrv::Mpu.cycle(dt);
  /*
  if(dt<=CYCLE_TO) Stat::StatStore.cnt[0]++;
  else if(dt<=CYCLE_TO<<1) Stat::StatStore.cnt[1]++;
  else if(dt<=CYCLE_TO<<2) Stat::StatStore.cnt[2]++;
  else Stat::StatStore.cnt[3]++;
  */
  // collect delay statistics
  int i=0;
  while(i<3 && dt>(CYCLE_TO<<i)) i++;
  Stat::StatStore.cnt[i]++;
   
  dt=t-last_slow_cycle;
  if(dt < CYCLE_SLOW_TO) return;
  last_slow_cycle = t;
  if(CfgDrv::Cfg.needToStore()) CfgDrv::Cfg.store(cfg_file);
  if(cmd.isSysLog()) cmd.sendSysLogStatus();
}

