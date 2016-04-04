#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "stat.h"
#include "cfg.h"
#include "cmd.h"
#include "mpu.h"
#include "controller.h"

void doCycle();

const char* ssid = "NETGEAR";
const char* password = "boarboar";
const char* cfg_file = "/config.json";
const int udp_port = 4444;
const int CYCLE_TO = 5;
const int CYCLE_SLOW_TO = 500;
const int MPU_SDA=0;
const int MPU_SDL=2;
const int MPU_INT=15;

const int PERIPH_UNIT_ID=4;

uint32_t last_cycle;
uint32_t last_slow_cycle;

CmdProc& cmd = CmdProc::Cmd;
 
void pcf_test();

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  if(CfgDrv::Cfg.init() && CfgDrv::Cfg.load(cfg_file)) {
    Serial.println(F("Cfg loaded"));
  } else {
    Serial.println(F("Failed to load cfg, using default!"));
  }
  
  WiFi.begin(ssid, password);
  Serial.print(F("\nConnecting to ")); Serial.print(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) {delay(500); Serial.print(".");/*delay(5); MpuDrv::Mpu.cycle(5);*/}
  Serial.println();
  if(i == 21){
    Serial.print(F("Could not connect to ")); Serial.println(ssid);
    delay(10000);
    ESP.reset();
  }
  
  delay(1000);

  cmd.sendAlarm(CmdProc::ALR_RESET, 0);
  
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

  Wire.begin(MPU_SDA, MPU_SDL);
    
  if(MpuDrv::Mpu.init(/*MPU_SDA, MPU_SDL, */MPU_INT)) { //sda, sdl, intr
    Serial.println(F("MPU Ready!"));
  } else {
    Serial.println(F("Failed to init MPU!"));
    //delay(1000);
    //ESP.reset();
  }  

  if(Controller::ControllerProc.init()) { 
    Serial.print(F("Ctrl Ready! Sensors: "));
    Serial.println(Controller::ControllerProc.getNumSensors());    
  } else {
    Serial.println(F("Failed to init Ctrl!"));
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
  
  // Do fast cycle
  int16_t mpu_res = MpuDrv::Mpu.cycle(dt);
/*
  if(mpu_res==1) {
    //integrate
  }
  */
  // collect delay statistics
  int i=0;
  while(i<3 && dt>(CYCLE_TO<<i)) i++;
  Stat::StatStore.cycle_delay_cnt[i]++;
  if(!mpu_res) Stat::StatStore.cycle_mpu_dry_cnt++;
  dt=t-last_slow_cycle;
  if(dt < CYCLE_SLOW_TO) return;
  last_slow_cycle = t;
  
  // Do medium cycle // (200ms - later on). Now in slow cycle
  if(Controller::ControllerProc.process()) {
    // integrate
    // couple controller & mpu data here (is it needed ???)
  }
  
  if(MpuDrv::Mpu.getFailReason()) {
    Serial.print(F("MPU FAILURE ")); Serial.println(MpuDrv::Mpu.getFailReason());
    cmd.sendAlarm(CmdProc::ALR_MPU_FAILURE, MpuDrv::Mpu.getFailReason());
    MpuDrv::Mpu.clearFailReason();
  }
  if(MpuDrv::Mpu.isNeedReset()) {
    cmd.sendAlarm(CmdProc::ALR_MPU_RESET, 0);
    MpuDrv::Mpu.init();
  }
  if(Controller::ControllerProc.getFailReason()) {
    Serial.print(F("CTL FAILURE ")); Serial.println(Controller::ControllerProc.getFailReason());
    cmd.sendAlarm(CmdProc::ALR_CTL_FAILURE, Controller::ControllerProc.getFailReason());
    Controller::ControllerProc.clearFailReason();
  }
  if(Controller::ControllerProc.isNeedReset()) {
    cmd.sendAlarm(CmdProc::ALR_CTL_RESET, 0);
    Controller::ControllerProc.init();
  }
  if(CfgDrv::Cfg.needToStore()) CfgDrv::Cfg.store(cfg_file);
  cmd.sendSysLogStatus();

  pcf_test();
}

int ps=0;
void pcf_test() { 
   ps++;
   uint8_t s=ps%2 ? 255 : 0;
   Wire.beginTransmission(0x20);
   Wire.write(s); 
   int res=Wire.endTransmission();
   
 /*
  Serial.print("PCF test ");
  Serial.print(s);
  Serial.print(" \tres=");
  Serial.println(res);
  */
}


