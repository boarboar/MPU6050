#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "controller.h"
#include "cfg.h"

const int DEV_ID=4;
const int M_POW_MIN=30; 
const int M_POW_MAX=240;
const int M_POW_NORM=140;
const int M_SPEED_NORM=200;

/*
const int gain_p=20;
const int gain_d=320;
const int gain_i=10;
const int gain_div=80;
const int limit_i=100;
*/
    
Controller Controller::ControllerProc ; // singleton

Controller::Controller() : pready(false), nsens(0), act_rot_rate{0},act_advance{0},act_power{0},sensors{0} {
  }

uint8_t Controller::getStatus() { return pready; }
uint8_t Controller::isDataReady() { return pready && data_ready; }
uint8_t Controller::isNeedReset() { return need_reset; }
void    Controller::needReset() {  need_reset=true; }
uint8_t Controller::getFailReason() { return fail_reason; }
void  Controller::clearFailReason() { fail_reason=CTL_FAIL_NONE; }

void Controller::raiseFail(uint8_t reason, int16_t p1, int16_t p2, int16_t p3, int16_t p4, int16_t p5, int16_t p6) {
  fail_reason=reason;
  fail_p[0]=p1;
  fail_p[1]=p2;
  fail_p[2]=p3;
  fail_p[3]=p4;
  fail_p[4]=p5;
  fail_p[5]=p6;
/*
  Serial.print(F("CTRL ALR: ")); Serial.print(reason); 
  for(int i=0; i<4; i++) {
    Serial.print(F(" \t ")); Serial.print(fail_p[i]); 
  }   
  Serial.println();
  */
}

void Controller::getFailParams(int16_t npa, int16_t *pa) {
  if(npa>6) npa=6;
  for(int i=0; i<npa; i++) pa[i]=fail_p[i]; 
 } 

bool Controller::init() {
  pready=false;
  data_ready=0;
  fail_reason=0;
  need_reset=false; 
  nsens=0;
  //targ_rot_rate[0]=targ_rot_rate[1]=0;
  act_rot_rate[0]=act_rot_rate[1]=0;
  act_advance[0]=act_advance[1]=0;
  //act_advance_0[0]=act_advance_0[1]=0;
  act_power[0]=act_power[1]=0;
  //targ_pow[0]=targ_pow[1]=0;
  cur_pow[0]=cur_pow[1]=0;
  targ_speed=0;
  for(int i=0; i<SENS_SIZE; i++) sensors[i]=0;
  resetIntegrator();
  pready=testConnection();
  if(pready) {
    raiseFail(CTL_FAIL_NONE);
    nsens=_getNumSensors();      
  } else {
    raiseFail(CTL_FAIL_INIT);
    need_reset=true;
  }

  gain_p=CfgDrv::Cfg.bear_pid.gain_p;
  gain_d=CfgDrv::Cfg.bear_pid.gain_d;
  gain_i=CfgDrv::Cfg.bear_pid.gain_i;
  gain_div=CfgDrv::Cfg.bear_pid.gain_div;
  limit_i=CfgDrv::Cfg.bear_pid.limit_i;

  Serial.print(F("CTRL INIT PID: ")); Serial.print(gain_p); Serial.print(F(" \t ")); Serial.print(gain_d); Serial.print(F(" \t ")); Serial.print(gain_i);
  Serial.print(F(" \t ")); Serial.print(gain_div); Serial.print(F(" \t ")); Serial.println(limit_i);
  
  return pready;
}

void Controller::resetIntegrator() {
  dist=angle=0.0f;
  r[0]=r[1]=0.0f;
  speed=0;
  for(int i=0; i<SPEED_R_SZ; i++) speed_r[i]=0;
  curr_yaw=0;
  targ_bearing=0;  
  err_bearing_p_0=err_bearing_i=0;
  act_advance_0[0]=act_advance[0];
  act_advance_0[1]=act_advance[1];
  err_speed_p_0=err_speed_i=0; 
  pid_cnt=0;
  Serial.print(F("CTRL RST INT R: ")); Serial.print(getX()); Serial.print(F(" \t ")); Serial.println(getY());  
  
}

bool Controller::start() {
  Serial.println(F("CTRL START"));
  setStart(1);
  return true;
}

uint8_t Controller::testConnection() {
  bool res = I2Cdev::readByte(DEV_ID, REG_WHO_AM_I, buf); 
  if(!res) return 0;
  return buf[0]==M_OWN_ID;
}

bool Controller::process(float yaw, uint32_t dt) {
  if(!pready) return false;
  curr_yaw=yaw;
  
 // if(!getControllerStatus()) return false;

 //Serial.print(F("CTRL STAT: ")); Serial.print(sta[0]); Serial.print(F(" \t ")); Serial.println(sta[1]);

  if(!getActAdvance()) return false;

/*
  if(sta[1]) { // experimental
    // Serial.print(F("CTRL STAT ALR: ")); Serial.print(sta[0]); Serial.print(F(" \t ")); Serial.println(sta[1]);
    raiseFail(CTL_FAIL_ALR, sta[0], sta[1], 0, 0);
    return false;
  }
*/

  float dist0=dist;
  dist=(float)(act_advance[0]+act_advance[1])*0.5f; // in mm;
  mov=dist-dist0;
   
  if(abs(act_advance[0]-act_advance_0[0])>512 || abs(act_advance[1]-act_advance_0[1])>512) {
    //raiseFail(CTL_FAIL_OVF, act_advance[0], act_advance[1]);
        Serial.print(F("ADV=")); 
        Serial.print(act_advance[0]); Serial.print(F("\t ")); Serial.println(act_advance[1]);
        Serial.print(F("\t ADV0=\t ")); 
        Serial.print(act_advance_0[0]); Serial.print(F("\t ")); Serial.println(act_advance_0[1]);

    raiseFail(CTL_FAIL_OVF, buf[0], buf[1], buf[2], buf[3]);
    act_advance_0[0]=act_advance[0];
    act_advance_0[1]=act_advance[1];
    return false;
  }
    
  rot=(float)((act_advance[0]-act_advance_0[0])-(act_advance[1]-act_advance_0[1]))/(float)WHEEL_BASE_MM;  
  angle+=rot;  
  if(angle>PI) angle-=PI*2.0f;
  else if(angle<-PI) angle+=PI*2.0f;

  act_advance_0[0]=act_advance[0];
  act_advance_0[1]=act_advance[1];
  
  // integrate
  r[0]+=mov*sin(yaw);
  r[1]+=mov*cos(yaw);

  //differnitaite
  //float raw_speed;
  if(dt) {
    // LPF
    //raw_speed = mov/(float)dt*1000.0f;
    //speed = (int16_t)((float)speed - ((float)speed - raw_speed)*0.5f);
    int16_t raw_speed=(int16_t)(mov/(float)dt*1000.0f);
    int16_t sum_speed=0;
    for(int i=0; i<SPEED_R_SZ-1; i++) {
      speed_r[i]=speed_r[i+1];
      sum_speed+=speed_r[i];
    }
    speed_r[SPEED_R_SZ-1]=raw_speed;
    sum_speed+=raw_speed;
    speed=sum_speed/SPEED_R_SZ;
  }

  getActPower();
  getSensors();

  //if(targ_pow[0] && targ_pow[1]) {
  
  if(targ_speed) {

    Serial.print(F("PCNT=")); Serial.print(pid_cnt); 
    Serial.print(F("\t\t ADV=")); Serial.print(act_advance[0]); Serial.print(F("\t ")); Serial.print(act_advance[1]);
    Serial.print(F("\t\t V=")); Serial.println(speed); 

    if(pid_cnt>0) {

      // simple proortional
/*
    int16_t err_bearing_p = (int16_t)((yaw-targ_bearing)*180.0/PI);
    if(err_bearing_p>180) err_bearing_p-=360;
    else if(err_bearing_p<-180) err_bearing_p+=360;
    int16_t err_bearing_d=err_bearing_p-err_bearing_p_0;
    if(err_bearing_d>180) err_bearing_d-=360;
    else if(err_bearing_d<-180) err_bearing_d+=360;
    //err_bearing_i=err_bearing_i/2+err_bearing_p;
    err_bearing_i=err_bearing_i+err_bearing_p;
    err_bearing_p_0=err_bearing_p;
    
    int16_t s=-(int16_t)((int32_t)gain_p*err_bearing_p+(int32_t)gain_d*err_bearing_d+(int32_t)gain_i*err_bearing_i)/gain_div;
*/    

      float err_bearing_p = (yaw-targ_bearing)*180.0f/PI;
      if(err_bearing_p>180.0f) err_bearing_p-=360.0f;
      else if(err_bearing_p<-180.0f) err_bearing_p+=360.0f;
      float err_bearing_d=err_bearing_p-err_bearing_p_0;
      if(err_bearing_d>180.0f) err_bearing_d-=360.0f;
      else if(err_bearing_d<-180.0f) err_bearing_d+=360.0f;
      err_bearing_i=err_bearing_i+err_bearing_p;
      if(err_bearing_i>limit_i) err_bearing_i=limit_i;
      if(err_bearing_i<-limit_i) err_bearing_i=-limit_i;
      err_bearing_p_0=err_bearing_p;

      
      int16_t s=-(int16_t)((err_bearing_p*gain_p+err_bearing_d*gain_d+err_bearing_i*gain_i)/gain_div);
      int16_t ss=0;
      
      if(pid_cnt>4) {
        float err_speed_p = speed - targ_speed;
        float err_speed_d=err_speed_p-err_speed_p_0;
        err_speed_i=err_speed_i+err_speed_p;      
        if(err_speed_i>limit_i/5) err_speed_i=limit_i/5;
        if(err_speed_i<-limit_i/5) err_speed_i=-limit_i/5;
        err_speed_p_0=err_speed_p;    
        ss=-(int16_t)((err_speed_p*gain_p+err_speed_d*gain_d/2+err_speed_i*gain_i)/gain_div/10);
        //Serial.print(F("V=")); Serial.print(speed); Serial.print(F("\t : VT=")); Serial.print(targ_speed); 
        Serial.print(F("VErr: ")); Serial.print(err_speed_p); Serial.print(F("\t ")); Serial.print(err_speed_d);  Serial.print(F("\t ")); Serial.print(err_speed_i);
        Serial.print(F("\t => ")); Serial.println(ss);

      }
    
      cur_pow[0]+=s;
      cur_pow[1]-=s;

      cur_pow[0]+=ss;
      cur_pow[1]+=ss;
    
      for(int i=0; i<2; i++) {
        if(cur_pow[i]<M_POW_MIN) cur_pow[i]=M_POW_MIN; 
        if(cur_pow[i]>M_POW_MAX) cur_pow[i]=M_POW_MAX; 
      // maybe a better idea would be to make limits proportional to the target?
      }
    
      setPower(cur_pow);      
      raiseFail(CTL_LOG_PID, round(err_bearing_p), round(err_bearing_d), round(err_bearing_i), s, cur_pow[0], cur_pow[1]);

      yield();

      Serial.print(F("BErr: ")); Serial.print(err_bearing_p); Serial.print(F("\t ")); Serial.print(err_bearing_d);  Serial.print(F("\t ")); Serial.print(err_bearing_i);
      Serial.print(F("\t => ")); Serial.print(s); Serial.print(F("\t : ")); Serial.print(cur_pow[0]); Serial.print(F("\t : ")); Serial.println(cur_pow[1]); 

      yield();
    
    } 
    pid_cnt++;    
    
  }
   
/*
  Serial.print(F("CTRL: ")); Serial.print(mov/10.0f); Serial.print(F(" \t ")); Serial.print(yaw);
  Serial.print(F(" \t ")); Serial.print(r[0]/10.0f); Serial.print(F(" \t ")); Serial.print(r[1]/10.0f);   
  Serial.println();
*/

  return true;
}

//int16_t *Controller::getTargPower() { return targ_pow;}
int16_t *Controller::getCurPower() { return cur_pow;}
uint8_t Controller::getNumSensors() { return nsens;}
float *Controller::getStoredRotRate() { return act_rot_rate;}
int32_t *Controller::getStoredAdvance() { return act_advance;}
int16_t *Controller::getStoredPower() { return act_power;}
int16_t *Controller::getStoredSensors() { return sensors;}
float Controller::getMovement() { return mov;}
float Controller::getRotation() { return rot;}
float Controller::getDistance() { return dist*0.1f;} //cm
int16_t Controller::getSpeed() { return speed/10;} // cm/s
float Controller::getAngle() { return angle;}
//float *Controller::getCoords() { return r;}
float Controller::getX() { return r[0]*0.1f;}
float Controller::getY() { return r[1]*0.1f;}

/*
bool Controller::setTargRotRate(float l, float r) {
  int16_t d[2];
  d[0]=(int16_t)(l*V_NORM);
  d[1]=(int16_t)(r*V_NORM);
  // init steering parameters
  setTargSteering(0);
  //err_bearing_p_0=0.0f;
  //err_bearing_i=0.0f;
  err_bearing_p_0=err_bearing_i=0;  
  if(targ_rot_rate[0]==d[0] && targ_rot_rate[1]==d[1]) return true;
  //if(!setTargRotRate(d)) return false;
  targ_pow[0]=l*120; targ_pow[1]=r*120; // temp  
  if(!setPower(targ_pow)) return false;
  targ_rot_rate[0]=d[0];
  targ_rot_rate[1]=d[1];
  return true;
}
*/

bool Controller::setTargPower(float l, float r) {
  // init steering parameters
  //if(targ_rot_rate[0]==d[0] && targ_rot_rate[1]==d[1]) return true;
  setTargSteering(0);
  err_bearing_p_0=err_bearing_i=0;    
  /*
  targ_pow[0]=l*M_POW_NORM; targ_pow[1]=r*M_POW_NORM; // temp  
  if(!setPower(targ_pow)) return false;
  */
  cur_pow[0]=l*M_POW_NORM; cur_pow[1]=r*M_POW_NORM; // temp  
  targ_speed=(l+r)*0.5f*M_SPEED_NORM;
  pid_cnt=0;
    
  Serial.print(F("STP TV=")); Serial.print(targ_speed); Serial.print(F("ADV=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);
  if(!setPower(cur_pow)) return false;
  return true;
}

bool Controller::setTargSteering(int16_t s) {
  targ_bearing = curr_yaw+(float)s/180.0*PI;
  if(targ_bearing>PI) targ_bearing-=PI*2.0f;
  else if(targ_bearing<-PI) targ_bearing+=PI*2.0f;  
  return true;
}

bool Controller::setTargSpeed(int16_t speed) {
  setTargSteering(0);
  err_bearing_p_0=err_bearing_i=0;    
  targ_speed=speed*10; //mm
  cur_pow[0]=cur_pow[1]=(int32_t)targ_speed*M_POW_NORM/M_SPEED_NORM; // temp  
  pid_cnt=0;
  
  Serial.print(F("STV TV=")); Serial.print(targ_speed); Serial.print(F("ADV=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);
  if(!setPower(cur_pow)) return false;
  return true;
}

/*
bool Controller::getControllerStatus() { 
  bool res = I2Cdev::readBytes(DEV_ID, REG_STATUS, 2, sta); 
  if(!res) raiseFail(CTL_FAIL_RD, REG_STATUS);
  return res;
}
*/

uint8_t Controller::_getNumSensors() {
  bool res = I2Cdev::readByte(DEV_ID, REG_SENSORS_CNT, buf); 
  if(!res) {
    //fail_reason=CTL_FAIL_RD;
    raiseFail(CTL_FAIL_RD, REG_SENSORS_CNT);
    return 0;
  }
  if(buf[0]>8) buf[0]=8;   
  return buf[0];
}

bool Controller::setStart(uint8_t p) {
  bool res = I2Cdev::writeByte(DEV_ID, REG_START, p);
  if(!res) raiseFail(CTL_FAIL_WRT, REG_START);
  return res;
}

/*
bool Controller::getTargRotRate(int16_t *d) { 
  bool res = readInt16_2(REG_TARG_ROT_RATE, d, d+1); 
  if(!res) raiseFail(CTL_FAIL_RD, REG_TARG_ROT_RATE);
  return res;
}

bool Controller::setTargRotRate(int16_t *d) {
  bool res = writeInt16_2(REG_TARG_ROT_RATE, d[0], d[1]); 
  if(!res) raiseFail(CTL_FAIL_WRT, REG_TARG_ROT_RATE);
  return res;
}
*/

bool Controller::stopDrive() {
  //bool res = writeInt16_2(REG_TARG_ROT_RATE, 0, 0); 
  //if(!res) raiseFail(CTL_FAIL_WRT, REG_TARG_ROT_RATE);
  bool res = writeInt16_2(REG_TARG_POW, 0, 0); 
  if(!res) raiseFail(CTL_FAIL_WRT, REG_TARG_POW);  
  return res;
}

bool Controller::setPower(int16_t *p) {
  bool res = writeInt16_2(REG_TARG_POW, p[0], p[1]); 
  if(!res) raiseFail(CTL_FAIL_WRT, REG_TARG_POW);
  return res;
}

bool Controller::setSteering(int16_t s) {
  bool res = writeInt16(REG_STEERING, s); 
  if(!res) raiseFail(CTL_FAIL_WRT, REG_STEERING);
  return res;
}

bool Controller::getActRotRate() {
  int16_t tmp[2];
  bool res = readInt16_2(REG_ACT_ROT_RATE, tmp, tmp+1);
  if(!res) {
    raiseFail(CTL_FAIL_RD, REG_ACT_ROT_RATE);
    return false;
  }
  act_rot_rate[0]=(float)tmp[0]/V_NORM;
  act_rot_rate[1]=(float)tmp[1]/V_NORM;
  return res;
}

bool Controller::getActAdvance(/*int16_t *d*/) {
  //bool res = readInt16_2(REG_ACT_ADV_ACC, act_advance, act_advance+1);
  bool res = readInt32_2(REG_ACT_ADV_ACC, act_advance);  
  if(!res) raiseFail(CTL_FAIL_RD, REG_ACT_ADV_ACC, buf[4]);
  return res;
}

bool Controller::getActPower(/*int16_t *d*/) {
  bool res = readInt16_2(REG_ACT_POW, act_power, act_power+1); 
  if(!res) raiseFail(CTL_FAIL_RD, REG_ACT_POW);
  return res;
}

bool Controller::getSensors(/*int16_t *sens*/) {
  bool res = readInt16_N(REG_SENSORS_ALL, 8, sensors); 
  if(!res) raiseFail(CTL_FAIL_RD, REG_SENSORS_ALL);
  return res;
}

bool Controller::writeInt16(uint16_t reg, int16_t v) {
    buf[0] = (uint8_t)(v>>8);
    buf[1] = (uint8_t)(v&0xFF);   
    bool res = I2Cdev::writeBytes(DEV_ID, reg, 2, buf);
    return res;
}


bool Controller::writeInt16_2(uint16_t reg, int16_t left, int16_t right) {
    buf[0] = (uint8_t)(left>>8);
    buf[1] = (uint8_t)(left&0xFF);   
    buf[2] = (uint8_t)(right>>8);
    buf[3] = (uint8_t)(right&0xFF);   
    bool res = I2Cdev::writeBytes(DEV_ID, reg, 4, buf);
    return res;
}

bool Controller::readInt16_2(uint16_t reg, int16_t *left, int16_t *right) {
    bool res = I2Cdev::readBytes(DEV_ID, reg, 4, buf);
    if(!res) return false;
    /*
    *left = (((int16_t)buf[0]) << 8) | buf[1];
    *right = (((int16_t)buf[2]) << 8) | buf[3];
    */
     
    *left = (int16_t)((((uint16_t)buf[0]) << 8) | buf[1]);
    *right = (int16_t)((((uint16_t)buf[2]) << 8) | buf[3]);
  
    return res;
}

bool Controller::readInt32_2(uint16_t reg, int32_t *d) {
    bool res = I2Cdev::readBytes(DEV_ID, reg, 8, buf);
    if(!res) return false;
    d[0] = (int32_t)(
    (((int32_t)buf[0]) << 24) |
    (((int32_t)buf[1]) << 16) |
    (((int32_t)(buf[2])) << 8) |
    buf[3]);
    d[1] = (int32_t)(
    (((int32_t)buf[4]) << 24) |
    (((int32_t)buf[5]) << 16) |
    (((int32_t)(buf[6])) << 8) |
    buf[7]);
    return res;
}

/*
bool Controller::readInt16_2_x(uint16_t reg, int16_t *left, int16_t *right) {
    bool res = I2Cdev::readBytes(DEV_ID, reg, 5, buf);
    if(buf[4] != M_MAGIC_ID) return false;
    *left = (((int16_t)buf[0]) << 8) | buf[1];
    *right = (((int16_t)buf[2]) << 8) | buf[3];
    return res;
}
*/

bool Controller::readInt16_N(uint16_t reg, uint16_t n, int16_t *d) {
    bool res = I2Cdev::readBytes(DEV_ID, reg, n*2, buf);
    if(!res) return false;    
    for(uint16_t i=0, j=0; i<n; i++) {
      //*(d+i) = (((int16_t)buf[j++]) << 8) | buf[j++]; 
      int16_t t=((int16_t)buf[j++]) << 8;
      t |=  buf[j++];
      *(d+i) = t;      
    }
    return res;
}

  
