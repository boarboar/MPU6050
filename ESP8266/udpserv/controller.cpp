#include <Arduino.h>
#include "I2Cdev.h"
#include "controller.h"

const int DEV_ID=4;

Controller Controller::ControllerProc ; // singleton

Controller::Controller() : pready(false), nsens(0), act_rot_rate{0},act_advance{0},act_power{0},sensors{0} {
  }

uint8_t Controller::getStatus() { return pready; }
uint8_t Controller::isDataReady() { return pready && data_ready; }
uint8_t Controller::isNeedReset() { return need_reset; }
void    Controller::needReset() {  need_reset=true; }
uint8_t Controller::getFailReason() { return fail_reason; }
void  Controller::clearFailReason() { fail_reason=CTL_FAIL_NONE; }

void Controller::raiseFail(uint8_t reason, int16_t p1, int16_t p2, int16_t p3, int16_t p4) {
  fail_reason=reason;
  fail_p[0]=p1;
  fail_p[1]=p2;
  fail_p[2]=p3;
  fail_p[3]=p4;

  Serial.print(F("CTRL ALR: ")); Serial.print(reason); 
  for(int i=0; i<4; i++) {
    Serial.print(F(" \t ")); Serial.print(fail_p[i]); 
  }   
  Serial.println();
}

void Controller::getFailParams(int16_t npa, int16_t *pa) {
  if(npa>4) npa=4;
  for(int i=0; i<npa; i++) pa[i]=fail_p[i]; 
 } 

bool Controller::init() {
  pready=false;
  data_ready=0;
  fail_reason=0;
  need_reset=false; 
  nsens=0;
  targ_rot_rate[0]=targ_rot_rate[1]=0;
  act_rot_rate[0]=act_rot_rate[1]=0;
  act_advance[0]=act_advance[1]=0;
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
  return pready;
}

void Controller::resetIntegrator() {
  dist=angle=0.0f;
  r[0]=r[1]=0.0f;
  curr_yaw=0;
  targ_bearing=0;  
  //err_bearing_p_0=0.0f;
  //err_bearing_i=0.0f;
  err_bearing_p_0=err_bearing_i=0;
  
}

uint8_t Controller::testConnection() {
  bool res = I2Cdev::readByte(DEV_ID, REG_WHO_AM_I, buf); 
  if(!res) return 0;
  return buf[0]==M_OWN_ID;
}

bool Controller::process(float yaw) {
  if(!pready) return false;
  //boolean alr=false;
  //int16_t tmp[2];

  curr_yaw=yaw;
  
  if(!getControllerStatus()) return false;

 //Serial.print(F("CTRL STAT: ")); Serial.print(sta[0]); Serial.print(F(" \t ")); Serial.println(sta[1]);

  if(!getActAdvance()) return false;

  if(sta[1]) { // experimental
    // Serial.print(F("CTRL STAT ALR: ")); Serial.print(sta[0]); Serial.print(F(" \t ")); Serial.println(sta[1]);
    raiseFail(CTL_FAIL_ALR, sta[0], sta[1], 0, 0);
    return false;
  }

  if(abs(act_advance[0])>512 || abs(act_advance[1])>512) {
    //raiseFail(CTL_FAIL_OVF, act_advance[0], act_advance[1]);
    raiseFail(CTL_FAIL_OVF, buf[0], buf[1], buf[2], buf[3]);
    return false;
  }
  
  mov=(float)(act_advance[0]+act_advance[1])*0.5f; // in mm
  rot=(float)(act_advance[0]-act_advance[1])/(float)WHEEL_BASE_MM;
  dist+=mov;
  angle+=rot;
  
  if(angle>PI) angle-=PI*2.0f;
  else if(angle<-PI) angle+=PI*2.0f;

  // integrate
  r[0]+=mov*sin(yaw);
  r[1]+=mov*cos(yaw);

/*
  if(!getActRotRate()) return false;  
  if(!getActPower()) return false;
  if(!getSensors()) return false; 
  */

  getActRotRate();
  getActPower();
  getSensors();

  if(targ_rot_rate[0] && targ_rot_rate[1]) {
    // simple proortional
    /*
    float err_bearing_p = yaw-targ_bearing;
    if(err_bearing_p>PI) err_bearing_p-=PI*2.0f;
    else if(err_bearing_p<-PI) err_bearing_p+=PI*2.0f;
    float err_bearing_d=err_bearing_p-err_bearing_p_0;
    if(err_bearing_d>PI) err_bearing_d-=PI*2.0f;
    else if(err_bearing_d<-PI) err_bearing_d+=PI*2.0f;
    err_bearing_i=err_bearing_i*0.5+err_bearing_p;
    err_bearing_p_0=err_bearing_p;
    
    int16_t s=-err_bearing_p*180.0/PI;
    */

    int16_t err_bearing_p = (int16_t)((yaw-targ_bearing)*180.0/PI);
    if(err_bearing_p>180) err_bearing_p-=360;
    else if(err_bearing_p<-180) err_bearing_p+=360;
    int16_t err_bearing_d=err_bearing_p-err_bearing_p_0;
    if(err_bearing_d>180) err_bearing_d-=360;
    else if(err_bearing_d<-180) err_bearing_d+=360;
    err_bearing_i=err_bearing_i/2+err_bearing_p;
    err_bearing_p_0=err_bearing_p;
    const int gain_p=20;
    const int gain_d=80;
    const int gain_i=1;
    const int gain_div=10;
    
    int16_t s=-(int16_t)((int32_t)gain_p*err_bearing_p+(int32_t)gain_d*err_bearing_p+(int32_t)gain_i*err_bearing_p)/gain_div;

    raiseFail(CTL_LOG_PID, err_bearing_p, err_bearing_d, err_bearing_i, s);
    
    Serial.print(F("Bearing error: ")); Serial.print(err_bearing_p); Serial.print(F("\t ")); Serial.print(err_bearing_d);  Serial.print(F("\t ")); Serial.print(err_bearing_i);
    Serial.print(F("\t => ")); Serial.println(s);    
    setSteering(s);  
  }
   
/*
  Serial.print(F("CTRL: ")); Serial.print(mov/10.0f); Serial.print(F(" \t ")); Serial.print(yaw);
  Serial.print(F(" \t ")); Serial.print(r[0]/10.0f); Serial.print(F(" \t ")); Serial.print(r[1]/10.0f);   
  Serial.println();
*/

  return true;
}

uint8_t Controller::getNumSensors() { return nsens;}
float *Controller::getStoredRotRate() { return act_rot_rate;}
int16_t *Controller::getStoredAdvance() { return act_advance;}
int16_t *Controller::getStoredPower() { return act_power;}
int16_t *Controller::getStoredSensors() { return sensors;}
float Controller::getMovement() { return mov;}
float Controller::getRotation() { return rot;}
float Controller::getDistance() { return dist*0.1f;}
float Controller::getAngle() { return angle;}
//float *Controller::getCoords() { return r;}
float Controller::getX() { return r[0]*0.1f;}
float Controller::getY() { return r[1]*0.1f;}

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
  if(!setTargRotRate(d)) return false;
  targ_rot_rate[0]=d[0];
  targ_rot_rate[1]=d[1];
  return true;
}

bool Controller::setTargSteering(int16_t s) {
  targ_bearing = curr_yaw+(float)s/180.0*PI;
  if(targ_bearing>PI) targ_bearing-=PI*2.0f;
  else if(targ_bearing<-PI) targ_bearing+=PI*2.0f;  
  return true;
}

bool Controller::getControllerStatus() { 
  bool res = I2Cdev::readBytes(DEV_ID, REG_STATUS, 2, sta); 
  if(!res) raiseFail(CTL_FAIL_RD, REG_STATUS);
  return res;
}

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

bool Controller::stopDrive() {
  bool res = writeInt16_2(REG_TARG_ROT_RATE, 0, 0); 
  if(!res) raiseFail(CTL_FAIL_WRT, REG_TARG_ROT_RATE);
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
  bool res = readInt16_2(REG_ACT_ADV_ACC, act_advance, act_advance+1);  
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


bool Controller::readInt16_2_x(uint16_t reg, int16_t *left, int16_t *right) {
    bool res = I2Cdev::readBytes(DEV_ID, reg, 5, buf);
    if(buf[4] != M_MAGIC_ID) return false;
    *left = (((int16_t)buf[0]) << 8) | buf[1];
    *right = (((int16_t)buf[2]) << 8) | buf[3];
    return res;
}

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

  
