#include <Arduino.h>
#include "I2Cdev.h"
#include "controller.h"

const int DEV_ID=4;

Controller Controller::ControllerProc ; // singleton

Controller::Controller() : pready(false), nsens(0), act_rot_rate{0},act_advance{0},sensors{0} {
  }

//int8_t Controller::getStatus() { return pready; }
uint8_t Controller::isDataReady() { return pready && data_ready; }
uint8_t Controller::isNeedReset() { return need_reset; }
void    Controller::needReset() {  need_reset=true; }
uint8_t Controller::getFailReason() { return fail_reason; }
void  Controller::clearFailReason() { fail_reason=CTL_FAIL_NONE; }

bool Controller::init() {
  pready=false;
  data_ready=0;
  fail_reason=0;
  need_reset=false; 
  nsens=0;
  act_rot_rate[0]=act_rot_rate[1]=0;
  act_advance[0]=act_advance[1]=0;
  //sensors={0};
  resetIntegrator();
  pready=testConnection();
  if(pready) {
    fail_reason=CTL_FAIL_NONE;
    nsens=_getNumSensors();      
  } else {
    fail_reason=CTL_FAIL_INIT;
  }
  return pready;
}

void Controller::resetIntegrator() {
  dist=angle=0;
}

uint8_t Controller::testConnection() {
  bool res = I2Cdev::readByte(DEV_ID, REG_WHO_AM_I, buf); 
  if(!res) return 0;
  return buf[0]==M_OWN_ID;
}

bool Controller::process() {
  if(!pready) return false;
  int16_t tmp[2];
  getActRotRate(tmp);
  getActAdvance(act_advance);
  getSensors(sensors);
  act_rot_rate[0]=(float)tmp[0]/V_NORM;
  act_rot_rate[1]=(float)tmp[1]/V_NORM;
  mov=((float)act_advance[0]+(float)act_advance[1])/0.5f;
  rot=(float)(act_advance[0]-(float)act_advance[1])/(float)WHEEL_BASE_MM;
  dist+=mov;
  angle+=rot;

  if(angle>PI) angle-=PI*2.0f;
  else if(angle<-PI) angle+=PI*2.0f;
            
  return true;
}

uint8_t Controller::getNumSensors() { return nsens;}
float *Controller::getStoredRotRate() { return act_rot_rate;}
int16_t *Controller::getStoredAdvance() { return act_advance;}
int16_t *Controller::getStoredSensors() { return sensors;}
float Controller::getMovement() { return mov;}
float Controller::getRotation() { return rot;}
float Controller::getDistance() { return dist;}
float Controller::getAngle() { return angle;}


uint8_t Controller::_getNumSensors() {
  bool res = I2Cdev::readByte(DEV_ID, REG_SENSORS_CNT, buf); 
  if(!res) {
    fail_reason=CTL_FAIL_RD;
    return 0;
  }
  if(buf[0]>8) buf[0]=8;   
  return buf[0];
}

bool Controller::getTargRotRate(int16_t *d) { 
  bool res = readInt16_2(REG_TARG_ROT_RATE, d, d+1); 
  if(!res) fail_reason=CTL_FAIL_RD;
  return res;
}

bool Controller::setTargRotRate(int16_t *d) {
  bool res = writeInt16_2(REG_TARG_ROT_RATE, d[0], d[1]); 
  if(!res) fail_reason=CTL_FAIL_WRT;
  return res;
}

bool Controller::stopDrive() {
  bool res = writeInt16_2(REG_TARG_ROT_RATE, 0, 0); 
  if(!res) fail_reason=CTL_FAIL_WRT;
  return res;
}

bool Controller::getActRotRate(int16_t *d) {
  bool res = readInt16_2(REG_ACT_ROT_RATE, d, d+1); 
  if(!res) fail_reason=CTL_FAIL_RD;
  return res;
}

bool Controller::getActAdvance(int16_t *d) {
  bool res = readInt16_2(REG_ACT_ADV_ACC, d, d+1); 
  if(!res) fail_reason=CTL_FAIL_RD;
  return res;
}

bool Controller::getSensors(int16_t *sens) {
  bool res = readInt16_N(REG_SENSORS_ALL, 8, sens); 
  if(!res) fail_reason=CTL_FAIL_RD;
  return res;
}


bool Controller::writeInt16_2(uint16_t reg, int16_t left, int16_t right) {
    buf[0] = (uint8_t)(left>>8);
    buf[1] = (uint8_t)(left&0xFF);   
    buf[2] = (uint8_t)(right>>8);
    buf[3] = (uint8_t)(right&0xFF);   
    bool res = I2Cdev::writeBytes(DEV_ID, reg, 4, buf);
    //Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.println("ms");
    return res;
}

bool Controller::readInt16_2(uint16_t reg, int16_t *left, int16_t *right) {
    //Serial.print("Requesting...\t ");
    //t=millis();  
    bool res = I2Cdev::readBytes(DEV_ID, reg, 4, buf);
    //dt=millis()-t;
    //Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.print("ms\t ");
    *left = (((int16_t)buf[0]) << 8) | buf[1];
    *right = (((int16_t)buf[2]) << 8) | buf[3];
    //Serial.print(*left); Serial.print("\t "); Serial.println(*right);
    return res;
}

bool Controller::readInt16_N(uint16_t reg, uint16_t n, int16_t *d) {
    //Serial.print("Requesting...\t ");
    //t=millis();  
    bool res = I2Cdev::readBytes(DEV_ID, reg, n*2, buf);
    //dt=millis()-t;
    //Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.print("ms\t ");
    for(uint16_t i=0, j=0; i<n; i++) {
      *(d+i) = (((int16_t)buf[j++]) << 8) | buf[j++];
      //Serial.print(d[i]); Serial.print("\t ");
    }
    return res;
}

  
