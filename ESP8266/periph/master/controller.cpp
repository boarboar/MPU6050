#include <Arduino.h>
#include "I2Cdev.h"
#include "controller.h"

const int DEV_ID=4;

Controller Controller::ControllerProc ; // singleton

Controller::Controller() {;}

bool Controller::testConnection() {
  bool res = I2Cdev::readByte(DEV_ID, REG_WHO_AM_I, buf); 
  if(!res) return 0;
  return buf[0]==M_OWN_ID;
}

uint8_t Controller::getNumSensors() {
  bool res = I2Cdev::readByte(DEV_ID, REG_SENSORS_CNT, buf); 
  if(!res) return 0;
  if(buf[0]>8) buf[0]=8;   
  return buf[0];
}

bool Controller::getTargRotRate(int16_t *d) {
  bool res = readInt16_2(REG_TARG_ROT_RATE, d); 
  return res;
}

bool Controller::setTargRotRate(int16_t *d) {
  bool res = writeInt16_2(REG_TARG_ROT_RATE, d); 
  return res;
}

bool Controller::getActRotRate(int16_t *d) {
  bool res = readInt16_2(REG_ACT_ROT_RATE, d); 
  return res;
}

bool Controller::getActAdvance(int16_t *d) {
  bool res = readInt16_2(REG_ACT_ADV_ACC, d); 
  return res;
}

bool Controller::getSensors(int16_t *sens) {
  bool res = readInt16_N(REG_SENSORS_ALL, 8, sens); 
  return res;
}

bool Controller::writeInt16_2(uint16_t reg, int16_t *d) {
    int16_t left=d[0];
    int16_t right=d[1];
    buf[0] = (uint8_t)(left>>8);
    buf[1] = (uint8_t)(left&0xFF);   
    buf[2] = (uint8_t)(right>>8);
    buf[3] = (uint8_t)(right&0xFF);   
    bool res = I2Cdev::writeBytes(DEV_ID, reg, 4, buf);
    //Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.println("ms");
    return res;
}

bool Controller::readInt16_2(uint16_t reg, int16_t *d) {
    //Serial.print("Requesting...\t ");
    //t=millis();  
    bool res = I2Cdev::readBytes(DEV_ID, reg, 4, buf);
    //dt=millis()-t;
    //Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.print("ms\t ");
    *d = (((int16_t)buf[0]) << 8) | buf[1];
    *(d+1) = (((int16_t)buf[2]) << 8) | buf[3];
    //Serial.print(d[0]); Serial.print("\t "); Serial.println(d[1]);
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

  
