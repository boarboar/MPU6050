#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "I2Cdev.h"
#include "controller.h"
#include "cfg.h"
#include "logger.h"

const int DEV_ID=4;
const int M_POW_MIN=30; 
const int M_POW_MAX=200;
const int M_POW_NORM=100;
const int M_SPEED_NORM=200;

/*
const int gain_p=20;
const int gain_d=320;
const int gain_i=10;
const int gain_div=80;
const int limit_i=100;
*/
    
Controller Controller::ControllerProc ; // singleton

Controller::Controller() : pready(false), nsens(0), act_advance{0},act_power{0},sensors{0} {
  }

uint8_t Controller::getStatus() { return pready; }
uint8_t Controller::isDataReady() { return pready && data_ready; }
uint8_t Controller::isNeedReset() { return need_reset; }
void    Controller::needReset() {  need_reset=true; }


bool Controller::init() {
  pready=false;
  data_ready=0;
  //fail_reason=0;
  need_reset=false; 
  nsens=0;
  //act_rot_rate[0]=act_rot_rate[1]=0;
  act_advance[0]=act_advance[1]=0;
  act_power[0]=act_power[1]=0;
  base_pow=0;
  delta_pow=0;
  targ_speed=0;
  rot_speed=0;
  for(int i=0; i<SENS_SIZE; i++) sensors[i]=0;
  resetIntegrator();
  pready=testConnection();
  if(pready) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_EVENT, CTL_FAIL_INIT, "CTL_INT_OK");  
    nsens=_getNumSensors();      
  } else {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_INIT, "CTL_INT_FL");  
    need_reset=true;
  }

  return pready;
}

void Controller::resetIntegrator() {
  dist=0.0f;
  //angle=0.0f;
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
  qsum_err=0;

  Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PBPID, CfgDrv::Cfg.bear_pid.gain_p, CfgDrv::Cfg.bear_pid.gain_d, CfgDrv::Cfg.bear_pid.gain_i, CfgDrv::Cfg.bear_pid.gain_div, CfgDrv::Cfg.bear_pid.limit_i);  
  
  Serial.print(F("CTRL INIT PID B: ")); Serial.print(CfgDrv::Cfg.bear_pid.gain_p); Serial.print(F(" \t ")); Serial.print(CfgDrv::Cfg.bear_pid.gain_d); Serial.print(F(" \t ")); 
  Serial.print(CfgDrv::Cfg.bear_pid.gain_i); Serial.print(F(" \t ")); Serial.print(CfgDrv::Cfg.bear_pid.gain_div); Serial.print(F(" \t ")); Serial.println(CfgDrv::Cfg.bear_pid.limit_i);
  Serial.print(F("CTRL INIT PID S: ")); Serial.print(CfgDrv::Cfg.speed_pid.gain_p); Serial.print(F(" \t ")); Serial.print(CfgDrv::Cfg.speed_pid.gain_d); Serial.print(F(" \t ")); 
  Serial.print(CfgDrv::Cfg.speed_pid.gain_i); Serial.print(F(" \t ")); Serial.print(CfgDrv::Cfg.speed_pid.gain_div); Serial.print(F(" \t ")); Serial.println(CfgDrv::Cfg.speed_pid.limit_i);
  
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
  float delta_yaw=yaw - curr_yaw;
  curr_yaw=yaw;
  if(!getActAdvance()) return false;
   
  if(abs(act_advance[0]-act_advance_0[0])>512 || abs(act_advance[1]-act_advance_0[1])>512) {
    
        Serial.print(F("! ADV=")); 
        Serial.print(act_advance[0]); Serial.print(F("\t ")); Serial.println(act_advance[1]);
        Serial.print(F("\t ADV0=\t ")); 
        Serial.print(act_advance_0[0]); Serial.print(F("\t ")); Serial.println(act_advance_0[1]);
  
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_ALARM, CTL_FAIL_OVF, act_advance[0], act_advance[1], act_advance_0[0], act_advance_0[1]);  
  
    //act_advance_0[0]=act_advance[0];
    //act_advance_0[1]=act_advance[1];
    // RESET PERIPH ?
    act_advance[0]=act_advance_0[0];
    act_advance[1]=act_advance_0[1];
    return false;
  }

  float mov;
  float dist0=dist;
  dist=(float)(act_advance[0]+act_advance[1])*0.5f; // in mm;
  mov=dist-dist0;
  run_dist+=fabs(mov);
/*
  float rot;  
  rot=(float)((act_advance[0]-act_advance_0[0])-(act_advance[1]-act_advance_0[1]))/(float)WHEEL_BASE_MM;  
  angle+=rot;  
  if(angle>PI) angle-=PI*2.0f;
  else if(angle<-PI) angle+=PI*2.0f;
*/

  act_advance_0[0]=act_advance[0];
  act_advance_0[1]=act_advance[1];
  
  // integrate
  r[0]+=mov*sin(yaw);
  r[1]+=mov*cos(yaw);

  getSensors();
  //getActPower();

  if(!dt) return true;
  
  
  {
    //differnitaite
    // LPF
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

  
  if((targ_speed || rot_speed)) { // movement

    //Serial.print(F("PCNT=")); Serial.print(pid_cnt);  Serial.print(F("\t TRG=")); Serial.print(targ_bearing);  Serial.print(F("\t YAW=")); Serial.println(yaw);  
    /*
    Serial.print(F("PCNT=")); Serial.print(pid_cnt);  Serial.print(F("\t TRG=")); Serial.print(targ_bearing); 
    Serial.print(F("\t ADV=")); Serial.print(act_advance[0]); Serial.print(F("\t ")); Serial.print(act_advance[1]);
    Serial.print(F("\t V=")); Serial.println(speed); 
*/
    if(pid_cnt>0) {
      
      int16_t limit=CfgDrv::Cfg.bear_pid.limit_i;
      int16_t err_bearing_p, err_bearing_d;

      //float err_bearing_p, err_bearing_d;
      err_bearing_p = (int16_t)((yaw-targ_bearing)*180.0f/PI);
      if(err_bearing_p>180) err_bearing_p-=360;
      else if(err_bearing_p<-180) err_bearing_p+=360;
      
      if(targ_speed) { // straight 
        if(targ_speed<0) err_bearing_p=-err_bearing_p;               
        err_bearing_i=err_bearing_i+err_bearing_p;
        // note: it should rather be +err_bearing_p*dt; 
        // or if normed to 100ms: (int32_t)(err_bearing_p)*dt/100;
        if(err_bearing_i>limit) err_bearing_i=limit;
        if(err_bearing_i<-limit) err_bearing_i=-limit;      
        if(run_dist>=100) //100mm
          qsum_err+=err_bearing_p*err_bearing_p;
      } else { // rot
        if((err_bearing_p<0 && rot_speed<0) || (err_bearing_p>0 && rot_speed>0)) { 
          rot_speed=-rot_speed;
          Serial.print(F("<< ROT >>"));  
          Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PID+1, rot_speed);     
        }
        if(err_bearing_p<0) err_bearing_p=-err_bearing_p;        
        if(err_bearing_p<5) { // at the moment - 5 degrees
          rot_speed=0;
          Serial.print(F("!! ROT !!"));     
          Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PID+2);     
        }        

      }
      
      err_bearing_d=err_bearing_p-err_bearing_p_0;      
      if(err_bearing_d>180) err_bearing_d-=360;
      else if(err_bearing_d<-180) err_bearing_d+=360;
      // note: it should rather be (err_bearing_p-err_bearing_p_0)/dt; 
      // or if normed to 100ms: (int32_t)(err_bearing_p-err_bearing_p_0)*100/dt;
        

      err_bearing_p_0=err_bearing_p;
      // use 32 bit math?
      delta_pow=-(int16_t)((err_bearing_p*CfgDrv::Cfg.bear_pid.gain_p+err_bearing_d*CfgDrv::Cfg.bear_pid.gain_d+err_bearing_i*CfgDrv::Cfg.bear_pid.gain_i)/CfgDrv::Cfg.bear_pid.gain_div);
      
      int16_t cur_pow[2];
      if(targ_speed) {
        cur_pow[0]=base_pow+delta_pow;
        cur_pow[1]=base_pow-delta_pow;

        /*
      int16_t ss=0;
      
      if(pid_cnt>=(SPEED_R_SZ*3/2)) {
        limit=CfgDrv::Cfg.speed_pid.limit_i;
        float err_speed_p = speed - targ_speed;
        float err_speed_d=err_speed_p-err_speed_p_0;
        err_speed_i=err_speed_i+err_speed_p;      
        if(err_speed_i>limit) err_speed_i=limit;
        if(err_speed_i<-limit) err_speed_i=-limit;
        err_speed_p_0=err_speed_p;    
        ss=-(int16_t)((err_speed_p*CfgDrv::Cfg.speed_pid.gain_p+err_speed_d*CfgDrv::Cfg.speed_pid.gain_d+err_speed_i*CfgDrv::Cfg.speed_pid.gain_i)/CfgDrv::Cfg.speed_pid.gain_div);
        //Serial.print(F("V=")); Serial.print(speed); Serial.print(F("\t : VT=")); Serial.print(targ_speed); 
        Serial.print(F("VErr: ")); Serial.print(err_speed_p); Serial.print(F("\t ")); Serial.print(err_speed_d);  Serial.print(F("\t ")); Serial.print(err_speed_i);
        Serial.print(F("\t => ")); Serial.println(ss);

      }
    
      cur_pow[0]+=ss;
      cur_pow[1]+=ss;
  */
  
      } else {
        cur_pow[0]=base_pow-delta_pow;
        cur_pow[1]=base_pow-delta_pow;
        /*
        float a=targ_bearing-curr_yaw;
        if(a>PI) a-=PI*2.0f;
        else if(a<-PI) a+=PI*2.0f;    
        if((a>0.01 && rot_speed<0) || (a<-0.01 && rot_speed>0)) { 
          rot_speed=-rot_speed;
          Serial.print(F("<< ROT >>"));     
        }
        if(fabs(a)<0.02) {
          rot_speed=0;
          Serial.print(F("!! ROT !!"));     
        }
        */
      }
      
      // what if reverse or rot ???????
      for(int i=0; i<2; i++) {
        if(cur_pow[i]<M_POW_MIN) cur_pow[i]=M_POW_MIN; 
        if(cur_pow[i]>M_POW_MAX) cur_pow[i]=M_POW_MAX; 
      // maybe a better idea would be to make limits proportional to the target?
      }
      if(targ_speed)
        setPowerStraight(targ_speed, cur_pow);      
      else {
        if(!rot_speed) startRotate(0); // stop rotate
        else setPowerRotate(rot_speed, cur_pow);      
      }
      //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PID, dt, round(err_bearing_p), round(err_bearing_d), round(err_bearing_i), s, cur_pow[0], cur_pow[1]);  
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PID, dt, err_bearing_p, err_bearing_d, err_bearing_i, delta_pow, cur_pow[0], cur_pow[1]);  
 
      //yield();

      if(targ_speed) Serial.print(F("S BErr: ")); 
      else Serial.print(F("R BErr: ")); 
      Serial.print(err_bearing_p); Serial.print(F("\t ")); Serial.print(err_bearing_d);  Serial.print(F("\t ")); Serial.print(err_bearing_i);
      Serial.print(F("\t => ")); Serial.print(delta_pow); Serial.print(F("\t : ")); Serial.print(cur_pow[0]); Serial.print(F("\t : ")); Serial.println(cur_pow[1]); 

      yield();
    
    } // pid_cnt
    else 
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL,  Logger::UMP_LOGGER_EVENT, CTL_LOG_PID, dt, err_bearing_p_0);  
      
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
int16_t Controller::getTargSpeed() { return targ_speed/10;}
//int16_t *Controller::getCurPower() { return cur_pow;}
uint8_t Controller::getNumSensors() { return nsens;}
//float *Controller::getStoredRotRate() { return act_rot_rate;}
int32_t *Controller::getStoredAdvance() { return act_advance;}
int16_t *Controller::getStoredPower() { return act_power;}
int16_t *Controller::getStoredSensors() { return sensors;}
//float Controller::getMovement() { return mov;}
//float Controller::getRotation() { return rot;}
float Controller::getDistance() { return dist*0.1f;} //cm
int16_t Controller::getSpeed() { return speed/10;} // cm/s
//float Controller::getAngle() { return angle;}
//float *Controller::getCoords() { return r;}
float Controller::getX() { return r[0]*0.1f;}
float Controller::getY() { return r[1]*0.1f;}

float Controller::getAVQErr() {
  if(!pid_cnt) return 0;
  return qsum_err/((uint32_t)pid_cnt*pid_cnt);
}

void Controller::adjustTargBearing(int16_t s, bool absolute) {
  targ_bearing = (float)s/180.0*PI;
  if(absolute) targ_bearing+=curr_yaw;
  if(targ_bearing>PI) targ_bearing-=PI*2.0f;
  else if(targ_bearing<-PI) targ_bearing+=PI*2.0f;    
}

bool Controller::setTargPower(float l, float r) {
  /*
  // init steering parameters
  //if(targ_rot_rate[0]==d[0] && targ_rot_rate[1]==d[1]) return true;
  setTargSteering(0);
  err_bearing_p_0=err_bearing_i=0;    
  
  cur_pow[0]=l*M_POW_NORM; cur_pow[1]=r*M_POW_NORM; // temp  
  targ_speed=(l+r)*0.5f*M_SPEED_NORM;
  pid_cnt=0;
    
  Serial.print(F("STP TV=")); Serial.print(targ_speed); Serial.print(F("ADV=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);

  //raiseFail(CTL_LOG_POW, 1, round(l), round(r), 0, cur_pow[0], cur_pow[1]);

  if(!setPower(cur_pow)) return false;
  return true;
  */
  return false;
}

bool Controller::setTargSteering(int16_t s) {
  adjustTargBearing(s, true);
  if(!rot_speed && !targ_speed) return startRotate(M_SPEED_NORM);
  else return true;  
}

bool Controller::setTargBearing(int16_t s) {
  adjustTargBearing(s, false);
  if(!rot_speed && !targ_speed) return startRotate(M_SPEED_NORM);
  else return true;  
}

bool Controller::setTargSpeed(int16_t tspeed) {
  if(targ_speed!=0 && tspeed==0) {
    // stop moving    
    Serial.print(F("Stop TV, AVQE=")); Serial.println(getAVQErr());     
  } else if(rot_speed!=0 && tspeed==0) {
    // stop rotating    
    rot_speed=0;
    Serial.println(F("Stop ROT")); 
  } else if(targ_speed==0 && tspeed!=0) { 
    // start moving
    pid_cnt=0;
    qsum_err=0;
    run_dist=0;
    //Serial.print(F("Start TV=")); Serial.println(tspeed);     
  } 

  //setTargSteering(0);
  adjustTargBearing(0, true);
  err_bearing_p_0=err_bearing_i=0;    
  targ_speed=tspeed*10; //mm
  base_pow=(int32_t)abs(targ_speed)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;
   
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STV TV=")); Serial.print(targ_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);
  if(!setPowerStraight(targ_speed, cur_pow)) return false;
  return true;
}


bool Controller::startRotate(int16_t tspeed) {
  float a=targ_bearing-curr_yaw;
  if(a>PI) a-=PI*2.0f;
  else if(a<-PI) a+=PI*2.0f;    
  if(a>0.01) { 
    rot_speed=tspeed;  
    Serial.println(F("Start ROT >>"));     
  }
  else if(a<-0.01) { 
    rot_speed=-tspeed;
    Serial.println(F("Start ROT <<")); 
    }
  else Serial.println(F("No ROT")); 
  err_bearing_p_0=((curr_yaw-targ_bearing)*180.0f/PI);      
  err_bearing_i=0;    
  base_pow=(int32_t)abs(rot_speed)*M_POW_NORM/M_SPEED_NORM; // temp
  delta_pow=0;  
  pid_cnt=0;    
  int16_t cur_pow[2]={base_pow, base_pow};
  //Serial.print(F("STR =")); Serial.print(rot_speed); Serial.print(F("POW=")); Serial.print(cur_pow[0]); Serial.print(F("\t ")); Serial.println(cur_pow[1]);  
  if(!setPowerRotate(rot_speed, cur_pow)) return false;    
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
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_CNT);
    return 0;
  }
  if(buf[0]>10) buf[0]=10;   
  return buf[0];
}

bool Controller::setStart(uint8_t p) {
  bool res = I2Cdev::writeByte(DEV_ID, REG_START, p);
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_START);
  return res;
}

bool Controller::setPowerStraight(int16_t dir, int16_t *p) {
  bool res=false;
  if(dir>=0) res=writeInt16_2(REG_TARG_POW, p[0], p[1]);
  else  res=writeInt16_2(REG_TARG_POW, -p[0], -p[1]);
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_TARG_POW);
  return res;
}

bool Controller::setPowerRotate(int16_t dir, int16_t *p) {
  bool res=false;
  if(dir>=0) res=writeInt16_2(REG_TARG_POW, p[0], -p[1]);
  else  res=writeInt16_2(REG_TARG_POW, -p[0], p[1]);
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_TARG_POW);
  return res;
}

/*
bool Controller::setSteering(int16_t s) {
  bool res = writeInt16(REG_STEERING, s); 
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_WRT, REG_STEERING);
  return res;
}

bool Controller::getActRotRate() {
  int16_t tmp[2];
  bool res = readInt16_2(REG_ACT_ROT_RATE, tmp, tmp+1);
  if(!res) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ACT_ROT_RATE);
    return false;
  }
  act_rot_rate[0]=(float)tmp[0]/V_NORM;
  act_rot_rate[1]=(float)tmp[1]/V_NORM;
  return res;
}
*/

bool Controller::getActAdvance() {
  bool res = readInt32_2(REG_ACT_ADV_ACC, act_advance);  
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ACT_ADV_ACC);
  return res;
}

bool Controller::getActPower() {
  bool res = readInt16_2(REG_ACT_POW, act_power, act_power+1); 
  if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_ACT_POW);
  return res;
}

bool Controller::getSensors() {
  ////bool res = readInt16_N(REG_SENSORS_ALL, 10, sensors);
  //bool res = readInt16_N(REG_SENSORS_ALL, nsens, sensors); 
  //if(!res) Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_ALL);
  bool res = readInt16_N(REG_SENSORS_1H, nsens/2, sensors); 
  if(!res) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_1H);
    return false;
  }
  res = readInt16_N(REG_SENSORS_2H, nsens/2, sensors+nsens/2); 
  if(!res) {
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_CTL, Logger::UMP_LOGGER_ALARM, CTL_FAIL_RD, REG_SENSORS_2H);
    return false;
  }
  return res;
}

bool Controller::writeInt16(uint16_t reg, int16_t v) {
    buf[0] = (uint8_t)(v>>8);
    buf[1] = (uint8_t)(v&0xFF);   
    bool res = I2Cdev::writeBytes(DEV_ID, reg, 2, buf);
    return res;
}


bool Controller::writeInt16_2(uint16_t reg, int16_t left, int16_t right) {

    //Serial.print(F("WRT=\t ")); Serial.print(left); Serial.print(F("\t ")); Serial.println(right);

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

  
