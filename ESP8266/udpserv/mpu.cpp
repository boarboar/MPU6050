#include <Arduino.h>
//#include "Wire.h"
#include "stat.h"
#include "logger.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"

//#define SCALE_A (2.0f*8192.0f) // 1g = (9.80665 m/s^2)
#define SCALE_A (8192.0f) // 1g = (9.80665 m/s^2)
#define G_FORCE 9.80665f
#define G_SCALE (G_FORCE/SCALE_A)
#define A_K 0.3f

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpStatus(ST_0) {}

int8_t MpuDrv::getStatus() { return dmpStatus; }
uint8_t MpuDrv::isDataReady() { return dmpStatus==ST_READY && data_ready; }
uint8_t MpuDrv::isNeedReset() { return need_reset; }
void    MpuDrv::needReset() {  need_reset=true; }
//uint8_t MpuDrv::getFailReason() { return fail_reason; }
//void  MpuDrv::clearFailReason() { fail_reason=MPU_FAIL_NONE; }

int16_t MpuDrv::init(uint16_t /*intrp*/) {
  return init();
}

int16_t MpuDrv::init() {
  dmpStatus=ST_0;
  data_ready=0;
  fifoCount=0;
  count=0;
  conv_count=0;
  need_reset=0;
  //fail_reason=0;
  for(int i=0; i<MPU_FAIL_CNT_SZ; i++) fail_cnt[i]=0;
  resetIntegrator();
  Serial.println(F("Init I2C dev..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Test device conn..."));
  if(!mpu.testConnection()) {
    Serial.println(F("MPU6050 conn fail"));
    need_reset=1;
    return 0;
  }
  // load and configure the DMP  
  Serial.println(F("Init DMP..."));
  yield();
  uint8_t devStatus = mpu.dmpInitialize();
  yield();  
  if (devStatus == 0) {
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
    // turn on the DMP, now that it's ready
    Serial.println(F("Enab DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //      Serial.println(F("Enab intr..."));
    //      attachInterrupt(INT_PIN, dmpDataReady, RISING);
    //mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    // enter warmup/convergence stage 
    dmpStatus=ST_WUP;
    start=millis();
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU, Logger::UMP_LOGGER_EVENT, MPU_FAIL_INIT_OK, "IMU_INT_OK");  
    Serial.print(F("DMP ok! Wait for int...FIFO sz is ")); Serial.println(packetSize);    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Init fail, code ")); Serial.println(devStatus);
    dmpStatus = ST_FAIL;
    Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_INIT_OK, "IMU_INT_FL");  
    need_reset=1;
  }
  return dmpStatus;
}

int16_t MpuDrv::cycle(uint16_t /*dt*/) {
  uint8_t i=0;
  bool settled=false;
  if (dmpStatus==ST_0 || dmpStatus==ST_FAIL) return -1;

  if(data_ready && (micros()-start)/1000000L>1) {
    // no data from MPU after 1sec
    //Serial.println(F("MPU - no data in 1s!!!"));
    Stat::StatStore.mpu_ndt_cnt++;
    //init(); // RESET MPU
    need_reset=1;
    data_ready=0;
    //fail_reason=MPU_FAIL_NODATA;
    //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_NODATA, "IMU_ND");  
    fail_cnt[MPU_FAIL_NODATA_IDX]++;
    return -10;
  }
/*
    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return ;
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    */
  uint8_t mpuIntStatus = mpu.getIntStatus();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount=0;
    Stat::StatStore.mpu_owfl_cnt++;
    //fail_reason=MPU_FAIL_FIFOOVFL;
    //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_FIFOOVFL, "IMU_OVF");  
    fail_cnt[MPU_FAIL_FIFOOVFL_IDX]++;
    //Serial.println(F("FIFO overflow!!!"));
    return -2;
  } 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)

  if (!(mpuIntStatus & 0x02) && (fifoCount < packetSize) ) return 0; // nothing to read
  
  fifoCount = mpu.getFIFOCount();
  //if(fifoCount < packetSize) return 0; // ???
  while (fifoCount < packetSize && i++<5) { fifoCount = mpu.getFIFOCount(); yield(); } 
  if(fifoCount < packetSize) {
    //Serial.println(F("FIFO wait - giveup!!!"));
    Stat::StatStore.mpu_gup_cnt++;
    //fail_reason=MPU_FAIL_FIFOTMO;
    //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_FIFOTMO, "IMU_FTMO");  
    fail_cnt[MPU_FAIL_FIFOTMO_IDX]++;
    return 0; // giveup
  }
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  //mpu.resetFIFO(); fifoCount=0; // this is in case of overflows... 
  fifoCount -= packetSize;
  if(fifoCount >0) { 
    //Serial.print(F("FIFO excess : ")); Serial.println(fifoCount);
    Stat::StatStore.mpu_exc_cnt++;
    mpu.resetFIFO();
    fifoCount=0;
    //fail_reason=MPU_FAIL_FIFOEXCESS;
    //Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_FIFOEXCESS, "IMU_XCS");  
    fail_cnt[MPU_FAIL_FIFOEXCESS_IDX]++;
    return -3;
  }   
    
  mpu.dmpGetQuaternion(q16, fifoBuffer);
  mpu.dmpGetAccel(&aa16, fifoBuffer);
    
  if(dmpStatus==ST_WUP && count%200==1) { // warmup covergence state, check every 200th reading: 1,201,401,...
    int16_t ad16[3];
    int32_t qe=0, ae=0, e;        
    ad16[0]=aa16.x-aa16_0.x; ad16[1]=aa16.y-aa16_0.y; ad16[2]=aa16.z-aa16_0.z;                   
    for(i=0; i<4; i++) {e=q16[i]-q16_0[i]; if(e<0) e=-e; if(e>qe) qe=e;}
    for(i=0; i<3; i++) {e=ad16[i]; if(e<0) e=-e; if(e>ae) ae=e;}        
        
   Serial.println((millis()-start)/1000);
   DbgPrintQInt16("\tQ16-0", q16_0);
   DbgPrintQInt16("\tQ16-1", q16);
   yield();
   DbgPrintVectorInt16("\tAcc-0\t", &aa16_0);
   DbgPrintVectorInt16("\tAcc-1\t", &aa16);
   Serial.print(F("Q16 Err:\t")); Serial.print(qe); Serial.print(F("\tA16 Err:\t")); Serial.print(ae); 
   Serial.print("\tCC:\t"); Serial.print(conv_count); Serial.print("\tT:\t"); Serial.println((millis()-start)/1000);
        
   if(qe<QUAT_INIT_TOL && ae<ACC_INIT_TOL) {
      conv_count++;
      if((millis()-start)/1000 > INIT_PERIOD_MIN && conv_count>3) settled=true;            
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_EVENT, MPU_FAIL_CONVTMO, "IMU_CVTMO");  
    } else conv_count=0;  
   if((millis()-start)/1000 > INIT_PERIOD_MAX) {
      Serial.println(F("===MPU Failed to converge, however switching to settled status...")); // TODO -?
      settled=true;
      //fail_reason=MPU_FAIL_CONVTMO;
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_CONVTMO, "IMU_CVTMO");  
    }

   for(i=0; i<4; i++) q16_0[i]=q16[i];
   aa16_0 = aa16;
        
   if(settled) {
      Serial.print(F("===MPU Converged, cnvcnt=")); Serial.println(conv_count);
      start=micros();
      dmpStatus=ST_READY;        
     }
  } // warmup

  count++; 
  
  if(dmpStatus==ST_READY) {
#ifdef IMU_USE_INTEGRATION    
    // =======actually, this is not needed if we do not use IMU accel-based integration integration
    // integrate motion
    Quaternion q, q0;
    VectorFloat ga;    
    VectorInt16 aaReal, aaWorld;
    
    float ts;
    uint32_t mcs;
    // get world frame accel (with adjustment) - needed for V-integration
    mpu.dmpGetQuaternion(&q, q16);
    mpu.dmpGetQuaternion(&q0, q16_0);
    q0=q0.getConjugate();
    q=q0.getProduct(q); // real quaternion (relative to base)
    mpu.dmpGetGravity(&ga, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa16, &ga);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);      
    //a.x=aaWorld.x*G_SCALE; a.y=aaWorld.y*G_SCALE; a.z=aaWorld.z*G_SCALE;      
    ga.x=aaWorld.x*G_SCALE; ga.y=aaWorld.y*G_SCALE; ga.z=aaWorld.z*G_SCALE;      
    if(settled) { // first time only - store base accel
      //a0=a;
      a0=ga;
      DbgPrintVectorFloat("A Base (m/s^2)\t", &a0);
    }      
    // adjust to base accel
    //a.x-=a0.x; a.y-=a0.y; a.z-=a0.z;
    ga.x-=a0.x; ga.y-=a0.y; ga.z-=a0.z;
    // low-pass filter for acceleration
    a.x = a.x - A_K * (a.x - ga.x);
    a.y = a.y - A_K * (a.y - ga.y);
    a.z = a.z - A_K * (a.z - ga.z);
    mcs=micros();
    ts=(float)(micros()-start)/1000000.0f;
    start=mcs;
    v.x+=a.x*ts; v.y+=a.y*ts; v.z+=a.z*ts;
//      r.x+=v.x*ts; r.y+=v.y*ts; r.z+=v.z*ts;
#endif
    data_ready=1; 
    return settled ? 2 : 1;
  }      
  return 10;
}

void MpuDrv::resetIntegrator() {
  a.x=a.y=a.z=0.0f;
  v.x=v.y=v.z=0.0f;  
  ypr[0]=ypr[1]=ypr[2]=1.0f;
}

void MpuDrv::process() {
  Quaternion q, q0;
  VectorFloat gravity;    
  mpu.dmpGetQuaternion(&q, q16);
  mpu.dmpGetQuaternion(&q0, q16_0);
  q0=q0.getConjugate();
  q=q0.getProduct(q); // real quaternion (relative to base)
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 

  // flush alarms
  for(int i=0; i<MPU_FAIL_CNT_SZ; i++) {
    if(fail_cnt[i]) {
      Logger::Instance.putEvent(Logger::UMP_LOGGER_MODULE_IMU,  Logger::UMP_LOGGER_ALARM, MPU_FAIL_CYCLE+i, fail_cnt[i]);  
      fail_cnt[i]=0;
    }
  }
}

float MpuDrv::getYaw() { return ypr[0]; }

void MpuDrv::getAll(float* ypr, float* af, float* vf) {        
  /*
  Quaternion q, q0;
  VectorFloat gravity;    
  mpu.dmpGetQuaternion(&q, q16);
  mpu.dmpGetQuaternion(&q0, q16_0);
  q0=q0.getConjugate();
  q=q0.getProduct(q); // real quaternion (relative to base)
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
  */
  ypr[0]=this->ypr[0]; ypr[1]=this->ypr[1]; ypr[2]=this->ypr[2];
  af[0]=a.x; af[1]=a.y; af[2]=a.z;
  vf[0]=v.x; vf[1]=v.y; vf[2]=v.z;
/*
  DbgPrintArr3Float("YPR", ypr);
  yield();
  DbgPrintArr3Float("V", vf);
  */
}  

void MpuDrv::DbgPrintVectorInt16(const char *s, VectorInt16 *v) {
  Serial.print(s); Serial.print(v->x); Serial.print("\t"); Serial.print(v->y); Serial.print("\t"); Serial.println(v->z);
}

void MpuDrv::DbgPrintQInt16(const char *s, int16_t *q) {
  Serial.print(s);
  for(uint8_t i=0; i<4; i++) {Serial.print("\t"); Serial.print(q[i]);}
  Serial.println();
}

void MpuDrv::DbgPrintVectorFloat(const char *s, VectorFloat *v) {
  Serial.print(s); Serial.print(v->x); Serial.print("\t"); Serial.print(v->y); Serial.print("\t"); Serial.println(v->z);
}

void MpuDrv::DbgPrintArr3Float(const char *s, float *q) {
  Serial.print(s);
  for(uint8_t i=0; i<3; i++) {Serial.print("\t"); Serial.print(q[i]);}
  Serial.println();
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
/*
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
*/



