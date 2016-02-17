#include <Arduino.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"

// TODO - check it out with Q 
// Add reset MPU commend

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpStatus(ST_0)/*, data_ready(0), fifoCount(0), count(0)*/ {}

uint8_t MpuDrv::getStatus() { return dmpStatus; }
uint8_t MpuDrv::isDataReady() { return data_ready; }
Quaternion& MpuDrv::getQuaternion() { return q; }
float* MpuDrv::getYPR() { return ypr; }
VectorFloat& MpuDrv::getGravity() {return gravity;}
VectorInt16& MpuDrv::getWorldAccel() {return aaWorld;}

int16_t MpuDrv::init(uint16_t sda, uint16_t sdl, uint16_t intrp) {
  Wire.begin(sda, sdl);
  return init();
}

int16_t MpuDrv::init() {
  // initialize device
  dmpStatus=ST_0;
  data_ready=0;
  fifoCount=0;
  count=0;
  Serial.println(F("Init I2C dev..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Test device conn..."));
  if(!mpu.testConnection()) {
    Serial.println(F("MPU6050 conn fail"));
    return 0;
  }
  // load and configure the DMP  
  Serial.println(F("Init DMP..."));
  yield();
  uint8_t devStatus = mpu.dmpInitialize();
  yield();
  

  // make sure it worked (returns 0 if so)
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
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // warmup 20 sec??? 
    dmpStatus=ST_WUP;
    start=millis();
    Serial.print(F("DMP ok! Wait for int...FIFO sz is ")); Serial.println(packetSize);
    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Init fail, code ")); Serial.println(devStatus);
    dmpStatus = ST_FAIL;
  }
  return dmpStatus;
}

int16_t MpuDrv::cycle(uint16_t dt) {
  if (dmpStatus==ST_0 || dmpStatus==ST_FAIL) return -1;
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
    Serial.println(F("FIFO overflow!!!"));
    return -2;
  } 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  if (mpuIntStatus & 0x02 || fifoCount >= packetSize) {
    fifoCount = mpu.getFIFOCount();
    //if(fifoCount < packetSize) return 0;
    // wait for correct available data length, should be a VERY short wait
    //while (fifoCount < packetSize) { fifoCount = mpu.getFIFOCount(); delay(1); } // add delay here
    uint8_t i=0;
    while (fifoCount < packetSize && i++<5) { fifoCount = mpu.getFIFOCount(); yield(); } 
    if(fifoCount < packetSize) {
      Serial.println(F("FIFO wait - giveup!!!"));
      return 0; // giveup
    }
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    if(fifoCount >0) { Serial.print(F("FIFO excess : ")); Serial.println(fifoCount);}   
    
    int16_t q16_0[4];
    int16_t aa16_0[3];
    for(i=0; i<4; i++) q16_0[i]=q16[i];
    aa16_0[0]=aa.x; aa16_0[1]=aa.y; aa16_0[2]=aa.z;
    
    mpu.dmpGetQuaternion(q16, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);

    if(dmpStatus==ST_WUP) { // warmup covergence state
      if(count%200==0) {
        Serial.println((millis()-start)/1000);
        yield();
        Serial.print("\tQ16");
        for(i=0; i<4; i++) {Serial.print("\t"); Serial.print(q16[i]);}
        Serial.println();
        Serial.print("\tAcc\t"); Serial.print(aa.x); Serial.print("\t"); Serial.print(aa.y); Serial.print("\t"); Serial.println(aa.z);
      }

      if(count) {
        int16_t aa16[3];
        int32_t qe=0, ae=0, e;
        aa16[0]=aa.x; aa16[1]=aa.y; aa16[2]=aa.z;                   
        for(i=0; i<4; i++) {e=q16[i]-q16_0[i]; if(e<0) e=-e; if(e>qe) qe=e;}
        for(i=0; i<3; i++) {e=aa16[i]-aa16_0[i]; if(e<0) e=-e; if(e>ae) ae=e;}        
        
        if(count%200==0) {Serial.print("Q16 Err:\t"); Serial.print(qe); Serial.print("\tA16 Err:\t"); Serial.println(ae);} 

        if(qe<QUAT_INIT_TOL && ae<ACC_INIT_TOL) {
          if((millis()-start) > INIT_PERIOD_MIN) {
            // TODO add conv count seq !!! (at least 3) 
            Serial.println(F("===MPU Converged"));
            dmpStatus=ST_READY;
          }
        } else if((millis()-start) > INIT_PERIOD_MAX) {
          Serial.println(F("===MPU Failed to converge"));
           dmpStatus=ST_READY; // temporarily
        }
      } // if count

      if(dmpStatus==ST_READY) {
        // TODO - store base data here
      }
    } // warmup

    if(dmpStatus==ST_READY) {
      //mpu.dmpGetQuaternion(&q, fifoBuffer);

      mpu.dmpGetQuaternion(&q, q16);
      mpu.dmpGetGravity(&gravity, &q);
    
      VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // not necessary at this step, should be moved to on-demand

      // use microseconds for V/R integration !!!
    
      if(count%200==0) { 
        //Serial.print("\tQuat\t"); Serial.print(q.w); Serial.print("\t"); Serial.print(q.x); Serial.print("\t"); Serial.print(q.y); Serial.print("\t"); Serial.print(q.z);
        //Serial.print("YPR\t"); Serial.print(ypr[0] * 180.0f/M_PI); Serial.print("\t"); Serial.print(ypr[1] * 180.0f/M_PI); Serial.print("\t"); Serial.println(ypr[2] * 180.0f/M_PI);
        //Serial.print("Cmd Grav\t");Serial.print(gravity.x);Serial.print("\t");Serial.print(gravity.y);Serial.print("\t");Serial.println(gravity.z);
      }
      data_ready=1; 
    }
    count++;       
    return 1;
  } // if data
  return 0;
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

