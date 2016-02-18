#include <Arduino.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"

/*
 FS_SEL | Full Scale Range   | LSB Sensitivity (dividers)
 -------+--------------------+----------------
 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 
  AFS_SEL | Full Scale Range | LSB Sensitivity (dividers)
 --------+------------------+----------------
 0       | +/- 2g           | 8192 LSB/mg
 1       | +/- 4g           | 4096 LSB/mg
 2       | +/- 8g           | 2048 LSB/mg
 3       | +/- 16g          | 1024 LSB/mg
 
 */

#define SCALE_A (2.0f*8192.0f) // 1g = (9.80665 m/s^2)
//#define SCALE_G 131.0f      // 
#define G_FORCE 9.80665f
#define G_SCALE (G_FORCE/SCALE_A)

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpStatus(ST_0)/*, data_ready(0), fifoCount(0), count(0)*/ {}

int8_t MpuDrv::getStatus() { return dmpStatus; }
uint8_t MpuDrv::isDataReady() { return data_ready; }
//Quaternion& MpuDrv::getQuaternion() { return q; }
//float* MpuDrv::getYPR() { return ypr; }
//VectorFloat& MpuDrv::getGravity() {return gravity;}
//VectorInt16& MpuDrv::getWorldAccel() {return aaWorld;}

int16_t MpuDrv::init(uint16_t sda, uint16_t sdl, uint16_t intrp) {
  Wire.begin(sda, sdl);
  return init();
}

int16_t MpuDrv::init() {
  // TODO - split into 2 stages
  // initialize device
  // STAGE-0
  dmpStatus=ST_0;
  data_ready=0;
  fifoCount=0;
  count=0;
  conv_count=0;
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
  
  // // STAGE-1 ?
  
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
    // enter warmup/convergence stage 
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
    uint8_t i=0;
    fifoCount = mpu.getFIFOCount();
    //if(fifoCount < packetSize) return 0;
    while (fifoCount < packetSize && i++<5) { fifoCount = mpu.getFIFOCount(); yield(); } 
    if(fifoCount < packetSize) {
      Serial.println(F("FIFO wait - giveup!!!"));
      return 0; // giveup
    }
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    if(fifoCount >0) { Serial.print(F("FIFO excess : ")); Serial.println(fifoCount);}   
    
    //int16_t q16_0[4];
    //int16_t aa16_0[3];
    //for(i=0; i<4; i++) q16_0[i]=q16[i];
    //aa16_0[0]=aa.x; aa16_0[1]=aa.y; aa16_0[2]=aa.z;
    
    mpu.dmpGetQuaternion(q16, fifoBuffer);
    mpu.dmpGetAccel(&aa16, fifoBuffer);
    
    if(dmpStatus==ST_WUP) { // warmup covergence state
      if(count%200==1) { // 1,201,401,...
        // check convergence
        //
        Serial.println((millis()-start)/1000);
        yield();
        Serial.print("\tQ16-0");
        for(i=0; i<4; i++) {Serial.print("\t"); Serial.print(q16_0[i]);}
        Serial.println();
        Serial.print("\tQ16-1");
        for(i=0; i<4; i++) {Serial.print("\t"); Serial.print(q16[i]);}
        Serial.println();
        //Serial.print("\tAcc-0\t"); Serial.print(aa16_0[0]); Serial.print("\t"); Serial.print(aa16_0[1]); Serial.print("\t"); Serial.println(aa16_0[2]);
        Serial.print("\tAcc-0\t"); Serial.print(aa16_0.x); Serial.print("\t"); Serial.print(aa16_0.y); Serial.print("\t"); Serial.println(aa16_0.z);
        Serial.print("\tAcc-1\t"); Serial.print(aa16.x); Serial.print("\t"); Serial.print(aa16.y); Serial.print("\t"); Serial.println(aa16.z);
        //
        int16_t ad16[3];
        int32_t qe=0, ae=0, e;
        ad16[0]=aa16.x-aa16_0.x; ad16[1]=aa16.y-aa16_0.y; ad16[2]=aa16.z-aa16_0.z;                   
        for(i=0; i<4; i++) {e=q16[i]-q16_0[i]; if(e<0) e=-e; if(e>qe) qe=e;}
        for(i=0; i<3; i++) {e=ad16[i]; if(e<0) e=-e; if(e>ae) ae=e;}        
        
        Serial.print("Q16 Err:\t"); Serial.print(qe); Serial.print("\tA16 Err:\t"); Serial.println(ae);
        
        if(qe<QUAT_INIT_TOL && ae<ACC_INIT_TOL) {
          conv_count++;
          if((millis()-start)/1000 > INIT_PERIOD_MIN) {
            // TODO add conv count seq !!! (at least 3) 
            Serial.print(F("===MPU Converged, cnvcnt=")); Serial.println(conv_count);
            dmpStatus=ST_READY;
          }
        } else if((millis()-start)/1000 > INIT_PERIOD_MAX) {
            Serial.print(F("===MPU Failed to converge, cnvcnt=")); Serial.println(conv_count);
            dmpStatus=ST_READY; // temporarily
        }

        for(i=0; i<4; i++) q16_0[i]=q16[i];
        aa16_0 = aa16;
        
      } // if count      
    } // warmup

    if(dmpStatus==ST_READY) {
      /*
      Quaternion q;
      VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
      VectorFloat gravity;
      
      mpu.dmpGetQuaternion(&q, q16);
      mpu.dmpGetGravity(&gravity, &q);
    
      mpu.dmpGetLinearAccel(&aaReal, &aa16, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

      //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // not necessary at this step, should be moved to on-demand
*/
      // use microseconds for V/R integration !!!
    /*
      if(count%200==0) { 
               Serial.println((millis()-start)/1000);
 
        //Serial.print("\tQuat\t"); Serial.print(q.w); Serial.print("\t"); Serial.print(q.x); Serial.print("\t"); Serial.print(q.y); Serial.print("\t"); Serial.print(q.z);
        Serial.print("YPR\t"); Serial.print(ypr[0] * 180.0f/M_PI); Serial.print("\t"); Serial.print(ypr[1] * 180.0f/M_PI); Serial.print("\t"); Serial.println(ypr[2] * 180.0f/M_PI);
        //Serial.print("Cmd Grav\t");Serial.print(gravity.x);Serial.print("\t");Serial.print(gravity.y);Serial.print("\t");Serial.println(gravity.z);
      }
      */
      data_ready=1; 
    }
    count++;       
    return 1;
  } // if data
  return 0;
}


void MpuDrv::getYPR(float* ypr) {        
  Quaternion q;
  VectorFloat gravity;    
  mpu.dmpGetQuaternion(&q, q16);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 

  Serial.print(F("Observ Quat\t")); Serial.print(q.w); Serial.print("\t"); Serial.print(q.x); Serial.print("\t"); Serial.print(q.y); Serial.print("\t"); Serial.println(q.z);
  Serial.print(F("Observ YPR\t")); Serial.print(ypr[0] * 180.0f/M_PI); Serial.print("\t"); Serial.print(ypr[1] * 180.0f/M_PI); Serial.print("\t"); Serial.println(ypr[2] * 180.0f/M_PI);
  Serial.print(F("Observ Grav\t"));Serial.print(gravity.x);Serial.print("\t");Serial.print(gravity.y);Serial.print("\t");Serial.println(gravity.z);
  yield();
  Quaternion q0;
  mpu.dmpGetQuaternion(&q0, q16_0);
  Serial.print(F("Base Quat\t")); Serial.print(q0.w); Serial.print("\t"); Serial.print(q0.x); Serial.print("\t"); Serial.print(q0.y); Serial.print("\t"); Serial.println(q0.z);
  q0=q0.getConjugate();
  Serial.print(F("Conj Quat\t")); Serial.print(q0.w); Serial.print("\t"); Serial.print(q0.x); Serial.print("\t"); Serial.print(q0.y); Serial.print("\t"); Serial.println(q0.z);
  q=q0.getProduct(q);
  Serial.print(F("Real Quat\t")); Serial.print(q.w); Serial.print("\t"); Serial.print(q.x); Serial.print("\t"); Serial.print(q.y); Serial.print("\t"); Serial.println(q.z);
  yield();
  float yprr[3];
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yprr, &q, &gravity); 
  Serial.print(F("Real Grav\t"));Serial.print(gravity.x);Serial.print("\t");Serial.print(gravity.y);Serial.print("\t");Serial.println(gravity.z);
  Serial.print(F("Real YPR\t")); Serial.print(yprr[0] * 180.0f/M_PI); Serial.print("\t"); Serial.print(yprr[1] * 180.0f/M_PI); Serial.print("\t"); Serial.println(yprr[2] * 180.0f/M_PI);
  yield();
}

VectorInt16 MpuDrv::getWorldAccel() {
  Quaternion q;
  VectorFloat gravity;    
  VectorInt16 aaReal; 
  VectorInt16 aaWorld;
  mpu.dmpGetQuaternion(&q, q16);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa16, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  Serial.print(F("Observ RGrav\t"));Serial.print(aa16.x);Serial.print("\t");Serial.print(aa16.y);Serial.print("\t");Serial.println(aa16.z);
  Serial.print(F("Observ WGrav\t"));Serial.print(aaWorld.x);Serial.print("\t");Serial.print(aaWorld.y);Serial.print("\t");Serial.println(aaWorld.z);

  yield();
  
  VectorInt16 aa;
  aa.x=aa16.x-aa16_0.x; aa.y=aa16.y-aa16_0.y; aa.z=aa16.z-aa16_0.z;
  mpu.dmpGetLinearAccel(&aaReal, &aa16, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  Serial.print(F("Adj RGrav\t"));Serial.print(aa16.x);Serial.print("\t");Serial.print(aa16.y);Serial.print("\t");Serial.println(aa16.z);
  Serial.print(F("Adj WGrav\t"));Serial.print(aaWorld.x);Serial.print("\t");Serial.print(aaWorld.y);Serial.print("\t");Serial.println(aaWorld.z);
  // but still in the tilted basis!!!

  // real A=ax/SCALE_A*G_FORCE
  yield();
  VectorFloat af;
  af.x=aaWorld.x*G_SCALE; af.y=aaWorld.y*G_SCALE; af.z=aaWorld.z*G_SCALE;

  Serial.print(F("Real WGrav (m/s^2)\t"));Serial.print(af.x);Serial.print("\t");Serial.print(af.y);Serial.print("\t");Serial.println(af.z);
  // but still in the tilted basis!!!
  
  return aaWorld;
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

