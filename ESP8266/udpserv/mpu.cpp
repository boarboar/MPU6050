#include <Arduino.h>
#include "Wire.h"
#include "stat.h"
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

//#define SCALE_A (2.0f*8192.0f) // 1g = (9.80665 m/s^2)
#define SCALE_A (8192.0f) // 1g = (9.80665 m/s^2)
//#define SCALE_G 131.0f      // 
#define G_FORCE 9.80665f
#define G_SCALE (G_FORCE/SCALE_A)

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpStatus(ST_0)/*, data_ready(0), fifoCount(0), count(0)*/ {}

int8_t MpuDrv::getStatus() { return dmpStatus; }
uint8_t MpuDrv::isDataReady() { return data_ready; }

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
  resetIntegrator();
  //v.x=v.y=v.z=0.0f;
  //r.x=r.y=r.z=0.0f;
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
    Stat::StatStore.mpu_owfl_cnt++;
    Serial.println(F("FIFO overflow!!!"));
    return -2;
  } 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)

  if (!(mpuIntStatus & 0x02) && (fifoCount < packetSize) ) return 0; // nothing to read
  
  /*if (mpuIntStatus & 0x02 || fifoCount >= packetSize)*/ {
    uint8_t i=0;
    bool settled=false;
    fifoCount = mpu.getFIFOCount();
    //if(fifoCount < packetSize) return 0; // ???
    while (fifoCount < packetSize && i++<5) { fifoCount = mpu.getFIFOCount(); yield(); } 
    if(fifoCount < packetSize) {
      Serial.println(F("FIFO wait - giveup!!!"));
      Stat::StatStore.mpu_gup_cnt++;
      return 0; // giveup
    }
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    //mpu.resetFIFO(); fifoCount=0; // this is in case of overflows... 
    fifoCount -= packetSize;
    if(fifoCount >0) { 
      Serial.print(F("FIFO excess : ")); Serial.println(fifoCount);
      Stat::StatStore.mpu_exc_cnt++;
      }   
    
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
        
        Serial.print("Q16 Err:\t"); Serial.print(qe); Serial.print("\tA16 Err:\t"); Serial.print(ae); 
        Serial.print("\tCC:\t"); Serial.print(conv_count); Serial.print("\tT:\t"); Serial.println((millis()-start)/1000);
        
        if(qe<QUAT_INIT_TOL && ae<ACC_INIT_TOL) {
          conv_count++;
          if((millis()-start)/1000 > INIT_PERIOD_MIN && conv_count>3) settled=true;            
        } else
          conv_count=0;  
          if((millis()-start)/1000 > INIT_PERIOD_MAX) {
            Serial.println(F("===MPU Failed to converge, stilling switch to settled status...")); // TODO -?
            settled=true;
        }

        for(i=0; i<4; i++) q16_0[i]=q16[i];
        aa16_0 = aa16;
        
      } // if count    

      if(settled) {
        Serial.print(F("===MPU Converged, cnvcnt=")); Serial.println(conv_count);
        start=micros();
        dmpStatus=ST_READY;        
      }
    } // warmup

    if(dmpStatus==ST_READY) {
      // integrate motion
      Quaternion q, q0;
      VectorFloat gravity;    
      VectorInt16 aaReal, aaWorld;
      float ts;
      uint32_t mcs;
      // get world frame accel (with adjustment) - needed for V-integration
      mpu.dmpGetQuaternion(&q, q16);
      mpu.dmpGetQuaternion(&q0, q16_0);
      q0=q0.getConjugate();
      q=q0.getProduct(q); // real quaternion (relative to base)
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa16, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      
      a.x=aaWorld.x*G_SCALE; a.y=aaWorld.y*G_SCALE; a.z=aaWorld.z*G_SCALE;
      
      if(settled) {
        a0=a;
        Serial.print(F("A Base (m/s^2)\t"));Serial.print(a0.x);Serial.print("\t");Serial.print(a0.y);Serial.print("\t");Serial.println(a0.z);        
      }
      
      a.x-=a0.x; a.y-=a0.y; a.z-=a0.z;
 //          a=a.getRotated(&q0);
      mcs=micros();
      ts=(float)(micros()-start)/1000000.0f;
      start=mcs;
      v.x+=a.x*ts; v.y+=a.y*ts; v.z+=a.z*ts;
//      r.x+=v.x*ts; r.y+=v.y*ts; r.z+=v.z*ts;
// todo - LOW pass filter
// float A_K=0.3f;
// val = val - A_K * (val - new_meas_val);
      data_ready=1; 
    }
    count++;       
    return 1;
  } // if data // to remove...
  return 0;
}

void MpuDrv::resetIntegrator() {
  v.x=v.y=v.z=0.0f;  
}

void MpuDrv::getAll(float* ypr, float* af, float* vf/*, float *rf*/) {        
  Quaternion q, q0;
  VectorFloat gravity;    
  uint8_t i;
  mpu.dmpGetQuaternion(&q, q16);
  //Serial.print(F("Meas Quat\t")); Serial.print(q.w); Serial.print("\t"); Serial.print(q.x); Serial.print("\t"); Serial.print(q.y); Serial.print("\t"); Serial.print(q.z); Serial.print("\tM: "); Serial.println(q.getMagnitude());
  mpu.dmpGetQuaternion(&q0, q16_0);
  q0=q0.getConjugate();
  q=q0.getProduct(q); // real quaternion (relative to base)
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 

/*
  Serial.print(F("Conj Quat\t")); Serial.print(q0.w); Serial.print("\t"); Serial.print(q0.x); Serial.print("\t"); Serial.print(q0.y); Serial.print("\t"); Serial.print(q0.z);Serial.print("\tM: "); Serial.println(q0.getMagnitude());
  Serial.print(F("Real Quat\t")); Serial.print(q.w); Serial.print("\t"); Serial.print(q.x); Serial.print("\t"); Serial.print(q.y); Serial.print("\t"); Serial.print(q.z);Serial.print("\tM: "); Serial.println(q.getMagnitude());
yield();
*/

  af[0]=a.x; af[1]=a.y; af[2]=a.z;
  vf[0]=v.x; vf[1]=v.y; vf[2]=v.z;
//  rf[0]=r.x; rf[1]=r.y; rf[2]=r.z;

  Serial.print(F("YPR")); 
  for(i=0; i<3; i++) { Serial.print("\t"); Serial.print(ypr[i]);}
  Serial.println();

  /*
  Serial.print(F("A")); 
  for(i=0; i<3; i++) { Serial.print("\t"); Serial.print(af[i]);}
  Serial.println();
*/
  yield();
  
  Serial.print(F("V")); 
  for(i=0; i<3; i++) { Serial.print("\t"); Serial.print(vf[i]);}
  Serial.println();

  /*
  Serial.print(F("R")); 
  for(i=0; i<3; i++) { Serial.print("\t"); Serial.print(rf[i]);}
  Serial.println();
*/
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

