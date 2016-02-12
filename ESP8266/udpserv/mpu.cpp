#include <Arduino.h>
#include "Wire.h"

//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpReady(false), data_ready(false), fifoCount(0) {;}

int16_t MpuDrv::init(uint16_t sda, uint16_t sdl, uint16_t intrp) {
  Wire.begin(sda, sdl);
  // initialize device
  Serial.println(F("Init I2C dev..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Test device conn..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 conn OK") : F("MPU6050 conn fail"));

  // load and configure the DMP
  
  Serial.println(F("Init DMP..."));
  yield();
  uint8_t devStatus = mpu.dmpInitialize();
  yield();
  /*
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  */
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
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
    dmpReady = true;
    Serial.print(F("DMP ok! Wait for int...FIFO sz is ")); Serial.println(packetSize);
    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Init fail, code ")); Serial.println(devStatus);
  }
  return dmpReady;
}

int16_t MpuDrv::cycle(uint16_t dt) {
  if (!dmpReady) return -1;
/*
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    */
  uint8_t mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount=0;
    Serial.println(F("FIFO overflow!!!"));
    return -2;
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
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
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    if(fifoCount >0) Serial.println(F("FIFO excess!"));
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);

            yield();
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);

    data_ready=true;        
    return 1;
  }  
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

