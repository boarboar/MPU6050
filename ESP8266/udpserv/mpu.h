#ifndef _UMP_MPU_H_
#define _UMP_MPU_H_

#include "MPU6050.h"
#include "helper_3dmath.h"

class MpuDrv {
public:
  static MpuDrv Mpu; // singleton  
  int16_t init(uint16_t sda, uint16_t sdl, uint16_t intr);
  int16_t cycle(uint16_t dt);
  uint8_t isReady();
  uint8_t isDataReady();
  //Quaternion& getQuaternion() { return q;}
  Quaternion& getQuaternion();
  VectorFloat& getGravity() { return gravity;}
  float* getYPR() { return ypr;}
  VectorInt16& getWorldAccel() { return aaWorld;}

protected:  
  MpuDrv();
  // MPU control/status vars
  MPU6050 mpu;
  uint8_t dmpReady;  // set true if DMP init was successful
  uint8_t data_ready;
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint32_t count;
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity; // for test
  float ypr[3];           // [yaw, pitch, roll]
  //VectorFloat ypr;           // [yaw, pitch, roll]   
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

  /*
  volatile uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  */
};

#endif /* _UMP_MPU_H_ */
