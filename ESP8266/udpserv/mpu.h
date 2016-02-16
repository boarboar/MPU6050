#ifndef _UMP_MPU_H_
#define _UMP_MPU_H_

#include "MPU6050.h"
#include "helper_3dmath.h"

class MpuDrv {
  public:
  static const int8_t ST_0=0;
  static const int8_t ST_FAIL=255;
  static const int8_t ST_INIT=1;
  static const int8_t ST_WUP=2;
  static const int8_t ST_READY=3;
public:
  static MpuDrv Mpu; // singleton  
  int16_t init(uint16_t sda, uint16_t sdl, uint16_t intr);
  int16_t init();
  int16_t cycle(uint16_t dt);
  uint8_t getStatus();
  uint8_t isDataReady();
  Quaternion& getQuaternion();
  VectorFloat& getGravity();
  float* getYPR();
  VectorInt16& getWorldAccel();

protected:  
  MpuDrv();
  // MPU control/status vars
  MPU6050 mpu;
  uint32_t start;
  uint8_t dmpStatus; 
  uint8_t data_ready;
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint32_t count;
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity; // for test
  float ypr[3];           // [yaw, pitch, roll]
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  int32_t g16[3]; 
  /*
  volatile uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  */
};

#endif /* _UMP_MPU_H_ */
