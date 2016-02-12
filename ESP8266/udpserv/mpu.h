#ifndef _UMP_MPU_H_
#define _UMP_MPU_H_

#include "MPU6050.h"
#include "helper_3dmath.h"

class MpuDrv {
public:
  static MpuDrv Mpu; // singleton  
  int16_t init(uint16_t sda, uint16_t sdl, uint16_t intr);
  int16_t cycle(uint16_t dt);
  boolean isReady() { return dmpReady; }
  boolean isDataReady() { return data_ready; }
  Quaternion& getQuaternion() { return q;}
protected:  
  MpuDrv();
  // MPU control/status vars
  MPU6050 mpu;
  bool dmpReady;  // set true if DMP init was successful
  bool data_ready;
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  /*
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  volatile uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  */
};

#endif /* _UMP_MPU_H_ */
