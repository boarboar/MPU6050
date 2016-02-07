#include <Arduino.h>
#include "Wire.h"
#include "mpu.h"

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() : dmpReady(false) {;}

int16_t MpuDrv::init(uint16_t sda, uint16_t sdl, uint16_t intrp) {
  Wire.begin(sda, sdl);
  Serial.println(F("Init I2C dev..."));

  //  
  return 0;
}

int16_t MpuDrv::cycle(uint16_t dt) {
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

