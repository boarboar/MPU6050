#include <Arduino.h>
#include "mpu.h"

MpuDrv MpuDrv::Mpu; // singleton

MpuDrv::MpuDrv() {;}

int16_t MpuDrv::init() {
  return 0;
}

