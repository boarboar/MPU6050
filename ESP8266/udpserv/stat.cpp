#include <Arduino.h>
#include "stat.h"

Stat Stat::StatStore ; // singleton

Stat::Stat() : cycle_delay_cnt{0}, cycle_mpu_dry_cnt(0) {
  }

