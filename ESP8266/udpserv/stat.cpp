#include <Arduino.h>
#include "stat.h"

Stat Stat::StatStore ; // singleton

Stat::Stat() : cnt{0} {
  }

