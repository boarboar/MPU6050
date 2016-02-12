#ifndef _UMP_STAT_H_
#define _UMP_STAT_H_

class Stat {
public:
  static Stat StatStore; // singleton  
  uint32_t cycle_delay_cnt[4];
  uint32_t cycle_mpu_dry_cnt;
protected:  
  Stat();
};

#endif //_UMP_STAT_H_

