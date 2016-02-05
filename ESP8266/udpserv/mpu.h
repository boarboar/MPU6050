class MpuDrv {
public:
  static MpuDrv Mpu; // singleton  
  int16_t init();
protected:  
  MpuDrv();
};

