class MpuDrv {
public:
  static MpuDrv Mpu; // singleton  
  int16_t init(uint16_t sda, uint16_t sdl, uint16_t intr);
  int16_t cycle(uint16_t dt);
protected:  
  MpuDrv();
  // MPU control/status vars
  bool dmpReady;  // set true if DMP init was successful
  /*
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  volatile uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  */
};

