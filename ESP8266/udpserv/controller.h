#ifndef _UMP_CONTROLLER_H_
#define _UMP_CONTROLLER_H_

#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_STATUS           0x01  // 2 unsigned bytes
#define REG_START            0x02  // 1 unsigned bytes
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_ACT_POW          0x0A  // 2 signed ints (4 bytes)
#define REG_TARG_POW         0x0B  // 2 signed ints (4 bytes)
#define REG_STEERING         0x0C  // 1 signed int (2 bytes)
#define REG_SENSORS_CNT      0x20  // 8 unsigned ints
#define REG_SENSORS_1H       0x21  // up to 6 unsigned ints
#define REG_SENSORS_2H       0x22  // up to 6 unsigned ints
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define M_OWN_ID 0x53
//#define M_MAGIC_ID 0x4C

#define SENS_SIZE 10
#define SPEED_R_SZ 8
#define V_NORM 10000

#define WHEEL_BASE_MM 130


class Controller {
public:
  enum FailReason {CTL_FAIL_NONE=0, CTL_FAIL_INIT=1, CTL_FAIL_WRT=2, CTL_FAIL_RD=3, CTL_FAIL_OVF=4, CTL_FAIL_ALR=5, CTL_FAIL_OBST=6, CTL_LOG_PID=100, CTL_LOG_POW=101, CTL_LOG_PBPID=102};
  static Controller ControllerProc; // singleton  
  bool init();
  bool start();
  uint8_t getStatus();
  uint8_t isDataReady();
  uint8_t isNeedReset();
  void needReset();
  void resetIntegrator();
  bool process(float yaw, uint32_t dt);  
  bool setTargPower(float l, float r);
  bool setTargSteering(int16_t s);
  bool setTargSpeed(int16_t s);
  int16_t getTargSpeed();
  bool setTargBearing(int16_t s);
  
  //bool setSteering(int16_t s);
  uint8_t getNumSensors();
  //float *getStoredRotRate();
  int32_t *getStoredAdvance();
  int16_t *getStoredPower();
  //int16_t *getCurPower();
  int16_t *getStoredSensors();  
  //float getMovement();
  //float getRotation();
  float getDistance();
  int16_t getSpeed();
  //float getAngle();
  float getX();
  float getY();
  bool getActPower();   
  float getAVQErr();

protected:  
  Controller();
  uint8_t testConnection();
  uint8_t _getNumSensors();
  //bool getControllerStatus();
  //bool getActRotRate(); // in RPS
  bool getActAdvance(); // in MMs
  bool getSensors(); 
  bool setStart(uint8_t p); 
  bool startRotate(int16_t tspeed);
  bool setPowerRotate(int16_t dir, int16_t *p);
  bool setPowerStraight(int16_t dir, int16_t *p);
  void adjustTargBearing(int16_t s, bool absolute);
  uint8_t checkObastacle();
  bool writeInt16(uint16_t reg, int16_t val);
  bool writeInt16_2(uint16_t reg, int16_t left, int16_t right);
  bool readInt16_2(uint16_t reg, int16_t *left, int16_t *right);  
  bool readInt16_2(uint16_t reg, int16_t *d);
  bool readInt32_2(uint16_t reg, int32_t *d);
  bool readInt16_N(uint16_t reg, uint16_t n, int16_t *d);
private:  
  uint8_t data_ready;
  uint8_t need_reset;
  
  uint8_t buf[20];  
  //uint8_t sta[2];
  //float mov, rot;
  uint8_t pready;
  uint8_t nsens;
  //float act_rot_rate[2];
  //int16_t targ_rot_rate[2];
  int32_t act_advance[2];
  int32_t act_advance_0[2];
  int16_t act_power[2];
  int16_t sensors[SENS_SIZE];
  //int16_t cur_pow[2];
  int16_t base_pow;
  int16_t delta_pow;
  int16_t targ_speed; //mm_s
  int16_t rot_speed; //mm_s
  
  float dist;
  //float angle;
  float curr_yaw;
  float r[2];
  float targ_bearing;
  int16_t speed; //mm/s
  int16_t speed_r[SPEED_R_SZ]; // raw speed readings mm/s
  //float err_bearing_p_0, err_bearing_i;
  int16_t err_bearing_p_0, err_bearing_i;
  float err_speed_p_0, err_speed_i;
  uint32_t pid_cnt;
  float qsum_err;
  float run_dist;
};

#endif //_UMP_CONTROLLER_H_

