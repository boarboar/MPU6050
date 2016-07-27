#ifndef _UMP_CONTROLLER_H_
#define _UMP_CONTROLLER_H_

#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_STATUS           0x01  // 2 unsigned bytes
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_ACT_POW          0x0A  // 2 signed ints (4 bytes)
#define REG_TARG_POW         0x0B  // 2 signed ints (4 bytes)
#define REG_STEERING         0x0C  // 1 signed int (2 bytes)
#define REG_SENSORS_CNT      0x20  // 8 unsigned ints
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define M_OWN_ID 0x53
#define M_MAGIC_ID 0x4C

#define SENS_SIZE 8

#define V_NORM 10000
#define WHEEL_BASE_MM 130


class Controller {
public:
  enum FailReason {CTL_FAIL_NONE=0, CTL_FAIL_INIT=1, CTL_FAIL_WRT=2, CTL_FAIL_RD=3, CTL_FAIL_OVF=4, CTL_FAIL_ALR=5, CTL_LOG_PID=100};
  static Controller ControllerProc; // singleton  
  bool init();
  uint8_t getStatus();
  uint8_t isDataReady();
  uint8_t isNeedReset();
  void needReset();
  uint8_t getFailReason();
  void getFailParams(int16_t npa, int16_t *pa);
  void clearFailReason();
  void resetIntegrator();
  bool process(float yaw);  
  /*
  bool setTargRotRate(float l, float r);
  bool setTargRotRate(int16_t *d);
  bool getTargRotRate(int16_t *d);
  */
  bool setTargPower(float l, float r);
  bool setTargSteering(int16_t s);
  bool stopDrive();
  bool setPower(int16_t *p);
  bool setSteering(int16_t s);
  uint8_t getNumSensors();
  float *getStoredRotRate();
  int16_t *getStoredAdvance();
  int16_t *getStoredPower();
  int16_t *getStoredSensors();  
  float getMovement();
  float getRotation();
  float getDistance();
  float getAngle();
  float getX();
  float getY();
  //float *getCoords();
protected:  
  Controller();
  uint8_t testConnection();
  uint8_t _getNumSensors();
  bool getControllerStatus();
  bool getActRotRate(); // in RPS
  bool getActAdvance(); // in MMs
  bool getActPower(); 
  bool getSensors(); 
  void raiseFail(uint8_t reason, int16_t p1=0, int16_t p2=0, int16_t p3=0, int16_t p4=0, int16_t p5=0, int16_t p6=0);  
  bool writeInt16(uint16_t reg, int16_t val);
  bool writeInt16_2(uint16_t reg, int16_t left, int16_t right);
  bool readInt16_2(uint16_t reg, int16_t *left, int16_t *right);  
  bool readInt16_2_x(uint16_t reg, int16_t *left, int16_t *right);  
  bool readInt16_2(uint16_t reg, int16_t *d);
  bool readInt16_N(uint16_t reg, uint16_t n, int16_t *d);
private:  
  uint8_t data_ready;
  uint8_t need_reset;
  uint8_t fail_reason;
  int16_t fail_p[6]; 
  
  uint8_t buf[16];  
  uint8_t sta[2];
  float mov, rot;
  uint8_t pready;
  uint8_t nsens;
  float act_rot_rate[2];
  //int16_t targ_rot_rate[2];
  int16_t act_advance[2];
  int16_t act_power[2];
  int16_t sensors[SENS_SIZE];
  int16_t targ_pow[2];
  
  float dist;
  float angle;
  float curr_yaw;
  float r[2];
  float targ_bearing;
  //float err_bearing_p_0, err_bearing_i;
  int16_t err_bearing_p_0, err_bearing_i;
  
};

#endif //_UMP_CONTROLLER_H_

