#ifndef _UMP_CONTROLLER_H_
#define _UMP_CONTROLLER_H_

#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_SENSORS_CNT      0x20  // 8 unsigned ints
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define M_OWN_ID 0x53

#define V_NORM 10000

class Controller {
public:
  static Controller ControllerProc; // singleton  
  bool init();
  uint8_t testConnection();
  uint8_t getNumSensors();
  bool getTargRotRate(int16_t *d);
  bool setTargRotRate(int16_t *d);
  bool getActRotRate(int16_t *d); // in V_NORM-ed RPS
  bool getActAdvance(int16_t *d); // in MMs
  bool getSensors(int16_t *sens);
  bool stopDrive();
  bool process();
  float *getStoredRotRate();
  int16_t *getStoredAdvance();
  int16_t *getStoredSensors();  
protected:  
  Controller();
  bool writeInt16_2(uint16_t reg, int16_t left, int16_t right);
  bool readInt16_2(uint16_t reg, int16_t *left, int16_t *right);  
  bool readInt16_2(uint16_t reg, int16_t *d);
  bool readInt16_N(uint16_t reg, uint16_t n, int16_t *d);
private:  
  uint8_t buf[16];  
  //int16_t act_rot_rate[2];
  float act_rot_rate[2];
  int16_t act_advance[2];
  int16_t sensors[8];
  uint8_t pready;
};

#endif //_UMP_CONTROLLER_H_

