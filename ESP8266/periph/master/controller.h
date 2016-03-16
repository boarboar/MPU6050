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
  bool testConnection();
  uint8_t getNumSensors();
  bool getTargRotRate(int16_t *d);
  bool setTargRotRate(int16_t *d);
  bool getActRotRate(int16_t *d);
  bool getActAdvance(int16_t *d);
  bool getSensors(int16_t *sens);
protected:  
  Controller();
  bool writeInt16_2(uint16_t reg, int16_t *d);
  bool readInt16_2(uint16_t reg, int16_t *d);
  bool readInt16_N(uint16_t reg, uint16_t n, int16_t *d);
private:
  uint8_t buf[16];  
};

#endif //_UMP_CONTROLLER_H_

