#ifndef _UMP_CFG_H_
#define _UMP_CFG_H_

class CfgDrv {
public:
  const int LAZY_WRITE_TIMEOUT=10; //seconds
  static CfgDrv Cfg; // singleton
  int16_t init();  
  int16_t load(const char* fname);
  int16_t store(const char* fname);
  //bool isDirty() { return dirty; }
  bool needToStore();
  bool setSysLog(JsonObject& root);
public:
  uint8_t log_on, debug_on;  
  IPAddress log_addr;
  uint16_t log_port;  
protected:  
  CfgDrv();
  bool fs_ok;
  bool dirty;
  uint32_t  last_chg;
};

#endif //_UMP_CFG_H_

