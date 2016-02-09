class CfgDrv {
public:
  static CfgDrv Cfg; // singleton
  int16_t init();  
  int16_t load(const char* fname);
  int16_t store(const char* fname);
  bool isDirty() { return dirty; }
  bool setSysLog(JsonObject& root);
public:
  uint8_t log_on, debug_on;  
  IPAddress log_addr;
  uint16_t log_port;  
protected:  
  CfgDrv();
  bool fs_ok;
  bool dirty;
};

