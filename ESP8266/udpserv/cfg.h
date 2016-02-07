class CfgDrv {
public:
  static CfgDrv Cfg; // singleton
  int16_t init();  
  int16_t load(const char* fname);
  int16_t store(const char* fname);
protected:  
  CfgDrv();
  bool fs_ok;
  bool dirty;
};

