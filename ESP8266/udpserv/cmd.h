#ifndef _UMP_CMD_H_
#define _UMP_CMD_H_

const int BUF_SZ = 255;

#define SL_LEVEL_NONE   0
#define SL_LEVEL_ALARM  1
#define SL_LEVEL_MESSAGE  2

class CmdProc {
public:  
  //static const int8_t ALR_MPU_RESET=10;
  enum Alarms {ALR_RESET=1, ALR_MPU_RESET=10, ALR_MPU_FAILURE=11, ALR_CTL_RESET=20, ALR_CTL_FAILURE=21}; 
public:  
  static CmdProc Cmd; // singleton  
  int16_t init(uint16_t port);
  int16_t doCmd();
  boolean connected();
  boolean read();
  void  respond();
  int16_t getSysLogLevel();
  boolean sendSysLog(const char *buf);
  boolean sendSysLogStatus();
  boolean sendAlarm(uint8_t alr, uint8_t param, int16_t p1=0, int16_t p2=0);
protected:
  CmdProc() : isConnected(false) {;}
  void _sendToSysLog(JsonObject& rootOut);
  char packetBuffer[BUF_SZ];
  bool isConnected;
  WiFiUDP udp_rcv;
  WiFiUDP udp_snd;
};

#endif //_UMP_CMD_H_

