#ifndef _UMP_CMD_H_
#define _UMP_CMD_H_

const int BUF_SZ = 255;

class CmdProc {
public:  
  static const int8_t ALR_MPU_RESET=10;
public:  
  static CmdProc Cmd; // singleton  
  int16_t init(uint16_t port);
  int16_t doCmd();
  boolean connected();
  boolean read();
  void  respond();
  boolean isSysLog();
  boolean sendSysLog(const char *buf);
  boolean sendSysLogStatus();
  boolean sendAlarm(uint8_t alr);
protected:
  CmdProc() : isConnected(false) {;}
  void _sendToSysLog(JsonObject& rootOut);
  char packetBuffer[BUF_SZ];
  bool isConnected;
  WiFiUDP udp_rcv;
  WiFiUDP udp_snd;
};

#endif //_UMP_CMD_H_

