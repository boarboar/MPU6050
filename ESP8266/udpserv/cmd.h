#ifndef _UMP_CMD_H_
#define _UMP_CMD_H_

const int BUF_SZ = 255;

class CmdProc {
public:  
  static CmdProc Cmd; // singleton  
  int16_t init(uint16_t port);
  int16_t doCmd();
  boolean connected() { return isConnected; }
  boolean read();
  void  respond();
  boolean isSysLog(); // { return log_on;}
  boolean sendSysLog(const char *buf);
  boolean sendSysLogStatus();
protected:
  CmdProc() : isConnected(false) {;}
  char packetBuffer[BUF_SZ];
  bool isConnected;
  WiFiUDP udp_rcv;
  WiFiUDP udp_snd;
};

#endif //_UMP_CMD_H_

