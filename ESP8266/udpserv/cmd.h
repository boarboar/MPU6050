
const int BUF_SZ = 255;

class CmdProc {
public:  
  static CmdProc Cmd; // singleton  
  int16_t init(uint16_t port);
  int16_t doCmd(/*char *buf*/);
  boolean connected() { return isConnected; }
  boolean read();
  void  respond();
  boolean setLogger(bool on, const char *addr, uint16_t port);
  boolean isSysLog() { return log_on;}
  boolean sendSysLog(const char *buf);
protected:
  CmdProc() : isConnected(false), log_on(false), log_port(0) {;}
  char packetBuffer[BUF_SZ];
  bool isConnected;
  WiFiUDP udp_rcv;
  WiFiUDP udp_snd;
  boolean log_on;
  IPAddress log_addr;
  uint16_t log_port;
};

