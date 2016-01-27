#include <ESP8266WiFi.h>

class CmdSession : public WiFiClient {
public:  
  CmdSession();
  CmdSession(const WiFiClient&);
  virtual ~CmdSession();   
  virtual void stop();
  boolean input();
  boolean cmd();
protected:
  static const uint16_t CMDLEN=63;  
  int16_t sessionId;
  //char buf[CMDLEN+1];
  uint8_t buf[CMDLEN+1];
  uint8_t bufptr;
  static int16_t sessionCounter;
};

