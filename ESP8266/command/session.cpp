#include "session.h"

int16_t CmdSession::sessionCounter=0;

CmdSession::CmdSession() : WiFiClient() {
   }
   
CmdSession::CmdSession(const WiFiClient& other) : WiFiClient(other), bufptr(0) { 
  sessionId=++sessionCounter;
  buf[0]=0;
  Serial.print("New session Id: "); Serial.println(sessionId);
  }
  
CmdSession::~CmdSession() {;}   

void CmdSession::stop() {
  WiFiClient::stop();
  Serial.print("Close session Id: "); Serial.println(sessionId);
  }

boolean CmdSession::input() {
  if(!available()) return false;
  //TODO - some reasonable limitation on a number of bytes read at onec to be set...
  uint16_t nbytes=0;
  uint16_t c;
  bool complete=false;
  while(available()) { // TODO: optimize, since read calls available and returns -1 if not available
    c=read(); 
    if(c==10 || c==13) {
      if(bufptr) { // buffer not empty, means it's a command
        complete=true;
        buf[bufptr]=0; 
      } // otherwise just skip it
    } else {
      if(bufptr<CMDLEN-1) buf[bufptr++]=(uint8_t)c;
    }
    nbytes++; 
    //Serial.write(c); 
    }
  if(complete) {
    Serial.print("Read cmd at session Id: "); Serial.print(sessionId); Serial.print(": "); Serial.println((const char *)buf);
  }
  return complete;
  }

boolean CmdSession::cmd() {
  Serial.print("Exec cmd at session Id: "); Serial.print(sessionId); Serial.print(": "); Serial.println((const char *)buf);
  uint32_t m1=millis();
  write("R (", 2);
  write((const uint8_t *)buf, (size_t)bufptr);
  write(")\n\r", 3);
  uint32_t m2=millis();
  Serial.print("Feedback written in (ms):"); Serial.println(m2-m1);
  bufptr=0;
  buf[0]=0;
}  
