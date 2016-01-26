#include "session.h"

int16_t CmdSession::sessionCounter=0;

CmdSession::CmdSession() : WiFiClient() {
   //Serial.println("empy constr");
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
  while(available()) { 
    c=read(); 
    if(c==10 || c==13) {
      if(bufptr) { // buffer not empty, means it's a command
        complete=true;
        buf[bufptr]=0; 
      } // otherwise just skip it
    } else {
      if(bufptr<CMDLEN-1) buf[bufptr++]=c;
    }
    nbytes++; 
    //Serial.write(c); 
    }
  //if(nbytes)  
  //  Serial.print("Read session Id: "); Serial.print(sessionId); Serial.print(" Bytes: "); Serial.println(nbytes);
  if(complete) {
    Serial.print("Read cmd at session Id: "); Serial.print(sessionId); Serial.print(": "); Serial.println(buf);
  }
  return complete;
  }

boolean CmdSession::cmd() {
  Serial.print("Exec cmd at session Id: "); Serial.print(sessionId); Serial.print(": "); Serial.println(buf);
  /*
  write("R (", 2);
  write(buf, bufptr);
  write(")\n\r", 3);
  */
  bufptr=0;
  buf[0]=0;
}  
