#include <ESP8266WiFi.h>
#include "session.h"

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
const char* ssid = "NETGEAR";
const char* password = "boarboar";

WiFiServer server(23);
//WiFiClient serverClients[MAX_SRV_CLIENTS];
CmdSession serverClients[MAX_SRV_CLIENTS];

void setup() {
  delay(2000);
  Serial.println("Start");
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  //start UART and the server
  //Serial.begin(115200);
  server.begin();
  server.setNoDelay(true);
  
  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");
}

void loop() {
  uint8_t i;

  //check if there are a new client // one by one...
  if (server.hasClient()){    
    i=0;
    while(i < MAX_SRV_CLIENTS && serverClients[i] && serverClients[i].connected()) i++;
    if(i==MAX_SRV_CLIENTS) {
      //no free/disconnected spot so reject
      WiFiClient serverClient = server.available();
      serverClient.stop();
      Serial.println("No slots, rejected");        
    } else {
      if(serverClients[i]) serverClients[i].stop();
      serverClients[i] = server.available(); // copy constructor be called?
      //Serial.print("New client: "); Serial.println(i);
    }
  }  
   
  //check clients for aliveness and data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){    
    if (serverClients[i]) {
      if(!serverClients[i].connected()){
        serverClients[i].stop(); // after that, inner _client=0, and bool should return 0
        //Serial.print("Client discon: "); Serial.println(i);
      } /* 
      else if(serverClients[i].available()){
        //get data from the telnet client and push it to the UART      
        //while(serverClients[i].available()) Serial.write(serverClients[i].read());
      } */
      else if(serverClients[i].input()) {
        //Serial.println("DO CMD");
        serverClients[i].cmd();
      }
    }    
  }
  
  //check UART for data
  if(Serial.available()){
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        serverClients[i].write(sbuf, len);
        delay(1);
      }
    }
  }
}
