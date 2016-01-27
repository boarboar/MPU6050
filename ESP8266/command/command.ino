#include <ESP8266WiFi.h>
#include "session.h"

void print_sys_info();

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
const char* ssid = "NETGEAR";
const char* password = "boarboar";

WiFiServer server(23);
CmdSession clients[MAX_SRV_CLIENTS];

void setup() {
  delay(2000);
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
  print_sys_info();
}

void loop() {
  uint8_t i;

  //check if there are a new client // one by one...
  if (server.hasClient()){    
    i=0;
    while(i < MAX_SRV_CLIENTS && clients[i] && clients[i].connected()) i++;
    if(i==MAX_SRV_CLIENTS) {
      //no free/disconnected spot so reject
      /*
      WiFiClient serverClient = server.available();
      serverClient.stop();
      */
      server.available().stop(); // reject connection
      Serial.println("No slots, rejected");        
    } else {
      if(clients[i]) clients[i].stop();
      clients[i] = server.available();
      print_sys_info();
    }
  }  
   
  //check clients for aliveness and data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){    
    if (clients[i]) {
      if(!clients[i].connected()){
        clients[i].stop(); // after that, inner _client=0, and bool should return 0
        print_sys_info();
      } else if(clients[i].input()) {
        yield();
        clients[i].cmd();
        print_sys_info();
      }
    }    
  }
  /*
  //check UART for data
  if(Serial.available()){
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (clients[i] && clients[i].connected()){
        clients[i].write(sbuf, len);
        delay(1);
      }
    }
  }*/
  
}

void print_sys_info() {
  Serial.print("Heap sz: "); Serial.println(ESP.getFreeHeap());
}

