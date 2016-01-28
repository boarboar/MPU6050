#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

void print_sys_info();

const char* ssid = "NETGEAR";
const char* password = "boarboar";
const int udp_port = 4444;
bool isConnected=false;
char packetBuffer[255];
WiFiUDP udp_rcv;
WiFiUDP udp_snd;

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

  if(udp_rcv.begin(udp_port)) {
    isConnected=true;
    Serial.print("Ready! Listening on ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(udp_port);
  } else {
    Serial.println("Failed to init UDP socket!");
  }  
  print_sys_info();
}

void loop() {
  if(!isConnected) return;
  uint32_t m1=millis();
  int packetSize = udp_rcv.parsePacket();
  //Serial.print("psz: "); 
  if (packetSize) {
    int len = udp_rcv.read(packetBuffer, 255);
    if(len>254) len=254;
    packetBuffer[len] = 0;    
    udp_snd.beginPacket(udp_rcv.remoteIP(), udp_rcv.remotePort());
    udp_snd.write(packetBuffer, len);
    udp_snd.endPacket();
    uint32_t m2=millis();
    Serial.print("From: "); Serial.print(udp_rcv.remoteIP());
    Serial.print(":"); Serial.print(udp_rcv.remotePort());
    Serial.print(" len: "); Serial.print(len);
    Serial.print(" T: "); Serial.print(m2-m1);
    Serial.print(" Val: "); Serial.println(packetBuffer);

    print_sys_info();
  }
  delay(10);
}

void print_sys_info() {
  Serial.print("Heap sz: "); Serial.println(ESP.getFreeHeap());
}
