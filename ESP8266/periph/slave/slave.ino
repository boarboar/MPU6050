// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with thisr

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

int rcv=0, req=0;
uint32_t llast=0L;
uint8_t cnt=0;
uint8_t reg=0, val=0;

void setup()
{  
 digitalWrite(P1_6, HIGH); 
 //pinMode(P1_6, INPUT_PULLUP);  
 digitalWrite(P1_7, HIGH); 
 //pinMode(P1_7, INPUT_PULLUP);
 
  digitalWrite(RED_LED, HIGH); 
  pinMode(RED_LED, OUTPUT);  
  
  for(int i=0;i<5;i++) {
    delay(100); 
    digitalWrite(RED_LED, HIGH); 
    delay(100);  
    digitalWrite(RED_LED, LOW);
  }
  
    delay(1000);
    Serial.begin(9600);
    Serial.println("Init Wire...");
    
  digitalWrite(RED_LED, LOW); 
  pinMode(RED_LED, OUTPUT);  
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  llast=millis();

}


void loop()
{
  if(rcv>0 || req>0) {
    if(rcv>0) {
    Serial.print("Rcv ");
    Serial.print(cnt);
    Serial.print("\tBytes=\t");
    Serial.print(rcv);
    Serial.print("\tReg=\t");
    Serial.print(reg, HEX);
    Serial.print("\tVal=\t");
    Serial.print(val, HEX);
    Serial.println();
    rcv=0;
    }
    if(req>0) {
      Serial.println("Req");
      req=0;
    }
    digitalWrite(RED_LED, HIGH); 
    llast=millis();  
   
  } else if(llast && millis()-llast > 5000L) { // switch off LED
    llast=0L;
    digitalWrite(RED_LED, LOW);     
  }  
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  reg=0;
  int i=0;
  while(Wire.available()>0) {
    int c = Wire.read();
    if(i==0) reg=(int)c;
    else if(i==1) val=(int)c;
    i++;
  }
  rcv=i;
  cnt++;
}

void requestEvent()
{
  req=1;  
  byte buffer[2];              // split int value into two bytes buffer
  //buffer[0] = returnValue >> 8;
  //buffer[1] = returnValue & 255;
  buffer[0] = reg;
  buffer[1] = cnt;
  Wire.write(buffer, 2);       // return response to last command  
}


