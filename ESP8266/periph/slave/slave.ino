// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

uint32_t llast=0L;
uint8_t cnt=0;

uint8_t reg=0;

void setup()
{
  delay(2000);
  Serial.begin(9600);
  Serial.println("Init Wire...");    
  digitalWrite(RED_LED, LOW); 
  pinMode(RED_LED, OUTPUT);  
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  //delay(100);
  if(llast && millis()-llast < 5000L) { // switch off LED
    llast=0L;
    digitalWrite(RED_LED, LOW);     
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  
  Serial.print("Received...REG=\t");    

  reg=0;
  if(Wire.available()) reg=Wire.read();    
  Serial.print(reg, HEX);
  Serial.print(",\tV=\t");
  while(Wire.available()) {
    char c = Wire.read();
    Serial.print(c, HEX);
    Serial.print(' ');
  }
  Serial.println();
    
  cnt++;
  
  digitalWrite(RED_LED, HIGH); 
  llast=millis();
}

void requestEvent()
{
  Serial.println("Requested");    
  /*
  byte buffer[2];              // split int value into two bytes buffer
  buffer[0] = returnValue >> 8;
  buffer[1] = returnValue & 255;
  Wire.write(buffer, 2);       // return response to last command
  */
  byte buffer[1];              // split int value into two bytes buffer
  buffer[0] = cnt;
  Wire.write(buffer, 1);       // return response to last command  
}
