#include <Wire.h>
#include "I2Cdev.h"

const int MPU_SDA=0;
const int MPU_SDL=2;
const int DEV_ID=4;


void setup() {
//  digitalWrite(RED_LED, LOW); 
//  pinMode(RED_LED, OUTPUT);
  
  delay(2000);
    Serial.begin(115200);
    //Serial.begin(9600);
    Serial.println("Init Wire...");

    Wire.begin(MPU_SDA, MPU_SDL);

//Wire.begin();

Wire.setClock(100000L);

}

byte x = 0;

void loop() {
  uint8_t buf[2];
  delay(500);
//  digitalWrite(RED_LED, HIGH);
  
  Serial.println("Requesting...");
  uint32_t t=millis();  
  //bool res = I2Cdev::writeByte(DEV_ID, 0xAB, x);
  bool res = I2Cdev::readBytes(DEV_ID, 0xAB, 2, buf);
  uint32_t dt=millis()-t;  
  
  Serial.print("R:\t");
  Serial.print(res);
  Serial.print("\tV:\t");
  Serial.print(buf[0], HEX);
  Serial.print(",");
  Serial.print(buf[1], HEX);
  Serial.print("\tT:\t");
  Serial.print(dt);
  Serial.println("ms");
  delay(500);
//  digitalWrite(RED_LED, LOW);
  
  x++;
}

