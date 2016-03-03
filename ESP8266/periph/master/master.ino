#include <Wire.h>
#include "I2Cdev.h"

const int MPU_SDA=0;
const int MPU_SDL=2;
const int DEV_ID=4;

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("Init Wire...");

    Wire.begin(MPU_SDA, MPU_SDL);    
}

void loop() {
  uint8_t buf[1];
  delay(1000);
  
  Serial.println("Requesting...");
  uint32_t t=millis();  
  //bool res = I2Cdev::writeByte(DEV_ID, 0x01, 0x02);
  bool res = I2Cdev::readByte(DEV_ID, 0x01, buf);
  uint32_t dt=millis()-t;  
  
  Serial.print("R:\t");
  Serial.print(res);
  Serial.print("\tV:\t");
  Serial.print(buf[0]);
  Serial.print("\tT:\t");
  Serial.print(dt);
  Serial.println("ms");
}
