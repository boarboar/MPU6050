#include <Wire.h>
#include "I2Cdev.h"
// working OK

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
  uint8_t buf[4];
  bool res;
  uint32_t t, dt;
  delay(500);
//  digitalWrite(RED_LED, HIGH);

  if(x%2) {
    int16_t left=x;
    int16_t right=-x;
    buf[0] = (uint8_t)(left>>8);
    buf[1] = (uint8_t)(left&0xFF);   
    buf[2] = (uint8_t)(right>>8);
    buf[3] = (uint8_t)(right&0xFF);   

    Serial.print("Writing...\t ");
    t=millis();  
    bool res = I2Cdev::writeBytes(DEV_ID, 0x03, 4, buf);
    dt=millis()-t;
    Serial.print(dt); Serial.println("ms");
  } else {
    Serial.print("Requesting...\t ");
    t=millis();  
    bool res = I2Cdev::readBytes(DEV_ID, 0x03, 4, buf);
    dt=millis()-t;
    Serial.print(dt); Serial.print("ms\t ");
    int16_t left = (((int16_t)buf[0]) << 8) | buf[1];
    int16_t right = (((int16_t)buf[2]) << 8) | buf[3];
    Serial.print(left); Serial.print("\t "); Serial.println(right);
  }
  
  
/*
  I2Cdev::writeByte(DEV_ID, 2, x%2);
  I2Cdev::writeByte(DEV_ID, 3, (x+1)%2);
  I2Cdev::writeByte(DEV_ID, 1, (x*10)%256);
  */
  
/*
  Serial.println("Requesting...\t ");
  bool res;
  uint32_t t, dt;
  
  t=millis();  
  res = I2Cdev::readBytes(DEV_ID, 0x01, 1, buf);
  res = I2Cdev::readBytes(DEV_ID, 0x02, 1, buf+1);
  res = I2Cdev::readBytes(DEV_ID, 0x03, 1, buf+2);
  dt=millis()-t;

         for(int i=0; i<3; i++) { Serial.print(buf[i]); Serial.print("\t "); }
Serial.println();
*/

  //Serial.print("Byte  U8:\t"); Serial.print(res); Serial.print("\tV:\t"); Serial.print(buf[0]); Serial.print("\t in "); Serial.print(dt); Serial.println("ms");

  /*
  Serial.println("Requesting...");
  bool res;
  uint32_t t, dt;
  t=millis();  
  res = I2Cdev::readBytes(DEV_ID, 0x01, 1, buf);
  dt=millis()-t;   
  Serial.print("Byte  U8:\t"); Serial.print(res); Serial.print("\tV:\t"); Serial.print(buf[0]); Serial.print("\t in "); Serial.print(dt); Serial.println("ms");

  t=millis();  
  res = I2Cdev::readBytes(DEV_ID, 0x02, 2, buf);
  uint16_t w = (((uint16_t)buf[0]) << 8) | buf[1];
  dt=millis()-t;   
  Serial.print("Word U16:\t"); Serial.print(res); Serial.print("\tV:\t"); Serial.print(w); Serial.print("\t in "); Serial.print(dt); Serial.println("ms");

  t=millis();  
  res = I2Cdev::readBytes(DEV_ID, 0x03, 1, buf);
  dt=millis()-t;
  int8_t i8= (int8_t)buf[0];   
  Serial.print("Byte  I8:\t"); Serial.print(res); Serial.print("\tV:\t"); Serial.print(i8); Serial.print("\t in "); Serial.print(dt); Serial.println("ms");

  t=millis();  
  res = I2Cdev::readBytes(DEV_ID, 0x04, 2, buf);
  int16_t i16 = (((int16_t)buf[0]) << 8) | buf[1];
  dt=millis()-t;   
  Serial.print("Word I16:\t"); Serial.print(res); Serial.print("\tV:\t"); Serial.print(i16); Serial.print("\t in "); Serial.print(dt); Serial.println("ms");
*/
  delay(500);
//  digitalWrite(RED_LED, LOW);
  
  x++;
}

