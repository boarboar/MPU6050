#include <Wire.h>
#include "I2Cdev.h"
// working OK

const int MPU_SDA=0;
const int MPU_SDL=2;
const int DEV_ID=4;

uint8_t buf[16];
uint32_t t, dt;
int16_t sens[8];

void setup() {
//  digitalWrite(RED_LED, LOW); 
//  pinMode(RED_LED, OUTPUT);
  
  delay(2000);
    Serial.begin(115200);
    //Serial.begin(9600);
    Serial.println("Init Wire...");

    Wire.begin(MPU_SDA, MPU_SDL);

//Wire.begin();

//Wire.setClock(100000L);

uint8_t nsens=0;
I2Cdev::readByte(DEV_ID, 0x20, &nsens); 
Serial.print("NSens:"); Serial.println(nsens);

int16_t d[2]={5000, -5000};
writeInt16_2(0x03, d); // set targ rot rate
readInt16_2(0x03, d);  // get targ rot rate
}

uint16_t x = 0;

void loop() {

  delay(500);
//  digitalWrite(RED_LED, HIGH);

  Serial.println("===");
  int16_t d[2];
  readInt16_2(0x06, d); // get act rot rate
  readInt16_2(0x09, d); // get advance
  readInt16_N(0x28, 8, sens);  // get sens

    
  //Serial.print("Byte  U8:\t"); Serial.print(res); Serial.print("\tV:\t"); Serial.print(buf[0]); Serial.print("\t in "); Serial.print(dt); Serial.println("ms");
  delay(500);
//  digitalWrite(RED_LED, LOW);
  x+=1000;
  
}

bool writeInt16_2(uint16_t reg, int16_t *d) {
    int16_t left=d[0];
    int16_t right=d[1];
    buf[0] = (uint8_t)(left>>8);
    buf[1] = (uint8_t)(left&0xFF);   
    buf[2] = (uint8_t)(right>>8);
    buf[3] = (uint8_t)(right&0xFF);   
    Serial.print("Writing...\t ");
    t=millis();  
    bool res = I2Cdev::writeBytes(DEV_ID, reg, 4, buf);
    dt=millis()-t;
    Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.println("ms");
    return res;
}

bool readInt16_2(uint16_t reg, int16_t *d) {
    Serial.print("Requesting...\t ");
    t=millis();  
    bool res = I2Cdev::readBytes(DEV_ID, reg, 4, buf);
    dt=millis()-t;
    Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.print("ms\t ");
    *d = (((int16_t)buf[0]) << 8) | buf[1];
    *(d+1) = (((int16_t)buf[2]) << 8) | buf[3];
    Serial.print(d[0]); Serial.print("\t "); Serial.println(d[1]);
    return res;
}


bool readInt16_N(uint16_t reg, uint16_t n, int16_t *d) {
    Serial.print("Requesting...\t ");
    t=millis();  
    bool res = I2Cdev::readBytes(DEV_ID, reg, n*2, buf);
    dt=millis()-t;
    Serial.print(res); Serial.print(" "); Serial.print(dt); Serial.print("ms\t ");
    for(uint16_t i=0, j=0; i<n; i++) {
      *(d+i) = (((int16_t)buf[j++]) << 8) | buf[j++];
      Serial.print(d[i]); Serial.print("\t ");
    }
    return res;
}

