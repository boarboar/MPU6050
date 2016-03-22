#include <Wire.h>
#include "I2Cdev.h"
#include "controller.h"
// working OK

const int MPU_SDA=0;
const int MPU_SDL=2;
const int DEV_ID=4;

//uint8_t buf[16];
uint32_t t, dt;
int16_t sens[8];
bool stat;

Controller& ctrl = Controller::ControllerProc; 

void setup() {
//  digitalWrite(RED_LED, LOW); 
//  pinMode(RED_LED, OUTPUT);
  
  delay(2000);
  Serial.begin(115200);
    //Serial.begin(9600);
  Serial.println("Init Wire...");
  Wire.begin(MPU_SDA, MPU_SDL);
    //Wire.begin();
  Serial.print("Testing connection..."); 
  stat=ctrl.init();
  if(stat) Serial.println("OK");
  else Serial.println("FAIL");
  
  Serial.print("NSens:"); 
  Serial.println(ctrl.getNumSensors());
  Serial.println("Setting target rate...");
  int16_t d[2]={5000, -5000};
  ctrl.setTargRotRate(d);
  d[0]=d[1]=0;
  Serial.println("Getting target rate...");
  ctrl.getTargRotRate(d);
  Serial.print(d[0]); Serial.print("\t "); Serial.println(d[1]);
}

uint16_t x = 0;

void loop() {

  delay(500);
//  digitalWrite(RED_LED, HIGH);

  //Serial.println("===");
  int16_t d[2];

  if(x==3000) {
    Serial.println("Stopping...");
    ctrl.stopDrive();
  }
  if(x<6000) {
    Serial.println("Processing...");
    ctrl.process();     
    Serial.print("RR:"); Serial.print(ctrl.getStoredRotRate()[0]); Serial.print("\t "); Serial.println(ctrl.getStoredRotRate()[1]);
    Serial.print("AD:"); Serial.print(ctrl.getStoredAdvance()[0]); Serial.print("\t "); Serial.println(ctrl.getStoredAdvance()[1]);
    Serial.print("SS:");
    for(int i=0; i<8; i++) {
      Serial.print(" "); Serial.print(ctrl.getStoredSensors()[i]);
    }
    Serial.println();
  }
  delay(500);
//  digitalWrite(RED_LED, LOW);
  x+=1000;
  
}

