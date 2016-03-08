#include <Wire.h>
// working OK

#define TEST_PIN_1     P2_1 // analog write
#define  TEST_PIN_2 P1_4
#define TEST_PIN_3  P1_3

int rcv=0, req=0;
uint32_t llast=0L;
uint8_t cnt=0;
uint8_t reg=0, val=0;
uint8_t regs[3];

void setup()
{  
 digitalWrite(P1_6, HIGH); 
 digitalWrite(P1_7, HIGH); 
 
  digitalWrite(RED_LED, HIGH); 
  pinMode(RED_LED, OUTPUT);
 
 digitalWrite(TEST_PIN_1, LOW);
  pinMode(TEST_PIN_1, OUTPUT);  
 digitalWrite(TEST_PIN_2, LOW);
  pinMode(TEST_PIN_2, OUTPUT);  
 digitalWrite(TEST_PIN_3, LOW);
  pinMode(TEST_PIN_3, OUTPUT);  
  
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
    Serial.print("Rc, regs=\t");
    /*
    Serial.print(cnt);
    Serial.print("\tBytes=\t");
    Serial.print(rcv);
    Serial.print("\tReg=\t");
    Serial.print(reg, HEX);
    Serial.print("\tVal=\t");
    Serial.print(val, HEX);*/
    for(int i=0; i<3; i++) { Serial.print(regs[i]); Serial.print("\t "); }
    Serial.println();
    rcv=0;
    }
    if(req>0) {
      Serial.print("Req reg ");
      Serial.println(reg, HEX);
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
  boolean set=false;
  while(Wire.available()>0) {
    int c = Wire.read();
    if(i==0) reg=(int)c;
    else if(i==1) { val=(int)c; set=true; }
    i++;
  }
  
  if(set) {
  switch(reg) {
    case 1: regs[0]=val; analogWrite(TEST_PIN_1, regs[0]); break;
    case 2: regs[1]=val; digitalWrite(TEST_PIN_2, regs[1]==0 ? LOW: HIGH); break;
  case 3: regs[2]=val; digitalWrite(TEST_PIN_3, regs[2]==0 ? LOW: HIGH); break;
  default:;
  }
  }
  rcv=i;
  cnt++;
}

void requestEvent()
{
  req=1;  
  uint8_t buffer[4];
  
  uint8_t len=1; 
 /* 
  uint8_t reg_0=224;
  uint16_t reg_1=60345;
  int8_t reg_2=-120;
  int16_t reg_3=-30345;
            
  switch(reg) {
    case 0x01: 
      buffer[0]=(uint8_t)reg_0;
      len=1; 
      break;
    case 0x02: 
      buffer[0] = (uint8_t)(reg_1>>8);
      buffer[1] = (uint8_t)(reg_1&0xFF);
      len=2;
      break;
    case 0x03: 
      buffer[0]=(uint8_t)reg_2;
      len=1; 
      break;
    case 0x04: 
      //buffer[0] = (uint8_t)(((uint16_t)reg_3)>>8);
      //buffer[1] = (uint8_t)(((uint16_t)reg_3)&0xFF);
      buffer[0] = (uint8_t)(reg_3>>8);
      buffer[1] = (uint8_t)(reg_3&0xFF);      
      len=2;
      break;
    default:;
  }
*/
  buffer[0]=0;
  if(reg<=3 && reg>0) buffer[0]=regs[reg-1];  
  Wire.write(buffer, len);       // return response to last command  
}


