
#define _MOTOR_ONE_WIRE_ 

// MOTOR OUT

#define M2_OUT_1  P1_4
#ifndef _MOTOR_ONE_WIRE_
  #define M2_OUT_2  P1_3
#endif
#define M2_EN     P2_1 // analog write

#define M1_OUT_1  P2_4
#ifndef _MOTOR_ONE_WIRE_
  #define M1_OUT_2  P2_3
#endif  
#define M1_EN     P2_5 // analog write

// ENC IN
#define ENC2_IN   P1_5
#define ENC1_IN   P2_0

uint8_t enc_cnt[2]={0,0}; 

// volatile encoder section
volatile uint8_t v_enc_cnt[2]={0,0}; 
volatile uint8_t v_es[2]={0,0};

void setup()
{
  
  uint8_t i;
#ifndef _MOTOR_ONE_WIRE_  
  int ports[9]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN};
  const int portlen=6;
#else
  int ports[7]={M1_OUT_1,M2_OUT_1,M1_EN, M2_EN};
  const int portlen=4;
#endif
  for(i=0;i<portlen;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }

  pinMode(ENC1_IN, INPUT);   
  pinMode(ENC2_IN, INPUT);   
  
  // encoders interrupts
  attachInterrupt(ENC1_IN, encodeInterrupt_1, CHANGE); 
  attachInterrupt(ENC2_IN, encodeInterrupt_2, CHANGE); 
  
  delay(1000);
    
  analogFrequency(32); 
  
  Serial.println("Go...");
 
  Drive(1, 100, 1, 100); 
 
  delay(200);
  
  Drive(0, 0, 0, 0); 
  
  delay(200);
  
  readEnc(400);
  
  Serial.print("Enc \t");  Serial.print(enc_cnt[0]);  Serial.print("\t\t");  Serial.print(enc_cnt[1]);  
  
  Drive(2, 100, 2, 100); 
 
  delay(200);
  
  Drive(0, 0, 0, 0); 
  
  delay(200);
  
  readEnc(400);
  
  Serial.print("Enc \t");  Serial.print(enc_cnt[0]);  Serial.print("\t\t");  Serial.print(enc_cnt[1]);  
  
} 

void loop()
{
   
   
}



void readEnc(uint16_t ctime)
{
  for(int i=0; i<2; i++) {
    enc_cnt[i]=v_enc_cnt[i];
    v_enc_cnt[i] = 0;
  }
}


void Drive(uint8_t ldir, uint8_t lpow, uint8_t rdir, uint8_t rpow) 
{
#ifndef _MOTOR_ONE_WIRE_  
  Drive_s(ldir, lpow, M1_EN, M1_OUT_1, M1_OUT_2);
  Drive_s(rdir, rpow, M2_EN, M2_OUT_1, M2_OUT_2);
#else
  Drive_s1(ldir, lpow, M1_EN, M1_OUT_1);
  Drive_s1(rdir, rpow, M2_EN, M2_OUT_1);
#endif  
}

void Drive_s(uint8_t dir, uint8_t pow, int16_t p_en, uint8_t p1, uint8_t p2) 
{
  if(dir==0 || pow==0) {
    digitalWrite(p_en, LOW); 
    digitalWrite(p1, LOW); digitalWrite(p2, LOW); 
    return;
  }
  else if(dir==1) {
    digitalWrite(p1, LOW); digitalWrite(p2, HIGH); 
  }
  else {
    digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
  } 
  analogWrite(p_en, pow);
}

void Drive_s1(uint8_t dir, uint8_t pow, int16_t p_en, uint8_t p1) 
{
  if(dir==0 || pow==0) {
    digitalWrite(p_en, LOW); 
    return;
  }
  else if(dir==1) {
    digitalWrite(p1, LOW); 
  }
  else {
    digitalWrite(p1, HIGH); 
  } 
  analogWrite(p_en, pow);
}


void encodeInterrupt_1() { baseInterrupt(0); }

void encodeInterrupt_2() { baseInterrupt(1); } 

void baseInterrupt(uint8_t i) {
  const uint8_t encp[]={ENC1_IN, ENC2_IN};
  uint8_t v=digitalRead(encp[i]);  
  if(v_es[i]==v) return;
  v_es[i]=v;  
  /*if(v_enc_cnt[i]==255) F_SETOVERFLOW();
  else*/
  v_enc_cnt[i]++; 
} 

