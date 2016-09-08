#include <Wire.h>
#include <Servo.h> 

//#define _SIMULATION_ 1
//#define _US_DEBUG_ 

#define _US_M_WIRE_  // multiple input wires 

#define _MOTOR_ONE_WIRE_ 

#define M_OWN_ID 0x53

// MOTOR OUT

#define M1_OUT_1  P1_4
//#ifndef _MOTOR_ONE_WIRE_
//  #define M1_OUT_2  P1_3
//#endif
#define M1_EN     P2_1 // analog write

#define M2_OUT_1  P2_4
//#ifndef _MOTOR_ONE_WIRE_
//  #define M2_OUT_2  P2_3
//#endif  
#define M2_EN     P2_5 // analog write

// ENC IN
#define ENC2_IN   P1_5
#define ENC1_IN   P2_0

// USSENS

#define   US_1_IN      P2_2 
#define   US_2_IN      P2_3

#define   US_1_OUT   P1_0
#define   US_2_OUT   P2_6   // XTAL remove sel

#define   SERVO_IN P1_3

#define  CYCLE_TIMEOUT 100
#define  CMD_TIMEOUT 1000 // !!!! 


#define   V_NORM 10000
#define   V_NORM_MAX 30000
#define   V_NORM_PI2 62832L
#define    WHEEL_CHGSTATES 40
#define    WHEEL_RAD_MM   33 // measured 32

#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_STATUS           0x01  // 2 unsigned bytes
#define REG_START            0x02  // 1 unsigned bytesc
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_ACT_POW          0x0A  // 2 signed ints (4 bytes)
#define REG_TARG_POW         0x0B  // 2 signed ints (4 bytes)
#define REG_STEERING         0x0C  // 1 signed int (2 bytes)
#define REG_SENSORS_CNT      0x20  // 1 unsigned byte
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define ST_DRIVE             0x01 
#define ST_START             0x08

#define ST_SET_DRIVE_ON()       (sta[0] |= ST_DRIVE)
#define ST_SET_DRIVE_OFF()      (sta[0] &= ~ST_DRIVE)
#define ST_IS_DRIVING()         (sta[0]&ST_DRIVE)  
#define ST_SET_START_ON()       (sta[0] |= ST_START)
#define ST_SET_START_OFF()      (sta[0] &= ~ST_START)
#define ST_IS_STARTED()         (sta[0]&ST_START)  


#define NSETQ 2

#define CHGST_TO_MM(CNT)  ((uint32_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM/WHEEL_CHGSTATES/V_NORM)
#define CHGST_TO_ANG_NORM(CNT)  ((uint32_t)(CNT)*V_NORM_PI2/WHEEL_CHGSTATES)
#define CHGST_TO_RPS_NORM(CNT, MSEC)  ((uint32_t)(CNT)*V_NORM*1000/WHEEL_CHGSTATES/(MSEC))
#define RPS_TO_CHGST_NORM(RPS, MSEC)  ((uint32_t)(RPS)*WHEEL_CHGSTATES*(MSEC)/V_NORM/1000)

//#define NPOW_CHART_N     6
//#define NPOW_CHART_MULT  2

/*
#define SERVO_NSTEPS  1
#define SERVO_TOT_STEPS  3
#define SERVO_STEP    60
#define SERVO_ZERO_SHIFT    5
*/

#define SERVO_NSTEPS  2
//#define SERVO_TOT_STEPS  5
#define SERVO_STEP    36
#define SERVO_ZERO_SHIFT    5
#define M_SENS_N      10 // number of sensors
#define M_SENS_CNT    2 

Servo sservo;

uint32_t lastEvTime, lastPidTime;
//uint8_t sens_fail_cnt[M_SENS_N];
int16_t targ_new_param[2]={0, 0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM), +/-
int32_t act_adv_accu_mm[2]={0,0};  // OUT - in mm, after last request.
//uint8_t targ_enc_cnt[2]={0,0}; 
uint8_t  drv_dir[2]={0,0}; // (0,1,2) - NO, FWD, REV
uint8_t enc_cnt[2]={0,0}; 

uint8_t cur_power[2]={0,0};
// 
volatile uint8_t sta[2]={0,0};
volatile uint8_t getRegister = 0;
volatile uint8_t getOverflow=0;
volatile uint8_t setOverflow=0;

//uint8_t buffer[16];
uint8_t buffer[20];

// volatile encoder section
volatile uint8_t v_enc_cnt[2]={0,0}; 
volatile uint8_t v_es[2]={0,0};

volatile uint8_t updating=0;
//volatile uint8_t qlock=0;

struct set_s {
  uint8_t r;
  int16_t p[2];
};

struct set_s set_q[NSETQ];
volatile uint8_t set_h, set_t;

;
int16_t sens[M_SENS_N];
uint8_t sens_step=1;
int8_t sservo_pos=0; //90
int8_t sservo_step=1; 
uint8_t uscount=0;

void setup()
{
  // prepare ports
  // push I2C to high (ESP8266 issue)
  digitalWrite(P1_6, HIGH); 
  digitalWrite(P1_7, HIGH); 
  // disable XTAL to get extra pins for US triggers
  P2SEL &= ~BIT6; 
  P2SEL &= ~BIT7;
  
  uint8_t i;
#ifndef _MOTOR_ONE_WIRE_  
  int ports[8]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN, US_1_OUT, US_2_OUT};
  const int portlen=9;
#else
  int ports[6]={M1_OUT_1,M2_OUT_1,M1_EN, M2_EN, US_1_OUT, US_2_OUT};
  const int portlen=7;
#endif
  for(i=0;i<portlen;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }

  pinMode(ENC1_IN, INPUT);   
  pinMode(ENC2_IN, INPUT);   
  
  pinMode(US_1_IN, INPUT);   
  pinMode(US_2_IN, INPUT);   

  // encoders interrupts
  attachInterrupt(ENC1_IN, encodeInterrupt_1, CHANGE); 
  attachInterrupt(ENC2_IN, encodeInterrupt_2, CHANGE); 
  
  delay(1000);
  Serial.begin(9600);
  
#ifdef   _SIMULATION_
  delay(5000);
  Serial.println("===SIMULATION===");
#endif  

  Serial.println("Init Wire...");
    
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event  
  
  analogFrequency(32); 
  
  for(int i=0; i<M_SENS_N; i++) { sens[i]=-1; /*sens_fail_cnt[i]=0;*/ }
    
  Serial.println("Init Servo...");
  sservo.attach(SERVO_IN);  // attaches the servo on pin 9 to the servo object
  delay(500);
  sservo.write(90-SERVO_ZERO_SHIFT);
  // init Q
  setQuInit();
  
  setQuPrint();
  
  Serial.println("Ready");
  
  uscount=0;
  
  lastEvTime = lastPidTime = millis();  
}

void loop()
{
  uint32_t cycleTime = millis();
  if ( cycleTime < lastPidTime) lastPidTime=0; // wraparound, not correct   
  uint16_t ctime = cycleTime - lastPidTime;
  if ( ctime >= CYCLE_TIMEOUT) { // PID cycle    
    if(cycleTime < lastEvTime) lastEvTime=0; // wraparound, not correct   
    if(cycleTime - lastEvTime >= CMD_TIMEOUT) {
      // comm lost!
      Serial.println("Comm lost!");
      lastEvTime = cycleTime;
      if(ST_IS_DRIVING()) stopDrive();
    } 
    
    readEnc(ctime);
    
    if(ST_IS_STARTED()) {
      /*
      uscount++;
      if(uscount==M_SENS_CNT) // commented for US test purposes
      {            
        readUSDist();
        uscount=0;
      } 
      */
      readUSDist();
    }
    lastPidTime=cycleTime;
  } // PID cycle 
          
  struct set_s sp;
  uint8_t qsz=setQuGet(&sp);
 
  if(qsz) {
    //setQuPrint();   
    Serial.print("SetReg "); Serial.print(sp.r); Serial.print("\t: "); Serial.print(sp.p[0]); Serial.print("\t, "); Serial.println(sp.p[1]);
    switch(sp.r) {
      case REG_START:     
        if(sp.p[0])  {
          ST_SET_START_ON();
          //uscount=M_SENS_CNT-1;
          Serial.println("Start"); 
        }  
        break;
      case REG_TARG_POW:         
        targ_new_param[0]=sp.p[0];
        targ_new_param[1]=sp.p[1];
        if(targ_new_param[0] || targ_new_param[1]) {
          startDrivePow();  
        }
        else   
          stopDrive();    
        break;         
      default:;    
    }
    Serial.println();
  }
 
 if(getOverflow) {Serial.println("===========Get overflow!"); getOverflow=0;}     
 if(setOverflow) {Serial.println("===========Set overflow!"); setOverflow=0;}
 if(qsz>1) {Serial.print("===========QSZ "); Serial.println(qsz);} 
}

void startDrivePow() {
  
  uint8_t chg=0;
  uint8_t new_dir=0;
  //Serial.print("St drv pow: "); 
   
  for(int i=0; i<2; i++) {
    if(targ_new_param[i]==0) new_dir=0;
    else if(targ_new_param[i]>0) new_dir=1;
    else {
      new_dir=2;
      targ_new_param[i]=-targ_new_param[i];
    }
    
    if(drv_dir[i] != new_dir || cur_power[i] != targ_new_param[i]) chg++;
    
    drv_dir[i] = new_dir;
    cur_power[i]=targ_new_param[i];
    targ_new_param[i]=0;
    
    /*  
    Serial.print(i==0 ? "\t L: " : "\t R: ");
    //Serial.print(changeDir ? " RST" : " PID"); Serial.print(", ");
    Serial.print(drv_dir[i]); Serial.print("\t "); Serial.print(cur_power[i]);
    Serial.print("\t ;"); 
    */
  }
  
   if(chg || !ST_IS_DRIVING()) {     
    //readEnc(0);
    Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); 
    ST_SET_DRIVE_ON();
    /*
    //Serial.print(" >>RST");    
    for(int i=0; i<2; i++) {
      Serial.print(i==0 ? "\t L: " : "\t R: ");
      //Serial.print(changeDir ? " RST" : " PID"); Serial.print(", ");
      //Serial.print(drv_dir[i]); Serial.print("\t "); 
      Serial.print(cur_power[i]);
      Serial.print("\t ;"); 
    }
    Serial.println();
    */
   }  
}


void stopDrive() {
  if(!ST_IS_DRIVING()) return;
  Drive(0, 0, 0, 0);
  cur_power[0]=cur_power[1]=0;
  ST_SET_DRIVE_OFF();
  Serial.println("Stop drive"); 
}

void readEnc(uint16_t ctime)
{
    //Serial.print("T1:\t ");
    //Serial.println(ctime);
  for(int i=0; i<2; i++) {
      enc_cnt[i]=v_enc_cnt[i]; 
      v_enc_cnt[i] = 0;          
      uint8_t alr=0;
      //uint16_t alrp=0;
      
      //while(updating);
      //updating=1;
      
      if(enc_cnt[i]>128) {
        alr = 1<<i;
        //alrp = enc_cnt[i];
      }

      int16_t mov=(int16_t)(CHGST_TO_MM(enc_cnt[i]));
      if(drv_dir[i]==2) mov=-mov;
      
      while(updating);
      updating=1;
      act_adv_accu_mm[i]+=mov;
      updating=0;
/*
      if(abs(act_adv_accu_mm[i])>512) {
        alr = 4<<i;
        alrp = act_adv_accu_mm[i];
      }
      */
      if(alr) sta[1]=alr;
      /*
      if(alr) {
          Serial.print("!!!!!!ALR-"); Serial.print(i); Serial.print("\t on "); Serial.print(i); Serial.print("\t : "); Serial.println(alrp);      
      }
      */
  } // for i
  
  if(ST_IS_DRIVING()) {
    //Serial.print(ctime);
    //Serial.print(" \t"); Serial.print(enc_cnt[0]);Serial.print(" \t");Serial.println(enc_cnt[0]);
    //Serial.print(" \t"); Serial.print(act_adv_accu_mm[0]);Serial.print(" \t");Serial.println(act_adv_accu_mm[1]);      
  }
}


void Drive(uint8_t ldir, uint8_t lpow, uint8_t rdir, uint8_t rpow) 
{
  Drive_s1(ldir, lpow, M1_EN, M1_OUT_1);
  Drive_s1(rdir, rpow, M2_EN, M2_OUT_1);
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
    digitalWrite(p1, HIGH); 
  }
  else {
    digitalWrite(p1, LOW); 
  } 
  analogWrite(p_en, pow);
}

void readUSDist() {
  if(sens_step==0) {
    if(uscount==0) {
      Serial.print("S0 "); Serial.print(sservo_pos); Serial.print("\t "); Serial.println(sservo_step);

      if((sservo_step>0 && sservo_pos>=SERVO_NSTEPS) || (sservo_step<0 && sservo_pos<=-SERVO_NSTEPS)) sservo_step=-sservo_step;
      sservo_pos+=sservo_step;
      int16_t sservo_angle=90-SERVO_ZERO_SHIFT+sservo_pos*SERVO_STEP;
      sservo.write(sservo_angle);
      Serial.print("S1 "); Serial.print(sservo_pos); Serial.print("\t "); Serial.println(sservo_step);
#ifdef _US_DEBUG_      
      Serial.print("Servo "); Serial.println(sservo_pos);
      for(int i=0; i<M_SENS_N; i++) { Serial.print(sens[i]); Serial.print("\t "); }
      Serial.println();
#endif    
    } else {
#ifdef _US_DEBUG_      
      Serial.println("Servo wait");
#endif          
    }
    uscount++;
    if(uscount<M_SENS_CNT) return;
    uscount=0;
  } else {
    int out_port=sens_step==1 ? US_1_OUT : US_2_OUT;
    int in_port=sens_step==1 ? US_1_IN : US_2_IN;
    if(digitalRead(in_port)==HIGH) {
#ifdef _US_DEBUG_            
      Serial.print("US read abort: "); Serial.println(sens_step);      
#endif
      //ignore=true;
      return; // TODO reset here ?
    }
    digitalWrite(out_port, LOW);
    delayMicroseconds(2); // or 5?
    digitalWrite(out_port, HIGH);
    delayMicroseconds(10);
    digitalWrite(out_port, LOW);
  
    // actual constant should be 58.138
    int16_t tmp =(int16_t)(pulseIn(in_port, HIGH, 40000)/58);  //play with timing ?
   if(tmp==0) tmp=-1;
   int8_t current_sens=-sservo_pos+1+(sens_step-1)*3;
#ifdef _US_DEBUG_  
/*
    Serial.print("\t\t");
    for(int j=0; j<sens_step; j++) Serial.print("\t");
    Serial.print(tmp);
    Serial.print("\t [");
    Serial.print(sservo_pos);
    Serial.print("\t ,");
    Serial.print(sens_step);
    Serial.print("\t -> ");
    Serial.print(current_sens);
    Serial.println("]");
    */
    Serial.print(current_sens); Serial.print("\t "); Serial.println(tmp);
#endif  

   sens[current_sens] = tmp;
  }
 sens_step=(sens_step+1)%3;  
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

void receiveEvent(int howMany)
{
  uint8_t reg=0;
  int16_t p[2];
  if(Wire.available()==0) return;
  reg=Wire.read();
  if(Wire.available()==0) { 
    if(getRegister) getOverflow=1;
    getRegister=reg; 
    return; 
  }
  p[0]=p[1]=0;
  switch(reg) {
    //case REG_TARG_ROT_RATE:
    case REG_TARG_POW:
      readInt16(p);
      readInt16(p+1);     
      break; 
    case REG_STEERING:
      readInt16(p);
      break; 
    case REG_START:
      p[0]=Wire.read();
      break;        
    default:;
  }  
  while(Wire.available()) Wire.read(); // consume whatever left  
  setOverflow=setQuAdd(reg, p[0], p[1]);
  lastEvTime = millis();
}
  
void requestEvent()
{
  if(!getRegister) return;
  
  switch(getRegister) {
    case REG_WHO_AM_I:  
      Wire.write((uint8_t)M_OWN_ID);
      break;
    case REG_STATUS:
      //while(updating);
      //updating=1;
      buffer[0]=sta[0];
      buffer[1]=sta[1];
      sta[1]=0;
      //updating=0;            
      Wire.write(buffer, 2);     
      break;
      /*
    case REG_TARG_ROT_RATE:
      writeInt16_2(targ_rot_rate);
      break;
    case REG_ACT_ROT_RATE:
      writeInt16_2(act_rot_rate);
      break;      
      */
    case REG_ACT_ADV_ACC:
      //writeInt16_2(act_adv_accu_mm);      
      int32_t t[2];
      while(updating);
      updating=1;
      t[0]=act_adv_accu_mm[0]; t[1]=act_adv_accu_mm[1];    
      updating=0;
      writeInt32_2(t);
      break;            
    case REG_ACT_POW:
      int16_t tmp[2];
      tmp[0]=cur_power[0]; tmp[1]=cur_power[1];    
      writeInt16_2(tmp);
      break;      
    case REG_SENSORS_CNT:  
      Wire.write((uint8_t)M_SENS_N);
      break;
    case REG_SENSORS_ALL:  
      writeInt16_N_M(M_SENS_N, 8, sens);
      break;  
    default:;
  }
  getRegister=0;
  lastEvTime = millis();
}

// Non-blocking queue

void setQuInit() {  
  for(uint8_t i=0; i<NSETQ; i++) set_q[i++].r=0;
  set_h=set_t=0;
}
  
uint8_t setQuAdd(uint8_t r, int16_t p1, int16_t p2) {
  uint8_t ovf=0;
  //while(qlock);
  //qlock=1;
  if(set_q[set_t].r) ovf=1;
  set_q[set_t].p[0]=p1;
  set_q[set_t].p[1]=p2;
  set_q[set_t].r=r;
  //qlock=0;
  set_t++;
  if(set_t==NSETQ) set_t=0;
  return ovf;
}

uint8_t setQuGet(struct set_s *sp) {
  
  /*
  while(qlock);
  qlock=1;
  if(!set_q[0].r) { qlock=0; return 0; }
  *sp=set_q[0];
  for(uint8_t i=1; i<=NSETQ-1; i++) { set_q[i-1]=set_q[i]; if(set_q[i].r) sz++; }
  set_q[NSETQ-1].r=0;
  qlock=0;
  return sz;
  */
  if(!set_q[set_h].r) return 0;
  uint8_t sz=1;
  *sp=set_q[set_h];
  set_q[set_h].r=0;
  set_h++;
  if(set_h==NSETQ) set_h=0;
  if(set_q[set_h].r) sz++;
  return sz;
}

void setQuPrint() {
  uint8_t r[NSETQ]; 
  //while(qlock);
  //qlock=1;
  for(uint8_t i=0; i<NSETQ; i++) r[i]=set_q[i].r;
  //qlock=0;
  uint8_t h=set_h;
  uint8_t t=set_t;
  Serial.print("QST["); Serial.print(h); Serial.print(":"); Serial.print(t);  Serial.print("]: ");  
  
  for(uint8_t i=0; i<NSETQ; i++) { 
    Serial.print("\t "); Serial.print(r[h]); 
    h++;
    if(h==NSETQ) h=0;
  }
  Serial.println();
}

void readInt16(int16_t *reg) {
  if(Wire.available()) buffer[0]=Wire.read();
  if(Wire.available()) buffer[1]=Wire.read();
  *reg = (((int16_t)buffer[0]) << 8) | buffer[1];
}

void writeInt16(int16_t *reg) {
  buffer[0] = (uint8_t)((*reg)>>8);
  buffer[1] = (uint8_t)((*reg)&0xFF);      
  Wire.write(buffer, 2);
}

void writeInt16_2(int16_t *reg) {
  buffer[0] = (uint8_t)((reg[0])>>8);
  buffer[1] = (uint8_t)((reg[0])&0xFF);
  buffer[2] = (uint8_t)((reg[1])>>8);
  buffer[3] = (uint8_t)((reg[1])&0xFF);  
  Wire.write(buffer, 4);
}

void writeInt32(int32_t v) {
  buffer[0] = (uint8_t)(v>>24);
  buffer[1] = (uint8_t)((v>>16)&0xFF);
  buffer[2] = (uint8_t)((v>>8)&0xFF);
  buffer[3] = (uint8_t)((v)&0xFF);  
}

void readInt32(int32_t *reg) {
  *reg = (int32_t)(
  (((int32_t)buffer[0]) << 24) |
  (((int32_t)buffer[1]) << 16) |
  (((int32_t)(buffer[2])) << 8) |
  buffer[3]);
}

void writeInt32_2(int32_t *reg) {
  buffer[0] = (uint8_t)(reg[0]>>24);
  buffer[1] = (uint8_t)((reg[0]>>16)&0xFF);
  buffer[2] = (uint8_t)((reg[0]>>8)&0xFF);
  buffer[3] = (uint8_t)(reg[0]&0xFF);
  buffer[4] = (uint8_t)(reg[1]>>24);
  buffer[5] = (uint8_t)((reg[1]>>16)&0xFF);
  buffer[6] = (uint8_t)((reg[1]>>8)&0xFF);
  buffer[7] = (uint8_t)(reg[1]&0xFF);  
  Wire.write(buffer, 8);
}

void writeInt16_N_M(uint16_t act, uint16_t tot, int16_t *reg) {
  for(uint16_t i=0, j=0; i<tot; i++) {
    if(i<act) {
      buffer[j++] = (uint8_t)((reg[i])>>8);
      buffer[j++] = (uint8_t)((reg[i])&0xFF);
    } else {
      buffer[j++]=0;
      buffer[j++]=0;
    }      
  }
  Wire.write(buffer, tot*2);
}

