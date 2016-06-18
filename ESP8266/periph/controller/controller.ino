#include <Wire.h>

//#define _SIMULATION_ 1
//#define _PID_DEBUG_ 

#define _MOTOR_ONE_WIRE_ 

#define M_OWN_ID 0x53

// MOTOR OUT

#define M1_OUT_1  P1_4
#ifndef _MOTOR_ONE_WIRE_
  #define M1_OUT_2  P1_3
#endif
#define M1_EN     P2_1 // analog write

#define M2_OUT_1  P2_4
#ifndef _MOTOR_ONE_WIRE_
  #define M2_OUT_2  P2_3
#endif  
#define M2_EN     P2_5 // analog write

// ENC IN
#define ENC2_IN   P1_5
#define ENC1_IN   P2_0

// USSENS
#define US_IN      P2_2 // try to tie all of them to one echo pin
#define US_1_OUT   P1_0
#define US_2_OUT   P2_6   // XTAL remove sel
#define US_3_OUT   P2_7   // XTAL remove sel

#define V_NORM 10000
#define V_NORM_MAX 30000
#define V_NORM_PI2 62832L

#define  PID_TIMEOUT 200
#define  CMD_TIMEOUT 6000 // !!!! 
#define  WHEEL_CHGSTATES 40
#define  WHEEL_RAD_MM   33 // measured 32

// for 7.5v
#define M_POW_LOWEST_LIM   10
#define M_POW_HIGH_LIM 100
//#define M_POW_MAX  120
#define M_POW_MAX  240
#define M_POW_STEP 2

// RPS 0.4 -> 60-100 POW

//#define M_PID_NORM 1000
#define M_PID_NORM 500
#define M_PID_KP_0   12
//#define M_PID_KD_0  140
#define M_PID_KD_0  100
#define M_PID_KI_0    1
#define M_PID_DIV   25

#define M_PID_KP M_PID_KP_0
#define M_PID_KD M_PID_KD_0
#define M_PID_KI M_PID_KI_0

#define M_WUP_PID_CNT 3

#define M_SENS_N       3 // number of sensors 
#define M_SENS_CYCLE   1 // number of sensors to read at cycle

#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_ACT_POW          0x0A  // 2 signed ints (4 bytes)
#define REG_SENSORS_CNT      0x20  // 1 unsigned byte
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define CHGST_TO_MM(CNT)  ((uint32_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM/WHEEL_CHGSTATES/V_NORM)
#define CHGST_TO_ANG_NORM(CNT)  ((uint32_t)(CNT)*V_NORM_PI2/WHEEL_CHGSTATES)
#define CHGST_TO_RPS_NORM(CNT, MSEC)  ((uint32_t)(CNT)*V_NORM*1000/WHEEL_CHGSTATES/(MSEC))

uint32_t lastEvTime, lastPidTime;
int16_t sens[M_SENS_N];
uint8_t sens_fail_cnt[M_SENS_N];
int16_t targ_rot_rate[2]={0,0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM)
int16_t targ_new_rot_rate[2]={0, 0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM), +/-
int16_t targ_old_rot_rate[2]={0, 0}; // prev
int16_t act_rot_rate[2]={0,0}; // OUT - actual rate, 10000 = 1 RPS  (use DRV_RPS_NORM)
int16_t act_adv_accu_mm[2]={0,0};  // OUT - in mm, after last request. Should be zeored after get request

uint8_t  drv_dir[2]={0,0}; // (0,1,2) - NO, FWD, REV
//uint8_t enc_cnt[2]={0,0}; 

// PID section
uint16_t pid_cnt=0;
int16_t int_err[2]={0,0};
int16_t  prev_err[2]={0,0};
uint8_t cur_power[2]={0,0};

// 
uint8_t setEvent = 0, getEvent = 0, eventRegister = 0;
uint8_t isDriving=0;

uint8_t current_sens=0;

uint8_t buffer[16];

// volatile encoder section
volatile uint16_t v_enc_cnt[2]={0,0}; 
volatile uint8_t v_es[2]={0,0};

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
  int ports[9]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN, US_1_OUT, US_2_OUT, US_3_OUT};
  const int portlen=9;
#else
  int ports[7]={M1_OUT_1,M2_OUT_1,M1_EN, M2_EN, US_1_OUT, US_2_OUT, US_3_OUT};
  const int portlen=7;
#endif
  for(i=0;i<portlen;i++) {
    digitalWrite(ports[i], LOW); 
    pinMode(ports[i], OUTPUT);
  }

  pinMode(ENC1_IN, INPUT);   
  pinMode(ENC2_IN, INPUT);   
  pinMode(US_IN, INPUT);   
  
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
  for(int i=0; i<M_SENS_N; i++) { sens[i]=0; sens_fail_cnt[i]=0; }
  
  Serial.println("Ready");
  
  lastEvTime = lastPidTime = millis();  
}

void loop()
{
  uint32_t cycleTime = millis();
  if ( cycleTime < lastPidTime) lastPidTime=0; // wraparound, not correct   
  uint16_t ctime = cycleTime - lastPidTime;
  if ( ctime >= PID_TIMEOUT) { // PID cycle    
    if(cycleTime < lastEvTime) lastEvTime=0; // wraparound, not correct   
    if(cycleTime - lastEvTime >= CMD_TIMEOUT) {
      // comm lost!
      Serial.println("Comm lost!");
      lastEvTime = cycleTime;
      if(isDriving) stopDrive();
    }    
    readEnc(ctime);
    if (isDriving) doPID(ctime);       
    readUSDist(); 
    lastPidTime=cycleTime;
  } // PID cycle 
          
  if(setEvent) {
    Serial.print("SetReg ");
    Serial.print(eventRegister);
    Serial.print(" : ");
    if(eventRegister==REG_TARG_ROT_RATE) {
      Serial.print(targ_new_rot_rate[0]);
      Serial.print(", ");
      Serial.println(targ_new_rot_rate[1]);
      if(targ_new_rot_rate[0] || targ_new_rot_rate[1])
        startDrive();    
      else   
        stopDrive();    
    }
    Serial.println();
    setEvent=0;
  } 
  if(getEvent) {
    //Serial.print("GetReg ");
    //Serial.println(eventRegister);
    getEvent=0;
  } 
}


void startDrive() {
  if(isDriving && targ_old_rot_rate[0]==targ_new_rot_rate[0] && targ_old_rot_rate[1]==targ_new_rot_rate[1]) {
    Serial.print("Continue drive"); 
    return;
  }
  
  Serial.print("Start drive: "); 
   
  for(int i=0; i<2; i++) {
    boolean changeDir=false; 
    //uint16_t oldRotRate=targ_rot_rate[i];
    if(targ_new_rot_rate[i]==0) {
      changeDir=true;
      drv_dir[i]=0;
      targ_rot_rate[i]=0;
    } else if(targ_new_rot_rate[i]>0) {
      changeDir=drv_dir[i]!=1;
      drv_dir[i]=1;
      targ_rot_rate[i]=(uint16_t)(targ_new_rot_rate[i]);
    } else {
      changeDir=drv_dir[i]!=2;
      drv_dir[i]=2;
      targ_rot_rate[i]=(uint16_t)(-targ_new_rot_rate[i]);
    }
    if(targ_rot_rate[i]>V_NORM_MAX) targ_rot_rate[i]=V_NORM_MAX;
    
    if(drv_dir[i]) {
       if(changeDir) {         
         cur_power[i]=map(targ_rot_rate[i], 0, V_NORM_MAX, 0, 255); // temp
       }
       else {
         // let PID do the job
         // todo - PID cycle for i-motor here
       }  
     } else cur_power[i]=0;
    
    targ_old_rot_rate[i]=targ_new_rot_rate[i];     
    prev_err[i]=0;
    int_err[i]=0;   
    Serial.print(changeDir ? "RST" : "PID"); Serial.print(", ");
    Serial.print(drv_dir[i]); Serial.print(", "); Serial.print(targ_rot_rate[i]); Serial.print(", "); Serial.print(cur_power[i]); Serial.print("\t : ");
  }
  Serial.println();
   
  readEnc(0);
  Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); 
  isDriving=true;
  pid_cnt=0;
  lastPidTime=millis(); 
}

void stopDrive() {
  if(!isDriving) return;
  Drive(0, 0, 0, 0);
  cur_power[0]=cur_power[1]=0;
  isDriving=0;
  pid_cnt=0;
  Serial.println("Stop drive"); 
}

void readEnc(uint16_t ctime)
{
  int16_t s[2];
  for(int i=0; i<2; i++) {
    /*
    enc_cnt[i]=v_enc_cnt[i];
    v_enc_cnt[i] = 0;
    if(drv_dir[i]==2) s[i]=-enc_cnt[i];
    else s[i]=enc_cnt[i];
    */
    s[i]=v_enc_cnt[i];
    v_enc_cnt[i] = 0;
    
    
    if(ctime>0) {
      //act_rot_rate[i]=CHGST_TO_RPS_NORM(enc_cnt[i], ctime); 
      act_rot_rate[i]=CHGST_TO_RPS_NORM(s[i], ctime); 
      /*
#ifdef _SIMULATION_
#if _SIMULATION_ == 0
      // test interface
      act_rot_rate[i]=(2*i-1)*100;
      act_adv_accu_mm[i]=(2*i-1)*10;
#else      
      //#error "NOT SUPPORTED"  
      act_rot_rate[i] = (uint32_t)targ_rot_rate[i]*cur_power[i]/196;
      act_rot_rate[i] += random(1000)/2;
      if(act_rot_rate[i]<0) act_rot_rate[i]=0;
      int16_t cnt=(uint32_t)act_rot_rate[i]*ctime*WHEEL_CHGSTATES/1000/V_NORM;
      if(drv_dir[i]==2) s[i]=-cnt;
      else s[i]=cnt;
      //Serial.print("SIM_");Serial.print(i);Serial.print(" \t");Serial.print(act_rot_rate[i]); Serial.println(" \t");Serial.print(s[i]);     
#endif      
#endif
*/
      int16_t mov=(int16_t)(CHGST_TO_MM(s[i]));
      if(drv_dir[i]==2) mov=-mov;      
      act_adv_accu_mm[i]+=mov;
      //act_adv_accu_mm[i]+=(int16_t)(CHGST_TO_MM(s[i]));
    } else { // ctime==0 
      act_rot_rate[i]=0;      
    }
  } // for i
}

void doPID(uint16_t ctime)
{
  if(ctime>0) {
    int i;  
#ifdef _PID_DEBUG_
    Serial.print(pid_cnt); Serial.print(" "); Serial.print(ctime);
#endif
    for(i=0; i<2; i++) {      
      int16_t p_err=0, d_err;
#ifdef _PID_DEBUG_
      Serial.print(i==0 ? " L: " : " R: ");
      //act_rot_rate[i]=CHGST_TO_RPS_NORM(enc_cnt[i], ctime); 
      Serial.print(act_rot_rate[i]);
#endif      
      if(pid_cnt>=M_WUP_PID_CNT) { // do not correct for the first cycles - ca 100-200ms(warmup)
        p_err = (int32_t)(targ_rot_rate[i]-act_rot_rate[i])/M_PID_NORM;
        d_err = p_err-prev_err[i];
        int_err[i]=int_err[i]+p_err;
        int16_t pow=cur_power[i]+((int16_t)p_err*M_PID_KP+(int16_t)int_err[i]*M_PID_KI+(int16_t)d_err*M_PID_KD)/M_PID_DIV;
        if(pow<0) pow=0;
        if(pow>M_POW_MAX) pow=M_POW_MAX;
        if(cur_power[i]!=pow) analogWrite(i==0 ? M1_EN : M2_EN , pow); 
        cur_power[i]=pow;
#ifdef _PID_DEBUG_        
        Serial.print(" , "); Serial.print(p_err);
        Serial.print(" , "); Serial.print(d_err);
        Serial.print(" , "); Serial.print(int_err[i]);        
        Serial.print(" > "); Serial.print(pow);
#endif        
      }
      prev_err[i]=p_err;
    } 
    pid_cnt++;
#ifdef _PID_DEBUG_            
    Serial.println();
#endif    
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


void readUSDist() {
  static bool ignore=false;
  //for(uint8_t i=0; i<M_SENS_CYCLE; i++) {
  if(digitalRead(US_IN)==HIGH) {
    //Serial.print("US read abort: "); Serial.println(current_sens);
    delay(50);
    ignore=true;
    return; // TODO reset here ?
  }
  // bad sensor strategy - skip next time ??? TODO
  
  int ports[M_SENS_N]={US_1_OUT, US_2_OUT, US_3_OUT};
  int out_port=ports[current_sens];
  digitalWrite(out_port, LOW);
  delayMicroseconds(2); // or 5?
  digitalWrite(out_port, HIGH);
  delayMicroseconds(10);
  digitalWrite(out_port, LOW);
  
  // actual constant should be 58.138
  //int16_t tmp =(int16_t)(pulseIn(US_IN, HIGH, 18000)/58);  //58.138
  int16_t tmp =(int16_t)(pulseIn(US_IN, HIGH, 40000)/58);  //play with timing ?
  
  if(ignore) {
    //Serial.println("Ignored");
    ignore=false;
    return;
  }
  
  /*
  Serial.print("\t\t\t\t\t");
  for(int j=0; j<current_sens; j++) Serial.print("\t");
   Serial.println(tmp);
  */
  
  if(tmp) {    
    sens_fail_cnt[current_sens] = 0;
    if(sens[current_sens]<=0) sens[current_sens] = tmp;
    else {
       // do LPM filter here ?
       //sens[current_sens] = (sens[current_sens]*2 - (sens[current_sens] - tmp))/2;
       sens[current_sens] = tmp;
    }
    //Serial.print("U."); Serial.print(current_sens); Serial.print("=");Serial.print(sens[current_sens]);Serial.print(" \tRAW="); Serial.println(tmp);
  } else {    
    sens_fail_cnt[current_sens]++;
    if(sens_fail_cnt[current_sens]>1) // this is to avoid one-time reading failures
      sens[current_sens] = -1;
      
    if(digitalRead(US_IN)==HIGH) { // need to reset
      sens[current_sens] = -2;
      //delay(50);
      ignore=true;
      /*
      //play with timing ?
      delay(100);
      pinMode(US_IN, OUTPUT);
      digitalWrite(US_IN, LOW);
      delay(100);
      pinMode(US_IN, INPUT);
      if(digitalRead(US_IN)==HIGH) { Serial.print("US Reset failed: "); Serial.println(current_sens);}
      else { Serial.print("US Reset OK: "); Serial.println(current_sens);}
      */
    }
      
    if(sens_fail_cnt[current_sens]>8) sens_fail_cnt[current_sens]=8; 
#ifdef _SIMULATION_
    sens[current_sens] = current_sens*100+random(50);
#endif
  }
  current_sens=(current_sens+1)%M_SENS_N;  
  //}
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
  if(Wire.available()==0) return;
  eventRegister=Wire.read();
  if(Wire.available()==0) return; 
  switch(eventRegister) {
    case REG_TARG_ROT_RATE:
      readInt16(targ_new_rot_rate);
      readInt16(targ_new_rot_rate+1);
      break;
    default:;
  }  
  while(Wire.available()) Wire.read(); // consume whatever left  
  setEvent=1;  
  lastEvTime = millis();
}
  
void requestEvent()
{
  if(!eventRegister) return;
  
  switch(eventRegister) {
    case REG_WHO_AM_I:  
      Wire.write((uint8_t)M_OWN_ID);
      break;
    case REG_TARG_ROT_RATE:
      writeInt16_2(targ_new_rot_rate);
      break;
    case REG_ACT_ROT_RATE:
      writeInt16_2(act_rot_rate);
      break;      
    case REG_ACT_ADV_ACC:
      writeInt16_2(act_adv_accu_mm);
      act_adv_accu_mm[0]=act_adv_accu_mm[1]=0;
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
  getEvent=1;
  lastEvTime = millis();
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

