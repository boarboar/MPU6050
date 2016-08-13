#include <Wire.h>
#include <Servo.h> 

//#define _SIMULATION_ 1
#define _PID_DEBUG_ 
//#define _US_DEBUG_ 

#define _US_M_WIRE_  // multiple input wires 

#define _MOTOR_ONE_WIRE_ 

#define M_OWN_ID 0x53
//#define M_MAGIC_ID 0x4C

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
/*
#ifdef _US_M_WIRE_
  #define US_1_IN      P2_2 
  #define US_2_IN      P1_3
  #define US_3_IN      P2_3
#else
  #define US_IN      P2_2 // try to tie all of them to one echo pin
#endif
*/

#ifdef _US_M_WIRE_
  #define US_1_IN      P2_2 
  #define US_2_IN      P2_3
  #define US_3_IN      P1_3
#else
  #define US_IN      P2_2 // try to tie all of them to one echo pin
#endif

#define SERVO_IN P1_3

// USSENS
#define US_1_OUT   P1_0
#define US_2_OUT   P2_6   // XTAL remove sel
#define US_3_OUT   P2_7   // XTAL remove sel

#define V_NORM 10000
#define V_NORM_MAX 30000
#define V_NORM_PI2 62832L

#define  PID_TIMEOUT 200
#define  CMD_TIMEOUT 1000 // !!!! 
#define  WHEEL_CHGSTATES 40
#define  WHEEL_RAD_MM   33 // measured 32

// for 3.7v
//#define M_POW_LOWEST_LIM   10
//#define M_POW_HIGH_LIM 100
//#define M_POW_MAX  120
#define M_POW_MIN_0  28 // left calibration
#define M_POW_MIN_1  32 // right calibration
#define M_POW_MAX  240
#define M_POW_STEP 2

// RPS 0.5 -> 50-70 POW

/*
#define M_PID_NORM 500
#define M_PID_KP_0   12
#define M_PID_KD_0  200
#define M_PID_KI_0    1
#define M_PID_DIV   25
*/

#define M_PID_KP_0   25
#define M_PID_KD_0  140
#define M_PID_KI_0    2
#define M_PID_DIV    50
#define M_STEER_DIV  10

#define M_PID_KP M_PID_KP_0
#define M_PID_KD M_PID_KD_0
#define M_PID_KI M_PID_KI_0

#define M_WUP_PID_CNT 1

#define M_SENS_N       3 // number of sensors
#define M_SENS_CYCLE   1 // number of sensors to read at cycle

#define REG_WHO_AM_I         0xFF  // 1 unsigned byte
#define REG_STATUS           0x01  // 2 unsigned bytes
#define REG_START            0x02  // 1 unsigned bytes
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)
#define REG_ACT_ROT_RATE     0x06  // 2 signed ints (4 bytes)
#define REG_ACT_ADV_ACC      0x09  // 2 signed ints (4 bytes)
#define REG_ACT_POW          0x0A  // 2 signed ints (4 bytes)
#define REG_TARG_POW         0x0B  // 2 signed ints (4 bytes)
#define REG_STEERING         0x0C  // 1 signed int (2 bytes)
#define REG_SENSORS_CNT      0x20  // 1 unsigned byte
#define REG_SENSORS_ALL      0x28  // 8 unsigned ints

#define ST_DRIVE             0x01 
//#define ST_SETEV             0x02 
//#define ST_GETEV             0x04
#define ST_START             0x08

#define ST_SET_DRIVE_ON()       (sta[0] |= ST_DRIVE)
#define ST_SET_DRIVE_OFF()      (sta[0] &= ~ST_DRIVE)
#define ST_IS_DRIVING()         (sta[0]&ST_DRIVE)  
/*
#define ST_SET_SETEV_ON()       (sta[0] |= ST_SETEV)
#define ST_SET_SETEV_OFF()      (sta[0] &= ~ST_SETEV)
#define ST_IS_SETEV()           (sta[0]&ST_SETEV)  

#define ST_SET_GETEV_ON()       (sta[0] |= ST_GETEV)
#define ST_SET_GETEV_OFF()      (sta[0] &= ~ST_GETEV)
#define ST_IS_GETEV()           (sta[0]&ST_GETEV)  
*/
#define ST_SET_START_ON()       (sta[0] |= ST_START)
#define ST_SET_START_OFF()      (sta[0] &= ~ST_START)
#define ST_IS_STARTED()         (sta[0]&ST_START)  


#define NSETQ 2

#define CHGST_TO_MM(CNT)  ((uint32_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM/WHEEL_CHGSTATES/V_NORM)
#define CHGST_TO_ANG_NORM(CNT)  ((uint32_t)(CNT)*V_NORM_PI2/WHEEL_CHGSTATES)
#define CHGST_TO_RPS_NORM(CNT, MSEC)  ((uint32_t)(CNT)*V_NORM*1000/WHEEL_CHGSTATES/(MSEC))
#define RPS_TO_CHGST_NORM(RPS, MSEC)  ((uint32_t)(RPS)*WHEEL_CHGSTATES*(MSEC)/V_NORM/1000)

#define NPOW_CHART_N     6
#define NPOW_CHART_MULT  2

Servo sservo;

//uint8_t M_POW_MIN[2]={M_POW_MIN_0, M_POW_MIN_1}; 

uint32_t lastEvTime, lastPidTime;
int16_t sens[M_SENS_N];
uint8_t sens_fail_cnt[M_SENS_N];
//int16_t targ_rot_rate[2]={0,0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM)
int16_t targ_new_param[2]={0, 0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM), +/-
//int16_t targ_old_rot_rate[2]={0, 0}; // prev
//int16_t act_rot_rate[2]={0,0}; // OUT - actual rate, 10000 = 1 RPS  (use DRV_RPS_NORM)
volatile int32_t act_adv_accu_mm[2]={0,0};  // OUT - in mm, after last request. Should be zeored after get request
uint8_t targ_enc_cnt[2]={0,0}; 
uint8_t  drv_dir[2]={0,0}; // (0,1,2) - NO, FWD, REV
uint8_t enc_cnt[2]={0,0}; 
//int16_t steering=0;

// PID section
/*
uint16_t pid_cnt=0;
int16_t int_err[2]={0,0};
int16_t  prev_err[2]={0,0};
*/
uint8_t cur_power[2]={0,0};
//uint8_t pow_chart[2][NPOW_CHART_N];

// 
//volatile uint8_t setEvent = 0, getEvent = 0;
volatile uint8_t sta[2]={0,0};
//volatile uint8_t setRegister = 0;
volatile uint8_t getRegister = 0;
volatile uint8_t getOverflow=0;
volatile uint8_t setOverflow=0;

uint8_t current_sens=0;

uint8_t buffer[16];

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

int16_t sservo_pos=0;
int8_t sservo_step=30;

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
  
#ifdef _US_M_WIRE_
  pinMode(US_1_IN, INPUT);   
  pinMode(US_2_IN, INPUT);   
  pinMode(US_3_IN, INPUT);   
#else
  pinMode(US_IN, INPUT);   
#endif

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
  
  /*
  for(int i=0; i<2; i++) 
    for(int j=0; j<NPOW_CHART_N; j++) pow_chart[i][j]=0;
  */
  
  Serial.println("Init Servo...");
  sservo.attach(SERVO_IN);  // attaches the servo on pin 9 to the servo object
  sservo.write(0);


  // init Q
  setQuInit();
  
  setQuPrint();
  
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
      if(ST_IS_DRIVING()) stopDrive();
    } 
    
    readEnc(ctime);
      //if (ST_IS_DRIVING()) doPID(ctime);   
    if(ST_IS_STARTED()) {
             
      readUSDist();
    } 
    lastPidTime=cycleTime;
  } // PID cycle 
          
  struct set_s sp;
  uint8_t qsz=setQuGet(&sp);
  /*  
  if(setRegister) {    
    Serial.print("SetReg "); Serial.print(setRegister); Serial.print(" : ");
    switch(setRegister) {
      //case REG_TARG_ROT_RATE: 
      case REG_TARG_POW: 
        Serial.print(targ_new_param[0]); Serial.print(", "); Serial.print(targ_new_param[1]);
        if(targ_new_param[0] || targ_new_param[1]) {
          //if(setRegister==REG_TARG_ROT_RATE) startDrive();    
          //else 
          startDrivePow();  
        }
        else   
          stopDrive();    
        break;  
      case REG_STEERING: 
        Serial.print(steering);    
      default:;    
    }
    Serial.println();
    setRegister=0;   
  }
 */
 
  if(qsz) {
    setQuPrint();   
    Serial.print("SetReg "); Serial.print(sp.r); Serial.print("\t: "); Serial.print(sp.p[0]); Serial.print("\t, "); Serial.print(sp.p[1]);
    switch(sp.r) {
      case REG_START:     
        if(sp.p[0])  
          ST_SET_START_ON();  
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
        /*
      case REG_STEERING:
        steering=sp.p[0];
        //Serial.print(steering);    
        */
        
      default:;    
    }
    Serial.println();
  }
 
 if(getOverflow) {Serial.println("===========Get overflow!"); getOverflow=0;}     
 if(setOverflow) {Serial.println("===========Set overflow!"); setOverflow=0;}
 if(qsz>1) {Serial.print("===========QSZ "); Serial.println(qsz);}
/*
  if(ST_IS_GETEV()){     
    //Serial.print("GetReg ");
    //Serial.println(eventRegister);
    //if(eventRegister==REG_STATUS) Serial.println("===STAT requested"); 
    //getEvent=0;
    ST_SET_GETEV_OFF();
  }
 */ 
}

void startDrivePow() {
  /*
  if(ST_IS_DRIVING() && cur_power[0]==targ_new_param[0] && cur_power[1]==targ_new_param[1]) {
    Serial.print("Continue drive"); 
    return;
  }
  */
  
  uint8_t chg=0;
  uint8_t new_dir=0;
  Serial.print("St drv pow: "); 
   
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
      
    Serial.print(i==0 ? "\t L: " : "\t R: ");
    //Serial.print(changeDir ? " RST" : " PID"); Serial.print(", ");
    Serial.print(drv_dir[i]); Serial.print("\t "); Serial.print(cur_power[i]);
    Serial.print("\t ;"); 
  }
  
   if(chg || !ST_IS_DRIVING()) {
     Serial.print(" >>RST");    
    //readEnc(0);
    Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); 
    ST_SET_DRIVE_ON();
    //pid_cnt=0;
    //lastPidTime=millis(); 
   }
   
  Serial.println();
  
}

/*
void startDrive() {
  if(ST_IS_DRIVING() && targ_old_rot_rate[0]==targ_new_param[0] && targ_old_rot_rate[1]==targ_new_param[1]) {
    Serial.print("Continue drive"); 
    return;
  }
  
  Serial.print("Start drive: "); 
   
  for(int i=0; i<2; i++) {
    uint8_t jt=0;
    if(targ_new_param[i]==0) {
      drv_dir[i]=0;
      targ_rot_rate[i]=0;
    } else if(targ_new_param[i]>0) {
      drv_dir[i]=1;
      targ_rot_rate[i]=(uint16_t)(targ_new_param[i]);
    } else {
      drv_dir[i]=2;
      targ_rot_rate[i]=(uint16_t)(-targ_new_param[i]);
    }
    if(targ_rot_rate[i]>V_NORM_MAX) targ_rot_rate[i]=V_NORM_MAX;
    targ_enc_cnt[i]=RPS_TO_CHGST_NORM(targ_rot_rate[i], PID_TIMEOUT);
    if(targ_rot_rate[i]>0 && targ_enc_cnt[i]==0) targ_enc_cnt[i]=1;
    
    if(drv_dir[i]) {
       if(targ_old_rot_rate[i]!=targ_new_param[i])  {      
         uint8_t j=targ_enc_cnt[i]/NPOW_CHART_MULT;
         Serial.print("Tsting chart for ");
         Serial.print(targ_enc_cnt[i]);
         Serial.print(" => ");
         Serial.print(j);
         jt=0;
         if(j<NPOW_CHART_N) {
           Serial.print(" in ");
           Serial.print(pow_chart[i][j]);
           if(pow_chart[i][j]) { jt=j; Serial.print(" hit; "); }
           else  if(j-1>=0 && pow_chart[i][j-1]) jt=j-1;
           else  if(j+1<NPOW_CHART_N && pow_chart[i][j+1]) jt=j+1;
         }
         if(jt) {
           cur_power[i]=pow_chart[i][jt];
         }  else {  
           cur_power[i]=map(targ_rot_rate[i], 0, V_NORM_MAX, 0, 255); 
           if(cur_power[i]<M_POW_MIN[i]) cur_power[i]=M_POW_MIN[i];
         }           
         //cur_power[i]=map(targ_rot_rate[i], 0, V_NORM_MAX, M_POW_MIN[i], 255); // temp
       }
     } else cur_power[i]=0;
    
    targ_old_rot_rate[i]=targ_new_param[i];     
    prev_err[i]=0;
    int_err[i]=0;   
    //uint8_t chgst=RPS_TO_CHGST_NORM(targ_rot_rate[i], PID_TIMEOUT);
    Serial.print(i==0 ? "\t L: " : "\t R: ");
    //Serial.print(changeDir ? " RST" : " PID"); Serial.print(", ");
    Serial.print(drv_dir[i]); Serial.print("\t "); Serial.print(targ_rot_rate[i]); Serial.print("\t "); Serial.print(cur_power[i]);
    Serial.print("\t "); Serial.print(targ_enc_cnt[i]); Serial.print("\t ");
    Serial.print(jt); Serial.print("\t ;"); 
  }
  Serial.println();
   
  readEnc(0);
  Drive(drv_dir[0], cur_power[0], drv_dir[1], cur_power[1]); 
  ST_SET_DRIVE_ON();
  pid_cnt=0;
  lastPidTime=millis(); 
}
*/

void stopDrive() {
  if(!ST_IS_DRIVING()) return;
  Drive(0, 0, 0, 0);
  cur_power[0]=cur_power[1]=0;
  //targ_old_rot_rate[0]=targ_rot_rate[0]=0;     
  //targ_old_rot_rate[1]=targ_rot_rate[1]=0;
  ST_SET_DRIVE_OFF();
  Serial.println("Stop drive"); 
  /*
  pid_cnt=0;
  Serial.println("Chart:");
  for(int j=0; j<NPOW_CHART_N; j++) {
    Serial.print(j*NPOW_CHART_MULT); Serial.print("\t ");
  }
  Serial.println();
  for(int i=0; i<2; i++) {
    for(int j=0; j<NPOW_CHART_N; j++) {
      Serial.print(pow_chart[i][j]); Serial.print("\t ");
    }
    Serial.println();
  }
  */
}

void readEnc(uint16_t ctime)
{
    //Serial.print("T1:\t ");
    //Serial.println(ctime);
  for(int i=0; i<2; i++) {
    enc_cnt[i]=v_enc_cnt[i]; 
    v_enc_cnt[i] = 0;          
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
      uint8_t alr=0;
      uint16_t alrp=0;
      
      //while(updating);
      //updating=1;
      
      if(enc_cnt[i]>128) {
        alr = 1<<i;
        alrp = enc_cnt[i];
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
      //updating=0;
      
      if(alr) {
          Serial.print("!!!!!!ALR-"); Serial.print(i); Serial.print("\t on "); Serial.print(i); Serial.print("\t : "); Serial.println(alrp);      
      }
    
  } // for i
  
  if(ST_IS_DRIVING()) {
    Serial.print(ctime);
    Serial.print(" \t"); Serial.print(enc_cnt[0]);Serial.print(" \t");Serial.println(enc_cnt[0]);
    //Serial.print(" \t"); Serial.print(act_adv_accu_mm[0]);Serial.print(" \t");Serial.println(act_adv_accu_mm[1]);      
  }
}

/*
void doPID(uint16_t ctime)
{
  if(ctime>0) {
    int8_t i;  
#ifdef _PID_DEBUG_
    Serial.print(pid_cnt); Serial.print("\t"); Serial.print(ctime);
#endif
    for(i=0; i<2; i++) {            
      int8_t steer_g;
      int16_t p_err=0, d_err;
#ifdef _PID_DEBUG_
      Serial.print(i==0 ? "\t L: " : "\t R: ");
      //act_rot_rate[i]=CHGST_TO_RPS_NORM(enc_cnt[i], ctime); 
      Serial.print(act_rot_rate[i]);
      Serial.print("\t, "); Serial.print(enc_cnt[i]);
#endif      
      if(pid_cnt>=M_WUP_PID_CNT) { // do not correct for the first cycles - ca 100-200ms(warmup)      
        // fill chart
        uint8_t j=enc_cnt[i]/NPOW_CHART_MULT;
        if(j<NPOW_CHART_N) {          
          if(pow_chart[i][j]) pow_chart[i][j]=(uint8_t)(((uint16_t)pow_chart[i][j]+cur_power[i])/2);
          else pow_chart[i][j]=cur_power[i];
        }
        steer_g=steering/M_STEER_DIV;
        if(steer_g>0 && steer_g>targ_enc_cnt[i]/2) steer_g=targ_enc_cnt[i]/2;
        if(steer_g<0 && steer_g<-targ_enc_cnt[i]/2) steer_g=-targ_enc_cnt[i]/2; 
        if(i) steer_g=-steer_g;        
        p_err = (int16_t)targ_enc_cnt[i]-(int16_t)enc_cnt[i]+steer_g;
        d_err = p_err-prev_err[i];
        int_err[i]=int_err[i]+p_err;        
        int16_t pow=cur_power[i]+((int16_t)p_err*M_PID_KP+(int16_t)int_err[i]*M_PID_KI+(int16_t)d_err*M_PID_KD)/M_PID_DIV;        
        if(pow<0) pow=0;
        if(drv_dir[i] && pow<M_POW_MIN[i]) pow=M_POW_MIN[i];
        if(pow>M_POW_MAX) pow=M_POW_MAX;
        
        if(cur_power[i]!=pow) analogWrite(i==0 ? M1_EN : M2_EN , pow); 
        cur_power[i]=pow;
                
#ifdef _PID_DEBUG_        
        Serial.print("\t, "); Serial.print(p_err);
        Serial.print("\t, "); Serial.print(d_err);
        Serial.print("\t, "); Serial.print(int_err[i]);        
        Serial.print("\t, "); Serial.print(steer_g);        
        Serial.print("\t > "); Serial.print(pow);
#endif        
      }
      prev_err[i]=p_err;
    } 
    pid_cnt++;
#ifdef _PID_DEBUG_
    Serial.print("\t STA: "); Serial.print(sta[0]); Serial.print("\t "); Serial.print(sta[1], BIN);        
    Serial.println();
#endif    
  }
}
*/

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

/*
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
*/

// ORIN2
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
  //uint32_t t=millis();
  sservo.write(sservo_pos);
  //uint32_t dt=millis()-t;
  //Serial.print(t); Serial.print(" \t"); Serial.println(sservo_pos); 
  sservo_pos+=sservo_step;
  if((sservo_step>0 && sservo_pos>=180) || (sservo_step<0 && sservo_pos<=0)) sservo_step=-sservo_step;
  /*
#ifdef _US_M_WIRE_
  //int ports_in[M_SENS_N]={US_1_IN, US_2_IN, US_3_IN};
  int ports_in[M_SENS_N]={US_1_IN, US_2_IN};
#else
  int ports_in[M_SENS_N]={US_IN, US_IN, US_IN};
#endif  
  //int ports[M_SENS_N]={US_1_OUT, US_2_OUT, US_3_OUT};
  int ports[M_SENS_N]={US_1_OUT, US_2_OUT};
  int out_port=ports[current_sens];
  int in_port=ports_in[current_sens];
  static bool ignore=false;
  //for(uint8_t i=0; i<M_SENS_CYCLE; i++) {
  if(digitalRead(in_port)==HIGH) {
    //Serial.print("US read abort: "); Serial.println(current_sens);
    delay(50);
    ignore=true;
    return; // TODO reset here ?
  }
  // bad sensor strategy - skip next time ??? TODO
  
  digitalWrite(out_port, LOW);
  delayMicroseconds(2); // or 5?
  digitalWrite(out_port, HIGH);
  delayMicroseconds(10);
  digitalWrite(out_port, LOW);
  
  // actual constant should be 58.138
  int16_t tmp =(int16_t)(pulseIn(in_port, HIGH, 40000)/58);  //play with timing ?
  
  if(ignore) {
    //Serial.println("Ignored");
    ignore=false;
    return;
  }
  
#ifdef _US_DEBUG_  
  Serial.print("\t\t\t\t\t");
  for(int j=0; j<current_sens; j++) Serial.print("\t");
   Serial.println(tmp);
#endif  
  
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
      
    if(digitalRead(in_port)==HIGH) { // need to reset
      sens[current_sens] = -2;
      //delay(50);
      ignore=true;
      
      //play with timing ?
      //delay(50);
      //pinMode(US_IN, OUTPUT);
      //digitalWrite(US_IN, LOW);
      //delay(50);
      //pinMode(US_IN, INPUT);
     
      
      //if(digitalRead(US_IN)==HIGH) { Serial.print("US Reset failed: "); Serial.println(current_sens);}
      //else { Serial.print("US Reset OK: "); Serial.println(current_sens);}
      
    }
      
    if(sens_fail_cnt[current_sens]>8) sens_fail_cnt[current_sens]=8; 
#ifdef _SIMULATION_
    sens[current_sens] = current_sens*100+random(50);
#endif
  }
  current_sens=(current_sens+1)%M_SENS_N;  
  //}
  */
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
  
uint8_t setQuAdd(uint8_t r, uint8_t p1, uint8_t p2) {
  uint8_t ovf=0;
  //while(qlock);
  /*
  qlock=1;
  uint8_t i=0;
  while(i<=NSETQ-1 && set_q[i].r) i++;  
  if(i==NSETQ) {
    ovf=1;
    i=NSETQ-1;
  }
  set_q[i].r=r;
  set_q[i].p[0]=p1;
  set_q[i].p[1]=p2;
  qlock=0;
  */
  if(set_q[set_t].r) ovf=1;
  set_q[set_t].p[0]=p1;
  set_q[set_t].p[1]=p2;
  set_q[set_t].r=r;
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


/*
void writeInt16_2_v(int16_t v1, int16_t v2) {
  uint16_t v1u=(uint16_t)v1;
  uint16_t v2u=(uint16_t)v2;
  buffer[0] = (uint8_t)((v1u)>>8);
  buffer[1] = (uint8_t)((v1u)&0xFF);
  buffer[2] = (uint8_t)((v2u)>>8);
  buffer[3] = (uint8_t)((v2u)&0xFF);  
  Wire.write(buffer, 4);
}
*/

/*
void writeInt16_2_v_x(int16_t v1, int16_t v2) {
  buffer[0] = (uint8_t)((v1)>>8);
  buffer[1] = (uint8_t)((v1)&0xFF);
  buffer[2] = (uint8_t)((v2)>>8);
  buffer[3] = (uint8_t)((v2)&0xFF);  
  buffer[4] = M_MAGIC_ID;
  Wire.write(buffer, 5);
}
*/

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

