#include <Wire.h>

// MOTOR OUT

#define M2_OUT_1  P1_4
#define M2_OUT_2  P1_3
#define M2_EN     P2_1 // analog write

#define M1_OUT_1  P2_4
#define M1_OUT_2  P2_3
#define M1_EN     P2_5 // analog write

// ENC IN
#define ENC2_IN   P1_5
#define ENC1_IN   P2_0

// USSENS
#define US_IN      P2_2 // try to tie all of them to one echo pin
#define US_1_OUT   P1_0
#define US_2_OUT   P2_6   // XTAL remove sel
#define US_3_OUT   P2_7   // XTAL remove sel

#define V_NORM 10000
#define V_NORM_PI2 62832L

#define  PID_TIMEOUT 100
#define  CMD_TIMEOUT 600 
#define  WHEEL_CHGSTATES 40
#define  WHEEL_RAD_MM   33 // measured 32

// for 7.5v
#define M_POW_LOWEST_LIM   10
#define M_POW_HIGH_LIM 100
#define M_POW_MAX  120
#define M_POW_STEP 2

#define M_PID_KP_0   25
#define M_PID_KD_0  140
#define M_PID_KI_0    2
#define M_PID_DIV   50

#define M_PID_KP M_PID_KP_0
#define M_PID_KD M_PID_KD_0
#define M_PID_KI M_PID_KI_0

#define M_WUP_PID_CNT 3

#define REG_TARG_ROT_RATE_1  0x01  // signed int (2 bytes)
#define REG_TARG_ROT_RATE_2  0x02  // signed int (2 bytes)
#define REG_TARG_ROT_RATE    0x03  // 2 signed ints (4 bytes)

#define CHGST_TO_MM(CNT)  ((int32_t)(CNT)*V_NORM_PI2*WHEEL_RAD_MM/WHEEL_CHGSTATES/V_NORM)
#define CHGST_TO_ANG_NORM(CNT)  ((int32_t)(CNT)*V_NORM_PI2/WHEEL_CHGSTATES)
#define CHGST_TO_RPS_NORM(CNT, MSEC)  ((int32_t)(CNT)*V_NORM*1000/WHEEL_CHGSTATES/(MSEC))

// volatile encoder section
volatile uint8_t v_enc_cnt[2]={0,0}; 
volatile uint8_t v_es[2]={0,0};

uint8_t  drv_dir[2]={0,0}; // (0,1,2) - NO, FWD, REV
uint16_t targ_rot_rate[2]={0,0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM)

uint16_t act_rot_rate[2]={0,0}; // OUT - actual rate, 10000 = 1 RPS  (use DRV_RPS_NORM)
int16_t act_adv_accu_mm[2]={0,0};  // OUT - in mm, after last request. Should be zeored after get request

uint8_t enc_cnt[2]={0,0}; 

// PID section
uint16_t pid_cnt=0;
int16_t int_err[2]={0,0};
int16_t  prev_err[2]={0,0};
uint8_t cur_power[2]={0,0};

// 
int16_t targ_new_rot_rate[2]={0, 0}; // RPS, 10000 = 1 RPS  (use DRV_RPS_NORM), +/-

uint32_t lastEvTime, lastPidTime;
uint8_t setEvent = 0, getEvent = 0, eventRegister = 0;
uint8_t isDriving=0;

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
  int ports[9]={M1_OUT_1,M1_OUT_2,M2_OUT_1,M2_OUT_2, M1_EN, M2_EN, US_1_OUT, US_2_OUT, US_3_OUT};
  for(i=0;i<9;i++) {
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
  
  Serial.println("Init Wire...");
    
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event  
  
  analogFrequency(32); 
  
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
      //if(isDriving) stopDrive();
    } 
    
    readEnc();
    if (isDriving) doPID(ctime);    
    readUSDist(); 
    
    lastPidTime=cycleTime;
  } // PID cycle 
          
  if(setEvent) {
    Serial.print("Set Reg ");
    Serial.println(eventRegister);
    setEvent=0;
  } 
  if(getEvent) {
    Serial.print("Get Reg ");
    Serial.println(eventRegister);
    getEvent=0;
  } 
}

void receiveEvent(int howMany)
{
  /*
  boolean set=false;
  while(Wire.available()>0) {
    int c = Wire.read();
    if(i==0) reg=(int)c;
    else if(i==1) { val=(int)c; set=true; }
    i++;
  }*/
  if(Wire.available()==0) return;
  eventRegister=Wire.read();
  if(Wire.available()==0) return;
  
  setEvent=1;  
  lastEvTime = millis();
}
  
void requestEvent()
{
  getEvent=1;
  lastEvTime = millis();
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

void readEnc()
{
  int16_t s[2];
  for(uint8_t i=0; i<2; i++) {
    enc_cnt[i]=v_enc_cnt[i];
    v_enc_cnt[i] = 0;
    if(drv_dir[i]==2) s[i]=-enc_cnt[i];
    else s[i]=enc_cnt[i];
    act_adv_accu_mm[i]+=(int16_t)(CHGST_TO_MM(s[i]));
  }
}

void doPID(uint16_t ctime)
{
  if(ctime>0) {
    uint8_t i;
    
    Serial.print(pid_cnt); 
    
    for(i=0; i<2; i++) {      
      int16_t p_err=0, d_err;
      Serial.print(i==0 ? " L: " : " R: ");
      act_rot_rate[i]=CHGST_TO_RPS_NORM(enc_cnt[i], ctime); 
      Serial.print(act_rot_rate[i]);
      if(pid_cnt>=M_WUP_PID_CNT) { // do not correct for the first cycles - ca 100-200ms(warmup)
        p_err = (targ_rot_rate[i]-act_rot_rate[i]);
        d_err = p_err-prev_err[i];
        int_err[i]=int_err[i]+p_err;
        Serial.print(" , "); Serial.print(p_err);
        Serial.print(" , "); Serial.print(d_err);
        Serial.print(" , "); Serial.print(int_err[i]);
        int16_t pow=cur_power[i]+((int16_t)p_err*M_PID_KP+(int16_t)int_err[i]*M_PID_KI+(int16_t)d_err*M_PID_KD)/M_PID_DIV;
        if(pow<0) pow=0;
        if(pow>M_POW_MAX) pow=M_POW_MAX;
        if(cur_power[i]!=pow) analogWrite(i==0 ? M1_EN : M2_EN , pow); 
        cur_power[i]=pow;
        
        Serial.print(" > "); Serial.print(pow);
      }
      prev_err[i]=p_err;
    } 
    pid_cnt++;
    Serial.println();
  }
}

void readUSDist() {
  /*
  int16_t tmp = us_dist;
  digitalWrite(US_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OUT, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OUT, LOW);
  us_dist=(int16_t)(pulseIn(US_IN, HIGH, 25000)/58);  
  if(!us_dist) {
    us_dist = tmp;
    return;
  }
  if(F_ISDRIVE()) {
      int8_t adv=0, ada=0;
      int16_t usd=0;
      adv=task.adv_d/100; //cm
      ada=RADN_TO_GRAD(task.adv_a); //grad
      
      if(tmp!=0xFFF)
        usd=tmp-us_dist;              
      else 
        usd=0;  

      for(uint8_t i=WALL_LOG_SZ-1; i>=1; i--) logw[i]=logw[i-1];

      if(logw[1].adv_k!=-127) {
        usd=( ((int16_t)usd*SENS_K_K+(int16_t)logw[1].usd_k*(SENS_K_DIV-SENS_K_K))/SENS_K_DIV ); // Kalman
        adv=(int8_t)( ((int16_t)adv*SENS_K_K+(int16_t)logw[1].adv_k*(SENS_K_DIV-SENS_K_K))/SENS_K_DIV ); // Kalman
        ada=(int8_t)( ((int16_t)ada*SENS_K_K+(int16_t)logw[1].ada_k*(SENS_K_DIV-SENS_K_K))/SENS_K_DIV ); // Kalman
      }
      logw[0].usd_k=usd;
      logw[0].adv_k=(int8_t)adv;      
      logw[0].ada_k=(int8_t)ada;
      
      uint8_t i;
      tmp=0;
      for(i=0; i<WALL_LOG_SZ; i++) if(logw[i].adv_k!=-127) tmp+=abs(logw[i].adv_k-logw[i].usd_k);
      tmp/=WALL_LOG_SZ; 
              
      if(tmp<4) { 
        us_dist_ver=us_dist; // verified dist // todo: add a check that the pows[] are comparable )
        bad_us_cnt=0;
      }
      else {
        //if(bad_us_cnt==0 && abs(logw[0].adv_k-logw[0].usd_k)>50) { // wild reflection ???
        if(bad_us_cnt==0 && abs(logw[0].usd_k)>50) { // wild reflection ???
          logw[0].usd_k=logw[1].usd_k; // use old val
        } else us_dist_ver = 0;
        bad_us_cnt++;        
      }
      
  }
  */
}

