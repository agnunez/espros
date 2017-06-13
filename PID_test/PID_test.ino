// version 1.0 12-06-2017 00:20 https://github.com/agnunez/espros
#include <ESP8266WiFi.h>
#include <PID_v1.h>
extern "C" {
#include "user_interface.h"
}
#define DEBUG 1

double lpwm=0, rpwm=0;   // motor pwm
int lmc=0,  rmc=0;    // left motor counter
int lmc0=0, rmc0=0;   // last encoder value 
int ldir=1, rdir=1;   // motor direction
int llevel, rlevel;
double whesep= 0.135; // wheel separtion in m
double whedia= 0.7;   // wheel diameter in m
int CPR = 40;         // Encoder Count per Revolutions 
int period=50;        // PID sample timer period in ms
double lv=0., rv=0., lvt=0., rvt=0. ;// motor speed innumber tics per period with two methods
int ti=0;             // tic timer counter
double lIn,rIn,lOut,rOut,lSet=0,rSet=0;   // PID Input Signal, Output command and Setting speed for each wheel 
double lkp=0.5,lki=10,lkd=0.0;     // Left/right wheel PID constants. Can be modiffied while running with:
double rkp=0.5,rki=10,rkd=0.0;     // s nnn (setting point), p nnn (kp), i nnn (ki), d nnn (kd). nnn is divided by 10 to get decimals nnn -> nn.n
double kt=1000/period;            // number of periods/sec
int rcurrenttime, rlasttime, lcurrenttime, llasttime;

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID lPID(&lIn, &lOut, &lSet, lkp, lki, lkd, DIRECT);
PID rPID(&rIn, &rOut, &rSet, rkp, rki, rkd, DIRECT);

os_timer_t myTimer;

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

// start of timerCallback, repeat every "period"
void tic(void *pArg) {
  lv=(lmc-lmc0);     
  lmc0=lmc;
  rv=(rmc-rmc0); 
  rmc0=rmc;
  lIn = lv*kt;
  rIn = rv*kt;
  if(abs(lv)>=1) lIn=abs(lvt)*sgn(lv); // use timing to calculate velocity only if ticks are greater than one
  if(abs(rv)>=1) rIn=abs(rvt)*sgn(rv); // to avoid problems with sign and inertia for a simple encoder (no quadrature)
  lPID.Compute();
  rPID.Compute();
  motion(lOut,rOut); 
  ti+=1;                  // tic counter
}

void stop(void){      // Stop both motors
    analogWrite(D1, 0);
    analogWrite(D2, 0);
}
 
void motion(double lpwm, double rpwm) {  
  if(abs(lIn)<1){
    if(lOut>=0){
      ldir=1; 
      llevel=HIGH; 
    } else {
      ldir=-1;
      llevel=LOW; 
    }
  }
  if(abs(rIn)<1){
    if(rOut>=0){
      rdir=1; 
      rlevel=HIGH; 
    } else {
      rdir=-1;
      rlevel=LOW; 
    }
  }
  analogWrite(D1, abs(lpwm));
  analogWrite(D2, abs(rpwm));
  digitalWrite(D3, llevel);
  digitalWrite(D4, rlevel);
}
void lencode() {          // GPIO ISR Interrupt service routines for encoder changes
  lcurrenttime = millis();
  lvt = ldir*1000./(lcurrenttime - llasttime);
  llasttime = lcurrenttime;
  lmc=lmc+ldir;
}
void rencode(){ 
  rcurrenttime = millis();
  rvt = rdir*1000./(rcurrenttime - rlasttime);
  rlasttime = rcurrenttime;
  rmc=rmc+rdir;
}

void dump(){
  Serial.print("lSet: ");
  Serial.print(lSet);
  Serial.print(" rSet: ");
  Serial.print(rSet);
  Serial.print(" lIn:");
  Serial.print(lIn);
  Serial.print(" rIn:");
  Serial.print(rIn);
  Serial.print(" lOut:"); 
  Serial.print(lOut);
  Serial.print(" rOut:");
  Serial.print(rOut);
  Serial.print(" lmc:");
  Serial.print(lmc);
  Serial.print(" rmc=");
  Serial.print(rmc);
  Serial.print(" tic#");
  Serial.print(ti);
  Serial.print(" lkp: ");
  Serial.print(lkp);
  Serial.print(" lki: ");
  Serial.print(lki);
  Serial.print(" lkd: ");
  Serial.println(lkd);
}

void setup() {
  pinMode(D1, OUTPUT); // 1,2EN aka D1 pwm left
  pinMode(D2, OUTPUT); // 3,4EN aka D2 pwm right
  pinMode(D3, OUTPUT); // 1A,2A aka D3
  pinMode(D4, OUTPUT); // 3A,4A aka D4
  pinMode(D5, INPUT);  // Left encoder
  pinMode(D6, INPUT);  // Right encoder
  //attachInterrupt(D5, lencode, CHANGE); // Setup Interrupt 
  //attachInterrupt(D6, rencode, CHANGE); // Setup Interrupt 
  attachInterrupt(D5, lencode, RISING);   // Setup Interrupt 
  attachInterrupt(D6, rencode, RISING);   // Setup Interrupt 
  sei();                                  // Enable interrupts  
  os_timer_setfn(&myTimer, tic, NULL);
  os_timer_arm(&myTimer, period, true);   // timer in ms
  lPID.SetSampleTime(period);
  rPID.SetSampleTime(period); 
  lPID.SetOutputLimits(-1023, 1023);  
  rPID.SetOutputLimits(-1023, 1023);  
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
  Serial.begin(115200);
  Serial.println("Ready");
}

char c=0;             // input char from keys
void loop(){
  // watch for input
  if(c==0){
    if (Serial.available() != 0) {
      c = Serial.read();
      Serial.println(c);
    }
  } else {
    if (Serial.available() != 0) {
      int gv = Serial.parseInt();
      if(c=='r'){
        rSet=gv/10.;
      } else if(c=='l'){
        lSet=gv/10.;
      } else if(c=='p'){
        lkp=gv/10.;
        rkp=gv/10.;
        lPID.SetTunings(lkp, lki, lkd);
        rPID.SetTunings(rkp, rki, rkd);
      } else if(c=='i'){
        lki=gv/10.;
        rki=gv/10.;
        lPID.SetTunings(lkp, lki, lkd);
        rPID.SetTunings(rkp, rki, rkd);
      } else if(c=='d'){
        lkd=gv/10.;
        rkd=gv/10.;
        lPID.SetTunings(lkp, lki, lkd);
        rPID.SetTunings(rkp, rki, rkd);
      } else if(c=='s'){
        dump();
      } else {
        Serial.println("Tunning command not recognized. Use l NNN, r NNN, p NNN, i NNN, d NNN with NNN=n*10, s to dump variables");
      }
      while (Serial.available()) Serial.read();
      c=0;
    }
  }
  if(DEBUG){
    Serial.print(lSet);
    Serial.print(",");
    Serial.print(rSet);
    Serial.print(",");
    Serial.print(lIn);
    Serial.print(",");
    Serial.print(rIn);    
    Serial.print(",");
    Serial.print(lOut);    
    Serial.print(",");
    Serial.println(rOut);    
  }
  delay(1000);
}

