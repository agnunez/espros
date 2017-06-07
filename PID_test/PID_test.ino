#include <ESP8266WiFi.h>
#include <PID_v1.h>

extern "C" {
#include "user_interface.h"
}

int lpwm=0;
int rpwm=0;
int len=300;    // period in ms
int lmc=0;      // left motor counter
int rmc=0;   
int lmc0=0;     // last encoder value 
int rmc0=0;
int ldir=1;     // left motor direction
int rdir=1;
int lv=0;       // left motor actual speed
int rv=0;       // measured in n tics/timer period
int ltp=0;      // left motor target position 
float av, th;   // average & angular velocity
int ti=0;
int gv=0;        // goal velocity
int period=100;    // timer period in ms
double lIn,rIn,tIn,lOut,rOut,tOut,lSet,rSet,tSet;
double lkp=1,lki=0,lkd=0;     // Left wheel PID constants
double rkp=1,rki=0,rkd=0;     // Right wheel PID constants
double tkp=1,tki=0,tkd=0;     // Right wheel PID constants

//Specify the links and initial tuning parameters
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID lPID(&lIn, &lOut, &lSet,lkp,lki,lkd, DIRECT);
PID rPID(&rIn, &rOut, &rSet,rkp,rki,rkd, DIRECT);
PID tPID(&tIn, &tOut, &tSet,tkp,tki,tkd, DIRECT);

os_timer_t myTimer;


void pidupdate(){
  rPID.Compute();
  lPID.Compute();
}
// start of timerCallback
void tic(void *pArg) {
  lv=lmc-lmc0;            // lv left instant velocity
  lmc0=lmc;
  rv=rmc-rmc0;            // rv right install velocity
  rmc0=rmc;
  th=rmc-lmc;             // angular position(*pi()/rstep(180))
  av=(lv+rv)/2.;          // center forward velocity
  ti+=1;                  // tic counter
  lIn=lv;
  rIn=rv;
  lpwm=lOut*4;
  rpwm=rOut*4; 
  motion(lpwm,rpwm,HIGH,HIGH);
}

void stop(void){      // Stop both motors
    analogWrite(D1, 0);
    analogWrite(D2, 0);
}
 
void motion(int lpwm, int rpwm, int llevel, int rlevel) {  
    if (llevel==HIGH) {
      ldir=1; 
    } else {
      ldir=-1;
    }
    if (rlevel==HIGH) {
      rdir=1; 
    } else {
      rdir=-1;
    }
    if(lpwm>1023) lpwm=1023;
    if(rpwm>1023) rpwm=1023;
    if(lpwm<0) lpwm=0;
    if(rpwm<0) rpwm=0;
    analogWrite(D1, lpwm);
    analogWrite(D2, rpwm);
    digitalWrite(D3, llevel);
    digitalWrite(D4, rlevel);
}
void lencode() {          // GPIO ISR Interrupt service routines for encoder changes
  lmc=lmc+ldir;
}
void rencode(){ 
  rmc=rmc+rdir;
}


void setup() {
  pinMode(D1, OUTPUT); // 1,2EN aka D1 pwm left
  pinMode(D2, OUTPUT); // 3,4EN aka D2 pwm right
  pinMode(D3, OUTPUT); // 1A,2A aka D3
  pinMode(D4, OUTPUT); // 3A,4A aka D4
  pinMode(D5, INPUT); //  Left encoder
  pinMode(D6, INPUT); //  Right encoder
  attachInterrupt(D5, lencode, CHANGE); // Setup Interrupt 
  attachInterrupt(D6, rencode, CHANGE); // Setup Interrupt 
  sei();                                // Enable interrupts  
  int currentTime = millis();
  int cloopTime = currentTime;
  os_timer_setfn(&myTimer, tic, NULL);
  os_timer_arm(&myTimer, period, true);   // timer in ms
  lPID.SetSampleTime(period);
  rPID.SetSampleTime(period); 
  tPID.SetSampleTime(period);  
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
  tPID.SetMode(AUTOMATIC);
  Serial.begin(115200);
  Serial.println("Ready");
}
char c=0;
void loop(){
  // watch for input
  if(c==0){
    if (Serial.available() != 0) {
      c = Serial.read();
      Serial.println(c);
    }
  } else {
    if (Serial.available() != 0) {
      gv = Serial.parseInt();
      if(c=='s'){
        lSet=gv/10.;
        rSet=gv/10.;
      }
      if(c=='p'){
        lkp=gv/10.;
        rkp=gv/10.;
      }
      if(c=='i'){
        lki=gv/10.;
        rki=gv/10.;
      }
      if(c=='d'){
        lkd=gv/10.;
        rkd=gv/10.;
      }
      while (Serial.available()) Serial.read();
      c=0;
    }
  }
  Serial.print("lv:rv=");
  Serial.print(lv);
  Serial.print(":");
  Serial.print(rv);
  Serial.print(" lpwm:rpwm="); 
  Serial.print(lpwm);
  Serial.print(":");
  Serial.print(rpwm);
  Serial.print(" lmc:rmc=");
  Serial.print(lmc);
  Serial.print(":");
  Serial.print(rmc);
  Serial.print(" tic#");
  Serial.print(ti);
  Serial.print(" th: ");
  Serial.print(th);
  Serial.print(" av: ");
  Serial.print(av);
  Serial.print(" lSet: ");
  Serial.print(lSet);
  Serial.print(" lIn: ");
  Serial.print(lIn);
  Serial.print(" lOut: ");
  Serial.print(lOut);
  Serial.print(" lkp: ");
  Serial.print(lkp);
  Serial.print(" lki: ");
  Serial.print(lki);
  Serial.print(" lkd: ");
  Serial.println(lkd);
  pidupdate();
  delay(1000);
}//read int or parseFloat for ..float...

