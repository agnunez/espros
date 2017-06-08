#include <ESP8266WiFi.h>
#include <PID_v1.h>
extern "C" {
#include "user_interface.h"
}

int lpwm=0, rpwm=0;  // motor pwm
int lmc=0,  rmc=0;   // left motor counter
int lmc0=0, rmc0=0;  // last encoder value 
int ldir=1, rdir=1;  // motor direction
double whesep= 0.135; // wheel separtion in m
double whedia= 0.7;   // wheel diameter in m
int CPR = 40;     // Encoder Count per Revolutions (double of holes using up & down interrupt)
int period=200;   // sample timer period in ms
double lv=0, rv=0;// motor speed innumber tics per period
int ti=0;         // tic timer counter
double lOut,rOut,lSet,rSet;   // PID output and demands
double lkp=1,lki=0,lkd=0;     // Left wheel PID constants
double rkp=1,rki=0,rkd=0;     // Right wheel PID constants
int kt=1000/period; // number of periods in 1sec

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID lPID(&lv, &lOut, &lSet, lkp, lki, lkd, DIRECT);
PID rPID(&rv, &rOut, &rSet, rkp, rki, rkd, DIRECT);

os_timer_t myTimer;

// start of timerCallback
void tic(void *pArg) {
  lv=(lmc-lmc0)*kt;    // left instant velocity tic/period
  lmc0=lmc;
  rv=(rmc-rmc0)*kt;    // rv right install velocity
  rmc0=rmc;
  ti+=1;                  // tic counter

}

void stop(void){      // Stop both motors
    analogWrite(D1, 0);
    analogWrite(D2, 0);
}
 
void motion(double lpwm, double rpwm, int llevel, int rlevel) {  
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
  lPID.SetOutputLimits(0, 1023);  
  rPID.SetOutputLimits(0, 1023);  
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
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
      int gv = Serial.parseInt();
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
  Serial.print(" lSet: ");
  Serial.print(lSet);
  Serial.print(" rSet: ");
  Serial.print(rSet);
  Serial.print(" lv:");
  Serial.print(lv);
  Serial.print(" rv:");
  Serial.print(rv);
  Serial.print(" lOut:"); 
  Serial.print(lOut*4);
  Serial.print(" rOut:");
  Serial.print(rOut*4);
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
  rPID.Compute();
  lPID.Compute();
  motion(lOut,rOut,HIGH,HIGH);  
  delay(1000);
}

