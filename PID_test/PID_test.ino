#include <ESP8266WiFi.h>
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
int gv=8;        // goal velocity
float kpl=2;     // Kp from PID
float kpr=2;     // Kp from PID

os_timer_t myTimer;

// start of timerCallback
void tic(void *pArg) {
  lv=lmc-lmc0;            // lv left instant velocity
  lmc0=lmc;
  rv=rmc-rmc0;            // rv right install velocity
  rmc0=rmc;
  th=rmc-lmc;             // angular position(*pi()/rstep(180))
  av=(lv+rv)/2.;          // center forward velocity
  ti+=1;                  // tic counter 
  if(lv>gv){ lpwm-=(lv-gv)*kpl;}
  if(lv<gv){ lpwm+=(gv-lv)*kpl;}
  if(rv>gv){ rpwm-=(rv-gv)*kpr;}
  if(rv<gv){ rpwm+=(gv-rv)*kpr;}
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
  attachInterrupt(D5, lencode, RISING); // Setup Interrupt 
  attachInterrupt(D6, rencode, RISING); // Setup Interrupt 
  sei();                                // Enable interrupts  
  int currentTime = millis();
  int cloopTime = currentTime;
  os_timer_setfn(&myTimer, tic, NULL);
  os_timer_arm(&myTimer, 500, true);   // timer in ms
  
  Serial.begin(115200);
  Serial.println("Ready");
}

void loop(){
  //waiting for input
  if (Serial.available() != 0) {
    gv = Serial.parseInt();
    motion(lpwm,rpwm,HIGH,HIGH);
    while (Serial.available()) Serial.read();
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
  Serial.print(" gv: ");
  Serial.println(gv);
  motion(lpwm,rpwm,HIGH,HIGH);
  delay(1000);
}//read int or parseFloat for ..float...

