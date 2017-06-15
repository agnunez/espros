////////////////////////////////
//
// Wifi ROS Car with ESP8266
//
// Find last versions at:
// https://github.com/agnunez/espros.git
//
// MIT License 2017 Agustin Nunez
/////////////////////////////////

#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#define DEBUG 1

int spd=800;
int lpwm=spd;
int rpwm=spd;
int len=500; // period in ms
int lmc=0;   // left motor counter
int rmc=0;   // right motor counter
int lmc0=0; // last encoder value 
int rmc0=0;
int ldir=1;  // left motor direction
int rdir=1;
int lv=0; // left motor actual speed
int rv=0; // measured in n tics/timer period
int ltp=0; // left motor target position 
int rtp=0;

//////////////////////
// WiFi Definitions //
//////////////////////
// WiFi configuration. Replace '***' with your data
const char* ssid = "GTC-Guest";
const char* password = ".gtcguest.";
IPAddress server(161,72,94,217);      // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;    // Set the rosserial socket server port

void stop(void){      // Stop both motors
    analogWrite(D1, 0);
    analogWrite(D2, 0);
}
void motion(int lpw, int rpw, int llevel, int rlevel, int period) {  // generic motion for a period in milisecods
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
    analogWrite(D1, lpw);
    analogWrite(D2, rpw);
    digitalWrite(D3, llevel);
    digitalWrite(D4, rlevel);
    delay(period);
    stop();
}
void lencode() {          // GPIO ISR Interrupt service routines for encoder changes
  lmc=lmc+ldir;
}
void rencode(){ 
  rmc=rmc+rdir;
}
void tic(void){           // timer tics for continuous velocity calculation
  lv=lmc-lmc0;            // lv left instant velocity
  lmc0=lmc;
  rv=rmc-rmc0;            // rv right install velocity
  rmc0=rmc0;
}
void leftCallback(const std_msgs::Int16& msg) { //  All subscriber messages callbacks here
//  len = abs(msg.data);
  lpwm = abs(msg.data);
  rpwm = lpwm;
  motion(lpwm,rpwm,LOW,HIGH,len);
}
void rightCallback(const std_msgs::Int16& msg) {
//  len = abs(msg.data);
  lpwm = abs(msg.data);
  motion(lpwm,rpwm,HIGH,LOW,len);
}
void forwardCallback(const std_msgs::Int16& msg) {
//  len = abs(msg.data);
  lpwm = abs(msg.data);
  motion(lpwm,rpwm,HIGH,HIGH,len);
}
void backwardCallback(const std_msgs::Int16& msg) {
//  len = abs(msg.data);
  lpwm = abs(msg.data);
  motion(lpwm,rpwm,LOW,LOW,len);
}

std_msgs::String str_msg;
std_msgs::Int16 int_msg;
ros::Publisher leftenc("/car/leftencoder", &int_msg);
ros::Publisher rightenc("/car/rightencoder", &int_msg);

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);

void setupWiFi() {                    // connect to ROS server as as a client
  Serial.begin(115200);               // Use ESP8266 serial only for to monitor the process
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

ros::NodeHandle nh;

void setup() {
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(leftenc);
  nh.advertise(rightenc);
  nh.subscribe(sub_r);
  nh.subscribe(sub_l);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);

 // configure GPIO's
  pinMode(D0, OUTPUT); // Ultrasonic Trigger
  pinMode(D1, OUTPUT); // 1,2EN aka D1 pwm left
  pinMode(D2, OUTPUT); // 3,4EN aka D2 pwm right
  pinMode(D3, OUTPUT); // 1A,2A aka D3
  pinMode(D4, OUTPUT); // 3A,4A aka D4
  pinMode(D5, INPUT); //  Left encoder
  pinMode(D6, INPUT); //  Right encoder
//  s.attach(D7);       //  Servo PWM
//  pinMode(ECHO, INPUT); //  Ultrasonic Echo. D0 with 1k,2k voltage divisor
//  pinMode(TRIGGER, OUTPUT); // Ultrasonic Trigger. D8 . Power Ultrasonic board with 5v.
// configure interrupts to their ISR's    
  attachInterrupt(D5, lencode, RISING); // Setup Interrupt 
  attachInterrupt(D6, rencode, RISING); // Setup Interrupt 
  sei();                                // Enable interrupts  
// configure timer
  int currentTime = millis();
  int cloopTime = currentTime;
  timer1_disable();
  timer1_isr_init();
  timer1_attachInterrupt(tic);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(8000000);
}

void loop() {
  int_msg.data = lmc;
  leftenc.publish( &int_msg );
  int_msg.data = rmc;
  rightenc.publish( &int_msg );
  nh.spinOnce();
  delay(500);
}
