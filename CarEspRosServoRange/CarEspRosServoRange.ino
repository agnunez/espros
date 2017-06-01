////////////////////////////////
//
// Wifi ROS Car with ESP8266 and
// ultrasonic range servo
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
#include <Servo.h>

#define DEBUG 1
#define TRIGGER D8  // ultrasonic trigger pin
#define ECHO    D0  // ultrasonic echo pin  
                    //(!!Note: use 5v Vcc on ultrasonic board & a 2k,1k divider for ECHO GPIO protection)

int spd=800;
int lpwm=spd;
int rpwm=spd;
int len=300; // period in ms
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
const char* ssid = "GTC-Guest";
const char* password = ".gtcguest.";

IPAddress server(161,72,94,217); // your ROS server IP here
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;
Servo s;
int i;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};

void stop(void)      // Stop both motors
{
    analogWrite(D1, 0);
    analogWrite(D2, 0);
}
 
void motion(int lpw, int rpw, int llevel, int rlevel, int steps) {  // generic motion with "steps" duration in milisecods
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
    delay(steps);
    stop();
}
//// GPIO ISR Interrupt service routines for encoder changes
void lencode() {
  lmc=lmc+ldir;
}
void rencode(){ 
  rmc=rmc+rdir;
}
//// Speed (lv,rv) calculation every timer tic
void tic(void){ 
  lv=lmc-lmc0;
  lmc0=lmc;
  rv=rmc-rmc0;
  rmc0=rmc0;
}

////  All subscriber messages callbacks here
void leftCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  motion(lpwm,rpwm,LOW,HIGH,len);
}
void rightCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  motion(lpwm,rpwm,HIGH,LOW,len);
}
void forwardCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  motion(lpwm,rpwm,HIGH,HIGH,len);
}
void backwardCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  motion(lpwm,rpwm,LOW,LOW,len);
}
void angleCallback(const std_msgs::Int16& msg) {
  i = abs(msg.data);
  s.write(i);
}

/// ROS topics object definitions
std_msgs::String str_msg;
std_msgs::Int16 int_msg;
ros::Publisher leftenc("/car/leftencoder", &int_msg);
ros::Publisher rightenc("/car/rightencoder", &int_msg);
ros::Publisher range("/car/range", &int_msg);

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);
ros::Subscriber<std_msgs::Int16> sub("/car/angle", &angleCallback);
ros::NodeHandle_<WiFiHardware> nh;

/// connect to ROS server WiFi as a client
void setupWiFi()
{
  WiFi.begin(ssid, password);
  if(DEBUG) {
    Serial.print("\nConnecting to "); 
    Serial.println(ssid);
  }
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    if(DEBUG){
      Serial.print("Could not connect to: "); 
      Serial.println(ssid);
    }
    while(1) delay(500);
  }
  if(DEBUG){
    Serial.print("Ready to use ");
    Serial.println(WiFi.localIP());
  }
  
}

int srange(){
  long duration, distance;
  digitalWrite(TRIGGER, LOW);  
  delayMicroseconds(2); 
  
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10); 
  
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1;
  if(DEBUG){
    Serial.println("cm: ");
    Serial.println(distance);
  }
  return (int) distance;
}
void setup() {
  if(DEBUG)Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(leftenc);
  nh.advertise(rightenc);
  nh.advertise(range);
  nh.subscribe(sub_r);
  nh.subscribe(sub_l);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);
  nh.subscribe(sub);

  pinMode(D0, OUTPUT); // Ultrasonic Trigger
  pinMode(D1, OUTPUT); // 1,2EN aka D1 pwm left
  pinMode(D2, OUTPUT); // 3,4EN aka D2 pwm right
  pinMode(D3, OUTPUT); // 1A,2A aka D3
  pinMode(D4, OUTPUT); // 3A,4A aka D4
  pinMode(D5, INPUT); //  Left encoder
  pinMode(D6, INPUT); //  Right encoder
  s.attach(D7);       //  Servo PWM
  pinMode(ECHO, INPUT); //  Ultrasonic Echo. D0 with 1k,2k voltage divisor
  pinMode(TRIGGER, OUTPUT); // Ultrasonic Trigger. D8 . Power Ultrasonic board with 5v.
    
  attachInterrupt(D5, lencode, RISING); // Setup Interrupt 
  attachInterrupt(D6, rencode, RISING); // Setup Interrupt 
  sei();                                // Enable interrupts  
  int currentTime = millis();
  int cloopTime = currentTime;
  timer1_disable();
  timer1_isr_init();
  timer1_attachInterrupt(tic);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(8000000);
}

void loop() {
  int_msg.data = srange();
  range.publish( &int_msg );
  int_msg.data = lmc;
  leftenc.publish( &int_msg );
  int_msg.data = rmc;
  rightenc.publish( &int_msg );
  nh.spinOnce();
  
  delay(200);
}
