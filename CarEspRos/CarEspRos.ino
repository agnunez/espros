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
const char* ssid = "***";
const char* password = "***";

IPAddress server(192, 168, 1, 100);
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

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

void stop(void)
{
    analogWrite(D1, 0);
    analogWrite(D2, 0);
}
 
void forward(void) {
    analogWrite(D1, lpwm);
    analogWrite(D2, rpwm);
    digitalWrite(D3, HIGH);
    digitalWrite(D4, HIGH);
    delay(len);
    stop();
}
 
void backward(void) {
    analogWrite(D1, lpwm);
    analogWrite(D2, rpwm);
    digitalWrite(D3, LOW);
    digitalWrite(D4, LOW);
    delay(len);
    stop();
}
 
void left(void) {
    analogWrite(D1, lpwm);
    analogWrite(D2, rpwm);
    digitalWrite(D3, LOW);
    digitalWrite(D4, HIGH);
    delay(len);
    stop();
}
 
void right(void) {
    analogWrite(D1, lpwm);
    analogWrite(D2, rpwm);
    digitalWrite(D3, HIGH);
    digitalWrite(D4, LOW);
    delay(len);
    stop();
}
 
void lencode() {
  lmc=lmc+ldir;
}
void rencode(){
  rmc=rmc+rdir;
}
void tic(void){
  lv=lmc-lmc0;
  lmc0=lmc;
  rv=rmc-rmc0;
  rmc0=rmc0;
}

void leftCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  ldir=-1;
  rdir=1;
  left();
}
void rightCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  ldir=1;
  rdir=-1;
  right();
}

void forwardCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  ldir=1;
  rdir=1;            
  forward();
}

void backwardCallback(const std_msgs::Int16& msg) {
  len = abs(msg.data);
  ldir=-1;
  rdir=-1;
  backward();
}

std_msgs::String str_msg;
std_msgs::Int16 int_msg;
ros::Publisher leftenc("/car/leftencoder", &int_msg);
ros::Publisher rightenc("/car/rightencoder", &int_msg);

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);
ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(leftenc);
  nh.advertise(rightenc);
  nh.subscribe(sub_r);
  nh.subscribe(sub_l);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);

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
