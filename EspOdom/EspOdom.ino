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
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
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
Servo s;
int sa=80; // Servo center position
int sd=1;  // Servo step
int smax=120; // Servo max angle
int smin=40;  // Servo min Angle
int sr=0;     // counter for 
int sp=10;    // number of loops among range measurements

// WiFi configuration. Replace *** by your data//
const char* ssid = "***";
const char* password = "***";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,***);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

// ROS nodes //
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
geometry_msgs::TransformStamped t2;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_pub;
sensor_msgs::Range range_msg;

// Functions definitions //

void setupWiFi() { /// connect to ROS server WiFi as a client
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
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

void stop(void){      // Stop both motors
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
  lv=lmc-lmc0;   // lv left instant velocity
  lmc0=lmc;
  rv=rmc-rmc0;   // rv right install velocity
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

int sstep(){
  sa+=sd;
  if(sa>smax || sa<smin) sd=-sd;
  s.write(sa);
}
int srange(){  // calculate distance from ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIGGER, LOW);  
  delayMicroseconds(2); 
  
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10); 
  
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = (duration/2) / 29.1;
  sstep();
  return (int) distance;
}

/// ROS topics object definitions PUBLISHERS
std_msgs::String str_msg;
std_msgs::Int16 int_msg;
ros::Publisher leftenc("/car/lencoder", &int_msg);
ros::Publisher rightenc("/car/rencoder", &int_msg);
ros::Publisher pub_range("/ultrasound", &range_msg);
ros::Publisher angle("/car/angle", &int_msg);

// ROS SUBSCRIBERS
ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);

double x = 1.0;
double y = 0.0;
double th = 0;
char base_link[] = "/base_link";
char odom[] = "/odom";
char ultrafrid[] = "/ultrasound";

void setup() {
  if(DEBUG)Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(leftenc);
  nh.advertise(rightenc);
  nh.advertise(pub_range);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  ultrafrid;
  range_msg.field_of_view = 0.1;
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  nh.advertise(angle);
  nh.subscribe(sub_r);
  nh.subscribe(sub_l);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);

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
// odometry configuration
ros::Time current_time = nh.now();
ros::Time last_time = current_time;
double DistancePerCount = (2 * 3.14159265 * 0.035) / 20;   // 2*PI*R/CPR
double lengthBetweenTwoWheels = 0.13;
int last_lmc = lmc;
int last_rmc = rmc;
int current_lmc = lmc;
int current_rmc = rmc;

void loop() {
  if (nh.connected()) {
    current_time = nh.now();
    current_lmc = lmc;
    current_rmc = rmc;
    double dt = current_time.toSec() - last_time.toSec();  
    double ld = (current_lmc-last_lmc)*DistancePerCount;   // left wheel linear distance
    double rd = (current_rmc-last_rmc)*DistancePerCount;  // right wheel linear distance
    double vlm = ld / dt;   // left wheel linear velocity
    double vrm = rd / dt;  // right wheel linear velocity
    double vx = (ld + rd) / 2.;
    double vy = 0;
    double th = (current_rmc - current_lmc) / 45. * 3.14159265;
    last_lmc = current_lmc;
    last_rmc = current_rmc;
    last_time = current_time;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    x += delta_x;
    y += delta_y;
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = x; 
    t.transform.translation.y = y; 
    t.transform.rotation = tf::createQuaternionFromYaw(th);
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    range_msg.range = srange()/100.;
    range_msg.header.stamp = current_time;
    pub_range.publish(&range_msg);
    t2.header.frame_id = base_link;
    t2.child_frame_id = ultrafrid;
    t2.transform.translation.x = 0.05; 
    t2.transform.translation.y = 0.0; 
    t2.transform.translation.z = 0.1;
    double saangle = (sa-80)*0.0314159265;
    t2.transform.rotation = tf::createQuaternionFromYaw(saangle);
    t2.header.stamp = current_time;
    broadcaster.sendTransform(t2);
    int_msg.data = sa;
    angle.publish( &int_msg );
    int_msg.data = lmc;
    leftenc.publish( &int_msg );
    int_msg.data = rmc;
    rightenc.publish( &int_msg );
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(200);
}
