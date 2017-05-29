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
int i;

// WiFi configuration. Replace *** by your data//
const char* ssid = "***";
const char* password = "**";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,***);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

// ROS nodes //
ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_pub;
tf::TransformBroadcaster broadcaster;

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
void angleCallback(const std_msgs::Int16& msg) {
  i = abs(msg.data);
  s.write(i);
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
  return (int) distance;
}

/// ROS topics object definitions PUBLISHERS
std_msgs::String str_msg;
std_msgs::Int16 int_msg;
ros::Publisher leftenc("/car/lencoder", &int_msg);
ros::Publisher rightenc("/car/rencoder", &int_msg);
ros::Publisher range("/car/range", &int_msg);

// ROS SUBSCRIBERS
ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);
ros::Subscriber<std_msgs::Int16> sub_a("/car/angle", &angleCallback);

double x = 1.0;
double y = 0.0;
double theta = 1.57;
char base_link[] = "/base_link";
char odom[] = "/odom";

void setup() {
  if(DEBUG)Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(leftenc);
  nh.advertise(rightenc);
  nh.advertise(range);
  nh.subscribe(sub_r);
  nh.subscribe(sub_l);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);
  nh.subscribe(sub_a);

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

void loop() {
  if (nh.connected()) {
    current_time = nh.now();
    double dt = current_time.toSec() - last_time.toSec();  
    double v_left = lv*DistancePerCount / dt;   // left wheel linear velocity
    double v_right = rv*DistancePerCount / dt;  // right wheel linear velocity
    double vx = ((v_right + v_left) / 2) * 10;
    double vy = 0;
    double vth = ((v_right - v_left)/ lengthBetweenTwoWheels);
    double th = (lmc - rmc) / 45. * 3.14159265;
    //compute odometry in a typical way given the velocities of the robot
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt; 
    x += delta_x;
    y += delta_y;
    th += delta_th;

    double dx = 0.2;
    double dtheta = 0.18;

    //theta=(lmc-rmc)*3.14/45.;
    x += cos(theta)*dx*0.1;
    y += sin(theta)*dx*0.1;
    theta += dtheta*0.1;
    if(theta > 3.14)
      theta=-3.14;
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = x; 
    t.transform.translation.y = y; 
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    int_msg.data = srange();
    range.publish( &int_msg );
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
