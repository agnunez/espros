#include <ESP8266WiFi.h>  
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
int led;

void messageCb(const std_msgs::Int16& msg) {
  if(msg.data > 0){
    led=abs(msg.data);
    digitalWrite(led, HIGH-digitalRead(led));   // blink the led
  }
}

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::Int16> sub("led", &messageCb);

char hello[15] = "ESP8266 alive!";

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(15, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
