# espros
**ROS serial for ESP8266 through WiFi**
```
2017 Agustin Nunez (https://github.com/agnunez/espros.git) 
```
This is an adaptation of ROS-serial-arduino to run on an stand-alone ESP8266 with WiFi
instead of wired USB UART cable, based on http://answers.ros.org/users/1034/ahendrix/
proposed solution for Arduino with WiFi shield.

The reason being that ESP8266 (or ESP32) are much powerful and cheaper tha AVR Arduinos
with more memory and speed and integrating WiFi for 10 times less cost. The use of I2C 
or SPI in ESP8266 allows to expand the number of GPIO's at wish. The vision is that 
every sensor or actuator node in a ROS-robot can be created using a single $2 ESP8266,
with just power supply or powerbank battery, avoiding harness and reducing total cost.

**EspRos**

This Esp8266 sketch creates a Car (similar to turtlebot) with all basic ROS objects, that connect via "WiFi" with rosserial in the server. 
This is an innovative way of connecting rosserial using tcp thanks to http://answers.ros.org/users/1034/ahendrix/ example program.
The hardware used was a inexpensive ($15) kit in eBay or AliExpress that includes an ESP-12E version with NodeMCU and motor shield (L293D) as (https://www.aliexpress.com/item/10pcs-lot-NodeMcu-Lua-WIFI-development-board-based-on-the-ESP8266-Internet-of-things/32339195240.html) but any other
similar configuration can be used. The Car includes differential DC motor configuration with encoder wheels but requires an extra sensor 
that is sold as optical end stop ($2) in same shops (i.e.https://www.aliexpress.com/item/2016-Newest-1-x-Optical-Endstop-End-Stop-Limit-Switch-Solution-for-3D-Printer-or-CNC/32656689805.html). I have also replaced the standard AAA battery case for one compatible with two  powerful 16850 Li-Ion.
Finally, an ultrasonic HC-SR04 is mounted on top of a tiny micro servo, to generate a scan distance measurements on any angle from the heading.
There is also an upgrade, using a 74HC14 (hex inverting schmitt trigger) to clean optical sensor squared signal into the MCU.
(Instructables comming soon).

The following Ros objects are implemented:

  - odom.    Using simple wheel encoders (no quadrature)
  - tf.      Defines 3 frames, Scenario, Car and Servo Range
  - cmd_vel. differential_drive Twist commands for translation & rotation
  - Range    To display in rviz walls and obstacle SLAM

Odometry has been implemented using the differential drive motor model, considering the wheel diameter and separation, and the CPR
(Counts per Revolutions) of the encoder wheels. There are 3 references frames, scenario ('odom'), car ('base_link'), and the HC-SR04 heading
('ultrasonic'), and the corresponding two tf transform ('t', 't2').

cmd_vel has been implemented using the Arduino PID_library from Brett Beauregard https://github.com/br3ttb/Arduino-PID-Library. Below, you
can find a PID_test utility to calibrate the Kp, Ki and Kd of your car. There are several constants that characterize your car inside the code

Finally, new Range ros objects has been implemented by providing ultrasonic distance on every specif angle, on top of a tf transformation. 
The ranges can be drawn with persistance thanks to rviz ros visualizer.

Installation

In order to use the code, you need to install Ros in a server, install rosserial, gather the IP of your server and configure WiFi parameters
on EspRos code, to allow your Car to connect to Ros server. After that, you can use standard rostopic commands like:
'''
$ rostopic pub /car/cmd_vel geometry_msgs/Twist "linear:
  x: 20.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
'''
to move your car with linear and angular speeds components. Also, you can use rviz to visualize Car position, motion and Ranges with:
'''
$ rosrun rviz rviz
'''
TODO's. In order to finish Car navigation capabilities, we need to transform Range object into LaserScan or PointCloud2 objects

**PID_test**

This is a stand-alone tool to tune differential drive robot PID's. You can install this sketch in your ESP8266 connected to the motors with encoders, and use the Arduino IDE monitor to send commands, while visualizing speed or other parameters in real time.
Commands available are:
```
  s     (show current PID constants settings)
  r nnn (set speed of rght motors to nn.n encoder tick per second. (nnn is divide by 10))
  l nnn (set speed of left motors to nn.n encoder tick per second. (nnn is divide by 10))
  p nnn (set proportional constant Kp of both motor pid's to nn.n)
  i nnn (set integral constant Ki of both motor pid's to nn.n)
  d nnn (set derivative constant Kp of both motors pid's to nn.n)
```
while doing this, you will keep receiving the speeds of both motors in CSV format, so one can cut&paste this on a spreadsheet and draw a chart with variations
Inside the code, there are other parameters that can be preconfigured like initial Kp,Ki,Kd PID period (implemented in a timer loop), and other constants.
The parameters preconfigured are suited for simple car but with DC motor and simple encoder (no quadrature), but can be easily modified for other robots.

**Examples**

Folows, a list of progressive codes created during the research and development of Ros compatibility with ESP8266. They are convenient
to be used during learning, as Ros has an step learning curve.

**esproswifi**

Example program taken from http://answers.ros.org/users/1034/ahendrix/ for Arduino and
adapted to ESP8266, NodeMCU dev module. Please, modify ssid/password of your Wifi and 
IP of your roscore server. Your ESP8266 will try to connect to 11411 port at that server IP
To allow the connection, run roscore of course, and later:
```
            $ rosrun rosserial_python serial_node.py tcp
```

**chatter**

The standard ros tutorial client, working via Serial cable adapted for ESP8266
This example is particulary suitable for Witty ESP8266 dev platform and allow
to toggle all its leds by publishing to /led topic with stdmsg/Int16 that 
correspond to its gpio pin ESP12E number, (i.e. 2,12,13,15)

**CarEspRos**

NodeMCU with motor shield and 2 DC motors plus infrared encoders wheels (DoIt car) implementation in ROS WiFi
with esproswifi approach. Encoder counters are pusblished and basic movement with time length as subscriptions.
Car is moved with messages like: 
```
$ rostopic pub -1 /car/forward std_msgs/Int16 1000 // to move foward 1000 ms, use backward, right or left
```
Car encoders can be read with:
```
$ rostopic echo /car/leftencoder
```

**CarEspRosServo**

This is a version of CarEspRos above with addition of Servo in D7 nodeMCU pin (GPIO13 of ESP12). This Servo
ROS subscription can be used, to rotate a Steering Wheel, or other usages. See later. Message to publish is:
```
$ rostopic pub -1 /car/angle std_msgs/Int16 80  // to adjust to Servo center angle (min:0, max:255)
```

**CarEspRosServoRange**

This version accumulate previous functionality plus provides a range distance measurement in centimeters
on the orientation given by servo angle. Range topic can be received with:
```
$ rostopic echo /car/range
```

**EspOdom**

This is a preliminary version with Odometry and tf support. Be careful that rosserial includes need
an update of ArduinoHardware.h, ESP8266Hardware.h and ros.h files. Check that your distro have them
or copy the ones included here to avoid typedef errors. With this EspOdom version, you can already
test rviz by running
```
$ rosrun rviz rviz
```


This code is part of a much larger project to build an open source ROS Robot, called Gara

Have fun,

Agustin

