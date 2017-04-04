# espros
ROS serial for ESP8266 through WiFi

2017 Agustin Nunez (https://github.com/agnunez/espros.git) 

This is an adaptation of ROS-serial-arduino to run on ESP8266 using wireless WiFi
instead of wired USB UART cable

The reason being that ESP8266 (or ESP32) are much powerful and cheaper tha AVR Arduinos
with more memory and speed and integrating WiFi for 10 times less cost. The use of I2C 
or SPI in ESP8266 allows to expand the number of GPIO's at wish. The vision is that 
every sensor or actuator node in a ROS-robot can be created using a single $2 ESP8266,
with just power supply or powerbank battery, avoiding harness and reducing total cost.

esproswifi  Example program taken from http://answers.ros.org/users/1034/ahendrix/ for Arduino and
            adapted to ESP8266, NodeMCU dev module. 
            Please, modify ssid/password of your Wifi and IP of your roscore server. 
            Your ESP8266 will try to connect to 11411 port at that server IP
            To allow the connection, run roscore of course, and later:
            $ rosrun rosserial_python serial_node.py tcp

chatter     The standard ros tutorial client, working via Serial cable adapted for ESP8266
	    This example is particulary suitable for Witty ESP8266 dev platform and allow
	    to toggle all its leds by publishing to /led topic with stdmsg/Int16 that 
            correspond to its gpio pin ESP12E number, (i.e. 2,12,13,15)

This code is part of a much larger project to build an open source ROS Robot, called Gara

Have fun,

Agustin

