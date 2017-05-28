/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#if ARDUINO>=100
  #include <Arduino.h>  // Arduino 1.0
#else
  #include <WProgram.h>  // Arduino 0022
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)
  #if defined(USE_TEENSY_HW_SERIAL)
    #define SERIAL_CLASS HardwareSerial // Teensy HW Serial
  #else
    #include <usb_serial.h>  // Teensy 3.0 and 3.1
    #define SERIAL_CLASS usb_serial_class
  #endif
#elif defined(_SAM3XA_)
  #include <UARTClass.h>  // Arduino Due
  #define SERIAL_CLASS UARTClass
#elif defined(USE_USBCON)
  // Arduino Leonardo USB Serial Port
  #define SERIAL_CLASS Serial_
#else 
  #include <HardwareSerial.h>  // Arduino AVR
  #define SERIAL_CLASS HardwareSerial
#endif

#ifndef ESP_H

class ArduinoHardware {
  public:
    ArduinoHardware(SERIAL_CLASS* io , long baud= 57600){
      iostream = io;
      baud_ = baud;
    }
    ArduinoHardware()
    {
#if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
      iostream = &Serial1;
#elif defined(USE_TEENSY_HW_SERIAL)
      iostream = &Serial1;
#else
      iostream = &Serial;
#endif
      baud_ = 57600;
    }
    ArduinoHardware(ArduinoHardware& h){
      this->iostream = h.iostream;
      this->baud_ = h.baud_;
    }
  
    void setBaud(long baud){
      this->baud_= baud;
    }
  
    int getBaud(){return baud_;}

    void init(){
#if defined(USE_USBCON)
      // Startup delay as a fail-safe to upload a new sketch
      delay(3000); 
#endif
      iostream->begin(baud_);
    }

    int read(){return iostream->read();};
    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++)
        iostream->write(data[i]);
    }

    unsigned long time(){return millis();}

  protected:
    SERIAL_CLASS* iostream;
    long baud_;
};

#else

#include <ESP8266WiFi.h>
IPAddress server(192, 168, 1, 100); // your ROS server IP here
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

class ArduinoHardware {

  public:
  ArduinoHardware() {};

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

#endif

#endif
