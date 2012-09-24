/*
  Joystick.h
  
  PARALLAX 2-AXIS JOYSTICK
  Radio Shack p/n: 27800
  
  Analog Input
  Takes X/Y analog input from the joystick connected as displayed at the bottom of
  http://learn.parallax.com/kickstart/27800
  
  Recommended circuit:
  * arduino +5V to +breadboard
  * +breadboard to joystick U/D+  
  * +breadboard to joystick L/R+
  * arduino A1 to joystick L/R (upper-left)
  * arduino A0 to joystick U/D (lower-left)
  * -breadboard to joystick GND (lower-left)
  
  Created by Sean Duncan
  for  http://www.parallax.com/Store/Microcontrollers/BASICStampModules/tabid/134/txtSearch/27800/List/1/ProductID/581/Default.aspx?SortField=ProductName%2cProductName
  based on http://learn.parallax.com/kickstart/27800
  and http://arduino.cc/en/Tutorial/AnalogInput
  
  */

#include "Arduino.h"
#include "Joystick.h"

Joystick::Joystick(int pinY, int pinX) {
  Serial.begin(9600);
  _pinY = pinY; // A0;         // pin connected to joystick U/D
  _pinX = pinX; //A1;         // pin connected to joystick L/R
//  boolean _debug = debug; // true;  // stream data to Serial Monitor
//  int _UD = 0;            // range 0-1023 ; center = 513-514
//  int _LR = 0;            // range 0-1023 ; center = 528-529
   _UDcenter = 513;    // UD default value
   _LRcenter = 528;    // LR default value
}

int Joystick::getX() {
  // report the current X coordinate value
  return analogRead(_pinX);
}

int Joystick::getY() {
  // report the current Y coordinate value
  return analogRead(_pinY);
}

int Joystick::getXvariance() {
  // report the difference between current X and center
  return analogRead(_pinX) - _LRcenter;
}

int Joystick::getYvariance() {
  // report the difference between current y and center
  return analogRead(_pinY) - _UDcenter;
}
    
void Joystick::report() {
    Serial.print("Y = ");
    Serial.print(getY(), DEC);
    Serial.print(", X = ");
    Serial.print(getX(), DEC);   
    Serial.print(" Y var = ");
    Serial.print(getYvariance(), DEC);
    Serial.print(", X var = ");
    Serial.println(getXvariance(), DEC); 
}