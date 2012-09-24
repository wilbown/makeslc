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

#ifndef Joystick_h
#define Joystick_h

#include "Arduino.h"

class Joystick
{
  public:
	 Joystick(int pinY, int pinX);
    int getX();
    int getY();
    int getXvariance();
    int getYvariance();
    void report();
  private:
	int _pinY;
    int _pinX;
    int _UDcenter;
	int _LRcenter;
};

#endif