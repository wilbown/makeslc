#include <Joystick.h>
#include <Servo.h>

/* INITIAL DELTA CODE FROM
   http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
*/

/*
    PROGRAM VARIABLES
*/
    int debug = 2;      // 0 prints NO updates to Serial Monitor
                        // 1 prints joystick updates to Serial Monitor
                        // 2 prints delta updates to Serial Monitor

/*
    JOYSTICK VARIABLES
*/
    int xPosition = 0; // X variance of joystick from center
    int yPosition = 0; // Y variance of joystick from center
    int zPosition = 0; // pseudo-Z variance from center

/*
    DELTA VARIABLES
*/
    float theta1; // servo 1 angle
    float theta2; // servo 2 angle
    float theta3; // servo 3 angle
    float oldT1;
    float oldT2;
    float oldT3;
    int result;
  
    const float e = 110.0;     // end effector
    const float f = 170.3;     // base
    const float re = 190.0;
    const float rf = 95.0;
   
    // trigonometric constants
    const float sqrt3 = sqrt(3.0);
    const float pi = 3.141592653;    // PI
    const float sin120 = sqrt3/2.0;   
    const float cos120 = -0.5;        
    const float tan60 = sqrt3;
    const float sin30 = 0.5;
    const float tan30 = 1/sqrt3;

    #define MAXANGLE 90
    #define MINANGLE -90
    
    float updateRate = 5.0;
    float updateRateZ = 10.0;
    float threshold = 3.0;
    
    float zDevSq = 5;

/*
    INSTANCE CREATION
*/
Joystick stick(A0, A1);


/*
    CORE FUNCIONS
*/
void setup() {
  if (debug > 0) {Serial.begin(9600);}
}

void loop() {
  // read joystick position
  if (debug == 1) {stick.report();}
  xPosition = stick.getXvariance();
  yPosition = stick.getYvariance();
  delay(200);
  
//# TODO - review following when adding in Z
//  // a simple statistical filter for removing pitch sensor noise
//  int z = -120 - stick.readPitch();
//  int z = 0; //# TODO - review previous when adding in Z
//  float dZSq = (zPosition - z)*(zPosition - z);
//  // update moving standard deviation
//  if (dZSq < 9*zDevSq)  // 3 stdevs from mean, both sides squared
//  {
//    zDevSq = zDevSq * (updateRateZ - 1) + dZSq;
//    zPos = zPos*(updateRateZ - 1.0) / updateRateZ + z / updateRateZ;
//  }

 // main calculation call
  result = delta_calcInverse(xPosition, yPosition, zPosition, theta1, theta2, theta3);
  if (debug ==2) {Serial.print(result == 0 ? "ok" : "no");}
  char buf[10];
  dtostrf(xPosition, 4, 0, buf);
  if (debug ==2) {Serial.print(" X");}
  if (debug ==2) {Serial.print(buf);}
  dtostrf(yPosition, 4, 0, buf);
  if (debug ==2) {Serial.print(" Y");}
  if (debug ==2) {Serial.print(buf);}
  dtostrf(zPosition, 4, 0, buf);
  if (debug ==2) {Serial.print(" Z");}
  if (debug ==2) {Serial.print(buf);}

  dtostrf(theta1, 6, 2, buf);
  if (debug ==2) {Serial.print(" T1");}
  if (debug ==2) {Serial.print(buf);}
  dtostrf(theta2, 6, 2, buf);
  if (debug ==2) {Serial.print(" T2");}
  if (debug ==2) {Serial.print(buf);}
  dtostrf(theta3, 6, 2, buf);
  if (debug ==2) {Serial.print(" T3");}
  if (debug ==2) {Serial.print(buf);}

  if (debug ==2) {Serial.println("");}
  
  if (result == 0) {
    if (abs(oldT1 - theta1) > threshold || abs(oldT2 - theta2) > threshold || abs(oldT3 - theta3) > threshold)
    {
      if (debug == 2) {
        Serial.print(theta1+90.0-14.0);
      }
      else {
//        servo1.write(theta1+90.0-14.0); //-14 is the correction for the angle for my serve
      }
      oldT1 = theta1;
      if (debug == 2) {
        Serial.print(theta2+90.0-10.0);
      }
      else {
//              servo2.write(theta2+90.0-10.0);
      }
      oldT2 = theta2;
      if (debug == 2) {
        Serial.print(theta3+90.0-00.0);
      }
      else {
//        servo3.write(theta3+90.0-00.0);
      }
      oldT3 = theta3;
    }
  }
  delay(5);
}

/*
    SUPPORT FUNCTIONS
*/

 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }
 
 // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }

