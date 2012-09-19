// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 

Servo s1,s2,s3;

float rx1 = -2; float ry1 = 2;
float rx2 = 2; float ry2 = 1.5;
float rx3 = 0; float ry3 = 4;
float rmax =
  sqrt(sq(rx1) + sq(ry1)) +
  sqrt(sq(rx2) + sq(ry2)) +
  sqrt(sq(rx3) + sq(ry3))
  ;

int potx;
int poty;
float x = 0;
float y = 0;
float z = 0;

void setup() {
  Serial.begin(115200);
  
  Serial.println(rmax);
  
  float r1 = sqrt(sq(rx1-x)+sq(ry1-y)+sq(z));
  Serial.println(r1);
  float r2 = sqrt(sq(rx2-x)+sq(ry2-y)+sq(z));
  Serial.println(r2);
  float r3 = sqrt(sq(rx3-x)+sq(ry3-y)+sq(z));
  Serial.println(r3);
  
  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
  
}

void loop() {
  potx = analogRead(1);            // LF/RT reads the value of the potentiometer (value between 0 and 1023)
  //potx = map(potx, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)
  
  potx = analogRead(1);            // LF/RT
  poty = analogRead(0);            // UP/DN
  
  
  float r1 = sqrt(sq(rx1-x)+sq(ry1-y)+sq(z));
  float r2 = sqrt(sq(rx2-x)+sq(ry2-y)+sq(z));
  float r3 = sqrt(sq(rx3-x)+sq(ry3-y)+sq(z));
  
  
  s1.write(map(r1, 0, 1023, 0, 179));                  // sets the servo position according to the scaled value
  s2.write(map(r2, 0, 1023, 0, 179));
  s3.write(map(r3, 0, 1023, 0, 179));
  delay(15);                           // waits for the servo to get there
}
