//#include "clicli.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define PIN 5
#define NUMPIXELS 16
#define PIN2 6

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(NUMPIXELS, PIN2, NEO_GRB + NEO_KHZ800);

Servo motor1;  // create servo object to control a servo
Servo motor2;
Servo motor3;
Servo motor4;

int defSpeed = 1200;
double previousTime = 0;
double lastErrorPitch = 0;
double lastErrorRoll = 0;
double rateErrorPitch = 0;
double rateErrorRoll = 0;
double cumErrorPitch = 0;
double cumErrorRoll = 0;

//clicli mycli;  

void setup() { 
  motor1.attach(11, 1000, 2000);  // attaches the servo on pin 9 to the servo object
  motor2.attach(10, 1000, 2000);
  motor3.attach(9, 1000, 2000);
  motor4.attach(6, 1000, 2000);
  move(1000);
  delay(2000);
  //mycli.begin();
  //myservo.write(1100);

  Serial.println("starts");
  previousTime = millis();
 }

void loop() { 
  float pvPitch = 0;
  float pvRoll = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
  //mycli.run();
  //GET PV
  //PIDcalc(0, 0, pvPitch, pvRoll, kp, ki, kd);
  motor1.write(1300);
 }

void PIDcalc(int spPitch, int spRoll, int pvPitch, int pvRoll, float kp, float ki, float kd)
{
  unsigned long currentTime = millis();
  double elapsedTime = (currentTime - previousTime) / 1000.0;
  bool integralFlagPitch = true;
  bool integralFlagRoll = true;

  float errorRoll = 0;
  float errorPitch = 0;

  float outRoll = 0;
  float outPitch = 0;

  errorPitch = spPitch - pvPitch;
  errorRoll = spRoll - pvRoll;

  if (errorPitch * lastErrorPitch < 0) {  // If error changes sign (direction), reset integral
    integralFlagPitch = true;  // Set flag to indicate error has changed direction
    cumErrorPitch = 0;         // Reset cumulative error when direction changes
    Serial.println("Error changed direction, resetting integral accumulator.");
  } else {
    integralFlagPitch = false;  // Continue accumulating integral error
  }
  if (!integralFlagPitch) {
    cumErrorPitch += errorPitch * elapsedTime;  // Compute integral (accumulate error over time)
  }

  if (errorRoll * lastErrorRoll < 0) {  // If error changes sign (direction), reset integral
    integralFlagRoll = true;  // Set flag to indicate error has changed direction
    cumErrorRoll = 0;         // Reset cumulative error when direction changes
    Serial.println("Error changed direction, resetting integral accumulator.");
  } else {
    integralFlagRoll = false;  // Continue accumulating integral error
  }
  if (!integralFlagRoll) {
    cumErrorRoll += errorRoll * elapsedTime;  // Compute integral (accumulate error over time)
  }

  if (elapsedTime > 0) {  // Compute derivative deltaError/deltaTime, avoid dividing by 0
  double rateErrorPitch = (errorPitch - lastErrorPitch) / elapsedTime;
  double rateErrorRoll = (errorRoll - lastErrorRoll) / elapsedTime;

  outPitch = (defSpeed + (abs(errorPitch) * kp)) + ki * cumErrorPitch + kd * rateErrorPitch;
  outRoll = (defSpeed + (abs(errorRoll) * kp)) + ki * cumErrorRoll + kd * rateErrorRoll;
  }

  previousTime = currentTime;
  lastErrorPitch = errorPitch;
  lastErrorRoll = errorRoll;
}


 void move(int speed){
  motor1.write(speed);
  motor2.write(speed);
  motor3.write(speed);
  motor4.write(speed);
 }
