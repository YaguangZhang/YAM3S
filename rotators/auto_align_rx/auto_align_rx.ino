/*
 Auto align the RX antenna via the pan and tilt kit controlled by Arduino PWM
 signals.

 Yaguang Zhang, Purdue University, 2020/01/28
*/

#include <Servo.h>

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// The PWM range for the continuous servos.
#define MAX_PWM 2000
#define MID_PWM 1500
#define MIN_PWM 1000

// X axis - Tilt (changing elevation); Z axis - Pan (changing azimuth).
int wpmPinX = 9, wpmPinZ = 10;
Servo servoX, servoZ;

// Fetch orientation data from the VR IMU.
BNO080 vrImu;

void setup() {
  servoX.attach(wpmPinX);  
  servoZ.attach(wpmPinZ);

  Serial.begin(9600);
}

void loop() {
  servoX.writeMicroseconds(MID_PWM);
  servoZ.writeMicroseconds(MID_PWM);
  while(true)
  {}
}
