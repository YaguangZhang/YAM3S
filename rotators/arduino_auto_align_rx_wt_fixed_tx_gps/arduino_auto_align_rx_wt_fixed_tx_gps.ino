/*
 Auto align the RX antenna via the pan and tilt kit controlled by Arduino via
 PWM signals.

 The program is controlled by character commands via the serial port to the
 Arduino:

  - r/R for starting auto alignment procedure after a successful initialization;
    a 'r' (indicating this script is running on the RX side) or 't' (indicating
    this script is running on the TX side) will be sent back from the Arduino as
    an acknowledgement.

  - s/S for stopping the program.

 We will collect sensor information and send it to the controller via the serial
 port. Debug information are prefixed by "#", while IMU readings are organized
 as a byte stream prefixed by "+":

  - IMU

 and GPS data are organized as a byte stream prefixed by "@":

  - Lat

 We used SparkFun Blackboard and Arduino IDE v1.8.1 to run this sketch. Required
 Arduino libraries include:

  - SparkFun BNO080 Cortex Based IMU
  - SparkFun Ublox

 Yaguang Zhang, Purdue University, 2020/01/28
*/

#include <Servo.h>

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// For enabling debugging over the serial port.
#define DEBUG false

// The PWM range for the continuous servos.
#define MAX_PWM 2000
#define MID_PWM 1500
#define MIN_PWM 1000

// For converting data to bytes for serial communication.
#define FLOAT_SIZE_IN_BYTE 4
#define UNSIGNED_LONG_SIZE_IN_BYTE 4

// X axis - Tilt (changing elevation); Z axis - Pan (changing azimuth).
int wpmPinX = 9, wpmPinZ = 10;
Servo servoX, servoZ;

// IMU data update period in millisecond.
int imuPeriodInMs = 100;

// Communication parameters.
int serialBoundRate = 9600;
long i2cClockInHz = 400000;
// For receiving command via the serial port from the controller.
char programCommand = 0;

// Fetch orientation data from the VR IMU.
BNO080 vrImu;

void setup() {
  // Sensors.
  Serial.begin(serialBoundRate);
  Wire.begin();

  if (vrImu.begin() == false) {
    Serial.println(
      "#Error: VR IMU (BNO080) not detected at default I2C address!");
    Serial.println(
      "#Freezing ...");
    while (true);
  }

  Wire.setClock(i2cClockInHz);

  vrImu.enableRotationVector(imuPeriodInMs);
  vrImu.enableMagnetometer(imuPeriodInMs);

  // Motors.
  servoX.attach(wpmPinX);
  servoZ.attach(wpmPinZ);

  servoX.writeMicroseconds(MID_PWM);
  servoZ.writeMicroseconds(MID_PWM);

  Serial.println(F("#Initialization succeeded! "
                   "Waiting for controllor's run command (r)..."));
  while(true) {
    if (Serial.available() > 0) {
      programCommand = toLowerCase(Serial.read());
      if (programCommand == 'r') {
        // Send 'r' to indicate this is a RX.
        Serial.println('r');
        Serial.println(F("#Auto cantenna alignment procedure started!"));
        break;
      }
    }
  }
}

void loop() {
  // React to the command from the serial port.
  if (Serial.available() > 0) {
    programCommand = toLowerCase(Serial.read());

    switch (programCommand) {
      case 's':
        printCommand(programCommand);
        Serial.println(F("#Freezing..."));
        while(true);
        break;
      default:
        break;
    }
  }

  // Read the IMU data.
  if (vrImu.dataAvailable() == true)
  {
    unsigned long upTime = millis();

    float quatReal = vrImu.getQuatReal();
    float quatI = vrImu.getQuatI();
    float quatJ = vrImu.getQuatJ();
    float quatK = vrImu.getQuatK();
    float quatRadianAccuracy = vrImu.getQuatRadianAccuracy();

    float magX = vrImu.getMagX();
    float magY = vrImu.getMagY();
    float magZ = vrImu.getMagZ();
    byte magAccuracy = vrImu.getMagAccuracy();

    // Send data over serial.
    Serial.print(F("+"));
    sendUnsignedLong(upTime);
    sendFloat(quatReal);
    sendFloat(quatI);
    sendFloat(quatJ);
    sendFloat(quatK);
    sendFloat(quatRadianAccuracy);
    sendFloat(magX);
    sendFloat(magY);
    sendFloat(magZ);
    printAccuracyLevel(magAccuracy);

    Serial.println();

    // Debug info.
    if (DEBUG) {
      Serial.print(F("#Up time: "));
      Serial.print((float) upTime, 0);

      Serial.println(" ms");

      Serial.print(F("#Rotation vector: "));
      Serial.print(quatReal, 2);
      Serial.print(F(","));
      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.print(F(","));
      Serial.print(quatRadianAccuracy, 2);

      Serial.println();

      Serial.print(F("#Magnetometer: "));
      Serial.print(magX, 2);
      Serial.print(F(","));
      Serial.print(magY, 2);
      Serial.print(F(","));
      Serial.print(magZ, 2);
      Serial.print(F(","));
      printAccuracyLevel(magAccuracy);

      Serial.println();
    }
  }
}

void printCommand (char command) {
  Serial.print(F("#Command received: "));
  Serial.println(command);
}

// Given an accuracy number, print to serial what it means.
void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Serial.print(F("U"));       // Unreliable
  else if(accuracyNumber == 1) Serial.print(F("L"));  // Low
  else if(accuracyNumber == 2) Serial.print(F("M"));  // Medium
  else if(accuracyNumber == 3) Serial.print(F("H"));  // High
}

void sendFloat (float arg)
{
  // Get access to the float as a byte-array and write the data to the serial.
  Serial.write((byte *) &arg, FLOAT_SIZE_IN_BYTE);
}

void sendUnsignedLong (unsigned long arg)
{
  // Get access to the unsigned long as a byte-array and write the data to the
  // serial.
  Serial.write((byte *) &arg, UNSIGNED_LONG_SIZE_IN_BYTE);
}