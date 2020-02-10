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
 port. Debug information are prefixed by "#", while sensor readings are
 organized as:

  - IMU

 We used SparkFun Blackboard and Arduino IDE v1.8.1 to run this sketch. Required
 Arduino libraries include:

  - SparkFun BNO080 Cortex Based IMU
  - SparkFun Ublox

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
    while (1);
  }

  Wire.setClock(i2cClockInHz);

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
        Serial.println(F("#Auto antenna alignment procedure started!"));
        break;
      }
    }
  }
}

void loop() {
  // React to the command from the serial port.
  while (Serial.available() > 0) {
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
}

void printCommand (char command) {
  Serial.print(F("#Command received: "));
  Serial.println(command);
}
