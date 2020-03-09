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

  - upTime
  - quatReal, quatI, quatJ, quatK
  - quatRadianAccuracy
  - magX, magY, magZ
  - magAccuracy

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
#include "SparkFun_Ublox_Arduino_Library.h"

// For enabling debugging over the serial port.
#define DEBUG true

// The PWM range for the continuous servos.
#define MAX_PWM 2000
#define MID_PWM 1500
#define MIN_PWM 1000

// For converting data to bytes for serial communication.
#define FLOAT_SIZE_IN_BYTE         4
#define UNSIGNED_LONG_SIZE_IN_BYTE 4
#define LONG_SIZE_IN_BYTE          4
#define UNSIGNED_INT_SIZE_IN_BYTE  2
#define BYTE_SIZE_IN_BYTE          1

// X axis - Tilt (changing elevation); Z axis - Pan (changing azimuth).
int wpmPinX = 9, wpmPinZ = 10;
Servo servoX, servoZ;

// IMU data update period in millisecond.
int imuPeriodInMs = 100;
// GPS data update period in millisecond.
int gpsPeriodInMs = 100;

// Communication parameters.
long serialBoundRate = 115200;
long i2cClockInHz    = 400000;

// For receiving command via the serial port from the controller.
char programCommand = 0;

// For orientation data from the VR IMU.
BNO080 vrImu;

// RTK GPS.
SFE_UBLOX_GPS rtkGps;

void setup() {
  // Serial COM.
  Serial.begin(serialBoundRate);
  //Wait for the controller to open terminal.
  while (!Serial);

  // I2C.
  Wire.begin();
  Wire.setClock(i2cClockInHz);

  // IMU.
  if (vrImu.begin() == false) {
    Serial.println(F(
      "#Error: VR IMU (BNO080) not detected at default I2C address!"));
    Serial.println(F("#Freezing ..."));
    while (true);
  }
  vrImu.enableRotationVector(imuPeriodInMs);
  vrImu.enableMagnetometer(imuPeriodInMs);

  // RTK GPS.
  if (rtkGps.begin() == false)
  {
    Serial.println(F(
      "#Error: RTK GPS (u-blox) not detected at default I2C address!"));
    Serial.println(F("#Freezing ..."));
    while (true);
  }
  // Set the I2C port to output UBX only (turn off NMEA noise).
  rtkGps.setI2COutput(COM_TYPE_UBX);
  rtkGps.setNavigationFrequency((int) 1000.0/gpsPeriodInMs);
  // Configure the GPS to update navigation reports automatically.
  rtkGps.setAutoPVT(true);
  rtkGps.saveConfiguration();

  // Motors.
  servoX.attach(wpmPinX);
  servoZ.attach(wpmPinZ);

  servoX.writeMicroseconds(MID_PWM);
  servoZ.writeMicroseconds(MID_PWM);

  // If there is no errors in the Arduino initialization, this will be the first
  // message to the controller.
  Serial.println(F("#Initialization succeeded! "
                   "Waiting for controllor's run command (r)..."));
  while(true) {
    if (Serial.available() > 0) {
      programCommand = toLowerCase(Serial.read());
      if (programCommand == 'r') {
        // Send 'r' to indicate this is a RX.
        Serial.println(F("r"));
        Serial.println(F("#Auto antenna alignment procedure started!"));
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
  if (vrImu.dataAvailable() == true) {
    // Arduino up time.
    unsigned long upTimeInMs = millis();

    // Fetch IMU data.
    float quatReal = vrImu.getQuatReal();
    float quatI = vrImu.getQuatI();
    float quatJ = vrImu.getQuatJ();
    float quatK = vrImu.getQuatK();
    float quatRadianAccuracy = vrImu.getQuatRadianAccuracy();

    float magX = vrImu.getMagX();
    float magY = vrImu.getMagY();
    float magZ = vrImu.getMagZ();
    byte magAccuracy = vrImu.getMagAccuracy();

    // Debug info.
    if (DEBUG) {
      Serial.print(F("#Up time: "));
      Serial.print(upTimeInMs);
      Serial.println(F(" ms"));

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

    // Send data over serial.
    Serial.print(F("+"));
    sendUnsignedLong(upTimeInMs);
    sendFloat(quatReal);
    sendFloat(quatI);
    sendFloat(quatJ);
    sendFloat(quatK);
    sendFloat(quatRadianAccuracy);
    sendFloat(magX);
    sendFloat(magY);
    sendFloat(magZ);
    printAccuracyLevel(magAccuracy);

    // End of package.
    Serial.println();
  }

  // Read the GPS data. Reference for data types:
  //   - long <=> int32_t
  //   - int  <=> int16_t
  //   - byte <=> int8_t
  if (rtkGps.getPVT() == true) {
    // Arduino up time.
    unsigned long upTimeInMs = millis();

    // Fetch high-precision GPS data.
    unsigned long timeOfWeekInMs = rtkGps.getTimeOfWeek();

    long latXe7 = rtkGps.getHighResLatitude();
    long lonXe7 = rtkGps.getHighResLongitude();
    long altInMmMeanSeaLevel = rtkGps.getMeanSeaLevel();
    long altInMmEllipsoid = rtkGps.getElipsoid();

    unsigned long horAccuracy = rtkGps.getHorizontalAccuracy();
    unsigned long verAccuracy = rtkGps.getVerticalAccuracy();

    // Extra information.
    byte satsInView = rtkGps.getSIV();
    byte fixType    = rtkGps.getFixType();

    unsigned int year = rtkGps.getYear();
    byte month        = rtkGps.getMonth();
    byte day          = rtkGps.getDay();
    byte hour   = rtkGps.getHour();
    byte minute = rtkGps.getMinute();
    byte second = rtkGps.getSecond();
    unsigned int millisecond = rtkGps.getMillisecond();
    // This value includes millisecond and can be negative.
    long nanosecond          = rtkGps.getNanosecond();

    // TODO: Speed; heading.

    // Debug info.
    if (DEBUG) {
      Serial.print(F("#Up time: "));
      Serial.print(upTimeInMs);
      Serial.println(F(" ms"));

      Serial.print(F("#GPS Time of the week: "));
      Serial.print(timeOfWeekInMs);
      Serial.println(F(" ms"));

      Serial.print(F("#GPS (latXe7, lonXe7): ("));
      Serial.print(latXe7);
      Serial.print(F(","));
      Serial.print(lonXe7);
      Serial.println(F(")"));

      Serial.print(F("#GPS (altInMmMeanSeaLevel, altInMmEllipsoid): ("));
      Serial.print(altInMmMeanSeaLevel);
      Serial.print(F(","));
      Serial.print(altInMmEllipsoid);
      Serial.println(F(")"));

      Serial.print(F("#GPS (horAccuracy, verAccuracy): ("));
      Serial.print(horAccuracy);
      Serial.print(F(","));
      Serial.print(verAccuracy);
      Serial.println(F(")"));

      Serial.print(F("#GPS satellites in view: "));
      Serial.println(satsInView);

      Serial.print(F("#GPS fixed type: "));
      Serial.println(fixType);
      Serial.println(F("#    0: no fix"));
      Serial.println(F("#    1: dead reckoning only"));
      Serial.println(F("#    2: 2D-fix"));
      Serial.println(F("#    3: 3D-fix"));
      Serial.println(F("#    4: GNSS + dead reckoning combined"));
      Serial.println(F("#    5: time only fix"));

      Serial.print(F("#Human-readable GPS time: "));
      Serial.print(year);
      Serial.print(F("-"));
      Serial.print(month);
      Serial.print(F("-"));
      Serial.print(day);
      Serial.print(F(" "));
      Serial.print(hour);
      Serial.print(F(":"));
      Serial.print(minute);
      Serial.print(F(":"));
      Serial.print(second);
      Serial.print(F("."));
      //Pretty print leading zeros.
      if (millisecond < 100) Serial.print(F("0"));
      if (millisecond < 10)  Serial.print(F("0"));
      Serial.println(millisecond);

      Serial.print(F("#GPS nano seconds: "));
      Serial.println(nanosecond);
    }

    // Send data over serial.
    Serial.print(F("@"));
    sendUnsignedLong(upTimeInMs);
    sendUnsignedLong(timeOfWeekInMs);
    sendLong(latXe7);
    sendLong(lonXe7);
    sendLong(altInMmMeanSeaLevel);
    sendLong(altInMmEllipsoid);
    sendUnsignedLong(horAccuracy);
    sendUnsignedLong(verAccuracy);
    sendByte(satsInView);
    sendByte(fixType);
    sendUnsignedInt(year);
    sendByte(month);
    sendByte(day);
    sendByte(hour);
    sendByte(minute);
    sendByte(second);
    sendUnsignedInt(millisecond);
    sendLong(nanosecond);

    // End of package.
    Serial.println();
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

void sendLong (long arg)
{
  // Get access to the long as a byte-array and write the data to the serial.
  Serial.write((byte *) &arg, LONG_SIZE_IN_BYTE);
}

void sendUnsignedInt (unsigned int arg)
{
  // Get access to the unsigned long as a byte-array and write the data to the
  // serial.
  Serial.write((byte *) &arg, UNSIGNED_INT_SIZE_IN_BYTE);
}

void sendByte (byte arg)
{
  // Get access to the byte as a byte-array and write the data to the serial.
  Serial.write((byte *) &arg, BYTE_SIZE_IN_BYTE);
}
