# The python controller to
#
#   - collect sensor readings from its Arduino core,
#   - push sensor records to a MySQL database on the server side,
#   - fetch servo adjustment PWM signal info from the server,
#   - send the servo adjustment info to Arduino.
#
# Developed and tested using Python 3.8.1 (with libraries SciPy, pyserial,
# sshtunnel and mysql-connector-python) and MySQL Community Server - GPL v8.0.19
# on Windows 10 machines. To install the Python libraries, one could run:
#
#   pip install scipy pyserial sshtunnel mysql-connector-python
#
# Yaguang Zhang, Purdue University, 2020/02/03

# For locating path, timing and logging.
import os
import time, datetime
import logging, sys
logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

# For loading settings.
import json

# For locating the serial port of the Arduino board.
import serial.tools.list_ports

# For ssh port forwarding.
import paramiko
from sshtunnel import SSHTunnelForwarder

# For MySQL operations.
import mysql.connector

# For receiving float type data over the serial port.
import struct

# For comprehending quaternions.
#   - from scipy.spatial.transform import Rotation as R

# For more math functions.
import numpy as np

# For receiving data from serial communication.
CHAR_SIZE_IN_BYTE  = 1          # char <=> short <=> int8_t

FLOAT_SIZE_IN_BYTE = 4          # float
UNSIGNED_LONG_SIZE_IN_BYTE = 4  # unsigned long <=> uint32_t
LONG_SIZE_IN_BYTE          = 4  #          long <=> int32_t
UNSIGNED_INT_SIZE_IN_BYTE = 2   # unsigned int <=> uint16_t
INT_SIZE_IN_BYTE          = 2   #          int <=> int16_t
BYTE_SIZE_IN_BYTE  = 1          # byte <=> unsigned short <=> uint8_t

# The PWM range for the continuous servos.
MIN_PWM = 1000
MID_PWM = 1500
MAX_PWM = 2000

def waitForDeviceOnSerial(deviceName, timeToWaitBetweenSerialScansInS=5):
    '''
    Keep scanning all the serial ports until one with description constaining
    the specific key word is found. Then, the corresponding port name is
    returned.
    '''
    logging.info("Scanning serial ports for keyword " + deviceName + " ...")

    serPortToDevice = None
    cntSerialScans = 0
    while not serPortToDevice:
        cntSerialScans = cntSerialScans+1
        logging.info("Trial " + str(cntSerialScans) + ":")
        curSerPorts = list(serial.tools.list_ports.comports())
        for p in curSerPorts:
            if deviceName in p.description:
                serPortToDevice = p.device
                print("    Found at port", serPortToDevice + "!")
                break

        if not serPortToDevice:
            logging.info("    Not found! Will try again after "
                + str(timeToWaitBetweenSerialScansInS) + "s ...")
            time.sleep(timeToWaitBetweenSerialScansInS)

    return serPortToDevice

def waitForDataOnSerial(ser, timeToWaitBeforeCheckAgainInS=0.01):
    '''
    Sleep until there is data shown up on the input serial port.
    '''
    while(ser.inWaiting()==0):
        time.sleep(timeToWaitBeforeCheckAgainInS)

def printAllSerData(ser, surfix=''):
    '''
    Print all the data available from the input serial.Serial port. One can
    specify a surfix for each line printed.
    '''
    while True:
        newline = ser.readline()
        if len(newline) > 0:
            print(surfix + newline.decode("utf-8").rstrip() )
        else:
            break

def receiveCharFromSerial(ser):
    '''
    Receive one char value from the serial byte stream.
    '''
    try:
        return ser.read(CHAR_SIZE_IN_BYTE).decode("utf-8")
    except:
        logging.warning("Unable to decode data!")
        return None

def receiveFloatFromSerial(ser):
    '''
    Receive one float value from the serial byte stream.
    '''
    try:
        return struct.unpack('<f', ser.read(FLOAT_SIZE_IN_BYTE))[0]
    except:
        logging.warning("Unable to decode data!")
        return None

def receiveUnsignedLongFromSerial(ser):
    '''
    Receive unsigned long value from the serial byte stream.
    '''
    try:
        return struct.unpack('<L', ser.read(UNSIGNED_LONG_SIZE_IN_BYTE))[0]
    except:
        logging.warning("Unable to decode data!")
        return None

def receiveLongFromSerial(ser):
    '''
    Receive unsigned long value from the serial byte stream.
    '''
    try:
        return struct.unpack('<l', ser.read(LONG_SIZE_IN_BYTE))[0]
    except:
        logging.warning("Unable to decode data!")
        return None

def receiveUnsignedIntFromSerial(ser):
    '''
    Receive unsigned int value from the serial byte stream.
    '''
    try:
        return struct.unpack('<H', ser.read(UNSIGNED_INT_SIZE_IN_BYTE))[0]
    except:
        logging.warning("Unable to decode data!")
        return None

def receiveIntFromSerial(ser):
    '''
    Receive unsigned int value from the serial byte stream.
    '''
    try:
        return struct.unpack('<h', ser.read(INT_SIZE_IN_BYTE))[0]
    except:
        logging.warning("Unable to decode data!")
        return None

def receiveByteFromSerial(ser):
    '''
    Receive unsigned long value from the serial byte stream.
    '''
    try:
        return struct.unpack('<B', ser.read(BYTE_SIZE_IN_BYTE))[0]
    except:
        logging.warning("Unable to decode data!")
        return None

def readImuPackageFromSerial(ser):
    '''
    Receive one IMU data package from the serial byte stream.
    '''
    upTimeInMs   = receiveUnsignedLongFromSerial(ser)
    quatReal = receiveFloatFromSerial(ser)
    quatI    = receiveFloatFromSerial(ser)
    quatJ    = receiveFloatFromSerial(ser)
    quatK    = receiveFloatFromSerial(ser)
    quatRadianAccuracy = receiveFloatFromSerial(ser)
    magX         = receiveFloatFromSerial(ser)
    magY         = receiveFloatFromSerial(ser)
    magZ         = receiveFloatFromSerial(ser)
    magAccuracy  = receiveCharFromSerial(ser)

    endOfPackage = ser.readline()
    assert endOfPackage == b'\r\n', "Expecting end of line!"

    return (upTimeInMs, quatReal, quatI, quatJ, quatK, quatRadianAccuracy,
            magX, magY, magZ, magAccuracy)

def readGpsPackageFromSerial(ser):
    '''
    Receive one GPS data package from the serial byte stream.
    '''
    upTimeInMs     = receiveUnsignedLongFromSerial(ser)
    timeOfWeekInMs = receiveUnsignedLongFromSerial(ser)
    # Naming convention: [a]In[U]Xe[N] equals to [a]In[U] times 10^N, where [a]
    # is the variable name and [U] is the unit name.
    latInDegXe7         = receiveLongFromSerial(ser)
    lonInDegXe7         = receiveLongFromSerial(ser)
    altInMmMeanSeaLevel = receiveLongFromSerial(ser)
    altInMmEllipsoid    = receiveLongFromSerial(ser)
    horAccuracyInMXe4 = receiveUnsignedLongFromSerial(ser)
    verAccuracyInMXe4 = receiveUnsignedLongFromSerial(ser)
    satsInView  = receiveByteFromSerial(ser)
    fixType     = receiveByteFromSerial(ser)
    year   = receiveUnsignedIntFromSerial(ser)
    month  = receiveByteFromSerial(ser)
    day    = receiveByteFromSerial(ser)
    hour   = receiveByteFromSerial(ser)
    minute = receiveByteFromSerial(ser)
    second = receiveByteFromSerial(ser)
    millisecond = receiveUnsignedIntFromSerial(ser)
    nanosecond  = receiveLongFromSerial(ser)
    speedInMPerSX3  = receiveLongFromSerial(ser)
    headingInDegXe5 = receiveLongFromSerial(ser)
    PDODXe2 = receiveUnsignedIntFromSerial(ser)

    endOfPackage = ser.readline()
    assert endOfPackage == b'\r\n', "Expecting end of line!"

    return (upTimeInMs, timeOfWeekInMs,
            latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel, altInMmEllipsoid,
            horAccuracyInMXe4, verAccuracyInMXe4, satsInView, fixType,
            year, month, day, hour, minute, second, millisecond, nanosecond,
            speedInMPerSX3, headingInDegXe5, PDODXe2)

def receiveDataFromSerial(ser, printSurfix=''):
    '''
    Receive one GPS data package from the serial byte stream.
    '''
    # We are expecting one IMU data update in every 0.1 s. To get a controller
    # system time with a precision of 1 ms, we will check the serial port more
    # often.
    waitForDataOnSerial(ser, 0.001)
    # Log the current system time as an integer value. We assume the controller
    # is way faster in processing (consuming) the serial data than the Arduino
    # board (generating the data), so that this timestamp can be used as the
    # data package arrive timestamp (with a tolerable offset).
    controllerUnixTimeInMs = int(time.time()*1000)

    # Read in the indication byte and proceed accordingling.
    while(ser.inWaiting()>0):
        indicationByte = receiveCharFromSerial(ser)
        if (indicationByte == '+'):
            # IMU data package (10 values): (upTimeInMs, quatReal, quatI, quatJ,
            # quatK, quatRadianAccuracy, magX, magY, magZ, magAccuracy)
            return (controllerUnixTimeInMs,)+readImuPackageFromSerial(ser)
        elif (indicationByte == '@'):
            # GPS data package (21 values): (upTimeInMs, timeOfWeekInMs,
            # latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel, altInMmEllipsoid,
            # horAccuracyInMXe4, verAccuracyInMXe4, satsInView, fixType, year,
            # month, day, hour, minute, second, millisecond, nanosecond,
            # speedInMPerSX3, headingInDegXe5, PDODXe2).
            return (controllerUnixTimeInMs,)+readGpsPackageFromSerial(ser)
        elif (indicationByte == '#'):
            # A message is received.
            logging.info(printSurfix + ser.readline().decode("utf-8").rstrip())
            return None
        else:
            logging.warning("Unknown serial data stream type: "
                + indicationByte + "!")
            return None

def fetchTxLoc(settings):
    '''
    Fetch the 3D location (latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel) of the
    TX from the settings JSON file.
    '''
    latInDegXe7Tx = int(
        settings['controllers']['tx']['gps_position']['latitude']*1e7)
    lonInDegXe7Tx = int(
        settings['controllers']['tx']['gps_position']['longitude']*1e7)
    altInMmMeanSeaLevelTx = int(
        settings['controllers']['tx']['gps_position']['altitude']*1e3)

    return(latInDegXe7Tx, lonInDegXe7Tx, altInMmMeanSeaLevelTx)

def createNewRecordSeries(settings, ser, db, cur):
    '''
    Add a new record in the record_series table on the server.

    - Output:
        * recordSeriesId
          - The id for the new record series entry in the database.
        * newGpsId
          - The id for the new RX GPS data entry in the database.
        * newSerialData
          - The GPS data used for initializing the record_series entry, which
            contains (controllerUnixTimeInMs, upTimeInMs, timeOfWeekInMs,
            latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel, altInMmEllipsoid,
            horAccuracyInMXe4, verAccuracyInMXe4, satsInView, fixType, year,
            month, day, hour, minute, second, millisecond, nanosecond,
            speedInMPerSX3, headingInDegXe5, PDODXe2).
        * (latInDegXe7Tx, lonInDegXe7Tx, altInMmMeanSeaLevelTx)
          - The latitude, longitude, and altitude relative to the mean see level
            for the TX.
    '''
    # SQL command to use.
    sqlCommand = '''INSERT INTO record_series (label,
             starting_up_time_in_ms_rx, starting_controller_unix_time_in_ms_rx,
             lat_in_deg_xe7_tx, lon_in_deg_xe7_tx, alt_in_mm_mean_sea_level_tx,
             starting_lat_in_deg_xe7_rx, starting_lon_in_deg_xe7_rx,
             starting_alt_in_mm_mean_sea_level_rx)
             VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)'''

    # Wait until a valid GPS package is received.
    flagValidGpsPackageReceived = False
    while(not flagValidGpsPackageReceived):
        try:
            newSerialData = receiveDataFromSerial(ser,
                '    Waiting for valid GPS data... Received: ')
            # GPS package generates 22 values.
            if (newSerialData is not None) and len(newSerialData)==22:
                (controllerUnixTimeInMs,
                    upTimeInMs, timeOfWeekInMs, latInDegXe7, lonInDegXe7,
                    altInMmMeanSeaLevel, altInMmEllipsoid,
                    horAccuracyInMXe4, verAccuracyInMXe4,
                    satsInView, fixType,
                    year, month, day, hour, minute, second,
                    millisecond, nanosecond, speedInMPerSX3, headingInDegXe5,
                    PDODXe2) = newSerialData
                # The GPS package has valid data if the location parameters are
                # nonzero.
                if all((latInDegXe7, lonInDegXe7, fixType)):
                    flagValidGpsPackageReceived = True
        except Exception as err:
            logging.warning(
                "Unknown error happened while waiting for valid GPS data!")
            logging.warning("    More info: {}".format(str(err)))

    # Assemble the new entry.
    labelPrefix = settings['measurement_attempt_label_prefix']
    label = labelPrefix + datetime.datetime.fromtimestamp(
        controllerUnixTimeInMs/1000.0)\
        .astimezone().strftime(" %Y-%m-%d %H:%M:%S %Z")
    (latInDegXe7Tx, lonInDegXe7Tx, altInMmMeanSeaLevelTx) = fetchTxLoc(settings)
    newData = (label, upTimeInMs, controllerUnixTimeInMs,
               latInDegXe7Tx, lonInDegXe7Tx, altInMmMeanSeaLevelTx,
               latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel)

    # Upload the new entry.
    cur.execute(sqlCommand, newData)
    db.commit()
    recordSeriesId = cur.lastrowid

    # Upload the GPS data, too.
    newGpsId = sendGpsDataToDatabase(recordSeriesId, newSerialData, 'rx', db, cur)

    logging.info("Record series #" + str(recordSeriesId) + " inserted.")
    logging.info("    New data: " + str(newData))

    return (recordSeriesId, newGpsId, newSerialData,
        (latInDegXe7Tx, lonInDegXe7Tx, altInMmMeanSeaLevelTx))

def updateCurrentRecordSeriesWithTxTimestamps():
    return None

def fetchNewRecordSeries(cur):
    '''
    Fetch the latest IMU and GPS information needed for antenna alignment.
    '''
    return None

def fetchCurrentRecordSeries(cur):
    '''
    Fetch the current record_series row.
    '''
    return None # TODO: Output currentRecordSeriesId.

def sendImuDataToDatabase(recordSeriesId, imuSerialData,
    controllerSide, db, cur):
    '''
    Upload IMU data to database.
    '''
    # SQL command to use.
    sqlCommand = "INSERT INTO " + controllerSide.lower() +'''_imu
             (record_series_id,
             controller_unix_time_in_ms, up_time_in_ms,
             quat_real, quat_i, quat_j, quat_k, quat_radian_accuracy,
             mag_x, mag_y, mag_z, mag_accuracy)
             VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)'''

    # Upload the new entry.
    cur.execute(sqlCommand, (recordSeriesId,)+imuSerialData )
    db.commit()
    imuId = cur.lastrowid

    logging.info("(Record series #" + str(recordSeriesId) + ") "
        + controllerSide.upper() + " IMU #"
        + str(imuId) + " inserted.")
    logging.info("    New data: " + str(imuSerialData))

    return imuId

def sendGpsDataToDatabase(recordSeriesId, gpsSerialData,
    controllerSide, db, cur):
    '''
    Upload GPS data to database.
    '''
    # SQL command to use.
    sqlCommand = "INSERT INTO " + controllerSide.lower() +'''_gps
             (record_series_id,
             controller_unix_time_in_ms, up_time_in_ms, time_of_week_in_ms,
             lat_in_deg_x_e7, lon_in_deg_x_e7,
             alt_in_mm_mean_sea_level, alt_in_mm_ellipsoid,
             hor_accuracy_in_m_xe4, ver_accuracy_in_m_xe4,
             sats_in_view, fix_type,
             year, month, day, hour, minute, second, millisecond, nanosecond,
             speed_in_m_per_sx3, heading_in_deg_xe5, pdod_xe2)
             VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s,
             %s, %s, %s, %s, %s, %s, %s, %s, %s, %s,
             %s, %s, %s)'''

    # Upload the new entry.
    cur.execute(sqlCommand, (recordSeriesId,)+gpsSerialData )
    db.commit()
    gpsId = cur.lastrowid

    logging.info("(Record series #" + str(recordSeriesId) + ") "
        + controllerSide.upper() + " GPS #"
        + str(gpsId) + " inserted.")
    logging.info("    New data: " + str(gpsSerialData))

    return gpsId

def adjustServos(ser, cur, printSurfix=''):
    pass

def fetchRxGps():
    pass

def setServoPwmSignals(ser, pwmX, pwmZ):
    '''
    Set PWM signal outputs on the Arduino board via the serial port ser.
    '''
    if pwmX is not None:
        serialComm = 'X'+str(int(round(pwmX)))+'\n'
        ser.write(serialComm.encode('utf-8'))
        logging.info("New PWD value for X-axis servo: " + str(pwmX))

    if pwmZ is not None:
        serialComm = 'Z'+str(int(round(pwmZ)))+'\n'
        ser.write(serialComm.encode('utf-8'))
        logging.info("New PWD value for Z-axis servo: " + str(pwmZ))

def stopServos(ser):
    '''
    Stop the servos.
    '''
    setServoPwmSignals(ser, MID_PWM, MID_PWM)

def freezeArduino(ser):
    '''
    Freeze the Arduino. This will stop the servos and freeze the Arduino.
    '''
    ser.write(b"s")

def setServoSpeeds(ser, speedX, speedZ,
                   minPwm = MIN_PWM, midPwd = MID_PWM, maxPwm = MAX_PWM):
    '''
    Set servo speeds (-1 to +1) via the serial port ser.

    The inputs speedX and speedZ should be in the range -1.0 to 1.0. A value of
    minPwm/midPwd/maxPwm will be set on the Arduino board for the speed value of
    -1.0/0.0/1.0 for the corresponding servos. Other PWM values will be computed
    via linear interpretation.
    '''
    if speedX is not None:
        speedX = min(max(float(speedX), -1.0), 1.0)
        pwmX = int(round(np.interp(speedX, (-1, 0, 1),
            (MIN_PWM, MID_PWM, MAX_PWM))))
    else:
        pwmX = None

    if speedZ is not None:
        speedZ = min(max(float(speedZ), -1.0), 1.0)
        pwmZ = int(round(np.interp(speedZ, (-1, 0, 1),
            (MIN_PWM, MID_PWM, MAX_PWM))))
    else:
        pwmZ = None

    setServoPwmSignals(ser, pwmX, pwmZ)

def main():
    curDirName = os.path.dirname(__file__)
    fullPathToSettings = os.path.join(curDirName, '../../settings.json')

    # Read system settings.
    with open(fullPathToSettings) as settings:
        settings = json.load(settings)

    # Step 1:
    #   - Connect with the arduino.

    # We will scan all the serial ports and connect to the first Arduino found.
    # For the SparkFun Blackboard, something like "USB-SERIAL CH340 (COM4)" will
    # show up in the port description.
    print("\nConnecting to Arduino ...")
    serPortToArduino = waitForDeviceOnSerial(
        settings['controllers']['arduino_serial_port']['keyword'])

    with serial.Serial(serPortToArduino,
        settings['controllers']['arduino_serial_port']['serial_baud_rate'],
        timeout=settings['controllers']['arduino_serial_port']['time_out_in_s']
    ) as serPortToArduino:
        print("Succeeded!")

        # Fetching and displaying all incoming messages from the Arduino.
        waitForDataOnSerial(serPortToArduino)
        printAllSerData(serPortToArduino,
            '    Arduino (Raw Message during Initialization): ')

        print("Signalling Arduino to start "
            + "the auto antenna alignment procedure ...")
        serPortToArduino.write(b"r")
        # Read the acknowledgement.
        waitForDataOnSerial(serPortToArduino)
        ackFromArduino = serPortToArduino.readline()

        # Determine which side (TX/RX) the controller is at.
        if ackFromArduino == b'r\r\n':
            controllerSide = 'rx'
        elif ackFromArduino == b't\r\n':
            controllerSide = 'tx'
        else:
            raise ValueError(
                "Unknown controller role ({ack}) requested by Arduino".format(
                ack=ackFromArduino))
        print("Succeeded!")

        # Step 2:
        #   - Connect to the remote mySQL database.

        # For easy and secure remote access of the database, we will ssh forward
        # the MySQL port from the remote server to localhost (the controller of
        # the rotator). Please make sure the public ssh key of the controller
        # machine is registered at the server.
        print("\nForwarding port for accessing remote database ...")
        ssh_public_key = paramiko.RSAKey.from_private_key_file(
            bytes(settings['controllers'][controllerSide]
                ['ssh']['private_key_dir'],
                encoding='utf-8'),
            password=settings['controllers'][controllerSide]
            ['ssh']['private_key_password'])

        with SSHTunnelForwarder(
            (settings['server']['url'],
                settings['controllers'][controllerSide]['ssh']['port']),
            ssh_username=settings['controllers'][controllerSide]
            ['ssh']['user_account'],
            ssh_pkey=ssh_public_key,
            remote_bind_address=(settings['server']['mysql_instance']['host'],
                settings['server']['mysql_instance']['port'])
        ) as tunnelledMySql:
            print("Succeeded!\n\nConnecting to remote database ...")
            databaseConnection = mysql.connector.connect(
                host='localhost',
                port=tunnelledMySql.local_bind_port,
                database=settings['server']['mysql_instance']['database'],
                user=settings['controllers'][controllerSide]
                ['database_credential']['username'],
                password=settings['controllers'][controllerSide]
                ['database_credential']['password'],
                auth_plugin=settings['server']['mysql_instance']['auth_plugin'])

            databaseCur = databaseConnection.cursor()
            print("Succeeded!\nAuto antenna alignment initiated ...")

            # Step 3:
            #   - Sensor data collection and
            #   - automatic antenna alignment.
            try:
                # Initialize the cache variables for checking when new data is
                # available for realign the antenna.

                # (latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel)
                currentGps     = None
                currentImuQuat = None # (quatReal, quatI, quatJ, quatK)
                currentImuMag  = None # (magX, magY, magZ)
                # (latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel)
                currentGpsCounterpart = None

                # We will need to adjust the servos if new GPS or IMU data
                # become available.
                flagNeedToAdjustServos = True

                if controllerSide == 'rx':
                    # Only the RX can create new record in the table
                    # record_series.
                    (currentRecordSeriesId, _,
                        currentGpsSerialData, txLoc) = createNewRecordSeries(
                        settings, serPortToArduino,
                        databaseConnection, databaseCur)
                    # Extract (latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel).
                    currentGps = currentGpsSerialData[3:6]
                    currentGpsCounterpart = txLoc
                elif controllerSide == 'tx':
                    # TODO: The RX will wait for new RX data entries and fetch
                    # and update the latest record_series record accordingly.
                    currentRecordSeriesId = fetchCurrentRecordSeries(databaseCur)
                    updateCurrentRecordSeriesWithTxTimestamps(databaseCur)
                    # Fetch (latInDegXe7, lonInDegXe7, altInMmMeanSeaLevel) for
                    # the TX.
                    currentGps = fetchTxLoc(settings)
                    # TODO: Fetch the latest RX loc.
                    currentGpsCounterpart = fetchRxGps()

                while(True):
                    # Fetch and process all incoming messages from the Arduino.
                    sensorData = receiveDataFromSerial(serPortToArduino,
                        '    Arduino (Sensor Data): ')

                    # GPS package generates 22 values.
                    if (sensorData is not None) and len(sensorData)==22:
                        newGpsSerialData = sensorData
                        sendGpsDataToDatabase(
                            currentRecordSeriesId, newGpsSerialData,
                            controllerSide, databaseConnection, databaseCur)
                        currentGps = newGpsSerialData[3:6]
                        flagNeedToAdjustServos = True
                    # IMU package generates 11 values.
                    elif (sensorData is not None) and len(sensorData)==11:
                        newImuSerialData = sensorData
                        sendImuDataToDatabase(
                            currentRecordSeriesId, newImuSerialData,
                            controllerSide, databaseConnection, databaseCur)
                        currentImuQuat = newImuSerialData[1:5]
                        currentImuMag  = newImuSerialData[6:9]
                        flagNeedToAdjustServos = True

                    # Counter part GPS location can change if this is the TX
                    # side.
                    if controllerSide == 'tx':
                        lastGpsCounterpart = currentGpsCounterpart
                        currentGpsCounterpart = fetchRxGps()
                        if ((lastGpsCounterpart is not None)
                             and (currentGpsCounterpart is not None)
                             and (currentGpsCounterpart != lastGpsCounterpart)):
                            flagNeedToAdjustServos = True

                    # Compute and adjust the PMW signals if necessary.
                    if flagNeedToAdjustServos:
                        adjustServos(serPortToArduino, currentGps,
                            '    Arduino (Motor Adjustment): ')
                        flagNeedToAdjustServos = False
            finally:
                # TODO
                stopServos()
                databaseConnection.close()
                databaseCur.close()

if __name__ == '__main__':
    main()