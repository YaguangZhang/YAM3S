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
import time
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
from scipy.spatial.transform import Rotation as R

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

def waitForDataOnSerial(ser):
    '''
    Sleep until there is data shown up on the input serial port.
    '''
    timeToWaitBeforeCheckAgainInS = 0.01
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
            print(surfix + newline.decode("utf-8") )
        else:
            break

def sendSerialDataToDatabase(ser, cur, printSurfix=''):
    # The first line should be byte stream for the sensor data.
    newline = ser.readline()
    sys.getsizeof(newline)

def adjustServosWhenNecessary(ser, cur, printSurfix=''):
    newline = ser.readline()
    if len(newline) > 0:
        print(surfix + newline.decode("utf-8") )

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

        print("Signalling Arduino to start the auto antenna alignment procedure ...")
        serPortToArduino.write(b"r")
        # Read the acknowledgement.
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

        # For easy and secure remote access of the database, we will ssh forward the
        # MySQL port from the remote server to localhost (the controller of the
        # rotator). Please make sure the public ssh key of the controller machine is
        # registered at the server.
        print("\nForwarding port for accessing remote database ...")
        ssh_public_key = paramiko.RSAKey.from_private_key_file(
            bytes(settings['controllers'][controllerSide]['ssh']['private_key_dir'], encoding='utf-8'),
            password=settings['controllers'][controllerSide]['ssh']['private_key_password'])

        with SSHTunnelForwarder(
            (settings['server']['url'],
            settings['controllers'][controllerSide]['ssh']['port']),
            ssh_username=settings['controllers'][controllerSide]['ssh']['user_account'],
            ssh_pkey=ssh_public_key,
            remote_bind_address=(settings['server']['mysql_instance']['host'],
                settings['server']['mysql_instance']['port'])
        ) as tunnelledMySql:
            print("Succeeded!\n\nConnecting to remote database ...")
            databaseConnection = mysql.connector.connect(
                host='localhost',
                port=tunnelledMySql.local_bind_port,
                database=settings['server']['mysql_instance']['database'],
                user=settings['controllers'][controllerSide]['database_credential']['username'],
                password=settings['controllers'][controllerSide]['database_credential']['password'],
                auth_plugin=settings['server']['mysql_instance']['auth_plugin'])

            databaseCur = databaseConnection.cursor()
            print("Succeeded!\nAuto antenna alignment initiated ...")

            try:
                while(True):
                    # We are expecting one IMU data update in every 0.1 s.
                    waitForDataOnSerial(serPortToArduino)
                    # Fetching and processing all incoming messages from the Arduino.
                    sendSerialDataToDatabase(ser, cur, '    Arduino (Sensor Data): ')
                    # Compute and adjust the PMW signal if necessary.
                    adjustServosWhenNecessary(ser, cur, '    Arduino (Motor Adjustment): ')
            finally:
                databaseConnection.close()
                databaseCur.close()

if __name__ == '__main__':
    main()