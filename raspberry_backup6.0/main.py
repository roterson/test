from BNO055_IMU import *
import qwiic
import RPi.GPIO as GPIO
import serial
import numpy as np
import socket,os
import time

# ToF Setup

XSHUT_1 = 18
XSHUT_2 = 6
XSHUT_3 = 7

GPIO.setmode(GPIO.BCM)
GPIO.setup(XSHUT_1, GPIO.OUT)
GPIO.output(XSHUT_1, GPIO.LOW)

GPIO.setup(XSHUT_2, GPIO.OUT)
GPIO.output(XSHUT_2, GPIO.LOW)

GPIO.setup(XSHUT_3, GPIO.OUT)
GPIO.output(XSHUT_3, GPIO.LOW)

GPIO.output(XSHUT_1, GPIO.HIGH)
ToF1 = qwiic.QwiicVL53L1X()
if (ToF1.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 1 online!\n")
ToF1.set_i2c_address(0x21)
ToF1.start_ranging()

'''
GPIO.output(XSHUT_2, GPIO.HIGH)
ToF2 = qwiic.QwiicVL53L1X()
if (ToF2.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 2 online!\n")
ToF2.set_i2c_address(0x22)
ToF2.start_ranging()

GPIO.output(XSHUT_3, GPIO.HIGH)
ToF3 = qwiic.QwiicVL53L1X()
if (ToF3.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 3 online!\n")
ToF3.set_i2c_address(0x23)
ToF3.start_ranging()
'''

# ToF Setup

#IMU SETUP
IMU = BNO055()
if IMU.begin() is not True:
    print("Error initializing device")
    exit()
time.sleep(1)
IMU.setExternalCrystalUse(True)
#IMU SETUP

#Serial Setup
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout = 0.01)
ser.flushInput()
ser.flushOutput()
time.sleep(0.01)
#Serial Setup

'''
#Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
sock.bind(('127.0.0.1', 12345))  
sock.listen(5)
'''



while True:

    #READ ToFs

    print(ToF1.get_distance())
    #print(ToF2.get_distance())
    #print(ToF3.get_distance())

    #READ ToFs

    #IMU READ
    print(IMU.getVector(BNO055.VECTOR_EULER))
    #IMU READ

    #Serial Write then Read
    values = bytearray([83, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 0, 10, 10, 0, 55, 69])
    ser.write(values)
    
    time.sleep(0.005)

    textout = ser.read(19)
#     print(len(textout))

    if(len(textout) == 19):
        if(textout[0] == 65 and textout[17] == 56 and textout[18] == 66):
            data_bytes = np.array([textout[1], textout[2], textout[3], textout[4]], dtype=np.uint8)
            data_as_float = data_bytes.view(dtype=np.float32)
            print(data_as_float)
    #Serial Write then Read
            
    time.sleep(0.005)




