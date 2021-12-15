import socket
import struct
from perpetualtimer import *
from BNO055_IMU import *
import qwiic
import RPi.GPIO as GPIO
import serial
import numpy as np
import socket,os
import time
import threading

# ToF Setup

XSHUT_1 = 18
XSHUT_2 = 20
XSHUT_3 = 21
XSHUT_4 = 16
XSHUT_5 = 7

GPIO.setmode(GPIO.BCM)
GPIO.setup(XSHUT_1, GPIO.OUT)
GPIO.output(XSHUT_1, GPIO.LOW)

GPIO.setup(XSHUT_2, GPIO.OUT)
GPIO.output(XSHUT_2, GPIO.LOW)

GPIO.setup(XSHUT_3, GPIO.OUT)
GPIO.output(XSHUT_3, GPIO.LOW)

GPIO.setup(XSHUT_4, GPIO.OUT)
GPIO.output(XSHUT_4, GPIO.LOW)

GPIO.setup(XSHUT_5, GPIO.OUT)
GPIO.output(XSHUT_5, GPIO.LOW)


GPIO.output(XSHUT_1, GPIO.HIGH)

time.sleep(0.2)

ToF1 = qwiic.QwiicVL53L1X()
if (ToF1.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 1 online!\n")
time.sleep(0.2)
ToF1.set_i2c_address(0x21)
time.sleep(0.2)
ToF1.start_ranging()

time.sleep(0.2)

GPIO.output(XSHUT_2, GPIO.HIGH)

time.sleep(0.2)

ToF2 = qwiic.QwiicVL53L1X()
if (ToF2.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 2 online!\n")
time.sleep(0.2)
ToF2.set_i2c_address(0x22)
time.sleep(0.2)
ToF2.start_ranging()

time.sleep(0.2)


GPIO.output(XSHUT_3, GPIO.HIGH)

time.sleep(0.2)

ToF3 = qwiic.QwiicVL53L1X()
if (ToF3.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 3 online!\n")
time.sleep(0.2)
ToF3.set_i2c_address(0x23)
time.sleep(0.2)
ToF3.start_ranging()

time.sleep(0.2)

GPIO.output(XSHUT_4, GPIO.HIGH)

time.sleep(0.2)

ToF4 = qwiic.QwiicVL53L1X()
if (ToF4.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 4 online!\n")
time.sleep(0.2)
ToF4.set_i2c_address(0x24)
time.sleep(0.2)
ToF4.start_ranging()

time.sleep(0.2)

GPIO.output(XSHUT_5, GPIO.HIGH)

time.sleep(0.2)

ToF5 = qwiic.QwiicVL53L1X()
if (ToF5.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 5 online!\n")
time.sleep(0.2)
ToF4.set_i2c_address(0x25)
time.sleep(0.2)
ToF4.start_ranging()

# ToF Setup

time.sleep(0.2)

#IMU SETUP
IMU = BNO055()
time.sleep(0.2)
if IMU.begin() is not True:
    print("Error initializing device")
    exit()
time.sleep(0.2)
IMU.setExternalCrystalUse(True)
#IMU SETUP

#Serial Setup
ser = serial.Serial('/dev/ttyS0', 115200, timeout = 0.01)
ser.flushInput()
ser.flushOutput()
time.sleep(0.01)
#Serial Setup

#UDP Transmission

# HOST1 = Commands
# HOST2 = Feedback

#HOST1 = '192.168.1.33'  # Standard loopback interface address (localhost)
HOST1 = '0.0.0.0'
PORT1 = 50005

#HOST2 = '192.168.1.35'  # Standard loopback interface address (localhost)
HOST2 = '0.0.0.0'
PORT2 = 50006        # Port to listen on (non-privileged ports are > 1023)

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock1.bind((HOST1, PORT1))
#sock2.connect((HOST2, PORT2))

#sock1.settimeout(0.0)
#sock2.settimeout(0.0)

motor_commands = bytearray([90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90])
relay_commands = bytearray([10, 10])
# transmission_array_STM32 = bytearray([83, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 10, 10, 0, 55, 69])
FSR_temp = bytearray([65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 66])
data_from_Simulink = b'A'
data_to_Simulink = b'A'
countdown_flag = 0

#UDP Tranmissions

def UDP_Transmit():

    global data_to_Simulink
    global FSR_temp
    #FSRdata=FSR_temp[1:17]
    #intdata1 = struct.unpack("f", FSRdata[0:4])
    #print(intdata1)
    
    try:
        Read_Sensors_Compose_Data()
        #print(vel1)
        '''distancex = np.mean(dis1)
        distancey = np.mean(dis2)
        distancez = np.mean(dis3)'''
    
        #print(velocity1)
        sock2.sendto(data_to_Simulink, (HOST2, PORT2))
        #print(vel1)
        #print("Transmission_Successful")
    except:
        1

def UDP_Recieve():

    global countdown_flag
    global motor_commands

    try:
        
        motor_commands = sock1.recv(1024)
        #print(motor_commands)
        if len(motor_commands) == 12:
            countdown_flag = 1
            #print("Recvd")
            #print(motor_commands.hex())

    except:
        1
            

def Read_Sensors_Compose_Data():

    global data_to_Simulink
    global FSR_temp
    
    
    #Serial Write then Read
    
    Write_STM32()

    '''distance1 = ToF1.get_distance()
    time.sleep(0.001)
    distance2 = ToF2.get_distance()
    time.sleep(0.001)
    distance3 = ToF3.get_distance()
    time.sleep(0.001)'''
    
    distance_back = ToF1.get_distance()
    time.sleep(0.001)
    distance_right = ToF2.get_distance()
    time.sleep(0.001)
    distance_down = ToF3.get_distance()
    time.sleep(0.001)
    distance_front = ToF4.get_distance()
    time.sleep(0.001)
    distance_left = ToF5.get_distance()
    time.sleep(0.001)
    
    #print(distance_front,distance_back,distance_right,distance_left,distance_down)
    
    distance_data = bytearray(struct.pack("f",distance_front)) + bytearray(struct.pack("f",distance_back)) + bytearray(struct.pack("f",distance_left)) + bytearray(struct.pack("f",distance_right)) + bytearray(struct.pack("f",distance_down))
    
    #distance3 = 500

    #print([distance1, distance2, distance3])

    #distance_sensors = bytearray(struct.pack("f", distance1)) +  bytearray(struct.pack("f", distance2)) + bytearray(struct.pack("f", distance3))

    quaternion = IMU.getQuat()
    #print(quaternion[0])

    IMU_Readings = bytearray(struct.pack("f", quaternion[0]))\
                   + bytearray(struct.pack("f", quaternion[1]))\
                   + bytearray(struct.pack("f", quaternion[2]))\
                   + bytearray(struct.pack("f", quaternion[3]))
    
    
    #print(textout)
    
    #textout = bytearray([0, 10, 20, 30, 40, 10, 20, 30, 40, 10, 20, 30, 40, 10, 20, 30, 40, 10, 20])
    #print(len(textout))
    
    textout = ser.read(19)
    #print(textout)

    '''if len(textout) == 19:
        for i in range(len(textout)):
            if (textout[i] == 65 and textout[(i + len(textout) - 1) % len(textout)] == 66):
                for z in range(len(textout)):
                    FSR_temp[z] = textout[(i+z) % len(textout)]'''
                
    FSR = textout[1:17]
    #print(FSR)

    data_to_Simulink = distance_data + IMU_Readings + FSR
    #print(data_to_Simulink)

def Write_STM32():

    global relay_commands
    global motor_commands

    header = bytearray([83])
    terminators = bytearray([0, 55, 69])

    transmission_array = header + motor_commands + relay_commands + terminators
    ser.write(transmission_array)
    #print(list(transmission_array))

def Countdown_Timer():
    
    global countdown_flag
    global relay_commands
    
    try:
        if countdown_flag == 1:
            relay_commands = bytearray([200, 200])


        if countdown_flag == 0:
            relay_commands = bytearray([10, 10])

        #print(countdown_flag)
        countdown_flag = 0
        
        
    except:
        1
        
'''def velocity_cal():
    global dis1
    global dis2
    global dis3
    
    distance_back = ToF1.get_distance()
    time.sleep(0.001)
    distance_right = ToF2.get_distance()
    time.sleep(0.001)
    distance_down = ToF3.get_distance()
    time.sleep(0.001)
    distance_front = ToF4.get_distance()
    time.sleep(0.001)
    distance_left = ToF5.get_distance()
    time.sleep(0.001)
    
    print(distance_front,distance_back,distance_right,distance_left,distance_down)'''
    
    
    
     
print("Started")
global vel_Ts
global dis1
global dis2
global dis3
global old_dis1
global old_dis2
global old_dis3
dis1 = [0, 0, 0, 0, 0]
dis2 = [0, 0, 0, 0, 0]
dis3 = [0, 0, 0, 0, 0]
vel_Ts = 0.04
old_dis1 = 0
old_dis2 = 0
old_dis3 = 0

t1 = perpetualTimer (0.05, UDP_Transmit, 'UDP_Transmit')
t2 = perpetualTimer (0.01, UDP_Recieve, 'UDP_Recieve')
#t3 = perpetualTimer (0.01, Read_Sensors_Compose_Data, 'Read_Sensors_Compose_Data')
#t4 = perpetualTimer (0.01, Write_STM32, 'Write_STM32')
t5 = perpetualTimer (2, Countdown_Timer, 'Countdown_Timer')

t1.start()
t2.start()
#t3.start()
#t4.start()
t5.start()






