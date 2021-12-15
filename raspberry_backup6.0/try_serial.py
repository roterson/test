import serial

ser = serial.Serial('/dev/ttyAMA0', 115200)
s = ser.read(100)
print(s)