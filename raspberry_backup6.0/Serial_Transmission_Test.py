import serial
import time
import numpy as np

ser = serial.Serial('/dev/ttyS0', 115200, timeout = 0.001)

count = 0

motor_value = 10.523

print("Here")

while True:
    values = bytearray([83, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, int(motor_value), 10, 10, 0, 55, 69])
    ser.write(values)

    time.sleep(0.01)

    textout = ser.read(19)
    # print(textout)
    # print(len(textout))
    if(len(textout) == 19):
        if(textout[0] == 65 and textout[17] == 56 and textout[18] == 66):
            data_bytes = np.array([textout[1], textout[2], textout[3], textout[4]], dtype=np.uint8)
            data_as_float = data_bytes.view(dtype=np.float32)
            #print(data_as_float)
    print(textout)

    if len(textout) == 0:
        count += 1

    print(count)
    
