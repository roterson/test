import socket
import time
import struct
import array
import numpy as np
from perpetualtimer import *

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

sock2.bind((HOST2, PORT2))

def write_to_server():
    sock1.sendto(bytearray([100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]), (HOST1, PORT1))

def read_from_server():
    data,_ = sock2.recvfrom(1024)
    arr =array.array('f')
    arr.frombytes(data)
    print(arr)

t1 = perpetualTimer(0.05, write_to_server, 'write_to_server')
t2 = perpetualTimer(0.05, read_from_server, 'read_from_server')

t1.start()
t2.start()
