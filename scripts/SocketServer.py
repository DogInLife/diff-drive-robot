import socket, select
import serial
import sys

'''
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
'''

ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'   # for Raspberry
# s.port = '/dev/ttyACM0'     # for Linux
ser.timeout = 0.5
ser.open()

# ========================
# socket.SOCK_STREAM - TCP
# socket.SOCK_DGRAM - UDP
# AF_INET - IPv4
# ========================

HOST = '192.168.0.114'
# HOST = '127.0.0.1'
PORT = 1500


sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sck.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sck.bind((HOST, PORT))
sck.listen(1)
conn, addr = sck.accept()
print("Connected to: ", addr)


while True:
    ser_in = conn.recv(1) # what is sent to serial
    print(ser_in)
    if (ser_in == b'\x03'):
        break
    ser.write(ser_in)

    # if (ser.in_waiting > 0):
    ser_out = ser.read(ser.in_waiting).decode('ascii') # what is received from serial
    print(ser_out)

conn.close()







