import socket, select
import struct
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
#ser.port = '/dev/ttyACM0'     # for Linux
ser.timeout = 0.5
ser.open()

# ========================
# socket.SOCK_STREAM - TCP
# socket.SOCK_DGRAM - UDP
# AF_INET - IPv4
# ========================

# HOST = '192.168.0.114' # 340
#HOST = '192.168.100.125' # 501
HOST = '192.168.122.97'
# HOST = '127.0.0.1'
PORT = 1500


sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sck.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sck.bind((HOST, PORT))
sck.listen(1)
conn, addr = sck.accept()
print("Connected to: ", addr)

while True:
    if(ser.in_waiting > 0):
        ser_recv = ser.readline().decode('ascii') # what is received from serial
        if ser_recv:
            print(ser_recv)                

    data = conn.recv(1) # data from socket client
    if not data == b'~' and not data == b'':
        print(data)
        if (data == b'\x18'): # Ctrl + x
            ser.write(b'z')
            break
        ser.write(data) # send to serial (arduino)
        # if (data == b'5'):
        #     while True:
        #         if(ser.in_waiting > 0):
        #             ser_recv = ser.readline().decode('ascii') # what is received from serial
        #             if ser_recv:
        #                 print(ser_recv)
        #                 if (str(ser_recv) == "finish"):
        #                     print("real finish")
        #                     break
        #         data_bytes = conn.recv(4)
        #         print("to arduino")
        #         print(data_bytes)
        #         ser.write(data_bytes)
        
conn.close()







