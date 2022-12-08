import serial
import os
import sys, select, termios, tty
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'   # for Raspberry
ser.timeout = 0.5
ser.open()

vel = 30

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = '~'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    print("Current speed: ", vel)
    settings = termios.tcgetattr(sys.stdin)

    try:
        while True:
            if(ser.in_waiting > 0):
                ser_recv = ser.readline().decode('ascii') # what is received from serial
                print(ser_recv)
        
            key = getKey()
            
            if key:
                data = str.encode(key)
                if not data == b'~':
                    print(data)
                    if (data == b'\x18'): # Ctrl + x
                        ser.write(b's')
                        break
                    ser.write(data) 
            
            if(key == '\x03'): # ctrl+c
                break

            '''
            if(key == 'e'):
                vel += 5
                print("Current speed: ", vel)
            if(key == 'q'):
                vel -= 5
                print("Current speed: ", vel)
            '''

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)












