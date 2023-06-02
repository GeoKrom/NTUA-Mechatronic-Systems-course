import serial
import keyboard
import time 

# from serial.tools import list_ports
# ports = list(list_ports.comports())
# print(ports)

ser=serial.Serial("/dev/tty10",9600)
ser.flush()
time.sleep(2)

while True:
    if keyboard.is_pressed('w'):
        ser.write(b'F')
        time.sleep(2)
    elif keyboard.is_pressed("s"):
        ser.write(b'B')
        time.sleep(2)
    elif keyboard.is_pressed("d"):
        ser.write(b'R')
        time.sleep(2)
    elif keyboard.is_pressed("a"):
        ser.write(b'L')
        time.sleep(2)
