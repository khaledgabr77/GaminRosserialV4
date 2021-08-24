#!/usr/bin/python
import serial
import math


ser = serial.Serial('/dev/ttyACM0', 115200)


while True:
    try:
        if (ser.inWaiting() > 0):
            data = ser.readline().decode().rstrip() # remove newline and carriage return characters
            data = data.split(',')
            motor = data[0]
            distance = data[1]
            print(motor, distance)
        else:
            print("nada")
    except:
        print('erro while decoding')
        ser.close()
        exit()


        #data = ser.readline().decode().rstrip() # remove newline and carriage return characters
        #data = data.split(',')
        #if len(data) >= 2:
        #    pos_motor = float(data[0])
        #    range_motor = float(data[1])
        #print("hello")