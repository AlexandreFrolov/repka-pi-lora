#!/usr/bin/python
# -*- coding: UTF-8 -*-

def get_board_type():
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip()
            if 'Raspberry Pi' in model:
                return 'Raspberry Pi'
            elif 'Repka-Pi' in model:
                return 'Repka-Pi'
            else:
                return 'Unknown'
    except IOError:
        return 'Unknown'

board_type = get_board_type()

if board_type == 'Repka-Pi':
    import RepkaPi.GPIO as GPIO
    GPIO.setboard(GPIO.REPKAPI3)
elif board_type == 'Raspberry Pi':
    import RPi.GPIO as GPIO

print(board_type + "\n")

import serial
import time
import sys
from time import sleep
import json

NODE_ADDR_CHAN = [b'\x00\x0B\x0F',
                  b'\x00\x0C\x0F',
                  b'\x00\x0D\x0F',
                  b'\x00\x0E\x0F']


def gpio_init ():
#    GPIO.setboard(GPIO.REPKAPI3)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    M0 = 22
    M1 = 27
    GPIO.setup(M0,GPIO.OUT)
    GPIO.setup(M1,GPIO.OUT)
    GPIO.output(M0,GPIO.LOW)
    GPIO.output(M1,GPIO.LOW)
    time.sleep(1)
    
    if board_type == 'Repka-Pi':
        ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)
    elif board_type == 'Raspberry Pi':
        ser = serial.Serial("/dev/serial0", 9600, timeout=1)
    
    ser.flushInput()
    print(ser.name)
    return ser


def wait_for_cmd(node):
    print('return data to node: ' + str(NODE_ADDR_CHAN[node]))
    try :
        while True:
            received_data = ser.readline()
            sleep(0.03)
            data_left = ser.inWaiting() 
            received_data += ser.read(data_left)
            rec = received_data.decode("utf-8").strip()
            if received_data:
              print(received_data)
              print(rec)
            else:
              print('.', end='')
        
            if 'getData' in rec:
                if ser.isOpen() :
                    ser.write(NODE_ADDR_CHAN[node])
                    s_data = '111;222;333;444\r\n'.encode()
                    ser.write(s_data)
#                    ser.write('111;222;333;444 \r\n')
                    print('Sended')
                    print(s_data)
    except :
        if ser.isOpen() :
            ser.close()
            GPIO.cleanup()
    return

print(serial.__version__)
ser = gpio_init()
wait_for_cmd(3)
