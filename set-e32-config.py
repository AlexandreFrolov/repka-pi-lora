#!/usr/bin/python3
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
print('Работает на платформе: ' + board_type + "\n")

import serial
import time
import sys
from time import sleep

NODE_CFG = [b'\xC0\x00\x0B\x1A\x0F\xC7', # 1: Node 1, Address 0x0B
            b'\xC0\x00\x0C\x1A\x0F\xC7', # 2: Node 2, Address 0x0C
            b'\xC0\x00\x0D\x1A\x0F\xC7', # 3: Node 3, Address 0x0D
            b'\xC0\x00\x0E\x1A\x0F\xC7'] # 4: Node 4, Address 0x0E

def wait_for_aux_pin():
    AUX = 18
    while not GPIO.input(AUX):
        sleep(0.04)

def gpio_init ():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    M0 = 22
    M1 = 27
    AUX = 18
    GPIO.setup(AUX,GPIO.IN)    
    GPIO.setup(M0,GPIO.OUT)
    GPIO.setup(M1,GPIO.OUT)
    GPIO.output(M0,GPIO.HIGH)
    GPIO.output(M1,GPIO.HIGH)
    time.sleep(0.1)
   
    if board_type == 'Repka-Pi':
        ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)
    elif board_type == 'Raspberry Pi':
        ser = serial.Serial("/dev/serial0", 9600, timeout=1)       
        
    ser.flushInput()
    return ser

def set_config(ser, node_id, new_cfg):
    try :
        if ser.isOpen() :
            wait_for_aux_pin()
            ser.write(new_cfg)
            time.sleep(1)
    except :
        if ser.isOpen() :
            ser.close()
        GPIO.cleanup()
    ser.flushInput()
    try :
        if ser.isOpen() :
            wait_for_aux_pin()
            ser.write(b'\xC1\xC1\xC1')
    except :
        if ser.isOpen() :
            ser.close()
        GPIO.cleanup()
    wait_for_aux_pin()
    received_data = ser.read(6)
    sleep(0.03)
    print('Node ' + str(node_id) + ' new config:')

    if sys.version_info[0] > 2:
      print(received_data.hex())
    else:
      print('{}'.format(received_data.encode('hex')))

def get_node_config():
    if len(sys.argv) != 2 or int(sys.argv[1]) < 1 or int(sys.argv[1]) > 4 :
        print("Please enter node id (1, 2, 3, 4)")
        sys.exit(0)

    node_id = int(sys.argv[1])
    new_cfg = NODE_CFG[node_id-1]
    print('Node ' + str(node_id) + ' set new config:')

    if sys.version_info[0] > 2:
      print(new_cfg.hex())
      confirm = input("Enter 'yes' to confirm: ")
    else:
      print('{}'.format(new_cfg.encode('hex')))
      confirm = raw_input("Enter 'yes' to confirm: ")

    if confirm != 'yes' :
    	print("cancelled")
    	sys.exit(0)
    return node_id, new_cfg

node_id, new_cfg = get_node_config()
ser = gpio_init()
set_config(ser, node_id, new_cfg)