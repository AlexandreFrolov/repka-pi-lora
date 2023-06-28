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
    import psutil

print(board_type + "\n")

import serial
import time
import sys
from time import sleep
import json
import subprocess


NODE_ADDR_CHAN = [b'\x00\x0B\x0F',
                  b'\x00\x0C\x0F',
                  b'\x00\x0D\x0F',
                  b'\x00\x0E\x0F']

def wait_for_aux_pin():
    AUX = 18
    while not GPIO.input(AUX):
        sleep(0.04)
        
def gpio_init ():
#    GPIO.setboard(GPIO.REPKAPI3)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    M0 = 22
    M1 = 27
    AUX = 18
    GPIO.setup(AUX,GPIO.IN)    
    GPIO.setup(M0,GPIO.OUT)
    GPIO.setup(M1,GPIO.OUT)
    GPIO.output(M0,GPIO.LOW)
    GPIO.output(M1,GPIO.LOW)
    time.sleep(1)
    
    if board_type == 'Repka-Pi':
        ser = serial.Serial("/dev/ttyS0", 9600, timeout=5)
    elif board_type == 'Raspberry Pi':
        ser = serial.Serial("/dev/serial0", 9600, timeout=5)
    
    ser.flushInput()
    print(ser.name)
    return ser

def get_sensor_temperatures():

    output = subprocess.check_output(["sensors", "-j"])
    sensor_data = json.loads(output)
    if board_type == 'Repka-Pi':
        cpu_temp = sensor_data["cpu_thermal-virtual-0"]["temp1"]["temp1_input"]
        gpu_temp = sensor_data["gpu_thermal-virtual-0"]["temp1"]["temp1_input"]

    elif board_type == 'Raspberry Pi':
        cpu_temp = sensor_data["cpu_thermal-virtual-0"]["temp1"]["temp1_input"]
        gpu_temp = 0
    return f"{int(cpu_temp)};{int(gpu_temp)}       \r\n"
        
def wait_for_cmd(address):
    print('return data to node: ' + str(address))
    try :
        while True:
            received_data = ser.readline()
            sleep(0.03)
            data_left = ser.inWaiting() 
            received_data += ser.read(data_left)
            rec = received_data.decode("utf-8").strip()
            if received_data:
              print(rec)
            else:
              print('.', end='', flush=True)
        
            if 'getData' in rec:
                temperature_data = get_sensor_temperatures()
                print(temperature_data.encode())
                s_data = temperature_data.encode()
#                s_data = '111;222     \r\n'.encode()
                if ser.isOpen() :
                    wait_for_aux_pin()
                    ser.write(address)
                    wait_for_aux_pin()
                    ser.write(s_data)
                    print(s_data)
    except :
        if ser.isOpen() :
            ser.close()
            GPIO.cleanup()
    return

print(serial.__version__)

ser = gpio_init()

temperature_data = get_sensor_temperatures()
print(temperature_data.encode())
s_data = temperature_data.encode()
print(s_data)



wait_for_cmd(b'\x00\x0E\x0F')
