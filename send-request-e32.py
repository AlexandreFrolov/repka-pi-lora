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
                  b'\x00\x0D\x0F']


def gpio_init ():
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
        ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)
    elif board_type == 'Raspberry Pi':
        ser = serial.Serial("/dev/serial0", 9600, timeout=1)
    ser.flushInput()
    return ser

def wait_for_aux_pin():
    AUX = 18
    while not GPIO.input(AUX):
        sleep(0.04)

def wait_for_serial_data(serial):
    while serial.inWaiting() == 0:
        sleep(0.4)


def send_cmd(ser, address):
    try :
        if ser.isOpen() :
            wait_for_aux_pin()
            ser.write(address)
            wait_for_aux_pin()
            ser.write('getData \n'.encode())
    except :
        if ser.isOpen() :
            ser.close()
            GPIO.cleanup()

    print('Sended to :' + str(address))
    
    wait_for_aux_pin()
    received_data = ser.readline()
    sleep(0.05)
    wait_for_aux_pin()
    data_left = ser.inWaiting() 
    received_data += ser.read(data_left)

    rec = received_data.decode("utf-8").strip()
    print('Received: ' + received_data.decode('utf-8') + "\n")
    
    node_data = rec.split(';')
    return node_data

def format_node_data(node, node_data):
    NODE_1_ITEMS = ['Температура',
                  'Давление',
                  'Влажность',
                  'Точка росы']
    NODE_2_3_ITEMS = ['Температура CPU',
                  'Температура GPU']
    node_dict={}
    i=0
    for val in node_data:
        val = str(val)
        if(node == 1):
            node_dict[NODE_1_ITEMS[i]]= val
        else:
            node_dict[NODE_2_3_ITEMS[i]]= val
        i = i+1
    return node_dict

def get_nodes_data(ser):
    nodes_dict={}
    node_data0 = send_cmd(ser, b'\x00\x0B\x0F')
    node_data1 = send_cmd(ser, b'\x00\x0C\x0F')
    node_data2 = send_cmd(ser, b'\x00\x0D\x0F')

    nodes_dict[0] = format_node_data(0, node_data0)
    nodes_dict[1] = format_node_data(1, node_data1)
    nodes_dict[2] = format_node_data(2, node_data2)
    return nodes_dict

def save_nodes_data_to_file(nodes_dict):
    jsonString = json.dumps(nodes_dict, indent=2, ensure_ascii=False)
    print(jsonString)
    with open('hosts_data.json', 'w') as f:
        json.dump(nodes_dict, f, indent=2, ensure_ascii=False)

ser = gpio_init()
nodes_dict = get_nodes_data(ser)
save_nodes_data_to_file(nodes_dict)
