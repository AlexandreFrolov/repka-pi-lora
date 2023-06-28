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

def wait_for_aux_pin():
    AUX = 18
    while not GPIO.input(AUX):
        sleep(0.04)

def print_e32_config (received_data):
    if sys.version_info[0] > 2:
      received_data_hex = received_data.hex();
    else:
      received_data_hex = received_data.encode('hex');

    received_long = int(received_data_hex, 16)
    print('Конфигурация модуля E32:\t\t\t0x' + str(format(received_long, '012x')))

    save_param = (received_long >> 40) & 0xFF
    save_param_str = str(format(save_param, '#02x'))
    if (save_param == 0xC0):
      print('Сохраняем параметры при выключении питания:\t' + save_param_str)
    elif (save_param == 0xC2):
      print('Не сохраняем параметры при выключении питания:\t' + save_param_str)
    else:
      print('Неверное значение для сохранения параметров:\t' + save_param_str)

    uart_pull_up_resistor_needed = (received_long & 0x40) >> 6
    print('Нужен подтягивающий резистор для UART:\t\t' + str(uart_pull_up_resistor_needed))

    energy_saving_timeout = (received_long & 0x38) >> 3
    print('Таймаут в режиме сохранения энергии:\t\t' + str(energy_saving_timeout))

    address = (received_long >> 24) & 0xFFFF
    print('Адрес:\t\t\t0x' + str(format(address, '04x')))

    uart_mode = ((received_long >> 16) & 0xC0) >> 6
    print('Режим UART:\t\t' + str(format(uart_mode, '#02x')))

    uart_speed = ((received_long >> 16) & 0x38) >> 3
    print('Скорость UART:\t\t' + str(format(uart_speed, '#02x')))

    air_data_rate = (received_long >> 16) & 0x7
    print('Скорость радиоканала:\t' + str(format(air_data_rate, '#02x')))

    channel = (received_long >> 16) & 0x1F
    print('Номер радиоканала:\t' + str(format(channel, '#02x')))

    fixed_mode = (received_long & 0x80) >> 7
    print('Режим Fixed:\t\t' + str(fixed_mode))

    fec = (received_long & 0x4) >> 2
    print('Включен режим FEC:\t' + str(fec))

    output_power = received_long & 0x3
    print('Выходная мощность:\t' + str(output_power))

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
    time.sleep(1)
    
    if board_type == 'Repka-Pi':
        ser = serial.Serial("/dev/ttyS0", 9600, timeout=1)
    elif board_type == 'Raspberry Pi':
        ser = serial.Serial("/dev/serial0", 9600, timeout=1)    
   
    ser.flushInput()
    return ser

def gpio_cleanup():
    GPIO.cleanup()

def wait_for_serial_data(serial):
    while serial.inWaiting() <= 0:
        sleep(0.4)

def e32_get_config():
    try :
     if ser.isOpen() :
      wait_for_aux_pin()
      ser.write(b'\xC1\xC1\xC1')
      sleep(1)
    except :
     if ser.isOpen() :
      ser.close()
      GPIO.cleanup()

    wait_for_aux_pin()
    if ser.inWaiting() > 0:
        wait_for_serial_data(ser)
        received_data = ser.read(6)
        sleep(1)
    return received_data

ser = gpio_init()
received_data = e32_get_config()
print_e32_config (received_data)
gpio_cleanup()