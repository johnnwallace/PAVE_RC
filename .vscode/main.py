from machine import Pin
from machine import UART
import numpy as np

CRESP = Pin(32, Pin.IN)
CMD = Pin(31, Pin.OUT)  # CMD pin
CTS = Pin(29, Pin.IN)  # CTS pin
txPin = Pin(21, Pin.OUT)  # tx pin
rxPin = Pin(22, Pin.IN)  # rx pin
MODE_IND = Pin(24, Pin.IN)

uart = UART(1, 9600, tx=txPin, rx=rxPin)  # initialize UART


def configure():
    if MODE_IND.value == 0:
        CMD.value(0)


def transmitArray(data):
    if CTS.value == 0:
        CMD.value(1)
        uart.write(data)
        CMD.value(0)

        return False
    else:
        return True
