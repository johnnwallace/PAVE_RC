from machine import Pin
from machine import UART
import numpy as np

CRESP = Pin(32, Pin.IN)  # CRESP pin (FOR INTERRUPT)
BE = Pin(27, Pin.IN)  # BE pin (CAN BE READ THROUGH LSTATUS REGESTER IF NEEDED)
CMD = Pin(31, Pin.OUT)  # CMD pin
CTS = Pin(29, Pin.IN)  # CTS pin
txPin = Pin(21, Pin.OUT)  # TX pin
rxPin = Pin(22, Pin.IN)  # RX pin
MODE_IND = Pin(24, Pin.IN)  # MODE_IND pin
uart = UART(1, 9600, tx=txPin, rx=rxPin)  # initialize UART

# used to configure the HumPRO's settings
def configure():
    if MODE_IND.value == 0:
        CMD.value(0)


# used to transmit data to the HumPRO for RF transmission
def transmitData(data):
    if CTS.value == 0:
        CMD.value(1)
        uart.write(data)
        CMD.value(0)

    # hold until HumPRO buffer is empty, indicating all data has been transmitted
    while True:
        if BE.value == 1:
            # ADD EEXFLAG REGISTER SAMPLING HERE --> RETURN TRUE IF NO ERRORS, FALSE IF ERRORS
            return


# used to read data from the uart connection with the HumPRO
def readData():
    return


# attach interrupt to CRESP so that data is read when it goes high
CRESP.irq(trigger=Pin.IRQ_RISING, handler=readData)

transmitData(5)
