# from machine import Pin, UART
import machine
import numpy as np

# controlString is a 9-element np array:
# forward, backward, left, right, power, ..., ..., ..., ...
# Returns two element array (s,t) that is sent to motor to tell it how many degrees to turn and how much throttle to use
def readRemote(controlString):
    throttle = 10 + 20 * controlString[4]  # more powerful if power button is pressed
    steering = 10 + 20 * controlString[4]  # more powerful if power button is pressed
    command = [0, 0]

    if controlString[0] == 1:
        command[0] = throttle
    elif controlString[1] == 1:
        command[0] = -throttle

    if controlString[2] == 1:
        command[1] = steering
    elif controlString[2] == 2:
        command[1] = -steering

    return command


# autoCommand are values from autonomous algorithm assuming other people will decide whether this is coming from GPS or CV
# remoteInput it a 4 element 0/1 column vector corresponding to forward, backward, left, right buttons
# is auto is 0 if remote, 1 if autonomous, determined from button status on remote
def control(autoCommand, remoteInput, isAuto):

    if isAuto == 0:
        return readRemote(remoteInput)
    else:
        return autoCommand


def toggle(current):
    if current == 1:
        current = 0
    else:
        current = 1


controlString = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # 9 buttons from remote
autoCommand = (0, 0)  # (s, t) from autonomous algorithm
isAuto = 0  # 0 if remote control, 1 if auto
command = (0, 0)  # final (s, t) for output to boat

# toggle rc control if button is pressed
if controlString[-1] == 1:
    toggle(isAuto)

command = control(autoCommand, controlString[0:4], isAuto)
