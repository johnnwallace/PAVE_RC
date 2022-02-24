# from machine import Pin, UART
import machine
import numpy as np

# controlString is a 8-element np array with 0/1 corresponding to unpressed/pressed for the 4 pad buttons and 4 circle buttons
# SOFT forward, backward, left, right
# HARD forward, backward, left, right
# Returns two element array (s,t) that is sent to motor to tell it how many degrees to turn and how much throttle to use
def readRemote(controlString):

    controlArray = np.array(
        [[0, 0, -10, 10, 0, 0, -30, 30], [10, -10, 0, 0, 30, -30, 0, 0]]
    )  # determines contributions to s, t based on each button

    command = np.matmul(controlArray, controlString)  # (s, t)

    return command

# autoCommand are values from autonomous algorithm assuming other people will decide whether this is coming from GPS or CV
# remoteInput it a 4 element 0/1 column vector corresponding to forward, backward, left, right buttons
# is auto is 0 if remote, 1 if autonomous, determined from button status on remote
def control(autoCommand, remoteInput, isAuto):

    if isAuto == 0:
        return readRemote(remoteInput)
    else:
        return autoCommand

def autonomousSteering(currentAngle, desiredAngle, thetaDot, thetaIntegral):
    return

controlString = [0,0,0,0,0,0,0,0,0] # 9 buttons from remote
autoCommand = (0,0) # (s, t) from autonomous algorithm
isAuto = 0 # 0 if remote control, 1 if auto
command = (0,0) # final (s, t) for output to boat

if controlString[-1] == 1:
    if isAuto == 0:
        isAuto = 1
    else:
        isAuto = 0
    
command = control(autoCommand, controlString[0:7], isAuto)