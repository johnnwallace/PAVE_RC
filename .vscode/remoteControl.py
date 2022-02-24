# from machine import Pin, UART
import numpy as np

# controlString is a 4-element np array with 0/1 corresponding to unpressed/pressed for each of 4 arrow buttons
# First element = forward. Second element = backward. Third element = left. Fourth element = right
# Returns two element array (s,t) that is sent to motor to tell it how many degrees to turn and how much throttle to use
def readRemote(controlString):
    
    controlArray = np.array(
        [[0, 0, -10, 10], [20, -20, 0, 0]]
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
