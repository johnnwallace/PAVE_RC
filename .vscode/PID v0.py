import numpy as np

# Find the “error” of the boat, i.e. the angle between the current orientation and the desired orientation
def error (t):
    errorFunction[t] = desiredDirection - currentDirection
    return errorFunction

# Proportional: Calculate correction towards the reference at a rate proportional to the error
def proportional (t):                               # maybe should take into account speed of boat?
    kp = 1/20                                      # kp proportional rate of turn
    return kp * errorFunction[t]                   # e.g. has to turn 90 degrees turns at rate 4.5 degrees/sec

# Integral: Calculate correction towards the reference at a rate proportional to the integral 
# of the error w.r.t time (with initial time being the time at which the desired orientation 
# was changed probably). “Positive” and “negative” errors should cancel out
def integral (startTime):                          # was told to do this numerically, so this might not be right?
    integralOfError = 0
    ki = 1/10                                      # ki proportional rate of turn
    for i in range(startTime,t):
        integralOfError += errorFunction[i]
    return ki * integralOfError    

# support function for derivative (need arrays of time and error to find line of best fit)
def buildCurrentTimeAndErrorArrays (startTime):    
    currentTimeArray = np.array([])
    currentTimeArray[t - startTime] = t - startTime
    currentErrorArray = np.array([])
    currentErrorArray[t-startTime] = errorFunction[t]
    return currentTimeArray, currentErrorArray

# Derivative: Calculate correction towards the reference at a rate proportional to the rate of 
# change of the error w.r.t
def derivative (startTime):
    kd = 1/40                                      # kd proportional rate of turn
    currentTimeArray, currentErrorArray = buildCurrentTimeAndErrorArrays(startTime)
    slope, yIntercept = np.polyfit(currentTimeArray, currentErrorArray, 1)
    return kd * slope
    

# Average these three calculations to get the actual correction that the steering applies
def correction (p, i, d):
    return (p + i + d) / 3

def turn(startTime, t):
    error(t)
    p = proportional(t)
    i = integral(startTime)
    d = derivative(startTime)
    steering = correction(p, i, d)    
    return steering

# global variables
t = 0
startTime = 0
currentDirection = 0.0
desiredDirection = 0.0
newDesiredDirection = 0.0
errorFunction = np.array([])

while True:
    s = turn(startTime, t)                              # Steering updates every time t increments
    # something here saying if CV/GPS updates the desired direction, make newDesiredDirection = this
    # this should be some angle with respect to 0 (straight ahead) and angle increasing clockwise lol
    if newDesiredDirection != desiredDirection:         # Reset the startTime for integral and derivative if the new desired direction changes
        startTime = t                                   # Set startTime to current time to calculate integral and derivative
        desiredDirection = newDesiredDirection          # Set desired direction to the new direction
    t += 1                                              # Increment time by 1
    