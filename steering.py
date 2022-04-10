import numpy as np


class PIDController:
    #MaxTurnSpeed is fastest turn speed in degrees/s, #dt is timestep in s
    def __init__(self, setPoint, Kp, Ki, Kd, dt):
        self.setPoint = setPoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integralError = self.lastError = 0
        self.dt = dt
    
    def updateError(self, currentState):  # update error and setpoint values
        self.error = self.setPoint - currentState  # get error
        self.integralError += self.error * self.dt  # get cumulative error
        self.derivativeError = (
            self.error - self.lastError
        ) / self.dt  # get derivative of error
        self.lastError = self.error  # save current error
    
    def getError(self):
        return self.error

    # maybe create a ramp to avoid integral windup
    def updateSetpoint(self, newSetPoint):
        self.setPoint = newSetPoint

    def evaluate(self):  # return command value
        return self.Kp * self.error + self.Ki * self.integralError + self.Kd * self.derivativeError
        

'''
# global variables
t = 0
dt = 0.05
maxTurnSpeed = 30
startTime = 0
currentDirection = 0.0
desiredDirection = 0.0
newDesiredDirection = 0.0

controller = PIDController(0.0, 1.0, 1.0, 1.0, maxTurnSpeed, dt)

while True:
    controller.updateError(currentDirection)
    controller.updateSetpoint(newDesiredDirection)

    s = controller.evaluate()  # Steering updates every time t increments
    # something here saying if CV/GPS updates the desired direction, make newDesiredDirection = this
    # this should be some angle with respect to 0 (straight ahead) and angle increasing clockwise lol

    t += dt  # Increment time by 1
    '''
