import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from steering import PIDController as PID

'''Assumptions made: kp, di, kd, dt, tf, setpoint, v, maxMotorAngle, minTurnRadius, external forces, motor angle stays constant'''



class Boat:
    
    def __init__(self, x, y, theta, speed, angularSpeed, motorAngle, maxMotorAngle, motorTurnRate, minTurnRadius):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed
        self.angularSpeed = angularSpeed
        self.motorAngle = motorAngle

        self.maxMotorAngle = maxMotorAngle #degrees
        self.motorTurnRate = motorTurnRate #degrees/ sec PROB WRONG
        self.minTurnRadius = minTurnRadius #meters (~20 ft)
    
    def getTheta(self):
        return self.theta

    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def getSpeed(self):
        return self.speed

    def getMotorAngle(self):
        return self.motorAngle

    #Assume linear relationship between motorAngle and turnRadius / orientationChange
    def updateStates(self, dt):
        ds = self.speed * dt
        maxAngularSpeed = (360 * self.speed)/ (2 * np.pi * self.minTurnRadius)

        k = 0.75 #To be tuned
        changeAngularSpeed = -k * self.motorAngle*dt
        if np.abs(self.angularSpeed + changeAngularSpeed) < maxAngularSpeed:
            self.angularSpeed += changeAngularSpeed
        else:
            self.angularSpeed = np.sign(self.angularSpeed)*maxAngularSpeed

        dtheta = self.angularSpeed*dt
        dx = ds * np.cos(self.theta)
        dy = ds * np.sin(self.theta)

        self.x += dx
        self.y += dy
        self.theta += dtheta

    # Assumption: Motor Angle can only be changed at a constant rate
    def changeMotorAngle(self, targetAngle):
        change = targetAngle - self.motorAngle
        if np.abs(change) > motorTurnRate * dt:
            change = np.sign(change)* motorTurnRate * dt
        
        if np.abs(self.motorAngle + change) > self.maxMotorAngle:
            self.motorAngle = np.sign(self.motorAngle)*self.maxMotorAngle
        else:
            self.motorAngle += change

    #Instantaneous motor change
    def setMotorAngle(self, newMotorAngle):
        if np.abs(newMotorAngle) > self.maxMotorAngle:
            self.motorAngle = np.sign(newMotorAngle)*self.maxMotorAngle
        else:
            self.motorAngle = newMotorAngle

# All in seconds
t = 0
tf = 50
dt = 0.01

# Assumptions
maxMotorAngle = 45 #degrees
motorTurnRate = 45 #degrees/ sec PROB WRONG
minTurnRadius = 6 #meters (~20 ft)

#TO BE TUNED
kp = 2
ki = 1
kd = 2

#Initial Boat State Variables
init_x = 0 # meters
init_y = 0 # meters
init_theta = 0.0 # orientation of boat: degrees from east (positive is ccw)
init_speed = 3 #m/s (constant velocity for now until auto throttle is added)
init_angularSpeed = 0 #deg/s
init_motorAngle = 0 #degrees from longitudinal axis of boat (positive is ccw)

#arrays for graph
thetaHistory = np.array(init_theta)
xHistory = np.array(init_x)
yHistory = np.array(init_y)
speedHistory = np.array(init_speed)
errorHistory = np.array(0)
motorAngleHistory = np.array(init_motorAngle)

boat = Boat(init_x, init_y, init_theta, init_speed, init_angularSpeed, init_motorAngle, \
    maxMotorAngle, motorTurnRate, minTurnRadius)

setpoint = 45           # Goal orientation
controller = PID(setpoint, kp, ki, kd, dt)

t += dt
while t < tf:
    controller.updateError(boat.getTheta())
    errorHistory = np.append(errorHistory, controller.getError())

    targetAngle = controller.evaluate()
    boat.setMotorAngle(-targetAngle)
    motorAngleHistory = np.append(motorAngleHistory, boat.getMotorAngle())
    boat.updateStates(dt)

    thetaHistory = np.append(thetaHistory,boat.getTheta())
    xHistory = np.append(xHistory, boat.getX())
    yHistory = np.append(yHistory, boat.getY())
    speedHistory = np.append(speedHistory, boat.getSpeed())
    t += dt

time_fit = np.linspace(0,tf,num = int((tf+dt) / dt))    # Time axis
thetaHistory = thetaHistory.transpose()             # Turn thetaHistory into a row vector
fig, axs = plt.subplots(2,3)
axs[0,0].plot(time_fit,thetaHistory)
axs[0,0].set_title('Steering PID Simulation Orientation')
axs[0,0].axhline(setpoint,color='red',linestyle='dotted')  

axs[1,0].plot(xHistory,yHistory)
axs[1,0].set_title('Steering PID Simulation (x,y) Coords')

axs[0,1].plot(time_fit,xHistory)
axs[0,1].set_title('x vs t')

axs[1,1].plot(time_fit,yHistory)
axs[1,1].set_title('y vs t')

axs[0,2].plot(time_fit, motorAngleHistory)
axs[0,2].set_title('motor angle vs. t')

axs[1,2].plot(time_fit, errorHistory)
axs[1,2].set_title('error vs. t')

plt.show()