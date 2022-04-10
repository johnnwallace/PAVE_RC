import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from steering import PIDController as PID

'''Assumptions made: kp, di, kd, dt, tf, setpoint, v, maxMotorAngle, minTurnRadius, external forces, motor angle stays constant'''

#Physical Constants
WATER_DENSITY = 997 #kg/m^3

class Boat:
    
    def __init__(self, x, y, theta, speed, angularSpeed, motor):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed
        self.angularSpeed = angularSpeed
        self.motor = motor

        #All estimates
        self.LENGTH = 3 #m
        self.WIDTH = 1 #m
        self.MASS = 1600 #kg
        self.DRAG_COEFFICIENT = 0.7 # Completely made up but the motor thrust is also made up to counter it

        motor.setThrust(self.calculateDrag(self.WIDTH * .3)) #Temporary measure to set a constant thrust to maintain the initial cruising speed

        #self.maxMotorAngle = maxMotorAngle #degrees
        #self.motorTurnRate = motorTurnRate #degrees/ sec PROB WRONG
        #self.minTurnRadius = minTurnRadius #meters (~20 ft)
    
    def getTheta(self):
        return self.theta
    def getX(self):
        return self.x
    def getY(self):
        return self.y    
    def getSpeed(self):
        return self.speed
    def getAngularSpeed(self):
        return self.angularSpeed
    def getMotor(self):
        return self.motor

    #Assume linear relationship between motorAngle and turnRadius / orientationChange
    def updateStates(self, dt):
        self.speed  += self.calculateForwardAcceleration()*dt
        self.angularSpeed += self.calculateRotationalAcceleration()*dt

        ds = self.speed * dt
        #maxOrientationChange = (360 * ds)/ (2 * np.pi * self.minTurnRadius)
        self.theta += self.angularSpeed*dt
        self.x += ds * np.cos(self.theta)
        self.y += ds * np.sin(self.theta)
    
    def calculateForwardAcceleration(self):
        thrust = self.motor.getLongitudinalThrust()
        drag = self.calculateDrag(self.WIDTH*.3)
        return (thrust-drag)/self.MASS
    
    def calculateRotationalAcceleration(self):
        motorMoment = self.motor.getLateralThrust() * self.LENGTH/2
        dragMoment = 2*(self.calculateDrag(self.LENGTH*.3) * self.LENGTH/2)
        momInertia = self.MASS*(self.LENGTH^2 + self.WIDTH^2)/12
        return (motorMoment-dragMoment)/momInertia

    def calculateDrag(self, area):
        drag = .5 * self.DRAG_COEFFICIENT * WATER_DENSITY * (area) * np.square((self.speed))
        return drag

    

class Motor:

    def __init__(self, motorAngle, thrust, TURN_RATE, MAX_MOTOR_ANGLE):
        self.motorAngle = motorAngle #Positive angle is to right(starboard), negative is to left (port)
        self.thrust = thrust
        self.TURN_RATE = TURN_RATE
        self.MAX_MOTOR_ANGLE = MAX_MOTOR_ANGLE  
    
    # Assumption: Motor Angle can only be changed at a constant rate
    def changeMotorAngle(self, targetAngle, dt):
        change = targetAngle - self.motorAngle
        if np.abs(change) > self.TURN_RATE * dt:
            np.sign(change) * self.TURN_RATE * dt
        
        if np.abs(self.motorAngle + change) < self.MAX_MOTOR_ANGLE:
            self.motorAngle += change
        else:
            self.motorAngle = np.sign(self.motorAngle)*self.MAX_MOTOR_ANGLE
    
    def setThrust(self, newThrust):
        self.thrust = newThrust
    
    def getMotorAngle(self):
        return self.motorAngle
    def getThrust(self):
        return self.thrust   
    def getLongitudinalThrust(self):
        return self.thrust * np.abs(np.cos(self.motorAngle))   
    # Positive lateral thrust is counterclockwise rotation
    def getLateralThrust(self):
        return self.thrust * (-np.sin(self.motorAngle))

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
ki = 2
kd = 2

#Initial Boat State Variables
init_x = 0 # meters
init_y = 0 # meters
init_theta = 0.0 # orientation of boat: degrees from east (positive is ccw)
init_speed = 3 #m/s (constant velocity for now until auto throttle is added)
init_angularSpeed = 0 #deg/s

init_motorAngle = 0 #degrees from longitudinal axis of boat (positive is ccw)
init_thrust = 0

#arrays for graph
thetaHistory = np.array(init_theta)
xHistory = np.array(init_x)
yHistory = np.array(init_y)
speedHistory = np.array(init_speed)
errorHistory = np.array(0)
motorAngleHistory = np.array(init_motorAngle)

motor = Motor(init_motorAngle, init_thrust, motorTurnRate, maxMotorAngle)
boat = Boat(init_x, init_y, init_theta, init_speed, init_angularSpeed, motor)

setpoint = 45           # Goal orientation
controller = PID(setpoint, kp, ki, kd, dt)

t += dt
while t < tf:
    controller.updateError(boat.getTheta())
    errorHistory = np.append(errorHistory, controller.getError())

    targetAngle = controller.evaluate()
    boat.getMotor().changeMotorAngle(targetAngle, dt)
    motorAngleHistory = np.append(motorAngleHistory, boat.getMotor().getMotorAngle())
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


#plt.axhline(setpoint,color='red',linestyle='dotted')            # Horizontal line on setpoint angle