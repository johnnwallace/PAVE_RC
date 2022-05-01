import numpy as np

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

        k = 1.25 #To be tuned
        changeAngularSpeed = -k * self.motorAngle*dt
        if np.abs(self.angularSpeed + changeAngularSpeed) < maxAngularSpeed:
            self.angularSpeed += changeAngularSpeed
        else:
            self.angularSpeed = np.sign(self.angularSpeed)*maxAngularSpeed

        dtheta = self.angularSpeed*dt
        self.theta += dtheta
        if (self.theta > 180):
            self.theta = self.theta - 360
        if (self.theta < -180):
            self.theta = self.theta + 360
        
        dx = ds * np.sin(self.theta)
        dy = ds * np.cos(self.theta)
        self.x += dx
        self.y += dy

    # Assumption: Motor Angle can only be changed at a constant rate
    def changeMotorAngle(self, targetAngle, dt):
        change = targetAngle - self.motorAngle
        if np.abs(change) > self.motorTurnRate * dt:
            change = np.sign(change)* self.motorTurnRate * dt
        
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