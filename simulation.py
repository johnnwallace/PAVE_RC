import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from steering import PIDController as PID

'''Assumptions made: kp, di, kd, dt, tf, setpoint, v, maxMotorAngle, minTurnRadius, external forces, motor angle stays constant'''

orientation = 0 #degrees

# All in seconds
t = 0
tf = 10
dt = 0.01 

# Assumptions:
maxMotorAngle = 45 #degrees
minTurnRadius = 6 #meters (~20 ft)
v = 13.4 #m/s (~30mph)

kp = 0.5
ki = 0.5
kd = 0.5

#arrays for graph
orientations = np.array(orientation)

#Assume linear relationship between motorAngle and turnRadius / orientationChange
def orientationChange(v, dt, minTurnRadius, maxMotorAngle, motorAngle):
    ds = v * dt
    maxOrientationChange = (360 * ds)/ (2 * np.pi * minTurnRadius)
    orientationChange = (maxOrientationChange / maxMotorAngle) * motorAngle
    return orientationChange

setpoint = 45           # Goal orientation

controller = PID(setpoint, kp,ki,kd, maxMotorAngle, dt)

t += dt
while t < tf:
    controller.updateError(orientation)
    motorAngle = controller.evaluate()
    orientation += orientationChange(v, dt, minTurnRadius, maxMotorAngle, motorAngle)

    orientations = np.append(orientations,orientation)
    t += dt



# Plot data
plt.figure(figsize=(8,6))
time_fit = np.linspace(0,tf,num = int((tf+dt) / dt))    # Time axis
orientations = orientations.transpose()             # Turn orientations into a row vector
plt.plot(time_fit,orientations)
#plt.errorbar(omega,VRatio,fmt='o',xerr=dOmega,yerr=dVRatio)
plt.title('Steering PID Simulation', fontsize = 18)

plt.grid()
# Set x and y limits, ticks, and label
plt.xlabel('Time (s)', fontsize = 16)            # Label the x-axis, including units
plt.ylabel('Orientation (degrees)', fontsize = 16)          # Label the y-axis, including units
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.ylim(0,75)
plt.axhline(setpoint,color='red',linestyle='dotted')            # Horizontal line on setpoint angle
plt.show()





