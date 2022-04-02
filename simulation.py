import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from steering import PIDController as PID

orientation = 0
t = 0
tf = 20
dt = 0.01
maxTurnSpeed = 30
kp = 0.3
ki = 0.3
kd = 0.3

#arrays for graph
time = np.arrange(start=t, stop=tf, step=dt)
orientations = np.array(orientation)

setpoint = 45

controller = PID(setpoint, kp,ki,kd, maxTurnSpeed, dt)

while t<tf:
    controller.updateError(orientation)
    orientation += controller.evaluate(orientation)
    orientations.append(orientation)
    t += dt








