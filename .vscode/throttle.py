import numpy as np
from steering import PIDController as PID

maxLateral = 20  # assuming maximum lateral acceleration of 20 m/s/s
maxLongitudinal = 30  # assuming maximum longitudinal acceleration of 20 m/s/s


class Throttle:
    def __init__(self, maxLat, maxLon, setPoint, Kp, Ki, Kd):
        self.maxLat = maxLat
        self.maxLon = maxLon

        pid = PID(setPoint, Kp, Ki, Kd)

    def getCircle(self, points):
        # find 2 perpendicular bisectors
        m1 = -1 / ((points[1, 1] - points[1, 0]) / (points[0, 1] - points[0, 0]))
        mp1 = np.mean(points[:, 0], points[:, 1])
        b1 = mp1[1, 0] - m1 * mp1[0, 0]

        m2 = -1 / ((points[1, 2] - points[1, 0]) / (points[0, 2] - points[0, 0]))
        mp2 = np.mean(points[:, 0], points[:, 1])
        b2 = mp2[1, 0] - m2 * mp2[0, 0]

        # find intersection --> this is the center of the circle
        xIntersect = (b2 - b1) / (m1 - m2)
        yIntersect = xIntersect * m1 + b1

        return (xIntersect, yIntersect)

    # def getCentripetal(self, path):


test = Throttle(10, 10, 10, 10, 10, 10)

points = np.array([(-1, 0, 1), (0, 1, 0)])

print(test.getCircle(points))
