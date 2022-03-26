import math
import numpy as np
from steering import PIDController as PID

# from steering import PIDController as PID

maxLateral = 20  # assuming maximum lateral acceleration of 20 m/s/s
maxLongitudinal = 30  # assuming maximum longitudinal acceleration of 20 m/s/s


class Circle:
    def perpBis(point1, point2):
        m1 = -1 / ((point2[1] - point1[1]) / (point2[0] - point1[0]))
        mp1 = (points[:, 0] + points[:, 1]) / 2
        b1 = mp1[1] - m1 * mp1[0]

        return m1, b1

    def getCircle(points):
        # find 2 perpendicular bisectors
        # check if y1 == y0 or y2 == y0 or x1 == x0 or x2 == x0
        if (
            np.array_equal(points[:, 0], points[:, 1])
            or np.array_equal(points[:, 0], points[:, 2])
            or np.array_equal(points[:, 1], points[:, 2])
        ):
            raise ValueError("Two or more points are the same")

        # find intersection of 2 perpendicular bisectors, considering cases when slope = 0 or is undefined
        if points[1, 0] == points[1, 1]:
            xIntersect = (points[0, 0] + points[0, 1]) / 2
        elif points[1, 0] == points[1, 2]:
            xIntersect = (points[0, 0] + points[0, 2]) / 2
        else:
            m1, b1 = Circle.perpBis(points[:, 0], points[:, 1])
            m2, b2 = Circle.perpBis(points[:, 0], points[:, 2])

            xIntersect = (b2 - b1) / (m1 - m2)

        if points[1, 0] == points[1, 1]:
            yIntersect = (points[1, 0] + points[1, 1]) / 2
        elif points[1, 0] == points[1, 2]:
            yIntersect = (points[1, 0] + points[1, 2]) / 2
        else:
            m1, b1 = Circle.perpBis(points[:, 0], points[:, 1])
            m2, b2 = Circle.perpBis(points[:, 0], points[:, 2])

            yIntersect = (b2 - b1) / (m1 - m2)

        radius = math.sqrt(
            (points[0, 2] - xIntersect) ** 2 + (points[1, 2] - yIntersect) ** 2
        )

        return (xIntersect, yIntersect, radius)

    def getCentripetal(points, velocity):
        radius = Circle.getCircle(points)[2]
        return velocity * velocity / radius


class Throttle:
    def __init__(self, maxLat, maxLon, points, lookAhead, Kp, Ki, Kd):
        self.maxLat = maxLat
        self.maxLon = maxLon
        self.points = points
        self.lookAheadDist = lookAhead
        self.current = 0

        self.controller = PID(0, Kp, Ki, Kd)

    def lookAhead(self):
        return Circle.getCentripetal()
    
points = np.array([(-5, 0, 5), (0, -5, 0)])
velocity = 10

print(Circle.getCentripetal(points, velocity))
