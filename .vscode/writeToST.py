# Figure out the what data/format (S,t) needs for GPS, Compass and RF data and how to send to (S,t) program 
# GPS, Compass, and RF data (to my understanding) are going to return a path for the boat. The angle of this path and the desired
# speed to approach it should be taken from these paths at every time t and sent too the steering and throttle code

# steering (PIDV0.py code) requires: 
# t = time
# currentDirection at every time
# newDesiredDirection given at startTime

# throttle requires:
# t = time 
# currentSpeed at every time
# newDesiredSpeed given at startTime

# would call steeringAndThrottle(t, currentDirection, newDesiredDirection, currentSpeed, newDesiredSpeed) which would then call the 
# separate steering and throttle PID programs at every time increment with newDesiredDirection and newDesiredSpeed staying the same until 
# a new one is desired
