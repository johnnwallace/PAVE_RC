import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from math import atan2
import math
from scipy import spatial

def get_dist(a, b):
    return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

# gives angle between two points with 0 being north
def get_angle(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.atan2(dx, dy)

def connect_two(start, end, end_angle, velocity):

    max_angle_change = np.radians(velocity / 10)
    end_angle += np.pi # accounts for bug in code

    # Get second to last point
    lx = end[0]
    ly = end[1]

    dx = -velocity*np.sin(end_angle)
    dy = -velocity*np.cos(end_angle)

    # calculate second to last point (slpoint)
    slx = lx + dx
    sly = ly + dy
    slpoint = np.array([slx, sly])

    path = []
    path.append(end)
    path.append(slpoint)
    

    while (get_dist(path[-1], start) > 5):
        prev_angle = get_angle(path[-2], path[-1])    
        current_x = path[-1][0]
        current_y = path[-1][1]
        next_point = 0
        best_dist = math.inf
        best_angle = 0
        for potential_angle in np.arange(-2, 3):
            potential_angle = np.radians(potential_angle)
            dx = velocity*np.sin(prev_angle + potential_angle)
            dy = velocity*np.cos(prev_angle + potential_angle)
            next_x = current_x + dx
            next_y = current_y + dy
            new_point = np.array([next_x, next_y])
            new_dist = get_dist(new_point, start)
            if(new_dist < best_dist):
                best_dist = new_dist
                best_angle = potential_angle
                next_point = new_point
        path.append(next_point)
    
    start_angle = get_angle(path[-2], path[-1])
        
    return np.array(path), start_angle

def get_full_path(current_location, current_heading, max_velocity, waypoints):
    paths = []
    for i, waypoint in enumerate(waypoints):
        if(i == 0):
            temp_path, angle = connect_two(waypoint, current_location, current_heading, max_velocity)
        else:
            temp_path, angle = connect_two(waypoints[i], waypoints[i-1], current_heading, max_velocity)
        paths.append(temp_path)
        start_angle = angle
    return paths

def graph_paths(paths):
    starting_point = paths[0][0]
    
    figure(figsize=(8, 8), dpi=80)
    for path in paths:
        plt.plot(path[:, 0], path[:, 1], 'o')

    plt.plot(starting_point[0], starting_point[1], 'o', label="Start")
    plt.plot(paths[-1][-1][0], paths[-1][-1][1], 'o', label="End")
    plt.gca().set_aspect('equal')
    plt.legend()

def graph_allPaths(allPaths):
    starting_point = allPaths[0]
    figure(figsize=(8, 8), dpi=80)
    x = []
    y = []
    for i in allPaths:
        x.append(i[0])
        y.append(i[1])
    plt.plot(x, y, 'o')
    plt.plot(allPaths[-1][0],allPaths[-1][1],'o', label="End")
    plt.plot(starting_point[0], starting_point[1], 'o', label="Start")
    plt.gca().set_aspect('equal')
    plt.legend()
  


'''velocity = 5
starting_point = np.array([-500, 0])
start_angle = np.radians(30)
buoys = [np.array([1000, 2000]), np.array([-1000, 1000]), np.array([150, 10]), np.array([400, -300])]'''

def get_full_path(current_location, current_heading, max_velocity, waypoints):
    paths = []
    for i, waypoint in enumerate(waypoints):
        if(i == 0):
            temp_path, angle = connect_two(waypoint, current_location, current_heading, max_velocity)
        else:
            temp_path, angle = connect_two(waypoints[i], waypoints[i-1], current_heading, max_velocity)
        paths.append(temp_path)
        current_heading = angle
    return paths


'''paths = get_full_path(starting_point, start_angle, velocity, buoys)
graph_paths(paths)'''


#Insert Obstacle
'''def graph_paths(paths, point):
    starting_point = paths[0][0]
    
    _ = figure(figsize=(8, 8), dpi=80)
    for path in paths:
        plt.plot(path[:, 0], path[:, 1], 'o')

    plt.plot(starting_point[0], starting_point[1], 'o', label="Start")
    plt.plot(paths[-1][-1][0], paths[-1][-1][1], 'o', label="End")
    plt.plot(point[0], point[1], 'o', label="Obstacle")
    plt.gca().set_aspect('equal')
    _ = plt.legend()'''

'''
paths = get_full_path(starting_point, start_angle, velocity, buoys)
obstacle = paths[0][35]

path_with_obstacle = -1
for i, path in enumerate(paths):
    loc = np.where((path == obstacle).all(axis=1))[0]
    if(len(loc) > 0):
        path_with_obstacle = i

buoys.insert(path_with_obstacle, np.array([obstacle[0]+25, obstacle[1]-25]))

paths_obstacle = get_full_path(starting_point, start_angle, velocity, buoys)
graph_paths(paths_obstacle, obstacle)
plt.show()'''