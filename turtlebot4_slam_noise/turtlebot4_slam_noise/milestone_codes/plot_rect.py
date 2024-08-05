#!/usr/bin/env python3

# Enter chmod +x plot_rect.py in terminal to make program executable from terminal
import numpy as np
import matplotlib.pyplot as plt
import time

def create_points(theta):
    obstacle_points_list = []
    point_1 = center + 0.5*np.array([[length, width], [width, -length]])@np.array([[np.cos(theta)], [np.sin(theta)]])
    point_2 = center + 0.5*np.array([[length, -width], [-width, -length]])@np.array([[np.cos(theta)], [np.sin(theta)]])
    point_3 = center + 0.5*np.array([[-length, -width], [-width, length]])@np.array([[np.cos(theta)], [np.sin(theta)]])
    point_4 = center + 0.5*np.array([[-length, width], [width, length]])@np.array([[np.cos(theta)], [np.sin(theta)]])

    obstacle_points_list.append(point_1)
    obstacle_points_list.append(point_2)
    obstacle_points_list.append(point_3)
    obstacle_points_list.append(point_4)
    return obstacle_points_list


def calc_dist_to_segment(point, p1, p2):
    A = point[0] - p1[0]
    B = point[1] - p1[1]
    C = p2[0] - p1[0]
    D = p2[1] - p1[1]

    dot = A*C + B*D
    len_sq = C**2 + D**2
    param = -1
    if len_sq != 0:  # in case of 0 length line
        param = dot/len_sq

    if param < 0:
        xx = p1[0]
        yy = p1[1]

    elif param > 1:
        xx = p2[0]
        yy = p2[1]

    else:
        xx = p1[0] + param * C
        yy = p1[1] + param * D

    dx = point[0] - xx
    dy = point[1] - yy

    return np.sqrt(dx**2 + dy**2)


def calc_point_to_line(point, p1, p2):
    return abs((p2[1] - p1[1])*point[0] - (p2[0] - p1[0])*point[1] + p2[0]*p1[1] - p2[1]*p1[0])/np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def dist_to_obs(obstacle_points_list):
    dist_to_zone_list = []
    distance_1 = calc_dist_to_segment(robot, obstacle_points_list[0], obstacle_points_list[1])
    dist_to_zone_list.append(distance_1)
    distance_2 = calc_dist_to_segment(robot, obstacle_points_list[1], obstacle_points_list[2])
    dist_to_zone_list.append(distance_2)
    distance_3 = calc_dist_to_segment(robot, obstacle_points_list[2], obstacle_points_list[3])
    dist_to_zone_list.append(distance_3)
    distance_4 = calc_dist_to_segment(robot, obstacle_points_list[3], obstacle_points_list[0])
    dist_to_zone_list.append(distance_4)
    return dist_to_zone_list


def angle_of_cloud():
    obstacle_angle_list = []
    for point in obstacle_points_list:
        angle = np.arctan2(point[1][0]-robot[1][0], point[0][0]-robot[0][0])
        obstacle_angle_list.append(angle * 180/np.pi)
    return obstacle_angle_list
    

def plot_points(obstacle_points_list):
    plt.plot(robot[0], robot[1], 'go')
    plt.plot(center[0], center[1], 'go')
    for point in obstacle_points_list:
        plt.plot(point[0], point[1], 'ro')
    plt.show()



robot = np.array([[-1], [-1.5]])
center = np.array([[1], [2]])
width = 2
length = 4
theta = 0.0

fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect('equal')
plt.title('Rectangle')


obstacle_points_list = create_points(theta)
obstacle_angle_list = angle_of_cloud()
obstacle_angle_range = (np.min(obstacle_angle_list), np.max(obstacle_angle_list))


# Printing results
for i in range(len(obstacle_points_list)):
    print(f"({obstacle_points_list[i][0][0]}, {obstacle_points_list[i][1][0]}), angle={obstacle_angle_list[i]}")
print(obstacle_angle_range)
if obstacle_angle_range[1] - obstacle_angle_range[0] > 180:
    print("robot in dust zone")

dist_to_zone_list = dist_to_obs(obstacle_points_list)
for dist in dist_to_zone_list:
    print(dist)

print(f"Shortest distance to the zone = {np.min(dist_to_zone_list)}")

plot_points(obstacle_points_list)




