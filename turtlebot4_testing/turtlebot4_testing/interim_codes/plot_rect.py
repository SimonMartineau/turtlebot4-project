#!/usr/bin/env python3

# Enter chmod +x plot_rect.py in terminal to make program executable from terminal
import numpy as np
import matplotlib.pyplot as plt
import time


def plot_points(theta):
    point_1 = center + 0.5*np.array([[length, width], [width, -length]])@np.array([[np.cos(theta)], [np.sin(theta)]])
    point_2 = center + 0.5*np.array([[length, -width], [-width, -length]])@np.array([[np.cos(theta)], [np.sin(theta)]])
    point_3 = center + 0.5*np.array([[-length, -width], [-width, length]])@np.array([[np.cos(theta)], [np.sin(theta)]])
    point_4 = center + 0.5*np.array([[-length, width], [width, length]])@np.array([[np.cos(theta)], [np.sin(theta)]])

    plt.plot(center[0], center[1], 'go')
    plt.plot(point_1[0], point_1[1], 'ro')
    plt.plot(point_2[0], point_2[1], 'ro')
    plt.plot(point_3[0], point_3[1], 'ro')
    plt.plot(point_4[0], point_4[1], 'ro')
    plt.show()

center = np.array([[1], [2]])
width = 2
length = 4

fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal')
plt.title('Rotating Rectangle')

while True:
    theta = time.time()
    plot_points(theta)



