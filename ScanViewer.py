#!/usr/bin/python
#-------------------------------------------------------------------------------

import matplotlib
import matplotlib.pyplot as plt

import math

data = open('scan.txt', mode='r')
x = []
y = []


for line in data:
    # parse in each line
    point = line.strip("\n").strip(" ").strip("(").strip(")").split(",")

    # get the values and convert them
    angle = (float(point[0]) * math.pi)/180
    distance = float(point[1])/100

    x.append(distance*math.cos(angle))
    y.append(distance*math.sin(angle))

data.close()

fig, ax = plt.subplots()
plt.axis('equal')
ax.plot(x,y, '.', markersize=1)
plt.show()
