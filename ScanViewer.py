#!/usr/bin/python
#-------------------------------------------------------------------------------

import matplotlib.pyplot as plt
import math

import rospy
from lidar_node.msg import LidarScanData

def main():
    Viewer = graph()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Lidar_Data", LidarScanData, Viewer.UpdateData)

    rospy.spin()


class graph():

    def __init__(self):
    self.ax = plt.subplots()
    self.ax.axis('equal')

    def UpdateData(Data):
            Angles = Data.Angles
            Distances = Data.Distances

            if len(Angles) == len(Distances):
                x = []
                y = []
                self.ax.clear()
                for i in range(0,len(Angles)):
                    angle = (float(Angles[i]) * math.pi)/180
                    distance = float(Distances[i])/100
                    x.append(distance*math.cos(angle))
                    y.append(distance*math.sin(angle))
                self.ax.plot(x,y, '.', markersize=1)
                plt.draw()
