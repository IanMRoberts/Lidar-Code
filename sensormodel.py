#!/usr/bin/python
#-------------------------------------------------------------------------------


import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class SensorModel:
    def __init__(self):
        self.LogOddsOcc = 0.85
        self.LogOddsFree = -0.4

    def sign(self,value):
        if value == 0:
            sign = 0
        else:
            sign = int(value/abs(value))
        return sign

    def GetPath(self,StartPoint,EndPoint):
        # an implimentation of Bresenham's line algorithm
        pixels = [] # the final points
        # break out the points
        x_0 = StartPoint[0]
        y_0 = StartPoint[1]

        x_1 = EndPoint[0]
        y_1 = EndPoint[1]
        # calculate the deltas and the direction
        delta_x = abs(x_1 - x_0)
        delta_y = abs(y_1 - y_0)
        S1 = self.sign(x_1-x_0)
        S2 = self.sign(y_1-y_0)
        # decide what ocatant it is in and flip the values accordingly
        if delta_y > delta_x:
            Temp = delta_x
            delta_x = delta_y
            delta_y = Temp
            interchange = True
        else:
            interchange = False

        #  calculate error and correction terms
        E = 2*delta_y - delta_x
        A = 2*delta_y
        B = 2*delta_y - 2*delta_x

        # start x and y at the first point
        x = x_0
        y = y_0
        pixels.append((x,y))
        for i in range(0,delta_x):
            if E < 0:
                if interchange:
                    y += S2
                else:
                    x += S1
                E += A
            else:
                y += S2
                x += S1
                E += B
            pixels.append((x,y))

        return pixels

    def SensorFunction(self,GlobalSensorPos,GlobalSensorReading,Resolution):
        # GlobalSensorPos = the descrete position of the sensor on the grid map
        # GlobalSensorReading = the descrete position of the sensor reading on the grid map
        # Resolution = the resolution of the grid map

        Path = self.GetPath(GlobalSensorPos,GlobalSensorReading)
        Values = []
        for Point in Path:
            #distance = math.sqrt(((Point[0]-GlobalSensorPos[0])*Resolution)**2+((Point[1]-GlobalSensorPos[1])*Resolution)**2)
            Values.append([Point[0],Point[1],self.LogOddsFree])
        Values[-1][2] = self.LogOddsOcc

        return Values

class GridMap:
    # create the sensor model
    Sensor = SensorModel()

    def __init__(self):
        # Initialize the map
        self.MapData = np.zeros((1023,1023))
        # min and max log odds value a cell can have
        self.Max_value = 3.5
        self.Min_value = -2.0

        self.resolution = 10 # units of measure
        # the extents of the grid map
        self.Min_x = -(self.resolution*(self.MapData.shape[0]-1))/2
        self.Min_y = -(self.resolution*(self.MapData.shape[1]-1))/2
        self.Max_x = (self.resolution*(self.MapData.shape[0]-1))/2
        self.Max_y = (self.resolution*(self.MapData.shape[1]-1))/2

    def GetIndex(self,x,y):
        # calculate the x and y inndex of a given point
        Index_x = math.ceil((x-self.Min_x)/self.resolution) - 1
        Index_y = math.ceil((y-self.Min_y)/self.resolution) - 1
        # return a tuple with both indexes
        return (int(Index_x),int(Index_y))

    def GetCoordinate(self,Index_x,Index_y):
        # calculate the x and y position of a given index set
        x = self.resolution*Index_x + self.Min_x
        y = self.resolution*Index_y + self.Min_y
        # return a tuple with the x and y position
        return (float(x),float(y))

    def SetValue(self,Index,Value):
        Index_x = Index[0]
        Index_y = Index[1]

        if Index_x >= 0 and Index_x < self.MapData.shape[0]:
            if Index_y >= 0 and Index_y < self.MapData.shape[1]:
                Value += self.MapData[Index_x,Index_y]
                if Value > self.Max_value:
                    self.MapData[Index_x,Index_y] = self.Max_value
                elif Value < self.Min_value:
                    self.MapData[Index_x,Index_y]  = self.Min_value
                else:
                    self.MapData[Index_x,Index_y]  = Value

    def PlotImage(self):
        max, min = self.MapData.max(),self.MapData.min()
        imgArray = (self.MapData-min)/(max - min)
        plt.imshow(imgArray,interpolation='nearest',cmap='Greys')
        plt.show()

    def AddScan(self,GlobalSensorPosition,ScanData):
        SensorIndex = self.GetIndex(GlobalSensorPosition[0],GlobalSensorPosition[1])
        # ScanData = (distance,angle)
        for Point in ScanData:
            angle = (float(Point[1]) * math.pi)/180
            distance = float(Point[0])
            x = distance*math.cos(angle) + GlobalSensorPosition[0]
            y = distance*math.sin(angle) + GlobalSensorPosition[1]
            ReadingIndex = self.GetIndex(x,y)

            Path = self.Sensor.SensorFunction(SensorIndex,ReadingIndex,self.resolution)
            for pixel in Path:
                self.SetValue((pixel[0],pixel[1]),pixel[2])


import rospy
from lidar_node.msg import LidarScanData

def main():
    Viewer = graph()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Lidar_Data", LidarScanData, Viewer.UpdateData)

    rospy.spin()


class graph():
    def __init__(self):
        self.Map = GridMap()

    def UpdateData(self,Data):
            Angles = Data.Angles
            Distances = Data.Distances
            scan = []
            if len(Angles) == len(Distances):
                for i in range(0,len(Angles)):
                    scan.append([Distances[i],Angles[i]])
            self.Map.AddScan((0,0),scan)
            self.Map.PlotImage()

if __name__ == '__main__':
        main()
