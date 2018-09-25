#!/usr/bin/python
#-------------------------------------------------------------------------------
from time import sleep

# Encoder Encoder
import serial

class Encoder:
    CountsPerRotation = 800.0

    def __init__(self):
        self.ser=serial.Serial("/dev/ttyACM0",9600)  #change ACM number as found from ls /dev/tty/ACM*
        self.ser.flushInput()
        sleep(2) # wait for serial to intalize
        self.ser.flushInput()

    def GetAngle(self):
        self.ser.flushInput()
        self.ser.write("r")
        count = float(self.ser.readline().strip())
        angle = (count/self.CountsPerRotation)*360
        #print(angle)
        return angle
    def ResetAngle(self):
        self.ser.write("w")

# Moter code
import piplates.MOTORplate as MOTOR #import the MOTORplate module

class Motor:
    Motor_Number = 0
    Direction = 'cw'
    DutyCycle = 0.0
    Acceleration = 2.5
    def __init__(self,MotorNumber):
        self.Motor_Number = MotorNumber
        MOTOR.dcCONFIG(0,self.Motor_Number,self.Direction,self.DutyCycle,self.Acceleration) # dcCONFIG(addr,motor,dir,speed,acceleration)
        MOTOR.dcSTART(0,self.Motor_Number) # dcSTART(addr,motor)

    def SetSpeed(self,Speed):
        self.DutyCycle = Speed
        MOTOR.dcSPEED(0,self.Motor_Number,self.DutyCycle)

    def SetAcceleration(self,Acceleration):
        self.Acceleration = Acceleration
        MOTOR.dcCONFIG(0,self.Motor_Number,self.Direction,self.DutyCycle,self.Acceleration)

    def SetDirection(self,Direction):
        if Direction == 1:
            self.Direction = 'ccw'
            self.Direction = 'cw'
    def Stop(self):
        MOTOR.dcSTOP(0,self.Motor_Number)

# Laser Code
import smbus
import statistics

class Laser:
    bus = smbus.SMBus(1)
    address = 0x62

    def GetDistance(self):
        self.bus.write_byte_data(self.address, 0x00,0x04)
        while self.bus.read_byte_data(self.address,0x01) % 2 == 1:
            pass
        high = self.bus.read_byte_data(self.address,0x0f)
        low = self.bus.read_byte_data(self.address,0x10)
        distance = (high << 8) + low
        return distance

# Lidar Code
class Lidar:
    GearRatio = 14.0/82
    Idler_Encoder = Encoder()
    Drive_Motor = Motor(1)
    LaserRangeFinder = Laser()

    def Start(self):
        self.Drive_Motor.SetSpeed(100.0)
    def Stop(self):
        self.Drive_Motor.Stop()

    def Scan(self):
        Scan = []
        StartAngle = self.Idler_Encoder.GetAngle()*self.GearRatio
        LastAngle = StartAngle
        CurrentAngle = StartAngle
        while abs(CurrentAngle - StartAngle) < 360:
            CurrentAngle = self.Idler_Encoder.GetAngle()*self.GearRatio
            if abs(CurrentAngle - LastAngle) >= 1:
                Distance = self.LaserRangeFinder.GetDistance() # Laser.GetDistance()
                Scan.append((round(CurrentAngle,1),Distance))
                LastAngle = CurrentAngle
        return Scan


import rospy
from lidar_node.msg import LidarScanData

import time
import math

def main():
    pub = rospy.Publisher('Lidar_Data', LidarScanData)
    rospy.init_node('Lidar_Publisher')

    MyLidar = Lidar()
    MyLidar.Start()
    sleep(2)
    while not rospy.is_shutdown():
        for x in range(0,10)
                scan = MyLidar.Scan()
                msg = LidarScanData()
                msg.header.frame_id = 'Lidar_Frame'
                currentTime = time.time()
                msg.header.stamp.secs = int(round(currentTime,0))
                msg.header.stamp.nsecs = int( (currentTime - math.floor(currentTime))*10**9 )
                angledata = []
                distancedata = []
                for point in scan:
                        angledata.append(point[0])
                        distancedata.append(point[1])
                msg.Angles = angledata
                msg.Distances = distancedata

                rospy.loginfo(msg)
                pub.publish(msg)


    MyLidar.Stop()

if __name__ == '__main__':
    main()#!/usr/bin/python
