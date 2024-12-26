#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np


class ScanAnglePublisher(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_cb)
        self.angle_pub = rospy.Publisher("scan_angle", Float64, queue_size=1)
        self.angle_offset = 0
        self.cangles = []
        self.calibrated = False


    def scan_cb(self, msg):
        # only use the scan where y >= 0 (i.e., to the left side of the robot)
        # print('entered')
        in_range = np.array(msg.ranges[0:180])
        angles = np.arange(0, 180) * np.pi / 180

        # computes the y-distance to each measurement
        y_range = in_range * np.sin(angles)

        # filter out measurements with ranges larger than 3m or y_range outside of 0.5-0.65m
        angles = angles[(y_range <= 0.65) & (y_range >= 0.5) & (in_range < 3)]

        # the median is NaN if there are no valid angles
        angle = np.median(angles) + self.angle_offset/180*np.pi
        # publish the message
        angle_msg = Float64()
        #initial angle to tower from origin is tan-1(0.6/1.8) = 18.4349488 degrees/0.321750554 rad
        angle_msg.data = angle
        # print(angle)
        if len(self.cangles) < 15:
            if angle != 'nan':
                self.cangles.append(angle)
        elif self.calibrated == False:
            print(self.cangles)
            self.angle_offset = np.average(self.cangles) - np.arctan2(1, 3)
            print(np.average(self.cangles), np.arctan2(1,3))
            print(self.angle_offset)
            self.calibrated = True
        
        # print(angle)
        self.angle_pub.publish(angle_msg)


def main():
    rospy.init_node("scan_angle_publisher")
    pub = ScanAnglePublisher()
    rospy.spin()



if __name__ == "__main__":
    main()
