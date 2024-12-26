#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    rospy.init_node("motor_node")
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    counter1 = 0
    twist1 = Twist()
    twist1.linear.x = 0.1
    twist1.angular.z = 0
    while counter1 < 200:
        cmd_pub.publish(twist1)
        counter1+=1


    
if __name__ == "__main__":
    main()
