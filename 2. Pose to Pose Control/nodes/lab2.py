#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from nav_msgs.msg import Odometry


def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def callback(odom_data):
    """TODO: complete the call back function for subscriber"""
    point = odom_data.pose.pose.position
    quart=odom_data.pose.pose.orientation
    theta = get_yaw_from_quaternion(quart)
    cur_pose=(point.x, point.y, theta)
    rospy.loginfo(cur_pose)
    pass

'''
def main1():
    rospy.init_node("motor_node")
    #receiver from odom 
    odom_subscriber=rospy.Subscriber('odom',Odometry,callback,queue_size=1)
    
    #publish to cmd_vel
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(10)
    
    
    counter1 = 0
    twist1 = Twist()
    twist1.linear.x = 0.1
    twist1.angular.z = 0
    while counter1 < 200:
        cmd_pub.publish(twist1)
        counter1+=1
        rate.sleep()

    
    counter2 = 0
    twist2 = Twist()
    twist2.linear.x = 0
    twist2.angular.z = 0.1
    while counter2 < 157:
        cmd_pub.publish(twist2)
        counter2+=1
        rate.sleep()

    
    counter3 = 0
    twist3 = Twist()
    twist3.linear.x = 0.1
    twist3.angular.z = 0
    while counter3 < 50:
        cmd_pub.publish(twist3)
        counter3+=1
        rate.sleep()

    counter4 = 0
    twist4 = Twist()
    twist4.linear.x = 0
    twist4.angular.z = 0.1
    while counter4 < 79:
        cmd_pub.publish(twist4)
        counter4+=1
        rate.sleep()
    '''

'''
def main():
    rospy.init_node("motor_node")
    #receiver from odom 
    odom_subscriber=rospy.Subscriber('odom',Odometry,callback,queue_size=1)
    
    #publish to cmd_vel
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(10)
    
    def one_loop():
        counter1 = 0
        twist1 = Twist()
        twist1.linear.x = 0.2
        twist1.angular.z = 0
        while counter1 < 50:
            cmd_pub.publish(twist1)
            counter1+=1
            rate.sleep()

        
        counter2 = 0
        twist2 = Twist()
        twist2.linear.x = 0
        twist2.angular.z = 0.2
        while counter2 < 80:
            cmd_pub.publish(twist2)
            counter2+=1
            rate.sleep()
    
    one_loop()
    one_loop()
    one_loop()

    counter3 = 0
    twist3 = Twist()
    twist3.linear.x = 0.2
    twist3.angular.z = 0
    while counter3< 50:
        cmd_pub.publish(twist3)
        counter3+=1
        rate.sleep()
    
    '''

'''
def main():
    rospy.init_node("motor_node")
    #receiver from odom 
    odom_subscriber=rospy.Subscriber('odom',Odometry,callback,queue_size=1)
    
    #publish to cmd_vel
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(10)
    
    def one_loop():
        counter1 = 0
        twist1 = Twist()
        twist1.linear.x = 0.2
        twist1.angular.z = 0
        while counter1 < 50:
            cmd_pub.publish(twist1)
            counter1+=1
            rate.sleep()

        
        counter2 = 0
        twist2 = Twist()
        twist2.linear.x = 0
        twist2.angular.z = 0.2
        while counter2 < 80:
            cmd_pub.publish(twist2)
            counter2+=1
            rate.sleep()
    
    one_loop()
    one_loop()
    one_loop()

    counter3 = 0
    twist3 = Twist()
    twist3.linear.x = 0.2
    twist3.angular.z = 0
    while counter3< 50:
        cmd_pub.publish(twist3)
        counter3+=1
        rate.sleep()
        '''
    
    
def main():
    rospy.init_node("motor_node")
    #receiver from odom 
    odom_subscriber=rospy.Subscriber('odom',Odometry,callback,queue_size=1)
    
    #publish to cmd_vel
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(10)
    
    '''
    counter1 = 0
    twist1 = Twist()
    twist1.linear.x = 0.2
        
    while counter1 < 98:
        cmd_pub.publish(twist1)
        counter1+=1
        rate.sleep()'''
    
    counter2 = 0
    twist2 = Twist()
    twist2.linear.x = 0.08
    twist2.angular.z = 0.4
        
    while counter2 < 59:
        cmd_pub.publish(twist2)
        counter2+=1
        rate.sleep()



    
'''

def main():
    rospy.init_node("motor_node")
    #receiver from odom 
    odom_subscriber=rospy.Subscriber('odom',Odometry,callback,queue_size=1)
    
    #publish to cmd_vel
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(10)
    
    counter1 = 0
    twist1 = Twist()
    twist1.linear.x = 0.2
    while counter1 < 90:
        cmd_pub.publish(twist1)
        counter1+=1
        rate.sleep()

    counter2 = 0
    twist2 = Twist()
    twist2.linear.x = 0.05847
    twist2.angular.z = 0.2
    while counter2 < 118:
        cmd_pub.publish(twist2)
        counter2+=1
        rate.sleep()'''

if __name__ == "__main__":
    main()
