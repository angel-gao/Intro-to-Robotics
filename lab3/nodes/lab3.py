#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32


class Controller(object):
    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )
        self.idx = 0
    


    def camera_callback(self, msg):
        """Callback for line index."""

        # access the value using msg.data
        print(msg)
        self.idx = msg.data
        
        return


    def follow_the_line_bang(self):
        rate = rospy.Rate(10)
        vel_msg = Twist()

        vel_msg.linear.x= 0.2 #for straightline
        
        desired = 320 #target have pixel value 320
        while not rospy.is_shutdown():
            actual = self.idx
            error = desired - actual
            if error < 0:
                vel_msg.angular.z = -0.3
            elif error > 0:
                vel_msg.angular.z = 0.3
            else:
                vel_msg.angular.z = 0

            self.cmd_pub.publish(vel_msg)
            rate.sleep()
        

    def follow_the_line_proportional(self):
        rate = rospy.Rate(10)
        vel_msg = Twist()
        vel_msg.angular.z = 0
        vel_msg.linear.x= 0.12
        
        kp = 0.003
        desired = 320
       
        while not rospy.is_shutdown():
            actual = self.idx
            error = desired - actual
            correction = kp * error
            vel_msg.angular.z = correction


            self.cmd_pub.publish(vel_msg)
            rate.sleep()

    def follow_the_line_proportional_integral(self):
        rate = rospy.Rate(10)
        vel_msg = Twist()
        vel_msg.angular.z = 0
        vel_msg.linear.x= 0.05
        
        integral = 0

        kp = 0.00
        ki = 0.00001

        desired = 320
       
        while not rospy.is_shutdown():
            actual = self.idx
            error = desired - actual
            integral += error
            correction = (kp * error) + (ki * integral)
            vel_msg.angular.z = correction


            self.cmd_pub.publish(vel_msg)
            rate.sleep()


    def follow_the_line_proportional_integral_differential(self):
        rate = rospy.Rate(10)
        vel_msg = Twist()
        vel_msg.angular.z = 0
        vel_msg.linear.x= 0.05
        
        integral = 0
        derivative = 0
        lasterror = 0

        kp = 0.0005
        ki = 0
        kd = 0.00002

        #desired = 320
        desired = 320

        while not rospy.is_shutdown():
            actual = self.idx
            error = desired - actual
            
            integral = integral + error 
            derivative = error - lasterror
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            vel_msg.angular.z = correction

            lasterror = error

            self.cmd_pub.publish(vel_msg)
            rate.sleep()

            print(error)
    
    def exp2(self):
        rate = rospy.Rate(10)
        vel_msg = Twist()
        vel_msg.angular.z = 0
        vel_msg.linear.x= 0.1
        
        integral = 0
        derivative = 0
        lasterror = 0

        kp = 0.002
        ki = 0.00001
        kd = 0.0015

        #desired = 320
        desired = 320

        while not rospy.is_shutdown():
            vel_msg.linear.x= 0.1
            actual = self.idx
            error = desired - actual
            
            integral = integral + error 
            derivative = error - lasterror
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            

            lasterror = error

            if abs(error) >= 90:
                vel_msg.linear.x= 0
                correction = correction * 0.5
            #if abs(error) >= 220:
            #    
            
            vel_msg.angular.z = correction

            self.cmd_pub.publish(vel_msg)
            rate.sleep()

    
    #turn only when experience an error greater than smth, don't have x
    def prop_v(self):
        rate = rospy.Rate(30)
        vel_msg = Twist()
        vel_msg.angular.z = 0
        vel_msg.linear.x= 0.1
        
        integral = 0
        derivative = 0
        lasterror = 0

        kp = 0.002
        ki = 0.00001
        kd = 0.0018

        #error range of 100
        desired = 320

        while not rospy.is_shutdown():
            actual = self.idx
            error = desired - actual
            
            integral = integral + error 
            derivative = error - lasterror
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            vel_msg.angular.z = correction  * 1.0
            lasterror = error

            # velocity prop to error
            if abs(error) <= 80:
                vel_msg.linear.x= 0.15
                #vel_msg.angular.z = correction  * 0.4
                #print("L")
            elif abs(error) <= 150 and abs(error) > 80:
                vel_msg.linear.x= 0.1
                #vel_msg.angular.z = correction  * 0.7
                #print("M")
            elif abs(error) > 150 and abs(error) <= 200:
                vel_msg.linear.x= 0.1
                #vel_msg.angular.z = correction  * 1.4
                #print("m")
            elif abs(error) > 200:
                vel_msg.linear.x = 0.1
                #vel_msg.angular.z = correction  * 1.5
                #print("s")
             


            self.cmd_pub.publish(vel_msg)
            rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line_proportional_integral_differential()
