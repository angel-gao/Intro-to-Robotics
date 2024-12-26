#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from std_msgs.msg import String, UInt32MultiArray
from std_msgs.msg import Float64MultiArray
import numpy as np
import colorsys
import time


class BayesLoc(object):
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber("mean_img_rgb", Float64MultiArray, self.colour_callback)

        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.camera_callback, queue_size=1)

        self.idx = 0
        
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.cur_colour = None  # most recent measured colour
        print(self.cur_colour)



        

    def camera_callback(self, msg):
        """Callback for line index."""

        # access the value using msg.data
        #print(msg)
        self.idx = msg.data
        
        return

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        print(self.cur_colour)

    def line_callback(self):



        rate = rospy.Rate(10)
        vel_msg = Twist()
        vel_msg.angular.z = 0
    
        tolerance = 25

        blue_r_error = abs(localizer.cur_colour[0]-208)
        blue_g_error = abs(localizer.cur_colour[1]-151)
        blue_b_error = abs(localizer.cur_colour[2]-189)

        yellow_r_error = abs(localizer.cur_colour[0]-194)
        yellow_g_error = abs(localizer.cur_colour[1]-175)
        yellow_b_error = abs(localizer.cur_colour[2]-149)

        green_r_error = abs(localizer.cur_colour[0]-163)
        green_g_error = abs(localizer.cur_colour[1]-196)
        green_b_error = abs(localizer.cur_colour[2]-177)

        orange_r_error = abs(localizer.cur_colour[0]-236)
        orange_g_error = abs(localizer.cur_colour[1]-126)
        orange_b_error = abs(localizer.cur_colour[2]-101)
        
        if blue_r_error < tolerance and blue_g_error < tolerance and blue_b_error < tolerance:
            vel_msg.linear.x = 0.1
            print("Color: Blue")
            self.cmd_pub.publish(vel_msg)

        elif yellow_r_error < tolerance and yellow_g_error < tolerance and yellow_b_error < tolerance:
            vel_msg.linear.x = 0.1
            print("Color: Yellow")
            self.cmd_pub.publish(vel_msg)

        elif green_r_error < tolerance and green_g_error< tolerance and green_b_error < tolerance :
            vel_msg.linear.x = 0.1
            print("Color: Green")
            self.cmd_pub.publish(vel_msg)

        elif orange_r_error< tolerance and orange_g_error < tolerance and orange_b_error < tolerance:
            vel_msg.linear.x = 0.1
            print("Color: Orange")
            self.cmd_pub.publish(vel_msg)

        else:
            print("Color: White")
            
            rate = rospy.Rate(10)
            vel_msg = Twist()
            vel_msg.angular.z = 0
            vel_msg.linear.x= 0.05
            
            integral = 0
            derivative = 0
            lasterror = 0

            # kp = 0.001
            # ki = 0
            # kd = 0.00002

            kp = 0.0005
            ki = 0
            kd = 0.00002

            # desired = (165+153+156)/3
            desired = 320

            # actual = np.mean(localizer.cur_colour)
            actual = self.idx
            error = desired - actual
            print("Actual:", actual)
            print("Desired:", desired)
            print("Error:", error)
            
            integral = integral + error 
            derivative = error - lasterror
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            vel_msg.angular.z = correction

            lasterror = error

            self.cmd_pub.publish(vel_msg)
            rate.sleep()
            

    def color_obtain(self, color_map):

        #initialize color if non
        if localizer.cur_colour is None:
            localizer.cur_colour = [255,255,255]


 
    
        tolerance = 13

        blue_r_error = abs(localizer.cur_colour[0]-color_map[0][0])
        blue_g_error = abs(localizer.cur_colour[1]-color_map[0][1])
        blue_b_error = abs(localizer.cur_colour[2]-color_map[0][2])

        yellow_r_error = abs(localizer.cur_colour[0]-color_map[1][0])
        yellow_g_error = abs(localizer.cur_colour[1]-color_map[1][1])
        yellow_b_error = abs(localizer.cur_colour[2]-color_map[1][2])

        green_r_error = abs(localizer.cur_colour[0]-color_map[2][0])
        green_g_error = abs(localizer.cur_colour[1]-color_map[2][1])
        green_b_error = abs(localizer.cur_colour[2]-color_map[2][2])

        orange_r_error = abs(localizer.cur_colour[0]-color_map[3][0])
        orange_g_error = abs(localizer.cur_colour[1]-color_map[3][1])
        orange_b_error = abs(localizer.cur_colour[2]-color_map[3][2])
        
        if blue_r_error < tolerance and blue_g_error < tolerance and blue_b_error < tolerance:
            return 'Blue'

        elif yellow_r_error < tolerance and yellow_g_error < tolerance and yellow_b_error < tolerance:
            return 'Yellow'

        elif green_r_error < tolerance and green_g_error< tolerance and green_b_error < tolerance :
            return 'Green'

        elif orange_r_error< tolerance and orange_g_error < tolerance and orange_b_error < tolerance:
            return 'Orange'
        else:
            return 'White'
            
    


    def pid_control(self, color_observe, integral, derivative, lasterror):


        
        
        vel_msg = Twist()
        vel_msg.angular.z = 0
        go_straight_time = 15
        go_stright_speed = 0.1
        normal_speed = 0.04

        if color_observe == 'Blue':
            
            integral = 0
            derivative = 0
            lasterror = 0
            print("Color: Blue")
            for i in range(go_straight_time):
                vel_msg.linear.x = go_stright_speed
                rate.sleep()
                print("going stright")
                self.cmd_pub.publish(vel_msg)

            
            #time.sleep(1)

        elif color_observe == 'Yellow':
            vel_msg.linear.x= normal_speed
            integral = 0
            derivative = 0
            lasterror = 0

            print("Color: Yellow")

            for i in range(go_straight_time):
                vel_msg.linear.x = go_stright_speed
                rate.sleep()
                print("going stright")
                self.cmd_pub.publish(vel_msg)

            
            #time.sleep(1)

        elif color_observe == 'Green':
            vel_msg.linear.x= normal_speed
            integral = 0
            derivative = 0
            lasterror = 0
            print("Color: Green")
            for i in range(go_straight_time):
                vel_msg.linear.x = go_stright_speed
                rate.sleep()
                print("going stright")
                self.cmd_pub.publish(vel_msg)

            
            
            #time.sleep(1)
            


        elif color_observe == 'Orange':
            vel_msg.linear.x= normal_speed
            integral = 0
            derivative = 0
            lasterror = 0
            
            print("Color: Orange")

            for i in range(go_straight_time):
                vel_msg.linear.x = go_stright_speed
                rate.sleep()
                print("going stright")
                self.cmd_pub.publish(vel_msg)
            
            #time.sleep(1)

           
            
        else:
            vel_msg.linear.x= normal_speed
            print("Color: White")

        
        
        

            # actual = np.mean(localizer.cur_colour)
            actual = self.idx
            error = desired - actual

            

            #print("Actual:", actual)
            #print("Desired:", desired)
            #print("Error:", error)
            
            integral = integral + error 
            derivative = error - lasterror
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            vel_msg.angular.z = correction

            lasterror = error

            if error > 120:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =0.4
                print("Error:", error)
            elif error < -120:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =-0.4
                print("Error:", error)
            elif error > 70:
                vel_msg.linear.x = 0.015
                vel_msg.angular.z =0.1
            elif error < -70:
                vel_msg.linear.x = 0.015
                vel_msg.angular.z =-0.1
            

                

            self.cmd_pub.publish(vel_msg)
        #print("Angular v:", vel_msg.angular.z)
    
        
            
            

        
    
    
            
            



            
    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        if self.cur_colour is None:
            self.wait_for_colour()

        prob = np.zeros(len(colourCodes))

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: line(white), 1: orange, 2: green, 3: yellow, 4: purple

    # code for line
    # colour_map = [0, 1, 0, 2, 0, 3, 0, 4]

    colour_map = [0, 1, 0, 2, 0, 3, 0, 4]


    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        # [150, 150, 150],  # 0 line
        [210, 160, 199], # 4 purple
        [221, 160, 160], # 3 yellow
        [235,  85, 135], # 2 green
        [252, 154, 105], # 1 orange
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)

    rospy.loginfo("Starting the main loop. Waiting for color data...")
    



    rate = rospy.Rate(10)
    
    integral = 0
    derivative = 0
    lasterror = 0

    kp = 0.0005
    ki = 0
    kd = 0.00004

    #kp = 0.0005
    #ki = 0
    #kd = 0.00002

    # desired = (165+153+156)/3
    desired = 320
    
    

    while not rospy.is_shutdown():
        #if localizer.cur_colour is not None:
        #    print("Current color:", localizer.cur_colour)
        observed_c = localizer.color_obtain(color_map=colour_codes)
        
        localizer.pid_control(color_observe= observed_c, integral=integral, derivative=derivative, lasterror=lasterror)
        rate.sleep()
        
    rospy.spin()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
