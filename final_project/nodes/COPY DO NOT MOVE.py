#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from std_msgs.msg import String, UInt32MultiArray
from std_msgs.msg import Float64MultiArray
import numpy as np
import colorsys


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
        print(msg)
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
            

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
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
        [208, 160,199], # 4 purple
        [192, 181, 169], # 3 yellow
        [160, 196, 171], # 2 green
        [252, 154, 90], # 1 orange
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)

    rospy.loginfo("Starting the main loop. Waiting for color data...")
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if localizer.cur_colour is not None:
            print("Current color:", localizer.cur_colour)
        rate.sleep()
        localizer.line_callback()
        
    rospy.spin()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)

