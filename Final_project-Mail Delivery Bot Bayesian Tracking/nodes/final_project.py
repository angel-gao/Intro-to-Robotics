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
    def __init__(self):
        self.colour_sub = rospy.Subscriber("mean_img_rgb", Float64MultiArray, self.colour_callback)

        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.camera_callback, queue_size=1)

        self.idx = 320
        
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = 11
        
        #self.colour_map = colour_map
        #self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.colour_codes = [
        # [150, 150, 150],  # 0 line
        [210, 160, 199], # 4 purple : green 
        [200, 160, 145], # 3 yellow #155->145 : orange
        [235,  85, 135], # 2 green : yello 
        [236, 145, 115], # 1 orange : blue
    ]
        
        self.colour_codes_HSV=[
        [colorsys.rgb_to_hsv(210/255.0, 160/255.0, 199/255.0)], # 4 purple
        [colorsys.rgb_to_hsv(221/255.0, 150/255.0, 120/255.0)], # 3 yellow
        [colorsys.rgb_to_hsv(235/255.0, 85/255.0, 135/255.0)], # 2 green
        [colorsys.rgb_to_hsv(236/255.0, 140/255.0, 115/255.0)], # 1 orange
    ]
        
        
        
        #additional stuff:
        self.cur_rgb = [0,0,0]  #measurement rgb valule
        self.colour = 'White'
        self.prev_colour = 'White'
        #prior estimate
        self.predict = [0.0]*self.num_states
        #current (and posterior) estimate
        #we do not have posterior stored cuz update directly on current 
        self.current = [1.0/self.num_states]*self.num_states
        #control, always forward
        self.u = 1

        self.turn = 0
        
        
        #address
        self.address = 0
        #confidence level
        self.conf = 0.0
        #probabilty for one loop, a total of 11 states for 11 timesteps
        self.update = np.zeros((self.num_states,self.num_states))
        #target location
        self.goal = [4]
        
                
        

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
        self.cur_rgb = np.array(msg.data)  # [r, g, b]
        #print(self.cur_rgb)

            

    def color_obtain(self, color_map):

        #initialize color if non
        if self.cur_rgb is None:
            self.cur_rgb = [255,255,255]
    
        tolerance = 13

        blue_r_error = abs(self.cur_rgb[0]-color_map[0][0])
        blue_g_error = abs(self.cur_rgb[1]-color_map[0][1])
        blue_b_error = abs(self.cur_rgb[2]-color_map[0][2])

        yellow_r_error = abs(self.cur_rgb[0]-color_map[1][0])
        yellow_g_error = abs(self.cur_rgb[1]-color_map[1][1])
        yellow_b_error = abs(self.cur_rgb[2]-color_map[1][2])

        green_r_error = abs(self.cur_rgb[0]-color_map[2][0])
        green_g_error = abs(self.cur_rgb[1]-color_map[2][1])
        green_b_error = abs(self.cur_rgb[2]-color_map[2][2])

        orange_r_error = abs(self.cur_rgb[0]-color_map[3][0])
        orange_g_error = abs(self.cur_rgb[1]-color_map[3][1])
        orange_b_error = abs(self.cur_rgb[2]-color_map[3][2])
        
        if blue_r_error < tolerance and blue_g_error < tolerance and blue_b_error < tolerance:
            self.colour = 'Blue'

        elif yellow_r_error < tolerance and yellow_g_error < tolerance and yellow_b_error < tolerance:
            self.colour = 'Yellow'

        elif green_r_error < tolerance and green_g_error< tolerance and green_b_error < tolerance :
            self.colour = 'Green'

        elif orange_r_error< tolerance and orange_g_error < tolerance and orange_b_error < tolerance:
            self.colour = 'Orange'
        else:
            self.colour = 'White'
            
        return self.colour
    
    
    def color_obtain_HSV(self, color_map):

        #initialize color if non
        if self.cur_rgb is None:
            self.cur_rgb = [255,255,255]

        cur_hsv = colorsys.rgb_to_hsv(self.cur_rgb[0]/255.0, self.cur_rgb[1]/255.0, self.cur_rgb[2]/255.0)
        
        print("HSV:", cur_hsv)
        
        tolerance_h = 0.13  # Corresponding to 13 degrees
        tolerance_s = 0.1
        tolerance_v = 0.13

        # Compute errors for each color
        blue_h_error = min(abs(cur_hsv[0] - color_map[0][0]), 1 - abs(cur_hsv[0] - color_map[0][0]))  # Wrap-around
        blue_s_error = abs(cur_hsv[1] - color_map[0][1])
        blue_v_error = abs(cur_hsv[2] - color_map[0][2])

        yellow_h_error = min(abs(cur_hsv[0] - color_map[1][0]), 1 - abs(cur_hsv[0] - color_map[1][0]))
        yellow_s_error = abs(cur_hsv[1] - color_map[1][1])
        yellow_v_error = abs(cur_hsv[2] - color_map[1][2])

        green_h_error = min(abs(cur_hsv[0] - color_map[2][0]), 1 - abs(cur_hsv[0] - color_map[2][0]))
        green_s_error = abs(cur_hsv[1] - color_map[2][1])
        green_v_error = abs(cur_hsv[2] - color_map[2][2])

        orange_h_error = min(abs(cur_hsv[0] - color_map[3][0]), 1 - abs(cur_hsv[0] - color_map[3][0]))
        orange_s_error = abs(cur_hsv[1] - color_map[3][1])
        orange_v_error = abs(cur_hsv[2] - color_map[3][2])

        # Determine the color based on tolerances
        if blue_h_error < tolerance_h and blue_s_error < tolerance_s and blue_v_error < tolerance_v:
            self.colour = 'Blue'

        elif yellow_h_error < tolerance_h and yellow_s_error < tolerance_s and yellow_v_error < tolerance_v:
            self.colour = 'Yellow'

        elif green_h_error < tolerance_h and green_s_error < tolerance_s and green_v_error < tolerance_v:
            self.colour = 'Green'

        elif orange_h_error < tolerance_h and orange_s_error < tolerance_s and orange_v_error < tolerance_v:
            self.colour = 'Orange'

        else:
            self.colour = 'White'

        return self.colour

            
            
            
            
    def color_obtain_EUdist(self, color_map):

        if self.cur_rgb is None:
            self.cur_rgb = [255,255,255]
    
        
        distances = []
        color_names = ['Green', "Orange", "Yellow", "Blue"] #order based on color_map
        for ref_color in color_map:
            distance = self.euclidean_distance(rgb1=self.cur_rgb, rgb2=ref_color)
            distances.append(distance)
        
        
        # Find the index of the minimum distance, closest to which color? 
        min_distance = min(distances)
        min_index = distances.index(min_distance)
        #print(f"Min distances: {min_distance}; Color: {color_names[min_index]}")
        
        #should be adjustable
        tolerance = 22
        
        if min_distance < tolerance:
            self.colour = color_names[min_index]
        else:
            self.colour = 'White'

        return self.colour
        
    def euclidean_distance(self, rgb1, rgb2):
        return np.sqrt(sum((a - b) ** 2 for a, b in zip(rgb1, rgb2)))
    


    def pid_control(self, integral, derivative, lasterror):

        vel_msg = Twist()
        vel_msg.angular.z = 0
        go_straight_time = 15
        go_stright_speed = 0.1
        normal_speed = 0.04
        
        rate = rospy.Rate(10)

        if self.colour == 'Blue':
            
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

        elif self.colour == 'Yellow':
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

        elif self.colour == 'Green':
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
            


        elif self.colour == 'Orange':
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
    
    
    
    
    
    def pid_control_new(self, integral, derivative, lasterror):
  
        
        vel_msg = Twist()
        
        go_straight_time = 15
        go_stright_speed = 0.08
        normal_speed = 0.05 #0.04
        
        rate = rospy.Rate(10)

        if self.colour == 'Blue' or self.colour == 'Yellow' or self.colour == 'Green' or self.colour == 'Orange':
            
            integral = 0
            derivative = 0
            lasterror = 0
            
            print("Color:", self.colour, "Previous Color:", self.prev_colour)
            
            

            
            
            

            
            
            #===================================================================
            #===================================================================
            #flexible going straight time???
            

            
            for i in range(go_straight_time):
                vel_msg.linear.x = go_stright_speed
                vel_msg.angular.z = -self.turn

                

                rate.sleep()
                print("Detect colour:", self.colour, "going straight")
                self.cmd_pub.publish(vel_msg)


            #ENTER UPDATE
            if self.prev_colour == 'White' and self.colour != 'White':
                print("Updating Belief!!! with color:", self.colour)
                self.state_predict(self.u)
                self.address, self.conf = self.state_update(self.colour)
                
                print("Current Prob distribution:, ", self.current)
                print("Address:", self.address, "Confidence:", self.conf)
                
                rospy.sleep(4)
                
                if self.conf > 0.9 and self.address in self.goal:
                    print("Goal Reached!!!!!!!!!!!!!!!!!!!")
                    rospy.sleep(8)
                
            
            #if go straight time is not enough to cover all color 
            if BL.color_obtain_EUdist(self.colour_codes) != 'White':
                for i in range(10):
                    vel_msg.linear.x = go_stright_speed
                    vel_msg.angular.z = -self.turn
                    
                    

                    rate.sleep()
                    print("Detect colour:", self.colour, "going straight")
                    self.cmd_pub.publish(vel_msg)
            #NOTe: update only done AFTER FAST FORWARD! (issue? )
            #do the state and measurement update, be aware 
            #only do the update when previous state is WHITE!!
            
            #self.turn = vel_msg.angular.z
            
                    
                
                
                
            

           
            
        else:
            vel_msg.linear.x= normal_speed
            print("Color: White", "Previous Color:", self.prev_colour)  

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

            if error > 110:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =0.4
                #print("Error:", error)
            elif error < -110:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =-0.4
                #print("Error:", error)
            elif error > 70:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =0.1
            elif error < -70:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =-0.1
            '''
            elif error > 30:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =0.05
            elif error < -30:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =-0.05
            '''
            
           
            
            

            self.turn = vel_msg.angular.z

            self.cmd_pub.publish(vel_msg)

        #print("Previous turning point:", vel_msg.angular.z, "Truning point:", self.turn)

    def pid_control_new_new_while(self, integral, derivative, lasterror):
  
        
        vel_msg = Twist()
        
        go_straight_time = 10
        go_stright_speed = 0.08
        normal_speed = 0.05 #0.04
        
        rate = rospy.Rate(10)

        if (self.colour == 'Blue' or self.colour == 'Yellow' or self.colour == 'Green' or self.colour == 'Orange'):
            
            integral = 0
            derivative = 0
            lasterror = 0
            
            print("Color:", self.colour, "Previous Color:", self.prev_colour)
            
            

            
            cur_color = self.colour
            

            
            
            #===================================================================
            #===================================================================
            #flexible going straight time???
            
            while self.colour != 'White':
                vel_msg.linear.x = go_stright_speed
                vel_msg.linear.z = -self.turn

                rate.sleep()
                
                print("Detect colour:", self.colour, "going straight")
                
                self.cmd_pub.publish(vel_msg)
                
                self.prev_colour = cur_color
                #constantly obtain the color, dynamically exit fast forward when white
                cur_color = BL.color_obtain_EUdist(color_map=self.colour_codes)
                self.color = cur_color

            #ENTER UPDATE
            #if self.prev_colour == 'White' and self.colour != 'White':
            print("Updating Belief!!! with color:", self.colour)
            self.state_predict(self.u)
            self.address, self.conf = self.state_update(self.colour)
                
            print("Current Prob distribution:, ", self.current)
            print("Address:", self.address, "Confidence:", self.conf)
                
            rospy.sleep(4)
                

                
                
            if self.conf > 0.9 and self.address in self.goal:
                print("Goal Reached!!!!!!!!!!!!!!!!!!!")
                rospy.sleep(8)



            
            
                
            
        
                
            
            
            #NOTe: update only done AFTER FAST FORWARD! (issue? )
            #do the state and measurement update, be aware 
            #only do the update when previous state is WHITE!!
            
            #self.turn = vel_msg.angular.z
            
                    
                
                
                
            

           
            
        else:
            vel_msg.linear.x= normal_speed
            print("Color: White", "Previous Color:", self.prev_colour)  

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

            if error > 110:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =0.4
                #print("Error:", error)
            elif error < -110:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =-0.4
                #print("Error:", error)
            elif error > 70:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =0.1
            elif error < -70:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =-0.1
            '''
            elif error > 30:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =0.05
            elif error < -30:
                vel_msg.linear.x = 0.02
                vel_msg.angular.z =-0.05
            '''
            
           
            
            

            self.turn = vel_msg.angular.z

            self.cmd_pub.publish(vel_msg)

        #print("Previous turning point:", vel_msg.angular.z, "Truning point:", self.turn)
            
    
    def pid_control_new_various_straighttime(self, integral, derivative, lasterror):
  
        
        vel_msg = Twist()
        vel_msg.angular.z = 0
        go_straight_time = 15
        go_stright_speed = 0.1
        normal_speed = 0.04
        
        

        
        if self.colour == 'Blue' or self.colour == 'Yellow' or self.colour == 'Green' or self.colour == 'Orange':
            
            integral = 0
            derivative = 0
            lasterror = 0
            cur_color = self.colour
            
            print("Color:", self.colour, "Previous color: ", self.prev_colour)
            
            #===================================================================
            #===================================================================
            #flexible going straight time???
            while self.colour != 'White':
                vel_msg.linear.x = go_stright_speed
                rate.sleep()
                print("Detect colour:", self.colour, "going straight")
                self.cmd_pub.publish(vel_msg)
                
                self.prev_colour = cur_color
                #constantly obtain the color, dynamically exit fast forward when white
                cur_color = BL.color_obtain(color_map=self.colour_codes)
                self.color = cur_color
                
                
            
            rospy.sleep(2)
            print("Color:", self.colour, "Previous color: ", self.prev_colour)
            #NOTe: update only done AFTER FAST FORWARD! (issue? )
            #do the state and measurement update, be aware 
            #only do the update when previous state is WHITE!!
            
            #EXIT UPDATE
            if self.prev_colour != 'White' and self.colour == 'White':
                print("Updating Belief!!!")
                self.state_predict(self.u)
                self.address, self.conf = self.state_update(self.colour)
                print("Current Prob distribution:, ", self.current)
                print("Address:", self.address, "Confidence:", self.conf)
                
                rospy.sleep(4)
                
                if self.conf > 0.9 and self.address in self.goal:
                    print("Goal Reached!!!!!!!!!!!!!!!!!!!")
                    rospy.sleep(8)
                    
                
                
                
            

           
            
        else:
            vel_msg.linear.x= normal_speed
            print("Color: White")

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

            if error > 110:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =0.4
                #print("Error:", error)
            elif error < -110:
                vel_msg.linear.x = 0.04
                vel_msg.angular.z =-0.4
                #print("Error:", error)
            elif error > 60:
                vel_msg.linear.x = 0.015
                vel_msg.angular.z =0.1
            elif error < -60:
                vel_msg.linear.x = 0.015
                vel_msg.angular.z =-0.1
            elif error > 40:
                vel_msg.linear.x = 0.01
                vel_msg.angular.z =0.05
            elif error < -40:
                vel_msg.linear.x = 0.01
                vel_msg.angular.z =-0.05
            

                

            self.cmd_pub.publish(vel_msg)
            

    def pid_control_new_new_for(self, integral, derivative, lasterror):
  
        
        vel_msg = Twist()
        
        go_straight_time = 10
        go_stright_speed = 0.08
        normal_speed = 0.05 #0.04
        
        rate = rospy.Rate(10)

        
                
            
        
                
            
            
            #NOTe: update only done AFTER FAST FORWARD! (issue? )
            #do the state and measurement update, be aware 
            #only do the update when previous state is WHITE!!
            
            #self.turn = vel_msg.angular.z
            
                    
                
                
                
            

           
            
        
        vel_msg.linear.x= normal_speed
        print("Color: White", "Previous Color:", self.prev_colour)  

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

        
        if error > 110:
            vel_msg.linear.x = 0.04
            vel_msg.angular.z =0.45
            #print("Error:", error)
        elif error < -110:
            vel_msg.linear.x = 0.04
            vel_msg.angular.z =-0.45
            #print("Error:", error)
        elif error > 70:
            vel_msg.linear.x = 0.02
            vel_msg.angular.z =0.1
        elif error < -70:
            vel_msg.linear.x = 0.02
            vel_msg.angular.z =-0.1
       
        
        
        if (self.colour == 'Blue' or self.colour == 'Yellow' or self.colour == 'Green' or self.colour == 'Orange'):
            vel_msg.angular.z = -self.turn

            
            
            for i in range(go_straight_time):
                vel_msg.linear.x = go_stright_speed
                vel_msg.angular.z = -self.turn

                

                rate.sleep()
                print("Detect colour:", self.colour, "going straight")
                self.cmd_pub.publish(vel_msg)

            if BL.color_obtain_EUdist(self.colour_codes) != 'White':
                for i in range(5):
                    vel_msg.linear.x = go_stright_speed
                    vel_msg.angular.z = -self.turn
                    
                    

                    rate.sleep()
                    print("Detect colour:", self.colour, "going straight")
                    self.cmd_pub.publish(vel_msg)
            
            #EXIT UPDATE
            if self.prev_colour != 'White' and self.colour == 'White':
                print("Updating Belief!!! with color:", self.colour)
                self.state_predict(self.u)
                self.address, self.conf = self.state_update(self.colour)
                
                print("Current Prob distribution:, ", self.current)
                print("Address:", self.address, "Confidence:", self.conf)
                
                rospy.sleep(4)

                
                
                
                if self.conf > 0.9 and self.address in self.goal:
                    print("Goal Reached!!!!!!!!!!!!!!!!!!!")
                    rospy.sleep(8)

            
            


            
            #must makesure it's not in color anymore
            '''
            cur_color = self.colour
            while self.colour != 'White' or self.prev_colour != 'White':
                vel_msg.linear.x = go_stright_speed
                rate.sleep()
                print("Detect colour:", self.colour, "going straight")
                self.cmd_pub.publish(vel_msg)
                
                self.prev_colour = cur_color
                #constantly obtain the color, dynamically exit fast forward when white
                cur_color = BL.color_obtain(color_map=self.colour_codes)
                self.color = cur_color'''
        

        self.turn = vel_msg.angular.z

        self.cmd_pub.publish(vel_msg)

    def state_predict(self, u):
        #prior of all 11 locations based on state model and posterio at k-1
        self.predict=[0.0]*self.num_states

        #only range from 0 to 11 inclusive!!! (because of mod)
        for i in range(len(self.current)):
            if u == 1:
                self.predict[(i-1)%self.num_states] += self.current[i]*0.05
                self.predict[i%self.num_states] += self.current[i]*0.10
                self.predict[(i+1)%self.num_states]+=self.current[i]*0.85
            elif u == 0:
                self.predict[(i-1)%self.num_states] += self.current[i]*0.05
                self.predict[i%self.num_states] += self.current[i]*0.90
                self.predict[(i+1)%self.num_states]+=self.current[i]*0.05
            elif u == -1:
                self.predict[(i-1)%self.num_states] += self.current[i]*0.85
                self.predict[i%self.num_states] += self.current[i]*0.10
                self.predict[(i+1)%self.num_states]+=self.current[i]*0.05
            else:
                print('ERR: invalid input')
                #raise err
                
                
    def state_update(self, colour):
        # Nothing = 0, Blue = 1, Green = 2, Yellow = 3, Orange = 4
        # Office color address
        #b_add = [4,8,9]
        #g_add = [0,2,5]
        #y_add = [3,7,11]
        #o_add = [1,6,10]


        #note that loc 0 is same as loc 10 
        #2 3 4 ... 12 -> 0 1 2 ... 10 
        b_add = [2,6,10]
        g_add = [1,5,9]
        y_add = [0,8]
        o_add = [3,4,7]
        
        if colour == 'Blue': # Blue
            for i in b_add:
                self.current[i] = self.predict[i]*0.60
            for i in g_add:
                self.current[i] = self.predict[i]*0.20
            for i in y_add:
                self.current[i] = self.predict[i]*0.05
            for i in o_add:
                self.current[i] = self.predict[i]*0.05
        
        elif colour == 'Green': # Green
            for i in b_add:
                self.current[i] = self.predict[i]*0.20
            for i in g_add:
                self.current[i] = self.predict[i]*0.60
            for i in y_add:
                self.current[i] = self.predict[i]*0.05
            for i in o_add:
                self.current[i] = self.predict[i]*0.05
                
        elif colour == 'Yellow': # Yellow
            for i in b_add:
                self.current[i] = self.predict[i]*0.05
            for i in g_add:
                self.current[i] = self.predict[i]*0.05
            for i in y_add:
                self.current[i] = self.predict[i]*0.65
            for i in o_add:
                self.current[i] = self.predict[i]*0.20
        
        elif colour == 'Orange': #Orange
            for i in b_add:
                self.current[i] = self.predict[i]*0.05
            for i in g_add:
                self.current[i] = self.predict[i]*0.05
            for i in y_add:
                self.current[i] = self.predict[i]*0.15
            for i in o_add:
                self.current[i] = self.predict[i]*0.60
                
        elif colour == 'White': #Nothing
            for i in b_add:
                self.current[i] = self.predict[i]*0.10
            for i in g_add:
                self.current[i] = self.predict[i]*0.10
            for i in y_add:
                self.current[i] = self.predict[i]*0.10
            for i in o_add:
                self.current[i] = self.predict[i]*0.10
        else:
            print('ERR: invalid color code', colour)
            
        self.current = self.normalize(self.current) #Normalization
        best_confidence = max(self.current)
        best_location = self.current.index(best_confidence)
        return best_location, best_confidence
    
    
    def normalize(self, l):
        norm = 0
        for i in range(len(l)):
            norm += l[i]
        for i in range(len(l)):
            l[i] = l[i]/norm
        return l
    



if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: line(white), 1: orange, 2: green, 3: yellow, 4: purple

    # code for line
    # colour_map = [0, 1, 0, 2, 0, 3, 0, 4]

    


    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    
    
    color_codes_HSV = [
        [colorsys.rgb_to_hsv(210/255.0, 160/255.0, 199/255.0)], # 4 purple
        [colorsys.rgb_to_hsv(221/255.0, 160/255.0, 145/255.0)], # 3 yellow
        [colorsys.rgb_to_hsv(235/255.0, 85/255.0, 135/255.0)], # 2 green
        [colorsys.rgb_to_hsv(252/255.0, 154/255.0, 105/255.0)], # 1 orange
    ]

    # initial probability of being at a given office is uniform
   
    BL = BayesLoc()

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
    cur_color = 'White'
    
    

    while not rospy.is_shutdown():
        #update prevous color 
        BL.prev_colour = cur_color 
        
        cur_colour = BL.color_obtain_EUdist(color_map=BL.colour_codes)
        
        BL.pid_control_new( integral=integral, derivative=derivative, lasterror=lasterror)
        
        
        
        
        
        rate.sleep()
        

    rospy.loginfo("finished!")
    #rospy.loginfo(localizer.probability)