#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import UInt32

import matplotlib.pyplot as plt
import math
import numpy as np

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

        self.u = 0  # initialize the cmd_vel input
        self.phi = 0  # initialize the measurement input

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)


        #for line following 
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )
        self.idx = 0




        #self approximated distance
        self.estimate_position = []
        self.covariance = []
        self.past_angle_measured = 0.32

    
    #for line following
    def camera_callback(self, msg):
        """Callback for line index."""
 
        # access the value using msg.data#print(msg)
        self.idx = msg.data
        
        return


    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        """
        TODO: update state via the motion model, and update the covariance with the process noise
        """
        
        return

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        """
        TODO: update state when a new measurement has arrived using this function
        """
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi

        """
        TODO: complete this function to update the state with current_input and current_measurement
        """

        
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

        reach1 = 0
        reach2 = 0
        reach3 = 0
        reach4 = 0

        #set initial angle:\
        self.phi = 0.32
        self.x = 0


        
        rate = rospy.Rate(10)

         
        while not rospy.is_shutdown():
            #bot moving
            actual = self.idx
            error = desired - actual
            
            integral = integral + error 
            derivative = error - lasterror
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            vel_msg.angular.z = correction * 0.5

            lasterror = error
            #update velocity
            self.cmd_pub.publish(vel_msg)

            
            
            
            #bot estimate its positino 
            
            #for no noise: x_new = self.x + self.u * 0.22
            x_new = self.x + self.u * 0.1
            #print("u_value:", self.u)
            #print("x_new:", x_new)
            tol = 0.15
            #self.x = x_new
            
            

            #get measurement prediction 
            phi_new_pred = math.atan2(self.h, (self.d - x_new))
            print("calculate angle: ", phi_new_pred)



            #measurement residual
            phi_error = self.phi - phi_new_pred

            print("current angle: ", self.phi)


            

            #if (not math.isnan(self.phi)) and (not math.isnan(phi_new_pred) and (abs(self.past_angle_measured - self.phi) < 0.2)):
            if (not math.isnan(self.phi)) and (not math.isnan(phi_new_pred)):
                #update covariance matrix (P and W)
                P_pred = self.P + self.Q
                matrix_D = self.h / ((self.d - x_new)**2 + self.h**2)

                matrix_S = matrix_D*P_pred*matrix_D + self.R
                W_new = P_pred*matrix_D*(1/matrix_S)
                self.P = P_pred - W_new*matrix_S*W_new
                
                
                #now have updated W, compute updated state estimate
                x_new_update = x_new + W_new*phi_error
                self.x = x_new_update

                self.past_angle_measured = self.phi
                print("See measurement!")
                print("original x:", x_new)
                print("update x:", x_new_update)


            else:
                print("Can't see!")
                P_pred = self.P + self.Q
                self.P = P_pred
                x_new_update = x_new
                self.x = x_new_update
                print("current position:", self.x)
                
            
            self.state_pub.publish(self.x)
            


            #store value into array for later plot
            self.estimate_position.append(self.x)
            self.covariance.append(self.P)

            if x_new_update > 0.62 and x_new_update < 0.62+tol and not reach1:
                reach1 = 1
                rospy.sleep(2)
            if x_new_update > 1.2 and x_new_update < 1.2+tol and not reach2:
                reach2 = 1
                rospy.sleep(2)
            if x_new_update > 2.4 and x_new_update < 2.4+tol and not reach3:
                reach3 = 1
                rospy.sleep(2)
            if x_new_update > 3.0 and x_new_update < 3.0+tol and not reach4:
                reach4=1
                rospy.sleep(2)

            rate.sleep()
            '''
            x_new = self.x + self.u * 2.2
            tol = 1
            self.x = x_new
            print(x_new)
            self.state_pub.publish(self.x)

            if x_new > 6.5 and x_new < 6.5+tol and not reach1:
                reach1 = 1
                rospy.sleep(2)
            if x_new > 12.7 and x_new < 12.7+tol and not reach2:
                reach2 = 1
                rospy.sleep(2)
            if x_new > 25.8 and x_new < 25.8+tol and not reach3:
                reach3 = 1
                rospy.sleep(2)
            if x_new > 32 and x_new < 32+tol and not reach4:
                reach4=1
                rospy.sleep(2)

            rate.sleep()'''

            
        

    
    def plot_data(self):
        plt.figure()
        plt.plot(self.covariance, marker='o')
        plt.title('Covariance Data')
        plt.xlabel('Index')
        plt.ylabel('Covariance Value')

        # Plot the data in self.a
        plt.figure()
        plt.plot(self.estimate_position, marker='x', color='orange')
        plt.title('estimated position Data')
        plt.xlabel('Index')
        plt.ylabel('estimated position Value')

        # Display both plots
        plt.show()


                


            



            

    
    def line_follow(self):
        rate = rospy.Rate(10)
        vel_msg = Twist()
        vel_msg.angular.z = 0
        vel_msg.linear.x= 0.15
        
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


if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 1  # TODO: Set process noise covariance
    R = 1  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    
    
    #kf.line_follow()

    
    
   
    kf.run_kf()
    kf.plot_data()
    
        