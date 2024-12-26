#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import math
import numpy as np
from std_msgs.msg import UInt32
import matplotlib.pyplot as plt



class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        
        )
        self.line = 320
        self.rate = rospy.Rate(30)
        self.int_err = 0
        self.kp = 1/320
        self.ki = 0.0015/320
        self.kd = 2/320
        self.errors = [0]        
        self.n = 0
        self.count = 0


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
    def camera_callback(self, msg):
       self.line = msg.data

    def follow_the_line(self, type, off=False):
        msg = Twist()
        err = self.line-320
        msg.linear.x = 0.1
        self.count = self.count + 1
        if off == False:
            if type == 'bang':

                msg.linear.x = 0.15
                if err > 0:
                    msg.angular.z = -0.2
                elif err < 0:
                    msg.angular.z = 0.2
                else:
                    msg.angular.z = 0
            elif type == 'P':
                
                msg.angular.z = -err*self.kp
            elif type == 'PI':
                self.int_err = self.int_err+err

                msg.angular.z = -err*self.kp - self.int_err*self.ki
            elif type == 'PID':
                self.int_err = self.int_err+err
                d_err1 = err - self.errors[-1]

                # if d_err < 10:
                #     msg.linear.x = 0.35
                msg.angular.z = -err*self.kp - self.int_err*self.ki - self.kd*d_err1 
                if abs(msg.angular.z) > 1.5:
                    msg.linear.x = 0.05
            self.errors.append(err)
        else:
            msg.linear.x = 0

        self.x_vel = msg.linear.x
        self.cmd_pub.publish(msg)
        self.rate.sleep()
        
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
        locations = [0.61,1.22,2.44,3.05]
        dt = 0.1
        # if abs(self.x - locations[self.n])<0.02 and self.count < 120:

        #     stopped = True
        #     self.count = self.count + 1
        # else:
        #     if self.count > 120:
        #         self.n = self.n+1
        #     stopped = False
        #     self.count = 0
        # self.follow_the_line("PID", off = stopped)

        # self.x = self.x+1/30*current_input
        # print(self.x)
        # self.state_pub.publish(self.x)

        t = 0
        
        xvals = []
        xvals.append(self.x)
        x_k = xvals[-1]

        tvals = [0]
        covvals = [1]
        phi_meas = []
        phi_meas.append(self.phi)
        phi_k = np.arctan2(1,3)
        n =0
        counter = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if counter == 0:
                self.follow_the_line("PID")
                Ak = 1
                Bk = dt
                Dk = self.h/(self.h**2+(self.d - x_k)**2)
                x_k_pred = Ak*x_k + self.x_vel*Bk
                
                phi_k_pred = np.arctan2(self.h, self.d-x_k_pred)
                if abs(self.phi-phi_k) < 0.2:
                    phi_k = self.phi
                else:
                    phi_k = phi_k_pred
                phi_res = phi_k-phi_k_pred
                P_k_1 = Ak*self.P*Ak + self.Q
                covvals.append(P_k_1)
                S_k = Dk*P_k_1*Dk + self.R
                W_k = P_k_1*Dk*(1/S_k)
                self.P = P_k_1-W_k*S_k*W_k
                x_k = x_k_pred + W_k*phi_res
                t = t+dt
                tvals.append(t)
                xvals.append(x_k)
                phi_meas.append(phi_k)
                if abs(x_k+0.125 - locations[n]) < 0.1:
                    counter = 30
                    n = n+1
            elif n > 3:
        
                plt.plot(tvals,xvals)
                plt.plot(tvals,covvals)
                plt.show()
                plt.savefig('BIGCOUNTRY.PNG')
                print(xvals, tvals, phi_meas)
                break
            else:
                counter = counter-1
            rate.sleep()
                


if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    #initial angle to tower from origin is tan-1(0.6/1.8) = 18.4349488 degrees/0.321750554 rad

    x_0 = 0  # initial state position

    Q = 1  # TODO: Set process noise covariance
    R = 1  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    rospy.sleep(1)

    rate = rospy.Rate(30)

    kf.run_kf()
