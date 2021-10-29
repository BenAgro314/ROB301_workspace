#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib
#matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.N = 0 # control input noise
        self.P = P_0
        self.x = x_0

        self.u = 0 # initialize the cmd_vel input
        self.phi = self.h(self.x)

        self.state_pub = rospy.Publisher('state', String, queue_size = 1)

        self.prev_time = rospy.get_time()
        self.dt = 0

        self.A = 1
        self.D = lambda x: self.h/((self.d - x)**2 + self.h**2)

        # plots for TAs
        self.times = []
        self.xs = []
        self.covs = []

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*math.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self, x , u):
        #rospy.loginfo("TODO: update state via the motion model, and update the covariance with the process noise")
        return x + self.dt*u

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self, x):
        #rospy.loginfo("TODO: update state when a new measurement has arrived using this function")
        return math.atan(self.h/(self.d - x))

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi
        curr_time = rospy.get_time()

        self.dt = curr_time - self.prev_time
        self.prev_time = curr_time

        self.x = self.predict(self.x , self.u) # state prediction, f(x,u)
        hat_phi = self.measurement_update(self.x) # measurment prediction, h(x)
        s = self.phi - hat_phi # measurement residual

        # note: these are all scalars so we don't need to transpose
        D = self.D(self.x) #D_{k+1}
        # a priori state covariance, the B term is here if we want to include control input noise of covariance B (currently 0)
        self.P = self.A*self.P*self.A + self.dt*self.B*self.dt + self.Q 
        S = D*self.P*D + self.R # a priori measurment covariance
        W = self.P*D*(1.0/S) # kalman gain
        self.P = self.P - W*S*W  # a posteriori state covariance

        self.x = self.x + W*s # state update 
        
        #rospy.loginfo("TODO: complete this function to update the state with current_input and current_measurement")
        # saving for plotting
        self.xs.append(self.x)
        self.covs.append(self.P)
        self.times.append(curr_time)
        
        self.state_pub.publish(str(float(self.x)))

    def plot(self):
        # plot the positions self.xs and self.covs over self.times with a legend
        plt.plot(self.times, self.xs, label = 'A posteriori state estimation')
        plt.xlabel('Time (s)')
        plt.ylabel('A Posteriori Position Estimate $\hat{x}_{k|k}$ (m)')
        # save the plot to a png file
        plt.savefig('kalman_filter.png', dpi = 300)
        plt.close()


        plt.plot(self.times, self.covs, label = 'A posteriori covariance estimation')
        plt.xlabel('Time (s)')
        plt.ylabel('A Posteriori Position Covariance $P_{k|k}$ ($m^2$)')


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 0.61 #y distance to tower
        d = 0.61*3 #x distance to tower (from origin)  
        
        x_0 = 0 #initial state position
        
        Q = 0.001**2 #TODO: Set process noise covariance (this is from pub_noisy_vel.py)
        R = np.radians(1) #TODO: measurement noise covariance (from: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/, 1 degree resolution)
        P_0 = 1 #TODO: Set initial state covariance 
        kf = KalmanFilter(h, d, x_0, Q, R, P_0)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        rospy.sleep(1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            kf.run_kf()  
            rate.sleep()
            
    except:
        print(e)

    finally:
        rospy.loginfo("goodbye")
