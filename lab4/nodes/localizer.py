#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64

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

class BaselineLocalization(object):
    
    def __init__(self, x_0):
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)
        self.prev_time = rospy.get_time()
        self.x = x_0
        self.u = 0
        self.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, self.cmd_callback)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    def run(self):
        time = rospy.get_time()
        dt = time - self.prev_time
        self.x = self.x + dt*self.u
        print(self.x)
        self.prev_time = time

        self.state_pub.publish(str(float(self.x)))

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, self.cmd_callback)
        self.scan_sub = rospy.Subscriber('scan_angle', Float64, self.scan_callback, queue_size=1)
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.N = 0 # control input noise
        self.P = P_0
        self.x = x_0

        self.u = 0 # initialize the cmd_vel input
        self.phi = self.measurement_update(self.x)

        self.state_pub = rospy.Publisher('state', String, queue_size = 1)

        self.prev_time = rospy.get_time()
        self.dt = 0

        self.A = 1
        self.D = lambda x: self.h/((self.d - x)**2 + self.h**2)

        # plots for TAs
        self.times = []
        self.xs = []
	self.phis = []
        self.covs = []

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = data.data#*math.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self, x , u):
        #rospy.loginfo("TODO: update state via the motion model, and update the covariance with the process noise")
        return x + self.dt*u

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self, x):
        #rospy.loginfo("TODO: update state when a new measurement has arrived using this function")
        return math.atan2(self.h,(self.d - x))

    def run(self):
        current_input = self.u
        print('u', self.u)
        current_measurement = self.phi
        curr_time = rospy.get_time()

        self.dt = curr_time - self.prev_time
        print('dt', self.dt)
        self.prev_time = curr_time

        self.x = self.predict(self.x , self.u) # state prediction, f(x,u)
        print('state prediction', self.x)
        self.P = self.A*self.P*self.A + self.Q 

	if not math.isnan(self.phi):
             # note: these are all scalars so we don't need to transpose
             D = self.D(self.x) #D_{k+1}
             # a priori state covariance, the B term is here if we want to include control input noise of covariance B (currently 0)
             S = D*self.P*D + self.R # a priori measurment covariance
             W = self.P*D*(1.0/S) # kalman gain
             self.P = self.P - W*D*self.P#W*S*W  # a posteriori state covariance

	     self.phis.append(self.phi)
             hat_phi = self.measurement_update(self.x) # measurment prediction, h(x)
             print('measurment prediction', hat_phi)
             s = self.phi - hat_phi # measurement residual
             print('s', s)
             self.x = self.x + W*s # state update 
             print('x', self.x)
	else:
	     self.phis.append(0)
        
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
        plt.savefig('kalman_filter_xs.png', dpi = 300)
        plt.close()


        plt.plot(self.times, self.covs, label = 'A posteriori covariance estimation')
        plt.xlabel('Time (s)')
        plt.ylabel('A Posteriori Position Covariance $P_{k|k}$ ($m^2$)')
        plt.savefig('kalman_filter_covs.png', dpi = 300)
        plt.close()

        plt.plot(self.times, self.phis, label = 'Measurment')
        plt.xlabel('Time (s)')
        plt.ylabel('Phis')
        plt.savefig('phis.png', dpi = 300)
        plt.close()
	


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    #try:
    h = 0.6 #y distance to tower
    d = 0.6*3 #x distance to tower (from origin)  
    
    x_0 = 0 #initial state position
    
    Q = 0.001 #TODO: Set process noise covariance (this is from pub_noisy_vel.py)
    R = 0.0005 #np.radians(5) #TODO: measurement noise covariance (from: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/, 1 degree resolution)
    P_0 = 0.05 #m 

    #localizer = BaselineLocalization(x_0)
    localizer = KalmanFilter(h, d, x_0, Q, R, P_0)

    rospy.sleep(1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        localizer.run()  
        rate.sleep()

    localizer.plot()
            
    #except:
        #print(e)

    #finally:
        #rospy.loginfo("goodbye")
