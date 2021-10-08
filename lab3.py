#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey(): #you can ignore this function. It's for stopping the robot when press 'Ctrl+C'
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


def send_cmd(pub, x_vel, z_ang_vel):
    #assert x_vel >=0.1 and x_vel <= 0.26, "Linear speed is out of bounds"
    #assert z_ang_vel < 1.82, "Angular velocity is out of bounds"
    twist = Twist()
    twist.linear.x = x_vel
    twist.angular.z = z_ang_vel
    pub.publish(twist)

class BangBangControl():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
	self.curr_px = None
	self.des_px = 320
	self.x_vel = 0.1
	self.max_theta_vel = 0.5

    def camera_callback(self, data):
        '''
        complete the function
        '''
	self.curr_px = int(data.data)


    def follow_the_line(self):
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	    if self.curr_px is None:
		rate.sleep()
		continue
	    error = self.des_px - self.curr_px
	    print("Error:", error)
	    if error > 0:
		send_cmd(self._cmd_pub, self.x_vel, self.max_theta_vel)
	    elif error < 0:
		send_cmd(self._cmd_pub, self.x_vel, -self.max_theta_vel)
	    else:
		send_cmd(self._cmd_pub, self.x_vel, 0)
	    rate.sleep()

class PIDControl():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
	self.curr_px = None
	self.des_px = 320
	self.x_vel = 0.1
	self.max_theta_vel = 0.5

	# for 0.1 lin vel:
	self.kp = 0.003#0.5/320.0
	self.kd = 0.001
	self.ki = 0.00015#self.kp/10

	self.integral = 0
	self.prev_error = 0

    def camera_callback(self, data):
        '''
        complete the function
        '''
	self.curr_px = int(data.data)


    def follow_the_line(self):
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	    if self.curr_px is None:
		rate.sleep()
		continue
	    error = self.des_px - self.curr_px
	    self.integral += error
	    
	    # prevent windup
	    self.integral = np.clip(self.integral, -1000, 1000)
	    if np.sign(error) != np.sign(self.prev_error):
		self.integral = 0
	    derivative = error - self.prev_error

	    print("Error:", error)
	    print("Integral:", self.integral)
	    print("Derivative:", derivative)
	    correction = self.kp*error + self.ki*self.integral + self.kd*derivative
	    send_cmd(self._cmd_pub, np.clip(self.x_vel, -0.26, 0.26), np.clip(correction, -1.82, 1.82))
	    self.prev_error = error
	    rate.sleep()

class RacerControl():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
	self.curr_px = None
	self.des_px = 320
	self.x_vel = 0.26
	self.max_theta_vel = 0.5

	# for 0.1 lin vel:
	#self.kp = 0.003#0.5/320.0
	#self.kd = 0.001
	#self.ki = 0.00015#self.kp/10

	# for x vel 0.2
	#self.kp = 0.006
	#self.kd = 0.002
	#self.ki = 0.0

	self.kp = 0.009
	self.kd = 0.003
	self.ki = 0.0

	self.integral = 0
	self.prev_error = 0

    def camera_callback(self, data):
        '''
        complete the function
        '''
	self.curr_px = int(data.data)


    def follow_the_line(self):
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	    if self.curr_px is None:
		rate.sleep()
		continue
	    error = self.des_px - self.curr_px
	    self.integral += error
	    
	    # prevent windup
	    self.integral = np.clip(self.integral, -1000, 1000)
	    if np.sign(error) != np.sign(self.prev_error):
		self.integral = 0
	    derivative = error - self.prev_error

	    print("Error:", error)
	    print("Integral:", self.integral)
	    print("Derivative:", derivative)
	    correction = self.kp*error + self.ki*self.integral + self.kd*derivative
	    lin_vel = self.x_vel
	    if abs(error) > 300:
		lin_vel = self.x_vel*((320.0-abs(error))/320.0)
	    send_cmd(self._cmd_pub, np.clip(lin_vel, -0.26, 0.26), np.clip(correction, -1.82, 1.82))
	    self.prev_error = error
	    rate.sleep()

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab3')
    clc = RacerControl
    ctrl = clc()
    try:
        while(1):
            key = getKey()
            ctrl.follow_the_line()
            if (key == '\x03'): #stop the robot when exit the program
                break
    except rospy.ROSInterruptException:
        print("comm failed")
