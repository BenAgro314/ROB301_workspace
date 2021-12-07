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

coords = [0.6, 1.2, 2.4, 3.0]

class PIDControl():
    def __init__(self):
        self.to_visit = [0.6, 1.2, 2.4, 3.0]
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
        self.curr_px = None
        self.des_px = 320
        self.x_vel = 0.1
        self.max_theta_vel = 0.5

        # for 0.1 lin vel:
        self.kp = 0.5/320.0
        self.kd = 0.001
        self.ki = 0

        self.integral = 0
        self.prev_error = 0
        self.state = None
        self.state_sub = rospy.Subscriber('state', String, self.state_callback, queue_size=1)
        self.timer = 0
        self.prev_time = rospy.get_time()

    def state_callback(self, data):

        self.state = float(data.data)

    def camera_callback(self, data):
        '''
        complete the function
        '''
        self.curr_px = int(data.data)


    def follow_the_line(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            if self.curr_px is None or self.state is None:
                print("Waiting for publishers")
                rate.sleep()
                continue

            if not len(self.to_visit):
                send_cmd(self._cmd_pub, 0, 0)
                rate.sleep()
                continue

            if self.state > self.to_visit[0]:
                print("STOPPED")
                self.timer = 2
                send_cmd(self._cmd_pub, 0, 0)
                self.to_visit.pop(0)
                self.prev_time = rospy.get_time()
                rate.sleep()
                continue

            if self.timer > 0:
                print("STOPPED")
                send_cmd(self._cmd_pub, 0, 0)
                dt = rospy.get_time() - self.prev_time
                print("dt", dt)
                self.timer -= dt
                self.prev_time = rospy.get_time()
                rate.sleep()
                continue

            error = self.des_px - self.curr_px
            self.integral += error
            
            # prevent windup
            self.integral = np.clip(self.integral, -1000, 1000)
            if np.sign(error) != np.sign(self.prev_error):
                self.integral = 0
            derivative = error - self.prev_error

            #print("Error:", error)
            #print("Integral:", self.integral)
            #print("Derivative:", derivative)
            correction = self.kp*error + self.ki*self.integral + self.kd*derivative
            send_cmd(self._cmd_pub, np.clip(self.x_vel, -0.26, 0.26), np.clip(correction, -1.82, 1.82))
            self.prev_error = error
            self.prev_time = rospy.get_time()
            rate.sleep()

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4_controller')
    ctrl = PIDControl()
    try:
        while(1):
            key = getKey()
            ctrl.follow_the_line()
            if (key == '\x03'): #stop the robot when exit the program
                break
    except rospy.ROSInterruptException:
        print("comm failed")
