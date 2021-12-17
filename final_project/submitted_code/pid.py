import numpy as np
import rospy
from geometry_msgs.msg import Twist

def send_cmd(pub, x_vel, z_ang_vel):
    twist = Twist()
    twist.linear.x = x_vel
    twist.angular.z = z_ang_vel
    pub.publish(twist)

class PID:

    def __init__(self, kp = 0.009, kd = 0.003, ki = 0.000, des_px = 320, x_vel = 0.07):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.des_px = des_px
        self.integral = 0
        self.prev_error = 0
        self.x_vel = x_vel

    def follow_the_line(self, curr_px, direction = 1):
        # input: current pixel measurment
        # output: tuple of (x_vel, omega) 
        assert direction == -1 or direction == 1, "Invalid direction for follow_the_line"

        if curr_px is None:
            return (0,0)

        error = self.des_px - curr_px
        self.integral += error

        derivative = error - self.prev_error
        
        # prevent windup
        self.integral = np.clip(self.integral, -1000, 1000)
        # reset integral error when crossing midpoint
        if np.sign(error) != np.sign(self.prev_error):
            self.integral = 0

        omega = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error

        return (direction*self.x_vel, direction*omega)