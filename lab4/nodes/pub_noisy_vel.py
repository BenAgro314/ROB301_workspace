#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math
import numpy as np


class NoiseMaker(object):
    def __init__(self, mu=0, std=1e-5):
        self.mu = mu
        self.std = std

        self.cmd_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel_noisy", Twist, queue_size=1)

    def cmd_callback(self, cmd_msg):
        # noisy twist is the same but with Gaussian noise on linear x
        twist = Twist()
        twist.linear.x = cmd_msg.linear.x + np.random.normal(
            loc=self.mu, scale=self.std
        )
        print(twist)
        self.cmd_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("noisyvel")

    mu_noise = -0.001
    std_noise = 0.001
    noise_maker = NoiseMaker(mu=mu_noise, std=std_noise)
    rospy.spin()
