#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import math
import numpy as np


def get_scan():
    scan = rospy.wait_for_message("scan", LaserScan)

    calib_offset = 0.04

    # only use the scan where y >= 0 (i.e., to the left side of the robot)
    in_range = np.array(scan.ranges[0:180])
    angles = np.arange(0, 180) * np.pi / 180

    # computes the y-distance to each measurement
    y_range = in_range * np.sin(angles)

    # filter out measurements with ranges larger than 3m or y_range outside of 0.5-0.65m
    angles = angles[(y_range <= 0.65) & (y_range >= 0.5) & (in_range < 3)]
    median_angle = np.median(angles)

    # take the median of the sensed angles
    return median_angle + calib_offset


def main():
    rospy.init_node("get_scan_angle")
    scan_pub = rospy.Publisher("scan_angle", Float64, queue_size=1)

    while not rospy.is_shutdown():
        angle = get_scan()
        scan_pub.publish(angle)


if __name__ == "__main__":
    main()
