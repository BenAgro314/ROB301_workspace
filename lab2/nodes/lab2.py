#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dubin import *


LIN_SPEED = 0.2 # m/s
TURN_LIN_SPEED = 0.08 # m/s
ROT_SPEED = 1 #  s^-1
RADIUS = 0.25

def sine(x, derivative = False):
    # x == np.array([])
    if derivative:
        return 2*np.pi*np.cos(np.pi/0.25*x)
    else:
        return 0.50*np.sin(np.pi/0.25*x)



def follow_sine(pub):
    x = np.linspace(0, 25, 4)
    y = sine(x)
    x = np.array([x,y])
    print(x)
    for i in range(len(x[0])):
        print(x[0][i], x[1][i])

	current_point = np.array([x[0][i], x[1][i]])
        next_point =  np.array([x[0][i+1], x[1][i+1]])
	
	delta_vec = np.array([x[0][i+1]-x[0][i], x[1][i+1]-x[1][i]])
        theta_a = np.arctan2(sine(current_point[0], True), current_point[0])
        theta_b = np.arctan2(sine(next_point[0], True), next_point[0])

        theta_a = x_aw[-1]
	theta_b = x_bw[-1]
	x_aw = x_aw[:2].reshape(2,1)
	x_bw = x_bw[:2].reshape(2,1)
	C_aw = np.array([[np.cos(theta_a), np.sin(theta_a)], [-np.sin(theta_a), np.cos(theta_a)]])
	dist_vec_a = C_aw.dot((x_bw - x_aw))
	delta_theta = theta_b - theta_a
	x_goal = np.array([dist_vec_a[0], dist_vec_a[1], delta_theta])
	print(x_goal)
	task1(x_goal, pub)
    
    








def go_circle(radius, theta, pub, left = True):
    assert theta>0
    global TURN_LIN_SPEED
    #global ROT_SPEED
    twist = Twist()
    twist.linear.x = TURN_LIN_SPEED
    twist.angular.z = (-1+2*int(left))*(TURN_LIN_SPEED/radius)
    duration = rospy.Duration(theta*radius/TURN_LIN_SPEED)
    rate = rospy.Rate(10)
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown() and (rospy.get_rostime() - start_time < duration):
	pub.publish(twist)
	rate.sleep()
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

def go_straight(distance, pub):
    # send robot distance in its positive x direction
    global LIN_SPEED
    twist = Twist()
    twist.linear.x = LIN_SPEED
    duration = rospy.Duration(distance/LIN_SPEED)
    rate = rospy.Rate(10)
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown() and (rospy.get_rostime() - start_time < duration):
	pub.publish(twist)
	rate.sleep()
    twist = Twist()
    twist.linear.x = 0
    pub.publish(twist)


def turn_in_place(delta_theta, pub):
   # rotates CCW about robot's z axis by theta radians
    global ROT_SPEED
    twist = Twist()
    twist.angular.z = (delta_theta/abs(delta_theta))*ROT_SPEED
    duration = rospy.Duration(abs(delta_theta)/ROT_SPEED)
    rate = rospy.Rate(10)
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown() and (rospy.get_rostime() - start_time < duration):
	pub.publish(twist)
	rate.sleep()
    twist = Twist()
    twist.angular.z = 0
    pub.publish(twist)

def task1(x_goal,pub):
    # robot starts at [0,0,0]
    # x_goal: numpy array of [x_goal, y_goal, theta_goal]
    theta_1 = np.arctan2(x_goal[1], x_goal[0])
    print(theta_1)
    distance = np.linalg.norm(x_goal[:2])
    print(distance)
    turn_in_place(theta_1, pub)
    go_straight(distance, pub)
    turn_in_place(x_goal[2] - theta_1, pub)


def task2(x_list, pub):

    for x_aw, x_bw in zip(x_list[:-1], x_list[1:]):
	theta_a = x_aw[-1]
	theta_b = x_bw[-1]
	x_aw = x_aw[:2].reshape(2,1)
	x_bw = x_bw[:2].reshape(2,1)
	C_aw = np.array([[np.cos(theta_a), np.sin(theta_a)], [-np.sin(theta_a), np.cos(theta_a)]])
	dist_vec_a = C_aw.dot((x_bw - x_aw))
	delta_theta = theta_b - theta_a
	x_goal = np.array([dist_vec_a[0], dist_vec_a[1], delta_theta])
	print(x_goal)
	task1(x_goal, pub)

def task3(x_goal, pub):
    global RADIUS
    pt1 = np.zeros(2)
    v1 = np.array([1,0])
    pt2 = x_goal[:2]
    theta_goal = x_goal[-1]
    v2 = np.array([np.cos(theta_goal), np.sin(theta_goal)])
    dubins_path = dubins(pt1, v1, pt2, v2, RADIUS)
    left_start = dubins_path.word[0] == "L"
    go_circle(RADIUS, dubins_path.theta_start, pub, left = left_start)
    go_straight(np.linalg.norm(dubins_path.S[1] - dubins_path.S[0]), pub)
    left_end = dubins_path.word[-1] == "L"
    go_circle(RADIUS, dubins_path.theta_end, pub, left = left_end)

def publisher_node():
    '''
    TODO: complete the publisher function here
    '''
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size  = 1)
    time.sleep(1)
    #task1(np.array([2, 0.5, np.radians(135)]), cmd_pub)
    #x_list = [
    #    np.array([0,0,0]),
    #    np.array([1, 0, np.pi/2]),
    #    np.array([1, 1, np.pi]),
    #    np.array([0, 1, 3*np.pi/2]),
    #    np.array([0,0,0]),
    #]
    #task2(x_list, cmd_pub)
    #go_circle(RADIUS, np.pi/2, cmd_pub, left = False)  
    #task3(np.array([2, 0.5, np.radians(135)]), cmd_pub)
    follow_sine(cmd_pub)

def main():

    try:
        rospy.init_node('lab02')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
