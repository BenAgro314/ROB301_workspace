#!/usr/bin/env python
import rospy
import cv2 as cv
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32MultiArray
import numpy as np
import colorsys
from enum import Enum

class Color(Enum):
    blue = 0
    green = 1
    yellow = 2
    orange = 3
    red = 4
    nothing = 5 # over the line
    nan = -1

def send_cmd(pub, x_vel, z_ang_vel):
    #assert x_vel >=0.1 and x_vel <= 0.26, "Linear speed is out of bounds"
    #assert z_ang_vel < 1.82, "Angular velocity is out of bounds"
    twist = Twist()
    twist.linear.x = x_vel
    twist.angular.z = z_ang_vel
    pub.publish(twist)

class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", UInt32MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.curr_color = None  # most recent measured colour

        # PID stuff:
        self.curr_px = None
        self.des_px = 320
        self.x_vel =0.08 #0.05 # max 0.1
        self.max_theta_vel = 0.5

        # for 0.1 lin vel:
        self.kp = 0.007
        self.kd = 0.003
        self.ki = 0.00015

        self.integral = 0
        self.prev_error = 0

        #elf.color_ranges = {
        #   Color.nothing: [(0.694, 0.018, 0.56), (0.967, 0.04, 0.654)],
        #   Color.yellow: [(0.176, 0.329, 0.611), (0.183, 0.35, 0.628)]
        #}

        self.color_ranges = {
            Color.red: np.array([160, 143, 155]),  # red
            Color.green: np.array([163, 184, 98]),  # green
            Color.blue: np.array([173, 166, 171]),  # blue
            Color.yellow: np.array([167, 170, 117]),  # yellow
            Color.nothing: np.array([150, 150, 150]),  # line
        }

        self.saved_colors = []

    def get_color(self):
        if self.curr_color is None:
            return Color.nan

        mean = np.mean(self.curr_color)

        if np.all(self.curr_color > mean -5) and np.all(self.curr_color < mean +5):
            print("Nothing")
            print(self.curr_color)
            print(mean)
            return Color.nothing
        else:
            print("color")
            print(self.curr_color)
            print(mean)
            return Color.red

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        r,g,b = msg.data
        #print("RGB", r, g, b)
        #self.curr_color = colorsys.rgb_to_hsv(r/ 255.0, g/ 255.0, b/ 255.0)
        self.curr_color = np.array([r, g, b])
        #self.saved_colors.append(self.curr_color)
        #print("HSV", self.curr_color)

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        self.curr_px = int(msg.data)

    def follow_the_line(self):

        if self.curr_px is None:
            return

        color = self.get_color()

        if color != Color.nothing:
            send_cmd(self.cmd_pub, np.clip(self.x_vel, -0.26, 0.26), 0)
            return

        error = self.des_px - self.curr_px
        self.integral += error
        
        # prevent windup
        self.integral = np.clip(self.integral, -1000, 1000)
        if np.sign(error) != np.sign(self.prev_error):
            self.integral = 0

        derivative = error - self.prev_error

        correction = self.kp*error + self.ki*self.integral + self.kd*derivative
        send_cmd(self.cmd_pub, np.clip(self.x_vel, -0.26, 0.26), np.clip(correction, -1.82, 1.82))
        self.prev_error = error

    def follow_the_line2(self):
        if self.curr_px is None:
            return

        color = self.get_color()

        if color != Color.nothing:
            send_cmd(self.cmd_pub, np.clip(self.x_vel, -0.26, 0.26), 0)
            return

        self.kp = 0.009
        # kd = 0.003 takes 26.86
        # kd = 0.005 takes around the same time but it is less robust around corners
        self.kd = 0.003
        self.ki = 0.0
        error = self.des_px - self.curr_px
        self.integral += error
        
        # prevent windup
        self.integral = np.clip(self.integral, -1000, 1000)
        if np.sign(error) != np.sign(self.prev_error):
            self.integral = 0
        derivative = error - self.prev_error

        correction = self.kp*error + self.ki*self.integral + self.kd*derivative
        lin_vel = self.x_vel
        if abs(error) > 270: #300
            lin_vel = self.x_vel*((320.0-abs(error))/320.0)
        send_cmd(self.cmd_pub, np.clip(lin_vel, -0.26, 0.26), np.clip(correction, -1.82, 1.82))
        self.prev_error = error
        rate.sleep()

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        if self.curr_colour is None:
            self.wait_for_colour()

        prob = np.zeros(len(colourCodes))

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """

if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        [167, 146, 158],  # red
        [163, 184, 100],  # green
        [173, 166, 171],  # blue
        [167, 170, 117],  # yellow
        [150, 150, 150],  # line
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        localizer.follow_the_line()
        rate.sleep()

    h = [np.inf, -np.inf]
    s = [np.inf, -np.inf]
    v = [np.inf, -np.inf]
    for color in localizer.saved_colors:
        h[0] = min(color[0], h[0])
        h[1] = max(color[0], h[1])

        s[0] = min(color[1], s[0])
        s[1] = max(color[1], s[1])

        v[0] = min(color[2], v[0])
        v[1] = max(color[2], v[1])

    print("h", h)
    print("s", s)
    print("v", v)
        
    """
    def control(self):#, u_k):
        # take you from current position to END of next office (or start if going in reverse)
        # +1 : forward one office
        # 0: stay at current office
        # -1:  backwards one office
        if self.current_control is None or self.current_control == 0:
            send_cmd(self.cmd_pub, 0, 0) # stay still
            self.current_control = None
            return

        assert self.current_control == -1 or self.current_control == 1, "Invalid self.control"
        # replace this condition with "While on the line"
        on_line = decode_px(self.curr_px)
        if on_line: # only follow line if we are on the line
            x_vel, omega = self.pid_controller.follow_the_line(self.curr_px, direction = self.control)
            send_cmd(self.cmd_pub, x_vel, omega)
        else:
            # we are on a color
            # if we are going forwards, keep moving until we see line again
            # if we are going backwards, keep moving until we see line again
            if self.move_until_line(self.current_control):
                self.state_update() # take a measurment and update the state
                return
            self.current_control = None # stop the current control 
            return

    def move_until_line(self, u):
        assert u == -1 or u == 1, "Invalid u for move_until_line"
        on_line = decode_px(self.curr_px)
        if on_line:
            return False
        else:
            send_cmd(self.cmd_pub, self.PID.x_vel*u)
            return True

    def step(self):
        if self.current_control is None:
            if len(self.control_queue) > 0:
                self.current_control = control_queue.pop(0)
                self.state_predict(self.current_control)
        self.control()
    """

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
