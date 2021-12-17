#!/usr/bin/env python
import os

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32MultiArray

from controls import Control
from pid import PID, send_cmd
from utils import kth_diag_indices
from vision import (BLUE, COLOR_CODES, GREEN , RED, YELLOW, p_rgb_given_color)

DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class BayesLoc:

    def __init__(self, p0_, color_codes_, color_map_):
        self.color_sub = rospy.Subscriber(
            "mean_img_rgb", UInt32MultiArray, self.color_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.curr_px = None

        self.color_codes = color_codes_
        self.color_map = color_map_

        self.current_control = None
        self.pid_controller = PID()

        self.n = len(p0_) # number of states
        self.x = p0_.reshape(self.n, 1) # 1x11: probability of being at office i+2, where i is the index
        
        # state_model[x_{k+1}][state update] = probability
        # e.g state_model[move forward (1)][u = 1] = 0.85
        self.state_update_probs = {
            -1: {-1: 0.85, 0: 0.05, 1: 0.01},
            0: {-1: 0.1, 0: 0.9, 1: 0.14},
            1: {-1: 0.05, 0: 0.05, 1:0.85}
        }

        self.to_visit = []

        self.control_queue = []
        self.saved_probs = []
        #self.saved_probs.append((self.x, "prediction", 0))
        self.curr_color = None

    def execute_control(self):
        if self.current_control is None:
            send_cmd(self.cmd_pub, 0, 0) # stay still
            return
        if self.current_control.is_alive():
            self.current_control.step(self)
    
    def go_to_rooms(self, threshold = 0.90):
        # e.g self.to_visit = [5, 9, 12]
        # threshold: minimum confidence level for stopping at a room

        if self.current_control is None: # we have not yet executed a contro
            if len(self.control_queue) > 0:
                self.current_control = self.control_queue.pop(0)
                print("ADDED CONTROL:" + str(self.current_control.u))
        elif not self.current_control.is_alive(): # we have finished executing the current control
            print("FINISHED CONTROL:" + str(self.current_control.u))
            self.state_predict(self.current_control.u) # predict where we are now
            self.state_update() # state updates
            if len(self.control_queue) > 0:
                self.current_control = self.control_queue.pop(0)
                print("ADDED CONTROL:" + str(self.current_control.u))
            else:
                self.current_control = None
        
        if len(self.to_visit) > 0:
            if self.current_control is None or not self.current_control.is_alive(): # we have finished executing last control, check our probability
                most_likely_room, max_prob= (np.argmax(self.x) + 2, max(self.x).item())
                print("Most likely room:", most_likely_room, "with probability", max_prob)
                print("Control queue", [str(s) for s in self.control_queue])
                if max_prob > threshold and most_likely_room in self.to_visit:
                    self.control_queue.append(Control(0)) # stop
                    self.to_visit.remove(most_likely_room)
                else:
                    self.control_queue.append(Control(1)) 
        self.execute_control()

    def color_callback(self, msg):
        r, g, b = msg.data
        self.curr_color = (r, g, b)

    def line_callback(self, msg):
        self.curr_px = int(msg.data)

    def state_model(self, u):
        """
        Return state update matrix
        """
        #[ p(x_{k+1} = 2| x_k = 2, u_k), p(x_{k+1} = 2 | x_k = 3, u_k), ....
        # p(x_{k+1} = 3| x_k = 2, u_k), p(x_{k+1} = 2 | x_k = 3, u_k), ....
        # ...
        # p(x_{k+1} = 12| x_k = 2, u_k), ... ]
        res = np.zeros((self.n, self.n))
        res[kth_diag_indices(res, 0)] = self.state_update_probs[0][u] # stay still
        res[kth_diag_indices(res, -1)] = self.state_update_probs[1][u] # move forwards
        res[kth_diag_indices(res, 1)] = self.state_update_probs[-1][u] # move backwards
        res[self.n-1][0] = self.state_update_probs[-1][u] # move backwards (wrap around)
        res[0][self.n-1] = self.state_update_probs[1][u] # move forwards (wrap around)
        return res

    def wait_for_color(self):
        """Loop until a color is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.curr_color is None:
            rate.sleep()

    def measurement_model(self):
        if self.curr_color is None:
            self.wait_for_color()

        prob = np.zeros((len(self.color_map), 1))
        for i in range(len(self.color_map)):
            prob[i][0] = p_rgb_given_color(self.curr_color, self.color_map[i])
            
        # we return an 11x1 numpy array where the ith row is
        # p(z_k = self.curr_color | at color of room i+1)
        # e.g if room 2 is yellow first entry is p(current rgb value| at a yellow room)
        return prob

    def state_predict(self, u):
        rospy.loginfo("Predicting state")
        self.x = self.state_model(u).dot(self.x)
        self.saved_probs.append((self.x, "prediction", u))

    def state_update(self):
        rospy.loginfo("Updating state")
        self.x = self.measurement_model()*self.x
        self.x = self.x/sum(self.x)
        self.saved_probs.append((self.x, "update", self.curr_color))

    def plot(self, name):
        fig, axs = plt.subplots(len(self.saved_probs), 1)
        for i, ax in enumerate(axs):
            probs = self.saved_probs[i][0]
            pred = self.saved_probs[i][1] == "prediction"
            ax.set_xticks([y for y in range(2, self.n+2)])

            plot_color = "black" if pred else tuple(
                [r/255.0 for r in self.saved_probs[i][2]]
            )
            ax.bar([y for y in range(2, self.n+2)], [float(p) for p in probs], color = plot_color)
            if pred: 
                title = ""#"Prediction: u=" + str(self.saved_probs[i][2])
            else:
                title = ""#"Update: color=" + str(COLOR_TO_NAME[self.saved_probs[i][2]])
            ax.set_title(title)
            ax.set_yticks([0] + list(max(probs)))

            for j, ticklabel in enumerate(ax.get_xticklabels()):
                color = tuple([r/255.0 for r in self.color_codes[color_map[j]]])
                ticklabel.set_color(color)

        fig.set_size_inches(6, 1.5*len(self.saved_probs))
        fig.tight_layout()
        plt.savefig(os.path.join(DIR_PATH, name), dpi = 400) 

if __name__ == "__main__":

    # This is the known map of offices by color
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    color_map = [YELLOW, RED, GREEN, BLUE, BLUE, RED, GREEN, BLUE, YELLOW, RED, GREEN]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(color_map) / float(len(color_map))

    localizer = BayesLoc(p0, COLOR_CODES, color_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    localizer.to_visit = [3, 9, 12]
    while not rospy.is_shutdown():
        localizer.go_to_rooms()
        rate.sleep()

    localizer.plot("probability_plot.png")

    rospy.loginfo("Finished!")
    rospy.loginfo(localizer.x)
