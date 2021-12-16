#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from enum import IntEnum
import os 

dir_path = os.path.dirname(os.path.realpath(__file__))

class Color(IntEnum):
    blue = 0
    green = 1
    yellow = 2
    orange = 3
    nothing = 4
    nan = -1

class BayesianLocalizer:

    def __init__(self):

        #measurement_model[z_{k+1}][x_k] = probability
        self. measurement_model = [
            [0.6, 0.2, 0.05, 0.05],
            [0.2, 0.6, 0.05, 0.05],
            [0.05, 0.05, 0.65, 0.20],
            [0.05, 0.05, 0.15, 0.60],
            [0.10, 0.10, 0.10, 0.10],
        ]

        # state_model[x_{k+1}][state update] = probability
        # e.g state_model[move forward (1)][u = 1] = 0.85
        self.state_model = {
            -1: {-1: 0.85, 0: 0.05, 1: 0.05},
            0: {-1: 0.1, 0: 0.9, 1: 0.1},
            1: {-1: 0.05, 0: 0.05, 1:0.85}
        }

        color_list = [Color.yellow, Color.green, Color.blue, Color.orange, Color.orange, Color.green, Color.blue, Color.orange, Color.yellow, Color.green, Color.blue]
        self.color_map = {
            Color.blue: "blue",
            Color.green: "green",
            Color.yellow: "yellow",
            Color.orange: "orange",
            Color.nothing: "nothing",
            Color.nan: "-"
        }

        #self.topo_map = []

        self.N = 11

        self.p_x = {}
        self.office_color = {}


        for office in range(2, 13):
            p = 1.0/self.N
            self.p_x[office] = p # priors: all offices are equally likely
            self.office_color[office] = color_list[office-2]
            #self.topo_map.append((office, p))

        self.saved_predictions = []
        self.saved_controls = []
        self.saved_measurements = [Color.nan]
        self.saved_updates = [self.p_x]

    def nearest(self, xk, xkp1): # minimum u_ks to go from xk to xkp1 (signed)
        CCW = (xkp1 - xk) % self.N
        if (self.N - CCW) < CCW:
            return CCW - self.N
        return CCW

    def p_xkp1_given_xk_uk(self, xkp1, xk, uk):
        assert uk in [-1, 0, 1], f"Invalid control input uk={uk}"
        req_u = self.nearest(xk, xkp1)
        if abs(req_u) > 1:
            return 0
        return self.state_model[req_u][uk] 

    def p_zk_given_xk(self, zk, xk):
        xk_color = self.office_color[xk]
        return self.measurement_model[zk][xk_color]

    def prediction(self, uk):
        self.saved_controls.append(uk)
        new_p_x = {} # so we don't modify p_x mid calculation
        for xkp1 in self.p_x.keys():
            new_p_x[xkp1] = 0
            for xkp, p_xk in self.p_x.items():
                new_p_x[xkp1] += self.p_xkp1_given_xk_uk(xkp1, xkp, uk)*p_xk
        self.p_x = new_p_x
        self.saved_predictions.append(self.p_x.copy())

    def state_update(self, zkp1):
        self.saved_measurements.append(zkp1)
        normalization_factor = 0
        new_p_x = {}
        for xkp1 in self.p_x.keys():
            new_p_x[xkp1] = self.p_zk_given_xk(zkp1, xkp1)*self.p_x[xkp1]
            normalization_factor += new_p_x[xkp1]
        for xkp1 in self.p_x.keys():
            self.p_x[xkp1] = new_p_x[xkp1]/normalization_factor
        self.saved_updates.append(self.p_x.copy())

    def plot(self, name):
        self.saved_controls.append(np.nan)
        fig, axs = plt.subplots(len(self.saved_controls), 2)#, sharex = 'col')

        for i, (prior_ax, post_ax) in enumerate(axs):

            prior_ax.set_xticks([o for o in self.p_x.keys()])
            post_ax.set_xticks([o for o in self.p_x.keys()])

            posts = self.saved_updates[i]
            data = list(posts.items())
            color = self.color_map[self.saved_measurements[i]]
            plot_color = color
            if color == '-' or color == "nothing":
                plot_color = 'grey'
            post_ax.bar([t[0] for t in data], [t[1] for t in data], color = plot_color)
            if i == 0:
                title = f"State Update\nz_k = {color}"
            else:
                title = f"z_k = {color}"
            post_ax.set_title(title)

            j = 2
            for ticklabel in post_ax.get_xticklabels():
                color = self.color_map[self.office_color[j]]
                ticklabel.set_color(color)
                j+=1

            if i < len(self.saved_predictions):
                priors = self.saved_predictions[i]
                if i == 0:
                    title = f"State Prediction\nu_k = {self.saved_controls[i]}"
                else:
                    title = f"u_k = {self.saved_controls[i]}"
                prior_ax.set_title(title)
            else:
                priors = self.saved_updates[-2]
                prior_ax.set_title(f"u_k = -")

            data = list(priors.items())
            prior_ax.bar([t[0] for t in data], [t[1] for t in data])

            j = 2
            for ticklabel in prior_ax.get_xticklabels():
                color = self.color_map[self.office_color[j]]
                ticklabel.set_color(color)
                j+=1



        fig.set_size_inches(12, 20)
        fig.tight_layout()
        plt.savefig(os.path.join(dir_path, name), dpi = 400) 
        


if __name__ == "__main__":
    trajectory = {
        "uk": [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, np.nan],
        "zk": [np.nan, Color.orange, Color.yellow, Color.green, Color.blue, Color.nothing, Color.green, Color.blue, Color.green, Color.orange, Color.yellow, Color.green, Color.blue],
    }

    BL = BayesianLocalizer()

    for k in range(len(trajectory["uk"])):
        uk = trajectory["uk"][k]
        zk = trajectory["zk"][k]
        if not np.isnan(uk):
            BL.prediction(uk)
            #print("after pred:", BL.p_x)
        if not np.isnan(zk):
            BL.state_update(zk)
            #print("after update:", BL.p_x)

    BL.plot("deliverable1_plot.png")

    # most likely started at 7
    # most likely finished at 7