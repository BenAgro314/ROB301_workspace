import numpy as np
import matplotlib.pyplot as plt
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

from vision import *
from utils import kth_diag_indices


class BayesianLocalizer:

    def __init__(self, p0, color_map):
        self.curr_color = None
        self.color_map = color_map
        self.n = len(p0) # number of states
        self.x = p0.reshape(self.n, 1) # 1x11: probability of being at office i+2, where i is the index
        self.state_update_probs = {
            -1: {-1: 0.85, 0: 0.05, 1: 0.05},
            0: {-1: 0.1, 0: 0.9, 1: 0.1},
            1: {-1: 0.05, 0: 0.05, 1:0.85}
        }

        self.measurment_probs = [
            [0.60, 0.05, 0.05, 0.15],
            [0.05, 0.60, 0.2, 0.05],
            [0.05, 0.20, 0.60, 0.05],
            [0.20, 0.05, 0.05, 0.65],
            [0.1, 0.1, 0.1, 0.1],
        ]

        self.saved_probs = []

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        return state update matrix
        """
        res = np.zeros((self.n,self.n))
        res[kth_diag_indices(res, 0)] = self.state_update_probs[0][u] # stay still
        res[kth_diag_indices(res, -1)] = self.state_update_probs[1][u] # move forwards
        res[kth_diag_indices(res, 1)] = self.state_update_probs[-1][u] # move backwards
        res[self.n-1][0] = self.state_update_probs[-1][u] # move backwards (wrap around)
        res[0][self.n-1] = self.state_update_probs[1][u] # move forwards (wrap around)
        return res

    def measurement_model(self):
        """
        Measurement model p(z_k | x_k = color) - given the pixel intensity,
        what's the probability that of each possible color z_k being observed?
        """

        prob = np.zeros((len(self.color_map), 1))
        for i in range(len(self.color_map)):
            prob[i][0] = self.measurment_probs[self.color_map[i]][self.curr_color]
            
        return prob

    def state_predict(self, u):
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """
        self.x = self.state_model(u).dot(self.x)
        self.saved_probs.append((self.x, "prediction", u))

    def state_update(self):
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """
        self.x = self.measurement_model()*self.x
        self.x = self.x/sum(self.x)
        self.saved_probs.append((self.x, "update", self.curr_color))

    def plot(self, name):
        fig, axs = plt.subplots(len(self.saved_probs), 1)
        for i, ax in enumerate(axs):
            probs = self.saved_probs[i][0]
            pred = self.saved_probs[i][1] == "prediction"
            ax.set_xticks([k for k in range(2, self.n+2)])

            plot_color = "black" if pred else tuple([r/255.0 for r in COLOR_CODES[self.saved_probs[i][2]]])
            ax.bar([k for k in range(2, self.n+2)], [float(p) for p in probs], color = plot_color)
            if pred: 
                title = f"Prediction: u={self.saved_probs[i][2]}"
            else:
                title = f"Update: color={COLOR_TO_NAME[self.saved_probs[i][2]]}"
            ax.set_title(title)
            ax.set_yticks([0] + list(max(probs)))

            for j, ticklabel in enumerate(ax.get_xticklabels()):
                color = tuple([r/255.0 for r in COLOR_CODES[color_map[j]]])
                ticklabel.set_color(color)

        fig.set_size_inches(6, 30)
        fig.tight_layout()
        plt.savefig(os.path.join(dir_path, name), dpi = 400) 

if __name__ == "__main__":
    trajectory = {
        "uk": [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, None],
        "zk": [None, RED, YELLOW, GREEN, BLUE, None, GREEN, BLUE, GREEN, RED, YELLOW, GREEN, BLUE]
    }

    color_map = [YELLOW, GREEN, BLUE, RED, RED, GREEN, BLUE, RED, YELLOW, GREEN, BLUE]
    p0 = np.ones_like(color_map) / len(color_map)
    BL = BayesianLocalizer(p0, color_map)

    for k in range(len(trajectory["uk"])):
        uk = trajectory["uk"][k]
        zk = trajectory["zk"][k]
        if uk is not None:
            BL.state_predict(uk)
            #print("after pred:", BL.p_x)
        if zk is not None:
            BL.curr_color = zk
            BL.state_update()
            #print("after update:", BL.p_x)

    BL.plot("simulation_plot.png")

    # most likely started at 7
    # most likely finished at 7