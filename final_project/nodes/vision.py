import colorsys
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

RED = 0
GREEN = 1
BLUE = 2
YELLOW = 3
LINE = 4

COLOR_CODES = [
    [201, 116, 133],  # red
    [120, 170, 132],  # green
    [107, 127, 169],  # blue
    [209, 193, 102],  # yellow
    [160, 160, 160],  # line
]

COLOR_TO_NAME = {
    RED: "red",
    GREEN: "green",
    BLUE: "blue",
    YELLOW: "yellow",
    LINE: "line",
}

def decode_px(line_idx):
    # input current line_idx
    # output: returns True if the robot is on the line
    return line_idx < 550 and line_idx > 30

def is_on_line(line_idx, rgb):
    #prob = get_color_prob(GREEN, rgb, mx = 4)
    #print(prob, rgb, line_idx)
    #print(line_idx, rgb, get_color_prob)
    return decode_px(line_idx)

def color_score(color, rgb):
    # get the h value of the detected rgb
    h, _, _ = colorsys.rgb_to_hsv(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0)
    # get the rgb value of color
    rgb_true = COLOR_CODES[color]
    # get the h value of the color
    h_true, _, _ = colorsys.rgb_to_hsv(rgb_true[0]/255.0, rgb_true[1]/255.0, rgb_true[2]/255.0)
    # normalization factor is calculated by taking the integral of normalization_factor/(h-h_true)**2 from
    # 0 to 2pi with respect to h, then setting the integral to 1 and solving for the normalization factor.
    # return p(color|rgb), added machine epsilon to prevent divison by zero
    return 1 / ((h-h_true)**2 + np.finfo(float).eps)

def color_score_v2(color, rgb):
    h, _, _ = colorsys.rgb_to_hsv(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0)
    # get the rgb value of color
    rgb_true = COLOR_CODES[color]
    # get the h value of the color
    h_true, _, _ = colorsys.rgb_to_hsv(rgb_true[0]/255.0, rgb_true[1]/255.0, rgb_true[2]/255.0)
    score = 1/(min(abs(h-h_true), 1-abs(h-h_true)) + np.finfo(float).eps)
    return score

def get_color_prob(color, rgb, mx = 4):
    tot = 0
    for i in range(mx):
        tot += color_score(i, rgb)
    return color_score(color, rgb)/tot

def plot_color_prob(name, rgb, mx = 4):
    probs = np.array([get_color_prob(c, rgb, mx = 5) for c in range(0, mx)])
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for i in range(0, mx):
        x = i*100
        norm = [p/255.0 for p in COLOR_CODES[i]]
        rect = matplotlib.patches.Rectangle((x, 0), 100, 100, color=norm)
        ax.text(x + 50, 50, str(round(probs[i], 2)) + "\n" + COLOR_TO_NAME[i], ha= "center")
        ax.add_patch(rect)

    rect = matplotlib.patches.Rectangle((100, 200), 200, 200, color=[c/255.0 for c in rgb])
    ax.text(200, 300, "rgb: " + str(rgb), ha = "center")
    ax.add_patch(rect)

    plt.xlim([0, 100*mx])
    plt.ylim([0, 100*mx])
    dir_path = os.path.dirname(os.path.realpath(__file__))

    plt.savefig(os.path.join(dir_path, name), dpi = 400) 
    print(probs)

if __name__ == "__main__":
    plot_color_prob("test.png",  [160, 155, 155], mx = 5)
