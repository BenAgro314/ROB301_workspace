import colorsys
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
    return line_idx < 550 and line_idx > 30

def is_on_line(line_idx, rgb):
    return decode_px(line_idx)

def p_rgb_given_color(rgb, color, kappa = np.pi):
    h, _, _ = colorsys.rgb_to_hsv(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0)
    rgb_true = COLOR_CODES[color]
    h_true, _, _ = colorsys.rgb_to_hsv(rgb_true[0]/255.0, rgb_true[1]/255.0, rgb_true[2]/255.0)
    d = np.pi*(h - h_true)
    return np.exp(kappa*np.cos(d))/(2*np.pi*np.i0(kappa))
