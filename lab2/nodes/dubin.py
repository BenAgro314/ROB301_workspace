import numpy as np
import matplotlib.pyplot as plt

DEBUG = True

class Circle:

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def plot(self, color = 'r'):
        circle1 = plt.Circle(self.center, self.radius, color=color, fill = False)
        plt.gca().add_patch(circle1)

def angle_between_vectors(v1, v2, counter_clockwise = True):
    # compute the clockwise angle between two vectors v1 and v2 in the plane
    # a negative value indicates a counter clockwise rotation
    det = np.linalg.det(np.array([v1, v2]))
    dot = v1.dot(v2)
    angle =  (-1 + 2*int(counter_clockwise))*np.arctan2(det,dot)
    if angle < 0:
        angle += 2*np.pi
    return angle


class DubinsPath:

    def __init__(self, word, theta_start, S, theta_end, p1, p2, v1, v2, start_circle, end_circle):
        self.word = word
        self.theta_start = theta_start
        self.theta_end = theta_end
        self.S = S
        self.p1 = p1
        self.p2 = p2
        self.v1 = v1
        self.v2 = v2
        self.start_circle = start_circle
        self.end_circle = end_circle

    @property
    def R(self):
        return self.start_circle.radius

    def length(self):
        return np.linalg.norm(self.S[0] - self.S[1]) + self.theta_start*self.R + self.theta_end*self.R

    def plot(self):
        # plot the path
        plt.plot([s[0] for s in self.S], [s[1] for s in self.S], 'k')
        v_start = self.p1 - self.start_circle.center
        arc = []
        for theta in np.linspace(0, self.theta_start):
            if self.word[0] == 'L':
                theta = -theta
            v_pt = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]).dot(v_start)
            arc.append(self.start_circle.center + v_pt)
        plt.plot([s[0] for s in arc], [s[1] for s in arc], 'k')
        v_end = self.S[1] - self.end_circle.center
        arc = []
        for theta in np.linspace(0, self.theta_end):
            if self.word[-1] == 'L':
                theta = -theta
            v_pt = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]).dot(v_end)
            arc.append(self.end_circle.center + v_pt)
        plt.plot([s[0] for s in arc], [s[1] for s in arc], 'k')
        plt.plot(self.p1[0], self.p1[1], 'ro', label = 'start')
        plt.plot(self.p2[0], self.p2[1], 'go', label = 'goal')
        plt.arrow(self.p1[0], self.p1[1], self.v1[0], self.v1[1], color = 'r', length_includes_head = True, head_width = 0.1)
        plt.arrow(self.p2[0], self.p2[1], self.v2[0], self.v2[1], color = 'g', length_includes_head = True, head_width = 0.1)


def dubins(pt1, v1, pt2, v2, R):
    # pt1: numpy array representing start point
    # v1: numpy array representing start direction
    # pt2: numpy array representing end direction
    # v2: numpy array representing end direction
    a1 = v1/np.linalg.norm(v1)
    a2 = np.array([[0,1],[-1,0]]).dot(a1)
    b1 = v2/np.linalg.norm(v2)
    b2 = np.array([[0,1],[-1,0]]).dot(b1)


    circles = {
        "L":
            [
                Circle(pt1 - a2*R, radius = R),
                Circle(pt2 - b2*R, radius = R)
            ],
        "R":
            [
                Circle(pt1 + a2*R, radius = R),
                Circle(pt2 + b2*R, radius = R)
            ]
    }

    #if DEBUG:
        #circles['L'][0].plot()
        #circles['R'][0].plot()
        #circles['L'][1].plot()
        #circles['R'][1].plot()

    words = ['LSL', 'LSR', 'RSL', 'RSR']

    min_length = np.inf
    min_length_path = None
    for word in words:
        start_circle = circles[word[0]][0]
        end_circle = circles[word[-1]][1]
        if word[0] == word[-1]:
            S = outer_tangent(start_circle,end_circle, counter_clockwise=word[0] == "L")
        else:
            S = inner_tangent(start_circle,end_circle, start_counter_clockwise=word[0] == "L")
        theta_start = angle_between_vectors(
            pt1-start_circle.center,
            S[0]-start_circle.center,
            counter_clockwise = word[0] == "L"
        )
        theta_end  = angle_between_vectors(
            S[1] - end_circle.center,
            pt2 - end_circle.center,
            counter_clockwise=word[-1] == "L"
        )
        #plt.plot([s[0] for s in S], [s[1] for s in S])
        path = DubinsPath(word, theta_start, S, theta_end, pt1, pt2, a1, b1, start_circle, end_circle)
        #print(f"Path: {path.word}, length: {path.length()}")
        #path.plot()
        #plt.title(f"Path: {path.word}, Length: {np.round(path.length(),3)}")
        #plt.xlabel('x')
        #plt.ylabel('y')
        #plt.legend()
        #plt.gca().set_aspect('equal', adjustable='box')
        #plt.savefig(word + ".png", dpi = 300)
        #plt.close()
        if min_length > path.length():
            min_length = path.length()
            min_length_path = path
   
    #print(f"Shorted Length Path: {min_length_path.word}, length: {min_length}")
    #min_length_path.plot()
    return min_length_path

def outer_tangent(start_circle, end_circle, counter_clockwise = True):
    assert start_circle.radius == end_circle.radius, "Only supports equal radius circles"
    x1, y1 = start_circle.center
    x2, y2 = end_circle.center
    R = start_circle.radius
    alpha = -np.arctan2((y2 - y1), (x2- x1))
    mult = 1 - 2*int(counter_clockwise)
    xo1 = x1 + mult*R*np.sin(alpha)
    yo1 = y1 + mult*R*np.cos(alpha)
    xo2 = x2 + mult*R*np.sin(alpha)
    yo2 = y2 + mult*R*np.cos(alpha)
    return np.array([xo1, yo1]), np.array([xo2, yo2])

def inner_tangent(start_circle, end_circle, start_counter_clockwise = True):
    assert start_circle.radius == end_circle.radius, "Only supports equal radius circles"
    x1, y1 = start_circle.center
    x2, y2 = end_circle.center
    R = start_circle.radius
    alpha= np.arctan2((y2 - y1), (x2- x1))
    beta = (1-2*int(start_counter_clockwise))*np.arccos((2*R)/(np.sqrt((x2-x1)**2 + (y2-y1)**2)))
    xi1 = x1 + R*np.cos(alpha+beta)
    yi1 = y1 + R*np.sin(alpha+beta)
    xi2 = x2 - R*np.cos(alpha+beta)
    yi2 = y2 - R*np.sin(alpha+beta)
    return np.array([xi1, yi1]), np.array([xi2, yi2])

if __name__ == "__main__":
    dubins(np.array([0,0]), np.array([1,0]), np.array([2,0.5]), np.array([-1,1]), 0.25)

