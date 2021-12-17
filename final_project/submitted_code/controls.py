import rospy
import numpy as np
from geometry_msgs.msg import Twist
from pid import send_cmd
from vision import decode_px, is_on_line

class Control:

    def __init__(self, u):
        self.alive = True
        self.time = 0
        self.u = u
        self.countdown = 0
        self.start_time = None

    def is_alive(self):
        return self.alive

    def __str__(self):
        return "Control(" + str(self.u) + ")"

    def pause(self, bayes_loc, duration):
        duration = rospy.Duration.from_sec(duration)
        done = False
        if self.start_time is None:
            self.start_time = rospy.get_rostime()
            send_cmd(bayes_loc.cmd_pub, 0, 0)
        if rospy.is_shutdown() or (rospy.get_rostime() - self.start_time > duration):
            done = True
            self.start_time = None
        return done

    def rotate(self, bayes_loc, radians, omega_vel):
        twist = Twist()
        twist.angular.z = (radians/abs(radians))*omega_vel
        duration = rospy.Duration(abs(radians)/omega_vel)
        if self.start_time is None:
            self.start_time = rospy.get_rostime()
        if not rospy.is_shutdown() and (rospy.get_rostime() - self.start_time < duration):
            bayes_loc.cmd_pub.publish(twist)
            return False

        self.start_time = None
        return True

    def forward(self, bayes_loc, meters, x_vel):
        twist = Twist()
        twist.linear.x = (meters/abs(meters))*x_vel
        duration = rospy.Duration(abs(meters)/x_vel)
        if self.start_time is None:
            self.start_time = rospy.get_rostime()
        if not rospy.is_shutdown() and (rospy.get_rostime() - self.start_time < duration):
            bayes_loc.cmd_pub.publish(twist)
            return False

        self.start_time = None
        return True

    def move_until_line(self, bayes_loc):
        assert self.u == -1 or self.u == 1, "Invalid u for move_until_line"
        on_line = is_on_line(bayes_loc.curr_px, bayes_loc.curr_color)
        if not on_line:
            send_cmd(bayes_loc.cmd_pub, bayes_loc.pid_controller.x_vel*self.u, 0)
        else:
            send_cmd(bayes_loc.cmd_pub, 0, 0)
        return on_line

    def move_until_color(self, bayes_loc):
        assert self.u == -1 or self.u == 1, "Invalid u for move_until_color"
        on_line = is_on_line(bayes_loc.curr_px, bayes_loc.curr_color)
        if on_line:
            x_vel, omega = bayes_loc.pid_controller.follow_the_line(bayes_loc.curr_px, direction = self.u)
            send_cmd(bayes_loc.cmd_pub, x_vel, omega)
        else:
            send_cmd(bayes_loc.cmd_pub, 0, 0)
        return on_line

    def step(self, bayes_loc):
        if not self.alive:
            return

        assert self.u in [-1, 0, 1]

        if self.u == 1 or self.u == -1:
            if self.time == 0: # move forward/backwards until line
                on_line = self.move_until_line(bayes_loc)
                if on_line:
                    self.time += 1
            if self.time == 1: # follow line forwards/backwards until color
                on_line = self.move_until_color(bayes_loc)
                if not on_line:
                    self.time += 1
                    self.alive = False
        else: # self.u = 0
            if self.time == 0: # forward 5 cm
                if self.forward(bayes_loc, 0.05, bayes_loc.pid_controller.x_vel):
                    self.time += 1
            if self.time == 1: # pause
                if self.pause(bayes_loc, 0.25):
                    self.time += 1
            if self.time == 2: # rotate 90 degrees to the left
                if self.rotate(bayes_loc, np.pi/2, 1):
                    self.time += 1
            if self.time == 3: # pause 0.5 s
                if self.pause(bayes_loc, 0.5):
                    self.time += 1
            if self.time == 4: # rotate 90 degrees to the right
                if self.rotate(bayes_loc, -np.pi/2, 1):
                    self.time += 1
                    self.alive = False
