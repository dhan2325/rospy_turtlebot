#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32

import matplotlib.pyplot as plt

import math

MAX_LIN_VEL = 0.17
MAX_ANG_VEL = 1.81 * (MAX_LIN_VEL / 0.26)
MAX_ERROR = 1500

positions = []


class Controller(object):

    def path_control(self):
        error = self.desire - self.actual
        error_dif = error - self.last_error
        self.total_error += error

        dif = self.desire - self.actual
        direction = 1
        if dif:
            direction = dif / abs(dif)

        self.twist.linear.x = MAX_LIN_VEL

        self.total_error = min(max(self.total_error, -MAX_ERROR), MAX_ERROR)

        angular_p = (direction * (((dif) / 180) ** 2) * MAX_ANG_VEL)
        
        angular_i = self.total_error * (MAX_ANG_VEL / 15000)

        angular_d = (error_dif) * (MAX_ANG_VEL / 80)

        current_angular = angular_p + angular_d + angular_i

        self.twist.angular.z = current_angular


        self.cmd_pub.publish(self.twist)
        self.last_error = error
        self.prev_angular = self.twist.angular.z


    def __init__(self):
        # publish motor commands
        self.freq = 30.0
        self.angular_velocity = 0.5
        self.velocity = 0.15
        self.rate = rospy.Rate(self.freq)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.twist = Twist()
        self.total_error = 0
        self.deriv = 0
        self.last_error = 0
        self.prev_angular = 0

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

        self.actual = 320
        self.desire = 320
        self.isfive = 0

    def camera_callback(self, msg):
        self.actual = msg.data
        self.path_control()

    def follow_the_line(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        f = open("positions.txt", "w")
        for p in positions:
            f.write(str(p) + '\n')
        f.close()


if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line()
