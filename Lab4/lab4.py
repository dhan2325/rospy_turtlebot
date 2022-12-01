#!/usr/bin/env python
from pickletools import uint2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import UInt32

import matplotlib.pyplot as plt
import math, time
import numpy as np

DISTANCE_BETWEEN_POINTS = 0.61
MAX_LIN_VEL = 0.1
MAX_ANG_VEL = 1

x = []
cov_plot = []
t = []

class KalmanFilter(object):
    
    def __init__(self, h, d, x_0, Q, R, P_0, timestep):
        self.timestep = timestep
        self.h = h
        self.d = d

        self.cov_Q = Q
        self.cov_R = R
        self.cov_P = P_0

        self.x = x_0

        self.u = 0  # initialize the cmd_vel input
        self.phi = 0  # initialize the measurement input

        self.x_updated_prediction = 0

        self.waypoints = [0.61, 1.22, 2.44, 3.05]
        self.currentDestination = 0

        self.line_desire = 320
        self.actual = 320

        self.current_distance = 0

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)
        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.prev_time = -1
        self.epoch = 0
        self.used = False

    def path_control(self):
        dif = self.line_desire - self.actual
        direction = 1
        if dif:
            direction = dif / abs(dif)
        self.twist.linear.x = MAX_LIN_VEL
        angular_p = (direction * (((dif) / 200) ** 2) * MAX_ANG_VEL)
        current_angular = angular_p
        self.twist.angular.z = current_angular
        self.cmd_pub.publish(self.twist)

    def pause(self, t):
        start_time = time.time()
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        while time.time() - start_time < t:
            self.cmd_pub.publish(self.twist)
            rospy.sleep(0.1)

    def camera_callback(self, msg):
        self.actual = msg.data

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data + 0.054
        self.measurement_used = False
    
    # state covariance estimation
    # also returns kalman gain
    def run_cov(self, x_prediction):
        cov_P_estimate = self.cov_P + self.cov_Q
        lin_measurement = self.h / ((self.d - x_prediction) ** 2 + self.h ** 2) 
        cov_measurement_prediction = lin_measurement ** 2 + self.cov_R
        kalman_gain = (cov_P_estimate  * lin_measurement) / cov_measurement_prediction
        self.cov_P = cov_P_estimate -  kalman_gain ** 2 * cov_measurement_prediction
        if self.measurement_used:
            self.cov_P = cov_P_estimate
        return kalman_gain

    def get_state_prediction(self, current_input):
        return self.x_updated_prediction + self.timestep * current_input

    def get_measurement_prediction(self, x_prediction):
        return math.atan2(self.h, (self.d - x_prediction))

    def get_measurement_residual(self, current_measurement, measurement_prediction):
        return current_measurement - measurement_prediction

    # state estimation
    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi

        x_prediction = self.get_state_prediction(current_input)
        measurement_prediction = self.get_measurement_prediction(x_prediction)
        measurement_residual = self.get_measurement_residual(current_measurement, measurement_prediction)
        kalman_gain = self.run_cov(x_prediction)

        self.x_updated_prediction = x_prediction
        if not self.measurement_used:
            self.x_updated_prediction += kalman_gain * measurement_residual

        self.measurement_used = True

        print("CURRENT ESTIMATION:", self.x_updated_prediction)

        x.append(self.x_updated_prediction)
        cov_plot.append(self.cov_P)
        if not t:
            self.epoch = time.time()
            t.append(0)
        else:
            t.append(time.time() - self.epoch)

        if self.x_updated_prediction > self.waypoints[self.currentDestination]:
            print("PAUSED")
            self.pause(2)
            print("UNPAUSED")
            self.currentDestination += 1
        else:
            self.path_control()

        self.state_pub.publish(self.x)


if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.61 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 0.01  # TODO: Set process noise covariance
    R = (math.pi / 20)  # TODO: measurement noise covariance
    P_0 = 0.04  # TODO: Set initial state covariance

    frequency = 30
    rate = rospy.Rate(frequency)

    timestep = 1 / frequency

    kf = KalmanFilter(h, d, x_0, Q, R, P_0, timestep)
    rospy.sleep(1)

    while not rospy.is_shutdown() and kf.waypoints: 
        kf.run_kf()
        rate.sleep()

    plt.plot(t, x)
    plt.show()

    plt.plot(t, cov_plot)
    plt.show()


    print("ENDING!")