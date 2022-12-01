#!/usr/bin/env python
from pickletools import uint2
import time
import rospy
import math
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from std_msgs.msg import String, Float64MultiArray, UInt32
import numpy as np
import colorsys

MAX_LIN_VEL = 0.06
MAX_ANG_VEL = 0.3
colours = ['red', 'green', 'blue', 'yellow', 'line']

# og 
colour_codes = [
    [230, 74, 132],  # red
    [145, 174, 139],  # green
    [176, 165, 183],  # blue
    [169, 158, 148],  # yellow
    [165, 154, 157],  # line
]

stateMatrix = {
    -1 : [0.85, 0.1, 0.05], 
    0 : [0.05, 0.9, 0.05], 
    1 : [0.01, 0.01, 0.98]
}
measurementMatrix = [
    [0.8, 0.1, 0.05, 0.05],
    [0.1, 0.8, 0.05, 0.05],
    [0.05, 0.05, 0.8, 0.1],
    [0.05, 0.05, 0.1, 0.8],
    [0.1, 0.1, 0.1, 0.1]
]


colourDic = {
    'blue' : 0,
    'green' : 1,
    'yellow' : 2,
    'red' : 3,
    'line' : 4,
}

colourMap = ['yellow', 'green', 'blue', 'red', 'red', 'green', 'blue', 'red', 'yellow', 'green', 'blue']

stops = [9, 7, 6]

move = [-1, 0, 1]


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.actual = 320
        self.twist = Twist()
        self.line_desire = 320
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", Float64MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.prev_dif = [0]
        self.dif = 0
        self.prev_dark = 0
        self.first_straight_time = time.time()
        self.pausing = False
        self.has_paused = False
        self.cur_state = [1 / 11] * 11
        self.cur_colour = (0,0,0)  # most recent measured colour


    def path_control(self):
        # need bot to be very stable
        dif = self.line_desire - self.actual
        #print(dif)
        direction = 1
        if dif:
            direction = dif / abs(dif)
        self.twist.linear.x = MAX_LIN_VEL
        angular_p = (direction * (((dif) / 200) ** 2) * MAX_ANG_VEL)
        current_angular = angular_p
        self.twist.angular.z = current_angular
        self.cmd_pub.publish(self.twist)

    def find_closest_colour(self, rgb):
        # more add constant value/multiplier to line term in array, so we start going straight earlier at boundary
        # if bot is slow, more likely to mess up on boundaries...
        difs = []
        for x, colour in enumerate(colour_codes):
            dif = (rgb[0] - colour[0]) ** 2 + (rgb[1] - colour[1]) ** 2 + (rgb[2] - colour[2]) ** 2
            difs.append((dif, colours[x]))
        difs.sort()
        return difs[0]


    def go_straight(self):
        self.twist.linear.x = MAX_LIN_VEL
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)

    def stay_put(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)

    def colour_callback(self, msg):
        #print(msg.data)
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        self.dark = self.cur_colour[-1]
        # col_array.append(self.cur_colour)
        if not self.pausing:
            self.callback()
        #print("DARKNESS", self.dark)

    def line_callback(self, msg):
        self.actual = msg.data

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def run_bayesian(self):
        prev_state = self.cur_state
        closest_colour = self.find_closest_colour(self.cur_colour)[1]
        prediction_state = self.state_prediction(prev_state, 1)
        self.cur_state = self.state_update(prediction_state, closest_colour) 
    

        print(closest_colour)
        print([round(state, 3) for state in self.cur_state])

        for stop in stops:
            if self.cur_state[stop] >= 0.75:
                print("PAUSING @ OFFICE NUMBER:", stop + 2)
                self.pause(3)
                

    def state_prediction(self, previousState, input):
        prediction = [0] * 11
        for i in range(len(previousState)):
            if previousState[i]:
                for m in move:
                    prediction[(i + m) % len(prediction)] += previousState[i] * stateMatrix[input][m + 1]
        return prediction

    def state_update(self, statePrediction, measurement):
        updated_state = [0] * 11
        normalizer = 0
        for i in range(len(statePrediction)):
            normalizer += statePrediction[i] * measurementMatrix[colourDic[measurement]][colourDic[colourMap[i]]] 
        
        for i in range(len(statePrediction)):
            if statePrediction[i]:
                updated_state[i] = statePrediction[i] * measurementMatrix[colourDic[measurement]][colourDic[colourMap[i]]] /normalizer
        return updated_state
        

    def pause(self, duration):
        self.pausing = True
        t = time.time()
        while time.time() - t < duration:
            self.stay_put()
            rate.sleep()
        self.pausing = False

    def callback(self):
        col_dif = self.find_closest_colour(self.cur_colour)
        dark_dif = abs(self.dark - self.prev_dark)
        delta_dif = abs(self.prev_dif[0] - col_dif[0])
        if self.dark <= 90:
            #print("PATH CONTROL")
            self.first_straight_time = 0
            self.has_paused = False
            self.path_control()
        else:
            if not self.first_straight_time:
                self.first_straight_time = time.time()
            #print("STRAIGHT")
            self.go_straight()
            if time.time() - self.first_straight_time > 1.2 and not self.has_paused:
                self.run_bayesian()
                self.has_paused = True
        self.prev_dif = col_dif
        self.prev_dark = self.dark

    def loop(self):
        while not rospy.is_shutdown():
            rate.sleep()

# for i, timeStep in enumerate(timeSteps):
#     if i:
#         previousState = state[-1]
#         newStatePrediction = state_prediction(previousState, timeSteps[i-1][0])
#         newState = state_update(newStatePrediction, timeStep[1])
#         state.append(newState)
#         print("TimeStep:", i, timeStep)
#         # print("PREVIOUS STATE:", previousState)
#         # print("NEW STATE PREDICTION:", newStatePrediction)
#         print("NEW STATE:", newState)
#         print(sum(newState))
#         print('*******************************************************')

# start at office 0
        
# stop @ 9, 7, 6

if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: green, 1: blue, 2: red, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rate = rospy.Rate(10)
    localizer.loop()

    # total_rgb = [0, 0, 0]
    # for c in col_array:
    #     total_rgb[0] += c[0]
    #     total_rgb[1] += c[1]
    #     total_rgb[2] += c[2]
    
    # n = len(col_array)
    # print(round(total_rgb[0] / n), round(total_rgb[1] / n), round(total_rgb[2] / n))

    # plt.plot([i for i in range(len(differences))], differences)
    # plt.ylabel('meow')
    # plt.show()


    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)