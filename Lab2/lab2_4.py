#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

rot_factor = 16 / 15


def move(dis, speed, cmd_pub, twist, rate, freq):
    cycles = round((dis * freq / speed))
    dir = dis / abs(dis)
    twist.linear.x = dir * speed

    for i in range(cycles):
        cmd_pub.publish(twist)
        rate.sleep()

    twist.linear.x = 0
    cmd_pub.publish(twist)

def rotate(theta, speed, cmd_pub, twist, rate, freq, isRadian = True):
    if not isRadian:
        theta = theta * math.pi / 180
    
    cycles = round((theta * freq / speed))
    dir = theta / abs(theta)

    twist.angular.z = dir * speed 
    for i in range(cycles):
        cmd_pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0
    cmd_pub.publish(twist)    

def curve(speed, radius, theta, cmd_pub, twist, rate, freq, isRadian = True):
    if not isRadian:
        theta = theta * math.pi / 180
    
    dir = theta / abs(theta)
    arc_length = theta * radius
    cycles = round(arc_length * freq / speed)
    
    twist.angular.z = (speed / radius) * rot_factor
    twist.linear.x = speed


    for i in range(cycles):
        cmd_pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0
    twist.linear.x = 0
    cmd_pub.publish(twist)

def func_y(x):
    return 50 * math.sin(math.pi * x / 25) / 100

def euc_dis(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def main():
    step_size = 0.01
    length = 3
    points = [(0, 0, 0)]
    for i in range(int(length / step_size)):
        x = i * step_size
        y_i = func_y(i * step_size)
        points.append([y_i, x, math.atan((y_i - points[-1][0]) / step_size)])

    path = []


    for i in range(2, len(points)):
        pose1 = points[i - 1]
        pose2 = points[i]

        theta_dif = pose2[2] - pose1[2]
        dis = euc_dis(pose1, pose2)
        gamma = 0.5 * (math.pi - theta_dif)
        radius = 0
        if theta_dif != 0:
            radius = math.sin(gamma) * dis / math.sin(theta_dif)
        path.append((radius, theta_dif, dis))

    velocity = 0.15
    
    rospy.init_node("motor_node")
    freq = 50.0
    velocity = 0.15
    rate = rospy.Rate(freq)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    twist = Twist()

    print("MOVING!")

    for p in path:
        if p[0] == 0:
            print("MOVE")
            move(p[2], velocity, cmd_pub, twist, rate, freq)
        else:
            print("CURVE")
            curve(velocity, p[0], p[1], cmd_pub, twist, rate, freq)
            print(p)

    
    print("DONE!")
    rospy.spin()



if __name__ == "__main__":
    main()
