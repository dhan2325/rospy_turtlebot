#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


def main():
    rospy.init_node("motor_node")
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    twist = Twist()
    twist.linear.x = 0.2
    rate = rospy.Rate(1000) #10Hz
    start = time.time()
    while (time.time() - start) < (1.0 / twist.linear.x):
        print(time.time() - start)
        cmd_pub.publish(twist)
        rate.sleep()

    print("EXIT")
    twist.linear.x = 0
    cmd_pub.publish(twist)

    twist.angular.z = 1
    start = time.time()
    while (time.time() - start) < ((math.pi * 2) / twist.angular.z):
        cmd_pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    cmd_pub.publish(twist)

    print("DONE!")


if __name__ == "__main__":
    main()
