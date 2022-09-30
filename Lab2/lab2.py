#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

def move_rotate():
    pass


def move(dis, speed, cmd_pub, twist, rate, freq):
    cycles = round((dis * freq / speed))
    dira = dis / abs(dis)
    twist.linear.x = dira * speed

    for i in range(cycles):
        cmd_pub.publish(twist)
        rate.sleep()

    twist.linear.x = 0
    cmd_pub.publish(twist)

def rotate(theta, speed, cmd_pub, twist, rate, freq, isRadian = True):
    if not isRadian:
        theta = theta * math.pi / 180
    
    cycles = round((theta * freq / speed))
    dira = theta / abs(theta)

    twist.angular.z = dira * speed 
    for i in range(cycles):
        cmd_pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0
    cmd_pub.publish(twist)    


def main():
    rospy.init_node("motor_node")
    freq = 50.0
    angular_velocity = 0.5
    velocity = 0.15
    rate = rospy.Rate(freq)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    twist = Twist()

    rotate(14, angular_velocity / 2, cmd_pub, twist, rate, freq, isRadian = False)

    move(2.06155, velocity, cmd_pub, twist, rate, freq)

    rotate(121, angular_velocity, cmd_pub, twist, rate, freq, isRadian = False)
    
    print("DONE!")
    rospy.spin()



if __name__ == "__main__":
    main()
