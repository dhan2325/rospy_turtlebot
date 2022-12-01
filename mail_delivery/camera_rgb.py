#!/usr/bin/env python
"""Get mean color (r, g, b) value and publish to /mean_img_rgb"""
import rospy
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


class CameraRGB(object):
    def __init__(self):
        self.color_sensor_publisher = rospy.Publisher(
            "mean_img_rgb", Float64MultiArray, queue_size=1
        )
        self.camera_subscriber = rospy.Subscriber(
            "camera/image", Image, self.camera_callback, queue_size=1
        )
        self.bridge = bridge = CvBridge()

    def camera_callback(self, msg):
        try:
            rgb_cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print(e)

        # publish the color sensor reading
        color = np.mean(rgb_cv_img[:200], axis=(0, 1))

        dark = 1e9
        for y in range(640):
            pixel = rgb_cv_img[10][y]
            avg = (int(pixel[0]) + int(pixel[1]) + int(pixel[2])) / 3
            dark = min(dark, avg)

        color_msg = Float64MultiArray()
        color_msg.data = list(color) + [dark]
        #print(color_msg.data)
        self.color_sensor_publisher.publish(color_msg)


def main():
    rospy.init_node("camera_rgb")
    camera = CameraRGB()
    rospy.spin()


if __name__ == "__main__":
    main()