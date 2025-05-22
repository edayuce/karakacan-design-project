#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import signal
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

class DashedLineFollower:
    def __init__(self):
        rospy.init_node('dashed_line_follower', anonymous=True)

        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("/karakacan/camera_node/image/compressed", CompressedImage, self.image_callback)
        self.pub_wheels = rospy.Publisher("/karakacan/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        signal.signal(signal.SIGINT, self.handle_sigint)
        rospy.loginfo("Dashed line follower initialized. Press Ctrl+C to stop.")

    def handle_sigint(self, signum, frame):
        rospy.loginfo("SIGINT received. Stopping wheels and exiting.")
        self.stop_wheels()
        os._exit(0)  # force exit (especially if this is PID 1 in the container)

    def stop_wheels(self):
        stop_cmd = WheelsCmdStamped()
        stop_cmd.vel_left = 0.0
        stop_cmd.vel_right = 0.0
        stop_cmd.header.stamp = rospy.Time.now()
        self.pub_wheels.publish(stop_cmd)

    def image_callback(self, msg):
        # Convert image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Crop and process
        height, width = image.shape[:2]
        crop = image[int(height*0.6):height, :]

        # Convert to HSV and filter yellow
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Contour detection
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cx = width // 2

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])

        # Compute error and control
        error = cx - width // 2
        correction = float(error) / 100.0

        left_speed = 0.2 - correction
        right_speed = 0.2 + correction

        # Clamp and publish
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.vel_left = np.clip(left_speed, -1.0, 1.0)
        wheels_cmd.vel_right = np.clip(right_speed, -1.0, 1.0)
        wheels_cmd.header.stamp = rospy.Time.now()
        self.pub_wheels.publish(wheels_cmd)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    follower = DashedLineFollower()
    follower.run()

