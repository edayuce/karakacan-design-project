#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import AprilTagDetectionArray  # Adjust if your tag message type is different
from std_msgs.msg import Header

FORWARD_SPEED = 0.3
TURN_SPEED = 0.4
TURN_DURATION = 1.0  # seconds
TARGET_TAG_ID = 57

class AprilTagFollowerNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.GENERIC)
        vehicle_name = os.environ['VEHICLE_NAME']

        self._wheels_pub = rospy.Publisher(
            f"/{vehicle_name}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )

        rospy.Subscriber(
            f"/{vehicle_name}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback
        )

        self._turning = False
        self._last_turn_time = rospy.Time(0)

    def tag_callback(self, msg):
        for detection in msg.detections:
            if TARGET_TAG_ID in detection.id:
                rospy.loginfo(f"Tag {TARGET_TAG_ID} detected, initiating left turn.")
                self._turning = True
                self._last_turn_time = rospy.Time.now()
                break

    def publish_wheel_command(self, left, right):
        cmd = WheelsCmdStamped()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = left
        cmd.vel_right = right
        self._wheels_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self._turning:
                elapsed = (rospy.Time.now() - self._last_turn_time).to_sec()
                if elapsed < TURN_DURATION:
                    self.publish_wheel_command(TURN_SPEED, -TURN_SPEED)  # left turn
                else:
                    self._turning = False  # resume forward motion
            else:
                self.publish_wheel_command(FORWARD_SPEED, FORWARD_SPEED)
            rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("Shutting down: stopping the wheels.")
        self.publish_wheel_command(0, 0)


if __name__ == '__main__':
    node = AprilTagFollowerNode(node_name='apriltag_follower_node')
    node.run()
    rospy.spin()
