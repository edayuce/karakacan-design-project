#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import AprilTagDetectionArray

class AprilTagDebuggerNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.GENERIC)
        vehicle_name = os.environ['VEHICLE_NAME']
        topic = f"/{vehicle_name}/apriltag_detector_node/detections"

        rospy.Subscriber(topic, AprilTagDetectionArray, self.callback)
        rospy.loginfo(f"Subscribed to {topic} to monitor tag detections.")

    def callback(self, msg):
        if not msg.detections:
            rospy.loginfo("No AprilTags detected.")
        else:
            for detection in msg.detections:
                rospy.loginfo(f"Detected tag ID(s): {detection.id}")
                if detection.pose:
                    pos = detection.pose.pose.pose.position
                    rospy.loginfo(f"Tag position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")

if __name__ == '__main__':
    node = AprilTagDebuggerNode(node_name='apriltag_debugger_node')
    rospy.spin()

