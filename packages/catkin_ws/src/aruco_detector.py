#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/detected_aruco_ids', Int32MultiArray, queue_size=1)
        self.sub = rospy.Subscriber('/karakacan/camera_node/image/compressed', CompressedImage, self.callback, queue_size=1)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters()
        rospy.loginfo("Aruco detector started.")
        rospy.spin()

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            flat_ids = ids.flatten().tolist()
            rospy.loginfo(f"Detected ArUco IDs: {flat_ids}")
            msg_out = Int32MultiArray(data=flat_ids)
            self.pub.publish(msg_out)

if __name__ == '__main__':
    try:
        ArucoDetector()
    except rospy.ROSInterruptException:
        pass

