#!/usr/bin/env python3
import os
import rospy
import signal
from std_msgs.msg import Int32MultiArray
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Header

# Define tag behaviors
LEFT_TAG_ID = 10
RIGHT_TAG_ID = 57
STOP_TAG_ID = 26
DISTANCE_THRESHOLD = 0.50 # meters

class ArucoMovementController:
    def __init__(self):
        rospy.init_node('aruco_movement_controller', disable_signals=True)  # disable to handle signals ourselves

        self.vehicle_name = os.environ['VEHICLE_NAME']
        self.cmd_pub = rospy.Publisher(
            f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )

        self.current_distance = None
        self.detected_tag = None
        self.movement_executed = False

        rospy.Subscriber('/detected_aruco_ids', Int32MultiArray, self.tag_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/front_center_tof_driver_node/range", Range, self.tof_callback)

        # Attach Ctrl+C handler
        signal.signal(signal.SIGINT, self.shutdown_handler)

        rospy.loginfo("Aruco movement controller with ToF started.")
        self.loop()

    def tag_callback(self, msg):
        # Only register a tag if it's a known one
        for tag_id in msg.data:
            if tag_id in [LEFT_TAG_ID, RIGHT_TAG_ID, STOP_TAG_ID]:
                self.detected_tag = tag_id
                self.movement_executed = False
                return
        self.detected_tag = None
        self.movement_executed = False

    def tof_callback(self, msg):
        self.current_distance = msg.range
	
    def move(self, left, right):
        cmd = WheelsCmdStamped()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = left
        cmd.vel_right = right
        self.cmd_pub.publish(cmd)

    def shutdown_handler(self, signum, frame):
        rospy.loginfo("Shutdown signal received. Stopping the robot.")
        self.move(0, 0)
        rospy.signal_shutdown("Shutdown via SIGINT")

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.detected_tag and self.current_distance is not None:
                if self.current_distance > DISTANCE_THRESHOLD and not self.movement_executed:
                    self.move(0.3, 0.25)  # Move forward
                elif not self.movement_executed:
                    rospy.loginfo(f"Close to tag {self.detected_tag}. Executing behavior...")
                    if self.detected_tag == LEFT_TAG_ID:
                        self.move(0.3, -0.3)  # Turn left
                        rospy.sleep(1.0)
                    elif self.detected_tag == RIGHT_TAG_ID:
                        self.move(-0.3, 0.3)  # Turn right
                        rospy.sleep(1.0)
                    elif self.detected_tag == STOP_TAG_ID:
                        self.move(0, 0)
                        rospy.sleep(1.0)
                    self.move(0, 0)
                    self.movement_executed = True
                else:
                    self.move(0, 0)  # Stay stopped after action
            else:
                self.move(0, 0)  # No tag or no range data
            rate.sleep()

if __name__ == '__main__':
    try:
        ArucoMovementController()
    except rospy.ROSInterruptException:
        pass

