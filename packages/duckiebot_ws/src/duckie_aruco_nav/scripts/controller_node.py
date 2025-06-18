#!/usr/bin/env python3
import rospy
import os
import signal
import math
import rospkg
from std_msgs.msg import Int32, Int32MultiArray, Header
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range
import yaml

# Tunable Parameters
DISTANCE_THRESHOLD = 0.1
FORWARD_SPEED = 0.2
ROTATE_SPEED = 0.1
ANGLE_TOLERANCE = 0.2  # radians

class DirectNavController:
    def __init__(self):
        rospy.init_node('direct_nav_controller_node', disable_signals=True)
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")

        # Load graph.yaml
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('duckiebot_ws')
        yaml_path = os.path.join(pkg_path, 'src', 'duckie_aruco_nav', 'config', 'graph.yaml')
        with open(yaml_path, 'r') as f:
            self.graph_data = yaml.safe_load(f)["nodes"]

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher(
            f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped, queue_size=1
        )
        rospy.Subscriber("/next_marker_id", Int32, self.next_marker_callback)
        rospy.Subscriber("/detected_aruco_ids", Int32MultiArray, self.aruco_callback)
        rospy.Subscriber(
            f"/{self.vehicle_name}/front_center_tof_driver_node/range",
            Range, self.tof_callback
        )

        # State
        self.current_marker = None
        self.next_marker = None
        self.distance = float("inf")
        self.state = "APPROACH"  # APPROACH -> ROTATE -> MOVE
        self.visible_markers = []

        signal.signal(signal.SIGINT, self.shutdown_handler)
        rospy.loginfo("DirectNavController started.")
        self.loop()

    def next_marker_callback(self, msg):
        if self.next_marker != msg.data:
            self.current_marker = self.next_marker
            self.next_marker = msg.data
            self.state = "APPROACH"
            rospy.loginfo(f"New target: {self.next_marker} (from {self.current_marker})")

    def aruco_callback(self, msg):
        self.visible_markers = msg.data

    def tof_callback(self, msg):
        self.distance = msg.range

    def move(self, left, right):
        cmd = WheelsCmdStamped(
            header=Header(stamp=rospy.Time.now()),
            vel_left=left,
            vel_right=right
        )
        self.cmd_pub.publish(cmd)

    def angle_to_target(self):
        if self.current_marker is None or self.next_marker is None:
            return None
        cur = self.graph_data[self.current_marker]
        nxt = self.graph_data[self.next_marker]
        dx = nxt['x'] - cur['x']
        dy = nxt['y'] - cur['y']
        angle = math.atan2(dy, dx)
        return angle

    def shutdown_handler(self, signum, frame):
        rospy.loginfo("Controller shutdown.")
        self.move(0, 0)
        rospy.signal_shutdown("SIGINT")

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.next_marker is None:
                self.move(0, 0)
                rate.sleep()
                continue

            if self.state == "APPROACH":
                if self.next_marker in self.visible_markers:
                    if self.distance < DISTANCE_THRESHOLD:
                        rospy.loginfo(f"Reached marker {self.next_marker}. Rotating to next direction.")
                        self.move(0, 0)
                        self.state = "ROTATE"
                        rospy.sleep(0.5)
                    else:
                        rospy.loginfo("Approaching marker...")
                        self.move(FORWARD_SPEED, FORWARD_SPEED)
                        rospy.sleep(0.5)
                else:
                    rospy.loginfo("Searching for marker...")
                    self.move(ROTATE_SPEED, -ROTATE_SPEED)

            elif self.state == "ROTATE":
                angle = self.angle_to_target()
                if angle is not None:
                    duration = abs(angle) / ROTATE_SPEED
                    direction = 1 if angle > 0 else -1
                    t_start = rospy.Time.now().to_sec()
                    while rospy.Time.now().to_sec() - t_start < duration and not rospy.is_shutdown():
                        self.move(ROTATE_SPEED * direction, -ROTATE_SPEED * direction)
                        rate.sleep()
                    self.move(0, 0)
                    rospy.sleep(0.5)
                    self.state = "MOVE"

            elif self.state == "MOVE":
                rospy.loginfo("Moving to next node...")
                self.move(FORWARD_SPEED, FORWARD_SPEED)
                rospy.sleep(1.5)  # adjust based on expected travel time
                self.move(0, 0)
                rospy.set_param("/navigation_node/advance", True)

                path = rospy.get_param("/navigation_node/planned_path", [])
                if self.next_marker == path[-1]:
                    rospy.loginfo("Final target reached. Exiting.")
                    rospy.signal_shutdown("Goal reached")
                    break

                self.state = "APPROACH"

            rate.sleep()

if __name__ == '__main__':
    try:
        DirectNavController()
    except rospy.ROSInterruptException:
        pass

