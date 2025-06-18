#!/usr/bin/env python3
import rospy
import signal
import yaml
import heapq
import os
import rospkg
from std_msgs.msg import Int32

class GraphPlanner:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            self.graph_data = yaml.safe_load(f)["nodes"]

    def dijkstra(self, start_id, goal_id):
        graph = self.graph_data
        visited = set()
        heap = [(0, start_id, [start_id])]

        while heap:
            cost, current, path = heapq.heappop(heap)
            if current == goal_id:
                return path
            if current in visited:
                continue
            visited.add(current)
            for neighbor in graph[current]["neighbors"]:
                if neighbor not in visited:
                    heapq.heappush(heap, (
                        cost + self._distance(current, neighbor),
                        neighbor,
                        path + [neighbor]
                    ))
        return None

    def _distance(self, node1, node2):
        n1 = self.graph_data[node1]
        n2 = self.graph_data[node2]
        dx = n1["x"] - n2["x"]
        dy = n1["y"] - n2["y"]
        return (dx**2 + dy**2) ** 0.5

class NavigationPlanner:
    def __init__(self):
        rospy.init_node("navigation_node", disable_signals=True)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('duckiebot_ws')  # use your actual package name here
        yaml_path = os.path.join(pkg_path, 'src', 'duckie_aruco_nav', 'config', 'graph.yaml')
        self.graph = GraphPlanner(yaml_path)

        self.start_marker = rospy.get_param("~start_marker", 11)
        self.goal_marker = rospy.get_param("~goal_marker", 9)
        self.path = self.graph.dijkstra(self.start_marker, self.goal_marker)
        self.current_index = 0

        if not self.path:
            rospy.logerr("No path found!")
            return

        rospy.set_param("/navigation_node/planned_path", self.path)
        rospy.loginfo(f"Planned path: {self.path}")
        self.pub = rospy.Publisher("/next_marker_id", Int32, queue_size=1)

        signal.signal(signal.SIGINT, self.shutdown_handler)
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)  # Slightly faster rate
        while not rospy.is_shutdown():
            # Check if the controller has signaled to advance
            if rospy.get_param("/navigation_node/advance", False):
                if self.current_index < len(self.path) - 1:
                    self.current_index += 1
                    rospy.loginfo(f"Advancing to marker {self.path[self.current_index]}")
                    rospy.set_param("/navigation_node/advance", False)
                else:
                    rospy.loginfo("End of path reached. Node will now shutdown.")
                    rospy.signal_shutdown("Navigation complete")
                    break

            # Publish the current target marker
            if self.current_index < len(self.path):
                marker = self.path[self.current_index]
                self.pub.publish(marker)

            rate.sleep()

    def shutdown_handler(self, signum, frame):
        rospy.loginfo("Navigation node shutdown.")
        rospy.signal_shutdown("SIGINT")

if __name__ == '__main__':
    try:
        NavigationPlanner()
    except rospy.ROSInterruptException:
        pass

