#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch both nodes in background
rosrun duckiebot_ws aruco_detector.py &
sleep 2
rosrun duckiebot_ws navigation_node.py &
sleep 2
rosrun duckiebot_ws controller_node.py

# wait for all apps to end
dt-launchfile-join
