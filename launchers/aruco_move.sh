#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch both nodes in background
rosrun catkin_ws aruco_detector.py &
rosrun catkin_ws aruco_movement_controller.py &

# wait for all apps to end
dt-launchfile-join

