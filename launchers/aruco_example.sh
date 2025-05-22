#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun catkin_ws aruco-example.py

# wait for app to end
dt-launchfile-join
