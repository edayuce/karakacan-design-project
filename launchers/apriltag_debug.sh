#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun catkin_ws apriltag_debug.py

# wait for app to end
dt-launchfile-join
