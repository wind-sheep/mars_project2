#!/bin/bash
export ROS_MASTER_URI=http://192.168.50.203:11311
export ROS_HOSTNAME=192.168.50.77
source devel/setup.bash
timeout 15 rosrun rosbridge_listener coordinate_listener.py
unset ROS_MASTER_URI
unset ROS_HOSTNAME
source devel/setup.bash
rosrun rosbridge_listener arm_control.py
