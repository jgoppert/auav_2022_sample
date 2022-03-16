#!/bin/bash
export ROS_IP=192.168.123.126
export ROS_MASTER_URI=http://192.168.123.126:11311
source /home/px4vision/catkin/devel/setup.bash
roslaunch auav_2022_sample live.launch
