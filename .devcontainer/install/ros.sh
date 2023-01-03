#!/bin/bash
set -e
ROS_VERSION="noetic"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get -y update
sudo apt-get -y upgrade
sudo DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
	ros-${ROS_VERSION}-desktop-full \
	python3-rosdep \
	python3-jinja2 \
	python3-catkin-tools

pip install pymavlink --user

echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc
