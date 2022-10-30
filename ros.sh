#!/bin/bash

# # setup ros_tutorials
su tork -c "wstool set -t ~/catkin_ws/src roscpp_tutorials https://github.com/ros/ros_tutorials.git -v $ROS_DISTRO-devel --git -y"

# # update and install
su tork -c 'wstool update -t ~/catkin_ws/src'
su tork -c "rosdep install -y --rosdistro $ROS_DISTRO --from-paths ~/catkin_ws/src --ignore-src"

# # show status
su tork -c wstool info -t ~/catkin_ws/src

# # compile with catkin
su tork -c "(cd ~/catkin_ws; . /opt/ros/$ROS_DISTRO/setup.sh; catkin build)"
