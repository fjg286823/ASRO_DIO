#! /bin/bash

source /home/javens/Workspace/catkin_dio/devel/setup.bash

#set ROSMaster as IP of desktop
export ROS_MASTER_URI=http://192.168.3.2:11311
#set LAN IP of this machine
export ROS_IP=192.168.3.7

cd /home/javens/Workspace/catkin_dio/src/ROGIO-ROS-threads/launch
roslaunch kinect_lab.launch
