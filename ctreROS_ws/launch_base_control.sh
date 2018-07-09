#!/bin/bash

source ~/Documents/CTRE/ROS_CTRE/ctreROS_ws/devel/setup.bash

sudo ~/Documents/ubuntu_setup/linux_settings/bash_scripts/canableStart.sh
roslaunch ros_control_boilerplate diff_drive_test.launch interface:=can0 &
sleep 10
roslaunch ros_control_boilerplate teleop_twist_joy.launch
