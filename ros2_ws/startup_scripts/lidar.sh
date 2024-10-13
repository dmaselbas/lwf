#!/usr/bin/env sh

cd /root/ros2_ws || exit
. /opt/ros/humble/setup.sh
. install/setup.sh
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyS7
