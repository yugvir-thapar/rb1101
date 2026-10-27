#!/bin/bash
source install/setup.bash
ros2 launch rb1101_gz rb1101_gazebo.launch.py x:=0.0 y:=0.9 yaw:=0
./kill.sh # kills any lingering Gazebo