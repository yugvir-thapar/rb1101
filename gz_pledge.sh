#!/bin/bash
source install/setup.bash
# Bottom-right of the maze, facing down (-y): 2 robot lengths from the bottom wall, 1 from the right wall
# (original start: x:=0.0 y:=0.9 yaw:=3.14159)
ros2 launch rb1101_gz rb1101_gazebo.launch.py x:=2.69 y:=0.01 yaw:=-1.5708
./kill.sh # kills any lingering Gazebo