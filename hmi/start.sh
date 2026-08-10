#!/bin/bash
# HMI startup — source ROS2 + workspace, then launch Flask
source /opt/ros/jazzy/setup.bash
source /home/cave/ros2_ws/install/setup.bash
exec python3 /home/cave/hmi/app.py
