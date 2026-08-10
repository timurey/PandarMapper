#!/usr/bin/env python3
"""
Hesai Pandar 40P bring-up launch (migration/hesai-web).

Replaces the Velodyne VLP-16 driver. Publishes the cloud on /velodyne_points
with frame_id 'velodyne' (see config_pandar40p.yaml) so every downstream
consumer — hmi/app.py recorder, hmi_bridge, ROBIN deskew, TF — keeps working
unchanged.

Run standalone:
    ros2 launch /home/cave/ros2_ws/hesai_config/hesai_pandar40p.launch.py

Eventually included by slam_sensors.launch.py in place of the velodyne include.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = '/home/cave/ros2_ws/hesai_config/config_pandar40p.yaml'


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='hesai_ros_driver',
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver_node',
            output='screen',
            parameters=[{'config_path': CONFIG}],
        )
    ])
