from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file that starts all sensors for SLAM data collection:
    - Velodyne VLP-16 LiDAR (driver + pointcloud)
    - Wheeltec N100 IMU
    - Encoder bridge + point cloud reconstructor (ROBIN reconstruction)

    OAK-D-W stereo camera previously launched here but removed 2026-05-15 —
    project no longer uses the OAK-D path.

    This is meant to be launched by the HMI bridge when user presses START
    """

    return LaunchDescription([
        # LiDAR driver — Hesai Pandar 40P (replaced Velodyne VLP-16 2026-05-30,
        # migration/hesai-web). Publishes /velodyne_points + frame 'velodyne' via
        # the remap in hesai_config/config_pandar40p.yaml, so this package's
        # recorder/monitor and the ROBIN deskew need no topic changes.
        # Old velodyne include preserved in slam_sensors.launch.py.bak.pre-hesai.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/home/cave/ros2_ws/hesai_config/hesai_pandar40p.launch.py'
            )
        ),

        # Launch IMU node
        Node(
            package='wheeltec_n100_imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            respawn=True,
            respawn_delay=5.0,
            parameters=[{
                'serial_port': '/dev/ttyIMU',
                'serial_baud': 921600
            }]
        ),

        # Include ROBIN reconstruction launch file (encoder bridge + point cloud reconstructor)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('encoder_bridge'),
                    'launch',
                    'robin_reconstruction.launch.py'
                ])
            ]),
            launch_arguments={
                'enable_correction': 'true',
                'use_per_point_time': 'true',
                'rotation_axis': 'x',
                'auto_zero_on_start': 'true',
                'angle_offset_deg': '0.0',
                'rotation_center': '[0.0, 0.010163, 0.0]',
                'max_rpm': '60.0',
                # Stable udev symlink — immune to ttyACM number changes
                'serial_port': '/dev/ttyTeensy',
            }.items()
        ),
    ])
