#!/bin/bash
# Phase 2c — STEP 4: launch the Hesai driver and verify a point cloud reaches ROS.
# IMPORTANT: the running sensor services use rmw_fastrtps_cpp, but this shell's
# bashrc defaults to rmw_cyclonedds_cpp. Match fastrtps so `ros2 topic` sees the
# same DDS graph as hmi_bridge / the dashboard.
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
source /home/cave/ros2_ws/install/setup.bash

echo "=== 1) Stop the old velodyne driver if the legacy stack is up ==="
echo "    (hmi_bridge launches velodyne; for a clean first-light test you may"
echo "     want: sudo systemctl stop hmi_bridge   — optional)"
echo
echo "=== 2) Launching Hesai Pandar40P driver (Ctrl-C to stop) ==="
echo "    Publishing /velodyne_points (frame 'velodyne')."
echo "    In ANOTHER terminal, verify with:"
echo "      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "      source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash"
echo "      ros2 topic hz /velodyne_points      # expect ~10 Hz"
echo "      ros2 topic echo --once /velodyne_points --field header   # frame_id: velodyne"
echo "    Then view over wifi via foxglove_bridge, or record from the dashboard."
echo
ros2 launch /home/cave/ros2_ws/hesai_config/hesai_pandar40p.launch.py
