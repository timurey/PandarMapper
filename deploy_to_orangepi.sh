#!/bin/bash
# Deploy PandarMapper web HMI to Orange Pi 5 Plus
# Run from this directory: bash deploy_to_orangepi.sh
set -e

PI=openclaw@192.168.1.108
ROS_WS=~/ros2_ws
HMI_DIR=~/hmi

echo "=== Deploying hmi_bridge ROS2 package ==="
ssh $PI "mkdir -p $ROS_WS/src/hmi_bridge"
scp -r ros2_ws/src/hmi_bridge/ $PI:$ROS_WS/src/

echo "=== Building hmi_bridge ==="
ssh $PI "source /opt/ros/jazzy/setup.bash && cd $ROS_WS && colcon build --packages-select hmi_bridge"

echo "=== Deploying Flask HMI ==="
ssh $PI "mkdir -p $HMI_DIR"
scp -r hmi/ $PI:$HMI_DIR/
ssh $PI "chmod +x $HMI_DIR/start.sh"

echo "=== Creating bags directory ==="
ssh $PI "mkdir -p ~/bags ~/bags_trash"

echo "=== Installing systemd services ==="
scp system/systemd/hmi_bridge.service system/systemd/hmi.service $PI:/tmp/
ssh $PI "sudo cp /tmp/hmi_bridge.service /tmp/hmi.service /etc/systemd/system/ && sudo systemctl daemon-reload"

echo "=== Enabling services ==="
ssh $PI "sudo systemctl enable hmi_bridge.service hmi.service"

echo ""
echo "=== Done. To start: ==="
echo "  ssh $PI 'sudo systemctl start hmi_bridge && sudo systemctl start hmi'"
echo "  Web HMI: http://192.168.1.108:3000"
