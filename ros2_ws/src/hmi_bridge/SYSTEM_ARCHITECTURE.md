# Rotating VLP-16 System Architecture

## Overview

This document describes the system architecture for the rotating Velodyne VLP-16 LiDAR platform running on a Raspberry Pi 5.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           RASPBERRY PI 5                                     │
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐     │
│  │                        SYSTEMD SERVICE                               │     │
│  │                     hmi_bridge.service                               │     │
│  │                                                                       │     │
│  │  Starts on boot → ros2 launch hmi_bridge hmi_bridge.launch.py        │     │
│  └──────────────────────────────┬────────────────────────────────────────┘     │
│                                 │                                              │
│                                 ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐     │
│  │                         HMI BRIDGE NODE                              │     │
│  │                       (hmi_bridge.py)                                │     │
│  │                                                                       │     │
│  │  • Serial communication with ESP32/CYD display                       │     │
│  │  • Monitors topic rates (LiDAR, IMU)                                 │     │
│  │  • Sends status updates to display                                   │     │
│  │  • Receives commands (start/stop sensors, recording)                 │     │
│  │  • Auto-starts sensors 2 seconds after boot                          │     │
│  │                                                                       │     │
│  │  On start_sensors() → subprocess.Popen(slam_sensors.launch.py)       │     │
│  └──────────────────────────────┬────────────────────────────────────────┘     │
│                                 │                                              │
│                                 ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐     │
│  │                     SLAM SENSORS LAUNCH                              │     │
│  │                 (slam_sensors.launch.py)                             │     │
│  │                                                                       │     │
│  │  Launches three subsystems:                                          │     │
│  │                                                                       │     │
│  │  1. Velodyne VLP-16 (velodyne-all-nodes-VLP16-launch.py)            │     │
│  │  2. IMU Node (wheeltec_n100_imu)                                     │     │
│  │  3. ROBIN Reconstruction (robin_reconstruction.launch.py)            │     │
│  └──────────────────────────────┬────────────────────────────────────────┘     │
│                                 │                                              │
│         ┌───────────────────────┼───────────────────────┐                     │
│         ▼                       ▼                       ▼                     │
│  ┌─────────────┐        ┌─────────────┐        ┌─────────────────────┐       │
│  │  VELODYNE   │        │     IMU     │        │ ROBIN RECONSTRUCTION │       │
│  │   VLP-16    │        │    NODE     │        │                      │       │
│  │             │        │             │        │  ┌────────────────┐  │       │
│  │ • Driver    │        │ wheeltec_   │        │  │  encoder_node  │  │       │
│  │ • Transform │        │ n100_imu    │        │  │                │  │       │
│  │ • Laserscan │        │             │        │  │ Reads Teensy   │  │       │
│  │             │        │ Serial:     │        │  │ encoder via    │  │       │
│  │ Ethernet:   │        │ /dev/ttyUSB0│        │  │ /dev/ttyAMA4   │  │       │
│  │ 192.168.1.x │        │ @ 921600    │        │  └───────┬────────┘  │       │
│  └──────┬──────┘        └──────┬──────┘        │          │           │       │
│         │                      │               │          ▼           │       │
│         │                      │               │  ┌────────────────┐  │       │
│         │                      │               │  │  point_cloud_  │  │       │
│         │                      │               │  │  reconstructor │  │       │
│         │                      │               │  │                │  │       │
│         │                      │               │  │ Corrects point │  │       │
│         │                      │               │  │ cloud for      │  │       │
│         │                      │               │  │ platform       │  │       │
│         │                      │               │  │ rotation       │  │       │
│         │                      │               │  └───────┬────────┘  │       │
│         │                      │               └──────────┼───────────┘       │
│         │                      │                          │                   │
│         ▼                      ▼                          ▼                   │
│  ┌─────────────────────────────────────────────────────────────────────┐     │
│  │                          ROS 2 TOPICS                                │     │
│  │                                                                       │     │
│  │  /velodyne_points           ← Raw point cloud from VLP-16            │     │
│  │  /velodyne_points_corrected ← Motion-corrected point cloud           │     │
│  │  /velodyne_packets          ← Raw UDP packets                        │     │
│  │  /scan                      ← 2D laser scan                          │     │
│  │  /imu                       ← IMU data (accel, gyro, orientation)    │     │
│  │  /imu_trueEast              ← True east corrected IMU                │     │
│  │  /rotating_platform/angle   ← Platform angle from encoder            │     │
│  │  /rotating_platform/joint_state ← Joint state with velocity          │     │
│  └─────────────────────────────────────────────────────────────────────┘     │
│                                                                               │
└─────────────────────────────────────────────────────────────────────────────┘


                    EXTERNAL HARDWARE CONNECTIONS

┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   VELODYNE      │     │   WHEELTEC      │     │    TEENSY       │
│   VLP-16        │     │   N100 IMU      │     │   ENCODER       │
│                 │     │                 │     │                 │
│  Ethernet       │     │  USB Serial     │     │  UART Serial    │
│  192.168.1.201  │     │  /dev/ttyUSB0   │     │  /dev/ttyAMA4   │
│                 │     │  921600 baud    │     │  230400 baud    │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         │    ┌──────────────────┼───────────────────────┘
         │    │                  │
         ▼    ▼                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                      RASPBERRY PI 5                              │
│                                                                   │
│  ETH0: 192.168.1.x (Velodyne network)                           │
│  WLAN0: WiFi for SSH access                                      │
│  USB: /dev/ttyUSB0 (IMU)                                         │
│  GPIO UART: /dev/ttyAMA0 (CYD), /dev/ttyAMA4 (Encoder)          │
└─────────────────────────────────────────────────────────────────┘
         │
         │ Serial UART /dev/ttyAMA0 @ 115200
         ▼
┌─────────────────┐
│   ESP32 CYD     │
│   DISPLAY       │
│                 │
│  Touch display  │
│  Start/Stop     │
│  Record control │
│  Status display │
└─────────────────┘
```

## Boot Sequence

1. **System Boot** → systemd starts `hmi_bridge.service`
2. **hmi_bridge.service** → `ros2 launch hmi_bridge hmi_bridge.launch.py`
3. **hmi_bridge.launch.py** → Starts `hmi_bridge` node
4. **hmi_bridge node** → Waits 2 seconds, then auto-starts sensors
5. **start_sensors()** → Spawns subprocess: `ros2 launch slam_sensors.launch.py`
6. **slam_sensors.launch.py** → Launches:
   - Velodyne driver + transform + laserscan
   - IMU node
   - Encoder bridge + Point cloud reconstructor

## Key Files

| File | Purpose |
|------|---------|
| `/etc/systemd/system/hmi_bridge.service` | Systemd service started on boot |
| `hmi_bridge/launch/hmi_bridge.launch.py` | Launches HMI bridge node |
| `hmi_bridge/launch/slam_sensors.launch.py` | Launches all sensors |
| `hmi_bridge/hmi_bridge/hmi_bridge.py` | Main HMI bridge node |
| `encoder_bridge/launch/robin_reconstruction.launch.py` | Encoder + reconstructor |

## ROS 2 Nodes

| Node | Package | Purpose |
|------|---------|---------|
| `hmi_bridge` | hmi_bridge | Serial comm with display, system control |
| `velodyne_driver_node` | velodyne_driver | VLP-16 UDP packet receiver |
| `velodyne_transform_node` | velodyne_pointcloud | Converts packets → PointCloud2 |
| `velodyne_laserscan_node` | velodyne_laserscan | Generates 2D laser scan |
| `imu_node` | wheeltec_n100_imu | Reads IMU data |
| `encoder_node` | encoder_bridge | Reads platform angle from Teensy |
| `point_cloud_reconstructor` | encoder_bridge | Corrects point cloud for rotation |

## Serial Ports

| Port | Device | Baud Rate | Purpose |
|------|--------|-----------|---------|
| `/dev/ttyAMA0` | ESP32 CYD | 115200 | HMI display communication |
| `/dev/ttyUSB0` | Wheeltec IMU | 921600 | IMU data |
| `/dev/ttyAMA4` | Teensy | 230400 | Platform encoder |

## Network

| Interface | IP | Purpose |
|-----------|-----|---------|
| `eth0` | 192.168.1.x | Velodyne VLP-16 (LiDAR at 192.168.1.201) |
| `wlan0` | DHCP | SSH access, Foxglove bridge |

## Commands

```bash
# Check service status
sudo systemctl status hmi_bridge

# View logs
journalctl -u hmi_bridge -f

# Restart service
sudo systemctl restart hmi_bridge

# List running nodes
ros2 node list

# Check topic rates
ros2 topic hz /velodyne_points_corrected
ros2 topic hz /imu

# Manual foxglove bridge (not auto-started)
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Rosbag Recording

Recording is controlled via the CYD display or serial commands:

- Records to: `/home/tthom/rosbags/`
- Topics recorded: `/velodyne_points_corrected`, `/imu`
- Compression: zstd

## Data Flow

```
Velodyne VLP-16 (Ethernet)
        │
        ▼
velodyne_driver_node
        │
        ▼ /velodyne_packets
        │
        ▼
velodyne_transform_node
        │
        ▼ /velodyne_points (raw)
        │
        ├──────────────────────────────────┐
        │                                  │
        ▼                                  ▼
velodyne_laserscan_node          point_cloud_reconstructor
        │                                  │
        ▼ /scan                            │ ← /rotating_platform/joint_state
                                           │     (from encoder_node)
                                           ▼
                               /velodyne_points_corrected
                                           │
                                           ▼
                                    rosbag / foxglove
```
