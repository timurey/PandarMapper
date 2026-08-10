# HMI Bridge

ROS2 node that bridges sensor data and rosbag control to an ESP32-based HMI display for handheld SLAM data collection.

## Features

- **System control** - Start/stop sensors (VLP-16 + IMU) via ESP32
- **Real-time monitoring** of `/velodyne_points` and `/imu` topic rates
- **Rosbag recording control** - Start/stop recording via serial commands
- **System status** - Disk space monitoring, recording duration, sensor health
- **Serial communication** - JSON protocol over USB to ESP32 CYD
- **Standalone testing** - Can run without serial for debugging

## Communication Protocol

### Status Updates (Pi → ESP32)
Sent every 500ms:
```json
{
  "sensors_running": true,
  "lidar_hz": 10.0,
  "imu_hz": 100.0,
  "lidar_ok": true,
  "imu_ok": true,
  "recording": false,
  "disk_gb": 45.2,
  "rec_duration": 0,
  "bag_name": ""
}
```

### Commands (ESP32 → Pi)

**Start Sensors:**
```json
{"cmd": "start_sensors"}
```

**Stop Sensors:**
```json
{"cmd": "stop_sensors"}
```

**Start Recording:**
```json
{"cmd": "start_recording", "name": "my_session"}
```
Or with auto-generated name:
```json
{"cmd": "start_recording"}
```

**Stop Recording:**
```json
{"cmd": "stop_recording"}
```

**Ping Test:**
```json
{"cmd": "ping"}
```
Response: `{"status": "pong"}`

## Typical Workflow

1. **Pi boots** → HMI Bridge auto-starts (via systemd service - optional)
2. **CYD connects** → Shows "READY - Press START"
3. **User presses START** → Sends `{"cmd": "start_sensors"}` → Launches VLP-16 + IMU
4. **CYD displays sensor status** → Shows LiDAR Hz, IMU Hz, disk space
5. **User presses RECORD** → Sends `{"cmd": "start_recording"}` → Starts rosbag
6. **User presses STOP** → Sends `{"cmd": "stop_recording"}` → Stops rosbag
7. **User presses SHUTDOWN** → Sends `{"cmd": "stop_sensors"}` → Stops sensors

## Screen Layout Suggestion for CYD

```
┌─────────────────────────────┐
│  SLAM Data Collection       │
├─────────────────────────────┤
│ LiDAR: 10.0 Hz ✓            │
│ IMU:  100.0 Hz ✓            │
│ Disk:  45.2 GB              │
│                             │
│ [●REC] 00:05:23             │ <- Recording indicator
│ session_20250106_143022     │
│                             │
│  [START SENSORS]            │ <- Big button
│  [START RECORD ]            │
│  [STOP RECORD  ]            │
│  [STOP SENSORS ]            │
└─────────────────────────────┘
```

## Usage

### Testing without ESP32

```bash
source install/setup.bash
ros2 launch hmi_bridge hmi_bridge.launch.py use_serial:=false
```

Or use the test script:
```bash
./test_hmi_bridge.sh
```

### With ESP32 Connected

```bash
source install/setup.bash
ros2 launch hmi_bridge hmi_bridge.launch.py use_serial:=true serial_port:=/dev/ttyUSB0
```

### Manual Testing with Serial Terminal

You can test the serial communication using `screen` or `minicom`:

```bash
# In one terminal - run the bridge
ros2 launch hmi_bridge hmi_bridge.launch.py use_serial:=true serial_port:=/dev/ttyUSB0

# In another terminal - connect to same serial port
screen /dev/ttyUSB0 115200

# Type commands:
{"cmd": "start_recording", "name": "test_session"}
{"cmd": "stop_recording"}
```

## Parameters

- `serial_port` (string, default: `/dev/ttyUSB0`) - Serial port device
- `baud_rate` (int, default: `115200`) - Serial baud rate
- `use_serial` (bool, default: `false`) - Enable serial communication
- `bag_directory` (string, default: `/home/tthom/rosbags`) - Directory for bag files

## Recorded Topics

- `/velodyne_points` - Point cloud data
- `/imu` - IMU data

Bags are recorded with zstd compression to save space.

## File Naming

Auto-generated bag names use format: `slam_session_YYYYMMDD_HHMMSS`

Example: `slam_session_20250612_143022`
