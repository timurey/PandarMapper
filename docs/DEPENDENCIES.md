# Dependencies

Everything in this file is a *dependency* of the scanner — install/clone it
per the instructions here, it is not vendored into this repo.

## OS / platform

- Raspberry Pi 5
- Ubuntu **24.04.3 LTS (Noble)**, 64-bit (arm64)

## ROS2

- **ROS2 Jazzy** — install per the [official docs](https://docs.ros.org/en/jazzy/Installation.html)
- `python3-colcon-common-extensions`

## Lidar driver (not vendored — clone separately)

- [`HesaiTechnology/HesaiLidar_ROS_2.0`](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)
  — pinned to commit `e7e112f0809f0eed5e3c81c55a1a0376474db234` (2026-04-27)
  - Ships its own SDK as a submodule: `HesaiLidar_SDK_2.0` at `v2.0.11-15-g9d5dc4f`
    (`9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168`)
  - Build dep: `libpcap-dev`
  - Clone into `ros2_ws/src/HesaiLidar_ROS_2.0` (with `--recurse-submodules`),
    `colcon build`. The pandar40p correction CSVs it ships are referenced by
    absolute path in `ros2_ws/hesai_config/config_pandar40p.yaml` — update
    those paths after cloning if your workspace isn't at `/home/cave/ros2_ws`.

## IMU driver (own repo, not vendored)

- [`tthom289/ros2_wheeltec_n100_imu`](https://github.com/tthom289/ros2_wheeltec_n100_imu)
  — clone into `ros2_ws/src/`, `colcon build`.

## Motor/encoder firmware (own repo, not vendored)

- Teensy 4.0 firmware — `<TEENSY_FIRMWARE_REPO_URL>` (fill in the actual link).
  Talks to `encoder_bridge` over serial (`/dev/teensy_encoder`, stable name
  via `system/udev/99-teensy-encoder.rules`).

## Python (for `hmi/`)

```
Flask==3.1.3
numpy==1.26.4
```
(`rclpy` etc. come from the ROS2 install above, not pip.)

## Known dropped-not-migrated

- `open_vins` (rpng/open_vins) was cloned during early experimentation but
  never wired into any launch file — not part of the working scanner, not
  included here.
- The original Velodyne VLP-16 driver package was fully replaced by the
  Hesai Pandar 40P (2026-05-30) and is not part of this repo.

## Portability note

A handful of files still have `/home/cave/...` absolute paths baked in
(matches the live rig this was extracted from):

- `hmi/app.py` — `BAG_DIR`, `BAG_TRASH_DIR`
- `hmi/start.sh`
- `ros2_ws/src/hmi_bridge/launch/hmi_bridge.launch.py` — default bag directory
- `ros2_ws/src/hmi_bridge/hmi_bridge/hmi_bridge.py` — `bag_directory` param default
- `ros2_ws/hesai_config/hesai_pandar40p.launch.py`, `04_verify_pandar_ros.sh`
- `ros2_ws/hesai_config/config_pandar40p.yaml` — SDK correction-file paths

If your username or workspace location differs, update these before running.
