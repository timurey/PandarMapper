# Pi5 Scanner

A Raspberry Pi 5 rig that spins a Hesai Pandar 40P lidar on a rotating platform
(driven by a Teensy-controlled motor + encoder) to build 3D scans, with a
wifi-reachable web dashboard for control and recording. ROS2 underneath.

## Architecture

```
                    ┌─────────────────────┐
   Hesai Pandar 40P │   HesaiLidar_ROS_2.0  │  (third-party driver,
   (Ethernet/UDP) ──┤   driver node         │   not vendored — see
                    └──────────┬───────────┘   docs/DEPENDENCIES.md)
                               │ /velodyne_points (remapped topic name,
                               │  kept for zero-churn compat)
   Teensy 4.0 ──serial──┐      │
   (motor + encoder,    │      │
    firmware is its own │      │
    repo — see below)   ▼      ▼
                    ┌─────────────────────┐
                    │  ros2_ws/src/        │
                    │  encoder_bridge      │  encoder_node, point_cloud_reconstructor
                    │  hmi_bridge          │  sensor orchestration + rosbag2 recorder
                    └──────────┬───────────┘
                               │ in-process rclpy node, /hmi_bridge/cmd
                               ▼
                    ┌─────────────────────┐
                    │  hmi/app.py          │  Flask dashboard, binds 0.0.0.0:3000
                    │  hmi/portal.py       │  captive-portal landing page (field AP)
                    └─────────────────────┘
                               ▲
                     wifi (home network or
                     the rig's own field AP)
```

`hmi_bridge` is the active backend: it owns the rosbag2 recorder and launches
all sensors. `hmi/app.py` is the browser-facing control surface — it does not
record directly, it sends commands to `hmi_bridge` over ROS2.

Offline processing (deskewing the point cloud using the encoder angle) happens
after the fact with `tools/offline_deskew_pandar.py` — see `docs/DESKEW_CONTEXT.md`.

## Repo layout

| Path | What it is |
|---|---|
| `ros2_ws/src/hmi_bridge/` | ROS2 pkg — sensor launch orchestration, rosbag2 recorder |
| `ros2_ws/src/encoder_bridge/` | ROS2 pkg — Teensy encoder bridge, point cloud reconstructor |
| `ros2_ws/hesai_config/` | Pandar 40P launch file, config, and bring-up scripts |
| `hmi/` | Flask web dashboard + captive-portal landing page |
| `tools/` | Offline processing (deskew) |
| `system/` | systemd units, netplan, sysctl, udev rules — everything the OS needs wired up |
| `docs/` | Dependency versions, hardware setup, install steps |

## What's *not* in this repo

- **Third-party drivers** (Hesai lidar driver + SDK, the IMU driver) — linked,
  not vendored. See `docs/DEPENDENCIES.md` for exact pinned commits.
- **Teensy firmware** — separate public repo: `<TEENSY_FIRMWARE_REPO_URL>`
- **ROS2 / Ubuntu itself** — installed per upstream instructions, see `docs/DEPENDENCIES.md`.
- **Recorded bag data** — that's the rig's output, not the software.

## Quick start

See `docs/INSTALL.md` for the full bring-up sequence on a bare Pi 5.

## License

MIT — see `LICENSE`. (Third-party components under `docs/DEPENDENCIES.md` keep
their own licenses.)
