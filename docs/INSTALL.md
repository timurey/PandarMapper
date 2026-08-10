# Install

Assumes a fresh Raspberry Pi 5 on Ubuntu 24.04 LTS (arm64) and the hardware
in `docs/HARDWARE_SETUP.md` already wired up.

## 1. ROS2 + system packages

```bash
sudo apt update && sudo apt upgrade -y
# Install ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
sudo apt install -y python3-colcon-common-extensions libpcap-dev
```

## 2. Clone this repo + dependencies

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone <THIS_REPO_URL> pi5-scanner
# Symlink or copy the ROS2 packages into src/
ln -s ~/pi5-scanner/ros2_ws/src/hmi_bridge src/hmi_bridge
ln -s ~/pi5-scanner/ros2_ws/src/encoder_bridge src/encoder_bridge
ln -s ~/pi5-scanner/ros2_ws/hesai_config hesai_config

# Third-party drivers (see docs/DEPENDENCIES.md for pinned commits)
git clone --recurse-submodules \
  https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git src/HesaiLidar_ROS_2.0
git clone https://github.com/tthom289/ros2_wheeltec_n100_imu.git src/ros2_wheeltec_n100_imu

colcon build
source install/setup.bash
```

If your username/paths differ from `/home/cave/...`, fix the paths listed
under "Portability note" in `docs/DEPENDENCIES.md` first.

## 3. Web dashboard

```bash
cd ~
ln -s ~/pi5-scanner/hmi hmi
pip3 install -r ~/pi5-scanner/requirements.txt
mkdir -p ~/rosbags ~/rosbags_trash   # or update BAG_DIR/BAG_TRASH_DIR in hmi/app.py
```

## 4. System wiring (services, network, udev, sysctl)

```bash
# systemd units
sudo cp ~/pi5-scanner/system/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now hmi.service hmi_bridge.service \
  captive-portal.service wifi-mode-selector.service wifi-regdom.service

# netplan (lidar eth0 static IP)
sudo cp ~/pi5-scanner/system/network/60-hesai-eth0.yaml /etc/netplan/
sudo netplan apply

# captive portal DNS override for the field AP
sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
sudo cp ~/pi5-scanner/system/network/dnsmasq-shared.d/captive.conf \
  /etc/NetworkManager/dnsmasq-shared.d/

# sysctl tuning (UDP recv buffer + rp_filter for the lidar's off-subnet source IP)
sudo cp ~/pi5-scanner/system/sysctl.d/*.conf /etc/sysctl.d/
sudo sysctl --system

# udev rule for stable Teensy device name
sudo cp ~/pi5-scanner/system/udev/99-teensy-encoder.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then create the field-AP wifi connection yourself — see
`docs/HARDWARE_SETUP.md` (don't skip picking your own SSID/password).

## 5. Verify

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # services already set this; needed for interactive `ros2 topic` too
ros2 topic hz /velodyne_points   # lidar topic name kept for compat, see README
```

Dashboard: `http://<pi-ip>:3000`
