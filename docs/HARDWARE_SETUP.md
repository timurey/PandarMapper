# Hardware Setup

## Components

- Raspberry Pi 5
- Hesai Pandar 40P lidar (Ethernet/UDP)
- Teensy 4.0 — motor controller + rotating-platform encoder (firmware in its
  own repo, see `docs/DEPENDENCIES.md`)
- Rotating platform: 1:1 direct drive, Pandar mounted on the motor shaft
  (no gearbox)

## Networking

Two separate networks on the Pi:

- **`eth0` → lidar**, isolated static subnet `192.168.3.100/24`
  (`system/network/60-hesai-eth0.yaml`). The Pandar ships on `192.168.1.x`
  by default; keeping it off the wifi subnet avoids collisions.
- **`wlan0` → dashboard access**, either:
  - the home wifi network (client mode), or
  - the rig's own field access point when no known network is in range
    (`system/sbin/wifi-mode-selector.sh`, run at boot via
    `system/systemd/wifi-mode-selector.service`). Connecting to the field AP
    lands you on a captive portal (`hmi/portal.py`,
    `system/systemd/captive-portal.service`) that links to the dashboard.

Create the AP connection yourself (pick your own SSID/password — don't reuse
someone else's example):

```bash
nmcli connection add type wifi ifname wlan0 con-name hotspot autoconnect no \
  ssid "<your-ssid>" mode ap
nmcli connection modify hotspot 802-11-wireless-security.key-mgmt wpa-psk \
  802-11-wireless-security.psk "<your-password>"
nmcli connection modify hotspot ipv4.method shared
```

## Rotating platform geometry

The platform spins about the Pandar's **Y axis** (Hesai datasheet convention:
X=red, Y=yellow, Z=green — not the ROS/Foxglove R=X/G=Y/B=Z convention).
This matters for the offline deskew tool (`tools/offline_deskew_pandar.py`) —
see `docs/DESKEW_CONTEXT.md` for the full derivation.

## No RTC

There's no battery-backed RTC — the Pi's clock is NTP-only. The lidar and
encoder are both timestamped on the system wall clock
(`use_timestamp_type: 1`), so an NTP step mid-recording would corrupt both
timelines. Not currently mitigated; worth confirming `timedatectl` shows
`synchronized: yes` before a real recording.
