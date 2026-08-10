# ROBIN Reconstruction Usage

Current working config (tested)
- Platform axis: `rotation_axis:=x`
- Angle zero trim: `angle_offset_deg:=184` (encoder reading that should become “level”)
- Optical center offset: start with `rotation_center:="[0.0,0.00947,0.0]"` (0.373 in ≈ 0.00947 m). Flip the sign on the Y term if the offset goes the other way.
- Per-point timing: `use_per_point_time:=true`
- Max RPM guard: `max_rpm:=20` (raise if you rotate faster by hand)
- Teensy baud: `230400`, checksum-framed payload: `$<ts_ms>,<platform_unwrapped_deg>,<platform_rpm>*<CS>\n` (CS = XOR of payload bytes)

Launch commands
- Start Velodyne stack:
  ```bash
  ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
  ```
- Run reconstruction (baseline working setup):
  ```bash
  ros2 launch encoder_bridge robin_reconstruction.launch.py \
    enable_correction:=true use_per_point_time:=true \
    rotation_axis:=x angle_offset_deg:=184 rotation_center:="[0.0,0.00947,0.0]" \
    max_rpm:=20
  ```
  If the 0.00947 m offset hurts instead of helps, try `rotation_center:="[0.0,-0.00947,0.0]"` or remove it.

Rebuild (if you change code)
```bash
cd ~/ros2_ws
colcon build --packages-select encoder_bridge
```

Notes
- Encoder stream uses checksum framing; corrupted lines are dropped.
- If you see permission errors writing build logs, set `COLCON_LOG_PATH=./log`.
- Foxglove bridge: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`

Quick troubleshooting
- Room spins about Z: ensure `rotation_axis:=x` and a sensible `angle_offset_deg`.
- Still smeared: bump `max_rpm` if rotating faster; try small `encoder_time_offset_ms` (e.g., 10–20).
- If encoder stream glitches: drop Teensy send rate to 50 Hz or lower baud (115200) and match on Pi.
