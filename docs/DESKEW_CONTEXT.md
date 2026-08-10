# Offline Deskew — Agent Context (Pandar 40P + rotating platform)

Read this before touching deskew code. A working reference implementation already exists:
**`~/rosbags/offline_deskew_pandar.py`** — copy its logic; do not reinvent it.

⚠️ The older `offline_deskew_guide.md` in this folder is **Velodyne-era and WRONG for this
rig** (uses `rotation_axis=x`, `angle_offset=184`, and reads a `'time'` field that does not
exist on Hesai). Do not follow it.

---

## Hardware / setup facts
- Lidar: **Hesai Pandar 40P**, 10 Hz spin. Published on topic **`/velodyne_points`** (remapped
  from the Hesai driver), frame_id **`velodyne`**. Standard 360° spinning lidar.
- It is mounted on a **rotating platform that spins about the lidar's Y axis** (1:1
  direct-drive from the motor; no gearbox). An encoder measures the platform angle.
- Deskew = undo the platform rotation per-point, putting every point into a common frame.

## Geometry parameters (use exactly these)
```
rotation_axis        = 'y'
angle_offset_deg     = 0.0
rotation_center      = [0.0, 0.0, 0.0]   # concentric mount, tolerances aside
invert_rotation      = False             # flip to True ONLY if output is mirrored
encoder_time_offset_ms = 0.0             # trim ~+2 ms if a small consistent smear remains
```
De-rotation about Y (rot = -platform_angle - deg2rad(angle_offset)):
```
rx = px*cos(rot) + pz*sin(rot)
ry = py
rz = -px*sin(rot) + pz*cos(rot)
```

## TIMESTAMPS — the critical part

**Per-point time field is `timestamp`, NOT `time`.** Hesai PointCloud2 fields are
`x, y, z, intensity, ring, timestamp` (timestamp = FLOAT64).

- `timestamp` is an **ABSOLUTE host-clock time in SECONDS** (driver `use_timestamp_type:1`
  → host `gettimeofday` at packet receive). It is on the **same wall clock as the encoder.**
- To get per-point absolute nanoseconds: **`pt_ns = points['timestamp'] * 1e9`** — used
  directly. **DO NOT add the cloud header stamp** (that's Velodyne's relative-`time`
  convention; adding it here double-counts and everything clips to the buffer edge).
- `header.stamp` == the first point's timestamp (frame start). Frame spans ~100 ms (10 Hz),
  with ~360 distinct azimuth time-steps (~278 µs).
- Known imperfections (acceptable, don't "fix" by changing the math): per-point host jitter
  up to ~2.6 ms; steady lidar↔encoder offset ~2 ms.

**Common failure mode to avoid:** code that does `if 'time' in fields` silently falls back to
applying ONE angle to the whole cloud → ~7–12° of smear per frame. Always branch on
`'timestamp'`.

## ENCODER — which topic to read

Prefer the stamped topic; fall back to the bare one.

| Topic | Type | Time source | Angle |
|---|---|---|---|
| `/rotating_platform/joint_state` | `sensor_msgs/JointState` | **`header.stamp`** (Pi-synced, jitter-free) — PREFER | `position[0]` = **continuous RADIANS** |
| `/rotating_platform/angle` | `std_msgs/Float64` | **bag `log_time`** only (no header; up to ~37 ms record jitter) — fallback | `data` = **degrees 0–360 (wrapped)** |

- `joint_state` only exists in bags recorded **after 2026-05-31** (encoder_node/hmi_bridge
  rebuild). Older bags (e.g. `static_20260530_test`) have only `/angle` → use log_time.
- Build a `(time_ns, angle_rad)` series sorted by time, then `np.unwrap` the angles
  (no-op on the already-continuous joint_state data; required for the wrapped `/angle` data).

## Reading the bag (mcap)
- Bags are **mcap**. Use `mcap_ros2.reader.read_ros2_messages(<path-to-.mcap-file>)` — pass the
  `.mcap` FILE, not the bag directory (resolve `dir/*.mcap`).
- In this mcap_ros2 version, `m.log_time` is a `datetime`; use **`m.log_time_ns`** for int ns.
  Access topic via `m.channel.topic`, message via `m.ros_msg`.
- Parse PointCloud2 with a numpy dtype built from field **offsets + `itemsize=point_step`**
  (the Hesai layout has padding; a naive packed dtype will misread).

## Algorithm
1. Read encoder series (prefer joint_state header.stamp + position rad; else /angle log_time +
   deg→rad). Sort by time, `np.unwrap`.
2. For each cloud: `pt_ns = points['timestamp']*1e9 (+ encoder_time_offset_ns)`, clip to
   `[enc_t[0], enc_t[-1]]`, `np.interp` the platform angle at each point's time.
3. `rot = -platform_angle - deg2rad(angle_offset)`; de-rotate each point about Y (formula
   above). rotation_center is zero so no translate needed.
4. The first 1–2 clouds at bag start usually predate the first encoder sample → low coverage;
   skip or accept them as edge effects.

## Sanity checks the agent SHOULD run before trusting output
- `'timestamp' in fields` is True (not `'time'`).
- Per-point time span per cloud ≈ 100 ms.
- `header.stamp` ≈ min(per-point timestamp).
- Encoder time range brackets the cloud's point times (except the first 1–2 clouds).
- Platform angle **varies several degrees across a single cloud** — proves per-point deskew is
  active, not the single-angle fallback.
