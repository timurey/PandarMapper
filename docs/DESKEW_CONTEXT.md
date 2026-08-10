# Offline Deskew (Pandar 40P + rotating platform)

The lidar spins on a platform that itself rotates, so a single scan sweep is
smeared across the platform's motion during that ~100 ms. `tools/offline_deskew_pandar.py`
undoes this after the fact by rotating each point back by the platform angle
at the instant it was captured.

## Hardware facts

- Lidar: Hesai Pandar 40P, 10 Hz spin, published on `/velodyne_points`
  (topic name is just what the driver remap is configured to; not a Velodyne
  device), frame_id `velodyne`.
- Mounted on a platform that spins about the lidar's **Y axis**, 1:1
  direct-drive (no gearbox). An encoder measures platform angle.

## Geometry parameters

```
rotation_axis        = 'y'
angle_offset_deg     = 0.0
rotation_center      = [0.0, 0.0, 0.0]   # concentric mount, tolerances aside
invert_rotation      = False             # flip to True ONLY if output is mirrored
encoder_time_offset_ms = 0.0             # trim ~+2 ms if a small consistent smear remains
```

De-rotation about Y (`rot = -platform_angle - deg2rad(angle_offset)`):
```
rx = px*cos(rot) + pz*sin(rot)
ry = py
rz = -px*sin(rot) + pz*cos(rot)
```

## Timestamps

Per-point time field is `timestamp` (FLOAT64) — an absolute host-clock time
in seconds (driver `use_timestamp_type: 1`, i.e. host `gettimeofday` at
packet receive), on the same wall clock as the encoder.

- Per-point absolute nanoseconds: `pt_ns = points['timestamp'] * 1e9` — used
  directly, do not add the cloud header stamp.
- `header.stamp` == the first point's timestamp (frame start). A frame spans
  ~100 ms at 10 Hz, roughly 360 distinct azimuth time-steps (~278 µs apart).
- Expected jitter: per-point host jitter up to ~2.6 ms; steady lidar↔encoder
  offset ~2 ms.

## Encoder

| Topic | Type | Time source | Angle |
|---|---|---|---|
| `/rotating_platform/joint_state` | `sensor_msgs/JointState` | `header.stamp` (jitter-free) | `position[0]` = continuous radians |
| `/rotating_platform/angle` | `std_msgs/Float64` | bag `log_time` only (no header) | `data` = degrees 0–360 (wrapped) |

Prefer `joint_state`; fall back to `/angle` if it isn't present. Build a
`(time_ns, angle_rad)` series sorted by time and `np.unwrap` the angles
(no-op on the already-continuous `joint_state` data; required for the wrapped
`/angle` data).

## Reading the bag (mcap)

- Use `mcap_ros2.reader.read_ros2_messages(<path-to-.mcap-file>)`, passing
  the `.mcap` file, not the bag directory.
- `m.log_time` is a `datetime`; use `m.log_time_ns` for integer nanoseconds.
  Topic via `m.channel.topic`, message via `m.ros_msg`.
- Parse `PointCloud2` with a numpy dtype built from field offsets +
  `itemsize=point_step` — the Hesai layout has padding, a naively packed
  dtype will misread it.

## Algorithm

1. Read the encoder series (prefer `joint_state`, else `/angle`), sort by
   time, `np.unwrap`.
2. For each cloud: `pt_ns = points['timestamp']*1e9 (+ encoder_time_offset_ns)`,
   clip to `[enc_t[0], enc_t[-1]]`, `np.interp` the platform angle at each
   point's time.
3. `rot = -platform_angle - deg2rad(angle_offset)`; de-rotate each point
   about Y (formula above). `rotation_center` is zero so no translation is
   needed.
4. The first 1–2 clouds in a recording usually predate the first encoder
   sample — low coverage; skip or accept as an edge effect.

## Sanity checks

- Per-point time span per cloud ≈ 100 ms.
- `header.stamp` ≈ min(per-point timestamp).
- Encoder time range brackets the cloud's point times (except the first 1–2 clouds).
- Platform angle varies several degrees across a single cloud — confirms
  per-point deskew is active rather than a single averaged angle.
