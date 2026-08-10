#!/usr/bin/env python3
"""
Offline deskewing of Hesai Pandar 40P point clouds using platform encoder angles.

Key facts for the Pandar + 1:1 direct-drive setup (see docs/DESKEW_CONTEXT.md
for the full derivation):

  1. Per-point time field is 'timestamp' (FLOAT64), NOT 'time'. Hesai's value is an
     ABSOLUTE host-clock time in seconds (use_timestamp_type:1 → gettimeofday at packet
     receive), already on the same wall clock as the encoder. So we interpolate at
     points['timestamp']*1e9 directly — we do NOT add the cloud header stamp.
  2. rotation_axis = 'y' (platform spins about the Pandar's Y axis), angle_offset_deg = 0,
     rotation_center = [0,0,0] (concentric mount, manufacturing tolerances aside).
  3. Encoder source: PREFER /rotating_platform/joint_state (carries header.stamp = the
     Pi-synced per-sample time). Fall back to /rotating_platform/angle + bag log_time for
     older bags that predate the stamped topic (log_time has tens of ms of record jitter).

Requirements:
    pip install mcap mcap-ros2-support numpy

Usage:
    python offline_deskew_pandar.py <bag_dir_or_mcap> [output.npz] [--max N]
"""

import sys
import numpy as np
from pathlib import Path
from mcap_ros2.reader import read_ros2_messages


# ── Geometry parameters (Pandar 40P, Y-axis platform spin) ─────────────────
ROTATION_AXIS = 'y'
ANGLE_OFFSET_DEG = 0.0
ROTATION_CENTER = np.array([0.0, 0.0, 0.0])
INVERT_ROTATION = False              # flip if the reconstruction comes out mirrored
ENCODER_TIME_OFFSET_MS = 0.0         # +ve treats encoder as earlier; trim residual skew

ANGLE_TOPIC = '/rotating_platform/angle'
STATE_TOPIC = '/rotating_platform/joint_state'
CLOUD_TOPIC = '/velodyne_points'

_ROS_TO_NP = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def parse_pointcloud2(msg):
    """PointCloud2 -> structured numpy array, honouring field offsets and point_step."""
    dt = np.dtype({
        'names':    [f.name for f in msg.fields],
        'formats':  [_ROS_TO_NP[f.datatype] for f in msg.fields],
        'offsets':  [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    return np.frombuffer(msg.data, dtype=dt).copy()


def point_times_ns(points, cloud_time_ns):
    """Per-point absolute time in ns. Hesai 'timestamp' is absolute seconds; VLP 'time'
    is a relative offset from the cloud stamp. Returns None if neither field exists."""
    names = points.dtype.names
    if 'timestamp' in names:                       # Hesai Pandar — absolute seconds
        return points['timestamp'].astype(np.float64) * 1e9
    if 'time' in names:                            # Velodyne — relative seconds
        return cloud_time_ns + points['time'].astype(np.float64) * 1e9
    return None


def derotate(px, py, pz, rot, axis):
    cos_a, sin_a = np.cos(rot), np.sin(rot)
    if axis == 'x':
        return px, py * cos_a - pz * sin_a, py * sin_a + pz * cos_a
    if axis == 'y':
        return px * cos_a + pz * sin_a, py, -px * sin_a + pz * cos_a
    # 'z'
    return px * cos_a - py * sin_a, px * sin_a + py * cos_a, pz


def deskew_cloud(points, cloud_time_ns, angle_times_ns, angles_unwrapped):
    x = points['x'].astype(np.float64)
    y = points['y'].astype(np.float64)
    z = points['z'].astype(np.float64)

    time_offset_ns = ENCODER_TIME_OFFSET_MS * 1e6
    pt_ns = point_times_ns(points, cloud_time_ns)
    if pt_ns is None:                              # no per-point time → one angle / cloud
        t = np.clip(cloud_time_ns + time_offset_ns, angle_times_ns[0], angle_times_ns[-1])
        platform = np.full(len(points), np.interp(t, angle_times_ns, angles_unwrapped))
        covered = 0.0
    else:
        pt_ns = pt_ns + time_offset_ns
        in_range = (pt_ns >= angle_times_ns[0]) & (pt_ns <= angle_times_ns[-1])
        covered = float(in_range.mean())
        pt_ns = np.clip(pt_ns, angle_times_ns[0], angle_times_ns[-1])
        platform = np.interp(pt_ns, angle_times_ns, angles_unwrapped)

    offset_rad = np.deg2rad(ANGLE_OFFSET_DEG)
    rot = (platform if INVERT_ROTATION else -platform) - offset_rad

    # mount + axis map are identity for this rig; rotation_center is zero.
    px = x - ROTATION_CENTER[0]
    py = y - ROTATION_CENTER[1]
    pz = z - ROTATION_CENTER[2]
    rx, ry, rz = derotate(px, py, pz, rot, ROTATION_AXIS)
    rx += ROTATION_CENTER[0]
    ry += ROTATION_CENTER[1]
    rz += ROTATION_CENTER[2]
    return np.column_stack([rx, ry, rz]), covered


def resolve_mcap(bag_path):
    """Accept either a .mcap file or a bag directory containing one."""
    p = Path(bag_path)
    if p.is_dir():
        mcaps = sorted(p.glob('*.mcap'))
        if not mcaps:
            raise SystemExit(f"ERROR: no .mcap file found in {p}")
        return mcaps[0]
    return p


def read_bag(bag_path, max_clouds=None):
    """Returns (angle_times_ns, angles_rad, clouds, source_str)."""
    have_state = have_angle = False
    state_t, state_a, angle_t, angle_a, clouds = [], [], [], [], []

    for m in read_ros2_messages(str(resolve_mcap(bag_path))):
        topic = m.channel.topic
        if topic == STATE_TOPIC:
            have_state = True
            h = m.ros_msg.header.stamp
            state_t.append(h.sec * 1_000_000_000 + h.nanosec)
            state_a.append(float(m.ros_msg.position[0]))          # continuous radians
        elif topic == ANGLE_TOPIC:
            have_angle = True
            angle_t.append(m.log_time_ns)                          # record time (jittery)
            angle_a.append(np.deg2rad(float(m.ros_msg.data)))      # 0..2pi
        elif topic == CLOUD_TOPIC:
            h = m.ros_msg.header.stamp
            clouds.append((h.sec * 1_000_000_000 + h.nanosec, m.ros_msg))
            if max_clouds and len(clouds) >= max_clouds:
                pass  # keep reading for trailing encoder samples; capped below

    # Prefer the stamped topic when present.
    if have_state and len(state_t) >= 2:
        t, a, src = np.array(state_t, np.float64), np.array(state_a, np.float64), \
            f"{STATE_TOPIC} (header.stamp, jitter-free)"
    elif have_angle and len(angle_t) >= 2:
        t, a, src = np.array(angle_t, np.float64), np.array(angle_a, np.float64), \
            f"{ANGLE_TOPIC} (bag log_time — record jitter present; re-record with " \
            f"{STATE_TOPIC} for best results)"
    else:
        raise SystemExit("ERROR: no usable encoder topic (need joint_state or angle)")

    order = np.argsort(t)
    t, a = t[order], a[order]
    a = np.unwrap(a)                                  # no-op on already-continuous data
    if max_clouds:
        clouds = clouds[:max_clouds]
    return t, a, clouds, src


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag = Path(sys.argv[1])
    out = "deskewed.npz"
    max_clouds = None
    rest = sys.argv[2:]
    if '--max' in rest:
        i = rest.index('--max')
        max_clouds = int(rest[i + 1])
        del rest[i:i + 2]
    if rest:
        out = rest[0]

    angle_t, angle_a, clouds, src = read_bag(bag, max_clouds)
    print(f"encoder source : {src}")
    print(f"encoder samples: {len(angle_t)}   clouds: {len(clouds)}")
    print(f"params         : axis={ROTATION_AXIS} offset={ANGLE_OFFSET_DEG}deg "
          f"center={ROTATION_CENTER.tolist()} invert={INVERT_ROTATION} "
          f"enc_offset={ENCODER_TIME_OFFSET_MS}ms")

    all_pts, all_ts, low_cov = [], [], 0
    for i, (cloud_ns, msg) in enumerate(clouds):
        pts = parse_pointcloud2(msg)
        if len(pts) == 0:
            continue
        corrected, covered = deskew_cloud(pts, cloud_ns, angle_t, angle_a)
        if covered < 0.99:
            low_cov += 1
            if low_cov <= 5:
                print(f"  WARN cloud {i}: only {covered*100:.0f}% of points within "
                      f"encoder time range (bag-edge or missing encoder data)")
        all_pts.append(corrected.astype(np.float32))
        all_ts.append(cloud_ns)

    print(f"deskewed {len(all_pts)} clouds ({low_cov} had <99% encoder coverage)")
    np.savez_compressed(
        out,
        points=np.array(all_pts, dtype=object),
        timestamps=np.array(all_ts),
        params=dict(rotation_axis=ROTATION_AXIS, angle_offset_deg=ANGLE_OFFSET_DEG,
                    rotation_center=ROTATION_CENTER.tolist(), invert=INVERT_ROTATION,
                    encoder_time_offset_ms=ENCODER_TIME_OFFSET_MS, encoder_source=src),
    )
    print(f"saved -> {out}")


if __name__ == '__main__':
    main()
