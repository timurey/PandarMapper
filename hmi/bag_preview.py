#!/usr/bin/env python3
"""
Downsampled point-cloud preview for the HMI web dashboard.

Reuses the calibration and deskew math from DeskewStudio
(~/Reference/DeskewStudio/deskewstudio/deskew.py) — that project is the
verified source of truth for this rig's mount geometry and encoder
handling, so this module borrows its constants and functions rather than
re-deriving them. What's different here is *bounded* reading: DeskewStudio
is a desktop tool that loads a whole bag into memory for interactive
tuning; this module stops reading as soon as it has seen a couple of full
platform rotations, so a preview stays fast even on a multi-GB bag —
a static scan repeats the same geometry every rotation, so a handful of
them is representative of the whole recording.

Single streaming pass, topic-filtered (skips /imu etc. entirely):
  - Encoder samples (joint_state preferred, angle topic as fallback) are
    tracked with an O(1) incremental-unwrap span so the stop condition is
    cheap to check on every cloud.
  - Point-cloud messages are buffered un-parsed until the target rotation
    span is reached, then decimated down to max_clouds and parsed/deskewed
    only for the kept subset.
  - A hard message/cloud ceiling guards against bags where the platform
    never completes a rotation (stalled motor, wrong topic, etc.).
"""

import os
import sys
import time

import numpy as np

DESKEWSTUDIO_ROOT = os.path.expanduser('~/Reference/DeskewStudio')
if DESKEWSTUDIO_ROOT not in sys.path:
    sys.path.insert(0, DESKEWSTUDIO_ROOT)

from deskewstudio.deskew import (          # noqa: E402
    parse_pointcloud2, deskew_cloud, filter_points, resolve_mcap_path,
    TOPIC_JOINT_STATE, TOPIC_ANGLE, TOPIC_POINTS,
)

DEFAULT_MAX_CLOUDS      = 25
DEFAULT_MAX_POINTS      = 150_000
DEFAULT_TARGET_ROTATIONS = 2.5   # "2-3 full rotations" is plenty for a static scan preview

HARD_MSG_CAP = 40_000    # safety net if the platform never completes a rotation
TWO_PI = 2 * np.pi


class _SpanTracker:
    """O(1)-per-sample running span of an unwrapped angle series."""
    def __init__(self):
        self._last_raw = None
        self._unwrapped = 0.0
        self._min = None
        self._max = None

    def push(self, raw_angle):
        if self._last_raw is None:
            self._unwrapped = raw_angle
        else:
            d = raw_angle - self._last_raw
            d = (d + np.pi) % TWO_PI - np.pi
            self._unwrapped += d
        self._last_raw = raw_angle
        u = self._unwrapped
        self._min = u if self._min is None else min(self._min, u)
        self._max = u if self._max is None else max(self._max, u)

    @property
    def span(self):
        return 0.0 if self._min is None else (self._max - self._min)


def _finalize_encoder_series(state_t, state_a, angle_t, angle_a):
    if len(state_t) >= 2:
        t, a = np.array(state_t, np.float64), np.array(state_a, np.float64)
        src = f'{TOPIC_JOINT_STATE} (jitter-free)'
    elif len(angle_t) >= 2:
        t, a = np.array(angle_t, np.float64), np.array(angle_a, np.float64)
        src = f'{TOPIC_ANGLE} (log_time - jittery)'
    else:
        return None, None, 'no usable encoder topic (need joint_state or angle, 2+ samples)'
    order = np.argsort(t)
    t, a = t[order], a[order]
    a = np.unwrap(a)
    return t, a, src


def build_bag_preview(bag_dir, max_clouds=DEFAULT_MAX_CLOUDS, max_points=DEFAULT_MAX_POINTS,
                       target_rotations=DEFAULT_TARGET_ROTATIONS):
    """Returns a dict with a raw Float32 xyz point buffer and preview stats.

    Raises FileNotFoundError / ValueError on bad input — callers turn those
    into HTTP error responses.
    """
    from mcap_ros2.reader import read_ros2_messages

    t0 = time.monotonic()
    mcap_path = resolve_mcap_path(bag_dir)
    target_span = target_rotations * TWO_PI

    state_t, state_a, angle_t, angle_a = [], [], [], []
    raw_clouds = []   # (cloud_time_ns, ros_msg) — parsed lazily, only for the kept subset
    have_state = False
    state_tracker = _SpanTracker()
    angle_tracker = _SpanTracker()
    hard_cloud_cap = max(max_clouds * 20, 200)

    for seen, m in enumerate(read_ros2_messages(
            str(mcap_path), topics=[TOPIC_POINTS, TOPIC_JOINT_STATE, TOPIC_ANGLE]), 1):
        topic = m.channel.topic
        if topic == TOPIC_JOINT_STATE:
            have_state = True
            h = m.ros_msg.header.stamp
            state_t.append(h.sec * 1_000_000_000 + h.nanosec)
            a = float(m.ros_msg.position[0])
            state_a.append(a)
            state_tracker.push(a)
        elif topic == TOPIC_ANGLE:
            angle_t.append(m.log_time_ns)
            a = np.deg2rad(float(m.ros_msg.data))
            angle_a.append(a)
            angle_tracker.push(a)
        elif topic == TOPIC_POINTS:
            h = m.ros_msg.header.stamp
            raw_clouds.append((h.sec * 1_000_000_000 + h.nanosec, m.ros_msg))

            span = state_tracker.span if have_state else angle_tracker.span
            if span >= target_span and len(raw_clouds) > 0:
                break
            if len(raw_clouds) >= hard_cloud_cap:
                break

        if seen >= HARD_MSG_CAP:
            break

    rotations_captured = (state_tracker.span if have_state else angle_tracker.span) / TWO_PI

    angle_t_arr, angle_a_arr, enc_src = _finalize_encoder_series(state_t, state_a, angle_t, angle_a)
    if angle_t_arr is None:
        raise ValueError(enc_src)

    clouds_seen = len(raw_clouds)
    stride = max(1, clouds_seen // max_clouds)
    selected = raw_clouds[::stride][:max_clouds]
    per_cloud_budget = max(1, max_points // max_clouds)

    blocks = []
    kept = low_cov = 0
    for cloud_ns, msg in selected:
        pts = parse_pointcloud2(msg)
        if len(pts) == 0:
            continue

        clipped = np.clip(
            (pts['timestamp'].astype(np.float64) * 1e9
             if 'timestamp' in pts.dtype.names else np.full(len(pts), float(cloud_ns))),
            angle_t_arr[0], angle_t_arr[-1])
        covered = float(((clipped >= angle_t_arr[0]) & (clipped <= angle_t_arr[-1])).mean())
        if covered < 0.99:
            low_cov += 1

        corrected = deskew_cloud(pts, cloud_ns, angle_t_arr, angle_a_arr)
        corrected = filter_points(corrected.astype(np.float32))
        if len(corrected) == 0:
            continue

        if len(corrected) > per_cloud_budget:
            dec_stride = int(np.ceil(len(corrected) / per_cloud_budget))
            corrected = corrected[::dec_stride]

        blocks.append(corrected)
        kept += 1

    if not blocks:
        raise ValueError('No usable point-cloud data in this bag (empty or unrecognised layout)')

    all_pts = np.concatenate(blocks, axis=0).astype(np.float32)
    return {
        'buf':                 all_pts.tobytes(),
        'points':              int(len(all_pts)),
        'clouds':              kept,
        'clouds_total':        clouds_seen,
        'low_coverage_clouds': low_cov,
        'encoder_source':      enc_src,
        'rotations_captured':  round(rotations_captured, 2),
        'compute_s':           round(time.monotonic() - t0, 2),
    }
