#!/usr/bin/env python3
"""
ROS2 HMI Backend — Flask server for Pi-5 kiosk display.
Monitors topic Hz via background ros2 topic hz processes,
tracks disk usage, and controls bag recording.
"""

import subprocess
import threading
import re
import shutil
import os
import time
import json
import psutil
import serial
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String as StringMsg
from flask import Flask, render_template, jsonify, request
from datetime import datetime, timedelta

app = Flask(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

TOPICS = {
    # 'Velodyne': '/velodyne_points',  # disabled — see commit history; Hz now comes
    #                                  # from hmi_bridge in-process, not subprocess.
    'IMU':       '/imu',
    'Encoder':   '/rotating_platform/angle',
    # OAK topics removed 2026-05-15 — project no longer uses the OAK-D path.
}

# Topics recorded per mode
RECORD_TOPICS = {
    'static': [
        '/velodyne_points',
        '/rotating_platform/angle',
        '/rotating_platform/velocity',
        '/rotating_platform/joint_state',   # stamped angle for offline per-point deskew
    ],
    'slam': [
        '/velodyne_points',
        '/rotating_platform/angle',
        '/rotating_platform/velocity',
        '/rotating_platform/joint_state',   # stamped angle for offline per-point deskew
        '/imu',
    ],
}

# Which topic cards are active (full opacity) per mode
TOPIC_MODES = {
    'Velodyne':  ['static', 'slam'],
    'IMU':       ['slam'],
    'Encoder':   ['static', 'slam'],
}

# (ok_min_hz, warn_min_hz) — below warn = red, between = yellow, above ok = green
THRESHOLDS = {
    'Velodyne':  (8,   1),
    'IMU':       (100, 20),
    'Encoder':   (150, 50),
}

BAG_DIR       = '/home/cave/rosbags'
BAG_TRASH_DIR = '/home/cave/rosbags_trash'  # discarded bags moved here, never rm -rf'd
ROS_SETUP    = '/opt/ros/jazzy/setup.bash'
WS_SETUP     = os.path.expanduser('~/ros2_ws/install/setup.bash')
HZ_WINDOW    = 5   # samples for ros2 topic hz rolling window
DEAD_AFTER   = 4.0 # seconds with no update → mark as dead
TEENSY_PORT  = '/dev/ttyTeensy'
TEENSY_BAUD  = 115200


# ── Hz Monitor ───────────────────────────────────────────────────────────────

class HzMonitor:
    """
    Spawns one `ros2 topic hz` subprocess per topic in a background thread.
    Continuously parses output and stores the latest value.
    If a process dies (topic gone), restarts after a short delay.
    """

    def __init__(self, topics: dict):
        self._lock = threading.Lock()
        self._hz: dict[str, float | None] = {name: None for name in topics}
        self._last_update: dict[str, float] = {name: 0.0 for name in topics}
        for name, topic in topics.items():
            t = threading.Thread(target=self._monitor, args=(name, topic), daemon=True)
            t.start()

    def _monitor(self, name: str, topic: str):
        pattern = re.compile(r'average rate:\s*([\d.]+)')
        while True:
            try:
                proc = subprocess.Popen(
                    f'source {ROS_SETUP} && source {WS_SETUP} && '
                    f'ros2 topic hz --window {HZ_WINDOW} {topic}',
                    shell=True, executable='/bin/bash',
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True
                )
                for line in proc.stdout:
                    m = pattern.search(line)
                    if m:
                        with self._lock:
                            self._hz[name] = round(float(m.group(1)), 1)
                            self._last_update[name] = time.monotonic()
                proc.wait()
            except Exception:
                pass
            # Process ended — topic likely stopped; brief pause then retry
            with self._lock:
                self._hz[name] = 0.0
            time.sleep(3)

    def get(self) -> dict:
        now = time.monotonic()
        with self._lock:
            result = {}
            for name, hz in self._hz.items():
                # If we haven't received an update in a while, treat as dead
                if hz is not None and hz > 0 and (now - self._last_update[name]) > DEAD_AFTER:
                    result[name] = 0.0
                else:
                    result[name] = hz
            return result


# ── Angle Monitor ────────────────────────────────────────────────────────────

class AngleMonitor:
    """
    Streams /rotating_platform/angle via `ros2 topic echo` and keeps
    the latest Float64 value. Falls back to None when topic is dead.
    """
    def __init__(self, topic: str = '/rotating_platform/angle'):
        self._topic = topic
        self._angle: float | None = None
        self._last  = 0.0
        self._lock  = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        pattern = re.compile(r'^data:\s*([\d.eE+\-]+)')
        while True:
            try:
                proc = subprocess.Popen(
                    f'source {ROS_SETUP} && source {WS_SETUP} && '
                    f'ros2 topic echo --no-daemon {self._topic}',
                    shell=True, executable='/bin/bash',
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True
                )
                for line in proc.stdout:
                    m = pattern.match(line.strip())
                    if m:
                        with self._lock:
                            self._angle = round(float(m.group(1)), 1)
                            self._last  = time.monotonic()
                proc.wait()
            except Exception:
                pass
            with self._lock:
                self._angle = None
            time.sleep(3)

    def get(self) -> float | None:
        with self._lock:
            if self._angle is None:
                return None
            if time.monotonic() - self._last > 4.0:
                return None
            return self._angle


# ── Velocity Monitor ─────────────────────────────────────────────────────────

class VelocityMonitor:
    """
    Streams /rotating_platform/velocity via `ros2 topic echo` and keeps
    the latest Float64 value. Falls back to None when topic is dead.
    """
    def __init__(self, topic: str = '/rotating_platform/velocity'):
        self._topic = topic
        self._rpm: float | None = None
        self._last  = 0.0
        self._lock  = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        pattern = re.compile(r'^data:\s*([\d.eE+\-]+)')
        while True:
            try:
                proc = subprocess.Popen(
                    f'source {ROS_SETUP} && source {WS_SETUP} && '
                    f'ros2 topic echo --no-daemon {self._topic}',
                    shell=True, executable='/bin/bash',
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True
                )
                for line in proc.stdout:
                    m = pattern.match(line.strip())
                    if m:
                        with self._lock:
                            self._rpm = round(float(m.group(1)), 1)
                            self._last = time.monotonic()
                proc.wait()
            except Exception:
                pass
            with self._lock:
                self._rpm = None
            time.sleep(3)

    def get(self) -> float | None:
        with self._lock:
            if self._rpm is None:
                return None
            if time.monotonic() - self._last > 4.0:
                return None
            return self._rpm


# ── ROS coordinator (rclpy node living in this Flask process) ────────────────
#
# Replaces the previous pattern of spawning `ros2 bag record` as a subprocess.
# Instead we publish a JSON command on /hmi_bridge/cmd; the hmi_bridge node owns
# the in-process rosbag2 SequentialWriter and writes from its own subscription
# callbacks. One persistent subscriber per topic on the DDS bus, no rosbag2
# subscribe/unsubscribe cycles -- which is what was causing the empty-cloud
# burst regression.

class ROSCoordinator:
    """Tiny in-process rclpy node + executor running in a daemon thread.
    Provides synchronous request/response over the /hmi_bridge/cmd <-> /response
    String topics so Flask handlers can call hmi_bridge cleanly."""

    def __init__(self):
        self._lock = threading.Lock()
        # Pending-action -> {'event': Event, 'result': dict|None}
        self._pending: dict[str, dict] = {}
        self._node = None
        self._executor = None
        # Latest full status JSON from hmi_bridge (lidar/imu/encoder Hz etc.).
        # hmi_bridge owns the one efficient PointCloud2 subscriber and publishes
        # the computed status on /hmi_bridge/status ~2 Hz; we just cache it.
        self._status: dict = {}
        self._status_time: float = 0.0

        try:
            rclpy.init()
        except Exception:
            # Already initialized — fine, just continue.
            pass
        self._node = rclpy.create_node('hmi_app_client')
        self._cmd_pub = self._node.create_publisher(
            StringMsg, '/hmi_bridge/cmd', 10
        )
        self._resp_sub = self._node.create_subscription(
            StringMsg, '/hmi_bridge/response', self._on_response, 10
        )
        self._status_sub = self._node.create_subscription(
            StringMsg, '/hmi_bridge/status', self._on_status, 10
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def _on_status(self, msg):
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._status = s
            self._status_time = time.monotonic()

    def get_status(self) -> dict:
        """Latest hmi_bridge status, or {} if stale (>3s old) / never received."""
        with self._lock:
            if self._status and (time.monotonic() - self._status_time) < 3.0:
                return dict(self._status)
            return {}

    def _on_response(self, msg):
        try:
            r = json.loads(msg.data)
        except Exception:
            return
        action = r.get('action')
        with self._lock:
            slot = self._pending.get(action)
            if slot is not None:
                slot['result'] = r
                slot['event'].set()

    def call(self, action: str, timeout: float = 5.0, **kwargs) -> dict | None:
        """Publish a command and block until the matching response arrives or
        timeout elapses. Returns the response dict, or None on timeout."""
        event = threading.Event()
        with self._lock:
            self._pending[action] = {'event': event, 'result': None}
        m = StringMsg()
        m.data = json.dumps({'action': action, **kwargs})
        self._cmd_pub.publish(m)
        event.wait(timeout)
        with self._lock:
            slot = self._pending.pop(action, None)
        return (slot or {}).get('result')


ros_coord: ROSCoordinator | None = None  # initialized at module bottom


# ── Bag Recorder ─────────────────────────────────────────────────────────────

class BagRecorder:
    """Thin wrapper that delegates start/stop to hmi_bridge over /hmi_bridge/cmd.
    Keeps the Flask-level state (recording flag, pending bag for the verify
    modal, rec_start for the elapsed timer) but never spawns a subprocess."""

    def __init__(self):
        self._lock        = threading.Lock()
        self.recording    = False
        self.bag_name     = None
        self.mode         = 'static'
        self._rec_start   = None
        # Pending finalization: bag has stopped but user hasn't saved/discarded yet.
        # Drives the post-stop modal in the HMI; cleared on finalize/discard.
        self.pending_bag  = None
        self.pending_mode = None
        os.makedirs(BAG_DIR, exist_ok=True)
        os.makedirs(BAG_TRASH_DIR, exist_ok=True)

    def start(self, mode: str = 'static'):
        with self._lock:
            if self.recording:
                return False, 'Already recording'
            if mode not in RECORD_TOPICS:
                mode = 'static'
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            proposed_name = f'{mode}_{ts}'

            if ros_coord is None:
                return False, 'ROS coordinator not initialized'
            resp = ros_coord.call(
                'start_record', timeout=5.0,
                mode=mode, name=proposed_name
            )
            if resp is None:
                return False, 'No response from hmi_bridge (timeout)'
            if not resp.get('ok'):
                return False, resp.get('msg', 'start_record failed')

            self.bag_name   = resp.get('name', proposed_name)
            self.mode       = mode
            self.recording  = True
            self._rec_start = time.monotonic()
            return True, self.bag_name

    def stop(self):
        with self._lock:
            if not self.recording:
                return False, 'Not recording'
            name = self.bag_name
            mode = self.mode
            if ros_coord is not None:
                resp = ros_coord.call('stop_record', timeout=5.0)
                if resp and resp.get('name'):
                    name = resp['name']
            # Even if response was bad, move out of recording state — hmi_bridge
            # will have closed the writer or crashed; either way Flask is no
            # longer recording from its perspective.
            self.recording    = False
            self._rec_start   = None
            self.pending_bag  = name
            self.pending_mode = mode
            self.bag_name     = None
            return True, name

    def clear_pending(self):
        with self._lock:
            self.pending_bag  = None
            self.pending_mode = None


# ── Disk ─────────────────────────────────────────────────────────────────────

def get_disk() -> dict:
    usage = shutil.disk_usage('/')
    return {
        'total_gb': round(usage.total / 1e9, 1),
        'used_gb':  round(usage.used  / 1e9, 1),
        'free_gb':  round(usage.free  / 1e9, 1),
        'pct':      round(usage.used  / usage.total * 100, 1),
    }


# ── System Diagnostics ────────────────────────────────────────────────────────

def get_cpu_temp() -> float | None:
    """Read CPU temperature from sysfs (works on Pi without vcgencmd)."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None


def get_throttle() -> dict:
    """
    Parse vcgencmd get_throttled bitmask.
    Bits 0-3: current flags. Bits 16-19: ever-occurred flags (since boot).
    """
    try:
        result = subprocess.run(
            ['sudo', 'vcgencmd', 'get_throttled'],
            capture_output=True, text=True, timeout=2
        )
        val = int(result.stdout.strip().split('=')[1], 16)
        current = {
            'undervolt':  bool(val & 0x1),
            'freq_cap':   bool(val & 0x2),
            'throttled':  bool(val & 0x4),
            'soft_temp':  bool(val & 0x8),
        }
        history = {
            'undervolt':  bool(val & 0x10000),
            'freq_cap':   bool(val & 0x20000),
            'throttled':  bool(val & 0x40000),
            'soft_temp':  bool(val & 0x80000),
        }
        ok = val == 0
        return {'ok': ok, 'current': current, 'history': history, 'raw': hex(val)}
    except Exception:
        return {'ok': None, 'current': {}, 'history': {}, 'raw': None}


def get_system() -> dict:
    cpu_pct   = psutil.cpu_percent(percpu=True)
    cpu_freq  = psutil.cpu_freq()
    mem       = psutil.virtual_memory()
    boot_time = psutil.boot_time()
    uptime_s  = int(time.time() - boot_time)
    uptime    = str(timedelta(seconds=uptime_s))

    return {
        'cpu_pct':    [round(p, 1) for p in cpu_pct],
        'cpu_avg':    round(sum(cpu_pct) / len(cpu_pct), 1),
        'cpu_freq_mhz': round(cpu_freq.current) if cpu_freq else None,
        'cpu_freq_max': round(cpu_freq.max)     if cpu_freq else None,
        'cpu_temp':   get_cpu_temp(),
        'ram_used_gb':  round(mem.used  / 1e9, 2),
        'ram_total_gb': round(mem.total / 1e9, 2),
        'ram_pct':    round(mem.percent, 1),
        'uptime':     uptime,
        'throttle':   get_throttle(),
    }


# ── Init ─────────────────────────────────────────────────────────────────────

monitor   = HzMonitor(TOPICS)
angle_mon = AngleMonitor()
vel_mon   = VelocityMonitor()
ros_coord = ROSCoordinator()
recorder  = BagRecorder()


# ── Bag Manager ──────────────────────────────────────────────────────────────

def dir_size_bytes(path: str) -> int:
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, f))
            except OSError:
                pass
    return total


def fmt_size(n: int) -> str:
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024:
            return f'{n:.1f} {unit}'
        n /= 1024
    return f'{n:.1f} TB'


def get_bags() -> list[dict]:
    bags = []
    if not os.path.isdir(BAG_DIR):
        return bags
    for name in sorted(os.listdir(BAG_DIR), reverse=True):
        full = os.path.join(BAG_DIR, name)
        if not os.path.isdir(full):
            continue
        # Skip non-bag items (scripts, docs etc.)
        if not any(f.endswith(('.db3', '.mcap', 'metadata.yaml'))
                   for _, _, files in os.walk(full) for f in files):
            continue
        stat    = os.stat(full)
        size_b  = dir_size_bytes(full)
        mtime   = datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
        bags.append({
            'name':    name,
            'size':    fmt_size(size_b),
            'size_b':  size_b,
            'mtime':   mtime,
        })
    return bags


# ── Sensor Control ───────────────────────────────────────────────────────────

def send_motor_cmd(cmd: str) -> tuple[bool, str]:
    """Write a single-char motor command directly to the Teensy serial port."""
    valid = {'x', 'r', 'm', 's', '+', '-'}
    if cmd not in valid:
        return False, f'Invalid command: {cmd!r}'
    try:
        with serial.Serial(TEENSY_PORT, TEENSY_BAUD, timeout=1, exclusive=False) as ser:
            ser.write(f'{cmd}\n'.encode())
        return True, cmd
    except Exception as e:
        return False, str(e)


def restart_all_sensors() -> tuple[bool, str]:
    """Restart the full hmi_bridge service — kills and relaunches all sensors."""
    try:
        subprocess.Popen(
            ['sudo', 'systemctl', 'restart', 'hmi_bridge'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return True, 'hmi_bridge service restarting — sensors back in ~10s'
    except Exception as e:
        return False, str(e)


# ── Routes ───────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html', thresholds=THRESHOLDS)


@app.route('/api/status')
def status():
    # Base Hz from app.py's own monitors (IMU, Encoder).
    hz = monitor.get()
    # Lidar Hz comes from hmi_bridge (the single efficient /velodyne_points
    # subscriber) via /hmi_bridge/status, so the dashboard shows it without
    # app.py opening its own multi-MB PointCloud2 subscriber. None = unknown
    # → frontend renders the card as dead, matching real "no data" behavior.
    bridge = ros_coord.get_status() if ros_coord is not None else {}
    hz['Velodyne'] = bridge.get('lidar_raw_hz') if bridge else None
    return jsonify({
        'hz':            hz,
        'encoder_angle': angle_mon.get(),
        'platform_rpm':  vel_mon.get(),
        'disk':          get_disk(),
        'system':        get_system(),
        'recording':     recorder.recording,
        'rec_duration':  int(time.monotonic() - recorder._rec_start) if recorder.recording and recorder._rec_start else 0,
        'bag_name':      recorder.bag_name,
        'mode':          recorder.mode,
        'pending_bag':   recorder.pending_bag,
        'pending_mode':  recorder.pending_mode,
        'topic_modes':   TOPIC_MODES,
    })


@app.route('/api/record/start', methods=['POST'])
def record_start():
    body = request.get_json(silent=True) or {}
    mode = body.get('mode', 'slam')
    ok, msg = recorder.start(mode)
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/record/stop', methods=['POST'])
def record_stop():
    ok, msg = recorder.stop()
    return jsonify({'ok': ok, 'msg': msg})


# ── Post-stop bag finalization (verify / rename / discard) ───────────────────

_BAG_NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')


def sanitize_bag_name(name: str) -> str | None:
    """Return a safe new bag-name or None if invalid. Restricts to [A-Za-z0-9_-],
    1–64 chars, must not start with '-' (would look like a CLI flag)."""
    if not isinstance(name, str):
        return None
    name = name.strip()
    if not name or name.startswith('-') or name.startswith('.'):
        return None
    if not _BAG_NAME_RE.match(name):
        return None
    return name


def _expected_topics_for_mode(mode: str) -> list[str]:
    return list(RECORD_TOPICS.get(mode, RECORD_TOPICS.get('slam', [])))


def verify_bag(bag_name: str, mode: str) -> dict:
    """Read the just-finalized mcap and run quick sanity checks.
    Returns {'ok': bool, 'checks': [{label, status: ok|warn|fail, detail}],
             'summary': str}."""
    import glob as _glob
    bag_path = os.path.join(BAG_DIR, bag_name)
    if not os.path.isdir(bag_path):
        return {'ok': False, 'summary': 'Bag directory not found',
                'checks': [{'label': 'bag', 'status': 'fail', 'detail': bag_path}]}
    mcap_files = sorted(_glob.glob(os.path.join(bag_path, '*.mcap')))
    if not mcap_files:
        return {'ok': False, 'summary': 'No .mcap file in bag',
                'checks': [{'label': 'mcap', 'status': 'fail', 'detail': 'missing'}]}

    try:
        from mcap.reader import make_reader
    except Exception as e:
        return {'ok': False, 'summary': f'mcap lib missing: {e}',
                'checks': [{'label': 'reader', 'status': 'fail', 'detail': str(e)}]}

    from collections import Counter, defaultdict
    counts        = Counter()
    empty_counts  = Counter()
    log_min       = {}
    log_max       = {}
    overall_min   = None
    overall_max   = None
    try:
        with open(mcap_files[0], 'rb') as f:
            for schema, channel, message in make_reader(f).iter_messages():
                t = channel.topic
                counts[t] += 1
                if len(message.data) == 0:
                    empty_counts[t] += 1
                lt = message.log_time
                if log_min.get(t) is None or lt < log_min[t]: log_min[t] = lt
                if log_max.get(t) is None or lt > log_max[t]: log_max[t] = lt
                if overall_min is None or lt < overall_min: overall_min = lt
                if overall_max is None or lt > overall_max: overall_max = lt
    except Exception as e:
        return {'ok': False, 'summary': f'Read failed: {e}',
                'checks': [{'label': 'mcap read', 'status': 'fail', 'detail': str(e)}]}

    duration_s = ((overall_max - overall_min) / 1e9) if (overall_min and overall_max) else 0.0
    checks: list[dict] = []

    # 1. Duration sanity (catches the LiDAR-PTP epoch-0 corruption pattern)
    if duration_s <= 0:
        checks.append({'label': 'Duration', 'status': 'fail', 'detail': '0 s — empty bag'})
    elif duration_s > 60 * 60 * 24 * 365:
        checks.append({'label': 'Duration', 'status': 'fail',
                       'detail': f'{duration_s:.0f}s — corrupted (epoch-0 messages present)'})
    else:
        checks.append({'label': 'Duration', 'status': 'ok', 'detail': f'{duration_s:.1f} s'})

    # 2. Per-topic checks. Rate windows are intentionally loose.
    rate_windows = {
        '/velodyne_points':            (10.0, 20.0, 14.0),
        '/rotating_platform/angle':    (150.0, 210.0, 180.0),
        '/rotating_platform/velocity': (150.0, 210.0, 180.0),
    }
    for t in _expected_topics_for_mode(mode):
        n = counts.get(t, 0)
        e = empty_counts.get(t, 0)
        if n == 0:
            checks.append({'label': t, 'status': 'fail', 'detail': 'no messages'})
            continue
        if e > 0:
            checks.append({'label': t, 'status': 'fail',
                           'detail': f'{e} of {n} are 0-byte (likely LiDAR-PTP regression)'})
            continue
        rate = (n / duration_s) if duration_s > 0 else 0.0
        rmin, rmax, rexp = rate_windows.get(t, (0.0, float('inf'), None))
        if rmin <= rate <= rmax:
            checks.append({'label': t, 'status': 'ok',
                           'detail': f'{n} msgs @ {rate:.1f} Hz'})
        else:
            exp = f' (expect ~{rexp:.0f} Hz)' if rexp is not None else ''
            checks.append({'label': t, 'status': 'warn',
                           'detail': f'{n} msgs @ {rate:.1f} Hz{exp}'})

    # 3. Unexpected-topic note (informational)
    extra = sorted(set(counts) - set(_expected_topics_for_mode(mode)))
    if extra:
        checks.append({'label': 'Extra topics', 'status': 'warn',
                       'detail': ', '.join(extra)})

    fails = sum(1 for c in checks if c['status'] == 'fail')
    warns = sum(1 for c in checks if c['status'] == 'warn')
    ok = fails == 0
    if fails:
        summary = f'{fails} check(s) failed — recording likely bad'
    elif warns:
        summary = f'{warns} warning(s) — usable, but inspect'
    else:
        summary = 'All checks passed'
    return {'ok': ok, 'checks': checks, 'summary': summary}


@app.route('/api/record/verify', methods=['POST'])
def record_verify():
    body = request.get_json(silent=True) or {}
    name = body.get('name') or recorder.pending_bag
    mode = body.get('mode') or recorder.pending_mode or 'slam'
    if not name:
        return jsonify({'ok': False, 'msg': 'No pending bag'}), 400
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    return jsonify(verify_bag(name, mode))


@app.route('/api/record/finalize', methods=['POST'])
def record_finalize():
    """Save the pending bag. Optionally rename. Clears pending state."""
    body     = request.get_json(silent=True) or {}
    cur_name = body.get('name') or recorder.pending_bag
    new_name = body.get('new_name')
    if not cur_name:
        return jsonify({'ok': False, 'msg': 'No pending bag'}), 400
    if not sanitize_bag_name(cur_name):
        return jsonify({'ok': False, 'msg': 'Invalid current name'}), 400
    src = os.path.join(BAG_DIR, cur_name)
    if not os.path.isdir(src):
        return jsonify({'ok': False, 'msg': 'Bag directory missing on disk'}), 404

    final_name = cur_name
    if new_name and new_name != cur_name:
        clean = sanitize_bag_name(new_name)
        if not clean:
            return jsonify({'ok': False, 'msg': 'Invalid new name (use A-Z a-z 0-9 _ -)'}), 400
        dst = os.path.join(BAG_DIR, clean)
        if os.path.exists(dst):
            return jsonify({'ok': False, 'msg': f'Name "{clean}" already exists'}), 409
        try:
            os.rename(src, dst)
            final_name = clean
        except Exception as e:
            return jsonify({'ok': False, 'msg': f'Rename failed: {e}'}), 500

    recorder.clear_pending()
    return jsonify({'ok': True, 'msg': f'Saved as {final_name}', 'name': final_name})


@app.route('/api/record/discard', methods=['POST'])
def record_discard():
    """Move the pending bag into BAG_TRASH_DIR (no rm -rf). Clears pending state."""
    body = request.get_json(silent=True) or {}
    name = body.get('name') or recorder.pending_bag
    if not name:
        return jsonify({'ok': False, 'msg': 'No pending bag'}), 400
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    src = os.path.join(BAG_DIR, name)
    if not os.path.isdir(src):
        # Nothing to move — still clear pending so UI returns to normal
        recorder.clear_pending()
        return jsonify({'ok': True, 'msg': 'Bag dir missing — pending cleared'})
    dst_name = name
    dst = os.path.join(BAG_TRASH_DIR, dst_name)
    if os.path.exists(dst):
        dst_name = f"{name}_trashed_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        dst = os.path.join(BAG_TRASH_DIR, dst_name)
    try:
        shutil.move(src, dst)
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'Move-to-trash failed: {e}'}), 500
    recorder.clear_pending()
    return jsonify({'ok': True, 'msg': f'Moved to trash as {dst_name}'})


@app.route('/bags')
def bags_page():
    return render_template('bags.html')


@app.route('/api/bags')
def api_bags():
    bags = get_bags()
    total_b = sum(b['size_b'] for b in bags)
    return jsonify({'bags': bags, 'total': fmt_size(total_b), 'count': len(bags)})


@app.route('/api/bags/<path:name>', methods=['DELETE'])
def api_bag_delete(name):
    # Sanitise — no path traversal
    if '/' in name or '..' in name:
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    full = os.path.join(BAG_DIR, name)
    if not os.path.isdir(full):
        return jsonify({'ok': False, 'msg': 'Not found'}), 404
    import shutil as _shutil
    try:
        _shutil.rmtree(full)
        return jsonify({'ok': True, 'msg': f'Deleted {name}'})
    except Exception as e:
        return jsonify({'ok': False, 'msg': str(e)}), 500


# ── Point-cloud preview (downsampled deskew, computed on-device) ─────────────
# Reuses the calibration/deskew math from ~/Reference/DeskewStudio; see
# bag_preview.py for why this reads a bounded, evenly-spaced subset instead
# of the full bag.

@app.route('/preview/<name>')
def preview_page(name):
    if not sanitize_bag_name(name):
        return 'Invalid bag name', 400
    if not os.path.isdir(os.path.join(BAG_DIR, name)):
        return 'Bag not found', 404
    return render_template('preview.html', bag_name=name)


@app.route('/api/bags/<name>/preview')
def api_bag_preview(name):
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    bag_path = os.path.join(BAG_DIR, name)
    if not os.path.isdir(bag_path):
        return jsonify({'ok': False, 'msg': 'Bag not found'}), 404

    max_clouds = max(1, min(request.args.get('max_clouds', 25, type=int), 300))
    max_points = max(1000, min(request.args.get('max_points', 150000, type=int), 1000000))
    target_rotations = max(0.5, min(request.args.get('target_rotations', 2.5, type=float), 30.0))

    try:
        from bag_preview import build_bag_preview
        result = build_bag_preview(bag_path, max_clouds=max_clouds, max_points=max_points,
                                    target_rotations=target_rotations)
    except (FileNotFoundError, ValueError) as e:
        return jsonify({'ok': False, 'msg': str(e)}), 400
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'Preview failed: {e}'}), 500

    resp = app.response_class(result['buf'], mimetype='application/octet-stream')
    resp.headers['X-Point-Count']          = str(result['points'])
    resp.headers['X-Cloud-Count']          = str(result['clouds'])
    resp.headers['X-Cloud-Total']          = str(result['clouds_total'])
    resp.headers['X-Low-Coverage-Clouds']  = str(result['low_coverage_clouds'])
    resp.headers['X-Encoder-Source']       = result['encoder_source']
    resp.headers['X-Rotations-Captured']   = str(result['rotations_captured'])
    resp.headers['X-Compute-S']            = str(result['compute_s'])
    return resp


@app.route('/api/motor/cmd', methods=['POST'])
def motor_cmd():
    body = request.get_json(silent=True) or {}
    cmd  = body.get('cmd', '').strip()
    ok, msg = send_motor_cmd(cmd)
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/sensors/restart_all', methods=['POST'])
def sensors_restart_all():
    ok, msg = restart_all_sensors()
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/shutdown', methods=['POST'])
def shutdown():
    try:
        subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'ok': False, 'msg': str(e)})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=3000, debug=False, threaded=True)
