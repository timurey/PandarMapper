# encoder_bridge — Known Issues & Fixes

## Issue 1: NTP Clock Jump Kills the Watchdog (FIXED)

### Symptom
Encoder node dies with `WATCHDOG: Serial thread stalled for 826300728.7s!` approximately
1.5 minutes after boot. Only happens on first boot before NTP syncs the Pi 5 clock.

### Root Cause
The Pi 5 boots with RTC clock at epoch ~Jan 1, 2000. After ~1.5 minutes, NTP corrects
the system clock to the real date (~2026). The watchdog used `time.time()` (wall clock)
for both the heartbeat and the stall comparison. The NTP jump caused `time.time()` to
leap forward by ~826 million seconds, making the stall appear enormous and triggering
`os._exit(1)`.

### Fix (applied 2026-03-09)
Replaced all `time.time()` calls in the watchdog/heartbeat chain with `time.monotonic()`.
`time.monotonic()` is immune to NTP or manual clock adjustments. Only the stats-rate
display (`log_statistics`) retains `time.time()` for wall-clock elapsed time.

**Files changed:** `encoder_bridge/encoder_node.py`

**Workaround if not yet fixed:** Set the clock manually before launching:
```bash
sudo timedatectl set-ntp false
sudo timedatectl set-time '2026-MM-DD HH:MM:SS'
```

---

## Issue 2: Pi 5 PL011 UART Driver Hang (FIXED)

### Symptom
Encoder node dies cleanly (no log entry — async logger not flushed before `os._exit`)
approximately every 90–110 seconds during normal operation. Watchdog fires correctly
but the underlying serial thread has permanently blocked.

### Root Cause
The Pi 5's PL011 UART driver occasionally hangs indefinitely inside `ioctl(FIONREAD)`,
which is what `pyserial`'s `in_waiting` property calls under the hood. This occurs
under I/O load (velodyne + IMU + encoder all running). Once `in_waiting` blocks, the
serial thread stops updating its heartbeat, and the watchdog kills the process after
`data_timeout` (3.0s) via `os._exit(1)`.

### Fix attempt 1 (2026-03-09) — select() — INSUFFICIENT
Replaced `in_waiting` with `select.select(timeout=0.1)`. Still failed after ~80s.

**Root cause of failure:** pyserial 3.5's `read()` method also calls `select()` internally
on every call (see `serialposix.py`), so the driver's `poll()` function was being hit
twice per loop iteration. With `timeout=0` (non-blocking), pyserial calls
`select([fd, pipe_abort], [], [], 0)` before every `os.read()`. Even with timeout=0,
`select()` calls the kernel driver's `poll()`, which is exactly what hangs.

### Fix attempt 2 (2026-03-09) — os.read() — CURRENT
Replaced both the outer `select()` and `self.serial_port.read()` with a direct
`os.read(fd, 4096)` call. With `O_NONBLOCK` set on the fd (done by pyserial with
`timeout=0`), `os.read()` raises `BlockingIOError` (EAGAIN) immediately when no data
is available, going through the VFS tty buffer directly without calling the driver's
`poll()` function at all.

**Files changed:** `encoder_bridge/encoder_node.py` — removed `import select`, added
`import errno`, replaced read path.

```python
# Before (both forms call driver poll() — hangs on Pi 5 PL011 AXI):
available = self.serial_port.in_waiting          # ioctl(FIONREAD) → poll
data = self.serial_port.read(4096)               # select() → poll

# After (bypasses poll entirely):
try:
    data = os.read(self.serial_port.fileno(), 4096)  # O_NONBLOCK → EAGAIN, no poll
except BlockingIOError:
    time.sleep(0.002); continue
```

### Notes
- `respawn=True, respawn_delay=2.0` in `robin_reconstruction.launch.py` is kept as
  a safety net for any unexpected crashes.
- The watchdog (`data_timeout=3.0s`, 1.0s timer) is also kept.
- The PL011 AXI variant on Pi 5 (`ttyAMA4`, irq=154) has `cts_event_workaround`
  enabled in the kernel — indicating known hardware issues with this UART.

---

## Issue 3: Reconnect Freezes ROS Executor on PL011 open() Hang (FIXED)

### Symptom
Encoder shows `encoder_hz: 0.0` permanently after starting a rosbag recording (or any
other sudden disk I/O spike). The node stays alive (sensors_running: True, lidar/IMU OK)
but never recovers — not even after the recording stops. HMI "Restart Encoder" button
has no effect. Only a full sensor restart brings it back.

### Root Cause
Two compounding bugs:

1. **data_timeout too short (3.0s → fixed to 12.0s in a previous session).**
   Teensy USB CDC + `initFOC()` takes 3-5s after a USB reset. 3s fired before Teensy
   could reconnect. (Fix already applied 2026-03-24.)

2. **Blocking serial open in the ROS executor thread (root cause of this failure).**
   When `_watchdog_check` detected data silence, it called `_schedule_reconnect()` which
   used `create_timer()` to schedule `_reconnect_callback`. After the delay, the executor
   called `_reconnect_callback` → `_connect_serial()` → `serial.Serial()`. On Pi 5, the
   PL011 UART driver can hang indefinitely inside `serial.Serial()` (blocking ioctls
   during `open()`) when the driver is in a stressed state from I/O load. With
   `serial.Serial()` blocking in the executor thread, the ENTIRE ROS executor froze:
   no more timer callbacks, no more publish callbacks, no more watchdog checks.
   The daemon watchdog thread only checks the serial thread heartbeat — since the serial
   thread was fine (sleeping in `not serial_connected` wait), it never fired `os._exit`.
   Result: permanently frozen executor, encoder stuck at 0 Hz forever.

### Fix (applied 2026-03-24)
Moved ALL `serial.Serial()` / `serial_port.close()` calls exclusively into the serial
reader background thread. The ROS executor now only sets Python attributes (`serial_connected`,
`_reconnect_at`) — both are non-blocking O(1) assignments.

**Reconnect flow (old):**
```
executor: _watchdog_check → serial_port.close() → create_timer(5s)
executor: _reconnect_callback → serial.Serial()  ← BLOCKS EXECUTOR IF PL011 HANGS
```

**Reconnect flow (new):**
```
executor: _watchdog_check → serial_connected=False; _reconnect_at=now+5s  (non-blocking)
serial thread: sees not serial_connected, waits for _reconnect_at timestamp
serial thread: _connect_serial() → serial.Serial()  ← blocks here, NOT in executor
daemon watchdog: if serial thread stalls >12s → os._exit(1) for respawn
```

**Key principle:** The ROS executor must NEVER call blocking serial operations. If
`serial.Serial()` hangs in the executor, no watchdog in the executor can fire to recover.
Only a raw Python daemon thread (`_watchdog_thread_fn`) is immune to executor stalls.

**Files changed:** `encoder_bridge/encoder_node.py`
- Removed `reconnect_timer`, `_schedule_reconnect()`, `_reconnect_callback()`
- Added `_reconnect_at` monotonic timestamp (set by executor, checked by serial thread)
- Initial connection moved to serial thread (no `_connect_serial()` call in `__init__`)
- All `serial.Serial()` calls now in `_connect_serial()`, called only from serial thread
- `_watchdog_check` and `_publish_latest` only set `_reconnect_at` — no serial calls

---

## Issue 4: Serial Reader Thread Blocked in ROS Logger pipe_write (FIXED)

### Symptom
Encoder shows `encoder_hz: 0.0` permanently after running for 7–15 minutes. The
encoder_node process stays alive at 30–45% CPU, port remains open (fd → `/dev/ttyACM0`),
Teensy is still sending valid NMEA frames, but no data is published. The log file stops
growing. Both encoder_node AND the slam_sensors launcher end up with `wchan=pipe_write`,
preventing the launcher from collecting the dead encoder zombie.

### Root Cause
The serial reader thread called `get_logger().info(f'Angle: ...')` 4 times per second
(whenever `teensy_timestamp_ms % 1000 < 20`). `get_logger()` in ROS2 Python sends the
message through a pipe to an async log-writer thread. If the async log-writer falls
behind (slow disk, filesystem flush, I/O burst from recording), the pipe buffer fills up.
The next `get_logger().info()` call in the serial reader thread blocks in `pipe_write`
indefinitely. With the serial reader thread stuck, no reads happen from the serial port,
the Teensy's CDC TX buffer fills up, `_last_data_time` stops updating, and encoder_hz
drops to zero. The `_watchdog_check` executor timer fires but can no longer trigger
recovery because the serial thread (which must do the reconnect) is permanently blocked.

**Confirmed via** `/proc/<pid>/task/<tid>/wchan = pipe_write` for the serial reader TID.

### Fix (applied 2026-03-24)
Removed the `get_logger().info()` call from the serial reader thread entirely.
The 10-second stats log (`log_statistics`) is sufficient for operational monitoring.
**Rule: the serial reader thread must NEVER call get_logger() in its hot path.**
Any blocking call in the serial reader thread — including logging — halts all reads.

**Files changed:** `encoder_bridge/encoder_node.py` — removed `Angle: X°` log call.

---

## System Clock Note

The Pi 5 has no battery-backed RTC. On every boot it starts at Jan 1, 2000 until NTP
syncs. If NTP is unavailable in the field, set the clock manually:

```bash
sudo timedatectl set-ntp false
sudo timedatectl set-time '2026-MM-DD HH:MM:SS'
```

Consider adding a GPS-disciplined NTP source or a DS3231 RTC module for field use.
