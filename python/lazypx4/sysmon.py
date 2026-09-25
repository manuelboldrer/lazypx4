"""Host-system monitor: CPU / RAM / disk load and a couple of service checks.

This watches the machine lazypx4 runs on (a UAV companion computer), not the
vehicle. It is deliberately self-contained - delete this module and the one
`HOST` block on the dashboard and nothing else changes.

Linux only (reads ``/proc``); on other platforms ``stats.ok`` stays False and
the dashboard simply omits the block.
"""

from __future__ import annotations

import os
import shutil
import threading
import time
from dataclasses import dataclass, field

from .config import settings
from .eventlog import log_warn
from .state import shutdown_event

SAMPLE_INTERVAL = 2.0


@dataclass
class SystemStats:
    ok: bool = False               # /proc was readable at least once

    cpu_percent: float = 0.0
    load1: float = 0.0

    mem_percent: float = 0.0
    mem_used_gb: float = 0.0
    mem_total_gb: float = 0.0

    disk_percent: float = 0.0
    disk_free_gb: float = 0.0

    rosbag_recording: bool = False
    zenoh_running: bool = False
    xrce_agent_running: bool = False

    updated: float = 0.0

    lock: threading.Lock = field(default_factory=threading.Lock)


stats = SystemStats()


# ---------------------------------------------------------------------------
# /proc sampling
# ---------------------------------------------------------------------------


def _read_cpu_times():
    """(total, idle) jiffies from /proc/stat, or None."""
    try:
        with open("/proc/stat", encoding="ascii") as handle:
            fields = handle.readline().split()
        values = [int(v) for v in fields[1:]]
    except (OSError, ValueError):
        return None

    idle = values[3] + (values[4] if len(values) > 4 else 0)  # idle + iowait
    return sum(values), idle


def _cpu_percent(previous, current):
    if not previous or not current:
        return 0.0
    total_delta = current[0] - previous[0]
    idle_delta = current[1] - previous[1]
    if total_delta <= 0:
        return 0.0
    return max(0.0, min(100.0, 100.0 * (total_delta - idle_delta) / total_delta))


def _read_mem_gb():
    """(used_gb, total_gb, used_percent) from /proc/meminfo, or None."""
    info = {}
    try:
        with open("/proc/meminfo", encoding="ascii") as handle:
            for line in handle:
                key, _, rest = line.partition(":")
                info[key] = int(rest.split()[0])  # kB
    except (OSError, ValueError):
        return None

    total_kb = info.get("MemTotal", 0)
    avail_kb = info.get("MemAvailable", info.get("MemFree", 0))
    if total_kb <= 0:
        return None

    used_kb = max(0, total_kb - avail_kb)
    to_gb = 1.0 / (1024 * 1024)
    return used_kb * to_gb, total_kb * to_gb, 100.0 * used_kb / total_kb


# ---------------------------------------------------------------------------
# Service detection (scan /proc/<pid>/cmdline)
# ---------------------------------------------------------------------------


def _iter_cmdlines():
    try:
        pids = os.listdir("/proc")
    except OSError:
        return
    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as handle:
                raw = handle.read()
        except OSError:
            continue
        if raw:
            yield raw.replace(b"\x00", b" ").decode("utf-8", "replace").lower()


def _detect_services():
    rosbag = False
    zenoh = False
    xrce_agent = False
    for cmd in _iter_cmdlines():
        if not rosbag and "record" in cmd and ("bag" in cmd or "rosbag2" in cmd):
            rosbag = True
        if not zenoh and "zenoh" in cmd:
            zenoh = True
        # Matches PX4's own MicroXRCEAgent binary, a `micro-xrce-dds-agent`
        # build from source, or anything else with "xrce" in its invocation -
        # this is the uXRCE-DDS bridge PX4 (v1.14+) needs to publish/subscribe
        # uORB topics as ROS 2 topics.
        if not xrce_agent and "xrce" in cmd:
            xrce_agent = True
        if rosbag and zenoh and xrce_agent:
            break
    return rosbag, zenoh, xrce_agent


# ---------------------------------------------------------------------------
# Background thread
# ---------------------------------------------------------------------------


def _sample(previous_cpu):
    cpu_now = _read_cpu_times()
    mem = _read_mem_gb()

    try:
        disk = shutil.disk_usage(settings.disk_path)
    except OSError:
        disk = None

    try:
        load1 = os.getloadavg()[0]
    except (OSError, AttributeError):
        load1 = 0.0

    rosbag, zenoh, xrce_agent = _detect_services()

    with stats.lock:
        if cpu_now is not None:
            stats.ok = True
            stats.cpu_percent = _cpu_percent(previous_cpu, cpu_now)
        stats.load1 = load1
        if mem is not None:
            stats.mem_used_gb, stats.mem_total_gb, stats.mem_percent = mem
        if disk is not None:
            stats.disk_free_gb = disk.free / (1024 ** 3)
            stats.disk_percent = 100.0 * disk.used / disk.total if disk.total else 0.0
        stats.rosbag_recording = rosbag
        stats.zenoh_running = zenoh
        stats.xrce_agent_running = xrce_agent
        stats.updated = time.monotonic()

    return cpu_now or previous_cpu


def sysmon_thread():
    previous_cpu = _read_cpu_times()
    warned = False

    while not shutdown_event.is_set():
        time.sleep(SAMPLE_INTERVAL)
        try:
            previous_cpu = _sample(previous_cpu)
        except Exception as exc:  # never let host monitoring take down the app
            if not warned:
                warned = True
                log_warn(f"System monitor error: {exc}")
