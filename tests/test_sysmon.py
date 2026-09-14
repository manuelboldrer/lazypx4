"""Tests for the host-system monitor."""

from __future__ import annotations

from lazypx4 import sysmon
from lazypx4.render.dashboard import _host_lines
from lazypx4.sysmon import stats


def test_cpu_percent_from_jiffy_deltas():
    # 100 total jiffies elapsed, 25 of them idle -> 75% busy
    assert sysmon._cpu_percent((1000, 500), (1100, 525)) == 75.0
    assert sysmon._cpu_percent(None, (1, 1)) == 0.0
    assert sysmon._cpu_percent((10, 5), (10, 5)) == 0.0  # no elapsed time


def test_detect_services(monkeypatch):
    fake = [
        "/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record -a -o /data/flight1",
        "zenoh-bridge-ros2dds -c /etc/zenoh/bridge.json5",
        "/usr/local/bin/MicroXRCEAgent udp4 --port 8888",
        "/lib/systemd/systemd --user",
    ]
    monkeypatch.setattr(sysmon, "_iter_cmdlines", lambda: iter(c.lower() for c in fake))
    rosbag, zenoh, xrce_agent = sysmon._detect_services()
    assert rosbag is True
    assert zenoh is True
    assert xrce_agent is True


def test_detect_services_none_running(monkeypatch):
    monkeypatch.setattr(
        sysmon, "_iter_cmdlines",
        lambda: iter(["/usr/bin/bash", "sshd: user@pts/0"]),
    )
    assert sysmon._detect_services() == (False, False, False)


def test_host_lines_render_without_a_sample():
    with stats.lock:
        stats.ok = False
    lines = _host_lines()
    assert len(lines) == 2
    assert "n/a" in lines[0]
    assert "rosbag" in lines[1] and "zenoh" in lines[1]


def test_host_lines_render_with_a_sample():
    with stats.lock:
        stats.ok = True
        stats.cpu_percent = 42.0
        stats.mem_percent = 55.0
        stats.mem_used_gb = 2.2
        stats.mem_total_gb = 4.0
        stats.disk_percent = 91.0
        stats.disk_free_gb = 3.0
        stats.load1 = 0.9
        stats.rosbag_recording = True
        stats.zenoh_running = False
        stats.xrce_agent_running = True
    host, svc = _host_lines()
    assert "CPU" in host and "42%" in host
    assert "REC" in svc
    assert "down" in svc
    assert "xrce-agent" in svc
