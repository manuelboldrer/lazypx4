"""Local V4L2/USB camera preview for the [w] screen: shells out to ``ffmpeg``
to grab single frames from ``settings.camera_device`` (default
``/dev/video0``), independent of MAVLink and of the vehicle - a webcam or
gimbal camera plugged into whatever machine is running lazypx4.

Never started automatically: opening the device and spawning ffmpeg every
frame is real CPU/USB cost, so capture only begins after the [w] screen's
[o] toggle is confirmed, same "never automatic" rule netmon.py's internet
speed test follows. [b] on that screen switches between two presets - see
CAMERA_* in lazypx4.config - "full" (bigger frames, more of them, rendered
as ANSI truecolor half-blocks) and "low bandwidth" (smaller, less frequent
frames rendered as a colourless ASCII ramp), the latter meant for a slow
link such as a telemetry radio or a thin cellular tether where every byte
the terminal redraw writes matters.

Delete this module and the [w] CAMERA screen and nothing else changes.
"""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field

from .config import (
    CAMERA_CAPTURE_TIMEOUT,
    CAMERA_FULL_HEIGHT,
    CAMERA_FULL_INTERVAL,
    CAMERA_FULL_WIDTH,
    CAMERA_LOW_BW_HEIGHT,
    CAMERA_LOW_BW_INTERVAL,
    CAMERA_LOW_BW_WIDTH,
    settings,
)
from .eventlog import log_warn
from .state import shutdown_event

_IDLE_POLL = 0.2


@dataclass
class CameraStats:
    available: bool = False       # `ffmpeg` found on PATH
    checked: bool = False         # the PATH check has run at least once

    enabled: bool = False         # user has turned the feed on
    capturing: bool = False       # a frame grab is in flight right now
    low_bandwidth: bool = False

    device: str = ""
    error: str = ""

    frame_width: int = 0
    frame_height: int = 0
    frame_rgb: bytes = b""        # raw RGB24, frame_width * frame_height * 3
    frame_count: int = 0
    fps: float = 0.0
    last_frame_at: float = 0.0

    lock: threading.Lock = field(default_factory=threading.Lock)


stats = CameraStats()


def camera_available():
    return shutil.which("ffmpeg") is not None


def _preset():
    with stats.lock:
        low_bw = stats.low_bandwidth
    if low_bw:
        return CAMERA_LOW_BW_WIDTH, CAMERA_LOW_BW_HEIGHT, CAMERA_LOW_BW_INTERVAL
    return CAMERA_FULL_WIDTH, CAMERA_FULL_HEIGHT, CAMERA_FULL_INTERVAL


def _capture_frame(device, width, height):
    """One RGB24 frame at (width, height), or None on failure (leaves
    ``stats.error`` set describing why)."""
    try:
        proc = subprocess.run(
            [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-f", "v4l2", "-i", device,
                "-vframes", "1",
                "-vf", f"scale={width}:{height}",
                "-pix_fmt", "rgb24", "-f", "rawvideo",
                "-",
            ],
            capture_output=True, timeout=CAMERA_CAPTURE_TIMEOUT, check=False,
        )
    except subprocess.TimeoutExpired:
        return None, "ffmpeg timed out reading the camera"
    except OSError as exc:
        return None, f"couldn't run ffmpeg: {exc}"

    expected = width * height * 3
    if proc.returncode != 0 or len(proc.stdout) < expected:
        detail = proc.stderr.decode("utf-8", errors="replace").strip().splitlines()
        return None, (detail[-1][:200] if detail else "ffmpeg produced no frame")

    return proc.stdout[:expected], ""


def toggle_enabled(enable):
    """Turn capture on/off. Starting is what needs the confirmation prompt
    upstream (in navigation.py) - stopping is always safe to do immediately."""
    with stats.lock:
        stats.enabled = enable
        if enable:
            stats.device = settings.camera_device
            stats.error = ""
        else:
            stats.capturing = False


def toggle_low_bandwidth():
    with stats.lock:
        stats.low_bandwidth = not stats.low_bandwidth


def camera_thread():
    with stats.lock:
        stats.available = camera_available()
        stats.checked = True
        stats.device = settings.camera_device

    if not stats.available:
        return

    last_rate_count = 0
    last_rate_time = time.monotonic()
    last_logged_error = ""

    while not shutdown_event.is_set():
        with stats.lock:
            enabled = stats.enabled
            device = stats.device or settings.camera_device

        if not enabled:
            time.sleep(_IDLE_POLL)
            last_rate_count = stats.frame_count
            last_rate_time = time.monotonic()
            continue

        width, height, interval = _preset()

        with stats.lock:
            stats.capturing = True

        try:
            frame, error = _capture_frame(device, width, height)
        except Exception as exc:  # never let a camera hiccup take down the app
            frame, error = None, str(exc)[:200]

        now = time.monotonic()

        with stats.lock:
            stats.capturing = False
            if frame is not None:
                stats.frame_rgb = frame
                stats.frame_width = width
                stats.frame_height = height
                stats.frame_count += 1
                stats.last_frame_at = now
                stats.error = ""
            else:
                stats.error = error

            if now - last_rate_time >= 1.0:
                stats.fps = (stats.frame_count - last_rate_count) / (now - last_rate_time)
                last_rate_count = stats.frame_count
                last_rate_time = now

        if frame is None:
            if error != last_logged_error:
                last_logged_error = error
                log_warn(f"Camera capture failed: {error}")
            time.sleep(max(interval, 1.0))
        else:
            last_logged_error = ""
            time.sleep(interval)
