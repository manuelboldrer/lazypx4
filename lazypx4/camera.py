"""Local V4L2/USB camera preview for the [w] screen: runs ``ffmpeg`` as a
long-lived subprocess streaming raw frames from ``settings.camera_device``
(default ``/dev/video0``), independent of MAVLink and of the vehicle - a
webcam or gimbal camera plugged into whatever machine is running lazypx4.

Never started automatically: opening the device and running ffmpeg is real
CPU/USB cost, so capture only begins after the [w] screen's [o] toggle is
confirmed, same "never automatic" rule netmon.py's internet speed test
follows. [b] on that screen switches between two presets - see CAMERA_* in
lazypx4.config - "full" (bigger, more frequent frames, rendered as ANSI
truecolor half-blocks) and "low bandwidth" (smaller, less frequent frames
rendered as a colourless ASCII ramp), the latter meant for a slow link such
as a telemetry radio or a thin cellular tether where every byte the
terminal redraw writes matters.

The ffmpeg process is kept running and read continuously rather than
re-spawned per frame: re-opening a V4L2 device on every capture pays that
device's negotiation/settling time (often hundreds of ms to seconds on USB
webcams) on every single frame, which caps effective fps far below what the
device can actually deliver. One process is started per capture session
(and restarted on preset/device change or if it dies) and frames are read
off its stdout as fast as the device produces them, up to the preset fps.

Delete this module and the [w] CAMERA screen and nothing else changes.
"""

from __future__ import annotations

import select
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field

from .config import (
    CAMERA_FRAME_TIMEOUT,
    CAMERA_FULL_FPS,
    CAMERA_FULL_HEIGHT,
    CAMERA_FULL_WIDTH,
    CAMERA_LOW_BW_FPS,
    CAMERA_LOW_BW_HEIGHT,
    CAMERA_LOW_BW_WIDTH,
    CAMERA_RETRY_INTERVAL,
    CAMERA_START_TIMEOUT,
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
    capturing: bool = False       # ffmpeg session is starting/running
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
        return CAMERA_LOW_BW_WIDTH, CAMERA_LOW_BW_HEIGHT, CAMERA_LOW_BW_FPS, True
    return CAMERA_FULL_WIDTH, CAMERA_FULL_HEIGHT, CAMERA_FULL_FPS, False


def _spawn(device, width, height, fps):
    return subprocess.Popen(
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-framerate", str(fps), "-video_size", f"{width}x{height}",
            "-i", device,
            "-vf", f"scale={width}:{height}",
            "-pix_fmt", "rgb24", "-f", "rawvideo",
            "-",
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )


def _read_exact(stream, n, deadline):
    """Read exactly `n` bytes from `stream`, or None on EOF/timeout. Uses
    ``select`` so a stalled (but still-open) pipe can't block past the
    deadline the way a plain ``stream.read()`` would."""
    buf = bytearray()
    fd = stream.fileno()
    while len(buf) < n:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        ready, _, _ = select.select([fd], [], [], remaining)
        if not ready:
            return None
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _drain_stderr(proc):
    try:
        data = proc.stderr.read()
    except Exception:
        data = b""
    text = (data or b"").decode("utf-8", errors="replace").strip()
    lines = text.splitlines()
    return lines[-1][:200] if lines else ""


def _stop(proc):
    if proc is None:
        return
    try:
        proc.kill()
        proc.wait(timeout=2.0)
    except Exception:
        pass


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


def _session_target():
    with stats.lock:
        return stats.enabled, stats.device or settings.camera_device, stats.low_bandwidth


def _run_session(device, width, height, fps, low_bandwidth):
    """Run one ffmpeg capture session until settings change, the feed is
    turned off, the stream errors out, or shutdown is requested."""
    frame_size = width * height * 3

    try:
        proc = _spawn(device, width, height, fps)
    except OSError as exc:
        with stats.lock:
            stats.capturing = False
            stats.error = f"couldn't run ffmpeg: {exc}"
        return

    with stats.lock:
        stats.capturing = True

    last_rate_count = 0
    last_rate_time = time.monotonic()
    logged_error = False
    got_frame = False

    try:
        while not shutdown_event.is_set():
            still_enabled, still_device, still_low_bw = _session_target()
            if not still_enabled or still_device != device or still_low_bw != low_bandwidth:
                break

            deadline = time.monotonic() + (
                CAMERA_START_TIMEOUT if not got_frame else CAMERA_FRAME_TIMEOUT
            )
            frame = _read_exact(proc.stdout, frame_size, deadline)
            now = time.monotonic()

            if frame is None:
                exited = proc.poll() is not None
                if not exited:
                    _stop(proc)  # stalled but still open - kill it so
                                 # stderr becomes readable without blocking
                error = _drain_stderr(proc) or (
                    "ffmpeg exited unexpectedly" if exited else "ffmpeg produced no frame (timed out)"
                )
                with stats.lock:
                    stats.error = error
                if not logged_error:
                    logged_error = True
                    log_warn(f"Camera capture failed: {error}")
                break

            got_frame = True
            with stats.lock:
                stats.frame_rgb = frame
                stats.frame_width = width
                stats.frame_height = height
                stats.frame_count += 1
                stats.last_frame_at = now
                stats.error = ""
                stats.capturing = False

                if now - last_rate_time >= 1.0:
                    stats.fps = (stats.frame_count - last_rate_count) / (now - last_rate_time)
                    last_rate_count = stats.frame_count
                    last_rate_time = now
    finally:
        _stop(proc)
        with stats.lock:
            stats.capturing = False


def camera_thread():
    with stats.lock:
        stats.available = camera_available()
        stats.checked = True
        stats.device = settings.camera_device

    if not stats.available:
        return

    while not shutdown_event.is_set():
        enabled, device, _ = _session_target()

        if not enabled:
            time.sleep(_IDLE_POLL)
            continue

        width, height, fps, low_bandwidth = _preset()
        _run_session(device, width, height, fps, low_bandwidth)

        if shutdown_event.is_set():
            break

        # Back off before retrying so a persistently failing device (wrong
        # path, unplugged, in use by another process) doesn't spin ffmpeg.
        still_enabled, _, _ = _session_target()
        if still_enabled:
            time.sleep(CAMERA_RETRY_INTERVAL)
