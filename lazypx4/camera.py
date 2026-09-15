"""ROS 2 camera preview for the [w] screen: subscribes to up to two
``sensor_msgs/Image`` or ``sensor_msgs/CompressedImage`` topics - see
``settings.camera_topic_1``/``camera_topic_2`` - and decodes each into a
plain RGB24 buffer :mod:`lazypx4.render.camera` draws as ANSI truecolor
half-blocks. Same shape as lazypx4.lidar's PointCloud2 handling: runs its
own rclpy node in its own ``rclpy.Context()`` (independent of rosclock's and
lidar's - the *implicit default* context can only be initialized once per
process, but callers each supplying their own Context coexist fine), and
the [w] screen's [1]/[2] keys change a topic at runtime without restarting
this thread.

Optional: if rclpy or sensor_msgs can't be imported (no ROS 2 environment
sourced) or numpy is missing, this module's thread returns immediately and
the camera screen just says so. A topic publishing CompressedImage
additionally needs Pillow (`pip install lazypx4[map]`) to decode - without
it that slot reports the topic is compressed and Pillow is missing, rather
than silently showing nothing.

Delete this module and the [w] CAMERA screen and nothing else changes.
"""

from __future__ import annotations

import io
import threading
import time
from dataclasses import dataclass, field

from .config import settings
from .eventlog import log_warn
from .state import shutdown_event

try:
    import numpy as np
except Exception:
    np = None

try:
    from PIL import Image as PILImage
except Exception:
    PILImage = None

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
except Exception:
    rclpy = None
    SingleThreadedExecutor = None
    Node = None
    qos_profile_sensor_data = None
    Image = None
    CompressedImage = None

#: Number of simultaneously subscribed topics/slots the [w] screen shows.
NUM_SLOTS = 2

#: sensor_msgs/Image ``encoding`` -> (channel count, already-RGB byte order,
#: or None for mono8 which is replicated to all 3 channels). Only the 8-bit
#: encodings common ROS camera drivers actually publish - anything else
#: reports "unsupported encoding" rather than guessing at a layout, except
#: "mono16" (16-bit grayscale - depth/thermal cameras), decoded separately
#: by _decode_mono16() since it needs contrast stretching, not a fixed
#: channel layout.
_ENCODINGS = {
    "rgb8": (3, True),
    "bgr8": (3, False),
    "rgba8": (4, True),
    "bgra8": (4, False),
    "mono8": (1, None),
}

_IDLE_POLL = 0.2

#: "Ironbow"-style false-color stops (value 0-255 -> RGB) for mono16 frames
#: - see _decode_mono16(). A plain grayscale ramp reads as "completely dark"
#: for real thermal/depth data: it has far more low/mid values than bright
#: ones, so most of a frame lands in the low, hard-to-distinguish end of a
#: linear gray ramp. This palette's own steepest color change sits in that
#: same low-mid range, trading many shades of near-black for a perceptible
#: black -> purple -> orange -> yellow -> white sweep.
_THERMAL_STOPS = (0, 64, 128, 192, 255)
_THERMAL_COLORS = ((0, 0, 0), (60, 0, 110), (170, 20, 90), (255, 120, 20), (255, 255, 200))


def _build_thermal_lut():
    if np is None:
        return None
    xs = np.arange(256, dtype=np.float32)
    channels = [
        np.interp(xs, _THERMAL_STOPS, [c[i] for c in _THERMAL_COLORS])
        for i in range(3)
    ]
    return np.stack(channels, axis=1).astype(np.uint8)


_THERMAL_LUT = _build_thermal_lut()


@dataclass
class CameraSlot:
    """One subscribed (or empty) topic's latest decoded frame + stats."""

    topic: str = ""
    subscribed_topic: str = ""
    #: "image", "compressed", or "" while the topic's type hasn't resolved yet.
    msg_kind: str = ""
    frame_width: int = 0
    frame_height: int = 0
    frame_rgb: bytes = b""       # raw RGB24, frame_width * frame_height * 3
    frame_count: int = 0
    fps: float = 0.0
    last_frame_at: float = 0.0
    error: str = ""
    #: Non-fatal note shown alongside a still-rendered frame - e.g. a mono16
    #: source with no pixel-to-pixel variation to color by (see
    #: _decode_mono16()), which isn't an error but does mean "no gradient to
    #: show" rather than "lazypx4 failed to render it".
    note: str = ""


@dataclass
class CameraStats:
    available: bool = False       # rclpy / sensor_msgs / numpy all importable
    checked: bool = False         # the availability check has run at least once
    low_bandwidth: bool = False   # render-side only - see CAMERA_LOW_BW_MAX_COLS
    slots: list = field(default_factory=lambda: [CameraSlot() for _ in range(NUM_SLOTS)])
    lock: threading.Lock = field(default_factory=threading.Lock)


stats = CameraStats()


def camera_available():
    return rclpy is not None and Image is not None and np is not None


def toggle_low_bandwidth():
    with stats.lock:
        stats.low_bandwidth = not stats.low_bandwidth


def _configured_topics():
    return [settings.camera_topic_1, settings.camera_topic_2]


def _decode_mono16(msg):
    """RGB24 bytes (or None, error) from a 16-bit-per-pixel grayscale
    sensor_msgs/Image (``mono16``) - common for depth/thermal cameras, where
    the raw values (millimetres, raw sensor counts, ...) span whatever range
    the source uses rather than a fixed 0-255 brightness. Contrast-stretched
    per frame using the 1st/99th percentile rather than the true min/max -
    a handful of dead/hot pixels at the extreme ends (common on real thermal
    sensors) would otherwise compress the entire rest of the frame into a
    few shades near one end, reading as "completely dark" - then mapped
    through the false-color _THERMAL_LUT instead of plain grayscale, since a
    linear gray ramp still under-differentiates the low/mid range typical
    scenes actually live in. Returns (rgb_bytes, error, note) - `note` is
    set instead of `error` for a frame with no pixel-to-pixel variation at
    all (some simulated thermal cameras publish a perfectly flat image),
    since that's "nothing to show a gradient over", not a decode failure."""
    if msg.step < msg.width * 2:
        return None, "row stride too small", ""

    needed = msg.height * msg.step
    if len(msg.data) < needed:
        return None, "short frame", ""

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=needed)
    rows = raw.reshape(msg.height, msg.step)
    pixel_bytes = np.ascontiguousarray(rows[:, : msg.width * 2])
    dtype = ">u2" if msg.is_bigendian else "<u2"
    values = pixel_bytes.view(dtype).reshape(msg.height, msg.width).astype(np.float32)

    vmin, vmax = np.percentile(values, (1.0, 99.0))
    if vmax <= vmin:
        vmin, vmax = float(values.min()), float(values.max())

    note = ""
    if vmax <= vmin:
        # Flat frame: no gradient to stretch, so every pixel gets the LUT's
        # midpoint color rather than 0 - a solid black frame here would be
        # indistinguishable from "nothing rendered", which is misleading
        # when the topic is fine and simply has a uniform-value source.
        scaled = np.full((msg.height, msg.width), 128, dtype=np.uint8)
        note = f"flat frame (every pixel = {vmin:.0f}) - source has no contrast to color by"
    else:
        scaled = np.clip((values - vmin) * (255.0 / (vmax - vmin)), 0, 255).astype(np.uint8)

    rgb = _THERMAL_LUT[scaled] if _THERMAL_LUT is not None else np.repeat(scaled[:, :, None], 3, axis=2)
    return np.ascontiguousarray(rgb).tobytes(), "", note


def _decode_raw(msg):
    """(rgb_bytes, error, note) from a sensor_msgs/Image - see
    _decode_mono16() for `note`."""
    if msg.width <= 0 or msg.height <= 0:
        return None, "empty frame", ""

    if msg.encoding == "mono16":
        return _decode_mono16(msg)

    info = _ENCODINGS.get(msg.encoding)
    if info is None:
        return None, f"unsupported encoding '{msg.encoding}'", ""
    channels, is_rgb = info

    if msg.step < msg.width * channels:
        return None, "row stride too small", ""

    needed = msg.height * msg.step
    if len(msg.data) < needed:
        return None, "short frame", ""

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=needed)
    rows = raw.reshape(msg.height, msg.step)
    pixels = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)

    if channels == 1:
        rgb = np.repeat(pixels, 3, axis=2)
    else:
        rgb = pixels[:, :, :3]
        if not is_rgb:
            rgb = rgb[:, :, ::-1]

    return np.ascontiguousarray(rgb).tobytes(), "", ""


def _decode_compressed(msg):
    """RGB24 bytes (or None, error) from a sensor_msgs/CompressedImage."""
    if PILImage is None:
        return None, None, None, "Pillow not installed - pip install lazypx4[map]"

    try:
        img = PILImage.open(io.BytesIO(bytes(msg.data))).convert("RGB")
    except Exception as exc:
        return None, None, None, f"couldn't decode: {exc}"

    return img.tobytes(), img.width, img.height, ""


def camera_thread():
    with stats.lock:
        stats.available = camera_available()
        stats.checked = True

    if not stats.available:
        return

    context = rclpy.Context()
    node = None
    executor = None
    subscriptions = [None] * NUM_SLOTS
    rate_bases = [0] * NUM_SLOTS

    def _resolve_kind(topic):
        """Returns "image" or "compressed" for `topic`, from the graph if
        it's already publishing, else a name-based guess (many conventions
        put compressed image transports under a "/compressed" suffix) so a
        topic can still be subscribed to before its publisher starts."""
        try:
            for name, types in node.get_topic_names_and_types():
                if name == topic:
                    if any("CompressedImage" in t for t in types):
                        return "compressed"
                    if any("/Image" in t for t in types):
                        return "image"
        except Exception:
            pass
        return "compressed" if topic.endswith("/compressed") else "image"

    def _make_callback(idx):
        def _on_frame(msg):
            frame_rgb, width, height, error = _decode_compressed(msg)
            with stats.lock:
                slot = stats.slots[idx]
                if error:
                    slot.error = error
                else:
                    slot.frame_rgb = frame_rgb
                    slot.frame_width = width
                    slot.frame_height = height
                    slot.frame_count += 1
                    slot.last_frame_at = time.monotonic()
                    slot.error = ""
        return _on_frame

    def _make_raw_callback(idx):
        def _on_frame(msg):
            frame_rgb, error, note = _decode_raw(msg)
            with stats.lock:
                slot = stats.slots[idx]
                if error:
                    slot.error = error
                    slot.note = ""
                else:
                    slot.frame_rgb = frame_rgb
                    slot.frame_width = msg.width
                    slot.frame_height = msg.height
                    slot.frame_count += 1
                    slot.last_frame_at = time.monotonic()
                    slot.error = ""
                    slot.note = note
        return _on_frame

    def _subscribe(idx, topic):
        if subscriptions[idx] is not None:
            node.destroy_subscription(subscriptions[idx])
            subscriptions[idx] = None

        with stats.lock:
            stats.slots[idx] = CameraSlot(topic=topic, subscribed_topic=topic)

        if not topic:
            return

        kind = _resolve_kind(topic)
        msg_cls = CompressedImage if kind == "compressed" else Image
        callback = _make_callback(idx) if kind == "compressed" else _make_raw_callback(idx)
        subscriptions[idx] = node.create_subscription(msg_cls, topic, callback, qos_profile_sensor_data)

        with stats.lock:
            stats.slots[idx].msg_kind = kind

        rate_bases[idx] = 0

    try:
        rclpy.init(args=None, context=context)
        node = Node("lazypx4_camera_reader", context=context)

        # One long-lived executor rather than rclpy.spin_once()'s free
        # function - see lazypx4.lidar's lidar_thread for why (a
        # shutdown-order bug in a fresh temporary executor's own __del__).
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        for idx, topic in enumerate(_configured_topics()):
            _subscribe(idx, topic)

        last_rate_time = time.monotonic()

        while not shutdown_event.is_set():
            wanted = _configured_topics()
            with stats.lock:
                current = [slot.subscribed_topic for slot in stats.slots]
            for idx in range(NUM_SLOTS):
                if wanted[idx] != current[idx]:
                    _subscribe(idx, wanted[idx])

            executor.spin_once(timeout_sec=0.2)

            now = time.monotonic()
            if now - last_rate_time >= 1.0:
                with stats.lock:
                    for idx, slot in enumerate(stats.slots):
                        slot.fps = float(slot.frame_count - rate_bases[idx]) / (now - last_rate_time)
                        rate_bases[idx] = slot.frame_count
                last_rate_time = now
    except Exception as exc:
        log_warn(f"Camera preview unavailable: {exc}")
    finally:
        if executor is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown(context=context)
        except Exception:
            pass
