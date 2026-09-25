"""LiDAR point-cloud overview: a lightweight summary of ``settings.lidar_topic``
(any ``sensor_msgs/PointCloud2`` topic - Livox's ``/livox/points`` by
default, but not specific to it) plus a decimated (x, y, z) sample for the
[v] screen's scatter views - drawn in the sensor's own frame
(``msg.header.frame_id``), not the vehicle's NED frame the [n] map uses,
since lazypx4 doesn't know the LiDAR's mount offset/orientation on the
airframe.

The [v] screen's [t] key changes ``settings.lidar_topic`` at runtime (a
plain string swap, safe to read from this thread without a lock under the
GIL); the loop below notices and re-subscribes without restarting this
thread's node/executor.

Runs its own rclpy node in its own ``rclpy.Context()``, independent of
:mod:`lazypx4.rosclock`'s node/context, so the two background ROS threads
can each start, run and shut down without interfering with the other's
lifecycle - the *implicit default* rclpy context can only be initialized
once per process, but two callers each supplying their own ``Context``
coexist fine.

Optional: if rclpy, sensor_msgs, or numpy can't be imported (no ROS 2
environment sourced, or numpy missing) this module's thread returns
immediately and the point-cloud screen just says so.
"""

from __future__ import annotations

import time

from .config import settings
from .eventlog import log_warn
from .state import shutdown_event, state

try:
    import numpy as np
except Exception:
    np = None

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
except Exception:
    rclpy = None
    SingleThreadedExecutor = None
    Node = None
    qos_profile_sensor_data = None
    PointCloud2 = None

#: How many points of a (possibly much larger) scan to keep for the
#: overview's range stats and scatter view - a modern LiDAR can publish tens
#: of thousands of points per message, more than is useful (or fast) to
#: decode every callback. The scatter view packs points into Braille
#: characters (8 sub-cell dots each - see lazypx4.render.pointcloud), which
#: is why this can afford to keep a lot more than a plain one-dot-per-
#: character grid could usefully show.
MAX_SCATTER_POINTS = 3000

#: PointField.datatype (sensor_msgs) -> numpy dtype string.
_DATATYPE_NP = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8",
}


def lidar_available():
    return rclpy is not None and PointCloud2 is not None and np is not None


def _decode_points(msg, max_points=MAX_SCATTER_POINTS):
    """(point_count, xs, ys, zs) - the full message's point count, and
    plain-Python coordinate lists decimated to at most `max_points` and with
    any non-finite (NaN/Inf) returns dropped. ``(0, [], [], [])`` if the
    message has no usable x/y/z fields or points.
    """
    if np is None:
        return 0, [], [], []

    fields = {f.name: f for f in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return 0, [], [], []

    point_count = msg.width * msg.height
    if point_count <= 0 or msg.point_step <= 0:
        return 0, [], [], []

    endian = ">" if msg.is_bigendian else "<"

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=point_count * msg.point_step)
    raw = raw.reshape(point_count, msg.point_step)

    stride = max(1, point_count // max_points)
    raw = raw[::stride]

    def column(name):
        field = fields[name]
        code = _DATATYPE_NP.get(field.datatype, "f4")
        width = np.dtype(code).itemsize
        chunk = np.ascontiguousarray(raw[:, field.offset:field.offset + width])
        return chunk.view(endian + code).reshape(-1).astype(float)

    xs, ys, zs = column("x"), column("y"), column("z")
    finite = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)

    return point_count, xs[finite].tolist(), ys[finite].tolist(), zs[finite].tolist()


def lidar_thread():
    if not lidar_available():
        return

    context = rclpy.Context()
    node = None
    executor = None
    subscription = None
    subscribed_topic = None
    warned = False
    message_count = 0

    def _on_cloud(msg):
        nonlocal message_count
        message_count += 1

        try:
            point_count, xs, ys, zs = _decode_points(msg)
        except Exception:
            return

        ranges = [
            (x * x + y * y + z * z) ** 0.5 for x, y, z in zip(xs, ys, zs)
        ]

        with state.lock:
            state.lidar_frame_id = msg.header.frame_id
            state.lidar_point_count = point_count
            state.lidar_sample_x = xs
            state.lidar_sample_y = ys
            state.lidar_sample_z = zs
            state.lidar_range_min = min(ranges) if ranges else 0.0
            state.lidar_range_max = max(ranges) if ranges else 0.0
            state.lidar_last_received = time.monotonic()

    def _subscribe(topic):
        """(Re-)subscribe to `topic`, dropping any previous subscription and
        the last message's stats - they belonged to a different topic and
        would otherwise sit on screen looking like data from the new one."""
        nonlocal subscription, subscribed_topic, message_count

        if subscription is not None:
            node.destroy_subscription(subscription)

        subscription = node.create_subscription(
            PointCloud2, topic, _on_cloud, qos_profile_sensor_data,
        )
        subscribed_topic = topic
        message_count = 0

        with state.lock:
            state.lidar_frame_id = ""
            state.lidar_point_count = 0
            state.lidar_rate_hz = 0.0
            state.lidar_last_received = 0.0
            state.lidar_sample_x = []
            state.lidar_sample_y = []
            state.lidar_sample_z = []
            state.lidar_range_min = 0.0
            state.lidar_range_max = 0.0

    try:
        rclpy.init(args=None, context=context)
        node = Node("lazypx4_lidar_reader", context=context)

        # One long-lived executor, not the `rclpy.spin_once(node, ...)` free
        # function - that convenience wrapper builds and tears down a fresh
        # temporary executor on *every* call, which on this context/rclpy
        # combination hits a shutdown-order bug in the executor's own
        # __del__ (logged as "Exception ignored in ... AttributeError:
        # 'SingleThreadedExecutor' object has no attribute '_sigint_gc'").
        # That's harmless to state (the exception is swallowed by Python's
        # __del__ handling) but prints to stderr on every ~0.2s poll - a
        # background thread spamming the terminal while the TUI owns the
        # screen is exactly the kind of thing that can visually corrupt it.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        _subscribe(settings.lidar_topic)

        with state.lock:
            state.lidar_supported = True

        last_rate_count = 0
        last_rate_time = time.monotonic()

        while not shutdown_event.is_set():
            if settings.lidar_topic != subscribed_topic:
                _subscribe(settings.lidar_topic)
                last_rate_count = message_count
                last_rate_time = time.monotonic()

            try:
                executor.spin_once(timeout_sec=0.2)
            except Exception as exc:
                if not warned:
                    warned = True
                    log_warn(f"LiDAR point-cloud read error: {exc}")
                time.sleep(0.2)
                continue

            now = time.monotonic()
            if now - last_rate_time >= 1.0:
                with state.lock:
                    state.lidar_rate_hz = float(message_count - last_rate_count)
                last_rate_count = message_count
                last_rate_time = now
    except Exception as exc:
        log_warn(f"LiDAR point-cloud unavailable: {exc}")
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
