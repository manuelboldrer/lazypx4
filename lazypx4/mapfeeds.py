"""ROS 2 feeds drawn on the [n] mission map: the planned path
(``settings.navpath_topic``, ``nav_msgs/Path``, ``/navsat_utm_path`` by
default) and the fire fix (``GPS_PUBLISH_TOPIC``, ``sensor_msgs/NavSatFix``,
``/fire_gps_loc``).

The path is in the ROS ``map`` frame, i.e. ENU relative to the PX4 local
origin (``x`` = East, ``y`` = North) - the same origin the map's local NED
frame uses, so a pose maps straight to ``(north, east) = (y, x)``. The fire
fix is global lat/lon/alt and is projected onto the map by the renderer with
the map's own local origin.

Note the upstream node publishes the path once per plan (not latched), so this
subscriber only sees it if it is running when that happens.

Runs its own rclpy node in its own ``rclpy.Context()``, independent of the
other background ROS threads - see :mod:`lazypx4.lidar` for why - and uses a
long-lived ``SingleThreadedExecutor`` for the same shutdown-order-bug reason.

Optional: if rclpy or the message packages can't be imported (no ROS 2
environment sourced) this thread returns immediately and the map just says so.
"""

from __future__ import annotations

import time

from .config import GPS_PUBLISH_TOPIC, settings
from .eventlog import log_warn
from .state import shutdown_event, state

try:
    import rclpy
    from nav_msgs.msg import Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
except Exception:
    rclpy = None
    Path = None
    SingleThreadedExecutor = None
    Node = None
    NavSatFix = None


def mapfeeds_available():
    return rclpy is not None and Path is not None and NavSatFix is not None


def mapfeeds_thread():
    if not mapfeeds_available():
        return

    context = rclpy.Context()
    node = None
    executor = None
    warned = False

    def _on_path(msg):
        # ENU map frame -> local (north, east).
        points = [(p.pose.position.y, p.pose.position.x) for p in msg.poses]

        with state.lock:
            state.navpath_points = points
            state.navpath_frame_id = msg.header.frame_id
            state.navpath_last_received = time.monotonic()

    def _on_fire(msg):
        with state.lock:
            state.fire_lat = msg.latitude
            state.fire_lon = msg.longitude
            state.fire_alt = msg.altitude
            state.fire_last_received = time.monotonic()

    try:
        rclpy.init(args=None, context=context)
        node = Node("lazypx4_mapfeeds", context=context)
        # Default (reliable, volatile) QoS: matches the publishers' defaults.
        node.create_subscription(Path, settings.navpath_topic, _on_path, 10)
        node.create_subscription(NavSatFix, GPS_PUBLISH_TOPIC, _on_fire, 10)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        with state.lock:
            state.mapfeeds_supported = True

        while not shutdown_event.is_set():
            try:
                executor.spin_once(timeout_sec=0.2)
            except Exception as exc:
                if not warned:
                    warned = True
                    log_warn(f"Map ROS feeds read error: {exc}")
                time.sleep(0.2)
    except Exception as exc:
        log_warn(f"Map ROS feeds unavailable: {exc}")
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
