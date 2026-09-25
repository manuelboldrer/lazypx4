"""ROS 2 feeds drawn on the [n] mission map: the planned path
(``settings.navpath_topic``, ``nav_msgs/Path``, ``/navsat_utm_path`` by
default) and the fire fix (``GPS_PUBLISH_TOPIC``, ``sensor_msgs/NavSatFix``,
``/fire_gps_loc``).

The path is in the ROS ``map`` frame, i.e. ENU relative to the PX4 local
origin (``x`` = East, ``y`` = North) - the same origin the map's local NED
frame uses, so a pose maps straight to ``(north, east) = (y, x)``. The fire
fix is global lat/lon/alt and is projected onto the map by the renderer with
the map's own local origin.

Both topics are subscribed twice, RELIABLE and BEST_EFFORT: a reliable
subscription never matches a best-effort publisher (e.g. a detector using the
sensor-data QoS) and ROS reports nothing when that happens. A reliable
publisher matches both, which is harmless - each message just sets the same
value twice. The path gets a third, RELIABLE + TRANSIENT_LOCAL subscription so
a latched path published once before lazypx4 started is still delivered (a
volatile subscriber never gets that stored message). `ros2 topic echo` adapts
its QoS to the publisher, so it can show a topic these fixed-QoS
subscriptions would otherwise miss.

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

#: Why the imports below failed, shown on the map screen instead of a bare
#: "unavailable".
IMPORT_ERROR = ""

try:
    import rclpy
    from nav_msgs.msg import Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix

    # rclpy.event_handler is Iron+; Humble has the same class in qos_event.
    try:
        from rclpy.event_handler import SubscriptionEventCallbacks
    except ImportError:
        from rclpy.qos_event import SubscriptionEventCallbacks
except Exception as exc:
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
    rclpy = None
    Path = None
    SingleThreadedExecutor = None
    Node = None
    SubscriptionEventCallbacks = None
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    NavSatFix = None

#: How often the fire topic's publishers are re-read from the ROS graph.
_GRAPH_POLL_S = 1.0

#: Our own [P] publisher (lazypx4.gpspub), left out of the publisher list.
_OWN_PUBLISHER_NODE = "lazypx4_gps_publisher"


def mapfeeds_available():
    return rclpy is not None and Path is not None and NavSatFix is not None


def _topic_publishers(node, topic):
    publishers = []
    for info in node.get_publishers_info_by_topic(topic):
        if info.node_name == _OWN_PUBLISHER_NODE:
            continue
        reliability = info.qos_profile.reliability
        qos = "best-effort" if reliability == ReliabilityPolicy.BEST_EFFORT else "reliable"
        if info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL:
            qos += ", latched"
        publishers.append(f"{info.node_name} ({qos})")
    return sorted(publishers)


def mapfeeds_thread():
    if not mapfeeds_available():
        return

    context = rclpy.Context()
    node = None
    executor = None
    warned = False

    def _on_path(msg):
        # ENU map frame -> local (north, east). Consecutive repeats (the
        # planner pads the path with its final pose) are dropped.
        points = []
        for p in msg.poses:
            pt = (p.pose.position.y, p.pose.position.x)
            if not points or points[-1] != pt:
                points.append(pt)

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
        # The reliable ones are expected to be incompatible with best-effort
        # publishers (and the transient-local one with volatile publishers);
        # replace rclpy's default handler, which would print that warning
        # across the TUI.
        quiet = SubscriptionEventCallbacks(incompatible_qos=lambda _event: None)
        node.create_subscription(Path, settings.navpath_topic, _on_path, 10, event_callbacks=quiet)
        node.create_subscription(
            Path, settings.navpath_topic, _on_path,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            event_callbacks=quiet,
        )
        node.create_subscription(
            Path, settings.navpath_topic, _on_path,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        node.create_subscription(
            NavSatFix, GPS_PUBLISH_TOPIC, _on_fire, 10,
            event_callbacks=quiet,
        )
        node.create_subscription(
            NavSatFix, GPS_PUBLISH_TOPIC, _on_fire,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        with state.lock:
            state.mapfeeds_supported = True

        next_graph_poll = 0.0

        while not shutdown_event.is_set():
            try:
                if time.monotonic() >= next_graph_poll:
                    next_graph_poll = time.monotonic() + _GRAPH_POLL_S
                    fire_publishers = _topic_publishers(node, GPS_PUBLISH_TOPIC)
                    navpath_publishers = _topic_publishers(node, settings.navpath_topic)
                    with state.lock:
                        state.fire_publishers = fire_publishers
                        state.navpath_publishers = navpath_publishers
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
