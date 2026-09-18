"""One-shot GPS position publish ([P] on the map screen).

Publishes the vehicle's current global position (``GLOBAL_POSITION_INT``,
the same fix the dashboard and map screen show) as a single
``sensor_msgs/NavSatFix`` message on ``config.GPS_PUBLISH_TOPIC`` - the
rclpy equivalent of:

    ros2 topic pub /fire_gps_loc sensor_msgs/msg/NavSatFix "{...}" --once

but filled in with the vehicle's actual position at the moment [P] is
pressed, rather than a value typed by hand. See
:func:`lazypx4.navigation.publish_gps_fix` for the keypress handler.

Runs its own rclpy node in its own ``rclpy.Context()``, independent of
:mod:`lazypx4.rosclock`'s node/context (see that reasoning in
:mod:`lazypx4.lidar`) - kept running for the app's whole lifetime, publisher
created once at startup, rather than a fresh node per keypress, so a
subscriber that's already listening has had time to discover this publisher
before the first (possibly only) message it ever sends. Uses a long-lived
``SingleThreadedExecutor`` rather than the ``rclpy.spin_once(node, ...)``
free function for the same shutdown-order-bug reason documented in
:mod:`lazypx4.lidar`.

Optional: if rclpy or sensor_msgs can't be imported (no ROS 2 environment
sourced) this module's thread returns immediately and [P] logs an error
instead of publishing.
"""

from __future__ import annotations

import threading
import time

from .config import GPS_PUBLISH_FRAME_ID, GPS_PUBLISH_TOPIC
from .eventlog import log_command, log_warn
from .state import shutdown_event, state

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
except Exception:
    rclpy = None
    SingleThreadedExecutor = None
    Node = None
    NavSatFix = None

#: How often the idle loop services the node / checks for a publish request.
_POLL_S = 0.2

_publish_requested = threading.Event()


def gps_pub_available():
    return rclpy is not None and NavSatFix is not None


def _set_feedback(message, ok):
    """Set the [P] on-screen banner (see lazypx4.render.mapview), cleared
    after config.GPS_PUBLISH_FEEDBACK_TIMEOUT_S by lazypx4.app.update_health
    - the same expire-after-N-seconds pattern as the dashboard's PREARM
    banner, the only other precedent for it in this codebase.
    """
    with state.lock:
        state.gps_pub_feedback = message
        state.gps_pub_feedback_ok = ok
        state.gps_pub_feedback_time = time.monotonic()


def request_gps_publish():
    """Ask the background node to publish the current position - called
    from the [P] keypress. Returns True if the request was accepted; the
    actual publish (and its own log line / banner update) happens
    asynchronously on the node's thread, the same fire-and-forget shape as
    the satellite-image download / speed-test keypresses.
    """
    if not gps_pub_available():
        message = "GPS publish: rclpy / sensor_msgs not found - no ROS 2 environment sourced"
        log_warn(message)
        _set_feedback(message, False)
        return False

    with state.lock:
        valid = state.global_pos_valid

    if not valid:
        message = "GPS publish: no global position yet (need a GPS fix)"
        log_warn(message)
        _set_feedback(message, False)
        return False

    _set_feedback(f"publishing to {GPS_PUBLISH_TOPIC} ...", True)
    _publish_requested.set()
    return True


def _publish_once(node, publisher):
    with state.lock:
        valid = state.global_pos_valid
        lat = state.global_lat
        lon = state.global_lon
        alt = state.global_alt

    if not valid:
        message = "GPS publish: lost the global position before it could be sent"
        log_warn(message)
        _set_feedback(message, False)
        return

    msg = NavSatFix()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = GPS_PUBLISH_FRAME_ID
    msg.status.status = 0  # STATUS_FIX
    msg.status.service = 1  # SERVICE_GPS
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    msg.position_covariance = [0.0] * 9
    msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN

    publisher.publish(msg)
    message = f"published to {GPS_PUBLISH_TOPIC}: {lat:.7f}, {lon:.7f} @ {alt:.2f} m"
    log_command(f"GPS position {message}")
    _set_feedback(message, True)


def gps_pub_thread():
    if not gps_pub_available():
        return

    context = rclpy.Context()
    node = None
    executor = None

    try:
        rclpy.init(args=None, context=context)
        node = Node("lazypx4_gps_publisher", context=context)
        publisher = node.create_publisher(NavSatFix, GPS_PUBLISH_TOPIC, 10)

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        warned = False

        while not shutdown_event.is_set():
            if _publish_requested.is_set():
                _publish_requested.clear()
                try:
                    _publish_once(node, publisher)
                except Exception as exc:
                    message = f"GPS publish failed: {exc}"
                    log_warn(message)
                    _set_feedback(message, False)

            try:
                executor.spin_once(timeout_sec=_POLL_S)
            except Exception as exc:
                if not warned:
                    warned = True
                    log_warn(f"GPS publisher error: {exc}")
                time.sleep(_POLL_S)
    except Exception as exc:
        log_warn(f"GPS publisher unavailable: {exc}")
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
