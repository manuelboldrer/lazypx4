"""ROS 2 clock: mirrors whatever a standalone rclpy node reports as "now".

This is deliberately independent of the MAVLink link - it runs its own tiny
rclpy node so the dashboard can show ROS's idea of the current time next to
the autopilot's SYSTEM_TIME, which is useful for spotting a stalled /clock
(sim paused, bridge down) or a wall/sim-time mismatch.

Optional: if rclpy can't be imported (no ROS 2 environment sourced) this
module's thread returns immediately and the dashboard just omits the ROS
reading. Delete this module and the ROS half of the dashboard's TIME line
and nothing else changes.
"""

from __future__ import annotations

import time

from .eventlog import log_warn
from .state import shutdown_event, state

try:
    import rclpy
    from rclpy.node import Node
except Exception:
    rclpy = None
    Node = None

#: How often to poll the node's clock and let it process any /clock
#: subscription (only relevant when the node's use_sim_time param is true).
SAMPLE_INTERVAL = 0.5


def ros_clock_available():
    return rclpy is not None


def ros_clock_thread():
    if rclpy is None:
        return

    node = None
    warned = False

    try:
        rclpy.init(args=None)
        node = Node("lazypx4_clock_reader")
        # rclpy declares use_sim_time on every Node by default - no need to.

        with state.lock:
            state.ros_clock_supported = True

        while not shutdown_event.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=SAMPLE_INTERVAL)
                use_sim_time = bool(node.get_parameter("use_sim_time").value)
                now_ns = node.get_clock().now().nanoseconds

                with state.lock:
                    state.ros_time_sim = use_sim_time
                    if now_ns > 0:
                        state.ros_time_ns = now_ns
                        state.last_ros_time = time.monotonic()
            except Exception as exc:
                if not warned:
                    warned = True
                    log_warn(f"ROS clock read error: {exc}")
                time.sleep(SAMPLE_INTERVAL)
    except Exception as exc:
        log_warn(f"ROS clock unavailable: {exc}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
