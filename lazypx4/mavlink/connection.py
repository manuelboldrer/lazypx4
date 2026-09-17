"""Opening the link, announcing ourselves as a GCS, and stream setup."""

from __future__ import annotations

import math
import time

from pymavlink import mavutil

from ..config import ESTIMATOR_PARAM_NAMES, HEARTBEAT_TIMEOUT, STREAM_MESSAGE_INTERVALS, settings
from ..eventlog import log_error, log_info, log_warn
from ..state import state
from ..util import safe_int


def connect_vehicle():
    """Open the UDP link and block until the first PX4 heartbeat.

    Returns the ``mavutil`` connection, or raises on failure.
    """
    log_info(f"Connecting to udpin:0.0.0.0:{settings.port}")

    try:
        # "development" is a superset of "common" (which is all this console
        # otherwise needs) plus the newer Standard Modes Protocol messages
        # (AVAILABLE_MODES / CURRENT_MODE) used to discover PX4 custom /
        # external (ROS 2) flight modes. The dialects normally shipped with
        # pymavlink don't define those messages yet, so without this they are
        # silently ignored.
        master = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{settings.port}",
            autoreconnect=True,
            dialect="development",
        )
    except Exception as exc:
        log_error(f"Connection failed: {exc}")
        raise

    log_info("Waiting for PX4 heartbeat...")

    try:
        master.wait_heartbeat(timeout=10)
    except Exception as exc:
        log_error(f"Heartbeat timeout: {exc}")
        raise

    vehicle_system = safe_int(master.target_system, 0)
    vehicle_component = safe_int(master.target_component, 0)

    with state.lock:
        state.target_system = vehicle_system
        state.target_component = vehicle_component
        state.vehicle_locked = True
        state.connected = True
        state.last_heartbeat = time.monotonic()

    log_info(f"PX4 vehicle locked: system={vehicle_system}, component={vehicle_component}")

    try:
        modes = master.mode_mapping()

        if modes:
            with state.lock:
                state.available_modes = sorted(modes.keys())

            log_info("PX4 modes: " + ", ".join(sorted(modes.keys())))

    except Exception as exc:
        log_warn(f"Could not get mode mapping: {exc}")

    return master


def vehicle_ready(master):
    """True only if we hold a locked identity and recent telemetry.

    Every state-changing command checks this first, so a command is never
    sent to a vehicle whose link has gone stale.
    """
    if master is None:
        log_error("No MAVLink connection")
        return False

    now = time.monotonic()

    with state.lock:
        if not state.vehicle_locked:
            log_error("Vehicle identity is not locked")
            return False

        if state.last_rx <= 0:
            log_error("No vehicle telemetry received")
            return False

        if now - state.last_rx > HEARTBEAT_TIMEOUT:
            log_error("Vehicle telemetry is stale")
            return False

    return True


def configure_streams(master):
    """Ask PX4 for faster estimation/sensor streams than the defaults."""
    if master is None:
        return

    for msg_id, interval_us in STREAM_MESSAGE_INTERVALS.items():
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                float(interval_us),
                0.0, 0.0, 0.0, 0.0, 0.0,
            )
        except Exception:
            pass

    # GPS_GLOBAL_ORIGIN (49) and HOME_POSITION (242) are emitted on change
    # rather than streamed, and AUTOPILOT_VERSION (148) only on request -
    # ask for each once so the origin, home and firmware version show up
    # immediately.
    for msg_id in (49.0, 242.0, 148.0):
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                msg_id,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            )
        except Exception:
            pass

    # Some PX4 MAVLink stream configs ("onboard" / minimal) do not include
    # STATUSTEXT (253) or EVENT (410), which hides calibration "[cal]" prompts
    # and other PX4 notices. Force both on at a real rate; SET_MESSAGE_INTERVAL
    # can add a stream that the mode did not include.
    for msg_id in (253.0, 410.0):
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                100000.0,   # 10 Hz cap; both are event-driven in practice
                0.0, 0.0, 0.0, 0.0, 0.0,
            )
        except Exception:
            pass


_last_gcs_heartbeat = 0.0


def send_gcs_heartbeat(master):
    """Announce this console as a GCS once per second.

    PX4 gates several streams (STATUSTEXT among them, on some builds and
    MAVLink modes) on whether the link has a live GCS heartbeat. Every real
    ground station (QGC, MAVSDK, mavproxy) sends one; without it PX4 can treat
    the link as idle and withhold messages.
    """
    global _last_gcs_heartbeat

    if master is None:
        return

    now = time.monotonic()
    if now - _last_gcs_heartbeat < 1.0:
        return

    _last_gcs_heartbeat = now

    try:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
    except Exception:
        pass


def request_estimator_params(master):
    """Individually read the EKF2_* aiding params for the estimation view."""
    if master is None:
        return

    with state.lock:
        if not state.vehicle_locked:
            return
        state.estimator_params_requested_at = time.monotonic()

    for name in ESTIMATOR_PARAM_NAMES:
        try:
            master.mav.param_request_read_send(
                master.target_system,
                master.target_component,
                name.encode("utf-8")[:16].ljust(16, b"\x00"),
                -1,
            )
        except Exception:
            pass


def request_single_param(master, name):
    """Read one named parameter (PARAM_REQUEST_READ), e.g. before a PARAM_SET
    that needs to know its wire type first - see :func:`get_param_value` and
    ``mavlink.parameters.send_parameter_set``."""
    if master is None:
        return

    try:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            name.encode("utf-8")[:16].ljust(16, b"\x00"),
            -1,
        )
    except Exception:
        pass


def get_param_value(name):
    """Return a received parameter's finite numeric value, or ``None``."""
    with state.lock:
        parameter = state.parameters.get(name)

        if parameter is None:
            return None

        value = parameter.value

    if not math.isfinite(value):
        return None

    return value
