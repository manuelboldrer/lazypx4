"""Arm/disarm, flight-mode set, hold and reboot command senders.

Every sender is gated by :func:`vehicle_ready` and returns a bool. Arm state
is *never* inferred from the ACK - it is only trusted once a HEARTBEAT
confirms it (see :func:`check_pending_arm` and the heartbeat handler).
"""

from __future__ import annotations

import time

from pymavlink import mavutil

from ..config import ARM_CONFIRM_TIMEOUT
from ..eventlog import log_command, log_error, log_warn
from ..models import PendingArm
from ..state import state
from .connection import vehicle_ready


def send_arm(master, arm: bool):
    desired = bool(arm)

    if not vehicle_ready(master):
        return False

    with state.lock:
        current = state.armed

        if current == desired:
            log_command(
                ("ARM" if desired else "DISARM")
                + " requested, but vehicle already reports "
                + ("ARMED" if current else "DISARMED")
            )
            return True

        if state.pending_arm is not None:
            log_warn("ARM/DISARM command already pending")
            return False

        state.pending_arm = PendingArm(desired_armed=desired, sent_at=time.monotonic())

    try:
        command_value = 1.0 if desired else 0.0

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            command_value,
            0.0,  # force: never force-arm
            0.0, 0.0, 0.0, 0.0, 0.0,
        )

        action = "ARM" if desired else "DISARM"
        log_command(f"{action} command SENT")
        log_command(f"{action}: waiting for HEARTBEAT confirmation")
        return True

    except Exception as exc:
        with state.lock:
            state.pending_arm = None
        log_error(f"Could not send {'ARM' if desired else 'DISARM'}: {exc}")
        return False


def set_mode(master, requested_mode):
    """Set a legacy (string-named) PX4 flight mode via ``master.set_mode``."""
    if not vehicle_ready(master):
        return False

    requested_mode = str(requested_mode)

    try:
        mapping = master.mode_mapping()

        if mapping and requested_mode not in mapping:
            log_error(f"PX4 does not report mode '{requested_mode}'")
            return False

        master.set_mode(requested_mode)
        log_command(f"MODE command SENT: {requested_mode}")
        return True

    except Exception as exc:
        log_error(f"Failed to set mode {requested_mode}: {exc}")
        return False


def send_hold(master):
    """Best-effort "stop and loiter": a HOLD-family mode, else DO_PAUSE_CONTINUE."""
    if not vehicle_ready(master):
        return False

    with state.lock:
        modes = list(state.available_modes)

    for requested_mode in ("HOLD", "AUTO.LOITER", "LOITER"):
        if requested_mode in modes and set_mode(master, requested_mode):
            log_command(f"HOLD using PX4 mode {requested_mode}")
            return True

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
            0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        log_command("HOLD command SENT using MAV_CMD_DO_PAUSE_CONTINUE")
        return True
    except Exception as exc:
        log_error(f"Failed to send HOLD: {exc}")
        return False


def send_kill(master):
    """Force-terminate the flight NOW - cuts motors even mid-flight.

    Distinct from :func:`send_arm`\\ (False): PX4 can refuse a plain disarm
    in flight for safety, but MAV_CMD_DO_FLIGHTTERMINATION is the same
    "kill switch" QGroundControl exposes and is not gated on being on the
    ground. Only ever send this as a real emergency stop.
    """
    if not vehicle_ready(master):
        return False

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        log_warn("KILL command SENT - flight termination")
        return True
    except Exception as exc:
        log_error(f"Failed to send KILL: {exc}")
        return False


def send_set_home_current(master):
    """Mark the vehicle's current position as home (MAV_CMD_DO_SET_HOME)."""
    if not vehicle_ready(master):
        return False

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        log_command("SET HOME command SENT: current position")
        return True
    except Exception as exc:
        log_error(f"Failed to set home: {exc}")
        return False


def send_fence_enable(master, enable: bool):
    if not vehicle_ready(master):
        return False

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
            0, 1.0 if enable else 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        log_command(f"GEOFENCE {'ENABLE' if enable else 'DISABLE'} command SENT")
        return True
    except Exception as exc:
        log_error(f"Failed to {'enable' if enable else 'disable'} geofence: {exc}")
        return False


def send_reboot(master):
    if not vehicle_ready(master):
        return False

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        log_command("PX4 REBOOT command SENT")
        log_warn("PX4 is rebooting; MAVLink connection will temporarily disappear")
        return True
    except Exception as exc:
        log_error(f"Failed to send PX4 reboot: {exc}")
        return False


def check_pending_arm():
    """Give up on an ARM/DISARM that no HEARTBEAT has confirmed in time."""
    now = time.monotonic()

    with state.lock:
        pending = state.pending_arm

        if pending is None:
            return

        if now - pending.sent_at < pending.timeout:
            return

        desired = pending.desired_armed
        actual = state.armed
        last_status = state.last_status_text

        state.pending_arm = None

    action = "ARM" if desired else "DISARM"

    if actual == desired:
        log_command(f"{action} confirmed by vehicle state")
        return

    if last_status:
        log_error(
            f"{action} was not confirmed within {ARM_CONFIRM_TIMEOUT:.1f}s. "
            f"Last PX4 status: {last_status}"
        )
    else:
        log_error(f"{action} was not confirmed within {ARM_CONFIRM_TIMEOUT:.1f}s")
