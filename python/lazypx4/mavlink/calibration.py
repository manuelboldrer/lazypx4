"""Sensor calibration (MAV_CMD_PREFLIGHT_CALIBRATION).

PX4 runs gyro / accel / level / magnetometer / baro calibration entirely
on-board: we send one COMMAND_LONG and then follow the "[cal] ..." STATUSTEXT
messages it streams back for progress and for the "place the vehicle
nose-down" style prompts. Accel and mag need the operator to physically move
the airframe; gyro / level / baro just need it to sit still. Must be done
DISARMED.
"""

from __future__ import annotations

import re
import time

from ..config import (
    CALIBRATION_LABELS,
    CALIBRATION_PARAMS,
    MAV_CMD_PREFLIGHT_CALIBRATION,
)
from ..eventlog import log_command, log_error, log_warn
from ..state import state
from ..util import clamp, safe_int
from .connection import vehicle_ready


def handle_calibration_text(text, now):
    """Fold a "[cal]" STATUSTEXT line into the calibration state."""
    lower = text.lower()

    with state.lock:
        state.cal_last_message = text
        state.cal_last_message_at = now
        state.cal_log.append(text)

        match = re.search(r"progress\s*<?\s*(\d+)", lower)
        if match:
            state.cal_progress = clamp(safe_int(match.group(1)), 0, 100)

        if "calibration started" in lower:
            state.cal_active = True
            state.cal_progress = 0
            state.cal_result = ""
            state.cal_started_at = now
        elif "calibration done" in lower:
            state.cal_active = False
            state.cal_progress = 100
            state.cal_result = "DONE"
        elif "calibration failed" in lower:
            state.cal_active = False
            state.cal_result = "FAILED"
        elif "calibration cancel" in lower:
            state.cal_active = False
            state.cal_result = "CANCELLED"


def send_calibration(master, cal_type):
    if not vehicle_ready(master):
        return False

    params = CALIBRATION_PARAMS.get(cal_type)
    if params is None:
        log_error(f"Unknown calibration type: {cal_type}")
        return False

    with state.lock:
        if state.armed:
            log_error("Calibration refused: vehicle is ARMED")
            return False
        if state.cal_active:
            log_warn("A calibration is already running")
            return False

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            MAV_CMD_PREFLIGHT_CALIBRATION,
            0,
            *[float(p) for p in params],
        )
    except Exception as exc:
        log_error(f"Failed to send calibration command: {exc}")
        return False

    with state.lock:
        now = time.monotonic()
        state.cal_active = True
        state.cal_type = cal_type
        state.cal_progress = 0
        state.cal_result = ""
        state.cal_started_at = now
        state.cal_sent_at = now
        state.cal_ack = ""
        state.cal_ack_at = 0.0
        state.cal_statustext_at_start = state.statustext_count
        state.cal_event_at_start = state.event_count
        state.cal_last_message = ""
        state.cal_log.clear()

    log_command(
        f"PREFLIGHT_CALIBRATION SENT: {CALIBRATION_LABELS.get(cal_type, cal_type)} "
        f"(sys {master.target_system} comp {master.target_component})"
    )
    return True


def cancel_calibration(master):
    if master is None:
        return False

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            MAV_CMD_PREFLIGHT_CALIBRATION,
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
    except Exception as exc:
        log_error(f"Failed to cancel calibration: {exc}")
        return False

    with state.lock:
        state.cal_active = False
        state.cal_result = "CANCELLED"

    log_command("Calibration CANCEL sent")
    return True


def check_calibration():
    """Give up on a calibration that PX4 never acknowledged or started.

    A real calibration always produces either a COMMAND_ACK or a stream of
    "[cal]" STATUSTEXT messages within a few seconds. If neither happens the
    command did not land - clear the "RUNNING" state so the screen is usable
    again instead of hanging forever.
    """
    now = time.monotonic()

    with state.lock:
        if not state.cal_active:
            return

        sent_at = state.cal_sent_at
        have_ack = bool(state.cal_ack)
        have_log = bool(state.cal_log)

        if not sent_at:
            return
        if have_ack or have_log:
            return
        if now - sent_at < 25.0:
            return

        state.cal_active = False
        state.cal_result = "NO RESPONSE"

    log_error(
        "Calibration: PX4 sent no ACK and no [cal] messages in 25 s - "
        "command was not accepted. Check [g] event log and PX4 firmware."
    )
