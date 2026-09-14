"""Standard Modes Protocol (AVAILABLE_MODES / CURRENT_MODE).

This is how PX4 v1.15+ exposes modes that ``master.mode_mapping()`` can't
see: custom internal modes it doesn't happen to know about, and PX4 ROS 2
"external" modes, which don't exist in any static table at all since they're
registered at runtime by a companion computer. Setting them uses a different
wire encoding than the classic string-based :func:`set_mode`.
"""

from __future__ import annotations

import time

from pymavlink import mavutil

from ..config import AVAILABLE_MODES_TIMEOUT, MAVLINK_MSG_ID_AVAILABLE_MODES
from ..eventlog import log_command, log_error, log_info, log_warn
from ..models import CustomMode
from ..state import session, state
from ..util import safe_int
from .commands import set_mode
from .connection import vehicle_ready


def standard_mode_name(value):
    try:
        entry = mavutil.mavlink.enums["MAV_STANDARD_MODE"][safe_int(value)]
        return entry.name.replace("MAV_STANDARD_MODE_", "")
    except Exception:
        return f"STANDARD_{safe_int(value)}"


def set_standard_mode(master, standard_mode_value, mode_name=""):
    if not vehicle_ready(master):
        return False

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_STANDARD_MODE,
            0,
            float(standard_mode_value),
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )

        log_command(f"DO_SET_STANDARD_MODE command SENT: {mode_name or standard_mode_value}")
        return True

    except Exception as exc:
        log_error(f"Failed to set standard mode {mode_name or standard_mode_value}: {exc}")
        return False


def set_custom_mode(master, custom_mode_value, mode_name=""):
    if not vehicle_ready(master):
        return False

    try:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(base_mode),
            float(custom_mode_value),
            0.0, 0.0, 0.0, 0.0, 0.0,
        )

        log_command(f"DO_SET_MODE command SENT: {mode_name or custom_mode_value}")
        return True

    except Exception as exc:
        log_error(f"Failed to set custom mode {mode_name or custom_mode_value}: {exc}")
        return False


def request_available_modes(master):
    if not vehicle_ready(master):
        return False

    try:
        with state.lock:
            state.custom_modes.clear()
            state.custom_modes_order.clear()
            state.custom_modes_total = 0
            state.custom_modes_received = 0
            state.custom_modes_complete = False
            state.custom_modes_requested_at = time.monotonic()

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(MAVLINK_MSG_ID_AVAILABLE_MODES),
            0.0,  # param2 = 0 -> emit for ALL available modes
            0.0, 0.0, 0.0, 0.0, 0.0,
        )

        log_command("AVAILABLE_MODES request SENT")
        return True

    except Exception as exc:
        with state.lock:
            state.custom_modes_requested_at = 0.0

        log_error(f"Failed to request AVAILABLE_MODES: {exc}")
        return False


def handle_available_modes(msg):
    number_modes = safe_int(getattr(msg, "number_modes", 0))
    mode_index = safe_int(getattr(msg, "mode_index", 0))
    standard_mode = safe_int(getattr(msg, "standard_mode", 0))
    custom_mode = safe_int(getattr(msg, "custom_mode", 0))
    properties = safe_int(getattr(msg, "properties", 0))

    name = getattr(msg, "mode_name", "")

    if isinstance(name, bytes):
        name = name.decode("utf-8", errors="replace")

    name = str(name).rstrip("\x00").strip()

    if not name:
        name = f"MODE_{custom_mode}"

    with state.lock:
        if mode_index not in state.custom_modes:
            state.custom_modes_order.append(mode_index)
            state.custom_modes_order.sort()

        state.custom_modes[mode_index] = CustomMode(
            mode_index=mode_index,
            custom_mode=custom_mode,
            standard_mode=standard_mode,
            properties=properties,
            name=name,
        )

        state.custom_modes_received = len(state.custom_modes)

        if number_modes > 0:
            state.custom_modes_total = number_modes

        if number_modes > 0 and state.custom_modes_received >= number_modes:
            state.custom_modes_complete = True
            state.custom_modes_requested_at = 0.0


def handle_current_mode(msg):
    standard_mode = safe_int(getattr(msg, "standard_mode", 0))
    custom_mode = safe_int(getattr(msg, "custom_mode", 0))

    with state.lock:
        state.current_standard_mode = standard_mode
        state.current_custom_mode_id = custom_mode


def handle_available_modes_monitor(msg):
    seq = safe_int(getattr(msg, "seq", -1), -1)

    with state.lock:
        previous_seq = state.available_modes_seq
        state.available_modes_seq = seq

        changed = previous_seq not in (-1, seq)

    if changed:
        log_info("PX4 available modes changed; refreshing")
        request_available_modes(session.link)


def check_available_modes_request():
    """Time out an AVAILABLE_MODES enumeration PX4 never finished."""
    now = time.monotonic()

    with state.lock:
        requested_at = state.custom_modes_requested_at
        complete = state.custom_modes_complete
        received = state.custom_modes_received
        total = state.custom_modes_total

    if not requested_at or complete:
        return

    if total > 0 and received >= total:
        with state.lock:
            state.custom_modes_complete = True
            state.custom_modes_requested_at = 0.0
        return

    if now - requested_at > AVAILABLE_MODES_TIMEOUT:
        with state.lock:
            state.custom_modes_requested_at = 0.0

        log_warn(
            f"AVAILABLE_MODES request timed out. Received {received}"
            + (f"/{total}" if total else "")
        )


def get_mode_options():
    """Build the selectable-mode list for the mode-select screen.

    Prefers the Standard Modes Protocol whenever it has produced anything
    (it can report custom / PX4-ROS2 external modes the legacy static
    ``mode_mapping()`` table does not know exist), and falls back to the
    legacy list only if it has produced nothing (PX4 older than v1.15).
    """
    with state.lock:
        custom_entries = [
            state.custom_modes[index]
            for index in state.custom_modes_order
            if index in state.custom_modes
        ]

        legacy_modes = list(state.available_modes)

    options = []

    if custom_entries:
        for entry in custom_entries:
            if entry.properties & mavutil.mavlink.MAV_MODE_PROPERTY_NOT_USER_SELECTABLE:
                continue

            if entry.standard_mode != mavutil.mavlink.MAV_STANDARD_MODE_NON_STANDARD:
                options.append({
                    "label": f"{entry.name} [{standard_mode_name(entry.standard_mode)}]",
                    "kind": "standard",
                    "value": entry.standard_mode,
                })
            else:
                options.append({
                    "label": f"{entry.name} [CUSTOM]",
                    "kind": "custom",
                    "value": entry.custom_mode,
                })
    else:
        for mode in legacy_modes:
            options.append({"label": mode, "kind": "legacy", "value": mode})

    return options


def confirm_mode_option(option):
    kind = option["kind"]
    value = option["value"]
    label = option["label"]

    if kind == "legacy":
        set_mode(session.link, value)
    elif kind == "standard":
        set_standard_mode(session.link, value, label)
    elif kind == "custom":
        set_custom_mode(session.link, value, label)
