"""PX4 parameter protocol: browse, filter, edit.

The awkward part is the float<->int reinterpretation. PARAM_VALUE / PARAM_SET
only carry a 4-byte float field; for non-REAL32/REAL64 parameters PX4 does
*not* convert the integer to its numeric float equivalent - it reinterprets
the raw 4 bytes of an int32/uint32 as a float ("union cast"). We must reverse
that exact operation or NaN/Inf bit patterns (very common for e.g. big
uint32 device IDs) get treated as literal float NaN/Inf.
"""

from __future__ import annotations

import json
import math
import os
import struct
import time

from pymavlink import mavutil

from ..config import (
    PARAM_REQUEST_TIMEOUT,
    PARAM_SET_TIMEOUT,
    PARAM_TYPE_NAMES,
    PARAM_VALUE_EPSILON,
    settings,
)
from ..eventlog import log_command, log_error, log_info, log_warn
from ..models import Parameter
from ..state import session, state
from ..util import safe_float, safe_int
from .connection import vehicle_ready


def parameter_name_from_message(msg):
    name = getattr(msg, "param_id", "")

    if isinstance(name, bytes):
        name = name.decode("utf-8", errors="replace")

    return str(name).rstrip("\x00").strip()


def parameter_type_name(param_type):
    return PARAM_TYPE_NAMES.get(safe_int(param_type), f"TYPE_{safe_int(param_type)}")


def parameter_is_integer(param_type):
    return safe_int(param_type) in (1, 2, 3, 4, 5, 6, 7, 8)


def parameter_is_unsigned(param_type):
    return safe_int(param_type) in (1, 3, 5, 7)


def decode_param_float_as_number(raw_value, param_type):
    raw_value = safe_float(raw_value, 0.0)

    if not parameter_is_integer(param_type):
        return raw_value

    try:
        packed = struct.pack("<f", raw_value)

        if parameter_is_unsigned(param_type):
            return float(struct.unpack("<I", packed)[0])

        return float(struct.unpack("<i", packed)[0])

    except Exception:
        return raw_value


def encode_number_as_param_float(value, param_type):
    if not parameter_is_integer(param_type):
        return float(value)

    try:
        int_value = int(round(value))

        if parameter_is_unsigned(param_type):
            packed = struct.pack("<I", int_value & 0xFFFFFFFF)
        else:
            packed = struct.pack("<i", int_value)

        return struct.unpack("<f", packed)[0]

    except Exception:
        return float(value)


def parameter_format_value(parameter):
    value = parameter.value

    try:
        if parameter_is_integer(parameter.param_type):
            if not math.isfinite(value):
                return "N/A"
            return str(int(round(value)))

        if parameter.param_type == 9:
            return f"{value:.6g}"

        if parameter.param_type == 10:
            return f"{value:.9g}"

        return f"{value:.6g}"

    except (ValueError, OverflowError):
        return "N/A"


def parse_parameter_value(text, parameter):
    text = str(text).strip()

    if not text:
        raise ValueError("Empty parameter value")

    value = float(text)

    if not math.isfinite(value):
        raise ValueError("Parameter value must be finite")

    if parameter_is_integer(parameter.param_type) and not value.is_integer():
        raise ValueError("This parameter requires an integer value")

    return value


def load_parameter_defaults():
    """Load a JSON file of PX4 parameter defaults, if one was configured."""
    path = settings.param_defaults_file

    if not path:
        return

    if not os.path.isfile(path):
        log_warn(f"PX4 parameter defaults file not found: {path}")
        return

    try:
        with open(path, "r", encoding="utf-8") as file:
            data = json.load(file)

        defaults = {}

        if isinstance(data, dict):
            if isinstance(data.get("parameters"), dict):
                for name, entry in data["parameters"].items():
                    if isinstance(entry, dict):
                        if "default" not in entry:
                            continue
                        value = entry["default"]
                    else:
                        value = entry

                    try:
                        defaults[str(name)] = float(value)
                    except Exception:
                        continue
            else:
                for name, value in data.items():
                    try:
                        defaults[str(name)] = float(value)
                    except Exception:
                        continue

        with state.lock:
            state.parameter_defaults = defaults

            for name, parameter in state.parameters.items():
                if name in defaults:
                    parameter.default_value = defaults[name]

        log_info(f"Loaded {len(defaults)} parameter defaults from {path}")

    except Exception as exc:
        log_error(f"Could not load parameter defaults: {exc}")


def handle_param_value(msg):
    now = time.monotonic()

    name = parameter_name_from_message(msg)

    if not name:
        return

    param_type = safe_int(getattr(msg, "param_type", 0))

    raw_value = getattr(msg, "param_value", 0.0)
    value = decode_param_float_as_number(raw_value, param_type)

    index = safe_int(getattr(msg, "param_index", -1), -1)
    count = safe_int(getattr(msg, "param_count", 0), 0)

    with state.lock:
        parameter = state.parameters.get(name)

        if parameter is None:
            default_value = state.parameter_defaults.get(name)

            parameter = Parameter(
                name=name,
                value=value,
                param_type=param_type,
                index=index,
                count=count,
                startup_value=value,
                default_value=default_value,
                last_update=now,
            )

            state.parameters[name] = parameter

        else:
            parameter.value = value

            if param_type:
                parameter.param_type = param_type

            parameter.index = index

            if count:
                parameter.count = count

            parameter.last_update = now

            if parameter.default_value is None and name in state.parameter_defaults:
                parameter.default_value = state.parameter_defaults[name]

        if name not in state.parameter_order:
            state.parameter_order.append(name)
            state.parameter_order.sort(key=lambda item: item.upper())

        state.parameters_received = len(state.parameters)

        if count > 0:
            state.parameter_count = max(state.parameter_count, count)

        if state.parameter_set_pending and state.parameter_set_name == name:
            expected = state.parameter_set_value

            if expected is not None:
                tolerance = PARAM_VALUE_EPSILON

                if parameter_is_integer(parameter.param_type):
                    tolerance = 0.0

                if abs(value - expected) <= tolerance:
                    parameter.pending = False
                    parameter.pending_value = None

                    state.parameter_set_pending = False
                    state.parameter_set_name = ""
                    state.parameter_set_value = None
                    state.parameter_set_sent_at = 0.0

                    log_command(
                        f"PARAMETER CONFIRMED: {name} = {parameter_format_value(parameter)}"
                    )

        if count > 0 and state.parameters_received >= count:
            state.parameter_count = count
            state.parameters_complete = True
            state.parameters_requested_at = 0.0


def request_parameters(master):
    if not vehicle_ready(master):
        return False

    try:
        with state.lock:
            state.parameters.clear()
            state.parameter_order.clear()
            state.parameter_count = 0
            state.parameters_received = 0
            state.parameters_complete = False
            state.parameters_requested_at = time.monotonic()
            state.parameter_full_list_requested = True
            state.parameter_index = 0
            state.parameter_page = 0
            state.parameter_set_pending = False
            state.parameter_set_name = ""
            state.parameter_set_value = None

        master.mav.param_request_list_send(
            master.target_system,
            master.target_component,
        )

        log_command("PARAM_REQUEST_LIST SENT")
        return True

    except Exception as exc:
        with state.lock:
            state.parameters_requested_at = 0.0

        log_error(f"Failed to request parameters: {exc}")
        return False


def get_visible_parameters():
    """The parameters in the active ALL / CHANGED view (search is a highlight,
    not a filter, so it does not narrow this list)."""
    with state.lock:
        parameters = [
            state.parameters[name]
            for name in state.parameter_order
            if name in state.parameters
        ]

        view = state.parameter_view

    if view == "CHANGED":
        changed = []

        for parameter in parameters:
            default_changed = parameter.changed_from_default

            if default_changed is True:
                changed.append(parameter)
            elif default_changed is None and parameter.changed_from_startup:
                changed.append(parameter)

        parameters = changed

    return parameters


def send_parameter_set(master, parameter_name, value):
    if not vehicle_ready(master):
        return False

    parameter_name = str(parameter_name)

    with state.lock:
        parameter = state.parameters.get(parameter_name)

    if parameter is None:
        log_error(f"Unknown PX4 parameter: {parameter_name}")
        return False

    try:
        param_type = parameter.param_type

        if param_type <= 0:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        encoded_name = parameter_name.encode("utf-8")[:16].ljust(16, b"\x00")

        # Re-encode the value the same way PX4 encodes it: integer types are
        # sent as the raw bit pattern of an int32/uint32, not a numeric
        # float conversion.
        wire_value = encode_number_as_param_float(value, param_type)

        with state.lock:
            parameter.pending = True
            parameter.pending_value = value
            parameter.pending_sent_at = time.monotonic()

            state.parameter_set_pending = True
            state.parameter_set_name = parameter_name
            state.parameter_set_value = value
            state.parameter_set_sent_at = time.monotonic()

        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            encoded_name,
            float(wire_value),
            param_type,
        )

        log_command(f"PARAM_SET SENT: {parameter_name} = {value}")
        return True

    except Exception as exc:
        with state.lock:
            parameter.pending = False
            parameter.pending_value = None

            state.parameter_set_pending = False
            state.parameter_set_name = ""
            state.parameter_set_value = None
            state.parameter_set_sent_at = 0.0

        log_error(f"Failed to set parameter {parameter_name}: {exc}")
        return False


def confirm_parameter_set(parameter_name, value):
    send_parameter_set(session.link, parameter_name, value)


def check_parameter_request():
    """Time out an incomplete PARAM_REQUEST_LIST response."""
    now = time.monotonic()

    with state.lock:
        requested_at = state.parameters_requested_at
        complete = state.parameters_complete
        received = state.parameters_received
        count = state.parameter_count

    if not requested_at or complete:
        return

    if count > 0 and received >= count:
        with state.lock:
            state.parameters_complete = True
            state.parameters_requested_at = 0.0
        return

    if now - requested_at > PARAM_REQUEST_TIMEOUT:
        with state.lock:
            state.parameters_requested_at = 0.0

        log_warn(
            f"Parameter request timed out. Received {received}"
            + (f"/{count}" if count else "")
        )


def check_pending_parameter():
    """Time out a PARAM_SET PX4 never echoed back."""
    now = time.monotonic()

    with state.lock:
        if not state.parameter_set_pending:
            return

        sent_at = state.parameter_set_sent_at
        name = state.parameter_set_name
        parameter = state.parameters.get(name)

    if sent_at <= 0 or now - sent_at <= PARAM_SET_TIMEOUT:
        return

    with state.lock:
        state.parameter_set_pending = False
        state.parameter_set_name = ""
        state.parameter_set_value = None
        state.parameter_set_sent_at = 0.0

        if parameter is not None:
            parameter.pending = False
            parameter.pending_value = None

    log_error(f"PARAM_SET not confirmed by PX4: {name}")
