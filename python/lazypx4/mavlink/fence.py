"""Geofence upload: MAVLink mission protocol, ``mission_type = FENCE``.

Uploads the polygon(s) from the currently loaded ``[o]`` KML overlay (see
:mod:`lazypx4.kml`) to PX4 as a real, enforced fence -
``MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`` items, one per vertex, grouped
into polygons the way PX4 expects: every vertex of the same polygon carries
that polygon's total vertex count in ``param1``.

This is the standard MAVLink mission-item upload handshake - the same one
QGroundControl's Plan editor uses to send a fence: we announce the item
count with ``MISSION_COUNT``, the vehicle asks for each item in turn with
``MISSION_REQUEST``/``MISSION_REQUEST_INT``, and it finishes with a
``MISSION_ACK``. Loading a ``.kml`` with ``[o]`` never does this by itself -
it is a local visualization only until this upload actually runs.

Every ``mavutil.mavlink.MAV_*`` constant is looked up inside a function body,
never at module import time - the real dialect (``development``, see
``mavlink.connection.connect_vehicle``) is only selected once the connection
opens, which happens after this module is first imported.
"""

from __future__ import annotations

import time

from pymavlink import mavutil

from ..config import FENCE_UPLOAD_TIMEOUT
from ..eventlog import log_command, log_error, log_info, log_warn
from ..state import session, state
from ..util import safe_int
from .connection import vehicle_ready


def _build_fence_items(fence_rings):
    """``[[(lat, lon), ...], ...]`` -> ``[{"lat", "lon", "vertex_count"}, ...]``.

    Drops a ring's closing vertex when it duplicates the first point (KML
    polygons repeat it; PX4's polygon is implicitly closed and expects only
    the unique vertices).
    """
    items = []

    for ring in fence_rings:
        vertices = list(ring)

        if len(vertices) >= 2 and vertices[0] == vertices[-1]:
            vertices = vertices[:-1]

        if len(vertices) < 3:
            continue

        vertex_count = len(vertices)

        for lat, lon in vertices:
            items.append({"lat": lat, "lon": lon, "vertex_count": vertex_count})

    return items


def start_fence_upload(master):
    if not vehicle_ready(master):
        return False

    with state.lock:
        if state.fence_upload_active:
            log_warn("A geofence upload is already in progress")
            return False

        fence_rings = [list(ring) for ring in state.kml_fence_rings]

    items = _build_fence_items(fence_rings)

    if not items:
        log_error("No fence polygon loaded - load one with [o] first")
        return False

    with state.lock:
        state.fence_upload_items = items
        state.fence_upload_total = len(items)
        state.fence_upload_active = True
        state.fence_upload_status = "UPLOADING"
        state.fence_upload_error = ""
        state.fence_upload_acked_seq = -1
        state.fence_upload_started_at = time.monotonic()

    try:
        master.mav.mission_count_send(
            master.target_system, master.target_component,
            len(items), mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        )
        log_command(f"GEOFENCE upload started: {len(items)} vertice(s)")
        return True
    except Exception as exc:
        with state.lock:
            state.fence_upload_active = False
            state.fence_upload_status = "ERROR"
            state.fence_upload_error = str(exc)
        log_error(f"Failed to start geofence upload: {exc}")
        return False


def _send_item(master, seq):
    with state.lock:
        items = state.fence_upload_items

    if seq < 0 or seq >= len(items):
        return

    item = items[seq]

    try:
        master.mav.mission_item_int_send(
            master.target_system, master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            0, 0,
            float(item["vertex_count"]), 0.0, 0.0, 0.0,
            int(round(item["lat"] * 1e7)), int(round(item["lon"] * 1e7)), 0.0,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        )
    except Exception as exc:
        log_error(f"Failed to send fence item {seq}: {exc}")
        return

    with state.lock:
        state.fence_upload_acked_seq = seq


def handle_mission_request(msg):
    """Shared by ``MISSION_REQUEST`` and ``MISSION_REQUEST_INT`` - both carry
    the fields that matter here (``seq``, ``mission_type``)."""
    if safe_int(getattr(msg, "mission_type", -1), -1) != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
        return

    with state.lock:
        active = state.fence_upload_active

    if not active or session.link is None:
        return

    seq = safe_int(getattr(msg, "seq", -1), -1)
    _send_item(session.link, seq)


def handle_mission_ack(msg):
    if safe_int(getattr(msg, "mission_type", -1), -1) != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
        return

    with state.lock:
        active = state.fence_upload_active
        total = state.fence_upload_total

    if not active:
        return

    result = safe_int(getattr(msg, "type", -1), -1)

    result_names = {
        getattr(mavutil.mavlink, name, None): label
        for name, label in (
            ("MAV_MISSION_ACCEPTED", "ACCEPTED"),
            ("MAV_MISSION_ERROR", "ERROR"),
            ("MAV_MISSION_UNSUPPORTED_FRAME", "UNSUPPORTED_FRAME"),
            ("MAV_MISSION_UNSUPPORTED", "UNSUPPORTED"),
            ("MAV_MISSION_NO_SPACE", "NO_SPACE"),
            ("MAV_MISSION_INVALID", "INVALID"),
            ("MAV_MISSION_INVALID_PARAM1", "INVALID_PARAM1"),
            ("MAV_MISSION_INVALID_PARAM2", "INVALID_PARAM2"),
            ("MAV_MISSION_INVALID_SEQUENCE", "INVALID_SEQUENCE"),
            ("MAV_MISSION_DENIED", "DENIED"),
        )
    }
    result_name = result_names.get(result, f"result {result}")

    with state.lock:
        state.fence_upload_active = False

        if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            state.fence_upload_status = "COMPLETE"
            state.fence_upload_error = ""
        else:
            state.fence_upload_status = "ERROR"
            state.fence_upload_error = result_name

    if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        log_info(f"GEOFENCE uploaded and ACCEPTED by PX4 ({total} vertice(s))")
    else:
        log_error(f"GEOFENCE upload rejected by PX4: {result_name}")


def check_fence_upload():
    """Give up on an upload the vehicle never finished requesting/acking."""
    now = time.monotonic()

    with state.lock:
        active = state.fence_upload_active
        started_at = state.fence_upload_started_at

    if not active:
        return

    if now - started_at > FENCE_UPLOAD_TIMEOUT:
        with state.lock:
            state.fence_upload_active = False
            state.fence_upload_status = "ERROR"
            state.fence_upload_error = "timed out waiting for the vehicle"
        log_error("GEOFENCE upload timed out")
