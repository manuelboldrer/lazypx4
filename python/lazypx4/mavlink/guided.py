"""Guided flight commands: takeoff, land, return-to-launch, and "goto".

``goto`` sends ``MAV_CMD_DO_REPOSITION`` (COMMAND_INT, frame
``GLOBAL_INT``) to an absolute lat/lon/AMSL target, computed one of three
ways - see :func:`send_goto_body` (body-relative: forward/right/down from
the current position and heading - also what backs the keyboard jog
control), :func:`send_goto_local` (an absolute point in the local NED frame,
same frame as the [n] map's grid/trail) and :func:`send_goto_global` (a
literal lat/lon/AMSL typed in directly). All three end up sending the exact
same command via :func:`_send_reposition_command`; only how the target
lat/lon/AMSL is computed differs. All of these need the vehicle armed; PX4
handles the mode switch itself.
"""

from __future__ import annotations

import math

from pymavlink import mavutil

from ..eventlog import log_command, log_error
from ..state import state
from .connection import vehicle_ready

# metres per degree of latitude (WGS-84 mean); good to ~0.5% anywhere.
_METRES_PER_DEG = 111320.0

_REPOSITION_CHANGE_MODE = float(
    getattr(mavutil.mavlink, "MAV_DO_REPOSITION_FLAGS_CHANGE_MODE", 1)
)


def send_takeoff(master, altitude_m):
    """Auto-takeoff to ``altitude_m`` metres above the current altitude.

    The vehicle must already be armed (lazypx4 never auto-arms) and have a
    global position. PX4 switches to Takeoff mode and climbs.
    """
    if not vehicle_ready(master):
        return False

    with state.lock:
        if not state.armed:
            log_error("Takeoff refused: vehicle is not armed (press [a] first)")
            return False
        if not state.global_pos_valid:
            log_error("Takeoff refused: no global position (need a GPS fix)")
            return False
        lat = state.global_lat
        lon = state.global_lon
        # Current AMSL altitude (GLOBAL_POSITION_INT.alt). PX4's mavlink
        # receiver copies MAV_CMD_NAV_TAKEOFF's z straight into
        # vehicle_command.param7 with NO frame conversion - the frame given
        # here (MAV_FRAME_GLOBAL) is only documentation; PX4 always reads
        # param7 as AMSL for this specific command regardless of what frame
        # is set, so a relative-altitude value here reads as "N metres above
        # sea level", which is below the vehicle almost everywhere and makes
        # PX4 refuse with "Already higher than takeoff altitude".
        current_amsl = state.global_alt

    target_alt = current_amsl + max(0.0, float(altitude_m))

    try:
        master.mav.command_int_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,
            0.0,          # param1: pitch (fixed-wing only)
            0.0,          # param2: empty
            0.0,          # param3: empty
            math.nan,     # param4: yaw (keep current)
            int(round(lat * 1e7)),
            int(round(lon * 1e7)),
            float(target_alt),
        )
    except Exception as exc:
        log_error(f"Failed to send takeoff: {exc}")
        return False

    log_command(f"TAKEOFF (NAV_TAKEOFF) SENT: climb to {altitude_m:.1f} m")
    return True


def send_land(master):
    """Land at the current position (``MAV_CMD_NAV_LAND``)."""
    if not vehicle_ready(master):
        return False

    with state.lock:
        have_global = state.global_pos_valid
        lat = state.global_lat
        lon = state.global_lon

    try:
        if have_global:
            master.mav.command_int_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0,
                0.0, 0.0, 0.0,
                math.nan,
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                0.0,
            )
        else:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            )
    except Exception as exc:
        log_error(f"Failed to send land: {exc}")
        return False

    log_command("LAND (NAV_LAND) SENT")
    return True


def send_rtl(master):
    """Return to the launch point (``MAV_CMD_NAV_RETURN_TO_LAUNCH``)."""
    if not vehicle_ready(master):
        return False

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
    except Exception as exc:
        log_error(f"Failed to send RTL: {exc}")
        return False

    log_command("RTL (RETURN_TO_LAUNCH) SENT")
    return True


def _body_to_ned(forward, right, yaw_deg):
    """Rotate a body-frame (forward, right) offset into world (north, east)."""
    c = math.cos(math.radians(yaw_deg))
    s = math.sin(math.radians(yaw_deg))
    north = forward * c - right * s
    east = forward * s + right * c
    return north, east


def _send_reposition_command(master, target_lat, target_lon, target_amsl, yaw_deg):
    """The one MAV_CMD_DO_REPOSITION send shared by every goto variant and
    the keyboard jog. Caller does its own armed/position checks and logging.

    ``target_amsl`` is AMSL metres (unambiguous). ``yaw_deg`` is an absolute
    compass heading in degrees, or ``None`` to keep the current heading.
    """
    # MAV_CMD_DO_REPOSITION's yaw param is specified in radians (unlike most
    # other yaw params in the spec, which use degrees), so convert here.
    yaw_param = math.nan if yaw_deg is None else math.radians(yaw_deg)

    try:
        master.mav.command_int_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,   # x/y = 1e7 deg, z = AMSL m
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0,
            -1.0,                     # param1: ground speed, -1 = default
            _REPOSITION_CHANGE_MODE,  # param2: switch to the reposition setpoint
            0.0,                      # param3: unused for multicopters
            yaw_param,                # param4: yaw (NaN = keep current)
            int(round(target_lat * 1e7)),
            int(round(target_lon * 1e7)),
            float(target_amsl),
        )
        return True
    except Exception as exc:
        log_error(f"Failed to send goto: {exc}")
        return False


def send_goto_body(master, forward, right, down, yaw_deg=None, quiet=False):
    """Reposition ``forward`` / ``right`` / ``down`` metres from the current
    position, relative to the current heading.

    ``down`` is positive-down (a positive value descends). ``yaw_deg`` is an
    absolute compass heading; ``None`` keeps the current heading. ``quiet``
    suppresses the log line (used by the jog control, which logs its own).
    """
    if not vehicle_ready(master):
        return False

    with state.lock:
        if not state.armed:
            log_error("Goto refused: vehicle is not armed")
            return False
        if not state.global_pos_valid:
            log_error("Goto refused: no global position (need a GPS fix)")
            return False

        lat = state.global_lat
        lon = state.global_lon
        amsl = state.global_alt
        yaw_now = state.yaw

    north, east = _body_to_ned(forward, right, yaw_now)

    target_lat = lat + north / _METRES_PER_DEG
    target_lon = lon + east / (_METRES_PER_DEG * max(0.05, math.cos(math.radians(lat))))
    # down == 0 -> target == current altitude, so a purely horizontal
    # "10 0 0" does not change height.
    target_amsl = amsl - down

    if not _send_reposition_command(master, target_lat, target_lon, target_amsl, yaw_deg):
        return False

    if not quiet:
        log_command(
            f"GOTO (DO_REPOSITION) SENT [relative]: fwd {forward:+.1f} right {right:+.1f} "
            f"down {down:+.1f} m -> {target_lat:.7f}, {target_lon:.7f} @ {target_amsl:.1f} m MSL"
        )
    return True


def send_goto_local(master, north, east, down, yaw_deg=None):
    """Reposition to an absolute point in the local NED frame - the same
    frame the [n] map's grid and trail use (origin-relative, not relative to
    the vehicle's current position or heading). ``down`` is positive-down.

    Needs a known local origin (``GPS_GLOBAL_ORIGIN``) to convert north/east
    into the lat/lon ``DO_REPOSITION`` actually takes.
    """
    if not vehicle_ready(master):
        return False

    with state.lock:
        if not state.armed:
            log_error("Goto refused: vehicle is not armed")
            return False
        if not state.local_origin_set:
            log_error("Goto (local) refused: no local origin yet (GPS_GLOBAL_ORIGIN)")
            return False

        origin_lat = state.local_origin_lat
        origin_lon = state.local_origin_lon
        origin_alt = state.local_origin_alt

    target_lat = origin_lat + north / _METRES_PER_DEG
    target_lon = origin_lon + east / (
        _METRES_PER_DEG * max(0.05, math.cos(math.radians(origin_lat)))
    )
    target_amsl = origin_alt - down

    if not _send_reposition_command(master, target_lat, target_lon, target_amsl, yaw_deg):
        return False

    log_command(
        f"GOTO (DO_REPOSITION) SENT [local NED]: N {north:+.1f} E {east:+.1f} "
        f"D {down:+.1f} m -> {target_lat:.7f}, {target_lon:.7f} @ {target_amsl:.1f} m MSL"
    )
    return True


def send_goto_global(master, lat, lon, alt_amsl, yaw_deg=None):
    """Reposition to a literal global position (lat, lon, AMSL altitude)."""
    if not vehicle_ready(master):
        return False

    with state.lock:
        if not state.armed:
            log_error("Goto refused: vehicle is not armed")
            return False

    if not _send_reposition_command(master, lat, lon, alt_amsl, yaw_deg):
        return False

    log_command(
        f"GOTO (DO_REPOSITION) SENT [global]: {lat:.7f}, {lon:.7f} @ {alt_amsl:.1f} m MSL"
    )
    return True
