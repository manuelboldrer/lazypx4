"""KML waypoint auto-navigation queue ([W] on the map screen).

Flies a loaded KML's waypoints - one specific one, all of them in document
order, or a random sequence of a given length (sampled with replacement, so
it can be longer than the number of waypoints loaded and can repeat one) -
or a lawnmower sweep of its polygon ([C], see :mod:`lazypx4.coverage`),
auto-advancing to the next target once the vehicle arrives at the current
one. See :func:`lazypx4.navigation.open_wp_queue_input` for the input dialog
and :mod:`lazypx4.render.mapview` for the on-screen progress readout.

Optionally ("facing target" mode) each leg is flown as two phases instead of
one: rotate in place until the nose is on the bearing to the next target,
*then* fly to it (nose locked the way it's now facing) - a stop-turn-go
pattern, rather than yawing while under way. Without it every leg is just
the travel phase, current heading kept throughout.

Every leg is sent with :func:`lazypx4.mavlink.guided.send_goto_global` - the
same ``MAV_CMD_DO_REPOSITION`` every other goto variant and the keyboard jog
use - so it inherits their armed / vehicle-ready checks and PX4's own guided-
mode handling. This only ever repositions the vehicle; nothing here uploads
a mission, and the KML itself remains a read-only visualization overlay (see
lazypx4.kml).
"""

from __future__ import annotations

import math
import random
import time

from ..config import (
    WP_QUEUE_ARRIVAL_M,
    WP_QUEUE_LEG_TIMEOUT_S,
    WP_QUEUE_POLL_S,
    WP_QUEUE_ROTATE_TIMEOUT_S,
    WP_QUEUE_YAW_TOLERANCE_DEG,
)
from ..eventlog import log_error, log_info
from ..state import session, shutdown_event, state
from .guided import send_goto_global

#: WGS-84 equatorial radius, for a two-point great-circle distance.
_EARTH_RADIUS_M = 6378137.0


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two lat/lon points, metres."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2) ** 2
    )
    return 2 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def _bearing_deg(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing from point 1 to point 2, as an absolute
    compass heading in degrees (0-360, 0 = north, clockwise)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360.0


def build_targets(kml_waypoints, mode, index=None, count=None):
    """Turn ``state.kml_waypoints`` (``[(lat, lon, alt, name), ...]``,
    document order) into an ordered ``[(lat, lon, name), ...]`` target list
    for the given mode ("single" / "sequence" / "random"). Raises
    :class:`ValueError` (with a user-facing message) on a bad index/count.

    "random" samples *with* replacement: ``count`` is how many legs to fly,
    not how many distinct waypoints to visit, so e.g. 3 loaded waypoints and
    ``count=8`` picks one of the 3 at random 8 times over (repeats and, for
    a single waypoint, back-to-back repeats are both expected).
    """
    if mode == "single":
        if index is None or not (1 <= index <= len(kml_waypoints)):
            raise ValueError(f"waypoint number must be 1-{len(kml_waypoints)}")
        lat, lon, _alt, name = kml_waypoints[index - 1]
        return [(lat, lon, name or f"waypoint {index}")]

    if mode == "sequence":
        return [
            (lat, lon, name or f"waypoint {i}")
            for i, (lat, lon, _alt, name) in enumerate(kml_waypoints, start=1)
        ]

    if mode == "random":
        if count is None or count < 1:
            raise ValueError("waypoint count must be at least 1")
        if not kml_waypoints:
            raise ValueError("no waypoints loaded")
        pool = list(enumerate(kml_waypoints, start=1))
        chosen = random.choices(pool, k=count)
        return [
            (lat, lon, name or f"waypoint {i}")
            for i, (lat, lon, _alt, name) in chosen
        ]

    raise ValueError(f"unknown waypoint queue mode: {mode!r}")


def _yaw_error_deg(desired, actual):
    """Signed shortest angular distance ``desired - actual``, wrapped to
    (-180, 180], so it works the same way across the 0/360 seam."""
    return ((desired - actual + 180.0) % 360.0) - 180.0


def _begin_rotate(master, target, alt_amsl, cur_lat, cur_lon):
    """Send a reposition-in-place command that only turns the nose to face
    ``target`` (position target == current position). Returns the bearing
    sent, or None if the send failed."""
    lat, lon, _name = target
    bearing = _bearing_deg(cur_lat, cur_lon, lat, lon)
    if send_goto_global(master, cur_lat, cur_lon, alt_amsl, bearing):
        return bearing
    return None


def _begin_travel(master, target, alt_amsl, face_target, cur_lat, cur_lon):
    """Send the translate-to-target command, locking the yaw to the bearing
    just turned to when facing the target, keeping the current heading
    otherwise. Returns True on success."""
    lat, lon, _name = target
    yaw_deg = _bearing_deg(cur_lat, cur_lon, lat, lon) if face_target else None
    return send_goto_global(master, lat, lon, alt_amsl, yaw_deg)


def start_wp_queue(master, targets, mode, face_target=False):
    """Activate the queue: lock in the current altitude and send the first
    leg. Caller already has the user's typed confirmation. Returns True if
    the queue actually started.

    ``face_target`` makes the vehicle stop, rotate in place to face each
    target, then fly to it, rather than yawing while it flies (the default,
    same as every other goto variant).
    """
    with state.lock:
        if not state.armed:
            log_error("Waypoint queue refused: vehicle is not armed")
            return False
        if not state.global_pos_valid:
            log_error("Waypoint queue refused: no global position (need a GPS fix)")
            return False
        alt_amsl = state.global_alt
        cur_lat = state.global_lat
        cur_lon = state.global_lon

    first = targets[0]
    if face_target:
        sent = _begin_rotate(master, first, alt_amsl, cur_lat, cur_lon)
        if sent is None:
            return False
        phase = "rotate"
        status = f"rotating to face '{first[2]}'"
    else:
        if not _begin_travel(master, first, alt_amsl, False, cur_lat, cur_lon):
            return False
        phase = "travel"
        status = "en route"

    with state.lock:
        state.wp_queue = list(targets)
        state.wp_path = list(targets)
        state.wp_queue_mode = mode
        state.wp_queue_total = len(targets)
        state.wp_queue_alt_amsl = alt_amsl
        state.wp_queue_face_target = face_target
        state.wp_queue_phase = phase
        state.wp_queue_active = True
        state.wp_queue_status = status
        state.wp_queue_phase_started_at = time.monotonic()

    log_info(
        f"WAYPOINT QUEUE started ({mode}{', facing target' if face_target else ''}): "
        f"{len(targets)} target(s), first '{first[2]}' @ {alt_amsl:.1f} m MSL, "
        f"arrival radius {WP_QUEUE_ARRIVAL_M:.1f} m"
    )
    return True


def cancel_wp_queue(reason):
    """Deactivate the queue (idempotent - a no-op if it isn't active)."""
    with state.lock:
        if not state.wp_queue_active:
            return
        state.wp_queue_active = False
        state.wp_queue_status = reason
        state.wp_queue = []
    log_info(f"WAYPOINT QUEUE cancelled: {reason}")


def _advance(master, face_target):
    """The current (travel-phase) target has been reached or abandoned: pop
    it, then either start rotating to face the next one (facing mode) or
    depart for it directly - or complete the queue if that was the last
    one. Caller holds no lock."""
    with state.lock:
        if state.wp_queue:
            state.wp_queue.pop(0)
        remaining = list(state.wp_queue)
        alt_amsl = state.wp_queue_alt_amsl
        cur_lat = state.global_lat
        cur_lon = state.global_lon

    if not remaining:
        with state.lock:
            state.wp_queue_active = False
            state.wp_queue_status = "complete"
        log_info("WAYPOINT QUEUE complete")
        return

    nxt = remaining[0]
    if face_target:
        sent = _begin_rotate(master, nxt, alt_amsl, cur_lat, cur_lon)
        if sent is None:
            cancel_wp_queue("goto send failed")
            return
        with state.lock:
            state.wp_queue_phase = "rotate"
            state.wp_queue_status = f"rotating to face '{nxt[2]}'"
            state.wp_queue_phase_started_at = time.monotonic()
    else:
        if not _begin_travel(master, nxt, alt_amsl, False, cur_lat, cur_lon):
            cancel_wp_queue("goto send failed")
            return
        with state.lock:
            state.wp_queue_phase = "travel"
            state.wp_queue_status = "en route"
            state.wp_queue_phase_started_at = time.monotonic()


def _finish_rotate(master, target, alt_amsl, cur_lat, cur_lon):
    """The rotate phase for ``target`` is done (on heading or timed out):
    depart for it, nose locked the way it's now facing."""
    if not _begin_travel(master, target, alt_amsl, True, cur_lat, cur_lon):
        cancel_wp_queue("goto send failed")
        return
    with state.lock:
        state.wp_queue_phase = "travel"
        state.wp_queue_status = "en route"
        state.wp_queue_phase_started_at = time.monotonic()


def wp_queue_thread():
    """Daemon loop (see lazypx4.app.run): idle whenever no queue is active.
    In "facing target" mode, each leg is two phases - rotate in place until
    on heading (or timed out), then travel until arrived (or timed out),
    then the next leg's rotate phase begins. Without it, every leg is just
    the travel phase, current heading kept throughout."""
    while not shutdown_event.is_set():
        time.sleep(WP_QUEUE_POLL_S)
        try:
            with state.lock:
                if not state.wp_queue_active:
                    continue
                armed = state.armed
                phase = state.wp_queue_phase
                face_target = state.wp_queue_face_target
                target = state.wp_queue[0] if state.wp_queue else None
                have_fix = state.global_pos_valid
                cur_lat = state.global_lat
                cur_lon = state.global_lon
                cur_yaw = state.yaw
                phase_started = state.wp_queue_phase_started_at
                alt_amsl = state.wp_queue_alt_amsl

            if not armed:
                cancel_wp_queue("vehicle disarmed")
                continue
            if target is None:
                cancel_wp_queue("empty queue")
                continue
            if not have_fix:
                continue

            elapsed = time.monotonic() - phase_started

            if phase == "rotate":
                desired = _bearing_deg(cur_lat, cur_lon, target[0], target[1])
                on_heading = abs(_yaw_error_deg(desired, cur_yaw)) <= WP_QUEUE_YAW_TOLERANCE_DEG
                if on_heading:
                    _finish_rotate(session.link, target, alt_amsl, cur_lat, cur_lon)
                elif elapsed > WP_QUEUE_ROTATE_TIMEOUT_S:
                    log_error(
                        f"WAYPOINT QUEUE timed out turning to face '{target[2]}' - departing anyway"
                    )
                    _finish_rotate(session.link, target, alt_amsl, cur_lat, cur_lon)
                continue

            # phase == "travel"
            dist = distance_m(cur_lat, cur_lon, target[0], target[1])
            if dist <= WP_QUEUE_ARRIVAL_M:
                log_info(f"WAYPOINT QUEUE arrived at '{target[2]}' ({dist:.1f} m)")
                _advance(session.link, face_target)
            elif elapsed > WP_QUEUE_LEG_TIMEOUT_S:
                log_error(f"WAYPOINT QUEUE timed out reaching '{target[2]}' - skipping")
                _advance(session.link, face_target)
        except Exception as exc:  # never let a bad tick kill the thread
            log_error(f"Waypoint queue error: {exc}")
