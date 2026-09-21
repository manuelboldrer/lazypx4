"""The [n] position map: an ASCII plan view (up = North) with an optional trail.

The vehicle is drawn twice when the data is there: the green heading arrow is
the EKF local solution (``LOCAL_POSITION_NED`` / ``ODOMETRY``), and a cyan
``⊕`` is the raw GNSS fix (``GLOBAL_POSITION_INT``) projected into the same
local frame through ``GPS_GLOBAL_ORIGIN``. Watching the two converge - and the
``EKF↔GNSS`` offset shrink - is the quickest read on an RTK bring-up.

``[o]`` loads a local .kml file (see :mod:`lazypx4.kml`) as a visualization
overlay: its ``Polygon`` rings are drawn as a dotted yellow boundary and its
``Point``/``LineString`` waypoints as numbered magenta markers, both
projected into the same local frame. This is read-only - nothing loaded this
way is ever uploaded to the vehicle.
"""

from __future__ import annotations

import math
import os
import time

from ..ansi import BG_GREEN, BG_RED, BLACK, BLUE, BOLD, CYAN, DIM, GREEN, MAGENTA, RED, RESET, WHITE, YELLOW
from ..config import JOG_YAW_STEP_DEG, MAP_RANGE_MAX_M, settings
from ..mavlink.wp_queue import distance_m
from ..state import session, state
from ..util import clamp, finite, safe_float
from .pointcloud import _COLOR_BANDS as _LIDAR_BANDS, _color_for as _lidar_color
from .chrome import (
    BRAILLE_BITS,
    braille_glyph,
    content_area,
    global_to_local,
    gps_fix_display,
    heading_arrow,
)


def _baseline_text(metres):
    if metres <= 0:
        return DIM + "n/a" + RESET
    if metres >= 1000.0:
        return f"{metres / 1000.0:.2f} km"
    return f"{metres:.0f} m"


def draw_map_screen():
    with state.lock:
        valid = state.local_pos_valid
        lx = state.local_x
        ly = state.local_y
        lz = state.local_z
        last_local = state.last_local_pos
        yaw = state.yaw
        vx = state.vx
        vy = state.vy
        mode = state.mode

        trail = list(state.position_trail) if state.map_trail_enabled else []
        trail_on = state.map_trail_enabled
        map_range = state.map_range
        fit_kml = state.map_fit_kml

        lidar_on = state.map_lidar_enabled
        lidar_xs = list(state.lidar_sample_x) if lidar_on else []
        lidar_ys = list(state.lidar_sample_y) if lidar_on else []
        lidar_zs = list(state.lidar_sample_z) if lidar_on else []
        lidar_last = state.lidar_last_received
        lidar_supported = state.lidar_supported

        navpath_on = state.map_navpath_enabled
        navpath_pts = list(state.navpath_points) if navpath_on else []
        navpath_last = state.navpath_last_received
        mapfeeds_supported = state.mapfeeds_supported

        fire_last = state.fire_last_received
        fire_lat = state.fire_lat
        fire_lon = state.fire_lon
        fire_alt = state.fire_alt

        has_target = (
            state.last_pos_target > 0
            and (time.monotonic() - state.last_pos_target) < 5.0
        )
        tx = state.pos_target_x
        ty = state.pos_target_y

        origin_set = state.local_origin_set
        origin_lat = state.local_origin_lat
        origin_lon = state.local_origin_lon
        origin_alt = state.local_origin_alt

        kml_ground_alt = state.kml_ground_alt
        rng_dist = state.rangefinder_distance
        rng_min = state.rangefinder_min
        rng_max = state.rangefinder_max
        rng_quality = state.rangefinder_quality
        last_rng = state.last_rangefinder

        home_set = state.home_set
        home_lat = state.home_lat
        home_lon = state.home_lon

        kml_path = state.kml_path
        kml_loaded = state.kml_loaded
        kml_error = state.kml_error
        kml_waypoints = list(state.kml_waypoints)
        kml_fence_rings = [list(ring) for ring in state.kml_fence_rings]

        wp_queue = list(state.wp_queue)
        wp_path = list(state.wp_path)
        wp_queue_mode = state.wp_queue_mode
        wp_queue_total = state.wp_queue_total
        wp_queue_face_target = state.wp_queue_face_target
        wp_queue_active = state.wp_queue_active
        wp_queue_status = state.wp_queue_status

        fence_upload_active = state.fence_upload_active
        fence_upload_status = state.fence_upload_status
        fence_upload_error = state.fence_upload_error
        fence_upload_total = state.fence_upload_total
        fence_upload_acked_seq = state.fence_upload_acked_seq

        global_valid = state.global_pos_valid
        glat = state.global_lat
        glon = state.global_lon
        galt = state.global_alt

        gps_fix = state.gps_fix
        gps_sats = state.gps_sats
        gps_h_acc = state.gps_h_acc
        gps_v_acc = state.gps_v_acc
        last_gps = state.last_gps

        rtk_health = state.rtk_health
        rtk_rate = state.rtk_rate
        rtk_nsats = state.rtk_nsats
        rtk_baseline_m = state.rtk_baseline_m
        last_gps_rtk = state.last_gps_rtk
        dgps_age = state.gps_dgps_age

        dl_active = state.map_download_active
        dl_status = state.map_download_status
        dl_path = state.map_download_path
        dl_error = state.map_download_error
        dl_started = state.map_download_started_at

        last_ack = state.last_ack
        armed = state.armed

        gps_pub_feedback = state.gps_pub_feedback
        gps_pub_feedback_ok = state.gps_pub_feedback_ok

    now = time.monotonic()
    panel_width, panel_height = content_area()

    lx = finite(lx)
    ly = finite(ly)
    lz = finite(lz)
    yaw = finite(yaw)
    vx = finite(vx)
    vy = finite(vy)

    # GNSS fix projected into the local NED frame (needs both a fix and the
    # local origin). ``gps_local`` is (north, east) or None.
    gps_local = None
    gps_offset = None
    if global_valid and origin_set:
        gn, ge = global_to_local(glat, glon, origin_lat, origin_lon)
        if math.isfinite(gn) and math.isfinite(ge):
            gps_local = (gn, ge)
            if valid:
                gps_offset = (gn - lx, ge - ly)

    home_local = None
    if home_set and origin_set:
        hn, he = global_to_local(home_lat, home_lon, origin_lat, origin_lon)
        if math.isfinite(hn) and math.isfinite(he):
            home_local = (hn, he)

    # (lat, lon, alt, name, local_(n,e)_or_None) - one entry per KML waypoint
    # in document order, so its index always matches the marker number this
    # list and the grid glyph both use. Kept even when the local projection
    # fails (rather than dropped), so numbering never shifts between the two.
    kml_waypoints_detail = []
    if origin_set:
        for lat, lon, alt, name in kml_waypoints:
            wn, we = global_to_local(lat, lon, origin_lat, origin_lon)
            local_ne = (wn, we) if math.isfinite(wn) and math.isfinite(we) else None
            kml_waypoints_detail.append((lat, lon, alt, name, local_ne))

    # The waypoint queue's current target ((lat, lon, name), see
    # lazypx4.mavlink.wp_queue), projected the same way, so it can be
    # highlighted on the grid and its distance read out below.
    wp_queue_target_local = None
    wp_queue_dist = None
    if wp_queue_active and wp_queue and origin_set:
        qlat, qlon, _qname = wp_queue[0]
        qn, qe = global_to_local(qlat, qlon, origin_lat, origin_lon)
        if math.isfinite(qn) and math.isfinite(qe):
            wp_queue_target_local = (qn, qe)
            if valid:
                wp_queue_dist = math.hypot(qn - lx, qe - ly)

    kml_fence_rings_local = []
    if origin_set:
        for ring in kml_fence_rings:
            ring_local = []
            for lat, lon in ring:
                rn, re_ = global_to_local(lat, lon, origin_lat, origin_lon)
                if math.isfinite(rn) and math.isfinite(re_):
                    ring_local.append((rn, re_))
            if len(ring_local) >= 3:
                kml_fence_rings_local.append(ring_local)

    # The planned queue path: the one awaiting its YES (previewed before
    # anything is sent) or the running queue's full path.
    path_preview = (
        session.confirm_active
        and session.wp_preview_callback is not None
        and session.confirm_callback is session.wp_preview_callback
    )
    path_targets = list(session.wp_preview) if path_preview else (wp_path if wp_queue_active else [])
    path_local = []
    if origin_set:
        for plat, plon, _pname in path_targets:
            pn, pe = global_to_local(plat, plon, origin_lat, origin_lon)
            if math.isfinite(pn) and math.isfinite(pe):
                path_local.append((pn, pe))
    path_length_m = sum(
        distance_m(a[0], a[1], b[0], b[1]) for a, b in zip(path_targets, path_targets[1:])
    )

    # /fire_gps_loc fix projected into the local NED frame: (north, east) and
    # the fix's own local down (from its AMSL altitude and the origin's).
    # ``fire_rel`` is the fire relative to the vehicle, NED, and 3-D distance.
    fire_local = None
    fire_rel = None
    if fire_last and origin_set:
        fn, fe = global_to_local(fire_lat, fire_lon, origin_lat, origin_lon)
        if math.isfinite(fn) and math.isfinite(fe):
            fire_d = -(fire_alt - origin_alt) if math.isfinite(fire_alt) else 0.0
            fire_local = (fn, fe)
            if valid:
                rn, re_, rd = fn - lx, fe - ly, fire_d - lz
                fire_rel = (rn, re_, rd, math.sqrt(rn * rn + re_ * re_ + rd * rd))

    def satellite_lines():
        out = []
        if dl_active:
            elapsed = time.monotonic() - dl_started if dl_started else 0.0
            out.append(YELLOW + f" Satellite image: {dl_status} ({elapsed:.0f}s)..." + RESET)
        elif dl_status == "COMPLETE" and dl_path:
            out.append(GREEN + f" Satellite image saved: {dl_path}" + RESET)
        elif dl_status == "ERROR" and dl_error:
            out.append(RED + f" Satellite image failed: {dl_error}" + RESET)
        return out

    def gnss_lines():
        """The GNSS / global / RTK read-out block, shared by both layouts."""
        out = []

        fix_color, fix_name = gps_fix_display(gps_fix)
        gps_age = now - last_gps if last_gps else None
        age_note = f"   {DIM}{gps_age:.1f}s ago{RESET}" if gps_age is not None else ""
        out.append(
            f" GNSS      {fix_color}{fix_name}{RESET}   sats {gps_sats}"
            f"   EPH {gps_h_acc:.3f} m   EPV {gps_v_acc:.3f} m"
            + age_note
        )

        if global_valid:
            out.append(
                f" Global    {glat:.7f}, {glon:.7f}   alt {galt:.1f} m (MSL)"
            )

        if last_gps_rtk:
            if rtk_health == 1:
                health = GREEN + "OK" + RESET
            elif rtk_health == 0:
                health = YELLOW + "unhealthy" + RESET
            else:
                health = DIM + "?" + RESET
            rtk_msg_age = now - last_gps_rtk
            corr = f"   corr age {dgps_age:.1f}s" if dgps_age > 0 else ""
            out.append(
                f" RTK       health {health}   corrections {rtk_rate:.1f} Hz"
                f"   base {_baseline_text(rtk_baseline_m)}   nsat {rtk_nsats}"
                + corr
                + f"   {DIM}{rtk_msg_age:.1f}s ago{RESET}"
            )
        else:
            out.append(
                " RTK       " + DIM + "no GPS_RTK - receiver is not being fed "
                "RTCM corrections" + RESET
            )

        if gps_offset is not None:
            d_n, d_e = gps_offset
            dist = math.hypot(d_n, d_e)
            colour = GREEN if dist < 0.5 else (YELLOW if dist < 2.0 else RED)
            out.append(
                f" EKF↔GNSS  {colour}{dist:.2f} m{RESET} offset"
                f"   (N {d_n:+.2f}  E {d_e:+.2f})   {CYAN}⊕{RESET} = GNSS fix on the grid"
            )
        elif valid and gps_local is None and global_valid and not origin_set:
            out.append(
                " EKF↔GNSS  " + DIM + "no local origin yet - cannot place the "
                "GNSS fix on the grid" + RESET
            )

        return out

    def altitude_lines():
        """Cross-check of the EKF altitude against independent references.

        Three heights above the ground, each from a different source:
        the EKF's global altitude minus the KML site altitude, the EKF's
        local -D, and the downward rangefinder. On flat ground they should
        agree, so the offsets between them show which source is off.
        """
        out = []

        def verdict(delta):
            mag = abs(delta)
            colour = GREEN if mag < 0.5 else (YELLOW if mag < 2.0 else RED)
            return f"{colour}{delta:+.2f} m{RESET}"

        h_site = None
        if global_valid:
            if kml_ground_alt is not None:
                h_site = galt - kml_ground_alt
                out.append(
                    f" ALTITUDE  EKF {galt:.1f} m AMSL   KML site {kml_ground_alt:.1f} m"
                    f"   {BOLD}{h_site:+.2f} m{RESET} above site"
                )
            else:
                out.append(
                    f" ALTITUDE  EKF {galt:.1f} m AMSL   "
                    + DIM
                    + ("no altitude in the KML - no site reference" if kml_loaded
                       else "[o] load a KML with altitude for a site reference")
                    + RESET
                )
        else:
            out.append(" ALTITUDE  " + DIM + "no global position yet" + RESET)

        if valid and global_valid and origin_set:
            h_local = -lz
            h_origin = galt - origin_alt
            out.append(
                f"   local/global  -D {h_local:+.2f} m   AMSL-origin {h_origin:+.2f} m"
                f"   diff {verdict(h_local - h_origin)}"
            )

        rng_age = now - last_rng if last_rng else None
        if rng_age is None:
            out.append("   rangefinder   " + DIM + "no DISTANCE_SENSOR data" + RESET)
        elif rng_age > 2.0:
            out.append("   rangefinder   " + YELLOW + f"stale ({rng_age:.0f}s ago)" + RESET)
        elif not (rng_min <= rng_dist <= rng_max):
            out.append(
                f"   rangefinder   {YELLOW}{rng_dist:.2f} m out of range{RESET}"
                f"   {DIM}(valid {rng_min:.2f}-{rng_max:.2f} m){RESET}"
            )
        else:
            quality = f"   q {rng_quality}" if rng_quality >= 0 else ""
            if h_site is not None:
                ref, ref_name = h_site, "above site"
            elif valid:
                ref, ref_name = -lz, "local -D"
            else:
                ref, ref_name = None, ""
            cmp_text = (
                f"   vs {ref:+.2f} m {ref_name}   diff {verdict(rng_dist - ref)}"
                if ref is not None else ""
            )
            out.append(f"   rangefinder   {rng_dist:.2f} m{quality}{cmp_text}")

        return out

    if has_target:
        tx = finite(tx, math.nan)
        ty = finite(ty, math.nan)
        if not (math.isfinite(tx) and math.isfinite(ty)):
            has_target = False

    trail = [
        (tn, te) for (tn, te) in trail
        if math.isfinite(safe_float(tn)) and math.isfinite(safe_float(te))
    ]

    speed_h = math.hypot(vx, vy)

    lines = []

    if gps_pub_feedback:
        banner_bg = BG_GREEN if gps_pub_feedback_ok else BG_RED
        lines.append(
            banner_bg + BLACK + BOLD
            + f" [P] GPS {gps_pub_feedback} "
            + RESET
        )

    arm_text = GREEN + "ARMED" + RESET if armed else DIM + "DISARMED" + RESET
    lines.append(f" MODE: {BOLD}{mode}{RESET}   {arm_text}")

    if session.jog_armed:
        lines.append(
            BG_RED + WHITE + BOLD
            + f" JOG ARMED   step {session.jog_step:.2f} m / {JOG_YAW_STEP_DEG:.0f}°   "
            + "w/s up·down  k/j fwd·back  a/d strafe  h/l yaw  [/] step  x off "
            + RESET
        )
        if last_ack:
            lines.append(DIM + f" last command: {last_ack}" + RESET)
        if not armed:
            lines.append(YELLOW + " vehicle not armed - nudges will be refused" + RESET)
    else:
        # Same flight commands as the dashboard, same keys, same
        # confirmations - see navigation._handle_flight_command_key. Only
        # shown with jog disarmed: those letters are jog's own while it's on.
        lines.append(
            " FLIGHT: " + BOLD + "[a]" + RESET + "arm  " + BOLD + "[d]" + RESET + "disarm  "
            + BOLD + "[T]" + RESET + "takeoff  " + BOLD + "[L]" + RESET + "land  "
            + BOLD + "[R]" + RESET + "RTL  " + BOLD + "[h]" + RESET + "hold  "
            + RED + BOLD + "[K]" + RESET + "kill  " + BOLD + "[H]" + RESET + "home  "
            + BOLD + "[G]" + RESET + "fence"
        )

    if not valid:
        lines.append("")
        lines.append(RED + " Waiting for LOCAL_POSITION_NED / ODOMETRY ..." + RESET)
        lines.append("")
        lines.append(DIM + " PX4 only publishes a local position once the estimator has one" + RESET)
        lines.append(DIM + " (needs GPS lock, or flow/vision). Global / GNSS detail below." + RESET)

        lines.append("")
        lines.extend(gnss_lines())
        lines.extend(altitude_lines())

        sat = satellite_lines()
        if sat:
            lines.append("")
            lines.extend(sat)

        if kml_loaded:
            lines.append("")
            lines.append(f" KML       {os.path.basename(kml_path)}   " + DIM + "loaded" + RESET)
        elif kml_error:
            lines.append("")
            lines.append(f" KML       {RED}load failed: {kml_error}{RESET}")

        lines.append("")
        if global_valid:
            lines.append(
                "[i] download satellite image (pins home + robot)   [o] load kml"
                "   [O] upload fence   [n] back   ESC panels"
            )
        else:
            lines.append(
                DIM + "[i] satellite image needs a GPS fix" + RESET
                + "   [o] load kml   [O] upload fence   [n] back   ESC panels"
            )
        return lines

    # Grid sized to the panel's actual interior (see chrome.content_area), odd
    # dimensions so there is a true centre. -2 for the "  " row prefix below;
    # -17 for this screen's own non-grid lines (range/local/gnss/footer etc).
    grid_w = clamp((panel_width - 2) | 1, 21, 103)
    grid_h = clamp((panel_height - 17) | 1, 11, 35)

    half_w = grid_w // 2
    half_h = grid_h // 2

    # KML content within the same span the manual zoom itself allows (see the
    # comment below) - collected once, used either as one more thing the
    # vehicle-centred view must reach, or as the thing the [f] fit view
    # centres and scales to.
    kml_points = []
    for _lat, _lon, _alt, _name, local_ne in kml_waypoints_detail:
        if local_ne is not None and abs(local_ne[0]) <= MAP_RANGE_MAX_M and abs(local_ne[1]) <= MAP_RANGE_MAX_M:
            kml_points.append(local_ne)
    for ring_local in kml_fence_rings_local:
        for rn, re_ in ring_local:
            if abs(rn) <= MAP_RANGE_MAX_M and abs(re_) <= MAP_RANGE_MAX_M:
                kml_points.append((rn, re_))

    for pn, pe in path_local:
        if abs(pn) <= MAP_RANGE_MAX_M and abs(pe) <= MAP_RANGE_MAX_M:
            kml_points.append((pn, pe))

    for pn, pe in navpath_pts:
        if abs(pn) <= MAP_RANGE_MAX_M and abs(pe) <= MAP_RANGE_MAX_M:
            kml_points.append((pn, pe))

    if fire_local is not None:
        if abs(fire_local[0]) <= MAP_RANGE_MAX_M and abs(fire_local[1]) <= MAP_RANGE_MAX_M:
            kml_points.append(fire_local)

    if fit_kml and kml_points:
        # [f] fit to KML - centre the grid on the loaded geometry's own
        # bounding box (plus the live markers, so none of them fall
        # off-screen) instead of always centring on the local origin.
        # Vehicle-centred zoom has to reach from the origin all the way to
        # the far side of the polygon, so when the polygon sits away from
        # the origin it only ever fills the fraction of the grid past it -
        # this fits the view to the content itself instead.
        all_points = [(lx, ly)] + kml_points
        if has_target:
            all_points.append((tx, ty))
        if gps_local is not None:
            all_points.append(gps_local)
        ns = [p[0] for p in all_points]
        es = [p[1] for p in all_points]
        center_n = (min(ns) + max(ns)) / 2.0
        center_e = (min(es) + max(es)) / 2.0
        effective_range = max(
            max(abs(n - center_n) for n in ns),
            max(abs(e - center_e) for e in es),
        ) * 1.08
        effective_range = max(effective_range, 2.0)
    else:
        center_n = 0.0
        center_e = 0.0
        # Range must always contain the vehicle, the GNSS marker and the target.
        effective_range = map_range
        effective_range = max(effective_range, abs(lx) * 1.08, abs(ly) * 1.08)
        if has_target:
            effective_range = max(effective_range, abs(tx) * 1.08, abs(ty) * 1.08)
        if gps_local is not None:
            effective_range = max(
                effective_range, abs(gps_local[0]) * 1.08, abs(gps_local[1]) * 1.08
            )
        # A loaded KML can carry placemarks from more than one site (e.g. a
        # shared Google Earth project accumulating unrelated test locations).
        # One placemark tens or hundreds of km away would otherwise force
        # effective_range out to match it, collapsing the geometry that's
        # actually near the vehicle down to a handful of pixels.
        for n, e in kml_points:
            effective_range = max(effective_range, abs(n) * 1.08, abs(e) * 1.08)
        effective_range = max(effective_range, 2.0)

    grid = [[" "] * grid_w for _ in range(grid_h)]

    def to_cell(north, east):
        try:
            col = int(round(((east - center_e) / effective_range) * half_w)) + half_w
            row = half_h - int(round(((north - center_n) / effective_range) * half_h))
        except (ValueError, OverflowError):
            return None, None
        return row, col

    def put(north, east, char, force=True):
        if not (math.isfinite(north) and math.isfinite(east)):
            return
        row, col = to_cell(north, east)
        if row is None:
            return
        if 0 <= row < grid_h and 0 <= col < grid_w:
            if force or grid[row][col] == " ":
                grid[row][col] = char

    # Axes through the local origin (0, 0).
    for col in range(grid_w):
        put(0, (col - half_w) / half_w * effective_range, "─", force=False)
    for row in range(grid_h):
        put((half_h - row) / half_h * effective_range, 0, "│", force=False)
    put(0, 0, "┼")

    if trail_on and trail:
        # Packed into Braille dots (8 sub-cell positions per character, see
        # chrome.BRAILLE_BITS) rather than one "·" per grid cell: at a normal
        # zoom a cell can easily span a metre or more, which collapsed a
        # smooth flight path into a coarse, blocky dotted line.
        dot_w, dot_h = grid_w * 2, grid_h * 4
        dot_half_w, dot_half_h = dot_w / 2.0, dot_h / 2.0
        trail_cells = {}

        for tn, te in trail:
            if not (math.isfinite(tn) and math.isfinite(te)):
                continue
            try:
                dot_col = int(round(((te - center_e) / effective_range) * dot_half_w) + dot_half_w)
                dot_row = int(dot_half_h - round(((tn - center_n) / effective_range) * dot_half_h))
            except (ValueError, OverflowError):
                continue
            if not (0 <= dot_row < dot_h and 0 <= dot_col < dot_w):
                continue

            char_row, char_col = dot_row // 4, dot_col // 2
            bit = BRAILLE_BITS[(dot_col % 2, dot_row % 4)]
            key = (char_row, char_col)
            trail_cells[key] = trail_cells.get(key, 0) | bit

        for (row, col), bitmask in trail_cells.items():
            if grid[row][col] == " ":
                grid[row][col] = DIM + braille_glyph(bitmask) + RESET

    fence_char_cell = YELLOW + "." + RESET

    if kml_fence_rings_local:
        # A background reference layer: sampled onto still-blank cells only,
        # so it never competes with the trail or the live vehicle/home/target
        # markers drawn below.
        fence_char = YELLOW + "." + RESET

        def put_fence(north, east):
            if not (math.isfinite(north) and math.isfinite(east)):
                return
            row, col = to_cell(north, east)
            if row is not None and 0 <= row < grid_h and 0 <= col < grid_w:
                if grid[row][col] == " ":
                    grid[row][col] = fence_char

        cell_size = max(effective_range / max(half_w, half_h), 0.01)

        for ring_local in kml_fence_rings_local:
            closed_ring = ring_local + [ring_local[0]]
            for (an, ae), (bn, be) in zip(closed_ring, closed_ring[1:]):
                length = math.hypot(bn - an, be - ae)
                steps = min(400, max(1, int(length / cell_size)))
                for i in range(steps + 1):
                    t = i / steps
                    put_fence(an + (bn - an) * t, ae + (be - ae) * t)

    if len(path_local) >= 2:
        # Planned path as Braille dots (same sub-cell packing as the trail),
        # in cyan so it reads apart from the dim trail and yellow fence.
        # Only blank / fence cells are taken, so the vehicle, home and
        # waypoint markers drawn afterwards always win.
        dot_w, dot_h = grid_w * 2, grid_h * 4
        dot_half_w, dot_half_h = dot_w / 2.0, dot_h / 2.0
        path_cells = {}
        dot_size = max(effective_range / max(dot_half_w, dot_half_h), 0.001)

        for (an, ae), (bn, be) in zip(path_local, path_local[1:]):
            steps = min(2000, max(1, int(math.hypot(bn - an, be - ae) / dot_size)))
            for i in range(steps + 1):
                t = i / steps
                pn = an + (bn - an) * t
                pe = ae + (be - ae) * t
                try:
                    dot_col = int(round(((pe - center_e) / effective_range) * dot_half_w) + dot_half_w)
                    dot_row = int(dot_half_h - round(((pn - center_n) / effective_range) * dot_half_h))
                except (ValueError, OverflowError):
                    continue
                if not (0 <= dot_row < dot_h and 0 <= dot_col < dot_w):
                    continue
                key = (dot_row // 4, dot_col // 2)
                path_cells[key] = path_cells.get(key, 0) | BRAILLE_BITS[(dot_col % 2, dot_row % 4)]

        for (row, col), bitmask in path_cells.items():
            if grid[row][col] == " " or grid[row][col] == fence_char_cell:
                grid[row][col] = CYAN + braille_glyph(bitmask) + RESET

        put(path_local[0][0], path_local[0][1], GREEN + BOLD + "S" + RESET)
        put(path_local[-1][0], path_local[-1][1], RED + BOLD + "E" + RESET)

    if len(navpath_pts) >= 2:
        # The ROS planned path (/navsat_utm_path): Braille dots in blue, same
        # sub-cell packing and blank/fence-only rule as the queue path above
        # (which therefore wins where they overlap), S/E at its two ends.
        dot_w, dot_h = grid_w * 2, grid_h * 4
        dot_half_w, dot_half_h = dot_w / 2.0, dot_h / 2.0
        navpath_cells = {}
        dot_size = max(effective_range / max(dot_half_w, dot_half_h), 0.001)

        for (an, ae), (bn, be) in zip(navpath_pts, navpath_pts[1:]):
            steps = min(2000, max(1, int(math.hypot(bn - an, be - ae) / dot_size)))
            for i in range(steps + 1):
                t = i / steps
                try:
                    dot_col = int(round((((ae + (be - ae) * t) - center_e) / effective_range) * dot_half_w) + dot_half_w)
                    dot_row = int(dot_half_h - round((((an + (bn - an) * t) - center_n) / effective_range) * dot_half_h))
                except (ValueError, OverflowError):
                    continue
                if not (0 <= dot_row < dot_h and 0 <= dot_col < dot_w):
                    continue
                key = (dot_row // 4, dot_col // 2)
                navpath_cells[key] = navpath_cells.get(key, 0) | BRAILLE_BITS[(dot_col % 2, dot_row % 4)]

        for (row, col), bitmask in navpath_cells.items():
            if grid[row][col] == " " or grid[row][col] == fence_char_cell:
                grid[row][col] = BLUE + braille_glyph(bitmask) + RESET

        put(navpath_pts[0][0], navpath_pts[0][1], GREEN + BOLD + "s" + RESET)
        put(navpath_pts[-1][0], navpath_pts[-1][1], RED + BOLD + "e" + RESET)

    lidar_fresh = bool(lidar_last) and (now - lidar_last) < 2.0
    lidar_drawn = 0
    z_min, z_max = (min(lidar_zs), max(lidar_zs)) if lidar_zs else (0.0, 0.0)
    if lidar_on and lidar_fresh and lidar_xs:
        # Latest scan, sensor frame (x forward, y left) -> local NED: rotate
        # by the EKF yaw, translate to the vehicle. Assumes the sensor sits
        # at the vehicle centre, aligned with the body (lazypx4 doesn't know
        # the real mount) and ignores roll/pitch. Lowest-priority layer:
        # blank cells only, so the fence, path, trail and markers win.
        cos_y = math.cos(math.radians(yaw))
        sin_y = math.sin(math.radians(yaw))
        dot_w, dot_h = grid_w * 2, grid_h * 4
        dot_half_w, dot_half_h = dot_w / 2.0, dot_h / 2.0
        # Coloured by height (sensor-frame Z, low->high = blue->red, same
        # bands as the [v] screen) so the floor separates from walls and
        # obstacles.
        # cell -> [dot bitmask, tallest point's Z seen in it]
        lidar_cells = {}

        for px, py, pz in zip(lidar_xs, lidar_ys, lidar_zs):
            pn = lx + px * cos_y + py * sin_y
            pe = ly + px * sin_y - py * cos_y
            try:
                dot_col = int(round(((pe - center_e) / effective_range) * dot_half_w) + dot_half_w)
                dot_row = int(dot_half_h - round(((pn - center_n) / effective_range) * dot_half_h))
            except (ValueError, OverflowError):
                continue
            if not (0 <= dot_row < dot_h and 0 <= dot_col < dot_w):
                continue
            key = (dot_row // 4, dot_col // 2)
            bit = BRAILLE_BITS[(dot_col % 2, dot_row % 4)]
            entry = lidar_cells.get(key)
            if entry is None:
                lidar_cells[key] = [bit, pz]
            else:
                entry[0] |= bit
                entry[1] = max(entry[1], pz)

        for (row, col), (bitmask, pz) in lidar_cells.items():
            if grid[row][col] == " ":
                grid[row][col] = _lidar_color(pz, z_min, z_max) + braille_glyph(bitmask) + RESET
                lidar_drawn += 1

    for i, (_lat, _lon, _alt, _name, local_ne) in enumerate(kml_waypoints_detail, start=1):
        if local_ne is not None:
            put(local_ne[0], local_ne[1], MAGENTA + BOLD + str(i % 10) + RESET)

    # The waypoint queue's current target, drawn over its plain KML marker
    # so it stands out among the rest while a queue is running.
    if wp_queue_target_local is not None:
        put(wp_queue_target_local[0], wp_queue_target_local[1], WHITE + BOLD + "◎" + RESET)

    if home_local is not None:
        put(home_local[0], home_local[1], YELLOW + "H" + RESET)
    elif origin_set:
        # No HOME_POSITION yet - fall back to the local origin, which usually
        # coincides with home at EKF init (but can diverge afterwards).
        put(0, 0, DIM + "H" + RESET)

    if has_target:
        put(tx, ty, "*")

    if fire_local is not None:
        put(fire_local[0], fire_local[1], RED + BOLD + "F" + RESET)

    # GNSS fix first, so the green EKF arrow wins the cell when they coincide.
    if gps_local is not None:
        put(gps_local[0], gps_local[1], CYAN + "⊕" + RESET)

    put(lx, ly, BOLD + GREEN + heading_arrow(yaw) + RESET)

    if fit_kml and kml_points:
        fit_note = f"   {GREEN}[f] fit:kml{RESET}  centre N {center_n:+.1f} E {center_e:+.1f}"
    elif fit_kml:
        fit_note = f"   {DIM}[f] fit:kml (no geometry loaded){RESET}"
    else:
        fit_note = ""
    lines.append(
        f" range +/-{effective_range:6.1f} m      "
        f"{DIM}cell ~ {effective_range / half_w:4.1f} m x {effective_range / half_h:4.1f} m{RESET}"
        + fit_note
    )
    lines.append("")

    for row in grid:
        lines.append("  " + "".join(row))

    lines.append("")
    lines.append(
        f" Local {GREEN}{heading_arrow(yaw)}{RESET}  N {lx:+8.2f}   E {ly:+8.2f}   D {lz:+8.2f} m"
        f"   Yaw {yaw:6.1f}°   Speed {speed_h:.2f} m/s"
    )

    if has_target:
        lines.append(
            f" Target *  N {tx:+8.2f}   E {ty:+8.2f} m"
            f"   dist {math.hypot(tx - lx, ty - ly):.2f} m"
        )

    if origin_set:
        lines.append(f" Origin H  {origin_lat:.7f}, {origin_lon:.7f}")
    if home_set:
        home_note = "" if home_local is not None else "  " + DIM + "(no local origin yet)" + RESET
        lines.append(f" Home      {YELLOW}H{RESET}  {home_lat:.7f}, {home_lon:.7f}" + home_note)

    lines.extend(gnss_lines())
    lines.extend(altitude_lines())

    if kml_loaded:
        if origin_set:
            lines.append(
                f" KML       {os.path.basename(kml_path)}   "
                f"waypoints ({MAGENTA}{BOLD}#{RESET}): {len(kml_waypoints)}   "
                f"fence ({YELLOW}.{RESET}): {len(kml_fence_rings)} ring(s)"
            )
        else:
            lines.append(
                f" KML       {os.path.basename(kml_path)}   " + DIM
                + "loaded, but no local origin yet - cannot place it on the grid" + RESET
            )

        for i, (wlat, wlon, walt, wname, local_ne) in enumerate(kml_waypoints_detail, start=1):
            label = wname or f"waypoint {i}"
            if local_ne is not None:
                local_text = f"N {local_ne[0]:+8.2f}  E {local_ne[1]:+8.2f} m"
            else:
                local_text = DIM + "out of range" + RESET
            lines.append(
                f"   {MAGENTA}{BOLD}{i % 10}{RESET} {label:<16.16} local {local_text}"
                f"   global {wlat:.7f}, {wlon:.7f}   alt {walt:.1f} m"
            )
    elif kml_error:
        lines.append(f" KML       {RED}load failed: {kml_error}{RESET}")

    if path_local:
        label = "PREVIEW  " if path_preview else "PATH     "
        lines.append(
            f" {label} {CYAN}⣿{RESET} {len(path_targets)} waypoint(s), {path_length_m:.0f} m"
            f"   {GREEN}{BOLD}S{RESET} start  {RED}{BOLD}E{RESET} end"
            + (f"   {YELLOW}awaiting YES{RESET}" if path_preview else "")
        )
    elif path_preview and path_targets:
        lines.append(f" PREVIEW   {DIM}no local origin yet - cannot draw the path{RESET}")

    if wp_queue_active and wp_queue:
        done = wp_queue_total - len(wp_queue) + 1
        _qlat, _qlon, qname = wp_queue[0]
        dist_text = f"   dist {wp_queue_dist:.1f} m" if wp_queue_dist is not None else ""
        face_text = "  facing target" if wp_queue_face_target else ""
        lines.append(
            f" WP QUEUE  {WHITE}{BOLD}◎{RESET} [{wp_queue_mode}]  {done}/{wp_queue_total}"
            f" -> {qname}{dist_text}{face_text}   {GREEN}{wp_queue_status}{RESET}   [W]/[C] cancel"
        )
    elif wp_queue_status and wp_queue_mode:
        lines.append(f" WP QUEUE  {DIM}{wp_queue_status}{RESET}")

    if lidar_on:
        if not lidar_supported:
            lidar_note = DIM + "no ROS 2 / numpy - LiDAR unavailable" + RESET
        elif not lidar_fresh:
            lidar_note = YELLOW + "no recent scan" + RESET + f" on {settings.lidar_topic}"
        else:
            swatches = "".join(f"{color}█{RESET}" for color in _LIDAR_BANDS)
            lidar_note = (
                f"{len(lidar_xs)} pts   height {z_min:.1f} m {swatches} {z_max:.1f} m   "
                f"{DIM}sensor assumed at vehicle centre, rotated by yaw{RESET}"
            )
        lines.append(f" LIDAR     {lidar_note}")

    if fire_last:
        if fire_local is None:
            lines.append(
                f" FIRE      {RED}{BOLD}F{RESET}  {fire_lat:.7f}, {fire_lon:.7f}   "
                + DIM + "no local origin yet - cannot place it on the grid" + RESET
            )
        else:
            lines.append(
                f" FIRE      {RED}{BOLD}F{RESET}  {fire_lat:.7f}, {fire_lon:.7f}   alt {fire_alt:.1f} m"
                f"   from origin H: N {fire_local[0]:+8.2f}  E {fire_local[1]:+8.2f} m"
            )
            if fire_rel is not None:
                lines.append(
                    f"   Fire rel. UAV (NED)  N {fire_rel[0]:+8.2f}   E {fire_rel[1]:+8.2f}"
                    f"   D {fire_rel[2]:+8.2f} m   dist {fire_rel[3]:.1f} m"
                )

    if navpath_on:
        if not mapfeeds_supported:
            navpath_note = DIM + "no ROS 2 - path unavailable" + RESET
        elif not navpath_last:
            navpath_note = YELLOW + "no path received yet" + RESET + f" on {settings.navpath_topic}"
        else:
            age = now - navpath_last
            navpath_note = (
                f"{BLUE}⣿{RESET} {len(navpath_pts)} pose(s)   {GREEN}{BOLD}s{RESET} start  "
                f"{RED}{BOLD}e{RESET} end   {DIM}received {age:.0f}s ago (ENU map frame){RESET}"
            )
        lines.append(f" NAVPATH   {navpath_note}")

    if fence_upload_active:
        sent = fence_upload_acked_seq + 1
        lines.append(
            f" FENCE UP  {YELLOW}uploading{RESET}   vertex {sent}/{fence_upload_total}"
        )
    elif fence_upload_status == "COMPLETE":
        lines.append(f" FENCE UP  {GREEN}ACCEPTED by PX4{RESET}   {fence_upload_total} vertice(s)")
    elif fence_upload_status == "ERROR":
        lines.append(f" FENCE UP  {RED}failed: {fence_upload_error}{RESET}")

    data_age = now - last_local if last_local else 0.0
    trail_text = f"on ({len(trail)})" if trail_on else "off"
    lines.append("")
    lines.append(f" data age {data_age:.1f}s    trail {trail_text}")

    sat = satellite_lines()
    if sat:
        lines.extend(sat)

    lines.append(
        "[g] goto   [W] wp queue   [C] coverage   [P] publish gps   [x] jog   [o] load kml"
        "   [O] upload fence   [+]/[-] zoom   [0] reset   [f] fit kml   [t] trail"
        "   [c] clear   [V] lidar   [N] navpath   [i] sat   [n] back   ESC panels"
    )

    return lines
