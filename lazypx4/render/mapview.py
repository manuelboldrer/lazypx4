"""The [n] position map: an ASCII plan view (up = North) with an optional trail.

The vehicle is drawn twice when the data is there: the green heading arrow is
the EKF local solution (``LOCAL_POSITION_NED`` / ``ODOMETRY``), and a cyan
``⊕`` is the raw GNSS fix (``GLOBAL_POSITION_INT``) projected into the same
local frame through ``GPS_GLOBAL_ORIGIN``. Watching the two converge - and the
``EKF↔GNSS`` offset shrink - is the quickest read on an RTK bring-up.
"""

from __future__ import annotations

import math
import time

from ..ansi import BG_RED, BOLD, CYAN, DIM, GREEN, RED, RESET, WHITE, YELLOW
from ..config import JOG_YAW_STEP_DEG
from ..state import session, state
from ..util import clamp, finite, safe_float
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

        trail = list(state.position_trail) if state.map_trail_enabled else []
        trail_on = state.map_trail_enabled
        map_range = state.map_range

        has_target = (
            state.last_pos_target > 0
            and (time.monotonic() - state.last_pos_target) < 5.0
        )
        tx = state.pos_target_x
        ty = state.pos_target_y

        origin_set = state.local_origin_set
        origin_lat = state.local_origin_lat
        origin_lon = state.local_origin_lon

        home_set = state.home_set
        home_lat = state.home_lat
        home_lon = state.home_lon

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

    if not valid:
        lines.append("")
        lines.append(RED + " Waiting for LOCAL_POSITION_NED / ODOMETRY ..." + RESET)
        lines.append("")
        lines.append(DIM + " PX4 only publishes a local position once the estimator has one" + RESET)
        lines.append(DIM + " (needs GPS lock, or flow/vision). Global / GNSS detail below." + RESET)

        lines.append("")
        lines.extend(gnss_lines())

        sat = satellite_lines()
        if sat:
            lines.append("")
            lines.extend(sat)

        lines.append("")
        if global_valid:
            lines.append("[i] download satellite image (pins home + robot)   [n] back   ESC panels")
        else:
            lines.append(DIM + "[i] satellite image needs a GPS fix" + RESET + "   [n] back   ESC panels")
        return lines

    # Grid sized to the panel's actual interior (see chrome.content_area), odd
    # dimensions so there is a true centre. -2 for the "  " row prefix below;
    # -17 for this screen's own non-grid lines (range/local/gnss/footer etc).
    grid_w = clamp((panel_width - 2) | 1, 21, 103)
    grid_h = clamp((panel_height - 17) | 1, 11, 35)

    half_w = grid_w // 2
    half_h = grid_h // 2

    # Range must always contain the vehicle, the GNSS marker and the target.
    effective_range = map_range
    effective_range = max(effective_range, abs(lx) * 1.08, abs(ly) * 1.08)
    if has_target:
        effective_range = max(effective_range, abs(tx) * 1.08, abs(ty) * 1.08)
    if gps_local is not None:
        effective_range = max(
            effective_range, abs(gps_local[0]) * 1.08, abs(gps_local[1]) * 1.08
        )
    effective_range = max(effective_range, 2.0)

    grid = [[" "] * grid_w for _ in range(grid_h)]

    def to_cell(north, east):
        try:
            col = int(round((east / effective_range) * half_w)) + half_w
            row = half_h - int(round((north / effective_range) * half_h))
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
                dot_col = int(round((te / effective_range) * dot_half_w) + dot_half_w)
                dot_row = int(dot_half_h - round((tn / effective_range) * dot_half_h))
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

    if origin_set:
        put(0, 0, "H")

    if has_target:
        put(tx, ty, "*")

    # GNSS fix first, so the green EKF arrow wins the cell when they coincide.
    if gps_local is not None:
        put(gps_local[0], gps_local[1], CYAN + "⊕" + RESET)

    put(lx, ly, BOLD + GREEN + heading_arrow(yaw) + RESET)

    lines.append(
        f" range +/-{effective_range:6.1f} m      "
        f"{DIM}cell ~ {effective_range / half_w:4.1f} m x {effective_range / half_h:4.1f} m{RESET}"
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
        lines.append(f" Home      {home_lat:.7f}, {home_lon:.7f}")

    lines.extend(gnss_lines())

    data_age = now - last_local if last_local else 0.0
    trail_text = f"on ({len(trail)})" if trail_on else "off"
    lines.append("")
    lines.append(f" data age {data_age:.1f}s    trail {trail_text}")

    sat = satellite_lines()
    if sat:
        lines.extend(sat)

    lines.append(
        "[g] goto   [x] jog   [+]/[-] zoom   [0] reset   [t] trail   [c] clear   [i] sat   [n] back   ESC panels"
    )

    return lines
