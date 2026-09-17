"""The [c] control / setpoints screen: attitude, position/velocity, guidance,
RC sticks/channels, wind estimate and raw actuator outputs.

RC: CH1-4 follow the near-universal AETR convention PX4 expects (roll, pitch,
throttle, yaw) and get a small ASCII stick visualisation each; every channel
(up to 18, whatever ``RC_CHANNELS`` actually carries) also gets a plain bar
so a channel with no stick meaning - a mode switch, a knob, a gimbal axis -
is still readable at a glance. See :mod:`lazypx4.state` (``rc_channels`` etc)
and ``handle_rc()`` in :mod:`lazypx4.mavlink.handlers`.
"""

from __future__ import annotations

import time

from ..ansi import BG_GREEN, BLACK, BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import (
    POS_TARGET_FRAME_NAMES,
    RC_PWM_CENTER,
    RC_PWM_DISPLAY_MAX,
    RC_PWM_DISPLAY_MIN,
    RC_PWM_MAX,
    RC_PWM_MIN,
    RC_SIGNAL_GOOD,
    RC_SIGNAL_OK,
)
from ..state import state
from ..util import clamp
from .chrome import graded_color, pos_target_active_axes

_STICK_WIDTH = 19
_STICK_HEIGHT = 9

_BAR_WIDTH = 32


def _centered_frac(raw):
    """-1..1 for a channel centred on RC_PWM_CENTER, clamped (a transmitter's
    endpoint calibration can overtravel slightly past 1000/2000)."""
    if raw <= 0:
        return 0.0
    return clamp((raw - RC_PWM_CENTER) / ((RC_PWM_MAX - RC_PWM_MIN) / 2.0), -1.0, 1.0)


def _throttle_frac(raw):
    """0..1 mapped to -1..1 (bottom..top of the stick grid) - throttle has no
    centre spring, so it reads end to end rather than around a midpoint."""
    if raw <= 0:
        return -1.0
    unit = clamp((raw - RC_PWM_MIN) / (RC_PWM_MAX - RC_PWM_MIN), 0.0, 1.0)
    return unit * 2.0 - 1.0


def _stick_grid(frac_x, frac_y):
    width, height = _STICK_WIDTH, _STICK_HEIGHT
    center_col, center_row = width // 2, height // 2

    grid = [[DIM + "·" + RESET for _ in range(width)] for _ in range(height)]
    for c in range(width):
        grid[center_row][c] = DIM + "─" + RESET
    for r in range(height):
        grid[r][center_col] = DIM + "│" + RESET
    grid[center_row][center_col] = DIM + "┼" + RESET

    col = clamp(int(round((frac_x + 1.0) / 2.0 * (width - 1))), 0, width - 1)
    row = clamp(int(round((1.0 - frac_y) / 2.0 * (height - 1))), 0, height - 1)
    grid[row][col] = BOLD + GREEN + "●" + RESET

    return ["".join(cells) for cells in grid]


def _bar(raw, width=_BAR_WIDTH):
    if raw <= 0:
        return DIM + "·" * width + RESET + "   " + DIM + "unused" + RESET

    frac = clamp((raw - RC_PWM_DISPLAY_MIN) / (RC_PWM_DISPLAY_MAX - RC_PWM_DISPLAY_MIN), 0.0, 1.0)
    filled = int(round(frac * width))
    bar = GREEN + "█" * filled + RESET + DIM + "░" * (width - filled) + RESET

    pct = (raw - RC_PWM_MIN) / (RC_PWM_MAX - RC_PWM_MIN) * 100.0
    return f"{bar}   {raw:4d} us  {pct:5.1f}%"


def draw_control_screen():
    with state.lock:
        armed = state.armed
        mode = state.mode

        roll = state.roll
        pitch = state.pitch
        yaw = state.yaw
        roll_rate = state.roll_rate
        pitch_rate = state.pitch_rate
        yaw_rate = state.yaw_rate

        throttle = state.thrust

        at_roll = state.att_target_roll
        at_pitch = state.att_target_pitch
        at_yaw = state.att_target_yaw
        at_thrust = state.att_target_thrust
        at_roll_rate = state.att_target_roll_rate
        at_pitch_rate = state.att_target_pitch_rate
        at_yaw_rate = state.att_target_yaw_rate
        last_att_target = state.last_att_target

        px = state.x
        py = state.y
        pz = state.z
        pvx = state.vx
        pvy = state.vy
        pvz = state.vz

        pt_x = state.pos_target_x
        pt_y = state.pos_target_y
        pt_z = state.pos_target_z
        pt_vx = state.pos_target_vx
        pt_vy = state.pos_target_vy
        pt_vz = state.pos_target_vz
        pt_yaw = state.pos_target_yaw
        pt_yaw_rate = state.pos_target_yaw_rate
        pt_mask = state.pos_target_type_mask
        pt_frame = state.pos_target_frame
        last_pos_target = state.last_pos_target

        nav_bearing = state.nav_bearing
        nav_target_bearing = state.nav_target_bearing
        nav_wp_dist = state.nav_wp_dist
        nav_alt_error = state.nav_alt_error
        nav_aspd_error = state.nav_aspd_error
        nav_xtrack_error = state.nav_xtrack_error
        last_nav_output = state.last_nav_output

        rc_channels = list(state.rc_channels)
        rc_received = state.rc_received
        rc_timeout_active = state.rc_timeout_active
        rc_rssi = state.rc_rssi
        rc_lq = state.rc_lq
        rc_failsafe = state.rc_failsafe
        last_rc = state.last_rc

        wind_speed = state.wind_speed
        wind_direction = state.wind_direction
        wind_speed_z = state.wind_speed_z
        last_wind = state.last_wind

        servo_outputs = list(state.servo_outputs)
        last_servo_output = state.last_servo_output

    now = time.monotonic()

    def age(timestamp):
        if not timestamp:
            return DIM + "never" + RESET
        delta = now - timestamp
        color = GREEN if delta < 2.0 else (YELLOW if delta < 10.0 else RED)
        return f"{color}{delta:.1f}s{RESET}"

    def err(value):
        color = GREEN if abs(value) < 1.0 else (YELLOW if abs(value) < 5.0 else RED)
        return f"{color}{value:+.2f}{RESET}"

    def ang_err(target, actual):
        return err((target - actual + 180.0) % 360.0 - 180.0)

    lines = []
    lines.append(
        f" MODE: {BOLD}{mode}{RESET}    "
        + (BG_GREEN + BLACK + BOLD + " ARMED " + RESET if armed else DIM + " DISARMED " + RESET)
    )

    # --- Attitude / rate control ---------------------------------
    lines.append("")
    lines.append(ui_section("ATTITUDE CONTROL", f"setpoint {age(last_att_target)}"))
    lines.append(f"   {'':10}{'roll':>10}{'pitch':>10}{'yaw':>10}")
    lines.append(f"   {'actual':<10}{roll:>10.2f}{pitch:>10.2f}{yaw:>10.2f}  deg")
    lines.append(f"   {'target':<10}{at_roll:>10.2f}{at_pitch:>10.2f}{at_yaw:>10.2f}  deg")
    lines.append(
        f"   error      roll {ang_err(at_roll, roll)}   pitch {ang_err(at_pitch, pitch)}"
        f"   yaw {ang_err(at_yaw, yaw)}  deg"
    )
    lines.append("")
    lines.append(f"   {'rate now':<10}{roll_rate:>10.1f}{pitch_rate:>10.1f}{yaw_rate:>10.1f}  deg/s")
    lines.append(
        f"   {'rate sp':<10}{at_roll_rate:>10.1f}{at_pitch_rate:>10.1f}{at_yaw_rate:>10.1f}  deg/s"
    )
    lines.append(
        f"   throttle (VFR_HUD): {throttle:5.1f} %      thrust setpoint: {at_thrust:.3f}"
    )

    # --- Position / velocity setpoints --------------------------
    lines.append("")
    frame_name = POS_TARGET_FRAME_NAMES.get(pt_frame, f"frame {pt_frame}")
    axes = pos_target_active_axes(pt_mask) if last_pos_target else []
    axes_text = ", ".join(axes) if axes else DIM + "none" + RESET
    lines.append(ui_section("POSITION / VELOCITY SETPOINT", age(last_pos_target)))

    if last_pos_target:
        lines.append(f"   frame: {frame_name}   controlling: {axes_text}")
        lines.append(f"   {'':10}{'x':>11}{'y':>11}{'z':>11}")
        lines.append(f"   {'pos now':<10}{px:>11.2f}{py:>11.2f}{pz:>11.2f}  m")
        lines.append(f"   {'pos sp':<10}{pt_x:>11.2f}{pt_y:>11.2f}{pt_z:>11.2f}  m")
        lines.append(f"   {'vel now':<10}{pvx:>11.2f}{pvy:>11.2f}{pvz:>11.2f}  m/s")
        lines.append(f"   {'vel sp':<10}{pt_vx:>11.2f}{pt_vy:>11.2f}{pt_vz:>11.2f}  m/s")
        lines.append(
            f"   yaw sp: {pt_yaw:+.1f} deg   yaw-rate sp: {pt_yaw_rate:+.1f} deg/s"
        )
    else:
        lines.append(
            "   " + DIM + "no POSITION_TARGET_LOCAL_NED (manual / rate mode, or not streamed)" + RESET
        )

    # --- Guidance (mission / auto) -----------------------------
    lines.append("")
    lines.append(ui_section("GUIDANCE · NAV_CONTROLLER_OUTPUT", f"{age(last_nav_output)}"))

    if last_nav_output:
        lines.append(
            f"   WP distance: {nav_wp_dist:.1f} m   altitude error: {err(nav_alt_error)} m"
            f"   crosstrack: {err(nav_xtrack_error)} m"
        )
        lines.append(
            f"   nav bearing: {nav_bearing:.0f} deg   target bearing: {nav_target_bearing:.0f} deg"
            f"   airspeed error: {nav_aspd_error:+.1f} m/s"
        )
    else:
        lines.append("   " + DIM + "not in an auto / mission mode" + RESET)

    # --- RC input: connection / sticks / all channels ------------
    lines.append("")

    if not rc_received:
        rc_conn_text = DIM + "NOT CONNECTED" + RESET
    elif rc_timeout_active:
        rc_conn_text = RED + BOLD + "NO SIGNAL" + RESET
    else:
        rc_conn_text = GREEN + "CONNECTED" + RESET

    rc_fs_text = RED + BOLD + "FAILSAFE" + RESET if rc_failsafe else GREEN + "ok" + RESET
    rssi_color = graded_color(rc_rssi, RC_SIGNAL_GOOD, RC_SIGNAL_OK) if rc_received else DIM
    lq_color = graded_color(rc_lq, RC_SIGNAL_GOOD, RC_SIGNAL_OK) if rc_received else DIM

    lines.append(ui_section("RC INPUT", age(last_rc)))
    lines.append(
        f"   {rc_conn_text}   RSSI: {rssi_color}{rc_rssi}{RESET}"
        f"   LQ: {lq_color}{rc_lq}%{RESET}   {rc_fs_text}"
    )

    if not rc_received or not any(rc_channels):
        lines.append("   " + DIM + "no RC_CHANNELS received yet" + RESET)
    else:
        def ch(index):
            return rc_channels[index] if index < len(rc_channels) else 0

        ch1, ch2, ch3, ch4 = ch(0), ch(1), ch(2), ch(3)

        lines.append("")
        lines.append(f"   {'LEFT: yaw / throttle':<{_STICK_WIDTH}}   {'RIGHT: roll / pitch'}")

        left_grid = _stick_grid(_centered_frac(ch4), _throttle_frac(ch3))
        right_grid = _stick_grid(_centered_frac(ch1), _centered_frac(ch2))

        for left_row, right_row in zip(left_grid, right_grid):
            lines.append(f"   {left_row}   {right_row}")

        lines.append(
            f"   CH1 roll: {ch1:4d}   CH2 pitch: {ch2:4d}"
            f"   CH3 thr: {ch3:4d}   CH4 yaw: {ch4:4d}"
        )

        lines.append("")
        active = sum(1 for c in rc_channels if c > 0)
        lines.append(ui_section("ALL CHANNELS", f"{active}/{len(rc_channels)} active"))

        for i, raw in enumerate(rc_channels, start=1):
            lines.append(f"   CH{i:<2} {_bar(raw)}")

    # --- Wind estimate ------------------------------------------
    lines.append("")
    lines.append(ui_section("WIND", f"{age(last_wind)}"))

    if last_wind:
        lines.append(
            f"   Horizontal: {wind_speed:5.1f} m/s   from {wind_direction:5.1f} deg"
            f"   Vertical: {wind_speed_z:+5.1f} m/s"
        )
    else:
        lines.append("   " + DIM + "no WIND / WIND_COV messages yet" + RESET)

    # --- Actuator / servo outputs --------------------------------
    lines.append("")
    lines.append(ui_section("ACTUATOR OUTPUTS", f"{age(last_servo_output)}"))

    if last_servo_output:
        row1 = "   " + "  ".join(f"S{i + 1} {v:4d}" for i, v in enumerate(servo_outputs[:4]))
        row2 = "   " + "  ".join(f"S{i + 1} {v:4d}" for i, v in enumerate(servo_outputs[4:], start=4))
        lines.append(row1 + "  us")
        if any(servo_outputs[4:]):
            lines.append(row2 + "  us")
    else:
        lines.append("   " + DIM + "no SERVO_OUTPUT_RAW messages yet" + RESET)

    lines.append("")
    lines.append("[r] re-request setpoint streams    [c] back    [ESC] panels")

    return lines
