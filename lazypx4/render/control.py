"""The [c] control / setpoints screen: attitude, position/velocity, guidance, RC."""

from __future__ import annotations

import time

from ..ansi import BG_GREEN, BLACK, BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import POS_TARGET_FRAME_NAMES
from ..state import state
from .chrome import pos_target_active_axes


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
        rc_rssi = state.rc_rssi
        last_rc = state.last_rc

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

    # --- RC sticks --------------------------------------------
    lines.append("")
    lines.append(ui_section("RC INPUT", f"{age(last_rc)}   RSSI {rc_rssi}"))

    if rc_received and any(rc_channels):
        def stick(index):
            raw = rc_channels[index] if index < len(rc_channels) else 0
            if raw <= 0:
                return "  --  "
            return f"{raw:4d}us"

        lines.append(
            f"   CH1 roll: {stick(0)}   CH2 pitch: {stick(1)}"
            f"   CH3 thr: {stick(2)}   CH4 yaw: {stick(3)}"
        )
    else:
        lines.append("   " + DIM + "no RC channels" + RESET)

    lines.append("")
    lines.append("[r] re-request setpoint streams    [c] back    [ESC] panels")

    return lines
