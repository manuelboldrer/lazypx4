"""The [r] screen: RC transmitter stick positions and raw channel values.

CH1-4 follow the near-universal AETR convention PX4 expects (roll, pitch,
throttle, yaw) and get a small ASCII stick visualisation each; every channel
(up to 18, whatever ``RC_CHANNELS`` actually carries) also gets a plain bar
so a channel with no stick meaning - a mode switch, a knob, a gimbal axis -
is still readable at a glance. See :mod:`lazypx4.state` (``rc_channels`` etc)
and ``handle_rc()`` in :mod:`lazypx4.mavlink.handlers`.
"""

from __future__ import annotations

import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import (
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
from .chrome import graded_color

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


def draw_rc_screen():
    with state.lock:
        rc_received = state.rc_received
        rc_timeout_active = state.rc_timeout_active
        rc_rssi = state.rc_rssi
        rc_lq = state.rc_lq
        rc_failsafe = state.rc_failsafe
        channels = list(state.rc_channels)
        last_rc = state.last_rc

    lines = []

    if not rc_received:
        conn_text = DIM + "NOT CONNECTED" + RESET
    elif rc_timeout_active:
        conn_text = RED + BOLD + "NO SIGNAL" + RESET
    else:
        conn_text = GREEN + "CONNECTED" + RESET

    fs_text = RED + BOLD + "FAILSAFE" + RESET if rc_failsafe else GREEN + "ok" + RESET
    rssi_color = graded_color(rc_rssi, RC_SIGNAL_GOOD, RC_SIGNAL_OK) if rc_received else DIM
    lq_color = graded_color(rc_lq, RC_SIGNAL_GOOD, RC_SIGNAL_OK) if rc_received else DIM

    age_text = DIM + "never" + RESET
    if last_rc:
        age = time.monotonic() - last_rc
        age_color = GREEN if age < 0.5 else (YELLOW if age < 2.0 else RED)
        age_text = f"{age_color}{age:.1f}s ago{RESET}"

    lines.append(
        f" RC: {conn_text}   RSSI: {rssi_color}{rc_rssi}{RESET}"
        f"   LQ: {lq_color}{rc_lq}%{RESET}   {fs_text}   last packet: {age_text}"
    )

    if not rc_received or not any(channels):
        lines.append("")
        lines.append("   " + DIM + "no RC_CHANNELS received yet" + RESET)
        lines.append("")
        lines.append("[r] back    ESC panels")
        return lines

    def ch(index):
        return channels[index] if index < len(channels) else 0

    ch1, ch2, ch3, ch4 = ch(0), ch(1), ch(2), ch(3)

    lines.append("")
    lines.append(ui_section("STICKS", "CH1 roll / CH2 pitch / CH3 throttle / CH4 yaw"))

    left_grid = _stick_grid(_centered_frac(ch4), _throttle_frac(ch3))
    right_grid = _stick_grid(_centered_frac(ch1), _centered_frac(ch2))

    lines.append(f"   {'LEFT: yaw / throttle':<{_STICK_WIDTH}}   {'RIGHT: roll / pitch'}")
    for left_row, right_row in zip(left_grid, right_grid):
        lines.append(f"   {left_row}   {right_row}")

    lines.append(
        f"   CH1 roll: {ch1:4d}   CH2 pitch: {ch2:4d}"
        f"   CH3 thr: {ch3:4d}   CH4 yaw: {ch4:4d}"
    )

    lines.append("")
    lines.append(ui_section("ALL CHANNELS", f"{sum(1 for c in channels if c > 0)}/{len(channels)} active"))

    for i, raw in enumerate(channels, start=1):
        lines.append(f"   CH{i:<2} {_bar(raw)}")

    lines.append("")
    lines.append("[r] back    ESC panels")

    return lines
