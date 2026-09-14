"""The [w] screen: a local V4L2/USB camera preview - see :mod:`lazypx4.camera`.

Full mode packs two vertical pixel samples into each character cell (the top
half foreground / bottom half background of a half-block glyph, in ANSI
truecolor) - the standard terminal-image trick, and a good match for a
typical character cell's roughly 1:2 width:height aspect. Low-bandwidth mode
drops color entirely and samples one pixel per cell into a plain ASCII
brightness ramp, which is what actually cuts the bytes this screen writes to
the terminal on a slow link.
"""

from __future__ import annotations

import time

from .. import camera as camera_mod
from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import settings
from .chrome import content_area

_ASCII_RAMP = " .:-=+*#%@"


def _age_text(last_frame_at):
    if not last_frame_at:
        return DIM + "never" + RESET
    age = time.monotonic() - last_frame_at
    color = GREEN if age < 2.0 else (YELLOW if age < 6.0 else RED)
    return f"{color}{age:.1f}s ago{RESET}"


def _fit_grid(frame_w, frame_h, max_w, max_h):
    """(grid_w, grid_h) character cells that fit `frame_w`x`frame_h` pixels
    into at most `max_w`x`max_h` cells, preserving aspect ratio - each cell
    holds one horizontal pixel sample and two vertical ones."""
    if frame_w <= 0 or frame_h <= 0 or max_w <= 0 or max_h <= 0:
        return 0, 0

    aspect_rows_per_col = frame_h / (2.0 * frame_w)

    grid_w = max_w
    grid_h = int(round(grid_w * aspect_rows_per_col))
    if grid_h > max_h:
        grid_h = max_h
        grid_w = int(round(grid_h / aspect_rows_per_col)) if aspect_rows_per_col > 0 else max_w

    return max(1, min(grid_w, max_w)), max(1, min(grid_h, max_h))


def _sample_axis(count, size):
    """`count` nearest-neighbour source indices (0..size-1) spanning `size`."""
    if count <= 0 or size <= 0:
        return []
    return [min(size - 1, int(i * size / count)) for i in range(count)]


def _pixel(frame, frame_w, x, y):
    idx = (y * frame_w + x) * 3
    return frame[idx], frame[idx + 1], frame[idx + 2]


def _render_color(frame, frame_w, frame_h, grid_w, grid_h):
    xs = _sample_axis(grid_w, frame_w)
    ys = _sample_axis(grid_h * 2, frame_h)

    lines = []
    for row in range(grid_h):
        top_y, bot_y = ys[row * 2], ys[row * 2 + 1]
        parts = []
        for x in xs:
            tr, tg, tb = _pixel(frame, frame_w, x, top_y)
            br, bg, bb = _pixel(frame, frame_w, x, bot_y)
            parts.append(f"\033[38;2;{tr};{tg};{tb}m\033[48;2;{br};{bg};{bb}m▀")
        lines.append("".join(parts) + RESET)
    return lines


def _render_ascii(frame, frame_w, frame_h, grid_w, grid_h):
    xs = _sample_axis(grid_w, frame_w)
    ys = _sample_axis(grid_h, frame_h)

    ramp_last = len(_ASCII_RAMP) - 1
    lines = []
    for y in ys:
        row_chars = []
        for x in xs:
            r, g, b = _pixel(frame, frame_w, x, y)
            luma = 0.299 * r + 0.587 * g + 0.114 * b
            row_chars.append(_ASCII_RAMP[int(luma / 255.0 * ramp_last)])
        lines.append("".join(row_chars))
    return lines


def draw_camera_screen():
    with camera_mod.stats.lock:
        available = camera_mod.stats.available
        checked = camera_mod.stats.checked
        enabled = camera_mod.stats.enabled
        capturing = camera_mod.stats.capturing
        low_bandwidth = camera_mod.stats.low_bandwidth
        device = camera_mod.stats.device or settings.camera_device
        error = camera_mod.stats.error
        frame = camera_mod.stats.frame_rgb
        frame_w = camera_mod.stats.frame_width
        frame_h = camera_mod.stats.frame_height
        frame_count = camera_mod.stats.frame_count
        fps = camera_mod.stats.fps
        last_frame_at = camera_mod.stats.last_frame_at

    lines = []
    lines.append(ui_section("CAMERA", device))

    if not checked:
        lines.append("   " + DIM + "starting up..." + RESET)
    elif not available:
        lines.append("   " + DIM + "'ffmpeg' not found on PATH - camera preview unavailable" + RESET)

    mode_text = YELLOW + "LOW BANDWIDTH" + RESET if low_bandwidth else GREEN + "FULL" + RESET
    state_text = GREEN + BOLD + "ON" + RESET if enabled else DIM + "OFF" + RESET
    lines.append(
        f"   Feed: {state_text}   Mode: {mode_text}"
        f"   Frames: {frame_count}   Rate: {fps:.1f} fps   Last frame: {_age_text(last_frame_at)}"
    )
    if error:
        lines.append("   " + RED + f"error: {error}" + RESET)
    lines.append("")

    if not available:
        lines.append(
            "[d] device   [w] back   [ESC] panels"
        )
        return lines

    if not enabled:
        lines.append("   " + DIM + "feed is off - press [o] to turn it on (needs confirmation)" + RESET)
    elif not frame:
        lines.append("   " + YELLOW + ("capturing first frame..." if capturing else "waiting for a frame...") + RESET)
    else:
        panel_w, panel_h = content_area()
        grid_w, grid_h = _fit_grid(frame_w, frame_h, max(1, panel_w), max(1, panel_h - 4))

        if low_bandwidth:
            lines.extend(_render_ascii(frame, frame_w, frame_h, grid_w, grid_h))
        else:
            lines.extend(_render_color(frame, frame_w, frame_h, grid_w, grid_h))

    lines.append("")
    lines.append(
        "[o] on/off   [b] low-bandwidth   [d] device   [w] back   [ESC] panels"
    )

    return lines
