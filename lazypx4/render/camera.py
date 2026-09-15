"""The [w] screen: up to two ROS 2 camera feeds - see :mod:`lazypx4.camera`.
Each configured topic (``settings.camera_topic_1``/``camera_topic_2``) gets
its own stacked pane, sized by splitting the panel's height between however
many of the two slots are actually in use.

Full mode packs two vertical pixel samples into each character cell (the top
half foreground / bottom half background of a half-block glyph, in ANSI
truecolor) - the standard terminal-image trick, and a good match for a
typical character cell's roughly 1:2 width:height aspect. Low-bandwidth mode
drops color entirely and samples one pixel per cell into a plain ASCII
brightness ramp, capped to a small column count - a purely client-side
choice (unlike the old ffmpeg-capture days, lazypx4 doesn't control what the
publisher sends) that cuts the bytes this screen writes to the terminal on a
slow link.
"""

from __future__ import annotations

import time

from .. import camera as camera_mod
from ..ansi import DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import CAMERA_LOW_BW_MAX_COLS
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


def _render_slot(slot, index, low_bandwidth, max_w, max_h):
    lines = []
    header = f" [{index + 1}] {slot.topic}"
    if slot.msg_kind == "compressed":
        header += " (compressed)"
    lines.append(header)

    if slot.error:
        lines.append("   " + RED + f"error: {slot.error}" + RESET)
        return lines

    lines.append(
        f"   Last frame: {_age_text(slot.last_frame_at)}"
        f"   Rate: {slot.fps:.1f} fps"
        f"   Size: {slot.frame_width}x{slot.frame_height}"
    )
    if slot.note:
        lines.append("   " + YELLOW + slot.note + RESET)

    if not slot.frame_rgb:
        lines.append("   " + DIM + "waiting for a frame..." + RESET)
        return lines

    body_h = max(1, max_h - len(lines))
    if low_bandwidth:
        grid_w, grid_h = _fit_grid(
            slot.frame_width, slot.frame_height, min(max_w, CAMERA_LOW_BW_MAX_COLS), body_h
        )
        lines.extend(_render_ascii(slot.frame_rgb, slot.frame_width, slot.frame_height, grid_w, grid_h))
    else:
        grid_w, grid_h = _fit_grid(slot.frame_width, slot.frame_height, max_w, body_h)
        lines.extend(_render_color(slot.frame_rgb, slot.frame_width, slot.frame_height, grid_w, grid_h))

    return lines


def draw_camera_screen():
    with camera_mod.stats.lock:
        checked = camera_mod.stats.checked
        available = camera_mod.stats.available
        low_bandwidth = camera_mod.stats.low_bandwidth
        slots = list(camera_mod.stats.slots)

    lines = []
    lines.append(ui_section("CAMERA"))

    if not checked:
        lines.append("   " + DIM + "starting up..." + RESET)
        return lines
    if not available:
        lines.append("   " + DIM + "rclpy / sensor_msgs not found - no ROS 2 environment sourced" + RESET)
        lines.append("")
        lines.append("[1] topic 1   [2] topic 2   [w] back   [ESC] panels")
        return lines

    mode_text = YELLOW + "LOW BANDWIDTH" + RESET if low_bandwidth else GREEN + "FULL" + RESET
    lines.append(f"   Render: {mode_text}")
    lines.append("")

    configured = [(i, s) for i, s in enumerate(slots) if s.topic]
    panel_w, panel_h = content_area()
    footer_lines = 2

    if not configured:
        lines.append(
            "   " + DIM
            + "no camera topics set - [1]/[2] to subscribe to a sensor_msgs/Image "
              "or CompressedImage topic"
            + RESET
        )
    else:
        avail_h = max(len(configured), panel_h - len(lines) - footer_lines)
        pane_h = avail_h // len(configured)
        for i, (index, slot) in enumerate(configured):
            if i > 0:
                lines.append("")
            lines.extend(_render_slot(slot, index, low_bandwidth, panel_w, pane_h))

    lines.append("")
    lines.append(
        "[1] topic 1   [2] topic 2   [b] low-bandwidth   [w] back   [ESC] panels"
    )

    return lines
