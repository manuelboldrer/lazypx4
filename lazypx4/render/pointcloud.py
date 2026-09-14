"""The [v] LiDAR point-cloud overview: a summary of the last point-cloud
message plus a scatter view, drawn in the sensor's own frame (not the
vehicle's NED frame the [n] map uses - see lazypx4.lidar). [1] switches to a
top-down (bird's eye) projection, [2] to a front (elevation) projection, [3]
to a 45-degree oblique projection that shows both at once. [t] changes the
subscribed topic without restarting lazypx4.

Points are packed into Unicode Braille characters (each cell holds a 2x4 dot
sub-grid) instead of one "." per terminal cell: an ordinary character grid at
typical terminal size only has a couple thousand cells, which made even a
few hundred sample points look sparse and hard to read as a shape. Braille
gives the same footprint 8x the positional resolution.
"""

from __future__ import annotations

import math
import time

from ..ansi import BLUE, BOLD, CYAN, DIM, GREEN, RED, RESET, YELLOW
from ..config import settings
from ..state import state
from ..util import clamp
from .chrome import BRAILLE_BITS, braille_glyph, content_area

#: Low-to-high gradient for whichever axis isn't shown positionally in the
#: active view (height Z on the top-down view, depth X on the front view) -
#: the blue-to-red order most LiDAR viewers use for a height/depth map.
_COLOR_BANDS = (BLUE, CYAN, GREEN, YELLOW, RED)

_COS45 = _SIN45 = math.sqrt(2.0) / 2.0

#: Per-view axis mapping: which raw (x, y, z) sensor-frame component becomes
#: the screen's horizontal position, vertical position, and the "depth"
#: value colored by - trivial to add another view here.
_VIEWS = {
    "top": {
        "label": "Top-down (X fwd / Y left)",
        "depth_label": "Height Z",
        "horiz": lambda x, y, z: -y,
        "vert": lambda x, y, z: x,
        "depth": lambda x, y, z: z,
    },
    "front": {
        "label": "Front (Y left / Z up)",
        "depth_label": "Depth X",
        "horiz": lambda x, y, z: -y,
        "vert": lambda x, y, z: z,
        "depth": lambda x, y, z: x,
    },
    "oblique": {
        "label": "45 deg oblique (Y left / X+Z diagonal)",
        "depth_label": "Perp. axis",
        "horiz": lambda x, y, z: -y,
        # Forward distance and height both push a point up the screen, at
        # 45 degrees each, so "far and low" and "near and high" points can
        # land in the same place - the trade-off for seeing both axes at
        # once in one flat projection instead of switching between [1]/[2].
        "vert": lambda x, y, z: x * _COS45 + z * _SIN45,
        "depth": lambda x, y, z: x * _SIN45 - z * _COS45,
    },
}


def _color_for(value, value_min, value_max):
    if value_max <= value_min:
        return _COLOR_BANDS[0]
    t = clamp((value - value_min) / (value_max - value_min), 0.0, 1.0)
    index = min(len(_COLOR_BANDS) - 1, int(t * len(_COLOR_BANDS)))
    return _COLOR_BANDS[index]


def _age_text(last_received):
    if not last_received:
        return DIM + "never" + RESET
    age = time.monotonic() - last_received
    color = GREEN if age < 1.0 else (YELLOW if age < 5.0 else RED)
    return f"{color}{age:.1f}s ago{RESET}"


def draw_pointcloud_screen():
    with state.lock:
        supported = state.lidar_supported
        frame_id = state.lidar_frame_id
        point_count = state.lidar_point_count
        rate = state.lidar_rate_hz
        last_received = state.lidar_last_received
        range_min = state.lidar_range_min
        range_max = state.lidar_range_max
        xs = list(state.lidar_sample_x)
        ys = list(state.lidar_sample_y)
        zs = list(state.lidar_sample_z)
        view_range = state.lidar_view_range
        view_mode = state.lidar_view_mode

    lines = []

    if not supported:
        lines.append(
            DIM + "rclpy / sensor_msgs not found - no ROS 2 environment sourced" + RESET
        )
        lines.append(DIM + f"Would subscribe to {settings.lidar_topic}" + RESET)
        lines.append("")
        lines.append(DIM + "v = back    ESC = panels" + RESET)
        return lines

    view = _VIEWS.get(view_mode, _VIEWS["top"])
    view_range = max(view_range, 0.5)

    lines.append(f" Topic: {settings.lidar_topic}   Frame: {frame_id or '--'}")
    lines.append(
        f" Last message: {_age_text(last_received)}"
        f"   Rate: {rate:.1f} Hz   Points/msg: {point_count}"
    )

    depth_vals = [view["depth"](x, y, z) for x, y, z in zip(xs, ys, zs)]

    if xs:
        lines.append(f" Range (sample): {range_min:.2f} - {range_max:.2f} m")
        depth_min, depth_max = min(depth_vals), max(depth_vals)
        swatches = "".join(f"{color}█{RESET}" for color in _COLOR_BANDS)
        lines.append(
            f" {view['depth_label']}: {depth_min:.2f} m {swatches} {depth_max:.2f} m"
        )
    else:
        lines.append(DIM + " No points received yet" + RESET)
        depth_min = depth_max = 0.0

    lines.append("")
    lines.append(f" {view['label']}   range +/-{view_range:.0f} m")

    panel_width, panel_height = content_area()

    # -9 for this screen's own non-grid lines above (topic/message/range/
    # depth-legend/blank/view-header = 6) and the footer below (2, a blank
    # separator + the key hint) - the extra 1 is slack, not an exact count,
    # since box() itself already clips a 1-line overrun harmlessly.
    grid_w = clamp((panel_width - 2) | 1, 21, 103)
    grid_h = clamp((panel_height - 9) | 1, 9, 35)

    dot_w, dot_h = grid_w * 2, grid_h * 4
    dot_half_w, dot_half_h = dot_w / 2.0, dot_h / 2.0

    def dot_position(h, v):
        col = int(round((h / view_range) * dot_half_w) + dot_half_w)
        row = int(dot_half_h - round((v / view_range) * dot_half_h))
        return row, col

    # One entry per character cell hit: [dot bitmask, tallest/farthest depth
    # value seen in it] - the depth that wins a cell's color is whichever of
    # its (up to 8) points has the largest depth value, same "most salient
    # point wins" rule the earlier single-dot-per-cell view used for height.
    cells = {}

    for x, y, z in zip(xs, ys, zs):
        dot_row, dot_col = dot_position(view["horiz"](x, y, z), view["vert"](x, y, z))
        if not (0 <= dot_row < dot_h and 0 <= dot_col < dot_w):
            continue

        char_row, char_col = dot_row // 4, dot_col // 2
        bit = BRAILLE_BITS[(dot_col % 2, dot_row % 4)]
        depth_val = view["depth"](x, y, z)

        entry = cells.get((char_row, char_col))
        if entry is None:
            cells[(char_row, char_col)] = [bit, depth_val]
        else:
            entry[0] |= bit
            if depth_val > entry[1]:
                entry[1] = depth_val

    grid = [[" "] * grid_w for _ in range(grid_h)]
    for (row, col), (bitmask, depth_val) in cells.items():
        glyph = braille_glyph(bitmask)
        grid[row][col] = _color_for(depth_val, depth_min, depth_max) + glyph + RESET

    origin_row, origin_col = dot_position(0.0, 0.0)
    origin_row, origin_col = origin_row // 4, origin_col // 2
    if 0 <= origin_row < grid_h and 0 <= origin_col < grid_w:
        grid[origin_row][origin_col] = BOLD + "+" + RESET

    for row in grid:
        lines.append("  " + "".join(row))

    lines.append("")
    lines.append(
        "[+]/[-] zoom   [0] reset   [1] top   [2] front   [3] 45deg"
        "   [t] topic   v = back   ESC = panels"
    )

    return lines
