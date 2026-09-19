"""Screen rendering.

Each screen is a function that reads :data:`lazypx4.state.state` under the
lock and returns a list of content-line strings. :func:`draw` dispatches to
the active screen, then hands its lines to
:func:`lazypx4.render.chrome.draw_frame`, which wraps them in the bordered
main panel, paints the sidebar of every other screen next to it, clips
everything to the real terminal, and writes the frame. :func:`draw` never
lets a rendering bug escape into the main loop.
"""

from __future__ import annotations

from ..ansi import RED, RESET
from ..eventlog import log_error
from ..panels import SCREEN_TITLES
from ..state import session
from .about import draw_about_screen
from .calibration import draw_calibration_screen
from .camera import draw_camera_screen
from .chrome import draw_frame, draw_lines, release_frame_size, snapshot_frame_size
from .control import draw_control_screen
from .dashboard import draw_dashboard
from .eventlog_screen import draw_log_screen
from .firmware import draw_firmware_screen
from .flightlog import draw_flight_log_screen
from .hostinfo import draw_host_screen
from .mapview import draw_map_screen
from .modeselect import draw_mode_select
from .parameters import draw_parameter_screen
from .pointcloud import draw_pointcloud_screen
from .shell import draw_shell_screen

DRAW_FUNCTIONS = {
    "dashboard": draw_dashboard,
    "about": draw_about_screen,
    "mode_select": draw_mode_select,
    "log": draw_log_screen,
    "flight_logs": draw_flight_log_screen,
    "parameters": draw_parameter_screen,
    "control": draw_control_screen,
    "camera": draw_camera_screen,
    "map": draw_map_screen,
    "calibration": draw_calibration_screen,
    "shell": draw_shell_screen,
    "host": draw_host_screen,
    "firmware": draw_firmware_screen,
    "pointcloud": draw_pointcloud_screen,
}


def draw():
    """Render the active screen, falling back to the dashboard on error.

    A rendering bug on one screen must never take down a session connected to
    a real vehicle: fall back to the dashboard and surface the error in the
    event log instead of letting it propagate out of the main loop.
    """
    # One terminal-size reading for everything this frame touches (the
    # screen's own width-dependent formatting, the sidebar/box layout, the
    # final per-line clip) - a resize landing mid-frame used to let those
    # disagree, which showed up as the title/border corrupting specifically
    # while actively resizing.
    snapshot_frame_size()
    try:
        try:
            screen = session.screen
            handler = DRAW_FUNCTIONS.get(screen, draw_dashboard)
            lines = handler()
            draw_frame(lines, SCREEN_TITLES.get(screen, "PX4"), screen)
            return
        except Exception as exc:
            broken = session.screen
            log_error(f"Screen '{broken}' failed to render: {exc}")

            if broken != "dashboard":
                session.screen = "dashboard"

        try:
            draw_frame(draw_dashboard(), SCREEN_TITLES["dashboard"], "dashboard")
        except Exception as exc:
            try:
                draw_lines([RED + f" Dashboard render error: {exc}" + RESET])
            except Exception:
                pass
    finally:
        release_frame_size()
