"""The [t] MAVLink shell (NSH) screen."""

from __future__ import annotations

import time

from ..ansi import DIM, GREEN, RESET
from ..state import session, state
from ..util import clamp
from .chrome import content_area


def draw_shell_screen():
    with state.lock:
        history = list(state.shell_lines)
        partial = state.shell_partial
        active = state.shell_active
        last_activity = state.shell_last_activity

    display_lines = history + [partial]

    _, panel_height = content_area()

    non_scroll = 5  # status, blank, blank, footer, footer
    available = max(3, panel_height - non_scroll)

    total = len(display_lines)
    max_scroll = max(0, total - available)

    if session.shell_follow:
        session.shell_scroll = max_scroll
    else:
        session.shell_scroll = clamp(session.shell_scroll, 0, max_scroll)
        if session.shell_scroll >= max_scroll:
            session.shell_follow = True

    start = session.shell_scroll
    visible = display_lines[start:start + available]

    lines = []

    if active:
        status_text = GREEN + "OPEN" + RESET
    else:
        status_text = DIM + "NOT OPENED" + RESET

    now = time.monotonic()
    idle_text = f"{now - last_activity:.0f}s ago" if last_activity else "--"
    follow_text = "FOLLOW" if session.shell_follow else "MANUAL"

    lines.append(
        f" Session: {status_text}"
        f"    Last output: {idle_text}"
        f"    Scroll: {follow_text}"
    )
    lines.append("")

    if not any(display_lines):
        lines.append(DIM + " (no output yet - try pressing ENTER)" + RESET)
    else:
        for local_index, line in enumerate(visible):
            absolute_index = start + local_index

            if absolute_index == total - 1:
                lines.append(" " + line + "_")
            else:
                lines.append(" " + line)

    lines.append("")
    lines.append("Type to send    ENTER = run    Ctrl+C = interrupt")
    lines.append(
        "UP/DOWN/PGUP/PGDN = scroll    END = jump to live    ESC = panels (session stays open)"
    )

    return lines
