"""The [g] scrolling event / warning / failsafe log."""

from __future__ import annotations

from ..ansi import DIM, RESET
from ..search import footer_line
from ..state import session, state
from ..util import clamp
from .chrome import highlight_matches, level_color, log_page_size


def draw_log_screen():
    with state.lock:
        events = list(state.events)

    total = len(events)
    page_size = log_page_size()

    start = clamp(session.log_scroll, 0, max(0, total - page_size))
    visible = events[start:start + page_size]

    lines = []

    query = session.search_query

    if not visible:
        lines.append(DIM + "No events." + RESET)
    else:
        for event in visible:
            color = level_color(event.level)

            lines.append(
                f"{DIM}{event.time_string}{RESET} "
                f"{color}{event.level:<8}{RESET} "
                f"{highlight_matches(event.message, query)}"
            )

    footer = footer_line()
    if footer:
        lines.append(footer)

    lines.append("")
    lines.append(
        f"Events: {total}    Showing: "
        f"{start + 1 if total else 0}-{min(start + page_size, total)}"
    )
    lines.append("UP/DOWN scroll   PgUp/PgDn page   / search   c=clear   g=back   ESC=panels")

    return lines
