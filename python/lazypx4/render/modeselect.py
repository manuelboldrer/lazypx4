"""The [m] flight-mode picker."""

from __future__ import annotations

import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW
from ..config import MODE_PAGE_SIZE
from ..mavlink.modes import get_mode_options
from ..search import footer_line
from ..state import session, state
from ..util import clamp
from .chrome import highlight_matches


def draw_mode_select():
    options = get_mode_options()

    with state.lock:
        current_mode = state.mode

        have_custom_modes = bool(state.custom_modes)
        custom_total = state.custom_modes_total
        custom_received = state.custom_modes_received
        custom_complete = state.custom_modes_complete
        custom_requested_at = state.custom_modes_requested_at

    now = time.monotonic()

    if custom_complete:
        source_text = GREEN + "STANDARD MODES PROTOCOL" + RESET
    elif custom_requested_at:
        source_text = YELLOW + f"LOADING ({now - custom_requested_at:.1f}s)" + RESET
    elif have_custom_modes:
        source_text = YELLOW + "PARTIAL" + RESET
    else:
        source_text = DIM + "LEGACY LIST (no custom modes yet)" + RESET

    lines = []

    lines.append(f"Current mode: {BOLD}{current_mode}{RESET}")
    lines.append(
        f"Source: {source_text}"
        f"    Modes: {custom_received}/{custom_total if custom_total else '--'}"
    )
    lines.append("")

    if not options:
        lines.append(RED + "No flight modes reported." + RESET)
    else:
        selected = clamp(session.mode_index, 0, len(options) - 1)
        page_start = (selected // MODE_PAGE_SIZE) * MODE_PAGE_SIZE
        page = options[page_start:page_start + MODE_PAGE_SIZE]
        query = session.search_query

        for local_index, option in enumerate(page):
            index = page_start + local_index
            label = highlight_matches(option["label"], query)
            if index == selected:
                lines.append(BOLD + GREEN + " > " + label + RESET)
            else:
                lines.append("   " + label)

        if len(options) > MODE_PAGE_SIZE:
            lines.append(
                DIM + f"   {selected + 1}/{len(options)}"
                "   (UP/DOWN, PGUP/PGDN, HOME/END)" + RESET
            )

    footer = footer_line()
    if footer:
        lines.append(footer)

    lines.append("")
    lines.append("UP/DOWN = select    ENTER = choose    / search    [r] refresh    [m] back    ESC = panels")
    lines.append("TAKEOFF / LAND / RTL are selected here as PX4 flight modes.")

    return lines
