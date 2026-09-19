"""Incremental "/" search shared by the list screens.

``session.search_query`` is a single term matched case-insensitively against a
screen-specific text row for each list entry. The render side highlights
matches; :func:`jump` moves the screen's cursor between them (``n`` / ``N``).
"""

from __future__ import annotations

from .ansi import DIM, RESET, YELLOW
from .state import session, state
from .util import clamp

#: Screens that support "/" search.
SEARCHABLE = ("mode_select", "log", "flight_logs", "parameters")


def _mode_rows():
    from .mavlink.modes import get_mode_options

    return [option["label"] for option in get_mode_options()]


def _log_rows():
    with state.lock:
        return [
            f"{event.time_string} {event.level} {event.message}"
            for event in state.events
        ]


def _flight_log_rows():
    with state.lock:
        logs = [
            state.flight_logs[log_id]
            for log_id in state.flight_log_order
            if log_id in state.flight_logs
        ]
    return [f"{e.id:06d} {e.time_string} {e.size_string}" for e in logs]


def _parameter_rows():
    from .mavlink.parameters import get_visible_parameters, parameter_format_value

    return [f"{p.name} {parameter_format_value(p)}" for p in get_visible_parameters()]


_ROWS = {
    "mode_select": _mode_rows,
    "log": _log_rows,
    "flight_logs": _flight_log_rows,
    "parameters": _parameter_rows,
}


def rows_for_screen(screen=None):
    fn = _ROWS.get(screen or session.screen)
    return fn() if fn else []


def match_indices(rows, query):
    q = query.lower()
    return [i for i, row in enumerate(rows) if q in row.lower()]


def _current_index(screen, row_count):
    if screen == "mode_select":
        return session.mode_index % row_count if row_count else 0
    if screen == "log":
        return clamp(session.log_scroll, 0, max(0, row_count - 1))
    if screen == "flight_logs":
        with state.lock:
            return clamp(state.flight_log_index, 0, max(0, row_count - 1))
    if screen == "parameters":
        with state.lock:
            return clamp(state.parameter_index, 0, max(0, row_count - 1))
    return 0


def _set_index(screen, index, row_count):
    if screen == "mode_select":
        session.mode_index = index
    elif screen == "log":
        # Deferred import: lazypx4.render imports this module (for
        # footer_line()) while it is itself being set up, so importing
        # render.chrome at module level here would be circular - see
        # render.chrome.log_page_size's docstring.
        from .render.chrome import log_page_size

        session.log_scroll = clamp(index, 0, max(0, row_count - log_page_size()))
    elif screen == "flight_logs":
        with state.lock:
            state.flight_log_index = index
    elif screen == "parameters":
        with state.lock:
            state.parameter_index = index
            state.parameter_page = index // session.param_page_size


def jump(direction):
    """Move the active screen's cursor to a match.

    ``direction`` 0 = first match at/after the cursor (used while typing),
    ``> 0`` = next match, ``< 0`` = previous match. Wraps around.
    """
    if not session.search_query:
        return

    screen = session.screen
    rows = rows_for_screen(screen)
    if not rows:
        return

    matches = match_indices(rows, session.search_query)
    if not matches:
        return

    cur = _current_index(screen, len(rows))

    if direction == 0:
        target = next((m for m in matches if m >= cur), matches[0])
    elif direction > 0:
        target = next((m for m in matches if m > cur), matches[0])
    else:
        earlier = [m for m in matches if m < cur]
        target = earlier[-1] if earlier else matches[-1]

    _set_index(screen, target, len(rows))


def footer_line():
    """One footer line describing the search state, or "" when idle."""
    if session.search_active:
        return DIM + " /" + RESET + session.search_query + "_"

    if not session.search_query:
        return ""

    count = len(match_indices(rows_for_screen(), session.search_query))
    if count:
        return (
            DIM
            + f" search {session.search_query!r}  {count} match(es)"
            + "   n/N next·prev   / edit   ESC clear"
            + RESET
        )
    return (
        YELLOW + f" search {session.search_query!r}  no matches" + RESET
        + DIM + "   / edit   ESC clear" + RESET
    )
