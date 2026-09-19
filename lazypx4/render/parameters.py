"""The [p] parameter browse / search / edit screen."""

from __future__ import annotations

import time

from .. import parammeta
from ..ansi import BOLD, CYAN, DIM, GREEN, RED, RESET, YELLOW, truncate_visible, visible_length
from ..config import KEY_PARAMETER_COLUMNS, PARAM_PAGE_SIZE, settings
from ..mavlink.parameters import (
    get_visible_parameters,
    parameter_format_value,
    parameter_type_name,
)
from ..search import footer_line
from ..state import session, state
from ..util import clamp
from .chrome import content_area, highlight_matches


def _detail_lines(parameter, width, expanded):
    """Description / allowed values / range of the selected parameter - the
    same information QGC shows next to a parameter. ``expanded`` (while
    editing) lists every option instead of a one-line summary."""
    entry = parammeta.get(parameter.name)
    if not entry:
        if not parammeta.source():
            return [YELLOW + " parameter descriptions not loaded (lazypx4/data/param_meta.json.xz missing)" + RESET]
        return [DIM + " no description available for this parameter" + RESET]

    def clip(text):
        return truncate_visible(text, max(10, width - 2))

    out = []
    reboot = f"  {YELLOW}(reboot required){RESET}" if entry.get("r") else ""
    out.append(clip(f" {BOLD}{entry.get('s', '')}{RESET}") + reboot)

    if "e" in entry:
        current = parameter.value
        options = []
        for value, text in entry["e"]:
            item = f"{value} ({text})"
            if abs(float(value) - current) < 1e-6:
                item = GREEN + BOLD + item + RESET
            options.append(item)
        if expanded:
            out.extend(clip("   " + item) for item in options)
        else:
            out.append(clip("   " + "   ".join(options)))
    elif "b" in entry:
        raw = int(round(parameter.value))
        if expanded:
            for bit, text in entry["b"]:
                mark = f"{GREEN}[x]{RESET}" if raw >> bit & 1 else f"{DIM}[ ]{RESET}"
                out.append(clip(f"   bit {bit} {mark} {text}"))
        else:
            out.append(clip("   " + "   ".join(
                f"{GREEN if raw >> bit & 1 else DIM}{bit}{RESET} {text}"
                for bit, text in entry["b"]
            )))
    else:
        facts = []
        if "min" in entry or "max" in entry:
            facts.append(f"range {entry.get('min', '..')} .. {entry.get('max', '..')}")
        if "u" in entry:
            facts.append(f"unit {entry['u']}")
        if facts:
            out.append(clip("   " + "   ".join(facts)))

    if "d" in entry:
        out.append(DIM + clip(f"   default {entry['d']}") + RESET)

    if expanded and entry.get("l"):
        out.append(DIM + clip(" " + entry["l"]) + RESET)

    return out


def _key_value_text(parameter, entry, room):
    """Right-hand side of a KEY PARAMETERS cell: the meaning first (as the
    user reads it - ``GPS``, ``2.5 m``), the raw number dimmed after it."""
    if parameter is None:
        return DIM + "--" + RESET

    number = parameter_format_value(parameter)
    if "e" in entry:
        label = parammeta.enum_label(entry, parameter.value)
        text = f"{GREEN}{number}{RESET} {DIM}({label if label is not None else '?'}){RESET}"
    elif "b" in entry:
        labels = parammeta.bitmask_labels(entry, parameter.value)
        text = f"{GREEN}{number}{RESET} {DIM}({', '.join(labels) or 'none'}){RESET}"
    else:
        unit = f" {entry['u']}" if entry.get("u") else ""
        text = f"{GREEN}{number}{RESET}{DIM}{unit}{RESET}"
    return truncate_visible(text, room)


def _key_parameter_lines(width):
    """The KEY PARAMETERS panel: KEY_PARAMETER_COLUMNS laid out side by side,
    each parameter as ``NAME  value``. Empty list if the terminal is too
    narrow for two columns."""
    col_width = (width - 4) // 2
    if col_width < 34:
        return []

    with state.lock:
        params = dict(state.parameters)

    columns = []
    for groups in KEY_PARAMETER_COLUMNS:
        cells = []
        for heading, names in groups:
            if cells:
                cells.append("")
            cells.append(BOLD + CYAN + heading + RESET)
            for name in names:
                value = _key_value_text(
                    params.get(name), parammeta.get(name), max(1, col_width - 19)
                )
                cells.append(f"{name:<18} {value}")
        columns.append(cells)

    out = [DIM + " KEY PARAMETERS" + RESET]
    for row in range(max(len(c) for c in columns)):
        parts = []
        for cells in columns:
            cell = cells[row] if row < len(cells) else ""
            cell = truncate_visible(cell, col_width)
            parts.append(cell + " " * (col_width - visible_length(cell)))
        out.append("  " + "  ".join(parts).rstrip())
    return out


def draw_parameter_screen():
    visible = get_visible_parameters()

    with state.lock:
        total_loaded = len(state.parameters)
        parameter_count = state.parameter_count
        received = state.parameters_received
        complete = state.parameters_complete
        requested_at = state.parameters_requested_at

        parameter_view = state.parameter_view

        selected_index = state.parameter_index

        edit_active = state.parameter_edit_active
        edit_name = state.parameter_edit_name
        edit_buffer = state.parameter_edit_buffer

        set_pending = state.parameter_set_pending
        set_name = state.parameter_set_name

    now = time.monotonic()
    width = content_area()[0]

    if complete:
        status_text = GREEN + "COMPLETE" + RESET
    elif requested_at:
        elapsed = now - requested_at
        status_text = YELLOW + f"LOADING ({elapsed:.1f}s)" + RESET
    elif total_loaded:
        status_text = YELLOW + "PARTIAL" + RESET
    else:
        status_text = DIM + "NOT LOADED" + RESET

    lines = []

    expected_text = str(parameter_count) if parameter_count else "--"

    lines.append(
        f" Status: {status_text}"
        f"    Received: {received}/{expected_text}"
        f"    View: {BOLD}{parameter_view}{RESET}"
    )

    if set_pending:
        lines.append(YELLOW + f" Pending PARAM_SET: {set_name}" + RESET)
    else:
        lines.append("")

    query = session.search_query

    # Lay the screen out top-down: the KEY PARAMETERS panel and the selected
    # parameter's detail block take what they need, and the list gets what is
    # left of the panel height (at most PARAM_PAGE_SIZE rows, at least 5). If
    # even that would leave the list under 6 rows the key panel is dropped.
    height = content_area()[1]
    footer = footer_line()
    detail = []
    if visible:
        selected_param = visible[min(selected_index, len(visible) - 1)]
        detail = _detail_lines(selected_param, width, expanded=edit_active) + [""]
    changed_note = 2 if parameter_view == "CHANGED" else 0
    fixed = len(lines) + 1 + (1 if footer else 0) + len(detail) + 2 + changed_note
    key_panel = _key_parameter_lines(width)
    if key_panel and height - fixed - (len(key_panel) + 1) < 6:
        key_panel = []
    if key_panel:
        key_panel.append("")
    page_size = clamp(height - fixed - len(key_panel), 5, PARAM_PAGE_SIZE)
    session.param_page_size = page_size

    lines.extend(key_panel)

    if not visible:
        lines.append(RED + " No parameters in the current view." + RESET)
    else:
        page = selected_index // page_size
        start = page * page_size
        page_items = visible[start:start + page_size]

        for local_index, parameter in enumerate(page_items):
            absolute_index = start + local_index
            selected = absolute_index == selected_index

            value_text = parameter_format_value(parameter)
            type_text = parameter_type_name(parameter.param_type)

            marker = ">" if selected else " "

            if parameter.pending:
                status = YELLOW + " PENDING" + RESET
            else:
                default_changed = parameter.changed_from_default

                if default_changed is True:
                    status = YELLOW + " CHANGED" + RESET
                elif default_changed is None and parameter.changed_from_startup:
                    status = YELLOW + " CHANGED*" + RESET
                else:
                    status = ""

            name_text = highlight_matches(f"{parameter.name[:34]:<34}", query)
            value_text = value_text[:12]

            # What the number means (enum label / set bits / unit), as QGC shows.
            mean_room = max(0, width - (4 + 34 + 1 + 12 + 1 + 8 + 9))
            meaning = parammeta.meaning(parammeta.get(parameter.name), parameter.value)
            meaning = truncate_visible(meaning, mean_room) if mean_room else ""
            status_pad = f" {status.lstrip()}" if status else ""

            lines.append(
                f" {GREEN if selected else ''}"
                f" {marker} "
                f"{name_text}"
                f" {value_text:>12}"
                f" {type_text:<8}"
                f"{CYAN}{meaning:<{mean_room}}{RESET if not selected else GREEN}"
                f"{status_pad}"
                f"{RESET if selected else ''}"
            )

    if footer:
        lines.append(footer)

    lines.append("")
    lines.extend(detail)

    if edit_active:
        lines.append(BOLD + f" Edit {edit_name}: " + RESET + edit_buffer + "_")
        lines.append(DIM + "ENTER = apply    ESC = cancel" + RESET)
    else:
        lines.append("UP/DOWN = select    PGUP/PGDN = page    ENTER = edit    / search")
        lines.append("[v] ALL/CHANGED    [r] refresh    [b] reboot PX4    [p] back    [ESC] panels")

    if parameter_view == "CHANGED" and settings.param_defaults_file is None:
        lines.append("")
        lines.append(DIM + "* CHANGED = different from value first loaded by this console." + RESET)
    elif parameter_view == "CHANGED":
        lines.append("")
        lines.append(DIM + "CHANGED = different from configured PX4 factory/default values." + RESET)

    return lines
