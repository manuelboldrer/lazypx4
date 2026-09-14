"""The [p] parameter browse / search / edit screen."""

from __future__ import annotations

import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW
from ..config import PARAM_PAGE_SIZE, settings
from ..mavlink.parameters import (
    get_visible_parameters,
    parameter_format_value,
    parameter_type_name,
)
from ..search import footer_line
from ..state import session, state
from .chrome import highlight_matches


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

    if not visible:
        lines.append(RED + " No parameters in the current view." + RESET)
    else:
        page = selected_index // PARAM_PAGE_SIZE
        start = page * PARAM_PAGE_SIZE
        page_items = visible[start:start + PARAM_PAGE_SIZE]

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
            value_text = value_text[:18]

            lines.append(
                f" {GREEN if selected else ''}"
                f" {marker} "
                f"{name_text}"
                f" {value_text:>18}"
                f" {type_text:<8}"
                f"{status}"
                f"{RESET if selected else ''}"
            )

    footer = footer_line()
    if footer:
        lines.append(footer)

    lines.append("")

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
