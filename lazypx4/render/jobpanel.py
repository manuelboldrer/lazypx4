"""Shared "background job" status block.

The flash-firmware, .ulog-upload and ecl_ekf screens all drive the same
:mod:`lazypx4.jobs` runner and want to show the same thing - a status line,
an error or result if the job finished, and a tail of its output - so they
render through this one helper instead of three copies of the same
formatting.
"""

from __future__ import annotations

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW
from ..state import state

_STATUS_COLOR = {
    "RUNNING": YELLOW,
    "SUCCESS": GREEN,
    "ERROR": RED,
    "CANCELLED": YELLOW,
}


def job_panel_lines(kind, max_tail=8):
    """Lines describing the most recent job of ``kind``, or ``[]`` if no job
    of that kind has run yet this session."""
    with state.lock:
        if state.job_kind != kind:
            return []
        status = state.job_status
        error = state.job_error
        result = state.job_result
        tail = list(state.job_lines)[-max_tail:]

    color = _STATUS_COLOR.get(status, DIM)
    lines = ["", color + BOLD + f"Job: {status}" + RESET]

    if error:
        lines.append(RED + f"  {error}" + RESET)
    if result:
        lines.append(GREEN + f"  {result}" + RESET)

    for line in tail:
        lines.append(DIM + f"  {line}" + RESET)

    return lines
