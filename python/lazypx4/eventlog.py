"""The in-memory event log shown on the ``[g]`` screen.

``add_log`` is called from every thread; it takes ``state.lock`` for the
append and the per-level counters.
"""

from __future__ import annotations

import re
import time

from .models import LogEvent
from .state import state

# Every event-log line is emitted as a single, fixed-width terminal row (see
# ansi.truncate_visible) - a raw control character surviving into it (a "\n"
# or "\r" from a PX4 STATUSTEXT/exception message, a stray "\t", ...) moves
# the real cursor instead of just occupying a column, which desyncs every
# row painted after it for that frame: later rows land shifted and a box's
# title can end up overwritten or pushed off screen. Collapse any run of
# such characters to a single space so a log message can never do that.
_CONTROL_CHARS_RE = re.compile(r"[\x00-\x1f\x7f]+")


def add_log(level, message):
    text = _CONTROL_CHARS_RE.sub(" ", str(message)).strip()
    event = LogEvent(timestamp=time.time(), level=level, message=text)
    with state.lock:
        state.events.append(event)
        if level == "WARN":
            state.warning_count += 1
        elif level == "ERROR":
            state.error_count += 1
        elif level == "FAILSAFE":
            state.failsafe_count += 1


def log_info(message):
    add_log("INFO", message)


def log_command(message):
    add_log("COMMAND", message)


def log_warn(message):
    add_log("WARN", message)


def log_error(message):
    add_log("ERROR", message)


def log_failsafe(message):
    add_log("FAILSAFE", message)


def classify_statustext(severity, text):
    """Map a PX4 STATUSTEXT to one of INFO / WARN / ERROR / FAILSAFE.

    Keyword matching takes precedence over the numeric MAV_SEVERITY so that,
    e.g., an "ARMING DENIED" notice at severity 6 is still surfaced as an
    error.
    """
    upper = text.upper()

    failsafe_words = [
        "FAILSAFE", "RC LOSS", "RC_LOSS", "OFFBOARD LOST", "OFFBOARD_LOST",
        "OFFBOARD LOSS", "DATA LINK LOST", "DATALINK LOST", "BATTERY FAILSAFE",
        "GPS FAILURE",
    ]

    error_words = [
        "PREFLIGHT FAIL", "PREFLIGHT FAILED", "ARMING DENIED", "ARM DENIED",
        "DISARM DENIED", "COMMAND DENIED", "REJECT", "FAILED", "ERROR", "CRITICAL",
    ]

    warning_words = [
        "WARNING", "WARN", "BATTERY", "LOW BATTERY", "EKF", "GPS", "RC", "ARM",
        "DISARM", "CALIBRAT",
    ]

    for word in failsafe_words:
        if word in upper:
            return "FAILSAFE"

    for word in error_words:
        if word in upper:
            return "ERROR"

    for word in warning_words:
        if word in upper:
            return "WARN"

    if severity <= 2:
        return "ERROR"
    if severity == 3:
        return "ERROR"
    if severity == 4:
        return "WARN"

    return "INFO"
