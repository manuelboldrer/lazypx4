"""The in-memory event log shown on the ``[g]`` screen.

``add_log`` is called from every thread; it takes ``state.lock`` for the
append and the per-level counters.
"""

from __future__ import annotations

import time

from .models import LogEvent
from .state import state


def add_log(level, message):
    event = LogEvent(timestamp=time.time(), level=level, message=str(message))
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
