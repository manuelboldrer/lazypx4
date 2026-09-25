"""MAVLink shell (NSH console) over SERIAL_CONTROL.

Raw-forwarding console: every keystroke is sent to PX4 immediately (like a
real serial terminal), and PX4's own NSH echoes it back along with command
output over the same SERIAL_CONTROL channel. We just render whatever comes
back - we never locally echo typed characters, since NSH does that for us.
"""

from __future__ import annotations

import time

from ..config import SHELL_DEVICE, SHELL_FLAG_EXCLUSIVE, SHELL_FLAG_RESPOND
from ..eventlog import log_error
from ..state import state
from ..util import safe_int


def send_shell_raw(master, data_bytes):
    if master is None:
        return False

    if not data_bytes:
        return True

    try:
        flags = SHELL_FLAG_RESPOND | SHELL_FLAG_EXCLUSIVE
        remaining = data_bytes

        while remaining:
            chunk = remaining[:70]
            remaining = remaining[70:]

            payload = list(chunk) + [0] * (70 - len(chunk))

            master.mav.serial_control_send(
                SHELL_DEVICE, flags, 0, 0, len(chunk), payload,
            )

        return True

    except Exception as exc:
        log_error(f"Shell write failed: {exc}")
        return False


def claim_shell(master):
    if master is None:
        return False

    try:
        flags = SHELL_FLAG_RESPOND | SHELL_FLAG_EXCLUSIVE
        master.mav.serial_control_send(SHELL_DEVICE, flags, 0, 0, 0, [0] * 70)
        return True

    except Exception as exc:
        log_error(f"Could not open MAVLink shell: {exc}")
        return False


def release_shell(master):
    if master is None:
        return False

    try:
        master.mav.serial_control_send(SHELL_DEVICE, 0, 0, 0, 0, [0] * 70)
        return True

    except Exception:
        return False


def _shell_append_text(text):
    # Caller must hold state.lock.
    partial = state.shell_partial

    for ch in text:
        if ch == "\r" or ch == "\x00":
            continue

        if ch == "\n":
            state.shell_lines.append(partial)
            partial = ""
            continue

        if ch in ("\x08", "\x7f"):
            partial = partial[:-1]
            continue

        partial += ch

    state.shell_partial = partial


def handle_serial_control(msg):
    device = safe_int(getattr(msg, "device", -1), -1)

    if device != SHELL_DEVICE:
        return

    count = safe_int(getattr(msg, "count", 0), 0)

    if count <= 0:
        return

    raw = getattr(msg, "data", [])

    try:
        chunk = bytes(safe_int(b) & 0xFF for b in list(raw)[:count])
    except Exception:
        return

    text = chunk.decode("utf-8", errors="replace")
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.shell_last_activity = now
        state.shell_active = True
        _shell_append_text(text)
