"""Raw-mode terminal setup and the keyboard reader thread.

The keyboard thread only decodes bytes into key names and pushes them onto
:data:`lazypx4.state.key_queue`; the main loop drains that queue and acts on
it (see :mod:`lazypx4.navigation`).
"""

from __future__ import annotations

import os
import select
import signal
import sys
import termios
import time
import tty

from .ansi import RESET, hide_cursor, show_cursor
from .eventlog import log_error
from .state import key_queue, session, shutdown_event

_terminal_fd = None
_terminal_old_settings = None

_keyboard_buffer = bytearray()
_keyboard_buffer_time = 0.0


def handle_sigwinch(signum, frame):
    session.resize_pending = True


def terminal_start():
    global _terminal_fd, _terminal_old_settings

    if not sys.stdin.isatty():
        raise RuntimeError("stdin is not a terminal")

    _terminal_fd = sys.stdin.fileno()
    _terminal_old_settings = termios.tcgetattr(_terminal_fd)

    tty.setcbreak(_terminal_fd)

    try:
        signal.signal(signal.SIGWINCH, handle_sigwinch)
    except (AttributeError, ValueError):
        # SIGWINCH doesn't exist on some platforms, or we're not in the main
        # thread. Resize still works, just without a forced full clear on the
        # frame right after a resize.
        pass

    sys.stdout.write("\033[?1049h\033[2J\033[H")
    hide_cursor()
    sys.stdout.flush()


def terminal_restore():
    try:
        if _terminal_fd is not None and _terminal_old_settings is not None:
            termios.tcsetattr(_terminal_fd, termios.TCSADRAIN, _terminal_old_settings)
    except Exception:
        pass

    show_cursor()
    sys.stdout.write(RESET + "\033[?1049l")
    sys.stdout.flush()


def keyboard_thread():
    fd = _terminal_fd

    while not shutdown_event.is_set():
        try:
            ready, _, _ = select.select([fd], [], [], 0.05)
            if ready:
                data = os.read(fd, 64)
                if data:
                    _parse_keyboard(data)

            if _keyboard_buffer and _keyboard_buffer[0] == 27:
                if time.monotonic() - _keyboard_buffer_time > 0.08:
                    key_queue.put("ESC")
                    del _keyboard_buffer[:1]

        except Exception as exc:
            if not shutdown_event.is_set():
                log_error(f"Keyboard error: {exc}")
            break


_CSI_SEQUENCES = {
    b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT",
    b"H": "HOME", b"F": "END", b"5~": "PGUP", b"6~": "PGDN",
}

_CONTROL_KEYS = {
    3: "CTRL_C", 4: "CTRL_D", 6: "CTRL_F",
    21: "CTRL_U", 2: "CTRL_B", 9: "TAB",
}


def _parse_keyboard(data):
    global _keyboard_buffer_time

    _keyboard_buffer.extend(data)

    while _keyboard_buffer:
        b = _keyboard_buffer[0]

        if b == 27:
            _keyboard_buffer_time = time.monotonic()

            if len(_keyboard_buffer) == 1:
                return

            if _keyboard_buffer[1] != ord("["):
                key_queue.put("ESC")
                del _keyboard_buffer[:1]
                continue

            sequence = bytes(_keyboard_buffer[2:])
            matched = False
            incomplete = False

            for encoded, key in _CSI_SEQUENCES.items():
                if sequence.startswith(encoded):
                    key_queue.put(key)
                    del _keyboard_buffer[:2 + len(encoded)]
                    matched = True
                    break
                if encoded.startswith(sequence):
                    incomplete = True

            if matched:
                continue
            if incomplete:
                return

            key_queue.put("ESC")
            del _keyboard_buffer[:1]
            continue

        del _keyboard_buffer[:1]

        if b in (10, 13):
            key_queue.put("ENTER")
        elif b in (8, 127):
            key_queue.put("BACKSPACE")
        elif b in _CONTROL_KEYS:
            key_queue.put(_CONTROL_KEYS[b])
        elif 32 <= b <= 126:
            key_queue.put(chr(b))
