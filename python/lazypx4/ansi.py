"""ANSI escape codes, terminal-cursor helpers and the shared screen chrome.

The UI is a fixed-width, cursor-addressed redraw (like ``top``), not a line
printer. Every rendered line is clipped to the real terminal width by
:func:`truncate_visible` before it is emitted so a line can never wrap - a
wrapped line scrolls the alt-screen buffer and desynchronises the redraw.
"""

from __future__ import annotations

import os
import re
import sys

RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
INVERSE = "\033[7m"
INVERSE_OFF = "\033[27m"

# "Synchronized output" (DEC private mode 2026, aka the iTerm2/kitty/foot/
# Windows Terminal "synchronized update" extension - see
# https://gitlab.com/gnachman/iterm2/-/wikis/synchronized-updates-spec).
# Bracketing a frame in these tells a supporting terminal to buffer the whole
# update and paint it as one atomic swap instead of live, line-by-line - a
# terminal that doesn't know the mode just ignores it (it's a no-op private
# mode, not a visible sequence). Without it, a large frame (a full-screen
# window on a content-heavy screen like the dashboard or event log takes
# noticeably more bytes to redraw than a short one) can be caught mid-paint
# by the terminal's own refresh, which shows up as the top border/title
# visibly tearing on every redraw.
#
# Set LAZYPX4_NO_SYNC=1 to turn this off - a diagnostic escape hatch for a
# reported "top border sometimes doesn't paint at full screen" issue that
# hasn't been pinned down yet, in case a particular terminal mishandles the
# sequence instead of the tearing it's meant to fix.
_SYNC_DISABLED = os.environ.get("LAZYPX4_NO_SYNC") == "1"
SYNC_BEGIN = "" if _SYNC_DISABLED else "\033[?2026h"
SYNC_END = "" if _SYNC_DISABLED else "\033[?2026l"

BLACK = "\033[30m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"

BG_RED = "\033[41m"
BG_GREEN = "\033[42m"
BG_YELLOW = "\033[43m"
BG_BLUE = "\033[44m"


def hide_cursor():
    sys.stdout.write("\033[?25l")


def show_cursor():
    sys.stdout.write("\033[?25h")


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")


def visible_length(text):
    """Columns ``text`` occupies on screen, ignoring ANSI colour codes."""
    return len(ANSI_ESCAPE_RE.sub("", text))


def truncate_visible(text, width):
    """Truncate ``text`` to at most ``width`` *visible* columns.

    ANSI colour/style escapes are passed through untouched (they occupy no
    columns). This exists so a line can never cause the terminal to wrap.
    """
    if width <= 0:
        return ""

    out = []
    visible_count = 0
    i = 0
    n = len(text)

    while i < n:
        match = ANSI_ESCAPE_RE.match(text, i)

        if match:
            out.append(match.group(0))
            i = match.end()
            continue

        if visible_count >= width:
            i += 1
            continue

        out.append(text[i])
        visible_count += 1
        i += 1

    return "".join(out)


# ---------------------------------------------------------------------------
# Shared "modern minimal" chrome: a rounded title bracket and thin, dimmed
# section rules. Everything is emitted at a fixed nominal width and left for
# draw_lines() -> truncate_visible() to clip to the real terminal, so it
# degrades cleanly on any resize and never wraps.
# ---------------------------------------------------------------------------

UI_RULE_WIDTH = 82
UI_LIGHT_RULE = "─"


def ui_header(title):
    """Two lines: a rounded-corner title, then a thin rule beneath it."""
    return [
        BOLD + CYAN + "╭─ " + title + RESET,
        DIM + "╰" + UI_LIGHT_RULE * (UI_RULE_WIDTH - 1) + RESET,
    ]


def ui_section(label, note=""):
    """A section label followed by a thin trailing rule.

    Information stays left of the rule so truncation only ever eats the rule.
    """
    text = " " + BOLD + label + RESET
    if note:
        text += "  " + DIM + note + RESET
    text += " "

    fill = UI_RULE_WIDTH - visible_length(text)
    if fill > 2:
        text += DIM + UI_LIGHT_RULE * fill + RESET

    return text
