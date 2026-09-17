"""Frame painting and the small colour/label helpers screens share."""

from __future__ import annotations

import math
import shutil
import sys
import time

from ..ansi import (
    BG_BLUE,
    BG_GREEN,
    BG_RED,
    BLACK,
    BOLD,
    CYAN,
    DIM,
    GREEN,
    INVERSE,
    INVERSE_OFF,
    RED,
    RESET,
    SYNC_BEGIN,
    SYNC_END,
    WHITE,
    YELLOW,
    truncate_visible,
    visible_length,
)
from ..config import (
    DISTANCE_SENSOR_ORIENTATION_DOWN,
    DISTANCE_SENSOR_ORIENTATION_NAMES,
    EKF_STATUS_REPORT_FLAG_LABELS,
    ESTIMATOR_STATUS_FLAG_LABELS,
    GPS_FIX_NAMES,
    MIN_TERMINAL_COLS,
    MIN_TERMINAL_ROWS,
    SYS_STATUS_SENSOR_LABELS,
    VIBRATION_GOOD,
    VIBRATION_OK,
)
from ..panels import NAV_ITEMS
from ..state import session
from ..util import clamp, safe_int

#: WGS-84 equatorial radius, for the small-angle lat/lon -> local NED projection.
_EARTH_RADIUS_M = 6378137.0


def global_to_local(lat, lon, origin_lat, origin_lon):
    """Project a lat/lon onto the local NED tangent plane at the origin.

    Equirectangular approximation - accurate to well under a metre over the
    ranges this UI shows, and it puts the GNSS fix in the exact same frame as
    ``LOCAL_POSITION_NED`` so the two are directly comparable.
    """
    north = math.radians(lat - origin_lat) * _EARTH_RADIUS_M
    east = (
        math.radians(lon - origin_lon)
        * _EARTH_RADIUS_M
        * math.cos(math.radians(origin_lat))
    )
    return north, east


def format_clock(epoch_seconds):
    """UTC ``HH:MM:SS.mmmZ`` for a Unix-epoch timestamp in seconds."""
    struct = time.gmtime(epoch_seconds)
    millis = int((epoch_seconds - math.floor(epoch_seconds)) * 1000) % 1000
    return f"{time.strftime('%H:%M:%S', struct)}.{millis:03d}Z"


def level_color(level):
    if level == "FAILSAFE":
        return BG_RED + WHITE
    if level == "ERROR":
        return RED
    if level == "WARN":
        return YELLOW
    if level == "COMMAND":
        return CYAN
    return WHITE


def state_text(armed):
    if armed:
        return BG_GREEN + BLACK + BOLD + " ARMED " + RESET
    return DIM + " DISARMED " + RESET


def gps_fix_display(fix_type):
    fix_type = safe_int(fix_type)
    name = GPS_FIX_NAMES.get(fix_type, f"FIX {fix_type}")

    if fix_type >= 5:
        return GREEN + BOLD, name
    if fix_type >= 3:
        return GREEN, name
    if fix_type == 2:
        return YELLOW, name
    return RED, name


def rtk_text(fix_type):
    fix_type = safe_int(fix_type)

    if fix_type == 6:
        return GREEN + BOLD + "FIXED" + RESET
    if fix_type == 5:
        return YELLOW + "FLOAT" + RESET
    return DIM + "no" + RESET


def distance_orientation_text(orientation):
    orientation = safe_int(orientation, -1)

    if orientation < 0:
        return "?"

    return DISTANCE_SENSOR_ORIENTATION_NAMES.get(orientation, f"rot {orientation}")


def estimator_flag_labels(flags, is_estimator_status):
    labels = (
        ESTIMATOR_STATUS_FLAG_LABELS if is_estimator_status
        else EKF_STATUS_REPORT_FLAG_LABELS
    )
    return [name for bit, name in labels if flags & bit]


def ekf_summary(flags, is_estimator_status, ekf_status):
    """Condense the estimator health flags into one short coloured verdict."""
    if not flags:
        if ekf_status == "OK":
            return "OK", GREEN
        return ekf_status, DIM

    has_att = bool(flags & 1)
    has_vel_h = bool(flags & 2)
    has_pos_h = bool(flags & (8 | 16))

    if is_estimator_status:
        gps_glitch = bool(flags & 1024)
        accel_error = bool(flags & 2048)
    else:
        gps_glitch = False
        accel_error = False
        if flags & 1024:  # UNINITIALIZED
            return "INITIALIZING", YELLOW

    if not has_att:
        return "NO ATTITUDE", RED
    if accel_error:
        return "ACCEL ERROR", RED
    if not has_vel_h or not has_pos_h:
        return "DEAD RECKONING", YELLOW
    if gps_glitch:
        return "GPS GLITCH", YELLOW

    return "OK", GREEN


def graded_color(value, good, ok, higher_is_better=True):
    """Green/yellow/red for ``value`` against a (good, ok) threshold pair.

    ``higher_is_better`` picks the direction: GPS satellite count or RC
    signal quality read better the higher they go, while HDOP or position
    accuracy read better the lower they go.
    """
    if higher_is_better:
        if value >= good:
            return GREEN
        if value >= ok:
            return YELLOW
        return RED

    if value <= good:
        return GREEN
    if value <= ok:
        return YELLOW
    return RED


#: bit 64 in both ESTIMATOR_STATUS_FLAG_LABELS and EKF_STATUS_REPORT_FLAG_LABELS -
#: "estimating height above ground", i.e. a rangefinder/flow-derived HAGL
#: estimate is currently valid and being fused.
_EKF_FLAG_POS_V_AGL = 64


def rangefinder_status(rng_ctrl, ekf_flags, orientation, age_seconds):
    """(text, colour) verdict for whether the rangefinder is feeding the EKF.

    Distinguishes "deliberately off" (EKF2_RNG_CTRL=0, dimmed - not a fault)
    from "should be fusing but isn't" (misconfigured mount, stale/missing
    data, or PX4 just hasn't accepted it into the height estimate yet).
    """
    if rng_ctrl is None:
        return "EKF2_RNG_CTRL not loaded", DIM
    rng_ctrl = int(round(rng_ctrl))

    if rng_ctrl == 0:
        return "disabled (EKF2_RNG_CTRL=0)", DIM

    if age_seconds is None or age_seconds > 3.0:
        return "enabled, no sensor data", RED
    if orientation != DISTANCE_SENSOR_ORIENTATION_DOWN:
        return "enabled, sensor not facing down", YELLOW
    if age_seconds > 1.0:
        return "enabled, data stale", YELLOW
    if ekf_flags & _EKF_FLAG_POS_V_AGL:
        return "fusing (HAGL valid)", GREEN
    return "enabled, not fusing yet", YELLOW


def sensor_health_items(present, enabled, health):
    """[(label, colour)] for each arming-relevant sensor SYS_STATUS reports
    as present on this airframe - skips bits the vehicle doesn't have at all,
    rather than padding the list with sensors that will never apply."""
    items = []
    for bit, label in SYS_STATUS_SENSOR_LABELS:
        if not (present & bit):
            continue
        if not (enabled & bit):
            items.append((label, DIM))
        elif health & bit:
            items.append((label, GREEN))
        else:
            items.append((label, RED))
    return items


def vibration_verdict(vx, vy, vz, clip0, clip1, clip2):
    """Condense IMU vibration levels + clipping counts into one verdict."""
    if clip0 or clip1 or clip2:
        return "CLIPPING", RED

    peak = max(vx, vy, vz)
    if peak >= VIBRATION_OK:
        return "HIGH", RED
    if peak >= VIBRATION_GOOD:
        return "ELEVATED", YELLOW
    return "OK", GREEN


def test_ratio_cell(label, value):
    if value is None or value <= 0:
        return f"{label}:{DIM}--{RESET}"

    if value < 0.5:
        color = GREEN
    elif value < 1.0:
        color = YELLOW
    else:
        color = RED

    return f"{label}:{color}{value:.2f}{RESET}"


def pos_target_active_axes(type_mask):
    # POSITION_TARGET_TYPEMASK bits mark axes to *ignore*; invert that.
    groups = (
        ("pos", (1 | 2 | 4)),
        ("vel", (8 | 16 | 32)),
        ("acc", (64 | 128 | 256)),
        ("yaw", 1024),
        ("yawrate", 2048),
    )
    active = [name for name, bits in groups if (type_mask & bits) != bits]

    if type_mask & 512:
        active.append("force")

    return active


# 8-way heading arrows: N NE E SE S SW W NW. The rest of the UI already relies
# on UTF-8 box-drawing characters, so these are safe too.
HEADING_ARROWS = "↑↗→↘↓↙←↖"


def heading_arrow(yaw_deg):
    if not math.isfinite(yaw_deg):
        return "?"
    index = int((yaw_deg % 360.0) / 45.0 + 0.5) % 8
    return HEADING_ARROWS[index]


def highlight_matches(text, query):
    """Wrap each case-insensitive occurrence of ``query`` in inverse video.

    Inverse video (rather than a colour) is used so it composes with whatever
    colour the row already carries. ``truncate_visible`` treats the escapes as
    zero-width.
    """
    if not query:
        return text

    low = text.lower()
    needle = query.lower()
    out = []
    i = 0

    while True:
        j = low.find(needle, i)
        if j < 0:
            out.append(text[i:])
            return "".join(out)
        out.append(text[i:j])
        out.append(INVERSE + text[j:j + len(needle)] + INVERSE_OFF)
        i = j + len(needle)


# ---------------------------------------------------------------------------
# Frame painting
# ---------------------------------------------------------------------------

# One frame used to mean up to three independent shutil.get_terminal_size()
# calls (the screen's own draw_*() for width-dependent formatting,
# _layout_metrics() for the sidebar/box geometry, draw_lines() for the final
# per-line clip) - each a live syscall. A resize landing between any two of
# them let one frame's geometry, content and clipping each assume a
# different terminal size, which is exactly what a user reported as the
# title/border "getting bugged" while actively resizing. snapshot_frame_size()
# freezes one reading that every one of those call sites reuses for the rest
# of that frame; release_frame_size() lets the next frame take a fresh one.
_frame_size = None


def snapshot_frame_size():
    global _frame_size
    _frame_size = shutil.get_terminal_size((110, 45))


def release_frame_size():
    global _frame_size
    _frame_size = None


def terminal_size():
    """The current frame's frozen terminal size, or a fresh live read if
    called outside a snapshot_frame_size()/release_frame_size() bracket (unit
    tests, a one-off script)."""
    return _frame_size if _frame_size is not None else shutil.get_terminal_size((110, 45))


_PROMPT_BAR = BG_BLUE + WHITE + BOLD


def _prompt_lines():
    """The confirm / input prompt rendered as normal frame lines.

    Drawn as part of the single top-down frame render (not a cursor-addressed
    overlay), so it can never leave stale characters behind.
    """
    if session.confirm_active:
        return [
            "",
            _PROMPT_BAR + " CONFIRM " + RESET + "  " + BOLD + session.confirm_text + RESET,
            _PROMPT_BAR + " YES ? " + RESET + "  " + BOLD + session.confirm_buffer + RESET
            + "█   " + DIM + "ENTER confirm · ESC cancel" + RESET,
        ]

    if session.input_active:
        return [
            "",
            _PROMPT_BAR + " INPUT " + RESET + "  " + session.input_prompt,
            _PROMPT_BAR + "   >   " + RESET + "  " + session.input_buffer + "█   "
            + DIM + "ENTER submit · ESC cancel" + RESET,
        ]

    return []


def draw_too_small(width, height):
    # Build the whole screen as one string and issue a single write(). stdout
    # is line-buffered when it's a tty, so a write() containing a trailing
    # "\n" auto-flushes as soon as it returns - a loop of several such
    # writes becomes several separate small writes to the pty regardless of
    # the SYNC_BEGIN/SYNC_END bracket around them. A terminal emulator's own
    # frame compositor usually hides the resulting tearing, but a
    # multiplexer like tmux redraws its client on its own event-loop tick
    # and has no such compositor, so the same drip-fed writes show up as a
    # visibly glitchy repaint there. One write() is one flush.
    message = f"Terminal too small ({width}x{height})."
    hint = f"Resize to at least {MIN_TERMINAL_COLS}x{MIN_TERMINAL_ROWS}."
    tip = "Tip: shrink your terminal's font (Ctrl -) to fit more columns/rows."

    out = [SYNC_BEGIN, "\033[H"]
    out.append(truncate_visible(message, max(0, width)) + "\033[K\n")
    out.append(truncate_visible(hint, max(0, width)) + "\033[K\n")
    out.append(truncate_visible(tip, max(0, width)) + "\033[K")
    out.append("\033[J")
    out.append(SYNC_END)

    sys.stdout.write("".join(out))
    sys.stdout.flush()


#: Terminal size as of the last rendered frame, so a resize can be told apart
#: from a shrink (see draw_lines()).
_last_frame_size = (0, 0)


def draw_lines(lines):
    global _last_frame_size

    size = terminal_size()
    width = max(1, size.columns)
    height = max(1, size.lines)
    last_width, last_height = _last_frame_size
    _last_frame_size = (width, height)

    if width < MIN_TERMINAL_COLS or height < MIN_TERMINAL_ROWS:
        session.resize_pending = False
        draw_too_small(width, height)
        return

    # Build the whole frame as one string and issue a single write().
    # sys.stdout is line-buffered when it's a tty, so any write() containing
    # a trailing "\n" auto-flushes as soon as it returns - the old
    # per-line loop below was therefore many separate small writes to the
    # pty, no matter that they were all wrapped in one SYNC_BEGIN/SYNC_END
    # bracket. A terminal emulator's own frame compositor usually hides the
    # resulting tearing, but a multiplexer like tmux redraws its client on
    # its own event-loop tick and has no such compositor (and tmux versions
    # before 3.7 don't even understand the SYNC_BEGIN/SYNC_END escape
    # itself), so the same drip-fed writes show up as a visibly glitchy
    # repaint there. One write() call is one flush and one paint.
    out = [SYNC_BEGIN]
    try:
        if session.resize_pending:
            session.resize_pending = False
            if width < last_width or height < last_height:
                # Full wipe: a shrink can leave characters from the previous,
                # larger frame past the edges of the new one that a partial
                # "clear below cursor" wouldn't reach. Only shrinking needs
                # this - doing it unconditionally on every resize (including
                # growing the terminal) paints a visible blank flash before
                # the new frame lands, which reads as the border/title
                # flickering.
                out.append("\033[2J\033[H")
            else:
                out.append("\033[H")
        else:
            out.append("\033[H")

        # Never emit more rows than the terminal has, and never let a single
        # line exceed the terminal width - either would wrap and scroll the
        # alt-screen buffer, which is what produces the visible glitches.
        max_lines = max(0, height - 1)

        prompt = _prompt_lines()
        if prompt:
            # Reserve the bottom rows for the prompt so it is always visible,
            # even when the screen content would otherwise fill the terminal.
            body = list(lines[:max(0, max_lines - len(prompt))])
            while len(body) < max_lines - len(prompt):
                body.append("")
            frame = body + prompt
        else:
            frame = lines[:max_lines]

        for line in frame[:max_lines]:
            out.append(truncate_visible(line, width) + "\033[K\n")

        # Clear only the area below the current frame. Never blank the whole UI.
        out.append("\033[J")
    finally:
        # Always close the synchronized-update bracket, even if something
        # above raised - otherwise a terminal that honours SYNC_BEGIN is left
        # buffering forever and nothing new ever paints again.
        out.append(SYNC_END)
        sys.stdout.write("".join(out))
        sys.stdout.flush()


# ---------------------------------------------------------------------------
# Panel layout (lazygit / lazydocker style)
#
# Every screen used to build its own top-to-bottom list of lines (starting
# with a ui_header() banner) and hand it straight to draw_lines(). Now each
# screen just returns that list of *content* lines (no banner - the title
# moves into the panel border) and draw_frame() below wraps it: a bordered
# main panel on the right, a bordered sidebar of every other screen on the
# left (so the rest of the console is never more than one ENTER away), and a
# bottom bar of the keys that apply wherever focus currently is. On a
# terminal too narrow for two columns it falls back to a single full-width
# bordered panel with no sidebar.
# ---------------------------------------------------------------------------

#: Below this, a sidebar + main-panel split would leave the main panel too
#: narrow to be worth it (most content lines assume UI_RULE_WIDTH, ~82 cols).
#: The show/hide values differ (hysteresis) so a terminal whose reported
#: width hovers right at the boundary - some tmux panes and SSH clients do
#: this - doesn't flip the whole top border between the two-box and
#: one-box shape every other frame.
_SIDEBAR_SHOW_WIDTH = 92
_SIDEBAR_HIDE_WIDTH = 84

_sidebar_shown = True


def _use_sidebar(term_width):
    global _sidebar_shown

    if _sidebar_shown and term_width < _SIDEBAR_HIDE_WIDTH:
        _sidebar_shown = False
    elif not _sidebar_shown and term_width >= _SIDEBAR_SHOW_WIDTH:
        _sidebar_shown = True

    return _sidebar_shown


def box(content_lines, width, height, title="", focused=False):
    """Wrap ``content_lines`` in a rounded border of exactly ``width`` x
    ``height`` visible cells, clipping/padding content to fit.

    ``title`` is painted into the top border rather than as a content line,
    and the whole border is coloured to show whether this panel has focus -
    the same two cues lazygit/lazydocker use to show which pane is active.
    """
    width = max(4, width)
    height = max(3, height)
    inner_w = width - 2

    accent = BOLD + CYAN if focused else DIM

    if title:
        label = f" {title} "[:max(0, inner_w - 2)]
        fill = max(0, inner_w - 2 - visible_length(label))
        top = (
            accent + "╭─" + RESET + BOLD + label + RESET
            + accent + ("─" * fill) + "╮" + RESET
        )
    else:
        top = accent + "╭" + ("─" * inner_w) + "╮" + RESET

    bottom = accent + "╰" + ("─" * inner_w) + "╯" + RESET

    body = []
    for i in range(height - 2):
        raw = content_lines[i] if i < len(content_lines) else ""
        clipped = truncate_visible(raw, inner_w)
        pad = max(0, inner_w - visible_length(clipped))
        body.append(accent + "│" + RESET + clipped + (" " * pad) + accent + "│" + RESET)

    return [top] + body + [bottom]


def _hjoin(left, right, gap=1):
    """Concatenate two equal-purpose panels (each already a fixed-width,
    fixed-height list of lines from :func:`box`) side by side."""
    n = max(len(left), len(right))
    sep = " " * gap
    return [
        (left[i] if i < len(left) else "") + sep + (right[i] if i < len(right) else "")
        for i in range(n)
    ]


def _nav_panel_lines(height):
    """The sidebar's rows: every screen, the active one marked, the
    keyboard-cursor row (while the sidebar has focus) in inverse video."""
    lines = []

    for i, (screen_key, label, hotkey) in enumerate(NAV_ITEMS):
        active = session.screen == screen_key
        cursor = session.nav_focus == "sidebar" and session.nav_index == i
        hint = f" [{hotkey}]" if hotkey else ""
        marker = "› " if active else "  "
        text = f"{marker}{label}{hint}"

        if cursor:
            text = INVERSE + text + INVERSE_OFF
        elif active:
            text = BOLD + CYAN + text + RESET
        else:
            text = DIM + text + RESET

        lines.append(text)

    return lines[:max(0, height)]


def _keybind_bar(width, scroll_note=""):
    if session.nav_focus == "sidebar":
        hint = "↑↓ / j k preview   ENTER open   TAB / ESC back to panel   q quit"
    else:
        hint = "TAB / ESC panels   q quit   / search"

    return DIM + " " + hint + scroll_note + RESET


def _layout_metrics():
    """Terminal size plus every derived dimension of the current frame.

    The single source of truth for how the frame is carved up, shared by
    :func:`draw_frame` (which paints it) and :func:`content_area` (which lets
    a screen that sizes its own output - the map's ASCII grid, the shell's
    scrollback window - ask how much room it will actually get instead of
    guessing from the raw terminal size and having box() silently clip the
    overrun).
    """
    size = terminal_size()
    term_width = max(1, size.columns)
    term_height = max(1, size.lines)

    too_small = term_width < MIN_TERMINAL_COLS or term_height < MIN_TERMINAL_ROWS

    max_lines = max(0, term_height - 1)
    prompt_rows = len(_prompt_lines())
    # The keybind bar and an active confirm/input prompt both live in the
    # bottom row(s) - never both at once, the prompt already states its own
    # keys (ENTER confirm / ESC cancel).
    footer_rows = 0 if prompt_rows else 1
    body_height = max(3, max_lines - prompt_rows - footer_rows)

    use_sidebar = _use_sidebar(term_width)
    if use_sidebar:
        sidebar_width = min(28, max(20, term_width // 5))
        main_width = term_width - sidebar_width - 1
    else:
        sidebar_width = 0
        main_width = term_width

    return {
        "term_width": term_width,
        "term_height": term_height,
        "too_small": too_small,
        "body_height": body_height,
        "footer_rows": footer_rows,
        "use_sidebar": use_sidebar,
        "sidebar_width": sidebar_width,
        "main_width": main_width,
    }


def content_area():
    """``(width, height)`` available to the active screen's content this
    frame - the main panel's interior once the sidebar, its border and the
    keybind bar are accounted for. Most screens don't need this (box() clips
    for them), but one that pre-sizes its own output - the map's ASCII grid,
    the shell's scrollback window - must use this instead of
    ``shutil.get_terminal_size()`` or it will size for the whole terminal and
    have the overrun silently clipped."""
    m = _layout_metrics()

    if m["too_small"]:
        return max(0, m["term_width"] - 2), max(0, m["term_height"] - 3)

    return max(0, m["main_width"] - 2), max(0, m["body_height"] - 2)


def dynamic_list_page_size(overhead_lines, minimum=1):
    """Rows of a variable-length list that fit in the main panel this frame,
    reserving ``overhead_lines`` below them for counters / key hints / an
    optional search footer.

    Most paginated screens (parameters, flight logs, mode select) use a
    fixed ``*_PAGE_SIZE`` constant and rely on :func:`draw_frame`'s own
    line-level scroll (``session.main_scroll``, moved with jk) to reveal
    whatever a too-short terminal clips off the bottom. The event log is
    different: its own [g] UP/DOWN move ``session.log_scroll`` - which
    *event* starts the page - not ``main_scroll``, so on a terminal short
    enough that a fixed-size page overflows the real panel height, chrome
    clipped the excess (including the "Showing: X-Y" counter and the
    key-hint line) with no key able to reach it. Sizing the page to the
    real, current panel height instead removes that mismatch entirely.
    """
    _, height = content_area()
    return max(minimum, height - overhead_lines)


#: Lines the event-log screen always reserves below its event rows: the
#: blank separator, the "Events: N Showing: X-Y" counter and the key-hint
#: line. When a search footer is also showing this slightly under-fills the
#: panel by one row rather than risk the over-fill `log_page_size()` exists
#: to avoid.
_LOG_OVERHEAD_LINES = 4


def log_page_size():
    """Event rows that fit in the main panel at the current terminal size -
    see :func:`dynamic_list_page_size`. Lives here (rather than next to
    :mod:`lazypx4.render.eventlog_screen`'s own draw function) so
    :mod:`lazypx4.navigation` and :mod:`lazypx4.search` can both import it
    without either creating an import cycle through
    ``eventlog_screen -> search -> eventlog_screen``. All three call sites
    must agree on this number, or [g]'s own scrolling, chrome's box-height
    clipping and "/" jump-to-match disagree about how tall a page is - which
    is exactly the mismatch that made the event log clip/misalign its bottom
    rows on a short terminal before this existed.
    """
    return dynamic_list_page_size(_LOG_OVERHEAD_LINES)


#: (sub_col, sub_row) -> Braille dot bit within a character's 2-wide x
#: 4-tall dot sub-grid (the glyph to draw is ``braille_glyph()`` of the OR of
#: every bit set in it). Shared by the map's position trail
#: (:mod:`lazypx4.render.mapview`) and the point-cloud scatter view
#: (:mod:`lazypx4.render.pointcloud`), both of which pack many data points
#: into one character cell instead of clipping to one point per cell - a
#: plain ASCII grid at typical terminal size only has a couple thousand
#: cells, nowhere near enough to show a dense trail or scan as a
#: recognisable shape. Braille gives the same footprint 8x the resolution.
BRAILLE_BITS = {
    (0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (0, 3): 0x40,
    (1, 0): 0x08, (1, 1): 0x10, (1, 2): 0x20, (1, 3): 0x80,
}


def braille_glyph(bitmask):
    return chr(0x2800 + bitmask)


def draw_frame(content_lines, title, screen_key=None):
    """Compose the sidebar + the active screen's ``content_lines`` into one
    frame and paint it - the single call site every screen's draw is routed
    through (see :func:`lazypx4.render.draw`).

    A screen with no navigable list of its own (dashboard, estimation,
    control, calibration) can return more lines than the panel is tall;
    rather than silently clip the bottom off, this scrolls ``content_lines``
    by ``session.main_scroll`` (moved with jk/UP/DOWN - see
    navigation._scroll_main) and marks the visible range in the panel title
    so it's obvious there's more below.
    """
    m = _layout_metrics()

    if m["too_small"]:
        session.resize_pending = False
        draw_too_small(m["term_width"], m["term_height"])
        return

    body_height = m["body_height"]
    interior_height = max(1, body_height - 2)

    if screen_key is not None and screen_key != session.main_scroll_screen:
        session.main_scroll = 0
        session.main_scroll_screen = screen_key

    max_scroll = max(0, len(content_lines) - interior_height)
    session.main_scroll = clamp(session.main_scroll, 0, max_scroll)
    visible_content = content_lines[session.main_scroll:session.main_scroll + interior_height]

    # The scroll-range note goes in the footer, not the panel title: folding
    # it into the border title made the border's dash-fill visibly reflow
    # every time content crossed the "fits without scrolling" threshold
    # (which happens mid-drag on any live terminal resize), reading as the
    # title flickering/jumping. The footer is a plain text row outside the
    # box's border geometry, so its length changing doesn't touch the border.
    panel_title = title
    scroll_note = ""
    if max_scroll > 0:
        first = session.main_scroll + 1
        last = min(len(content_lines), session.main_scroll + interior_height)
        scroll_note = f"   {first}-{last}/{len(content_lines)}  jk to scroll"

    if m["use_sidebar"]:
        sidebar_box = box(
            _nav_panel_lines(body_height - 2), m["sidebar_width"], body_height,
            title="PANELS", focused=session.nav_focus == "sidebar",
        )
        main_box = box(
            visible_content, m["main_width"], body_height,
            title=panel_title, focused=session.nav_focus == "main",
        )
        body = _hjoin(sidebar_box, main_box)
    else:
        # Too narrow to show the sidebar at all - focus can only mean main.
        session.nav_focus = "main"
        body = box(visible_content, m["term_width"], body_height, title=panel_title, focused=True)

    frame = body if m["footer_rows"] == 0 else body + [_keybind_bar(m["term_width"], scroll_note)]
    draw_lines(frame)
