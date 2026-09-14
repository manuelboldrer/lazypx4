"""Keyboard controller: turns key names from the queue into actions.

This is the glue between :mod:`lazypx4.terminal` (which produces key names)
and the MAVLink command senders / render screens. It owns screen switching,
per-screen key handling, and the "type YES" confirmation flow.
"""

from __future__ import annotations

import math
import os
import queue
import time

from . import search
from .config import (
    FLIGHT_LOG_PAGE_SIZE,
    JOG_MIN_INTERVAL,
    JOG_STEP_M,
    JOG_STEP_MAX_M,
    JOG_STEP_MIN_M,
    JOG_YAW_STEP_DEG,
    MODE_PAGE_SIZE,
    PARAM_PAGE_SIZE,
    settings,
)
from .eventlog import log_command, log_error, log_info, log_warn
from .jobs import cancel_job, job_running
from .mavlink.calibration import cancel_calibration, send_calibration
from .mavlink.commands import send_arm, send_hold, send_reboot
from .mavlink.connection import configure_streams, request_estimator_params, vehicle_ready
from .mavlink.guided import send_goto_body, send_land, send_rtl, send_takeoff
from .mavlink.flightlog import (
    cancel_flight_log_download,
    request_flight_logs,
    start_flight_log_download,
)
from .mavlink.modes import confirm_mode_option, get_mode_options, request_available_modes
from .mavlink.parameters import (
    confirm_parameter_set,
    get_visible_parameters,
    parameter_format_value,
    parse_parameter_value,
    request_parameters,
)
from .mavlink.shell import claim_shell, send_shell_raw
from .netmon import run_speedtest_async
from .panels import NAV_ITEMS
from .render.chrome import log_page_size
from .pxtools import (
    discover_firmware_files,
    discover_serial_ports,
    start_ecl_ekf_check,
    start_firmware_flash,
    start_ulog_upload,
)
from .satellite import start_satellite_download
from .state import key_queue, session, shutdown_event, state
from .util import clamp


# ---------------------------------------------------------------------------
# Confirmation prompt ("type YES")
# ---------------------------------------------------------------------------


def request_confirmation(text, callback):
    session.confirm_active = True
    session.confirm_text = text
    session.confirm_buffer = ""
    session.confirm_callback = callback


def cancel_confirmation():
    session.confirm_active = False
    session.confirm_text = ""
    session.confirm_buffer = ""
    session.confirm_callback = None


def process_confirmation(key):
    if key == "ESC":
        cancel_confirmation()
        return

    if key == "BACKSPACE":
        session.confirm_buffer = session.confirm_buffer[:-1]
        return

    if key == "ENTER":
        if session.confirm_buffer.upper() == "YES":
            callback = session.confirm_callback
            cancel_confirmation()

            if callback is not None:
                callback()
        else:
            session.confirm_buffer = ""
            log_warn("Type YES to confirm")
        return

    if len(key) == 1 and key.isprintable():
        session.confirm_buffer += key.upper()

        if len(session.confirm_buffer) > 3:
            session.confirm_buffer = session.confirm_buffer[-3:]


def confirm_arm():
    send_arm(session.link, True)


def confirm_disarm():
    send_arm(session.link, False)


def confirm_hold():
    send_hold(session.link)


def confirm_reboot():
    send_reboot(session.link)


# ---------------------------------------------------------------------------
# Generic single-line text input (used by "goto")
# ---------------------------------------------------------------------------


def request_input(prompt, callback):
    session.input_active = True
    session.input_prompt = prompt
    session.input_buffer = ""
    session.input_callback = callback


def _clear_input():
    session.input_active = False
    session.input_prompt = ""
    session.input_buffer = ""
    session.input_callback = None


def process_input(key):
    if key == "ESC":
        _clear_input()
        return

    if key == "ENTER":
        callback = session.input_callback
        text = session.input_buffer
        _clear_input()
        if callback is not None:
            callback(text)
        return

    if key == "BACKSPACE":
        session.input_buffer = session.input_buffer[:-1]
        return

    if len(key) == 1 and key.isprintable():
        session.input_buffer += key
        if len(session.input_buffer) > 64:
            session.input_buffer = session.input_buffer[-64:]


# ---------------------------------------------------------------------------
# Incremental "/" search
# ---------------------------------------------------------------------------


def start_search():
    # Keep any existing query so "/" re-opens it for refinement.
    session.search_active = True


def process_search(key):
    if key == "ESC":
        session.search_active = False
        session.search_query = ""
        return

    if key == "ENTER":
        session.search_active = False
        search.jump(0)
        return

    if key == "BACKSPACE":
        session.search_query = session.search_query[:-1]
        search.jump(0)
        return

    if len(key) == 1 and key.isprintable():
        session.search_query += key
        if len(session.search_query) > 48:
            session.search_query = session.search_query[:48]
        search.jump(0)


# ---------------------------------------------------------------------------
# Flight actions: takeoff / land / RTL
# ---------------------------------------------------------------------------

#: Default climb for [T] takeoff, matching PX4's MIS_TAKEOFF_ALT default.
TAKEOFF_DEFAULT_ALT = 2.5


def open_takeoff_input():
    with state.lock:
        armed = state.armed
        have_global = state.global_pos_valid

    if not armed:
        log_warn("Takeoff: arm first with [a]")
        return
    if not have_global:
        log_warn("Takeoff needs a GPS / global position")
        return

    request_input(
        f"Takeoff altitude in metres (blank = {TAKEOFF_DEFAULT_ALT:g}):",
        _takeoff_submit,
    )


def _takeoff_submit(text):
    text = text.strip()

    if not text:
        altitude = TAKEOFF_DEFAULT_ALT
    else:
        try:
            altitude = float(text)
        except ValueError:
            log_error("Takeoff: altitude must be a number")
            return

    if altitude <= 0:
        log_error("Takeoff: altitude must be positive")
        return

    request_confirmation(
        f"TAKEOFF and climb to {altitude:.1f} m? Type YES",
        lambda: send_takeoff(session.link, altitude),
    )


# ---------------------------------------------------------------------------
# Goto - forward/right/down offset from the current position (map screen)
# ---------------------------------------------------------------------------


def open_goto_input():
    with state.lock:
        have_global = state.global_pos_valid
        armed = state.armed

    if not have_global:
        log_warn("Goto needs a GPS / global position")
        return
    if not armed:
        log_warn("Goto: vehicle is not armed")

    request_input(
        "Goto  forward right down [yaw]   metres, deg   e.g.  10 0 -2",
        _goto_submit,
    )


def _goto_submit(text):
    parts = text.replace(",", " ").split()

    try:
        nums = [float(p) for p in parts]
    except ValueError:
        log_error("Goto: expected numbers 'forward right down [yaw]'")
        return

    if len(nums) < 3:
        log_error("Goto: need forward right down (metres)")
        return

    forward, right, down = nums[0], nums[1], nums[2]
    yaw = nums[3] if len(nums) >= 4 else None

    horizontal = math.hypot(forward, right)
    prompt = (
        f"GOTO  fwd {forward:+.1f}  right {right:+.1f}  down {down:+.1f} m"
        + (f"  yaw {yaw:.0f}" if yaw is not None else "")
        + f"   ({horizontal:.1f} m horizontal). Type YES"
    )
    request_confirmation(
        prompt,
        lambda: send_goto_body(session.link, forward, right, down, yaw),
    )


# ---------------------------------------------------------------------------
# Keyboard jog / nudge control (on the map screen, [x] to arm)
# ---------------------------------------------------------------------------

_jog_last_nudge = 0.0

# key -> (forward, right, down, yaw-delta) unit deltas, scaled by the step.
_JOG_MOVES = {
    "k": (1.0, 0.0, 0.0, 0.0),    # forward
    "j": (-1.0, 0.0, 0.0, 0.0),   # backward
    "a": (0.0, -1.0, 0.0, 0.0),   # strafe left
    "d": (0.0, 1.0, 0.0, 0.0),    # strafe right
    "w": (0.0, 0.0, -1.0, 0.0),   # up
    "s": (0.0, 0.0, 1.0, 0.0),    # down
    # Yaw is an absolute compass heading (see _body_to_ned() below): increasing
    # it turns the vehicle clockwise/right, so "yaw left" must decrease it.
    "h": (0.0, 0.0, 0.0, -1.0),   # yaw left
    "l": (0.0, 0.0, 0.0, 1.0),    # yaw right
}


def toggle_jog():
    if session.jog_armed:
        session.jog_armed = False
        log_command("JOG disarmed")
        return

    with state.lock:
        armed = state.armed
        have_global = state.global_pos_valid

    if not armed:
        log_warn("Jog: arm the vehicle first with [a]")
        return
    if not have_global:
        log_warn("Jog needs a GPS / global position")
        return

    request_confirmation(
        "Arm keyboard JOG? w/s/k/j/a/d/h/l then nudge the vehicle immediately. Type YES",
        _arm_jog,
    )


def _arm_jog():
    session.jog_step = JOG_STEP_M
    session.jog_armed = True
    log_command("JOG armed - w/s up/down, k/j fwd/back, a/d strafe, h/l yaw, [/] step")


def _jog_key(key):
    """Handle a jog key on the map screen. Returns True if it was consumed."""
    global _jog_last_nudge

    if key in ("[",):
        session.jog_step = clamp(session.jog_step / 2.0, JOG_STEP_MIN_M, JOG_STEP_MAX_M)
        return True
    if key in ("]",):
        session.jog_step = clamp(session.jog_step * 2.0, JOG_STEP_MIN_M, JOG_STEP_MAX_M)
        return True

    move = _JOG_MOVES.get(key)
    if move is None:
        return False

    now = time.monotonic()
    if now - _jog_last_nudge < JOG_MIN_INTERVAL:
        return True
    _jog_last_nudge = now

    step = session.jog_step
    fwd_unit, right_unit, down_unit, yaw_sign = move
    forward = fwd_unit * step
    right = right_unit * step
    down = down_unit * step

    yaw = None
    if yaw_sign:
        with state.lock:
            yaw = (state.yaw + yaw_sign * JOG_YAW_STEP_DEG) % 360.0

    if send_goto_body(session.link, forward, right, down, yaw, quiet=True):
        if yaw is not None:
            log_command(f"JOG yaw -> {yaw:.0f}")
        else:
            log_command(f"JOG fwd {forward:+.1f} right {right:+.1f} down {down:+.1f} m")
    return True


# ---------------------------------------------------------------------------
# Mode-select screen
# ---------------------------------------------------------------------------


def open_mode_select():
    session.screen = "mode_select"

    options = get_mode_options()

    if options:
        with state.lock:
            current_mode = state.mode

        try:
            session.mode_index = next(
                i for i, option in enumerate(options)
                if option["kind"] == "legacy" and option["value"] == current_mode
            )
        except StopIteration:
            session.mode_index = 0
    else:
        session.mode_index = 0

    # Lazily fetch the full mode list (including custom / external modes) the
    # first time this screen is opened, same pattern as the parameter screen.
    if session.link is not None and vehicle_ready(session.link):
        with state.lock:
            have_custom_modes = bool(state.custom_modes)
            already_requested = state.custom_modes_requested_at > 0

        if not have_custom_modes and not already_requested:
            request_available_modes(session.link)


def handle_mode_key(key):
    if key == "m":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if key == "r":
        request_available_modes(session.link)
        return

    options = get_mode_options()

    if not options:
        return

    count = len(options)
    session.mode_index %= count

    if key == "UP":
        session.mode_index = (session.mode_index - 1) % count
        return
    if key == "DOWN":
        session.mode_index = (session.mode_index + 1) % count
        return
    if key == "PGUP":
        session.mode_index = max(0, session.mode_index - MODE_PAGE_SIZE)
        return
    if key == "PGDN":
        session.mode_index = min(count - 1, session.mode_index + MODE_PAGE_SIZE)
        return
    if key == "HOME":
        session.mode_index = 0
        return
    if key == "END":
        session.mode_index = count - 1
        return

    if key == "ENTER":
        selected_option = options[session.mode_index]
        session.screen = "dashboard"

        request_confirmation(
            f"Set mode to {selected_option['label']}? Type YES",
            lambda option=selected_option: confirm_mode_option(option),
        )


# ---------------------------------------------------------------------------
# Event-log screen
# ---------------------------------------------------------------------------


def open_log_screen():
    session.screen = "log"

    with state.lock:
        total = len(state.events)

    session.log_scroll = max(0, total - log_page_size())


def handle_log_scroll(key):
    with state.lock:
        total = len(state.events)

    page_size = log_page_size()
    maximum = max(0, total - page_size)

    if key == "UP":
        session.log_scroll -= 1
    elif key == "DOWN":
        session.log_scroll += 1
    elif key == "PGUP":
        session.log_scroll -= page_size
    elif key == "PGDN":
        session.log_scroll += page_size
    elif key == "HOME":
        session.log_scroll = 0
    elif key == "END":
        session.log_scroll = maximum

    session.log_scroll = clamp(session.log_scroll, 0, maximum)


# ---------------------------------------------------------------------------
# Estimation / control screens (read-only, just re-request streams)
# ---------------------------------------------------------------------------


def _scroll_main(key):
    """jk/UP/DOWN/PGUP/PGDN/HOME/END scroll a screen's content when the main
    panel is too short to show all of it - see chrome.draw_frame(). Only for
    screens with no navigable list of their own (dashboard, estimation,
    control, calibration); every other screen's UP/DOWN already means
    something (select a mode/parameter/log line/flight log) and takes
    priority, so this is never wired into their key handlers.

    Returns True if `key` was one of those and was consumed.
    """
    if key == "UP":
        session.main_scroll = max(0, session.main_scroll - 1)
    elif key == "DOWN":
        session.main_scroll += 1  # clamped to the real max by draw_frame()
    elif key == "PGUP":
        session.main_scroll = max(0, session.main_scroll - 10)
    elif key == "PGDN":
        session.main_scroll += 10
    elif key == "HOME":
        session.main_scroll = 0
    elif key == "END":
        session.main_scroll = 1 << 30  # clamped down to the real max
    else:
        return False

    return True


def open_estimation_screen():
    session.screen = "estimation"

    if session.link is None:
        return

    now = time.monotonic()

    with state.lock:
        last_request = state.estimator_params_requested_at

    # Re-apply the faster stream rates and (re)read the EKF2_* aiding params
    # each time the screen is opened, but not more than once every few seconds.
    if now - last_request > 3.0:
        configure_streams(session.link)
        request_estimator_params(session.link)


def handle_estimation_key(key):
    if key == "e":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if _scroll_main(key):
        return

    if key == "r":
        configure_streams(session.link)
        request_estimator_params(session.link)
        log_info("Re-requested estimation streams and EKF2 params")


def open_control_screen():
    session.screen = "control"

    if session.link is not None:
        configure_streams(session.link)


def handle_control_key(key):
    if key == "c":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if _scroll_main(key):
        return

    if key == "r":
        configure_streams(session.link)
        log_info("Re-requested control / setpoint streams")


# ---------------------------------------------------------------------------
# Position map screen
# ---------------------------------------------------------------------------


def open_map_screen():
    session.screen = "map"

    if session.link is not None:
        configure_streams(session.link)


def handle_map_key(key):
    if key == "n":
        session.jog_armed = False
        session.screen = "dashboard"
        return

    if key == "ESC":
        # ESC always disarms jog, whether or not it was armed - a safety
        # action that must survive the switch to the general "focus the
        # sidebar" meaning below.
        session.jog_armed = False
        _focus_sidebar()
        return

    if key == "x":
        toggle_jog()
        return

    # When jog is armed, the movement keys nudge the vehicle.
    if session.jog_armed and _jog_key(key):
        return

    if key == "i":
        start_satellite_download()
        return

    if key == "g":
        open_goto_input()
        return

    if key in ("+", "="):
        with state.lock:
            state.map_range = clamp(state.map_range / 1.5, 2.0, 5000.0)
        return

    if key in ("-", "_"):
        with state.lock:
            state.map_range = clamp(state.map_range * 1.5, 2.0, 5000.0)
        return

    if key == "0":
        with state.lock:
            state.map_range = 30.0
        return

    if key == "t":
        with state.lock:
            state.map_trail_enabled = not state.map_trail_enabled
        return

    if key == "c":
        with state.lock:
            state.position_trail.clear()
        return

    if key == "r":
        configure_streams(session.link)


# ---------------------------------------------------------------------------
# LiDAR point-cloud screen
# ---------------------------------------------------------------------------


def open_pointcloud_screen():
    session.screen = "pointcloud"


def handle_pointcloud_key(key):
    if key == "v":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if key in ("+", "="):
        with state.lock:
            state.lidar_view_range = clamp(state.lidar_view_range / 1.5, 1.0, 200.0)
        return

    if key in ("-", "_"):
        with state.lock:
            state.lidar_view_range = clamp(state.lidar_view_range * 1.5, 1.0, 200.0)
        return

    if key == "0":
        with state.lock:
            state.lidar_view_range = 10.0
        return

    if key == "1":
        with state.lock:
            state.lidar_view_mode = "top"
        return

    if key == "2":
        with state.lock:
            state.lidar_view_mode = "front"
        return

    if key == "3":
        with state.lock:
            state.lidar_view_mode = "oblique"
        return

    if key == "t":
        open_pointcloud_topic_input()
        return

    _scroll_main(key)


def open_pointcloud_topic_input():
    request_input(
        f"Point-cloud topic (current: {settings.lidar_topic}):",
        _pointcloud_topic_submit,
    )


def _pointcloud_topic_submit(text):
    text = text.strip()
    if not text:
        return
    if not text.startswith("/"):
        text = "/" + text

    settings.lidar_topic = text
    log_command(f"Point-cloud topic changed to {text}")


# ---------------------------------------------------------------------------
# Calibration screen
# ---------------------------------------------------------------------------


def open_calibration_screen():
    session.screen = "calibration"

    if session.link is not None:
        configure_streams(session.link)


def handle_calibration_key(key):
    with state.lock:
        active = state.cal_active

    if key == "r":
        configure_streams(session.link)
        log_info(f"Re-requested STATUSTEXT / EVENT / streams on port {settings.port}")
        return

    if active:
        if key in ("ESC", "x"):
            cancel_calibration(session.link)
        return

    if key == "s":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if _scroll_main(key):
        return

    prompts = {
        "g": ("gyro", "Calibrate GYRO? Keep the vehicle completely still. Type YES"),
        "a": ("accel", "Calibrate ACCEL? You must place the vehicle on all 6 sides when prompted. Type YES"),
        "l": ("level", "Calibrate LEVEL HORIZON? Set the vehicle level and still. Type YES"),
        "c": ("mag", "Calibrate COMPASS? You must rotate the vehicle about all axes when prompted. Type YES"),
        "b": ("baro", "Calibrate BAROMETER? Keep the vehicle still. Type YES"),
    }

    if key in prompts:
        cal_type, message = prompts[key]
        request_confirmation(
            message,
            lambda t=cal_type: send_calibration(session.link, t),
        )


# ---------------------------------------------------------------------------
# Host USB / network screen
# ---------------------------------------------------------------------------


def open_about_screen():
    session.screen = "about"


def handle_about_key(key):
    if key == "?":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return


def open_host_screen():
    session.screen = "host"


def handle_host_key(key):
    if key == "u":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if key == "i":
        if run_speedtest_async():
            log_command("Speed test started (download/upload/ping to a public server)")
        else:
            log_warn("Speed test already running")
        return

    _scroll_main(key)


# ---------------------------------------------------------------------------
# MAVLink shell screen
# ---------------------------------------------------------------------------


def open_shell_screen():
    session.screen = "shell"
    session.shell_follow = True

    with state.lock:
        already_active = state.shell_active

    if not already_active:
        if claim_shell(session.link):
            with state.lock:
                state.shell_active = True
            log_command("MAVLink shell opened (NSH)")
        else:
            log_error("Could not open MAVLink shell")


def handle_shell_key(key):
    if key == "ESC":
        # The NSH session stays open in the background either way - ESC just
        # sends focus to the sidebar instead of forcing the dashboard.
        _focus_sidebar()
        return

    if key == "CTRL_C":
        send_shell_raw(session.link, b"\x03")
        return

    if key == "ENTER":
        send_shell_raw(session.link, b"\n")
        session.shell_follow = True
        return

    if key == "BACKSPACE":
        send_shell_raw(session.link, b"\x7f")
        return

    if key == "TAB":
        send_shell_raw(session.link, b"\t")
        return

    if key == "UP":
        session.shell_scroll -= 1
        session.shell_follow = False
        return

    if key == "DOWN":
        session.shell_scroll += 1
        return

    if key == "PGUP":
        session.shell_scroll -= 10
        session.shell_follow = False
        return

    if key == "PGDN":
        session.shell_scroll += 10
        return

    if key == "HOME":
        session.shell_scroll = 0
        session.shell_follow = False
        return

    if key == "END":
        session.shell_follow = True
        return

    if len(key) == 1 and key.isprintable():
        send_shell_raw(session.link, key.encode("utf-8", errors="ignore"))


# ---------------------------------------------------------------------------
# Flight-log screen
# ---------------------------------------------------------------------------


def open_flight_log_screen():
    session.screen = "flight_logs"

    with state.lock:
        need_request = (
            not state.flight_log_order
            and state.flight_log_list_requested_at <= 0
        )

    if need_request:
        request_flight_logs(session.link)


def _last_downloaded_log_path():
    """The most recently *completed* download's local path, or None.

    Both [u] upload and [a] EKF check operate on this rather than on
    whichever row the list cursor happens to be sitting on, since the point
    of both actions is to inspect a log this console already pulled off the
    vehicle.
    """
    with state.lock:
        path = state.flight_log_download_path
        status = state.flight_log_download_status

    if status == "COMPLETE" and path and os.path.isfile(path):
        return path
    return None


def upload_last_flight_log():
    if job_running():
        log_warn("A background job is already running")
        return

    path = _last_downloaded_log_path()
    if not path:
        log_warn("Download a flight log first (ENTER), then press u to upload it")
        return

    start_ulog_upload(path)


def analyze_last_flight_log():
    if job_running():
        log_warn("A background job is already running")
        return

    path = _last_downloaded_log_path()
    if not path:
        log_warn("Download a flight log first (ENTER), then press a to run the EKF check")
        return

    start_ecl_ekf_check(path)


def handle_flight_log_key(key):
    with state.lock:
        active = state.flight_log_download_active
        job_active = state.job_active and state.job_kind in ("ulog_upload", "ecl_ekf")
        logs = [
            state.flight_logs[log_id]
            for log_id in state.flight_log_order
            if log_id in state.flight_logs
        ]
        selected = state.flight_log_index

    if active:
        if key == "ESC":
            cancel_flight_log_download()
        return

    if job_active:
        if key == "ESC":
            cancel_job()
        return

    if key == "l":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if key == "r":
        request_flight_logs(session.link)
        return

    if key == "u":
        upload_last_flight_log()
        return

    if key == "a":
        analyze_last_flight_log()
        return

    if not logs:
        return

    selected = clamp(selected, 0, len(logs) - 1)

    if key == "UP":
        with state.lock:
            state.flight_log_index = (selected - 1) % len(logs)
    elif key == "DOWN":
        with state.lock:
            state.flight_log_index = (selected + 1) % len(logs)
    elif key == "PGUP":
        with state.lock:
            state.flight_log_index = max(0, selected - FLIGHT_LOG_PAGE_SIZE)
    elif key == "PGDN":
        with state.lock:
            state.flight_log_index = min(len(logs) - 1, selected + FLIGHT_LOG_PAGE_SIZE)
    elif key == "HOME":
        with state.lock:
            state.flight_log_index = 0
    elif key == "END":
        with state.lock:
            state.flight_log_index = len(logs) - 1
    elif key == "ENTER":
        start_flight_log_download(logs[selected])


# ---------------------------------------------------------------------------
# Flash-firmware screen
# ---------------------------------------------------------------------------


def refresh_firmware_lists():
    files = discover_firmware_files()
    ports = discover_serial_ports()

    with state.lock:
        state.firmware_files = files
        state.firmware_index = clamp(state.firmware_index, 0, max(0, len(files) - 1))
        state.firmware_ports = ports
        state.firmware_port_index = clamp(state.firmware_port_index, 0, max(0, len(ports) - 1))


def open_firmware_screen():
    session.screen = "firmware"
    refresh_firmware_lists()


def _confirm_firmware_flash(fw_path, port):
    request_confirmation(
        f"FLASH {os.path.basename(fw_path)} to {port}?"
        " This reboots the flight controller. Type YES",
        lambda: start_firmware_flash(fw_path, port),
    )


def handle_firmware_key(key):
    with state.lock:
        job_active = state.job_active and state.job_kind == "firmware"
        armed = state.armed
        files = list(state.firmware_files)
        file_index = state.firmware_index
        ports = list(state.firmware_ports)
        port_index = state.firmware_port_index

    if job_active:
        if key == "ESC":
            cancel_job()
        return

    if key == "f":
        session.screen = "dashboard"
        return

    if key == "ESC":
        _focus_sidebar()
        return

    if key == "r":
        refresh_firmware_lists()
        return

    if key == "UP" and files:
        with state.lock:
            state.firmware_index = (file_index - 1) % len(files)
        return

    if key == "DOWN" and files:
        with state.lock:
            state.firmware_index = (file_index + 1) % len(files)
        return

    if key == "LEFT" and ports:
        with state.lock:
            state.firmware_port_index = (port_index - 1) % len(ports)
        return

    if key == "RIGHT" and ports:
        with state.lock:
            state.firmware_port_index = (port_index + 1) % len(ports)
        return

    if key == "ENTER":
        if not files:
            log_warn(f"No .px4 firmware files found in {settings.firmware_dir}")
            return
        if not ports:
            log_warn("No serial ports detected - plug in the flight controller via USB")
            return
        if armed:
            log_warn("Refusing to flash firmware while the vehicle is ARMED")
            return

        fw_path = files[clamp(file_index, 0, len(files) - 1)]
        port = ports[clamp(port_index, 0, len(ports) - 1)]
        _confirm_firmware_flash(fw_path, port)


# ---------------------------------------------------------------------------
# Parameter screen
# ---------------------------------------------------------------------------


def open_parameter_screen():
    session.screen = "parameters"

    with state.lock:
        state.parameter_index = 0
        state.parameter_page = 0
        state.parameter_edit_active = False

    if session.link is not None and vehicle_ready(session.link):
        with state.lock:
            already_requested = state.parameter_full_list_requested

        # A handful of EKF2_* params are fetched individually for the
        # estimation view; that must not suppress the first full load here.
        if not already_requested:
            request_parameters(session.link)


def cancel_parameter_edit():
    with state.lock:
        state.parameter_edit_active = False
        state.parameter_edit_name = ""
        state.parameter_edit_buffer = ""


def start_parameter_edit():
    visible = get_visible_parameters()

    with state.lock:
        if not visible:
            return

        index = clamp(state.parameter_index, 0, len(visible) - 1)
        parameter = visible[index]

        if parameter.pending:
            log_warn(f"{parameter.name} is already waiting for PX4 confirmation")
            return

        state.parameter_edit_active = True
        state.parameter_edit_name = parameter.name
        state.parameter_edit_buffer = parameter_format_value(parameter)


def submit_parameter_edit():
    with state.lock:
        name = state.parameter_edit_name
        buffer = state.parameter_edit_buffer

        if not name:
            return

        parameter = state.parameters.get(name)

    if parameter is None:
        log_error(f"Parameter disappeared: {name}")
        cancel_parameter_edit()
        return

    try:
        value = parse_parameter_value(buffer, parameter)
    except Exception as exc:
        log_error(f"Invalid value for {name}: {exc}")
        return

    with state.lock:
        state.parameter_edit_active = False
        state.parameter_edit_name = ""
        state.parameter_edit_buffer = ""

    request_confirmation(
        f"Set {name} to {value:g}? Type YES",
        lambda name=name, value=value: confirm_parameter_set(name, value),
    )


def _param_move(new_index_fn):
    with state.lock:
        visible_count = len(get_visible_parameters())
        if visible_count:
            state.parameter_index = new_index_fn(state.parameter_index, visible_count)
            state.parameter_page = state.parameter_index // PARAM_PAGE_SIZE


def handle_parameter_key(key):
    with state.lock:
        edit_active = state.parameter_edit_active

    if edit_active:
        if key == "ESC":
            cancel_parameter_edit()
            return

        if key == "BACKSPACE":
            with state.lock:
                state.parameter_edit_buffer = state.parameter_edit_buffer[:-1]
            return

        if key == "ENTER":
            submit_parameter_edit()
            return

        if len(key) == 1 and key.isprintable():
            with state.lock:
                state.parameter_edit_buffer += key
                if len(state.parameter_edit_buffer) > 64:
                    state.parameter_edit_buffer = state.parameter_edit_buffer[-64:]
        return

    if key == "p":
        session.screen = "dashboard"
    elif key == "ESC":
        _focus_sidebar()
    elif key == "UP":
        _param_move(lambda i, n: (i - 1) % n)
    elif key == "DOWN":
        _param_move(lambda i, n: (i + 1) % n)
    elif key == "PGUP":
        _param_move(lambda i, n: max(0, i - PARAM_PAGE_SIZE))
    elif key == "PGDN":
        _param_move(lambda i, n: min(max(0, n - 1), i + PARAM_PAGE_SIZE))
    elif key == "HOME":
        with state.lock:
            state.parameter_index = 0
            state.parameter_page = 0
    elif key == "END":
        _param_move(lambda i, n: n - 1)
    elif key == "ENTER":
        start_parameter_edit()
    elif key == "v":
        with state.lock:
            state.parameter_view = "CHANGED" if state.parameter_view == "ALL" else "ALL"
            state.parameter_index = 0
            state.parameter_page = 0
    elif key == "r":
        request_parameters(session.link)
    elif key == "b":
        request_confirmation(
            "REBOOT PX4? Vehicle will restart. Type YES",
            confirm_reboot,
        )


# ---------------------------------------------------------------------------
# Sidebar panel (lazygit/lazydocker-style TAB-to-focus navigation)
# ---------------------------------------------------------------------------


def _sync_nav_index_to_screen():
    """Point the sidebar cursor at whichever row matches the active screen."""
    for i, (screen_key, _label, _hotkey) in enumerate(NAV_ITEMS):
        if screen_key == session.screen:
            session.nav_index = i
            return


def _focus_sidebar():
    """Send focus to the sidebar, cursor on whatever screen is showing.

    This is ESC's general-browsing meaning everywhere except the handful of
    screen-local "busy" states (an active calibration, an armed jog, a
    parameter mid-edit, a flight-log mid-download) where ESC must keep its
    more specific cancel/abort meaning instead - those are checked first in
    each screen's own key handler, before it falls through to this.
    """
    session.nav_focus = "sidebar"
    _sync_nav_index_to_screen()


def _preview_selected():
    """Show the sidebar cursor's row in the main panel without "opening" it -
    no stream requests, no per-screen state reset. Committing (ENTER) is what
    runs the real opener; this is just a live look at each row as you pass it,
    the way lazygit's main panel follows the selection in the side list."""
    session.screen = NAV_ITEMS[session.nav_index][0]


def _handle_sidebar_key(key):
    """Keys while the sidebar panel has focus - see the TAB toggle below.

    The sidebar owns the keyboard outright while focused (like a lazygit
    panel): every other screen's own key handling is skipped.
    """
    if key == "CTRL_C":
        shutdown_event.set()
        return

    if key in ("ESC", "TAB"):
        session.nav_focus = "main"
        return

    if key in ("UP", "k"):
        session.nav_index = (session.nav_index - 1) % len(NAV_ITEMS)
        _preview_selected()
        return

    if key in ("DOWN", "j"):
        session.nav_index = (session.nav_index + 1) % len(NAV_ITEMS)
        _preview_selected()
        return

    if key == "ENTER":
        screen_key, _label, hotkey = NAV_ITEMS[session.nav_index]
        session.nav_focus = "main"

        # Route through the same opener the hotkey would use, so switching
        # panels from the sidebar re-requests streams / params exactly like
        # pressing the hotkey directly does.
        if hotkey and hotkey in _SCREEN_OPENERS:
            _SCREEN_OPENERS[hotkey]()
        else:
            session.screen = screen_key
        return

    if key == "q":
        shutdown_event.set()


# ---------------------------------------------------------------------------
# Top-level key dispatch
# ---------------------------------------------------------------------------


def normalize_vim_key(key):
    """Map common Vim navigation keys to the console's navigation actions."""
    return {
        "j": "DOWN",
        "k": "UP",
        "CTRL_F": "PGDN",
        "CTRL_B": "PGUP",
        "CTRL_D": "PGDN",
        "CTRL_U": "PGUP",
    }.get(key, key)


def process_key(key):
    if session.confirm_active:
        if key == "CTRL_C":
            shutdown_event.set()
            return
        process_confirmation(key)
        return

    if session.input_active:
        if key == "CTRL_C":
            shutdown_event.set()
            return
        process_input(key)
        return

    if session.search_active:
        if key == "CTRL_C":
            shutdown_event.set()
            return
        process_search(key)
        return

    # nav_focus == "main" excludes a sidebar live-preview of the shell row
    # (which sets session.screen = "shell" to show it in the main panel
    # without actually being in it yet) - only the committed shell, entered
    # via ENTER, should swallow every key including Tab and hotkey letters.
    if session.screen == "shell" and session.nav_focus == "main":
        handle_shell_key(key)
        return

    if key == "CTRL_C":
        shutdown_event.set()
        return

    # TAB toggles focus to/from the sidebar panel (lazygit/lazydocker style);
    # not reachable from the shell screen above, which keeps Tab for NSH
    # autocomplete. While the sidebar has focus it owns every other key too.
    if key == "TAB":
        if session.nav_focus == "sidebar":
            session.nav_focus = "main"
        else:
            _focus_sidebar()
        return

    if session.nav_focus == "sidebar":
        _handle_sidebar_key(key)
        return

    # "/" search and n/N jump on the list screens (but not while editing a
    # parameter value, where the keys are literal text).
    if session.screen in search.SEARCHABLE:
        with state.lock:
            param_editing = (
                session.screen == "parameters" and state.parameter_edit_active
            )
        if not param_editing:
            if key == "/":
                start_search()
                return
            if key == "n":
                search.jump(1)
                return
            if key == "N":
                search.jump(-1)
                return

    # 'q' quits from any browse screen, but not while typing into the
    # parameter editor or while a flight-log download / calibration runs.
    if key == "q":
        with state.lock:
            busy = (
                (session.screen == "parameters" and state.parameter_edit_active)
                or state.flight_log_download_active
                or state.cal_active
            )
        if not busy and not (session.screen == "map" and session.jog_armed):
            shutdown_event.set()
            return

    # Vim navigation is global for browse screens, but not for the MAVLink
    # shell / map (jog) / parameter text-entry, where the letters are data.
    if session.screen not in ("shell", "map"):
        with state.lock:
            parameter_editing = (
                session.screen == "parameters" and state.parameter_edit_active
            )
        if not parameter_editing:
            key = normalize_vim_key(key)

    if session.screen == "parameters":
        handle_parameter_key(key)
        return

    if session.screen == "log":
        if key == "g":
            session.screen = "dashboard"
            return
        if key == "ESC":
            _focus_sidebar()
            return
        if key == "c":
            with state.lock:
                state.events.clear()
            log_info("Event log cleared")
            return
        if key in ("UP", "DOWN", "PGUP", "PGDN", "HOME", "END"):
            handle_log_scroll(key)
        return

    if session.screen == "mode_select":
        handle_mode_key(key)
        return

    if session.screen == "flight_logs":
        handle_flight_log_key(key)
        return

    if session.screen == "estimation":
        handle_estimation_key(key)
        return

    if session.screen == "control":
        handle_control_key(key)
        return

    if session.screen == "map":
        handle_map_key(key)
        return

    if session.screen == "pointcloud":
        handle_pointcloud_key(key)
        return

    if session.screen == "calibration":
        handle_calibration_key(key)
        return

    if session.screen == "host":
        handle_host_key(key)
        return

    if session.screen == "about":
        handle_about_key(key)
        return

    if session.screen == "firmware":
        handle_firmware_key(key)
        return

    _handle_dashboard_key(key)


_SCREEN_OPENERS = {
    "g": open_log_screen,
    "m": open_mode_select,
    "l": open_flight_log_screen,
    "p": open_parameter_screen,
    "t": open_shell_screen,
    "e": open_estimation_screen,
    "c": open_control_screen,
    "n": open_map_screen,
    "s": open_calibration_screen,
    "u": open_host_screen,
    "f": open_firmware_screen,
    "v": open_pointcloud_screen,
    "?": open_about_screen,
}


def _handle_dashboard_key(key):
    opener = _SCREEN_OPENERS.get(key)
    if opener is not None:
        opener()
        return

    if _scroll_main(key):
        return

    if key == "a":
        with state.lock:
            if state.pending_arm is not None:
                log_warn("ARM/DISARM command already pending")
                return
            if state.armed:
                log_warn("ARM ignored: vehicle already ARMED")
                return
        request_confirmation("ARM vehicle? Type YES", confirm_arm)
        return

    if key == "d":
        with state.lock:
            if state.pending_arm is not None:
                log_warn("ARM/DISARM command already pending")
                return
            if not state.armed:
                log_warn("DISARM ignored: vehicle already DISARMED")
                return
        request_confirmation("DISARM vehicle? Type YES", confirm_disarm)
        return

    if key == "h":
        request_confirmation("HOLD vehicle? Type YES", confirm_hold)
        return

    if key == "T":
        open_takeoff_input()
        return

    if key == "L":
        request_confirmation(
            "LAND here? Type YES",
            lambda: send_land(session.link),
        )
        return

    if key == "R":
        request_confirmation(
            "RETURN TO LAUNCH? Type YES",
            lambda: send_rtl(session.link),
        )
        return

    if key == "ESC":
        _focus_sidebar()


def process_all_keys():
    while True:
        try:
            key = key_queue.get_nowait()
        except queue.Empty:
            break

        process_key(key)
