"""The [s] sensor-calibration screen."""

from __future__ import annotations

import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW
from ..config import CALIBRATION_LABELS, settings
from ..state import state
from ..util import clamp

CALIBRATION_MENU = (
    ("g", "gyro", "Gyroscope", "keep the vehicle completely still (~5 s)"),
    ("a", "accel", "Accelerometer", "place on each of the 6 sides when prompted"),
    ("l", "level", "Level horizon", "set the vehicle exactly level, keep still"),
    ("c", "mag", "Compass / magnetometer", "rotate about all axes when prompted"),
    ("b", "baro", "Barometer", "keep the vehicle still"),
)


def draw_calibration_screen():
    with state.lock:
        armed = state.armed
        imu = state.imu
        mag = state.mag
        baro = state.baro
        gps_sensor = state.gps_sensor

        active = state.cal_active
        cal_type = state.cal_type
        progress = state.cal_progress
        result = state.cal_result
        started_at = state.cal_started_at
        sent_at = state.cal_sent_at
        cal_ack = state.cal_ack
        cal_log = list(state.cal_log)
        last_message = state.cal_last_message
        last_status_text = state.last_status_text
        statustext_since = state.statustext_count - state.cal_statustext_at_start
        event_since = state.event_count - state.cal_event_at_start
        statustext_total = state.statustext_count

    now = time.monotonic()
    port = settings.port

    lines = []

    if armed:
        arm_text = RED + BOLD + "ARMED - calibration is blocked" + RESET
    else:
        arm_text = GREEN + "DISARMED" + RESET
    lines.append(f" Vehicle: {arm_text}")

    def health(value):
        return GREEN + "OK" + RESET if value else RED + "FAIL" + RESET

    lines.append(
        f" Sensor health:  IMU {health(imu)}   MAG {health(mag)}"
        f"   BARO {health(baro)}   GPS {health(gps_sensor)}"
    )
    lines.append("")

    if active:
        elapsed = now - started_at if started_at else 0.0
        label = CALIBRATION_LABELS.get(cal_type, cal_type)

        filled = int(round(clamp(progress, 0, 100) / 10.0))
        bar = "#" * filled + "-" * (10 - filled)

        lines.append(
            BOLD + YELLOW + f" RUNNING: {label} calibration" + RESET
            + f"    [{bar}] {progress:3d}%    {elapsed:4.0f}s"
        )

        if cal_ack:
            ack_color = GREEN if cal_ack in ("ACCEPTED", "IN_PROGRESS") else RED
            lines.append(f" PX4 command ACK: {ack_color}{cal_ack}{RESET}")
        elif sent_at:
            lines.append(DIM + f" PX4 command ACK: waiting ({now - sent_at:.0f}s)" + RESET)

        lines.append(
            f" link rx since start:  STATUSTEXT {statustext_since}   EVENT {event_since}"
        )

        lines.append("")
        lines.append(BOLD + " PX4 [cal] messages:" + RESET)

        if cal_log:
            for message in cal_log[-11:]:
                lines.append("   " + message)
        else:
            lines.append(DIM + "   (none yet)" + RESET)

        lines.append("")

        if last_status_text:
            lines.append(DIM + " last PX4 status: " + last_status_text[:80] + RESET)

        no_progress = (not cal_log) and sent_at and (now - sent_at) > 6.0

        if cal_ack in ("DENIED", "UNSUPPORTED", "FAILED", "TEMPORARILY_REJECTED"):
            lines.append(RED + f" PX4 rejected the command ({cal_ack})." + RESET)
            lines.append(RED + " Vehicle must be DISARMED and on the ground. Press [x]." + RESET)
        elif no_progress and statustext_total == 0:
            lines.append(RED + " PX4 accepted the command but no STATUSTEXT reaches this link." + RESET)
            lines.append(
                DIM + f" Nothing on UDP {port} carries PX4's text messages, so [cal]"
                " prompts can't be shown. On the vehicle's PX4 console run" + RESET
            )
            lines.append(DIM + "   mavlink status" + RESET)
            lines.append(
                DIM + " find the instance on this UDP port; its mode should be 'normal'"
                " (not 'onboard'/'minimal'). Restart it with -m normal, or point lazypx4" + RESET
            )
            lines.append(DIM + " (--port) at a normal-mode instance. Then [r]." + RESET)
        elif no_progress:
            lines.append(YELLOW + " PX4 accepted it but has sent no [cal] progress yet." + RESET)
            lines.append(
                DIM + " Check the [g] event log. If EVENT count above keeps rising, this"
                " firmware reports calibration via events (no text).  [r] re-request; [x] abort." + RESET
            )
        else:
            lines.append(YELLOW + " Follow the prompts above, then wait for 'calibration done'." + RESET)

        lines.append("[r] re-request STATUSTEXT/EVENT streams    [x]/ESC = cancel")

        return lines

    if result:
        if result == "DONE":
            lines.append(GREEN + f" Last calibration: DONE ({CALIBRATION_LABELS.get(cal_type, cal_type)})" + RESET)
        elif result == "CANCELLED":
            lines.append(YELLOW + " Last calibration: CANCELLED" + RESET)
        else:
            lines.append(RED + f" Last calibration: {result}" + RESET)
        if last_message:
            lines.append(DIM + "   " + last_message + RESET)
        lines.append("")

    lines.append(BOLD + " Select a calibration (vehicle must be DISARMED):" + RESET)
    lines.append("")

    for key, _cal_type, name, hint in CALIBRATION_MENU:
        lines.append(f"   [{key}]  {name:<24}{DIM}{hint}{RESET}")

    lines.append("")
    lines.append(DIM + " Accel and compass need you to physically move the airframe;" + RESET)
    lines.append(DIM + " PX4 detects each position automatically and reports progress here." + RESET)
    lines.append(DIM + " A reboot ([b] on the parameter screen) is recommended afterwards." + RESET)

    if statustext_total == 0:
        lines.append("")
        lines.append(
            YELLOW + f" Note: no STATUSTEXT seen on port {port} yet - calibration prompts"
            " may not be visible. [r] to re-request." + RESET
        )

    lines.append("")
    lines.append("[r] re-request streams    [s] back    [ESC] panels")

    return lines
