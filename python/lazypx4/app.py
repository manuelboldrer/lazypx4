"""The application: connect, start the threads, run the render/poll loop."""

from __future__ import annotations

import threading
import time

from .config import (
    BATTERY_CRITICAL,
    BATTERY_LOW,
    GPS_PUBLISH_FEEDBACK_TIMEOUT_S,
    HEARTBEAT_TIMEOUT,
    PREFLIGHT_FAIL_TIMEOUT,
    REFRESH_HZ,
    TELEMETRY_TIMEOUT,
    settings,
)
from .camera import camera_thread
from .eventlog import log_error, log_failsafe, log_info, log_warn
from .gpspub import gps_pub_thread
from .lidar import lidar_thread
from .mapfeeds import mapfeeds_thread
from .mavlink.calibration import check_calibration
from .mavlink.commands import check_pending_arm
from .mavlink.connection import (
    configure_streams,
    connect_vehicle,
    request_estimator_params,
    send_gcs_heartbeat,
)
from .mavlink.fence import check_fence_upload
from .mavlink.flightlog import check_flight_log_list
from .mavlink.modes import check_available_modes_request
from .mavlink.parameters import (
    check_parameter_request,
    check_pending_parameter,
    load_parameter_defaults,
)
from .mavlink.receiver import mavlink_thread
from .mavlink.wp_queue import wp_queue_thread
from .navigation import load_kml_file, process_all_keys
from .netmon import netmon_thread
from .render import draw
from .rosclock import ros_clock_thread
from .state import session, shutdown_event, state
from .sysmon import sysmon_thread
from .terminal import keyboard_thread, terminal_restore, terminal_start


def _health_transition(active_attr, is_bad, on_bad, on_good):
    """Toggle a latched health flag and log only on the edges."""
    if is_bad:
        if not getattr(state, active_attr):
            setattr(state, active_attr, True)
            on_bad()
    else:
        if getattr(state, active_attr):
            setattr(state, active_attr, False)
            on_good()


def update_health():
    """Latch link / telemetry / battery health and log state transitions."""
    now = time.monotonic()

    with state.lock:
        heartbeat_stale = (
            state.last_heartbeat > 0 and now - state.last_heartbeat > HEARTBEAT_TIMEOUT
        )

        if heartbeat_stale:
            if not state.heartbeat_timeout_active:
                state.heartbeat_timeout_active = True
                state.connected = False
                log_error("PX4 HEARTBEAT TIMEOUT")
        else:
            if state.heartbeat_timeout_active:
                state.heartbeat_timeout_active = False
                state.connected = True
                log_info("PX4 HEARTBEAT recovered")

        _health_transition(
            "position_timeout_active",
            state.last_position > 0 and now - state.last_position > TELEMETRY_TIMEOUT,
            lambda: log_warn("Position telemetry timeout"),
            lambda: log_info("Position telemetry recovered"),
        )
        _health_transition(
            "gps_timeout_active",
            state.last_gps > 0 and now - state.last_gps > TELEMETRY_TIMEOUT,
            lambda: log_warn("GPS telemetry timeout"),
            lambda: log_info("GPS telemetry recovered"),
        )
        _health_transition(
            "ekf_timeout_active",
            state.last_ekf > 0 and now - state.last_ekf > TELEMETRY_TIMEOUT,
            lambda: log_warn("EKF telemetry timeout"),
            lambda: log_info("EKF telemetry recovered"),
        )
        _health_transition(
            "rc_timeout_active",
            state.last_rc > 0 and now - state.last_rc > TELEMETRY_TIMEOUT,
            lambda: log_warn("RC telemetry timeout"),
            lambda: log_info("RC telemetry recovered"),
        )

        if (
            state.last_preflight_fail
            and now - state.last_preflight_fail_time > PREFLIGHT_FAIL_TIMEOUT
        ):
            state.last_preflight_fail = ""
            log_info("PREARM check cleared (PX4 stopped reporting it)")

        if (
            state.gps_pub_feedback
            and now - state.gps_pub_feedback_time > GPS_PUBLISH_FEEDBACK_TIMEOUT_S
        ):
            state.gps_pub_feedback = ""

        if state.battery >= 0:
            if state.battery <= BATTERY_CRITICAL:
                if not state.battery_critical_active:
                    state.battery_critical_active = True
                    log_failsafe(f"CRITICAL BATTERY: {state.battery:.0f}%")
            else:
                if state.battery_critical_active:
                    state.battery_critical_active = False
                    log_info("Battery recovered above critical threshold")

            if state.battery <= BATTERY_LOW and state.battery > BATTERY_CRITICAL:
                if not state.battery_low_active:
                    state.battery_low_active = True
                    log_warn(f"LOW BATTERY: {state.battery:.0f}%")
            else:
                if state.battery_low_active:
                    state.battery_low_active = False
                    log_info("Battery recovered above low threshold")


_STARTUP_NOTES = (
    "Console ready",
    "ARM/DISARM are explicit commands",
    "Heartbeat is authoritative for actual ARM state",
    "[T] takeoff  [L] land  [R] return-to-launch  (or pick the mode with [m])",
    "Use [m] MODE for all other PX4 flight-mode changes",
    "Press [p] for PX4 parameters ([/] to filter)",
    "Press [c] control/setpoints, [n] mission map",
    "Press [s] for sensor calibration (gyro / accel / level / compass / baro)",
)


def run():
    """Blocking: run lazypx4 until the user quits or a fatal error occurs."""
    try:
        load_parameter_defaults()

        if settings.kml_path:
            load_kml_file(settings.kml_path)

        session.link = connect_vehicle()

        threading.Thread(
            target=mavlink_thread,
            args=(session.link,),
            daemon=True,
            name="MAVLinkThread",
        ).start()

        # Announce ourselves as a GCS before asking for anything, so PX4
        # treats this link as an active ground-station connection.
        send_gcs_heartbeat(session.link)

        # Ask PX4 for slightly faster estimation/sensor streams and read the
        # EKF2_* aiding parameters shown on the dashboard.
        configure_streams(session.link)
        request_estimator_params(session.link)

        terminal_start()

        threading.Thread(target=keyboard_thread, daemon=True, name="KeyboardThread").start()
        threading.Thread(target=sysmon_thread, daemon=True, name="SysMonThread").start()
        threading.Thread(target=netmon_thread, daemon=True, name="NetMonThread").start()
        threading.Thread(target=ros_clock_thread, daemon=True, name="RosClockThread").start()
        threading.Thread(target=lidar_thread, daemon=True, name="LidarThread").start()
        threading.Thread(target=mapfeeds_thread, daemon=True, name="MapFeedsThread").start()
        threading.Thread(target=camera_thread, daemon=True, name="CameraThread").start()
        threading.Thread(target=wp_queue_thread, daemon=True, name="WpQueueThread").start()
        threading.Thread(target=gps_pub_thread, daemon=True, name="GpsPubThread").start()

        for note in _STARTUP_NOTES:
            log_info(note)
        log_info(f"On the map, [i] saves a pinned satellite image to {settings.map_dir}")
        log_info(
            f"Press [l] for PX4 flight logs; downloads go to {settings.log_dir} "
            "([u] upload to web, [a] EKF health-check)"
        )
        log_info(f"Press [f] to flash firmware from {settings.firmware_dir}")
        log_info(f"Press [v] for a LiDAR point-cloud overview ({settings.lidar_topic})")
        log_info("Press [c] / [r] for control, setpoints and RC stick positions / channels")
        log_info(f"Press [w] for a camera preview ({settings.camera_topic_1 or 'no topic set'})")

        frame_period = 1.0 / REFRESH_HZ
        next_frame = time.monotonic()

        while not shutdown_event.is_set():
            process_all_keys()

            send_gcs_heartbeat(session.link)
            check_pending_arm()
            check_parameter_request()
            check_pending_parameter()
            check_available_modes_request()
            check_flight_log_list()
            check_calibration()
            check_fence_upload()

            update_health()

            now = time.monotonic()

            if now >= next_frame:
                draw()
                next_frame = now + frame_period
            else:
                time.sleep(min(0.01, max(0.0, next_frame - now)))

    except KeyboardInterrupt:
        shutdown_event.set()

    except Exception as exc:
        try:
            log_error(f"Fatal error: {exc}")
        except Exception:
            pass

        shutdown_event.set()

    finally:
        shutdown_event.set()
        time.sleep(0.15)
        terminal_restore()
