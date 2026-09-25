"""The background MAVLink receiver thread.

Receives, filters to the locked vehicle, and dispatches each message to a
handler. Runs until :data:`lazypx4.state.shutdown_event` is set.
"""

from __future__ import annotations

import time

from ..eventlog import log_error, log_info
from ..state import shutdown_event, state
from ..util import safe_int
from . import fence, handlers, modes
from .flightlog import handle_log_data, handle_log_entry
from .parameters import handle_param_value
from .shell import handle_serial_control

# msg_type -> handler(msg). Several PX4/ArduPilot variants share a handler.
_DISPATCH = {
    "HEARTBEAT": handlers.handle_heartbeat,
    "SYSTEM_TIME": handlers.handle_system_time,
    "GLOBAL_POSITION_INT": handlers.handle_global_position,
    "LOCAL_POSITION_NED": handlers.handle_local_position,
    "ODOMETRY": handlers.handle_odometry,
    "ATTITUDE": handlers.handle_attitude,
    "ATTITUDE_TARGET": handlers.handle_attitude_target,
    "POSITION_TARGET_LOCAL_NED": handlers.handle_position_target_local_ned,
    "NAV_CONTROLLER_OUTPUT": handlers.handle_nav_controller_output,
    "BATTERY_STATUS": handlers.handle_battery_status,
    "SYS_STATUS": handlers.handle_sys_status,
    "GPS_RAW_INT": handlers.handle_gps,
    "GPS2_RAW": handlers.handle_gps2,
    "GPS_RTK": handlers.handle_gps_rtk,
    "GPS2_RTK": handlers.handle_gps_rtk,
    "GPS_STATUS": handlers.handle_gps_status,
    "GPS_GLOBAL_ORIGIN": handlers.handle_gps_global_origin,
    "HOME_POSITION": handlers.handle_home_position,
    "ALTITUDE": handlers.handle_altitude,
    "DISTANCE_SENSOR": handlers.handle_distance_sensor,
    "VIBRATION": handlers.handle_vibration,
    "WIND": handlers.handle_wind,
    "WIND_COV": handlers.handle_wind_cov,
    "SERVO_OUTPUT_RAW": handlers.handle_servo_output_raw,
    "FENCE_STATUS": handlers.handle_fence_status,
    "AUTOPILOT_VERSION": handlers.handle_autopilot_version,
    "SCALED_PRESSURE": handlers.handle_scaled_pressure,
    "SCALED_PRESSURE2": handlers.handle_scaled_pressure,
    "SCALED_PRESSURE3": handlers.handle_scaled_pressure,
    "ESTIMATOR_STATUS": handlers.handle_estimator_status,
    "EKF_STATUS_REPORT": handlers.handle_ekf_status_report,
    "RC_CHANNELS": handlers.handle_rc,
    "RC_CHANNELS_RAW": handlers.handle_rc,
    "VFR_HUD": handlers.handle_vfr_hud,
    "STATUSTEXT": handlers.handle_statustext,
    "EVENT": handlers.handle_event,
    "COMMAND_ACK": handlers.handle_command_ack,
    "PARAM_VALUE": handle_param_value,
    "AVAILABLE_MODES": modes.handle_available_modes,
    "CURRENT_MODE": modes.handle_current_mode,
    "AVAILABLE_MODES_MONITOR": modes.handle_available_modes_monitor,
    "SERIAL_CONTROL": handle_serial_control,
    "LOG_ENTRY": handle_log_entry,
    "LOG_DATA": handle_log_data,
    "MISSION_REQUEST": fence.handle_mission_request,
    "MISSION_REQUEST_INT": fence.handle_mission_request,
    "MISSION_ACK": fence.handle_mission_ack,
}


def mavlink_thread(master):
    log_info("MAVLink receiver thread started")
    last_rate_time = time.monotonic()
    last_rate_count = 0

    while not shutdown_event.is_set():
        try:
            msg = master.recv_match(blocking=True, timeout=0.5)
        except Exception as exc:
            if not shutdown_event.is_set():
                log_error(f"MAVLink receive error: {exc}")
            time.sleep(0.1)
            continue

        if msg is None:
            continue

        msg_type = msg.get_type()

        with state.lock:
            locked = state.vehicle_locked
            target_system = state.target_system

        try:
            source_system = safe_int(msg.get_srcSystem(), 0)
        except Exception:
            source_system = 0

        if locked and source_system != target_system:
            continue

        with state.lock:
            state.rx_messages += 1
            state.last_rx = time.monotonic()

        handler = _DISPATCH.get(msg_type)
        if handler is not None:
            try:
                handler(msg)
            except Exception as exc:
                log_error(f"Error processing {msg_type}: {exc}")

        now = time.monotonic()
        if now - last_rate_time >= 1.0:
            with state.lock:
                current_count = state.rx_messages
                state.rx_rate = float(current_count - last_rate_count)
            last_rate_count = current_count
            last_rate_time = now

    log_info("MAVLink receiver thread stopped")
