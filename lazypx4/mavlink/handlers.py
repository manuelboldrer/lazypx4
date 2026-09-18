"""Per-message telemetry handlers.

Each ``handle_*`` takes a decoded ``mavutil`` message and folds it into
:data:`lazypx4.state.state` under ``state.lock``. They are dispatched by
:mod:`lazypx4.mavlink.receiver`.
"""

from __future__ import annotations

import math
import time

from pymavlink import mavutil

from ..config import (
    ACK_NAMES,
    COMMAND_NAMES,
    DISTANCE_SENSOR_ORIENTATION_DOWN,
    FIRMWARE_VERSION_TYPE_NAMES,
    MAV_CMD_PREFLIGHT_CALIBRATION,
    POSITION_TRAIL_MIN_SPACING_M,
)
from ..eventlog import (
    classify_statustext,
    log_command,
    log_error,
    log_failsafe,
    log_info,
    log_warn,
)
from ..state import state
from ..util import finite, safe_float, safe_int
from .calibration import handle_calibration_text


def handle_heartbeat(msg):
    now = time.monotonic()
    source_system = safe_int(msg.get_srcSystem(), 0)
    source_component = safe_int(msg.get_srcComponent(), 0)
    autopilot = safe_int(getattr(msg, "autopilot", -1), -1)

    if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return

    with state.lock:
        if state.vehicle_locked and source_system != state.target_system:
            return

        if not state.vehicle_locked:
            state.target_system = source_system
            state.target_component = source_component
            state.vehicle_locked = True
            log_info(f"Locked onto MAVLink vehicle system={source_system}")

    base_mode = safe_int(getattr(msg, "base_mode", 0))
    armed_now = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    try:
        mode_now = mavutil.mode_string_v10(msg) or "UNKNOWN"
    except Exception:
        mode_now = "UNKNOWN"

    custom_mode = safe_int(getattr(msg, "custom_mode", 0))
    system_status = safe_int(getattr(msg, "system_status", 0))

    with state.lock:
        previous_armed = state.armed
        previous_mode = state.mode

        state.last_heartbeat = now
        state.last_rx = now
        state.base_mode = base_mode
        state.custom_mode = custom_mode
        state.system_status = system_status

        if mode_now == "UNKNOWN":
            for entry in state.custom_modes.values():
                if entry.custom_mode == custom_mode:
                    mode_now = entry.name
                    break

        state.armed = armed_now
        state.mode = mode_now

        if armed_now and not previous_armed:
            state.last_preflight_fail = ""

        pending = state.pending_arm
        confirmed_action = None
        if pending is not None and armed_now == pending.desired_armed:
            confirmed_action = "ARM" if pending.desired_armed else "DISARM"
            state.pending_arm = None

    if confirmed_action:
        log_command(f"{confirmed_action} CONFIRMED by HEARTBEAT")

    if armed_now != previous_armed:
        log_command(
            "HEARTBEAT: VEHICLE IS ARMED" if armed_now else "HEARTBEAT: VEHICLE IS DISARMED"
        )

    if mode_now != previous_mode:
        log_info(f"PX4 MODE: {previous_mode} -> {mode_now}")


def _store_local_position(x, y, z, now):
    # Caller holds state.lock.
    if not (math.isfinite(x) and math.isfinite(y)):
        return

    state.local_x = x
    state.local_y = y
    state.local_z = z if math.isfinite(z) else 0.0
    state.local_pos_valid = True
    state.last_local_pos = now

    trail = state.position_trail
    if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= POSITION_TRAIL_MIN_SPACING_M:
        trail.append((x, y))


def handle_global_position(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_position = now

        state.global_lat = safe_float(msg.lat) / 1e7
        state.global_lon = safe_float(msg.lon) / 1e7
        state.global_alt = safe_float(getattr(msg, "alt", 0)) / 1000.0
        state.global_pos_valid = True


def handle_local_position(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_position = now

        state.x = safe_float(msg.x)
        state.y = safe_float(msg.y)
        state.z = safe_float(msg.z)

        state.vx = safe_float(msg.vx)
        state.vy = safe_float(msg.vy)
        state.vz = safe_float(msg.vz)

        _store_local_position(state.x, state.y, state.z, now)


def handle_odometry(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_position = now

        state.x = safe_float(getattr(msg, "x", 0))
        state.y = safe_float(getattr(msg, "y", 0))
        state.z = safe_float(getattr(msg, "z", 0))

        state.vx = safe_float(getattr(msg, "vx", 0))
        state.vy = safe_float(getattr(msg, "vy", 0))
        state.vz = safe_float(getattr(msg, "vz", 0))

        _store_local_position(state.x, state.y, state.z, now)


def handle_attitude(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now

        state.roll = math.degrees(safe_float(msg.roll))
        state.pitch = math.degrees(safe_float(msg.pitch))
        state.yaw = math.degrees(safe_float(msg.yaw))

        state.roll_rate = math.degrees(safe_float(getattr(msg, "rollspeed", 0)))
        state.pitch_rate = math.degrees(safe_float(getattr(msg, "pitchspeed", 0)))
        state.yaw_rate = math.degrees(safe_float(getattr(msg, "yawspeed", 0)))


def quaternion_to_euler_deg(q):
    values = [safe_float(v) for v in list(q)[:4]]
    values += [0.0] * (4 - len(values))
    w, x, y, z = values

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def handle_attitude_target(msg):
    now = time.monotonic()

    roll, pitch, yaw = quaternion_to_euler_deg(getattr(msg, "q", [1.0, 0.0, 0.0, 0.0]))

    with state.lock:
        state.last_rx = now
        state.last_att_target = now

        state.att_target_roll = roll
        state.att_target_pitch = pitch
        state.att_target_yaw = yaw
        state.att_target_thrust = safe_float(getattr(msg, "thrust", 0))
        state.att_target_roll_rate = math.degrees(safe_float(getattr(msg, "body_roll_rate", 0)))
        state.att_target_pitch_rate = math.degrees(safe_float(getattr(msg, "body_pitch_rate", 0)))
        state.att_target_yaw_rate = math.degrees(safe_float(getattr(msg, "body_yaw_rate", 0)))


def handle_position_target_local_ned(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_pos_target = now

        state.pos_target_x = safe_float(getattr(msg, "x", 0))
        state.pos_target_y = safe_float(getattr(msg, "y", 0))
        state.pos_target_z = safe_float(getattr(msg, "z", 0))
        state.pos_target_vx = safe_float(getattr(msg, "vx", 0))
        state.pos_target_vy = safe_float(getattr(msg, "vy", 0))
        state.pos_target_vz = safe_float(getattr(msg, "vz", 0))
        state.pos_target_yaw = math.degrees(safe_float(getattr(msg, "yaw", 0)))
        state.pos_target_yaw_rate = math.degrees(safe_float(getattr(msg, "yaw_rate", 0)))
        state.pos_target_type_mask = safe_int(getattr(msg, "type_mask", 0))
        state.pos_target_frame = safe_int(getattr(msg, "coordinate_frame", 0))


def handle_nav_controller_output(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_nav_output = now

        state.nav_roll = safe_float(getattr(msg, "nav_roll", 0))
        state.nav_pitch = safe_float(getattr(msg, "nav_pitch", 0))
        state.nav_bearing = safe_float(getattr(msg, "nav_bearing", 0))
        state.nav_target_bearing = safe_float(getattr(msg, "target_bearing", 0))
        state.nav_wp_dist = safe_float(getattr(msg, "wp_dist", 0))
        state.nav_alt_error = safe_float(getattr(msg, "alt_error", 0))
        state.nav_aspd_error = safe_float(getattr(msg, "aspd_error", 0))
        state.nav_xtrack_error = safe_float(getattr(msg, "xtrack_error", 0))


def handle_battery_status(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now

        remaining = safe_float(getattr(msg, "battery_remaining", -1), -1)

        if remaining >= 0:
            state.battery = remaining

        voltage_mv = safe_float(getattr(msg, "voltage_battery", 0), 0)

        if voltage_mv > 0:
            state.voltage = voltage_mv / 1000.0

        current = safe_float(getattr(msg, "current_battery", 0), 0)

        if current > 0:
            state.current = current


def handle_sys_status(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now

        remaining = safe_float(getattr(msg, "battery_remaining", -1), -1)

        if remaining >= 0:
            state.battery = remaining

        voltage_mv = safe_float(getattr(msg, "voltage_battery", 0), 0)

        if voltage_mv > 0:
            state.voltage = voltage_mv / 1000.0

        current = safe_float(getattr(msg, "current_battery", 0), 0)

        if current > 0:
            state.current = current / 100.0

        health = safe_int(getattr(msg, "onboard_control_sensors_health", 0))

        state.sensors_present = safe_int(getattr(msg, "onboard_control_sensors_present", 0))
        state.sensors_enabled = safe_int(getattr(msg, "onboard_control_sensors_enabled", 0))
        state.sensors_health = health
        state.last_sys_status = now

        try:
            state.imu = bool(health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
            state.mag = bool(health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
            state.baro = bool(health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
            state.gps_sensor = bool(health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)
        except Exception:
            pass


def _gps_dop(value):
    # eph / epv are uint16 centi-units; 0 and UINT16_MAX mean "unknown".
    value = safe_float(value, 0.0)
    if 0 < value < 65535:
        return value / 100.0
    return None


def _gps_acc_m(value):
    # h_acc / v_acc / vel_acc are uint32 millimetre units; 0 and UINT32_MAX
    # mean "unknown".
    value = safe_float(value, 0.0)
    if 0 < value < 4294967295:
        return value / 1000.0
    return None


def _gps_cog(value):
    # cog is uint16 centidegrees; UINT16_MAX means "unknown".
    value = safe_float(value, 0.0)
    if 0 <= value < 65535:
        return value / 100.0
    return None


def _gps_heading(yaw_value, hdg_acc_value):
    # yaw (dual-antenna GPS heading, uint16 centidegrees) is 0 when the
    # receiver has none to report; PX4 sends 36000 for true north instead of
    # 0 so it isn't confused with that "unavailable" marker. hdg_acc (degE5)
    # is only ever set alongside a valid yaw, so 0 there means "unknown".
    yaw_value = safe_float(yaw_value, 0.0)
    if yaw_value <= 0:
        return None, None

    hdg_acc_value = safe_float(hdg_acc_value, 0.0)
    accuracy = hdg_acc_value / 1e5 if hdg_acc_value > 0 else None
    return yaw_value / 100.0, accuracy


def handle_gps(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_gps = now

        state.gps_fix = safe_int(getattr(msg, "fix_type", 0))
        state.gps_sats = safe_int(getattr(msg, "satellites_visible", 0))

        hdop = _gps_dop(getattr(msg, "eph", 0))
        if hdop is not None:
            state.gps_hdop = hdop

        vdop = _gps_dop(getattr(msg, "epv", 0))
        if vdop is not None:
            state.gps_vdop = vdop

        alt_mm = safe_float(getattr(msg, "alt", 0))
        if alt_mm:
            state.gps_alt = alt_mm / 1000.0

        h_acc = _gps_acc_m(getattr(msg, "h_acc", 0))
        if h_acc is not None:
            state.gps_h_acc = h_acc

        v_acc = _gps_acc_m(getattr(msg, "v_acc", 0))
        if v_acc is not None:
            state.gps_v_acc = v_acc

        vel_acc = _gps_acc_m(getattr(msg, "vel_acc", 0))
        if vel_acc is not None:
            state.gps_vel_acc = vel_acc

        vel = safe_float(getattr(msg, "vel", 0))
        if 0 <= vel < 65535:
            state.gps_speed = vel / 100.0

        cog = _gps_cog(getattr(msg, "cog", 65535))
        if cog is not None:
            state.gps_cog = cog

        alt_ellipsoid_mm = safe_float(getattr(msg, "alt_ellipsoid", 0))
        if alt_ellipsoid_mm:
            state.gps_alt_ellipsoid = alt_ellipsoid_mm / 1000.0

        heading, heading_acc = _gps_heading(
            getattr(msg, "yaw", 0), getattr(msg, "hdg_acc", 0)
        )
        state.gps_heading = heading if heading is not None else -1.0
        if heading_acc is not None:
            state.gps_heading_acc = heading_acc


def handle_gps2(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_gps2 = now

        state.gps2_fix = safe_int(getattr(msg, "fix_type", 0))
        state.gps2_sats = safe_int(getattr(msg, "satellites_visible", 0))

        hdop = _gps_dop(getattr(msg, "eph", 0))
        if hdop is not None:
            state.gps2_hdop = hdop

        # Age of the last RTCM/DGPS correction (ms) and number of correction
        # channels - the clearest "corrections are still arriving" signal PX4
        # exposes over telemetry. UINT16_MAX / -1 mean "unknown".
        dgps_age = safe_float(getattr(msg, "dgps_age", 0), 0.0)
        if 0 <= dgps_age < 4294967295:
            state.gps_dgps_age = dgps_age / 1000.0
        state.gps_dgps_numch = safe_int(getattr(msg, "dgps_numch", 0), 0)


def handle_gps_rtk(msg):
    # GPS_RTK / GPS2_RTK: the RTK-solution detail. Only meaningful once the
    # receiver is being fed RTCM corrections from a base station / NTRIP caster.
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_gps_rtk = now

        state.rtk_health = safe_int(getattr(msg, "rtk_health", -1), -1)
        state.rtk_rate = safe_float(getattr(msg, "rtk_rate", 0), 0.0)
        state.rtk_nsats = safe_int(getattr(msg, "nsats", 0), 0)
        state.rtk_iar_hypotheses = safe_int(getattr(msg, "iar_num_hypotheses", 0), 0)

        # baseline_a/b/c_mm is the base -> rover vector (ECEF or NED depending
        # on baseline_coords_type); its magnitude is the distance to the base
        # station, which is what bounds the achievable RTK accuracy.
        a = safe_float(getattr(msg, "baseline_a_mm", 0), 0.0)
        b = safe_float(getattr(msg, "baseline_b_mm", 0), 0.0)
        c = safe_float(getattr(msg, "baseline_c_mm", 0), 0.0)
        state.rtk_baseline_m = math.sqrt(a * a + b * b + c * c) / 1000.0


def handle_gps_global_origin(msg):
    with state.lock:
        state.last_rx = time.monotonic()
        state.local_origin_set = True
        state.local_origin_lat = safe_float(getattr(msg, "latitude", 0)) / 1e7
        state.local_origin_lon = safe_float(getattr(msg, "longitude", 0)) / 1e7
        state.local_origin_alt = safe_float(getattr(msg, "altitude", 0)) / 1000.0


def handle_home_position(msg):
    with state.lock:
        state.last_rx = time.monotonic()
        state.home_set = True
        state.home_lat = safe_float(getattr(msg, "latitude", 0)) / 1e7
        state.home_lon = safe_float(getattr(msg, "longitude", 0)) / 1e7
        state.home_alt = safe_float(getattr(msg, "altitude", 0)) / 1000.0


def handle_altitude(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_altitude = now

        state.alt_monotonic = finite(getattr(msg, "altitude_monotonic", 0))
        state.alt_amsl = finite(getattr(msg, "altitude_amsl", 0))
        state.alt_local = finite(getattr(msg, "altitude_local", 0))
        state.alt_relative = finite(getattr(msg, "altitude_relative", 0))
        state.alt_terrain = finite(getattr(msg, "altitude_terrain", 0))
        state.alt_bottom_clearance = finite(getattr(msg, "bottom_clearance", 0))


def handle_system_time(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_system_time = now

        state.autopilot_unix_usec = safe_int(getattr(msg, "time_unix_usec", 0))
        state.autopilot_boot_ms = safe_int(getattr(msg, "time_boot_ms", 0))


def handle_distance_sensor(msg):
    now = time.monotonic()

    orientation = safe_int(getattr(msg, "orientation", -1), -1)

    with state.lock:
        # Prefer the down-facing sensor, since that is what feeds AGL /
        # terrain. Only fall back to another orientation if we have never
        # seen a down-facing one.
        if (
            state.rangefinder_orientation == DISTANCE_SENSOR_ORIENTATION_DOWN
            and orientation != DISTANCE_SENSOR_ORIENTATION_DOWN
        ):
            return

        state.last_rx = now
        state.last_rangefinder = now

        state.rangefinder_orientation = orientation
        state.rangefinder_distance = safe_float(getattr(msg, "current_distance", 0)) / 100.0
        state.rangefinder_min = safe_float(getattr(msg, "min_distance", 0)) / 100.0
        state.rangefinder_max = safe_float(getattr(msg, "max_distance", 0)) / 100.0
        state.rangefinder_quality = safe_int(getattr(msg, "signal_quality", -1), -1)


def handle_scaled_pressure(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_baro = now

        press = safe_float(getattr(msg, "press_abs", 0))
        if press > 0:
            state.baro_pressure = press

        state.baro_temp = safe_float(getattr(msg, "temperature", 0)) / 100.0


def handle_vibration(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_vibration = now

        state.vibration_x = safe_float(getattr(msg, "vibration_x", 0))
        state.vibration_y = safe_float(getattr(msg, "vibration_y", 0))
        state.vibration_z = safe_float(getattr(msg, "vibration_z", 0))
        state.clipping_0 = safe_int(getattr(msg, "clipping_0", 0))
        state.clipping_1 = safe_int(getattr(msg, "clipping_1", 0))
        state.clipping_2 = safe_int(getattr(msg, "clipping_2", 0))


def handle_wind(msg):
    """Legacy WIND message: direction/speed given directly (ArduPilot-style)."""
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_wind = now

        state.wind_direction = safe_float(getattr(msg, "direction", 0)) % 360.0
        state.wind_speed = safe_float(getattr(msg, "speed", 0))
        state.wind_speed_z = safe_float(getattr(msg, "speed_z", 0))


def handle_wind_cov(msg):
    """WIND_COV: wind given as a NED velocity vector - what PX4 streams."""
    now = time.monotonic()

    wind_x = safe_float(getattr(msg, "wind_x", 0))
    wind_y = safe_float(getattr(msg, "wind_y", 0))
    wind_z = safe_float(getattr(msg, "wind_z", 0))

    speed = math.hypot(wind_x, wind_y)
    # wind_x/y point in the direction the wind is blowing TOWARDS; add 180 deg
    # to report the heading it is blowing FROM, the usual aviation convention.
    direction = (math.degrees(math.atan2(wind_y, wind_x)) + 180.0) % 360.0

    with state.lock:
        state.last_rx = now
        state.last_wind = now

        state.wind_speed = speed
        state.wind_direction = direction
        state.wind_speed_z = wind_z


def handle_servo_output_raw(msg):
    now = time.monotonic()

    outputs = [
        safe_int(getattr(msg, f"servo{i}_raw", 0))
        for i in range(1, 9)
    ]

    with state.lock:
        state.last_rx = now
        state.last_servo_output = now
        state.servo_outputs = outputs


def handle_fence_status(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_fence_status = now

        state.fence_breach_status = safe_int(getattr(msg, "breach_status", 0))
        state.fence_breach_count = safe_int(getattr(msg, "breach_count", 0))
        state.fence_breach_type = safe_int(getattr(msg, "breach_type", 0))


def _decode_fw_version(flight_sw_version, flight_custom_version):
    major = (flight_sw_version >> 24) & 0xFF
    minor = (flight_sw_version >> 16) & 0xFF
    patch = (flight_sw_version >> 8) & 0xFF
    kind = FIRMWARE_VERSION_TYPE_NAMES.get(flight_sw_version & 0xFF, "?")

    version_text = f"v{major}.{minor}.{patch} ({kind})"

    hash_bytes = bytes(flight_custom_version or ())
    git_hash = hash_bytes.hex()[:8] if any(hash_bytes) else ""

    return version_text, git_hash


def handle_autopilot_version(msg):
    now = time.monotonic()

    try:
        version_text, git_hash = _decode_fw_version(
            safe_int(getattr(msg, "flight_sw_version", 0)),
            getattr(msg, "flight_custom_version", None),
        )
    except Exception:
        return

    with state.lock:
        state.last_rx = now
        state.fw_version_text = version_text
        state.fw_git_hash = git_hash
        state.autopilot_version_received = True


def handle_gps_status(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_gps = now

        state.gps_sats = safe_int(getattr(msg, "satellites_visible", 0))


def handle_estimator_status(msg):
    # PX4's estimator health report. `flags` here use ESTIMATOR_STATUS_FLAGS.
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_ekf = now

        state.est_is_estimator_status = True
        state.ekf_flags = safe_int(getattr(msg, "flags", 0))
        state.ekf_status = "OK"

        state.est_vel_ratio = finite(getattr(msg, "vel_ratio", 0))
        state.est_pos_horiz_ratio = finite(getattr(msg, "pos_horiz_ratio", 0))
        state.est_pos_vert_ratio = finite(getattr(msg, "pos_vert_ratio", 0))
        state.est_mag_ratio = finite(getattr(msg, "mag_ratio", 0))
        state.est_hagl_ratio = finite(getattr(msg, "hagl_ratio", 0))
        state.est_tas_ratio = finite(getattr(msg, "tas_ratio", 0))
        state.est_pos_horiz_accuracy = finite(getattr(msg, "pos_horiz_accuracy", 0))
        state.est_pos_vert_accuracy = finite(getattr(msg, "pos_vert_accuracy", 0))


def handle_ekf_status_report(msg):
    # ArduPilot-style report. `flags` here use EKF_STATUS_FLAGS and it carries
    # variances rather than innovation test ratios.
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_ekf = now

        state.est_is_estimator_status = False
        state.ekf_flags = safe_int(getattr(msg, "flags", 0))
        state.ekf_status = "OK"

        state.ekf_vel_variance = finite(getattr(msg, "velocity_variance", 0))
        state.ekf_pos_horiz_variance = finite(getattr(msg, "pos_horiz_variance", 0))
        state.ekf_pos_vert_variance = finite(getattr(msg, "pos_vert_variance", 0))
        state.ekf_compass_variance = finite(getattr(msg, "compass_variance", 0))
        state.ekf_terrain_variance = finite(getattr(msg, "terrain_alt_variance", 0))


def handle_rc(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.last_rc = now

        state.rc_received = True

        state.rc_rssi = safe_int(getattr(msg, "rssi", 0))
        state.rc_lq = safe_int(getattr(msg, "lq", 0))
        state.rc_failsafe = bool(getattr(msg, "rc_failsafe", False))

        state.rc_channels = [
            safe_int(getattr(msg, f"chan{i}_raw", 0)) for i in range(1, 19)
        ]


def handle_vfr_hud(msg):
    now = time.monotonic()

    with state.lock:
        state.last_rx = now
        state.thrust = safe_float(getattr(msg, "throttle", 0))
        state.groundspeed = safe_float(getattr(msg, "groundspeed", 0))
        state.airspeed = safe_float(getattr(msg, "airspeed", 0))
        state.climb_rate = safe_float(getattr(msg, "climb", 0))
        state.vfr_alt = safe_float(getattr(msg, "alt", 0))


def handle_statustext(msg):
    now = time.monotonic()

    try:
        text = getattr(msg, "text", "")

        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")

        text = str(text).rstrip("\x00")

    except Exception:
        text = repr(msg)

    severity = safe_int(getattr(msg, "severity", 6), 6)
    level = classify_statustext(severity, text)

    stripped = text.lstrip().lower()
    is_preflight_fail = (
        "preflight fail" in stripped or "prearm" in stripped or "arming denied" in stripped
    )

    with state.lock:
        state.last_rx = now
        state.last_status_text = text
        state.last_status_level = level
        state.statustext_count += 1
        calibrating = state.cal_active

        if is_preflight_fail:
            state.last_preflight_fail = text
            state.last_preflight_fail_time = now

    if stripped.startswith("[cal]") or (calibrating and "calibrat" in stripped):
        handle_calibration_text(text, now)

    if level == "FAILSAFE":
        log_failsafe(f"PX4: {text}")
    elif level == "ERROR":
        log_error(f"PX4: {text}")
    elif level == "WARN":
        log_warn(f"PX4: {text}")
    else:
        log_info(f"PX4: {text}")


def handle_event(msg):
    # PX4's structured EVENT interface (MAVLink msg 410). Decoding the
    # human-readable text needs the vehicle's event-metadata file, which this
    # console does not fetch - but counting them, and noting them during a
    # calibration, tells the operator that PX4 is reporting via events rather
    # than STATUSTEXT.
    now = time.monotonic()
    event_id = safe_int(getattr(msg, "id", -1), -1)

    with state.lock:
        state.last_rx = now
        state.event_count += 1
        calibrating = state.cal_active

    if calibrating:
        with state.lock:
            state.cal_last_message = f"PX4 EVENT id={event_id} (no text available)"
            state.cal_last_message_at = now
            state.cal_log.append(state.cal_last_message)


def _log_ack_result(prefix, result, result_name):
    """Log a COMMAND_ACK result at the right level."""
    if result == 0:
        log_command(f"{prefix}: ACCEPTED")
    elif result == 5:
        log_command(f"{prefix}: IN_PROGRESS")
    elif result == 1:
        log_warn(f"{prefix}: TEMPORARILY_REJECTED")
    elif result == 2:
        log_error(f"{prefix}: DENIED")
    elif result == 3:
        log_error(f"{prefix}: UNSUPPORTED")
    elif result == 4:
        log_error(f"{prefix}: FAILED")
    else:
        log_warn(f"{prefix}: {result_name}")


def handle_command_ack(msg):
    command_id = safe_int(getattr(msg, "command", -1), -1)
    result = safe_int(getattr(msg, "result", -1), -1)

    command_name = COMMAND_NAMES.get(command_id, f"COMMAND_{command_id}")
    result_name = ACK_NAMES.get(result, f"RESULT_{result}")

    with state.lock:
        state.last_ack = f"{command_name}: {result_name}"
        state.last_ack_time = time.monotonic()

    # Stream-rate tweaks (SET_MESSAGE_INTERVAL) and one-shot REQUEST_MESSAGE
    # calls are best-effort: a vehicle without a second GPS or a rangefinder
    # will reject some of them, which is expected and not worth logging.
    if command_id in (
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    ):
        return

    if command_id == MAV_CMD_PREFLIGHT_CALIBRATION:
        with state.lock:
            state.cal_ack = result_name
            state.cal_ack_at = time.monotonic()

        if result in (0, 5):
            log_command(
                "Calibration ACCEPTED" if result == 0 else "Calibration IN PROGRESS"
            )
        else:
            with state.lock:
                state.cal_active = False
                state.cal_result = result_name
            log_error(f"Calibration rejected by PX4: {result_name}")
        return

    if command_id == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        with state.lock:
            if state.pending_arm is not None:
                state.pending_arm.ack_received = True
                state.pending_arm.ack_result = result

        # Plain "-", not an em dash: the event-log screen pads every row to
        # the panel width by counting characters, and a glyph a given
        # terminal/font renders wider than one column (some render an em
        # dash that way) throws that count off, which shows up as the
        # row's right border landing in the wrong column.
        if result == 0:
            log_command("COMMAND_ACK 400: ACCEPTED - waiting for HEARTBEAT confirmation")
        elif result == 5:
            log_command("COMMAND_ACK 400: IN_PROGRESS - waiting for HEARTBEAT confirmation")
        elif result == 1:
            log_warn("COMMAND_ACK 400: TEMPORARILY_REJECTED")
            with state.lock:
                state.pending_arm = None
        elif result == 2:
            log_error("COMMAND_ACK 400: DENIED")
            with state.lock:
                state.pending_arm = None
        elif result == 3:
            log_error("COMMAND_ACK 400: UNSUPPORTED")
            with state.lock:
                state.pending_arm = None
        elif result == 4:
            log_error("COMMAND_ACK 400: FAILED")
            with state.lock:
                state.pending_arm = None
        elif result == 6:
            log_warn("COMMAND_ACK 400: CANCELLED")
            with state.lock:
                state.pending_arm = None
        else:
            log_warn(f"COMMAND_ACK 400: {result_name}")
            with state.lock:
                state.pending_arm = None

        return

    if command_id == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        if result == 0:
            log_command("PX4 REBOOT: COMMAND ACCEPTED")
        elif result == 5:
            log_command("PX4 REBOOT: IN PROGRESS")
        elif result == 1:
            log_warn("PX4 REBOOT: TEMPORARILY REJECTED")
        elif result == 2:
            log_error("PX4 REBOOT: DENIED")
        elif result == 3:
            log_error("PX4 REBOOT: UNSUPPORTED")
        elif result == 4:
            log_error("PX4 REBOOT: FAILED")
        else:
            log_warn(f"PX4 REBOOT: {result_name}")
        return

    _log_ack_result(command_name, result, result_name)
