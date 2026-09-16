"""The default screen: link / arm / mode / battery and a live telemetry summary."""

from __future__ import annotations

import math
import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import (
    BATTERY_CRITICAL,
    BATTERY_LOW,
    EKF2_HGT_REF_NAMES,
    GPS_HDOP_GOOD,
    GPS_HDOP_OK,
    GPS_SATS_GOOD,
    GPS_SATS_OK,
    POS_ACC_GOOD,
    POS_ACC_OK,
    RANGEFINDER_SIGNAL_GOOD,
    RANGEFINDER_SIGNAL_OK,
    RC_SIGNAL_GOOD,
    RC_SIGNAL_OK,
    RX_RATE_GOOD,
    RX_RATE_OK,
    settings,
)
from ..mavlink.connection import get_param_value
from ..state import state
from ..sysmon import stats as host
from ..util import finite
from .chrome import (
    distance_orientation_text,
    ekf_summary,
    format_clock,
    global_to_local,
    gps_fix_display,
    graded_color,
    level_color,
    rangefinder_status,
    rtk_text,
    sensor_health_items,
    state_text,
    terminal_size,
    vibration_verdict,
)


def _load_color(percent):
    if percent >= 90.0:
        return RED
    if percent >= 70.0:
        return YELLOW
    return GREEN


def _host_lines():
    """The HOST / SVC block, or [] if the system monitor has nothing yet."""
    with host.lock:
        ok = host.ok
        cpu = host.cpu_percent
        load1 = host.load1
        mem_pct = host.mem_percent
        mem_used = host.mem_used_gb
        mem_total = host.mem_total_gb
        disk_pct = host.disk_percent
        disk_free = host.disk_free_gb
        rosbag = host.rosbag_recording
        zenoh = host.zenoh_running
        xrce_agent = host.xrce_agent_running

    # U+25CF BLACK CIRCLE: renders single-width in virtually every terminal
    # font, which matters because the dashboard pads every row to the panel
    # width by counting characters - a glyph that renders wider than one
    # column would land the row's right border in the wrong place.
    def _dot(active):
        return (GREEN if active else RED) + "●" + RESET

    svc = (
        " SVC:  rosbag " + _dot(rosbag)
        + "    zenoh " + _dot(zenoh)
        + "    xrce-agent " + _dot(xrce_agent)
    )

    if not ok:
        return [" HOST: " + DIM + "n/a (not Linux / /proc unreadable)" + RESET, svc]

    return [
        " HOST: "
        + f"CPU {_load_color(cpu)}{cpu:3.0f}%{RESET}"
        + f"   RAM {_load_color(mem_pct)}{mem_pct:3.0f}%{RESET} "
        + DIM + f"{mem_used:.1f}/{mem_total:.1f}G" + RESET
        + f"   DISK {_load_color(disk_pct)}{disk_pct:3.0f}%{RESET} "
        + DIM + f"{disk_free:.0f}G free" + RESET
        + f"   load {load1:.2f}",
        svc,
    ]


def _time_line(now, ros_clock_supported, ros_time_ns, ros_time_sim, last_ros_time,
                autopilot_unix_usec, autopilot_boot_ms, last_system_time):
    if not ros_clock_supported:
        ros_text = DIM + "rclpy not found" + RESET
    elif ros_time_ns <= 0:
        ros_text = DIM + ("waiting for /clock" if ros_time_sim else "waiting...") + RESET
    else:
        ros_age = now - last_ros_time if last_ros_time else None
        age_note = f" {DIM}({ros_age:.1f}s ago){RESET}" if ros_age is not None else ""
        sim_note = f" {DIM}[sim]{RESET}" if ros_time_sim else ""
        # ns // 1000, not ns / 1e9 truncated, so the printed digits are exact
        # rather than float-rounded - this is the same unix_usec convention
        # MAVLink's SYSTEM_TIME uses, so it lines up with the FCU field below.
        ros_us = ros_time_ns // 1000
        ros_text = (
            format_clock(ros_time_ns / 1e9)
            + f" {DIM}({ros_us}){RESET}"
            + sim_note + age_note
        )

    if autopilot_unix_usec <= 0:
        fcu_text = DIM + "no wall clock yet (needs GPS lock)" + RESET
    else:
        fcu_age = now - last_system_time if last_system_time else None
        age_note = f" {DIM}({fcu_age:.1f}s ago){RESET}" if fcu_age is not None else ""
        fcu_text = (
            format_clock(autopilot_unix_usec / 1e6)
            + f" {DIM}({autopilot_unix_usec}){RESET}"
            + age_note
        )

    boot_note = ""
    if autopilot_boot_ms:
        boot_note = f"   {DIM}boot {autopilot_boot_ms / 1000.0:.1f}s{RESET}"

    return f" TIME:  ROS {ros_text}   FCU {fcu_text}" + boot_note


def draw_dashboard():
    with state.lock:
        connected = state.connected
        vehicle_locked = state.vehicle_locked
        target_system = state.target_system
        target_component = state.target_component

        autopilot_unix_usec = state.autopilot_unix_usec
        autopilot_boot_ms = state.autopilot_boot_ms
        last_system_time = state.last_system_time

        fw_version_text = state.fw_version_text
        fw_git_hash = state.fw_git_hash
        autopilot_version_received = state.autopilot_version_received

        last_preflight_fail = state.last_preflight_fail

        sensors_present = state.sensors_present
        sensors_enabled = state.sensors_enabled
        sensors_health = state.sensors_health

        vibration_x = state.vibration_x
        vibration_y = state.vibration_y
        vibration_z = state.vibration_z
        clipping_0 = state.clipping_0
        clipping_1 = state.clipping_1
        clipping_2 = state.clipping_2
        last_vibration = state.last_vibration

        ros_clock_supported = state.ros_clock_supported
        ros_time_ns = state.ros_time_ns
        ros_time_sim = state.ros_time_sim
        last_ros_time = state.last_ros_time

        armed = state.armed
        mode = state.mode

        battery = state.battery
        voltage = state.voltage
        current = state.current

        local_pos_valid = state.local_pos_valid
        local_x = state.local_x
        local_y = state.local_y
        local_z = state.local_z
        last_local_pos = state.last_local_pos

        global_pos_valid = state.global_pos_valid
        global_lat = state.global_lat
        global_lon = state.global_lon
        global_alt = state.global_alt
        last_gps = state.last_gps

        local_origin_set = state.local_origin_set
        local_origin_lat = state.local_origin_lat
        local_origin_lon = state.local_origin_lon

        home_set = state.home_set

        position_timeout_active = state.position_timeout_active
        gps_timeout_active = state.gps_timeout_active
        ekf_timeout_active = state.ekf_timeout_active
        rc_timeout_active = state.rc_timeout_active

        vx = state.vx
        vy = state.vy
        vz = state.vz

        roll = state.roll
        pitch = state.pitch
        yaw = state.yaw

        roll_rate = state.roll_rate
        pitch_rate = state.pitch_rate
        yaw_rate = state.yaw_rate

        throttle = state.thrust
        att_target_thrust = state.att_target_thrust

        nav_wp_dist = state.nav_wp_dist
        nav_alt_error = state.nav_alt_error
        nav_xtrack_error = state.nav_xtrack_error
        last_nav_output = state.last_nav_output

        groundspeed = state.groundspeed
        climb_rate = state.climb_rate

        alt_amsl = state.alt_amsl
        alt_relative = state.alt_relative
        alt_local = state.alt_local
        alt_terrain = state.alt_terrain
        alt_bottom_clearance = state.alt_bottom_clearance
        last_altitude = state.last_altitude
        vfr_alt = state.vfr_alt

        baro_pressure = state.baro_pressure
        rangefinder_distance = state.rangefinder_distance
        rangefinder_min = state.rangefinder_min
        rangefinder_max = state.rangefinder_max
        rangefinder_quality = state.rangefinder_quality
        rangefinder_orientation = state.rangefinder_orientation
        last_rangefinder = state.last_rangefinder

        gps_fix = state.gps_fix
        gps_sats = state.gps_sats
        gps_hdop = state.gps_hdop
        gps_vdop = state.gps_vdop
        gps_h_acc = state.gps_h_acc

        ekf_status = state.ekf_status
        ekf_flags = state.ekf_flags
        est_is_estimator_status = state.est_is_estimator_status
        est_pos_horiz_accuracy = state.est_pos_horiz_accuracy
        est_pos_vert_accuracy = state.est_pos_vert_accuracy

        rc_received = state.rc_received
        rc_rssi = state.rc_rssi
        rc_lq = state.rc_lq
        rc_failsafe = state.rc_failsafe

        rx_rate = state.rx_rate

        warning_count = state.warning_count
        error_count = state.error_count
        failsafe_count = state.failsafe_count

        pending_arm = state.pending_arm

        last_ack = state.last_ack
        last_status = state.last_status_text
        last_status_level = state.last_status_level

        imu = state.imu
        mag = state.mag
        baro = state.baro
        gps_sensor = state.gps_sensor

        parameter_total = len(state.parameters)
        parameter_changed = 0

        for parameter in state.parameters.values():
            if (
                parameter.changed_from_default is True
                or (parameter.changed_from_default is None and parameter.changed_from_startup)
            ):
                parameter_changed += 1

    width = terminal_size().columns
    now = time.monotonic()

    lines = []

    if connected:
        link_text = GREEN + "CONNECTED" + RESET
    else:
        link_text = RED + "DISCONNECTED" + RESET

    lines.append(
        f" LINK: {link_text}"
        f"    PORT: {settings.port}"
        f"    SYS: {target_system}"
        f"    COMP: {target_component}"
        f"    LOCKED: {'YES' if vehicle_locked else 'NO'}"
    )

    if autopilot_version_received:
        fw_text = fw_version_text
        if fw_git_hash:
            fw_text += f" {DIM}{fw_git_hash}{RESET}"
        lines.append(f" FW: {BOLD}{fw_text}{RESET}")

    lines.append(
        _time_line(
            now, ros_clock_supported, ros_time_ns, ros_time_sim, last_ros_time,
            autopilot_unix_usec, autopilot_boot_ms, last_system_time,
        )
    )

    lines.append(f" MODE: {BOLD}{mode}{RESET}")

    arm_display = state_text(armed)

    if pending_arm is not None:
        if pending_arm.desired_armed:
            arm_display += " " + YELLOW + BOLD + "ARMING..." + RESET
        else:
            arm_display += " " + YELLOW + BOLD + "DISARMING..." + RESET

    lines.append(f" STATE: {arm_display}")

    if last_preflight_fail:
        lines.append(" " + RED + BOLD + "PREARM: " + last_preflight_fail + RESET)

    timed_out = []
    if position_timeout_active:
        timed_out.append("POSITION")
    if gps_timeout_active:
        timed_out.append("GPS")
    if ekf_timeout_active:
        timed_out.append("EKF")
    if rc_timeout_active:
        timed_out.append("RC")

    if timed_out:
        lines.append(
            " " + RED + BOLD + "ALERT: " + ", ".join(timed_out) + " TELEMETRY TIMEOUT" + RESET
        )

    lines.extend(_host_lines())

    lines.append(
        " FLIGHT: " + BOLD + "[a]" + RESET + "arm  " + BOLD + "[d]" + RESET + "disarm  "
        + BOLD + "[T]" + RESET + "takeoff  " + BOLD + "[L]" + RESET + "land  "
        + BOLD + "[R]" + RESET + "RTL  " + BOLD + "[h]" + RESET + "hold  "
        + BOLD + "[m]" + RESET + "mode   " + DIM + "goto/jog on the [n] map" + RESET
    )

    speed_h = math.hypot(vx, vy)
    speed_3d = math.sqrt(vx * vx + vy * vy + vz * vz)

    lines.append("")
    lines.append(ui_section("POSITION / VELOCITY"))

    if local_pos_valid:
        lx, ly, lz = finite(local_x), finite(local_y), finite(local_z)
        local_age = now - last_local_pos if last_local_pos else None
        age_note = f"  {DIM}{local_age:.1f}s ago{RESET}" if local_age is not None else ""
        lines.append(
            f"   LOCAL (EKF):  N {lx:9.3f}   E {ly:9.3f}   D {lz:9.3f} m" + age_note
        )
    else:
        lines.append("   LOCAL (EKF):  " + DIM + "no LOCAL_POSITION_NED / ODOMETRY yet" + RESET)

    gps_local = None
    if global_pos_valid and local_origin_set:
        gn, ge = global_to_local(global_lat, global_lon, local_origin_lat, local_origin_lon)
        if math.isfinite(gn) and math.isfinite(ge):
            gps_local = (gn, ge)

    if gps_local is not None:
        gn, ge = gps_local
        gps_age = now - last_gps if last_gps else None
        age_note = f"  {DIM}{gps_age:.1f}s ago{RESET}" if gps_age is not None else ""
        lines.append(
            f"   GPS (global): N {gn:9.3f}   E {ge:9.3f}   Alt {global_alt:6.2f} m" + age_note
        )
    elif global_pos_valid:
        lines.append(
            "   GPS (global): " + DIM
            + f"{global_lat:.7f}, {global_lon:.7f}  (no local origin - can't grid it)"
            + RESET
        )
    else:
        lines.append("   GPS (global): " + DIM + "no GLOBAL_POSITION_INT yet" + RESET)

    if gps_local is not None and local_pos_valid:
        d_n, d_e = gps_local[0] - lx, gps_local[1] - ly
        dist = math.hypot(d_n, d_e)
        offset_color = GREEN if dist < 0.5 else (YELLOW if dist < 2.0 else RED)
        lines.append(
            f"   EKF <-> GPS offset: {offset_color}{dist:5.2f} m{RESET}"
            f"   (N {d_n:+.2f}  E {d_e:+.2f})"
        )

    lines.append(f"   VX: {vx:9.3f}   VY: {vy:9.3f}   VZ: {vz:9.3f} m/s")
    lines.append(
        f"   |V|: {speed_3d:8.3f} m/s   Horiz: {speed_h:8.3f} m/s"
        f"   Ground: {groundspeed:7.2f} m/s   Climb: {climb_rate:+7.2f} m/s"
    )

    lines.append("")
    lines.append(ui_section("ATTITUDE / CONTROL"))
    lines.append(f"   Roll:  {roll:8.2f} deg   Pitch: {pitch:8.2f} deg   Yaw:   {yaw:8.2f} deg")
    lines.append(
        f"   Rates R/P/Y: {roll_rate:+6.1f} / {pitch_rate:+6.1f} / {yaw_rate:+6.1f} deg/s"
        f"   Throttle: {throttle:5.1f} %   Thrust sp: {att_target_thrust:.2f}"
    )

    nav_line = (
        f"   Nav: WP dist {nav_wp_dist:7.1f} m   Alt err {nav_alt_error:+6.1f} m"
        f"   XTrack {nav_xtrack_error:+6.1f} m"
    )
    if last_nav_output:
        lines.append(nav_line)

    lines.append("")
    lines.append(ui_section("ALTITUDE"))

    amsl_value = alt_amsl if last_altitude else vfr_alt
    agl_value = alt_bottom_clearance or alt_terrain or (
        rangefinder_distance if last_rangefinder else 0.0
    )
    agl_text = f"{agl_value:.2f} m" if agl_value else DIM + "--" + RESET

    hgt_ref = get_param_value("EKF2_HGT_REF")
    if hgt_ref is None:
        ref_text = DIM + "?" + RESET
    else:
        ref_text = EKF2_HGT_REF_NAMES.get(int(round(hgt_ref)), str(int(round(hgt_ref))))

    lines.append(
        f"   AMSL: {amsl_value:8.2f} m   Rel: {alt_relative:8.2f} m   Local: {alt_local:8.2f} m"
    )
    lines.append(
        f"   AGL: {agl_text}   Baro: {baro_pressure:.1f} hPa   Height ref: {ref_text}"
    )

    lines.append("")
    lines.append(ui_section("RANGEFINDER"))

    rng_ctrl = get_param_value("EKF2_RNG_CTRL")
    rf_age = now - last_rangefinder if last_rangefinder else None
    use_text, use_color = rangefinder_status(rng_ctrl, ekf_flags, rangefinder_orientation, rf_age)

    if last_rangefinder:
        rf_q_color = graded_color(
            rangefinder_quality, RANGEFINDER_SIGNAL_GOOD, RANGEFINDER_SIGNAL_OK,
        ) if rangefinder_quality >= 0 else DIM
        rf_q_text = f"{rf_q_color}{rangefinder_quality}%{RESET}" if rangefinder_quality >= 0 else DIM + "n/a" + RESET
        age_note = f"  {DIM}{rf_age:.1f}s ago{RESET}" if rf_age is not None else ""
        lines.append(
            f"   Raw: {rangefinder_distance:.2f} m"
            f"   (range {rangefinder_min:.2f}-{rangefinder_max:.2f} m)"
            f"   Facing: {distance_orientation_text(rangefinder_orientation)}"
            f"   Signal: {rf_q_text}" + age_note
        )
    else:
        lines.append("   " + DIM + "no DISTANCE_SENSOR messages received" + RESET)

    lines.append(f"   Used by EKF: {use_color}{use_text}{RESET}")

    lines.append("")
    lines.append(ui_section("BATTERY"))

    if battery < 0:
        battery_text = "--"
    elif battery <= BATTERY_CRITICAL:
        battery_text = RED + BOLD + f"{battery:.0f}%" + RESET
    elif battery <= BATTERY_LOW:
        battery_text = YELLOW + f"{battery:.0f}%" + RESET
    else:
        battery_text = GREEN + f"{battery:.0f}%" + RESET

    lines.append(f"   Level: {battery_text}   Voltage: {voltage:.2f} V   Current: {current:.2f} A")

    lines.append("")
    lines.append(ui_section("NAVIGATION / ESTIMATION"))

    fix_color, fix_name = gps_fix_display(gps_fix)
    sats_color = graded_color(gps_sats, GPS_SATS_GOOD, GPS_SATS_OK)
    hdop_color = graded_color(gps_hdop, GPS_HDOP_GOOD, GPS_HDOP_OK, higher_is_better=False) if gps_hdop > 0 else DIM
    vdop_color = graded_color(gps_vdop, GPS_HDOP_GOOD, GPS_HDOP_OK, higher_is_better=False) if gps_vdop > 0 else DIM
    eph_color = graded_color(gps_h_acc, POS_ACC_GOOD, POS_ACC_OK, higher_is_better=False) if gps_h_acc > 0 else DIM

    lines.append(
        f"   GPS: {fix_color}{fix_name}{RESET}   Sats: {sats_color}{gps_sats}{RESET}"
        f"   HDOP: {hdop_color}{gps_hdop:.2f}{RESET}   VDOP: {vdop_color}{gps_vdop:.2f}{RESET}"
        f"   EPH: {eph_color}{gps_h_acc:.2f} m{RESET}   RTK: {rtk_text(gps_fix)}"
    )

    ekf_verdict, ekf_color = ekf_summary(ekf_flags, est_is_estimator_status, ekf_status)
    rc_fs = RED + "YES" + RESET if rc_failsafe else GREEN + "NO" + RESET
    home_text = GREEN + "SET" + RESET if home_set else RED + "NOT SET" + RESET

    pos_h_color = graded_color(est_pos_horiz_accuracy, POS_ACC_GOOD, POS_ACC_OK, higher_is_better=False)
    pos_v_color = graded_color(est_pos_vert_accuracy, POS_ACC_GOOD, POS_ACC_OK, higher_is_better=False)

    if not rc_received:
        rc_conn_text = DIM + "NOT CONNECTED" + RESET
    elif rc_timeout_active:
        rc_conn_text = RED + BOLD + "NO SIGNAL" + RESET
    else:
        rc_conn_text = GREEN + "CONNECTED" + RESET

    rssi_color = graded_color(rc_rssi, RC_SIGNAL_GOOD, RC_SIGNAL_OK) if rc_received else DIM
    lq_color = graded_color(rc_lq, RC_SIGNAL_GOOD, RC_SIGNAL_OK) if rc_received else DIM

    lines.append(
        f"   EKF: {ekf_color}{ekf_verdict}{RESET}"
        f"   PosAcc H/V: {pos_h_color}{est_pos_horiz_accuracy:.2f}{RESET}"
        f"/{pos_v_color}{est_pos_vert_accuracy:.2f}{RESET} m"
        f"   Home: {home_text}"
    )
    lines.append(
        f"   RC: {rc_conn_text}"
        f"   RSSI: {rssi_color}{rc_rssi}{RESET}   LQ: {lq_color}{rc_lq}%{RESET}"
        f"   Failsafe: {rc_fs}"
    )

    lines.append("")
    lines.append(ui_section("SENSORS"))

    if sensors_present:
        items = sensor_health_items(sensors_present, sensors_enabled, sensors_health)
        if not items:
            lines.append("   " + DIM + "none reported" + RESET)
        else:
            chunk_size = 6
            for i in range(0, len(items), chunk_size):
                chunk = items[i:i + chunk_size]
                lines.append(
                    "   " + "   ".join(f"{color}{label}{RESET}" for label, color in chunk)
                )
    else:
        def sensor_text(value):
            if value:
                return GREEN + "OK" + RESET
            return RED + "FAIL" + RESET

        lines.append(
            f"   IMU: {sensor_text(imu)}   MAG: {sensor_text(mag)}"
            f"   BARO: {sensor_text(baro)}   GPS: {sensor_text(gps_sensor)}"
        )

    if last_vibration:
        vib_verdict, vib_color = vibration_verdict(
            vibration_x, vibration_y, vibration_z, clipping_0, clipping_1, clipping_2,
        )
        clip_total = clipping_0 + clipping_1 + clipping_2
        clip_note = f"   clipping: {RED}{clip_total}{RESET}" if clip_total else ""
        lines.append(
            f"   Vibration: {vib_color}{vib_verdict}{RESET}"
            f"   X {vibration_x:.1f}  Y {vibration_y:.1f}  Z {vibration_z:.1f} m/s²"
            + clip_note
        )
    else:
        lines.append("   Vibration: " + DIM + "no VIBRATION messages yet" + RESET)

    lines.append("")
    lines.append(ui_section("SYSTEM"))

    rx_color = graded_color(rx_rate, RX_RATE_GOOD, RX_RATE_OK) if connected else DIM
    warn_color = YELLOW if warning_count else DIM
    err_color = RED if error_count else DIM
    fs_color = RED + BOLD if failsafe_count else DIM

    lines.append(
        f"   MAVLink RX: {rx_color}{rx_rate:.0f} msg/s{RESET}"
        f"   Warnings: {warn_color}{warning_count}{RESET}"
        f"   Errors: {err_color}{error_count}{RESET}"
        f"   Failsafes: {fs_color}{failsafe_count}{RESET}"
    )

    if parameter_total:
        lines.append(f"   Parameters: {parameter_total}   Changed: {parameter_changed}")

    if last_ack:
        lines.append(f"   Last ACK: {last_ack}")

    if last_status:
        max_status = max(20, width - 15)
        status_color = level_color(last_status_level) if last_status_level else ""
        lines.append(f"   PX4: {status_color}{last_status[:max_status]}{RESET if status_color else ''}")

    lines.append("")
    lines.append(ui_section("SCREENS"))
    lines.append("   [m] MODE   [s] CALIBRATE   [n] MAP (goto + jog)   [e] ESTIMATION   [c] CONTROL")
    lines.append("   [p] PARAMETERS   [g] EVENT LOG   [l] FLIGHT LOGS   [t] NSH   [u] USB/NETWORK")
    lines.append("   [q] EXIT   [ESC] panels")
    return lines
