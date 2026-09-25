//! The background receiver thread and the per-message telemetry handlers.
//!
//! Each handler folds one decoded message into [`State`] while the caller
//! holds the lock. Mirrors `mavlink/receiver.py` + `mavlink/handlers.py`.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;

use mavlink::dialects::development::{MavMessage, *};

use crate::config::*;
use crate::link::{Link, from_char_array};
use crate::mav::{calibration, flightlog, guided, modes, params, shell};
use crate::px4mode;
use crate::state::{self, Shared, State, classify_statustext, now};

const MAV_AUTOPILOT_INVALID: u32 = 8;
const MAV_AUTOPILOT_ARDUPILOTMEGA: u32 = 3;

/// Run until `shutdown` is set: receive, filter to the locked vehicle,
/// dispatch.
pub fn receiver_thread(
    link: Arc<Link>,
    shared: Shared,
    log_tx: Sender<flightlog::Chunk>,
    shutdown: Arc<AtomicBool>,
) {
    state::lock(&shared).info("MAVLink receiver thread started");
    let mut last_rate_time = now();
    let mut last_rate_count = 0u64;

    while !shutdown.load(Ordering::Relaxed) {
        let frames = match link.recv() {
            Ok(frames) => frames,
            Err(e) => {
                if !shutdown.load(Ordering::Relaxed) {
                    state::lock(&shared).error(format!("MAVLink receive error: {e}"));
                }
                std::thread::sleep(std::time::Duration::from_millis(100));
                continue;
            }
        };

        for (header, msg) in frames {
            // LOG_DATA goes straight to the download worker, never through
            // the shared lock (see mav/flightlog.rs for why).
            if let MavMessage::LOG_DATA(d) = &msg {
                flightlog::route_log_data(&shared, &log_tx, d);
            }

            let mut st = state::lock(&shared);
            if st.vehicle_locked && header.system_id != st.target_system {
                continue;
            }
            st.rx_messages += 1;
            st.last_rx = now();

            let request_modes = dispatch(&mut st, &link, header.system_id, header.component_id, &msg);
            drop(st);
            if request_modes {
                modes::request_available_modes(&link, &shared);
            }
        }

        let t = now();
        if t - last_rate_time >= 1.0 {
            let mut st = state::lock(&shared);
            st.rx_rate = (st.rx_messages - last_rate_count) as f64 / (t - last_rate_time);
            last_rate_count = st.rx_messages;
            last_rate_time = t;
        }
    }
}

/// Returns true when the available-modes list changed and must be
/// re-requested (done by the caller, outside the lock).
fn dispatch(st: &mut State, link: &Link, src_sys: u8, src_comp: u8, msg: &MavMessage) -> bool {
    use MavMessage as M;
    match msg {
        M::HEARTBEAT(d) => heartbeat(st, link, src_sys, src_comp, d),
        M::SYSTEM_TIME(d) => {
            st.last_system_time = now();
            st.autopilot_unix_usec = d.time_unix_usec;
            st.autopilot_boot_ms = d.time_boot_ms;
        }
        M::GLOBAL_POSITION_INT(d) => {
            st.last_position = now();
            st.global_lat = d.lat as f64 / 1e7;
            st.global_lon = d.lon as f64 / 1e7;
            st.global_alt = d.alt as f64 / 1000.0;
            st.global_pos_valid = true;
        }
        M::LOCAL_POSITION_NED(d) => {
            local_position(st, d.x, d.y, d.z, d.vx, d.vy, d.vz);
        }
        M::ODOMETRY(d) => {
            local_position(st, d.x, d.y, d.z, d.vx, d.vy, d.vz);
        }
        M::ATTITUDE(d) => {
            st.roll = (d.roll as f64).to_degrees();
            st.pitch = (d.pitch as f64).to_degrees();
            st.yaw = (d.yaw as f64).to_degrees();
            st.roll_rate = (d.rollspeed as f64).to_degrees();
            st.pitch_rate = (d.pitchspeed as f64).to_degrees();
            st.yaw_rate = (d.yawspeed as f64).to_degrees();
        }
        M::ATTITUDE_TARGET(d) => {
            let (r, p, y) = quaternion_to_euler_deg(d.q);
            st.last_att_target = now();
            st.att_target_roll = r;
            st.att_target_pitch = p;
            st.att_target_yaw = y;
            st.att_target_thrust = finite(d.thrust);
            st.att_target_roll_rate = finite(d.body_roll_rate).to_degrees();
            st.att_target_pitch_rate = finite(d.body_pitch_rate).to_degrees();
            st.att_target_yaw_rate = finite(d.body_yaw_rate).to_degrees();
        }
        M::POSITION_TARGET_LOCAL_NED(d) => {
            st.last_pos_target = now();
            st.pos_target_x = finite(d.x);
            st.pos_target_y = finite(d.y);
            st.pos_target_z = finite(d.z);
            st.pos_target_vx = finite(d.vx);
            st.pos_target_vy = finite(d.vy);
            st.pos_target_vz = finite(d.vz);
            st.pos_target_yaw = finite(d.yaw).to_degrees();
            st.pos_target_yaw_rate = finite(d.yaw_rate).to_degrees();
            st.pos_target_type_mask = d.type_mask.bits();
            st.pos_target_frame = d.coordinate_frame as u32;
        }
        M::NAV_CONTROLLER_OUTPUT(d) => {
            st.last_nav_output = now();
            st.nav_roll = finite(d.nav_roll);
            st.nav_pitch = finite(d.nav_pitch);
            st.nav_bearing = d.nav_bearing as f64;
            st.nav_target_bearing = d.target_bearing as f64;
            st.nav_wp_dist = d.wp_dist as f64;
            st.nav_alt_error = finite(d.alt_error);
            st.nav_aspd_error = finite(d.aspd_error);
            st.nav_xtrack_error = finite(d.xtrack_error);
        }
        M::BATTERY_STATUS(d) => battery_status(st, d),
        M::SYS_STATUS(d) => sys_status(st, d),
        M::GPS_RAW_INT(d) => gps_raw(st, d),
        M::GPS2_RAW(d) => {
            st.last_gps2 = now();
            st.gps2_fix = d.fix_type as i32;
            st.gps2_sats = d.satellites_visible as i32;
            if let Some(h) = gps_dop(d.eph) {
                st.gps2_hdop = h;
            }
            if d.dgps_age < u32::MAX {
                st.gps_dgps_age = d.dgps_age as f64 / 1000.0;
            }
            st.gps_dgps_numch = d.dgps_numch as i32;
        }
        M::GPS_RTK(d) => gps_rtk(st, d.rtk_health, d.rtk_rate, d.nsats, d.iar_num_hypotheses,
            [d.baseline_a_mm, d.baseline_b_mm, d.baseline_c_mm]),
        M::GPS2_RTK(d) => gps_rtk(st, d.rtk_health, d.rtk_rate, d.nsats, d.iar_num_hypotheses,
            [d.baseline_a_mm, d.baseline_b_mm, d.baseline_c_mm]),
        M::GPS_STATUS(d) => {
            st.last_gps = now();
            st.gps_sats = d.satellites_visible as i32;
        }
        M::GPS_GLOBAL_ORIGIN(d) => {
            st.local_origin_set = true;
            st.local_origin_lat = d.latitude as f64 / 1e7;
            st.local_origin_lon = d.longitude as f64 / 1e7;
            st.local_origin_alt = d.altitude as f64 / 1000.0;
        }
        M::HOME_POSITION(d) => {
            st.home_set = true;
            st.home_lat = d.latitude as f64 / 1e7;
            st.home_lon = d.longitude as f64 / 1e7;
            st.home_alt = d.altitude as f64 / 1000.0;
        }
        M::ALTITUDE(d) => {
            st.last_altitude = now();
            st.alt_monotonic = finite(d.altitude_monotonic);
            st.alt_amsl = finite(d.altitude_amsl);
            st.alt_local = finite(d.altitude_local);
            st.alt_relative = finite(d.altitude_relative);
            st.alt_terrain = finite(d.altitude_terrain);
            st.alt_bottom_clearance = finite(d.bottom_clearance);
        }
        M::DISTANCE_SENSOR(d) => {
            let orientation = d.orientation as i32;
            // Prefer the down-facing sensor (it feeds AGL / terrain); only
            // fall back to another orientation if we never saw one.
            if st.rangefinder_orientation == DISTANCE_SENSOR_ORIENTATION_DOWN
                && orientation != DISTANCE_SENSOR_ORIENTATION_DOWN
            {
                return false;
            }
            st.last_rangefinder = now();
            st.rangefinder_orientation = orientation;
            st.rangefinder_distance = d.current_distance as f64 / 100.0;
            st.rangefinder_min = d.min_distance as f64 / 100.0;
            st.rangefinder_max = d.max_distance as f64 / 100.0;
            // signal_quality: 0 = unknown, 1..100 = quality (MAVLink spec).
            st.rangefinder_quality = if d.signal_quality == 0 { -1 } else { d.signal_quality as i32 };
        }
        M::SCALED_PRESSURE(d) => scaled_pressure(st, d.press_abs, d.temperature),
        M::SCALED_PRESSURE2(d) => scaled_pressure(st, d.press_abs, d.temperature),
        M::SCALED_PRESSURE3(d) => scaled_pressure(st, d.press_abs, d.temperature),
        M::VIBRATION(d) => {
            st.last_vibration = now();
            st.vibration_x = finite(d.vibration_x);
            st.vibration_y = finite(d.vibration_y);
            st.vibration_z = finite(d.vibration_z);
            st.clipping = [d.clipping_0, d.clipping_1, d.clipping_2];
        }
        M::WIND_COV(d) => {
            let (wx, wy) = (finite(d.wind_x), finite(d.wind_y));
            st.last_wind = now();
            st.wind_speed = wx.hypot(wy);
            // wind_x/y point where the wind blows TO; report where it's FROM.
            st.wind_direction = (wy.atan2(wx).to_degrees() + 180.0).rem_euclid(360.0);
            st.wind_speed_z = finite(d.wind_z);
        }
        M::SERVO_OUTPUT_RAW(d) => {
            st.last_servo_output = now();
            st.servo_outputs = [
                d.servo1_raw, d.servo2_raw, d.servo3_raw, d.servo4_raw,
                d.servo5_raw, d.servo6_raw, d.servo7_raw, d.servo8_raw,
            ];
        }
        M::FENCE_STATUS(d) => {
            st.last_fence_status = now();
            st.fence_breach_status = d.breach_status;
            st.fence_breach_count = d.breach_count;
        }
        M::AUTOPILOT_VERSION(d) => {
            let v = d.flight_sw_version;
            st.fw_version_text = format!(
                "v{}.{}.{} ({})",
                (v >> 24) & 0xFF,
                (v >> 16) & 0xFF,
                (v >> 8) & 0xFF,
                firmware_version_type_name(v & 0xFF)
            );
            st.fw_git_hash = if d.flight_custom_version.iter().any(|&b| b != 0) {
                d.flight_custom_version.iter().map(|b| format!("{b:02x}")).collect::<String>()[..8].to_string()
            } else {
                String::new()
            };
            st.autopilot_version_received = true;
        }
        M::ESTIMATOR_STATUS(d) => {
            st.last_ekf = now();
            st.est_is_estimator_status = true;
            st.ekf_flags = d.flags.bits() as u32;
            st.ekf_status = "OK".into();
            st.est_vel_ratio = finite(d.vel_ratio);
            st.est_pos_horiz_ratio = finite(d.pos_horiz_ratio);
            st.est_pos_vert_ratio = finite(d.pos_vert_ratio);
            st.est_mag_ratio = finite(d.mag_ratio);
            st.est_hagl_ratio = finite(d.hagl_ratio);
            st.est_tas_ratio = finite(d.tas_ratio);
            st.est_pos_horiz_accuracy = finite(d.pos_horiz_accuracy);
            st.est_pos_vert_accuracy = finite(d.pos_vert_accuracy);
        }
        M::RC_CHANNELS(d) => {
            let ch = [
                d.chan1_raw, d.chan2_raw, d.chan3_raw, d.chan4_raw, d.chan5_raw, d.chan6_raw,
                d.chan7_raw, d.chan8_raw, d.chan9_raw, d.chan10_raw, d.chan11_raw, d.chan12_raw,
                d.chan13_raw, d.chan14_raw, d.chan15_raw, d.chan16_raw, d.chan17_raw, d.chan18_raw,
            ];
            rc(st, d.rssi, &ch);
        }
        M::RC_CHANNELS_RAW(d) => {
            // Only channels 1-8; keep the rest from RC_CHANNELS.
            let ch = [
                d.chan1_raw, d.chan2_raw, d.chan3_raw, d.chan4_raw,
                d.chan5_raw, d.chan6_raw, d.chan7_raw, d.chan8_raw,
            ];
            rc(st, d.rssi, &ch);
        }
        M::VFR_HUD(d) => {
            st.thrust = d.throttle as f64;
            st.groundspeed = finite(d.groundspeed);
            st.airspeed = finite(d.airspeed);
            st.climb_rate = finite(d.climb);
            st.vfr_alt = finite(d.alt);
        }
        M::STATUSTEXT(d) => statustext(st, d.severity as u32, &from_char_array(&d.text[..])),
        M::EVENT(d) => {
            // PX4's structured EVENT interface. Decoding the text needs the
            // vehicle's event metadata, which we don't fetch - but counting
            // them tells the operator PX4 is reporting via events.
            st.event_count += 1;
            if st.cal_active {
                let text = format!("PX4 EVENT id={} (no text available)", d.id);
                st.cal_last_message = text.clone();
                push_bounded(&mut st.cal_log, text, 14);
            }
        }
        M::COMMAND_ACK(d) => command_ack(st, d.command as u32, d.result as u32),
        M::PARAM_VALUE(d) => params::handle_param_value(st, d),
        M::AVAILABLE_MODES(d) => modes::handle_available_modes(st, d),
        M::CURRENT_MODE(d) => {
            st.current_standard_mode = d.standard_mode as u32;
            st.current_custom_mode_id = d.custom_mode;
        }
        M::AVAILABLE_MODES_MONITOR(d) => {
            let seq = d.seq as i32;
            let previous = st.available_modes_seq;
            st.available_modes_seq = seq;
            if previous != -1 && previous != seq {
                st.info("PX4 available modes changed; refreshing");
                return true;
            }
        }
        M::SERIAL_CONTROL(d) => shell::handle_serial_control(st, d),
        M::LOG_ENTRY(d) => flightlog::handle_log_entry(st, d),
        // Deprecated in favour of _INT, but older PX4 still sends it.
        #[allow(deprecated)]
        M::MISSION_REQUEST(d) => guided::handle_mission_request(st, link, d.seq, d.mission_type),
        M::MISSION_REQUEST_INT(d) => guided::handle_mission_request(st, link, d.seq, d.mission_type),
        M::MISSION_ACK(d) => guided::handle_mission_ack(st, d.mavtype, d.mission_type),
        _ => {}
    }
    false
}

fn heartbeat(st: &mut State, link: &Link, src_sys: u8, src_comp: u8, d: &HEARTBEAT_DATA) {
    let autopilot = d.autopilot as u32;
    if autopilot == MAV_AUTOPILOT_INVALID {
        // Another GCS / companion component, not the vehicle.
        return;
    }
    if st.vehicle_locked && src_sys != st.target_system {
        return;
    }
    if !st.vehicle_locked {
        st.target_system = src_sys;
        st.target_component = src_comp;
        st.vehicle_locked = true;
        st.connected = true;
        link.set_target(src_sys, src_comp);
        st.info(format!("PX4 vehicle locked: system={src_sys}, component={src_comp}"));
    }

    let base_mode = d.base_mode.bits();
    let armed_now = base_mode & px4mode::SAFETY_ARMED != 0;
    let mut mode_now = px4mode::mode_string(autopilot, base_mode, d.custom_mode);

    let previous_armed = st.armed;
    let previous_mode = st.mode.clone();

    st.last_heartbeat = now();
    st.base_mode = base_mode;
    st.custom_mode = d.custom_mode;
    st.system_status = d.system_status as u8;
    st.autopilot = autopilot as i32;

    if mode_now == "UNKNOWN"
        && let Some(entry) = st.custom_modes.values().find(|m| m.custom_mode == d.custom_mode) {
            mode_now = entry.name.clone();
        }

    st.armed = armed_now;
    st.mode = mode_now.clone();

    if armed_now && !previous_armed {
        st.last_preflight_fail.clear();
    }

    if let Some(pending) = &st.pending_arm
        && armed_now == pending.desired_armed {
            let action = if pending.desired_armed { "ARM" } else { "DISARM" };
            st.pending_arm = None;
            st.command(format!("{action} CONFIRMED by HEARTBEAT"));
        }

    if armed_now != previous_armed {
        st.command(if armed_now {
            "HEARTBEAT: VEHICLE IS ARMED"
        } else {
            "HEARTBEAT: VEHICLE IS DISARMED"
        });
    }
    if mode_now != previous_mode {
        st.info(format!("PX4 MODE: {previous_mode} -> {mode_now}"));
    }
}

fn local_position(st: &mut State, x: f32, y: f32, z: f32, vx: f32, vy: f32, vz: f32) {
    let t = now();
    st.last_position = t;
    st.x = x as f64;
    st.y = y as f64;
    st.z = z as f64;
    st.vx = finite(vx);
    st.vy = finite(vy);
    st.vz = finite(vz);

    if x.is_finite() && y.is_finite() {
        st.local_x = x as f64;
        st.local_y = y as f64;
        st.local_z = finite(z);
        st.local_pos_valid = true;
        st.last_local_pos = t;
        st.trail_push(x as f64, y as f64);
    }
}

fn battery_remaining(v: i8) -> f64 {
    if (0..=100).contains(&v) { v as f64 } else { -1.0 }
}

fn battery_current_a(v: i16) -> f64 {
    // centi-amps; -1 = not measured.
    if v >= 0 { v as f64 / 100.0 } else { -1.0 }
}

fn battery_status(st: &mut State, d: &BATTERY_STATUS_DATA) {
    // Follow a single pack (the lowest id seen).
    let id = d.id as i32;
    if st.battery_id < 0 || id < st.battery_id {
        st.battery_id = id;
    }
    if id != st.battery_id {
        return;
    }
    st.battery = battery_remaining(d.battery_remaining);
    // No pack voltage field: sum the cells (UINT16_MAX = unused; the ext
    // cells use 0 as unused too).
    let total_mv: f64 = d.voltages.iter().filter(|&&v| v != u16::MAX).map(|&v| v as f64).sum::<f64>()
        + d.voltages_ext.iter().filter(|&&v| v != 0 && v != u16::MAX).map(|&v| v as f64).sum::<f64>();
    st.voltage = if total_mv > 0.0 { total_mv / 1000.0 } else { -1.0 };
    st.current = battery_current_a(d.current_battery);
}

fn sys_status(st: &mut State, d: &SYS_STATUS_DATA) {
    // SYS_STATUS only carries the first battery; once BATTERY_STATUS flows
    // it's the richer source, so don't mix the two.
    if st.battery_id < 0 {
        st.battery = battery_remaining(d.battery_remaining);
        let mv = d.voltage_battery;
        st.voltage = if mv > 0 && mv < u16::MAX { mv as f64 / 1000.0 } else { -1.0 };
        st.current = battery_current_a(d.current_battery);
    }
    let health = d.onboard_control_sensors_health.bits();
    st.sensors_present = d.onboard_control_sensors_present.bits();
    st.sensors_enabled = d.onboard_control_sensors_enabled.bits();
    st.sensors_health = health;

    st.imu = health & 1 != 0;
    st.mag = health & 4 != 0;
    st.baro = health & 8 != 0;
    st.gps_sensor = health & 32 != 0;

    // MAVLink has no RC failsafe flag; it shows as the RC receiver being
    // present and enabled but unhealthy.
    let rc_bit = 65536;
    st.rc_failsafe = st.sensors_present & st.sensors_enabled & rc_bit != 0 && health & rc_bit == 0;
}

fn gps_dop(v: u16) -> Option<f64> {
    (v > 0 && v < u16::MAX).then(|| v as f64 / 100.0)
}

fn gps_acc_m(v: u32) -> Option<f64> {
    (v > 0 && v < u32::MAX).then(|| v as f64 / 1000.0)
}

fn gps_raw(st: &mut State, d: &GPS_RAW_INT_DATA) {
    st.last_gps = now();
    st.gps_fix = d.fix_type as i32;
    st.gps_sats = d.satellites_visible as i32;
    if let Some(v) = gps_dop(d.eph) {
        st.gps_hdop = v;
    }
    if let Some(v) = gps_dop(d.epv) {
        st.gps_vdop = v;
    }
    if d.alt != 0 {
        st.gps_alt = d.alt as f64 / 1000.0;
    }
    if let Some(v) = gps_acc_m(d.h_acc) {
        st.gps_h_acc = v;
    }
    if let Some(v) = gps_acc_m(d.v_acc) {
        st.gps_v_acc = v;
    }
    if let Some(v) = gps_acc_m(d.vel_acc) {
        st.gps_vel_acc = v;
    }
    if d.vel < u16::MAX {
        st.gps_speed = d.vel as f64 / 100.0;
    }
    if d.cog < u16::MAX {
        st.gps_cog = d.cog as f64 / 100.0;
    }
    if d.alt_ellipsoid != 0 {
        st.gps_alt_ellipsoid = d.alt_ellipsoid as f64 / 1000.0;
    }
    // yaw == 0: no dual-antenna heading (PX4 sends 36000 for true north).
    if d.yaw > 0 {
        st.gps_heading = d.yaw as f64 / 100.0;
        if d.hdg_acc > 0 {
            st.gps_heading_acc = d.hdg_acc as f64 / 1e5;
        }
    } else {
        st.gps_heading = -1.0;
    }
}

fn gps_rtk(st: &mut State, health: u8, rate: u8, nsats: u8, iar: i32, baseline_mm: [i32; 3]) {
    st.last_gps_rtk = now();
    st.rtk_health = health as i32;
    st.rtk_rate = rate as f64;
    st.rtk_nsats = nsats as i32;
    st.rtk_iar_hypotheses = iar;
    let [a, b, c] = baseline_mm.map(|v| v as f64);
    st.rtk_baseline_m = (a * a + b * b + c * c).sqrt() / 1000.0;
}

fn scaled_pressure(st: &mut State, press_abs: f32, temperature: i16) {
    st.last_baro = now();
    if press_abs > 0.0 {
        st.baro_pressure = press_abs as f64;
    }
    st.baro_temp = temperature as f64 / 100.0;
}

fn rc(st: &mut State, rssi: u8, channels: &[u16]) {
    st.last_rc = now();
    st.rc_received = true;
    // UINT8_MAX = unknown. PX4 already sends 0-100 %; the spec (and
    // ArduPilot) use 0-254, so rescale that.
    st.rc_rssi = if rssi == u8::MAX {
        -1
    } else if st.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA as i32 {
        (rssi as f64 * 100.0 / 254.0).round() as i32
    } else {
        (rssi as i32).min(100)
    };
    for (dst, &src) in st.rc_channels.iter_mut().zip(channels) {
        *dst = src;
    }
}

fn statustext(st: &mut State, severity: u32, text: &str) {
    let level = classify_statustext(severity, text);
    let lower = text.trim_start().to_lowercase();
    let is_preflight_fail =
        lower.contains("preflight fail") || lower.contains("prearm") || lower.contains("arming denied");

    st.last_status_text = text.to_string();
    st.last_status_level = Some(level);
    st.statustext_count += 1;

    if is_preflight_fail {
        st.last_preflight_fail = text.to_string();
        st.last_preflight_fail_time = now();
    }

    if lower.starts_with("[cal]") || (st.cal_active && lower.contains("calibrat")) {
        calibration::handle_calibration_text(st, text);
    }

    st.add_log(level, format!("PX4: {text}"));
}

fn command_ack(st: &mut State, command: u32, result: u32) {
    let command_name = command_name(command);
    let result_name = ack_name(result);
    st.last_ack = format!("{command_name}: {result_name}");

    // Stream-rate tweaks and one-shot requests are best-effort: a vehicle
    // without a second GPS or a rangefinder rejects some, which is expected.
    if command == 511 || command == 512 {
        return;
    }

    if command == 241 {
        st.cal_ack = result_name.clone();
        if result == 0 || result == 5 {
            st.command(if result == 0 { "Calibration ACCEPTED" } else { "Calibration IN PROGRESS" });
        } else {
            st.cal_active = false;
            st.cal_result = result_name.clone();
            st.error(format!("Calibration rejected by PX4: {result_name}"));
        }
        return;
    }

    let prefix = match command {
        400 => "COMMAND_ACK 400".to_string(),
        246 => "PX4 REBOOT".to_string(),
        _ => command_name,
    };

    match result {
        0 | 5 => {
            let what = if result == 0 { "ACCEPTED" } else { "IN_PROGRESS" };
            if command == 400 {
                st.command(format!("{prefix}: {what} - waiting for HEARTBEAT confirmation"));
            } else {
                st.command(format!("{prefix}: {what}"));
            }
            return;
        }
        1 | 6 => st.warn(format!("{prefix}: {result_name}")),
        2..=4 => st.error(format!("{prefix}: {result_name}")),
        _ => st.warn(format!("{prefix}: {result_name}")),
    }

    // Any final non-success answer to ARM/DISARM ends the pending request.
    if command == 400 {
        st.pending_arm = None;
    }
}

pub fn quaternion_to_euler_deg(q: [f32; 4]) -> (f64, f64, f64) {
    let [w, x, y, z] = q.map(finite);
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    (roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
}

/// PX4 sends NaN in fields it isn't populating; anything headed for a
/// fixed-width format must be finite first.
pub fn finite(v: f32) -> f64 {
    if v.is_finite() { v as f64 } else { 0.0 }
}

pub fn push_bounded<T>(deque: &mut std::collections::VecDeque<T>, item: T, max: usize) {
    if deque.len() >= max {
        deque.pop_front();
    }
    deque.push_back(item);
}

