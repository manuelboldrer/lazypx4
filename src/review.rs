//! The plots PX4 Flight Review shows, as ready-made pages for a ULog
//! (ported from lazyjagg's `review.rs`). Topics and fields changed across
//! PX4 releases, so each curve lists its names newest first and a plot only
//! appears if it has data.

use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;

use crate::ulog::{Meta, Series};

const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;

/// Catppuccin Macchiato curve colours, as in lazyjagg.
pub const PALETTE: [u32; 10] = [0xa6da95, 0x8aadf4, 0xf5a97f, 0xc6a0f6, 0xed8796, 0x8bd5ca, 0xeed49f, 0xf5bde6, 0x91d7e3, 0xb7bdf8];
const RED: u32 = 0xed8796;
const GREEN: u32 = 0xa6da95;
const YELLOW: u32 = 0xeed49f;

// ---------------------------------------------------------------------
// Store and plot model
// ---------------------------------------------------------------------

#[derive(Default)]
pub struct Store {
    pub series: BTreeMap<String, Arc<Series>>,
    pub meta: Meta,
    /// Series whose "time" is an index (correlation curves): not timed.
    pub aux: HashSet<String>,
    /// Headline of the measured EV delay, once computed.
    pub ev_delay_title: Option<String>,
}

impl Store {
    pub fn get(&self, name: &str) -> Option<&Arc<Series>> {
        self.series.get(name)
    }

    pub fn insert(&mut self, s: Series) {
        self.series.insert(s.name.clone(), Arc::new(s));
    }

    /// First and last sample over every timed series.
    pub fn time_span(&self) -> Option<(f64, f64)> {
        let mut span: Option<(f64, f64)> = None;
        for s in self.series.values().filter(|s| !self.aux.contains(&s.name)) {
            if let (Some(&a), Some(&b)) = (s.t.first(), s.t.last()) {
                span = Some(span.map_or((a, b), |(x, y)| (x.min(a), y.max(b))));
            }
        }
        span
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    Time,
    /// x, y pairs.
    Xy,
    /// One lane per series, filled while it is non-zero.
    Flags,
    /// Magnitude spectrum of the visible window.
    Fft,
}

#[derive(Clone, Debug)]
pub struct Curve {
    /// The Y series (already scaled).
    pub name: String,
    /// XY plots: the X series.
    pub x: Option<String>,
    pub label: String,
    pub color: u32,
}

#[derive(Clone, Debug)]
pub struct Plot {
    pub title: String,
    pub mode: Mode,
    pub steps: bool,
    /// Same scale on both axes (XY tracks).
    pub equal: bool,
    pub curves: Vec<Curve>,
}

impl Plot {
    fn next_color(&self) -> u32 {
        let used = |c: u32| self.curves.iter().filter(|k| k.color == c).count();
        PALETTE.iter().copied().min_by_key(|c| used(*c)).unwrap_or(PALETTE[0])
    }
}

pub struct Page {
    pub name: &'static str,
    pub plots: Vec<Plot>,
}

// ---------------------------------------------------------------------
// Page specs
// ---------------------------------------------------------------------

struct Line {
    names: Vec<String>,
    label: &'static str,
    scale: Option<f64>,
    color: Option<u32>,
    /// Lane name when it comes from the log itself.
    field: Option<String>,
}

/// `"topic/field"` alternatives.
fn line(label: &'static str, names: &[&str]) -> Line {
    Line { names: names.iter().map(|s| s.to_string()).collect(), label, scale: None, color: None, field: None }
}

fn scaled(label: &'static str, scale: f64, names: &[&str]) -> Line {
    Line { scale: Some(scale), ..line(label, names) }
}

fn named(name: String, label: &'static str) -> Line {
    Line { names: vec![name], label, scale: None, color: None, field: None }
}

fn lane(label: &'static str, color: u32, names: &[&str]) -> Line {
    Line { color: Some(color), ..line(label, names) }
}

struct Spec {
    title: String,
    mode: Mode,
    steps: bool,
    lines: Vec<Line>,
}

fn time(title: impl Into<String>, lines: Vec<Line>) -> Spec {
    Spec { title: title.into(), mode: Mode::Time, steps: false, lines }
}

fn steps(title: impl Into<String>, lines: Vec<Line>) -> Spec {
    Spec { steps: true, ..time(title, lines) }
}

fn flags(title: impl Into<String>, lines: Vec<Line>) -> Spec {
    Spec { mode: Mode::Flags, ..time(title, lines) }
}

fn fft(title: impl Into<String>, lines: Vec<Line>) -> Spec {
    Spec { mode: Mode::Fft, ..time(title, lines) }
}

fn xy(title: impl Into<String>, lines: Vec<Line>) -> Spec {
    Spec { mode: Mode::Xy, ..time(title, lines) }
}

fn primary(store: &Store) -> u32 {
    store.get("/estimator_selector_status/primary_instance").and_then(|s| s.v.last()).map_or(0, |v| *v as u32)
}

/// Where a `topic/field` lives: multi-instance topics carry `.00`, and
/// estimator topics follow the instance EKF selector ended up on.
pub fn resolve(store: &Store, name: &str) -> Option<String> {
    if name.starts_with('/') {
        return store.get(name).is_some_and(|s| !s.is_empty()).then(|| name.to_string());
    }
    let (topic, field) = name.split_once('/')?;
    let primary = if topic.starts_with("estimator_") { primary(store) } else { 0 };
    [format!("/{topic}.{primary:02}/{field}"), format!("/{topic}/{field}"), format!("/{topic}.00/{field}")]
        .into_iter()
        .find(|n| store.get(n).is_some_and(|s| !s.is_empty()))
}

/// Topic prefix of `topic` in this log, for walking its fields.
fn topic_path(store: &Store, topic: &str) -> Option<String> {
    let primary = primary(store);
    [format!("/{topic}.{primary:02}/"), format!("/{topic}/"), format!("/{topic}.00/")]
        .into_iter()
        .find(|p| store.series.range(p.clone()..).next().is_some_and(|(k, _)| k.starts_with(p.as_str())))
}

/// Lanes for the fields of `topic` picked by `keep` that were ever on.
fn lanes_of(store: &Store, topic: &str, color: u32, keep: impl Fn(&str) -> bool) -> Vec<Line> {
    let Some(prefix) = topic_path(store, topic) else { return Vec::new() };
    store
        .series
        .range(prefix.clone()..)
        .take_while(|(k, _)| k.starts_with(&prefix))
        .filter(|(k, s)| keep(&k[prefix.len()..]) && s.labels.is_none() && s.v.iter().any(|v| *v != 0.0 && !v.is_nan()))
        .map(|(k, _)| Line { names: vec![k.clone()], label: "", scale: None, color: Some(color), field: Some(k[prefix.len()..].to_string()) })
        .collect()
}

fn specs(store: &Store) -> Vec<(&'static str, Vec<Spec>)> {
    let mut fail = lanes_of(store, "failsafe_flags", RED, |f| !f.starts_with("mode_req") && f != "timestamp");
    fail.insert(0, lane("FAILSAFE ACTIVE", RED, &["vehicle_status/failsafe"]));
    fail.insert(1, lane("user took over", YELLOW, &["vehicle_status/failsafe_and_user_took_over"]));
    let detector = lanes_of(store, "failure_detector_status", RED, |f| f.starts_with("fd_") || f.ends_with("_mask"));
    let estimator_flags = lanes_of(store, "estimator_status_flags", GREEN, |f| {
        f.starts_with("cs_ev")
            || f.starts_with("cs_gnss")
            || ["cs_baro_hgt", "cs_rng_hgt", "cs_opt_flow", "cs_mag_hdg", "cs_mag_3d", "cs_in_air", "cs_inertial_dead_reckoning", "cs_wind_dead_reckoning"].contains(&f)
    });
    let estimator_bad = lanes_of(store, "estimator_status_flags", RED, |f| {
        (f.starts_with("fs_") || f.starts_with("reject_") || f.contains("_fault") || f.ends_with("_stuck")) && !f.ends_with("_changes")
    });
    let land = lanes_of(store, "vehicle_land_detected", YELLOW, |f| ["landed", "maybe_landed", "ground_contact", "freefall", "in_ground_effect"].contains(&f));
    let saturated = lanes_of(store, "control_allocator_status", RED, |f| f.starts_with("actuator_saturation"));
    let delay_title = store.ev_delay_title.clone().unwrap_or_else(|| "EV delay".to_string());

    let xyz3 = |t: &'static str| [format!("{t}[0]"), format!("{t}[1]"), format!("{t}[2]")];
    let axes = |labels: [&'static str; 3], names: [String; 3], unit: Option<f64>| -> Vec<Line> {
        labels.into_iter().zip(names).map(|(l, n)| Line { names: vec![n], label: l, scale: unit, color: None, field: None }).collect()
    };
    let indexed = |n: usize, f: &dyn Fn(usize) -> String| -> Vec<Line> { (0..n).map(|i| Line { names: vec![f(i)], label: "", scale: None, color: None, field: None }).collect() };
    let xyz = ["x", "y", "z"];
    let imus = ["IMU 0", "IMU 1", "IMU 2"];
    vec![
        (
            "overview",
            vec![
                time("Altitude", vec![
                    line("GPS", &["sensor_gps/altitude_msl_m", "vehicle_gps_position/altitude_msl_m"]),
                    scaled("GPS", 1e-3, &["sensor_gps/alt", "vehicle_gps_position/alt"]),
                    line("baro", &["vehicle_air_data/baro_alt_meter"]),
                    line("fused", &["vehicle_global_position/alt"]),
                    line("setpoint", &["position_setpoint_triplet/current/alt"]),
                ]),
                xy("Position (top view)", vec![line("estimate", &["vehicle_local_position/y"]), line("estimate", &["vehicle_local_position/x"]), line("setpoint", &["vehicle_local_position_setpoint/y"]), line("setpoint", &["vehicle_local_position_setpoint/x"])]),
                steps("Flight mode and arming", vec![line("mode", &["_review/flight_mode"]), line("arming", &["vehicle_status/arming_state"]), line("failsafe", &["vehicle_status/failsafe"])]),
            ],
        ),
        (
            "failsafe",
            vec![
                flags("Failsafe flags that fired", fail),
                flags("Failure detector", detector),
                steps("Flight mode", vec![line("mode", &["_review/flight_mode"]), line("user intention", &["vehicle_status/nav_state_user_intention"])]),
                time("Offboard signal [Hz]", vec![line("offboard_control_mode", &["_review/offboard_rate"]), line("trajectory_setpoint", &["_review/setpoint_rate"])]),
            ],
        ),
        (
            "attitude",
            vec![
                time("Roll angle [deg]", vec![line("estimated", &["vehicle_attitude/roll_deg"]), line("setpoint", &["vehicle_attitude_setpoint/roll_deg"])]),
                time("Pitch angle [deg]", vec![line("estimated", &["vehicle_attitude/pitch_deg"]), line("setpoint", &["vehicle_attitude_setpoint/pitch_deg"])]),
                time("Yaw angle [deg]", vec![line("estimated", &["vehicle_attitude/yaw_deg"]), line("setpoint", &["vehicle_attitude_setpoint/yaw_deg"])]),
            ],
        ),
        (
            "rates",
            vec![
                time("Roll rate [deg/s]", vec![scaled("estimated", RAD_TO_DEG, &["vehicle_angular_velocity/xyz[0]"]), scaled("setpoint", RAD_TO_DEG, &["vehicle_rates_setpoint/roll"])]),
                time("Pitch rate [deg/s]", vec![scaled("estimated", RAD_TO_DEG, &["vehicle_angular_velocity/xyz[1]"]), scaled("setpoint", RAD_TO_DEG, &["vehicle_rates_setpoint/pitch"])]),
                time("Yaw rate [deg/s]", vec![scaled("estimated", RAD_TO_DEG, &["vehicle_angular_velocity/xyz[2]"]), scaled("setpoint", RAD_TO_DEG, &["vehicle_rates_setpoint/yaw"])]),
            ],
        ),
        (
            "position",
            vec![
                time("Local position X [m]", vec![line("estimated", &["vehicle_local_position/x"]), line("setpoint", &["vehicle_local_position_setpoint/x"])]),
                time("Local position Y [m]", vec![line("estimated", &["vehicle_local_position/y"]), line("setpoint", &["vehicle_local_position_setpoint/y"])]),
                time("Local position Z [m]", vec![line("estimated", &["vehicle_local_position/z"]), line("setpoint", &["vehicle_local_position_setpoint/z"])]),
            ],
        ),
        (
            "velocity",
            vec![
                time("Velocity X [m/s]", vec![line("estimated", &["vehicle_local_position/vx"]), line("setpoint", &["vehicle_local_position_setpoint/vx"])]),
                time("Velocity Y [m/s]", vec![line("estimated", &["vehicle_local_position/vy"]), line("setpoint", &["vehicle_local_position_setpoint/vy"])]),
                time("Velocity Z [m/s]", vec![line("estimated", &["vehicle_local_position/vz"]), line("setpoint", &["vehicle_local_position_setpoint/vz"])]),
            ],
        ),
        (
            "control",
            vec![
                time("Manual control inputs", vec![
                    line("roll", &["manual_control_setpoint/roll", "manual_control_setpoint/y"]),
                    line("pitch", &["manual_control_setpoint/pitch", "manual_control_setpoint/x"]),
                    line("yaw", &["manual_control_setpoint/yaw", "manual_control_setpoint/r"]),
                    line("throttle", &["manual_control_setpoint/throttle", "manual_control_setpoint/z"]),
                ]),
                time("Actuator controls", vec![line("roll", &["vehicle_torque_setpoint/xyz[0]"]), line("pitch", &["vehicle_torque_setpoint/xyz[1]"]), line("yaw", &["vehicle_torque_setpoint/xyz[2]"]), line("thrust", &["vehicle_thrust_setpoint/xyz[2]"])]),
                time("Actuator outputs (motors)", indexed(12, &|i| format!("actuator_motors/control[{i}]"))),
                time("Actuator outputs (PWM)", indexed(16, &|i| format!("actuator_outputs/output[{i}]"))),
            ],
        ),
        (
            "sensors",
            vec![
                time("Raw acceleration [m/s2]", axes(["x", "y", "z"], xyz3("sensor_combined/accelerometer_m_s2"), None)),
                time("Raw angular speed [deg/s]", axes(["x", "y", "z"], xyz3("sensor_combined/gyro_rad"), Some(RAD_TO_DEG))),
                time("Magnetic field [gauss]", axes(["x", "y", "z"], xyz3("vehicle_magnetometer/magnetometer_ga"), None)),
                time("Vibration metrics", vec![line("gyro [rad/s]", &["vehicle_imu_status/gyro_vibration_metric"]), line("accel [m/s2]", &["vehicle_imu_status/accel_vibration_metric"]), line("coning", &["vehicle_imu_status/delta_angle_coning_metric"])]),
            ],
        ),
        (
            "spectrum",
            vec![
                fft("Actuator controls FFT", vec![line("roll", &["vehicle_torque_setpoint/xyz[0]"]), line("pitch", &["vehicle_torque_setpoint/xyz[1]"]), line("yaw", &["vehicle_torque_setpoint/xyz[2]"])]),
                fft("Angular rate FFT", vec![line("x", &["sensor_combined/gyro_rad[0]"]), line("y", &["sensor_combined/gyro_rad[1]"]), line("z", &["sensor_combined/gyro_rad[2]"])]),
                fft("Acceleration FFT", vec![line("x", &["sensor_combined/accelerometer_m_s2[0]"]), line("y", &["sensor_combined/accelerometer_m_s2[1]"]), line("z", &["sensor_combined/accelerometer_m_s2[2]"])]),
            ],
        ),
        (
            "ev",
            vec![
                time("EV latency [ms], arrival minus stamp", vec![line("latency", &["_review/ev_latency_ms"]), line("EKF2_EV_DELAY", &["_review/ev_delay_param_ms"])]),
                time("EV arrival gap [ms] and rate", vec![line("gap ms", &["_review/ev_gap_ms"]), line("time since fused ms", &["_review/ev_since_fuse_ms"])]),
                xy(delay_title, vec![line("gyro vs EV rotation", &["_review/ev_delay/lag_ms"]), line("", &["_review/ev_delay/corr_gyro"]), line("velocity vs EV speed", &["_review/ev_delay/lag_ms"]), line("", &["_review/ev_delay/corr_vel"])]),
                flags("EV fusion", {
                    let mut v = vec![
                        lane("ev_pos fused", GREEN, &["estimator_aid_src_ev_pos/fused"]),
                        lane("ev_hgt fused", GREEN, &["estimator_aid_src_ev_hgt/fused"]),
                        lane("ev_yaw fused", GREEN, &["estimator_aid_src_ev_yaw/fused"]),
                        lane("ev_pos rejected", RED, &["estimator_aid_src_ev_pos/innovation_rejected"]),
                        lane("ev_hgt rejected", RED, &["estimator_aid_src_ev_hgt/innovation_rejected"]),
                        lane("ev_yaw rejected", RED, &["estimator_aid_src_ev_yaw/innovation_rejected"]),
                    ];
                    v.retain(|l| l.names.iter().any(|n| resolve(store, n).is_some()));
                    v
                }),
                time("EV innovation [m, rad]", vec![line("pos N", &["estimator_aid_src_ev_pos/innovation[0]"]), line("pos E", &["estimator_aid_src_ev_pos/innovation[1]"]), line("hgt", &["estimator_aid_src_ev_hgt/innovation"]), line("yaw", &["estimator_aid_src_ev_yaw/innovation"])]),
                time("EV test ratio (1 = rejected)", vec![line("pos N", &["estimator_aid_src_ev_pos/test_ratio[0]"]), line("pos E", &["estimator_aid_src_ev_pos/test_ratio[1]"]), line("hgt", &["estimator_aid_src_ev_hgt/test_ratio"]), line("yaw", &["estimator_aid_src_ev_yaw/test_ratio"])]),
                time("EV vs estimate X [m]", vec![line("EV", &["vehicle_visual_odometry/position[0]"]), line("estimate", &["vehicle_local_position/x"])]),
                time("EV vs estimate Y [m]", vec![line("EV", &["vehicle_visual_odometry/position[1]"]), line("estimate", &["vehicle_local_position/y"])]),
                time("EV vs estimate Z [m]", vec![line("EV", &["vehicle_visual_odometry/position[2]"]), line("estimate", &["vehicle_local_position/z"])]),
                time("EV quality and reset", vec![line("quality", &["vehicle_visual_odometry/quality"]), line("resets", &["vehicle_visual_odometry/reset_counter"])]),
            ],
        ),
        (
            "estimator",
            vec![
                flags("Aiding in use", estimator_flags),
                flags("Faults and rejections", estimator_bad),
                time("Test ratios (1 = limit)", vec![line("velocity", &["estimator_status/vel_test_ratio"]), line("position", &["estimator_status/pos_test_ratio"]), line("height", &["estimator_status/hgt_test_ratio"]), line("heading", &["estimator_status/hdg_test_ratio"])]),
                time("Estimated accuracy [m]", vec![line("horizontal", &["estimator_status/pos_horiz_accuracy"]), line("vertical", &["estimator_status/pos_vert_accuracy"]), line("eph", &["vehicle_local_position/eph"]), line("epv", &["vehicle_local_position/epv"])]),
                time("Output tracking error", vec![line("angle", &["estimator_status/output_tracking_error[0]"]), line("velocity", &["estimator_status/output_tracking_error[1]"]), line("position", &["estimator_status/output_tracking_error[2]"])]),
                time("Time slip [s], EKF falling behind", vec![line("time_slip", &["estimator_status/time_slip"])]),
                time("State resets", vec![line("vel NE", &["estimator_status/reset_count_vel_ne"]), line("vel D", &["estimator_status/reset_count_vel_d"]), line("pos NE", &["estimator_status/reset_count_pos_ne"]), line("pos D", &["estimator_status/reset_count_pod_d"]), line("yaw", &["estimator_status/reset_count_quat"])]),
                time("Gyro bias [rad/s]", vec![line("x", &["estimator_sensor_bias/gyro_bias[0]"]), line("y", &["estimator_sensor_bias/gyro_bias[1]"]), line("z", &["estimator_sensor_bias/gyro_bias[2]"])]),
                time("Accel bias [m/s2]", vec![line("x", &["estimator_sensor_bias/accel_bias[0]"]), line("y", &["estimator_sensor_bias/accel_bias[1]"]), line("z", &["estimator_sensor_bias/accel_bias[2]"])]),
                time("IMU disagreement", vec![line("gyro 1", &["sensors_status_imu/gyro_inconsistency_rad_s[1]"]), line("gyro 2", &["sensors_status_imu/gyro_inconsistency_rad_s[2]"]), line("accel 1", &["sensors_status_imu/accel_inconsistency_m_s_s[1]"]), line("accel 2", &["sensors_status_imu/accel_inconsistency_m_s_s[2]"])]),
            ],
        ),
        (
            "tracking",
            vec![
                time("Position error [m], setpoint - estimate", vec![line("x", &["_review/err_x"]), line("y", &["_review/err_y"]), line("z", &["_review/err_z"])]),
                time("Velocity error [m/s]", vec![line("x", &["_review/err_vx"]), line("y", &["_review/err_vy"]), line("z", &["_review/err_vz"])]),
                time("Attitude error [deg]", vec![line("roll", &["_review/err_roll"]), line("pitch", &["_review/err_pitch"]), line("yaw", &["_review/err_yaw"])]),
                time("Rate error [deg/s]", vec![scaled("roll", RAD_TO_DEG, &["_review/err_rollrate"]), scaled("pitch", RAD_TO_DEG, &["_review/err_pitchrate"]), scaled("yaw", RAD_TO_DEG, &["_review/err_yawrate"])]),
            ],
        ),
        (
            "actuators",
            vec![
                time("Rate integrators", vec![line("roll", &["rate_ctrl_status/rollspeed_integ"]), line("pitch", &["rate_ctrl_status/pitchspeed_integ"]), line("yaw", &["rate_ctrl_status/yawspeed_integ"])]),
                time("Unallocated torque", vec![line("roll", &["control_allocator_status/unallocated_torque[0]"]), line("pitch", &["control_allocator_status/unallocated_torque[1]"]), line("yaw", &["control_allocator_status/unallocated_torque[2]"])]),
                time("Unallocated thrust", vec![line("x", &["control_allocator_status/unallocated_thrust[0]"]), line("y", &["control_allocator_status/unallocated_thrust[1]"]), line("z", &["control_allocator_status/unallocated_thrust[2]"])]),
                flags("Actuators at a limit", saturated),
                time("ESC rpm", indexed(8, &|i| format!("esc_status/esc[{i}]/esc_rpm"))),
                time("ESC current [A]", indexed(8, &|i| format!("esc_status/esc[{i}]/esc_current"))),
            ],
        ),
        (
            "link",
            vec![
                flags("Link and landing", {
                    let mut v = vec![lane("RC lost", RED, &["input_rc/rc_lost"]), lane("RC failsafe", RED, &["input_rc/rc_failsafe"]), lane("GCS lost", RED, &["vehicle_status/gcs_connection_lost"])];
                    v.retain(|l| l.names.iter().any(|n| resolve(store, n).is_some_and(|n| store.get(&n).is_some_and(|s| s.v.iter().any(|x| *x != 0.0)))));
                    v.extend(land);
                    v
                }),
                time("RC signal", vec![line("rssi", &["input_rc/rssi"]), line("link quality", &["input_rc/link_quality"]), line("rssi dBm", &["input_rc/rssi_dbm"])]),
                time("Timesync offset jitter [ms]", vec![line("observed - estimated", &["_review/timesync_error_ms"])]),
                time("Timesync round trip [ms]", vec![scaled("round trip", 1e-3, &["timesync_status/round_trip_time"])]),
                time("MAVLink rates [B/s]", vec![line("tx", &["telemetry_status/tx_rate_avg"]), line("rx", &["telemetry_status/rx_rate_avg"])]),
                time("MAVLink loss", vec![line("rx lost %", &["telemetry_status/rx_message_lost_rate"]), line("tx errors", &["telemetry_status/tx_error_rate_avg"]), line("overruns", &["telemetry_status/tx_buffer_overruns"])]),
            ],
        ),
        (
            "imu",
            vec![
                time("Vibration, per IMU (gyro)", (0..3).map(|i| named(format!("vehicle_imu_status.{i:02}/gyro_vibration_metric"), imus[i])).collect()),
                time("Vibration, per IMU (accel)", (0..3).map(|i| named(format!("vehicle_imu_status.{i:02}/accel_vibration_metric"), imus[i])).collect()),
                time("Accel clipping counts", (0..3).map(|i| named(format!("vehicle_imu_status/accel_clipping[{i}]"), xyz[i])).collect()),
                time("Gyro clipping counts", (0..3).map(|i| named(format!("vehicle_imu_status/gyro_clipping[{i}]"), xyz[i])).collect()),
                time("IMU sample rate [Hz]", vec![line("accel", &["vehicle_imu_status/accel_rate_hz"]), line("gyro", &["vehicle_imu_status/gyro_rate_hz"])]),
                time("Sensor errors", vec![line("accel", &["vehicle_imu_status/accel_error_count"]), line("gyro", &["vehicle_imu_status/gyro_error_count"])]),
                time("Filtered acceleration [m/s2]", (0..3).map(|i| named(format!("vehicle_acceleration/xyz[{i}]"), xyz[i])).collect()),
                time("Filtered angular acceleration [rad/s2]", (0..3).map(|i| named(format!("vehicle_angular_velocity/xyz_derivative[{i}]"), xyz[i])).collect()),
            ],
        ),
        (
            "innovations",
            vec![
                time("GPS position [m]", vec![line("N", &["estimator_innovations/gps_hpos[0]"]), line("E", &["estimator_innovations/gps_hpos[1]"]), line("D", &["estimator_innovations/gps_vpos"])]),
                time("GPS velocity [m/s]", vec![line("N", &["estimator_innovations/gps_hvel[0]"]), line("E", &["estimator_innovations/gps_hvel[1]"]), line("D", &["estimator_innovations/gps_vvel"])]),
                time("Baro height [m]", vec![line("innovation", &["estimator_innovations/baro_vpos"]), line("bias", &["estimator_baro_bias/bias"])]),
                time("Heading [rad]", vec![line("innovation", &["estimator_innovations/heading"])]),
                time("Magnetic field [gauss]", (0..3).map(|i| named(format!("estimator_innovations/mag_field[{i}]"), xyz[i])).collect()),
                time("Optical flow [rad/s]", vec![line("x", &["estimator_innovations/flow[0]"]), line("y", &["estimator_innovations/flow[1]"])]),
                time("Range height [m]", vec![line("innovation", &["estimator_innovations/rng_vpos"])]),
                time("Gravity [m/s2]", (0..3).map(|i| named(format!("estimator_innovations/gravity[{i}]"), xyz[i])).collect()),
                time("Test ratios (1 = rejected)", vec![
                    line("gps pos", &["estimator_innovation_test_ratios/gps_hpos[0]"]),
                    line("gps vel", &["estimator_innovation_test_ratios/gps_hvel[0]"]),
                    line("baro", &["estimator_innovation_test_ratios/baro_vpos"]),
                    line("heading", &["estimator_innovation_test_ratios/heading"]),
                    line("mag", &["estimator_innovation_test_ratios/mag_field[0]"]),
                ]),
            ],
        ),
        (
            "motors",
            vec![
                time("Motor spread: max - min command", vec![line("spread", &["_review/motor_spread"]), line("mean", &["_review/motor_mean"])]),
                time("Mean motor offset (imbalance)", (0..8).map(|i| named(format!("_review/motor_offset_{i}"), ["M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8"][i])).collect()),
                time("Thrust [normalized]", vec![line("hover estimate", &["hover_thrust_estimate/hover_thrust"]), line("commanded", &["_review/thrust"])]),
                time("Tilt and speed", vec![line("tilt deg", &["_review/tilt_deg"]), line("h speed m/s", &["_review/h_speed"]), line("v speed m/s", &["vehicle_local_position/vz"])]),
            ],
        ),
        (
            "battery",
            vec![
                time("Voltage [V]", vec![line("pack", &["battery_status/voltage_v"])]),
                time("Current [A]", vec![line("current", &["battery_status/current_a"]), line("average", &["battery_status/current_average_a"])]),
                time("Cell voltage [V]", indexed(14, &|i| format!("battery_status/voltage_cell_v[{i}]"))),
                time("Power [W] and energy used [Wh]", vec![line("power", &["_review/power_w"]), line("energy", &["_review/energy_wh"])]),
            ],
        ),
        (
            "environment",
            vec![
                time("Wind [m/s]", vec![line("north", &["wind/windspeed_north"]), line("east", &["wind/windspeed_east"])]),
                time("Range sensor [m]", vec![line("distance", &["distance_sensor/current_distance"]), line("ground dist", &["vehicle_local_position/dist_bottom"])]),
                time("Optical flow quality", vec![line("quality", &["sensor_optical_flow/quality"]), line("flow quality", &["vehicle_optical_flow/quality"])]),
                time("Magnetic strength [gauss]", vec![line("measured", &["estimator_status/mag_strength_gs"]), line("expected", &["estimator_status/mag_strength_ref_gs"])]),
                time("Magnetic inclination [deg]", vec![line("measured", &["estimator_status/mag_inclination_deg"]), line("expected", &["estimator_status/mag_inclination_ref_deg"])]),
                time("Baro pressure [Pa]", vec![line("pressure", &["vehicle_air_data/baro_pressure_pa"])]),
            ],
        ),
        (
            "system",
            vec![
                time("GPS uncertainty", vec![
                    line("eph [m]", &["sensor_gps/eph", "vehicle_gps_position/eph"]),
                    line("epv [m]", &["sensor_gps/epv", "vehicle_gps_position/epv"]),
                    line("satellites", &["sensor_gps/satellites_used", "vehicle_gps_position/satellites_used"]),
                    line("fix type", &["sensor_gps/fix_type", "vehicle_gps_position/fix_type"]),
                ]),
                time("GPS noise and jamming", vec![line("noise /ms", &["sensor_gps/noise_per_ms", "vehicle_gps_position/noise_per_ms"]), line("jamming", &["sensor_gps/jamming_indicator", "vehicle_gps_position/jamming_indicator"])]),
                time("Power", vec![line("voltage [V]", &["battery_status/voltage_v"]), line("current [A]", &["battery_status/current_a"]), scaled("discharged [Ah]", 1e-3, &["battery_status/discharged_mah"]), line("remaining", &["battery_status/remaining"])]),
                time("Temperature [C]", vec![line("baro", &["sensor_baro/temperature"]), line("accel", &["sensor_accel/temperature"]), line("air", &["vehicle_air_data/ambient_temperature"]), line("battery", &["battery_status/temperature"])]),
                time("CPU and RAM", vec![line("cpu", &["cpuload/load"]), line("ram", &["cpuload/ram_usage"])]),
            ],
        ),
    ]
}

fn dead(store: &Store, name: &str) -> bool {
    store.get(name).is_none_or(|s| s.v.iter().all(|v| *v == 0.0 || !v.is_finite()))
}

fn flat(store: &Store, name: &str) -> bool {
    let Some(s) = store.get(name) else { return true };
    let mut v = s.v.iter().filter(|x| x.is_finite());
    let Some(first) = v.next() else { return true };
    v.all(|x| x == first)
}

/// The scaled copy of `name`, made once and kept in `extra`.
fn scaled_name(store: &Store, extra: &mut Vec<Series>, name: &str, scale: f64) -> String {
    let out = format!("{name}*{scale}");
    if store.get(&out).is_none()
        && !extra.iter().any(|s| s.name == out)
        && let Some(s) = store.get(name)
    {
        extra.push(Series { name: out.clone(), t: s.t.clone(), v: s.v.iter().map(|v| v * scale).collect(), labels: None });
    }
    out
}

/// Short label for a series without one: its path, less the leading `/`.
fn short(name: &str) -> String {
    name.trim_start_matches('/').to_string()
}

fn build_plot(spec: &Spec, store: &Store, extra: &mut Vec<Series>) -> Option<Plot> {
    let mut plot = Plot { title: spec.title.clone(), mode: spec.mode, steps: spec.steps, equal: spec.mode == Mode::Xy, curves: Vec::new() };
    if spec.mode == Mode::Xy {
        // Lines come as x, y pairs.
        for pair in spec.lines.chunks(2) {
            if let [x, y] = pair
                && let (Some(xn), Some(yn)) = (resolve(store, &x.names[0]), resolve(store, &y.names[0]))
            {
                // A correlation curve is a graph, not a track.
                plot.equal = !xn.starts_with("/_review");
                let color = plot.next_color();
                plot.curves.push(Curve { name: yn, x: Some(xn), label: x.label.to_string(), color });
            }
        }
        return (!plot.curves.is_empty()).then_some(plot);
    }
    for l in &spec.lines {
        let Some(name) = l.names.iter().find_map(|n| resolve(store, n)) else { continue };
        // Unused motor and servo channels sit still; Flight Review drops them.
        if l.label.is_empty() && l.field.is_none() && flat(store, &name) {
            continue;
        }
        // A sensor that isn't fitted logs zeros: no use plotting them.
        if spec.mode == Mode::Time && dead(store, &name) {
            continue;
        }
        let color = l.color.unwrap_or_else(|| plot.next_color());
        let label = l.field.clone().unwrap_or_else(|| if l.label.is_empty() { short(&name) } else { l.label.to_string() });
        let name = match l.scale {
            Some(scale) => scaled_name(store, extra, &name, scale),
            None => name,
        };
        plot.curves.push(Curve { name, x: None, label, color });
    }
    (!plot.curves.is_empty()).then_some(plot)
}

/// The pages this log has data for. Adds the scaled copies they plot.
pub fn pages(store: &mut Store) -> Vec<Page> {
    let mut out = Vec::new();
    let mut extra = Vec::new();
    let has_ev = store.get("/vehicle_visual_odometry/timestamp_sample").is_some()
        || resolve(store, "estimator_aid_src_ev_pos/fused").is_some_and(|n| store.get(&n).is_some_and(|s| s.v.iter().any(|v| *v != 0.0)));
    for (name, list) in specs(store) {
        if name == "ev" && !has_ev {
            continue;
        }
        let plots: Vec<Plot> = list.iter().filter_map(|s| build_plot(s, store, &mut extra)).collect();
        if !plots.is_empty() {
            out.push(Page { name, plots });
        }
    }
    for s in extra {
        store.insert(s);
    }
    out
}

/// Does this look like a PX4 log at all?
pub fn available(store: &Store) -> bool {
    resolve(store, "vehicle_status/nav_state").is_some() || resolve(store, "sensor_combined/gyro_rad[0]").is_some()
}

// ---------------------------------------------------------------------
// Derived series
// ---------------------------------------------------------------------

pub const MODES: [&str; 30] = [
    "Manual", "Altitude", "Position", "Mission", "Hold", "Return", "Position slow", "free5", "Altitude cruise", "free3", "Acro", "free2", "Descend", "Termination", "Offboard", "Stabilized", "free1", "Takeoff", "Land", "Follow",
    "Precision land", "Orbit", "VTOL takeoff", "External 1", "External 2", "External 3", "External 4", "External 5", "External 6", "External 7",
];

/// Quaternion [w, x, y, z] to roll, pitch, yaw in radians.
pub fn quat_to_rpy(q: [f64; 4]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw]
}

/// `/topic/q[0..3]` as `/topic/roll_deg`, `pitch_deg`, `yaw_deg`.
fn quat_series(store: &Store, base: &str) -> Vec<Series> {
    let q: Option<Vec<Arc<Series>>> = (0..4).map(|i| store.get(&format!("{base}[{i}]")).cloned()).collect();
    let Some(q) = q else { return Vec::new() };
    let n = q[0].len().min(q[1].len()).min(q[2].len()).min(q[3].len());
    let mut rpy = [Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n)];
    for i in 0..n {
        let angles = quat_to_rpy([q[0].v[i], q[1].v[i], q[2].v[i], q[3].v[i]]);
        for (out, a) in rpy.iter_mut().zip(angles) {
            out.push(a.to_degrees());
        }
    }
    let prefix = base.rsplit_once('/').map_or(base, |b| b.0);
    let t = q[0].t.clone();
    let [r, p, y] = rpy;
    [("roll_deg", r), ("pitch_deg", p), ("yaw_deg", y)].into_iter().map(|(n, v)| Series { name: format!("{prefix}/{n}"), t: t.clone(), v, labels: None }).collect()
}

/// Messages per second over the last second.
fn sample_rate(s: &Series) -> Vec<f64> {
    let mut out = Vec::with_capacity(s.len());
    let mut start = 0;
    for i in 0..s.len() {
        while s.t[i] - s.t[start] > 1.0 {
            start += 1;
        }
        out.push((i - start + 1) as f64);
    }
    out
}

/// `a - b` on a's samples, b read at or before each.
fn difference(store: &Store, name: &str, a: &str, b: &str, wrap_deg: bool) -> Option<Series> {
    let (a, b) = (store.get(&resolve(store, a)?)?, store.get(&resolve(store, b)?)?);
    let mut v = Vec::with_capacity(a.len());
    for i in 0..a.len() {
        let mut d = a.v[i] - b.value_at(a.t[i]).unwrap_or(f64::NAN);
        if wrap_deg {
            d = (d + 180.0).rem_euclid(360.0) - 180.0;
        }
        v.push(d);
    }
    Some(Series { name: name.to_string(), t: a.t.clone(), v, labels: None })
}

/// Rotation speed from quaternion samples, frame independent.
fn rotation_speed(t: &[f64], q: [&[f64]; 4]) -> (Vec<f64>, Vec<f64>) {
    let (mut ts, mut vs) = (Vec::new(), Vec::new());
    for i in 1..t.len() {
        let dt = t[i] - t[i - 1];
        if dt <= 0.0 || dt > 0.5 {
            continue;
        }
        let dot: f64 = (0..4).map(|k| q[k][i] * q[k][i - 1]).sum::<f64>().abs().min(1.0);
        if dot.is_nan() {
            continue;
        }
        ts.push(t[i] - dt / 2.0);
        vs.push(2.0 * dot.acos() / dt);
    }
    (ts, vs)
}

fn resample(t: &[f64], v: &[f64], start: f64, dt: f64, n: usize) -> Vec<f64> {
    let mut out = Vec::with_capacity(n);
    let mut i = 0;
    for k in 0..n {
        let at = start + k as f64 * dt;
        while i + 1 < t.len() && t[i + 1] <= at {
            i += 1;
        }
        let x = if i + 1 < t.len() && t[i + 1] > t[i] {
            let f = ((at - t[i]) / (t[i + 1] - t[i])).clamp(0.0, 1.0);
            v[i] + (v[i + 1] - v[i]) * f
        } else {
            v[i]
        };
        out.push(if x.is_finite() { x } else { 0.0 });
    }
    out
}

const LAG_MIN_MS: i32 = -50;
const LAG_MAX_MS: i32 = 300;
const STEP_MS: i32 = 2;

/// Normalised correlation of `late` against `early` for each lag: a peak at
/// +30 ms means `late` shows what `early` showed 30 ms before.
pub fn correlate(early: (&[f64], &[f64]), late: (&[f64], &[f64])) -> Option<Vec<(f64, f64)>> {
    let dt = STEP_MS as f64 * 1e-3;
    let start = early.0.first()?.max(*late.0.first()?) + 0.4;
    let end = early.0.last()?.min(*late.0.last()?) - 0.4;
    let n = ((end - start) / dt) as usize;
    if n < 500 {
        return None;
    }
    let mut a = resample(late.0, late.1, start, dt, n);
    let mean = a.iter().sum::<f64>() / n as f64;
    a.iter_mut().for_each(|x| *x -= mean);
    let norm_a = a.iter().map(|x| x * x).sum::<f64>().sqrt();
    let mut out = Vec::new();
    for lag in (LAG_MIN_MS..=LAG_MAX_MS).step_by(STEP_MS as usize) {
        let mut b = resample(early.0, early.1, start - lag as f64 * 1e-3, dt, n);
        let mean = b.iter().sum::<f64>() / n as f64;
        b.iter_mut().for_each(|x| *x -= mean);
        let norm_b = b.iter().map(|x| x * x).sum::<f64>().sqrt();
        let c: f64 = a.iter().zip(&b).map(|(x, y)| x * y).sum();
        out.push((lag as f64, c / (norm_a * norm_b).max(1e-12)));
    }
    Some(out)
}

fn peak(curve: &[(f64, f64)]) -> (f64, f64) {
    curve.iter().copied().max_by(|a, b| a.1.total_cmp(&b.1)).unwrap_or((0.0, 0.0))
}

pub fn param(store: &Store, name: &str) -> Option<f64> {
    store.meta.params.iter().rev().find(|(k, _)| k == name)?.1.parse().ok()
}

fn ev_delay(store: &mut Store) {
    let Some(t) = store.get("/vehicle_visual_odometry/timestamp_sample").map(|s| s.v.iter().map(|v| v * 1e-6).collect::<Vec<f64>>()) else { return };
    let q: Option<Vec<_>> = (0..4).map(|i| store.get(&format!("/vehicle_visual_odometry/q[{i}]")).cloned()).collect();
    let mut gyro_curve = None;
    if let Some(q) = q {
        let ev = rotation_speed(&t, [&q[0].v, &q[1].v, &q[2].v, &q[3].v]);
        let g: Option<Vec<_>> = (0..3).map(|i| store.get(&format!("/vehicle_angular_velocity/xyz[{i}]")).cloned()).collect();
        if let Some(g) = g {
            let speed: Vec<f64> = (0..g[0].len()).map(|i| (g[0].v[i].powi(2) + g[1].v[i].powi(2) + g[2].v[i].powi(2)).sqrt()).collect();
            gyro_curve = correlate((&g[0].t, &speed), (&ev.0, &ev.1));
        }
    }
    let p: Option<Vec<_>> = (0..3).map(|i| store.get(&format!("/vehicle_visual_odometry/position[{i}]")).cloned()).collect();
    let v: Option<Vec<_>> = ["vx", "vy", "vz"].iter().map(|f| store.get(&format!("/vehicle_local_position/{f}")).cloned()).collect();
    let mut vel_curve = None;
    if let (Some(p), Some(v)) = (p, v) {
        let (mut ts, mut vs) = (Vec::new(), Vec::new());
        for i in 2..t.len().min(p[0].len()) {
            let dt = t[i] - t[i - 2];
            if dt > 0.0 && dt < 0.5 {
                let d: f64 = (0..3).map(|k| (p[k].v[i] - p[k].v[i - 2]).powi(2)).sum();
                if d.is_finite() {
                    ts.push(t[i - 1]);
                    vs.push(d.sqrt() / dt);
                }
            }
        }
        let speed: Vec<f64> = (0..v[0].len()).map(|i| (v[0].v[i].powi(2) + v[1].v[i].powi(2) + v[2].v[i].powi(2)).sqrt()).collect();
        vel_curve = correlate((&v[0].t, &speed), (&ts, &vs));
    }
    let Some(any) = gyro_curve.as_ref().or(vel_curve.as_ref()) else { return };
    let index: Arc<Vec<f64>> = Arc::new((0..any.len()).map(|i| i as f64).collect());
    let param_text = param(store, "EKF2_EV_DELAY").map_or(String::new(), |p| format!(", EKF2_EV_DELAY {p:.0} ms"));
    let mut parts = Vec::new();
    if let Some(c) = &gyro_curve {
        let (lag, r) = peak(c);
        parts.push(format!("gyro {lag:.0} ms r={r:.2}"));
    }
    if let Some(c) = &vel_curve {
        let (lag, r) = peak(c);
        parts.push(format!("velocity {lag:.0} ms r={r:.2}"));
    }
    let title = format!("EV delay: {}{param_text}", parts.join(", "));
    let mut put = |name: &str, v: Vec<f64>| {
        store.aux.insert(name.to_string());
        store.insert(Series { name: name.to_string(), t: index.clone(), v, labels: None });
    };
    put("/_review/ev_delay/lag_ms", any.iter().map(|p| p.0).collect());
    let curve = gyro_curve.as_ref().unwrap_or(any);
    put("/_review/ev_delay/corr_gyro", curve.iter().map(|p| p.1).collect());
    if let Some(c) = &vel_curve {
        put("/_review/ev_delay/corr_vel", c.iter().map(|p| p.1).collect());
    }
    store.ev_delay_title = Some(title);
}

/// Spread of the motor commands, and each motor against the mean: a motor
/// always above the others points at imbalance or a weak motor.
fn motor_series(store: &Store) -> Vec<Series> {
    let motors: Vec<_> = (0..12).filter_map(|i| store.get(&format!("/actuator_motors/control[{i}]")).cloned()).filter(|s| s.v.iter().any(|v| v.is_finite() && *v != 0.0)).collect();
    let Some(first) = motors.first() else { return Vec::new() };
    let n = motors.iter().map(|m| m.len()).min().unwrap_or(0);
    let mean: Vec<f64> = (0..n).map(|i| motors.iter().map(|m| m.v[i]).sum::<f64>() / motors.len() as f64).collect();
    let spread: Vec<f64> = (0..n).map(|i| motors.iter().map(|m| m.v[i]).fold(f64::MIN, f64::max) - motors.iter().map(|m| m.v[i]).fold(f64::MAX, f64::min)).collect();
    let make = |name: String, v: Vec<f64>| Series { name, t: first.t.clone(), v, labels: None };
    let mut out = vec![make("/_review/motor_mean".into(), mean.clone()), make("/_review/motor_spread".into(), spread)];
    for (k, m) in motors.iter().take(8).enumerate() {
        out.push(make(format!("/_review/motor_offset_{k}"), m.v.iter().zip(&mean).map(|(a, b)| a - b).collect()));
    }
    out
}

fn flight_series(store: &Store) -> Vec<Series> {
    let mut out = Vec::new();
    if let (Some(r), Some(p)) = (store.get("/vehicle_attitude/roll_deg"), store.get("/vehicle_attitude/pitch_deg")) {
        let v = r.v.iter().zip(&p.v).map(|(a, b)| (a.to_radians().cos() * b.to_radians().cos()).clamp(-1.0, 1.0).acos().to_degrees()).collect();
        out.push(Series { name: "/_review/tilt_deg".into(), t: r.t.clone(), v, labels: None });
    }
    if let (Some(x), Some(y)) = (store.get("/vehicle_local_position/vx"), store.get("/vehicle_local_position/vy")) {
        out.push(Series { name: "/_review/h_speed".into(), t: x.t.clone(), v: x.v.iter().zip(&y.v).map(|(a, b)| a.hypot(*b)).collect(), labels: None });
    }
    if let Some(z) = store.get("/vehicle_thrust_setpoint/xyz[2]") {
        out.push(Series { name: "/_review/thrust".into(), t: z.t.clone(), v: z.v.iter().map(|v| -v).collect(), labels: None });
    }
    if let (Some(u), Some(i)) = (resolve(store, "battery_status/voltage_v"), resolve(store, "battery_status/current_a"))
        && let (Some(u), Some(i)) = (store.get(&u), store.get(&i))
    {
        let power: Vec<f64> = u.v.iter().zip(&i.v).map(|(a, b)| a * b).collect();
        let mut sum = 0.0;
        let mut energy = vec![0.0];
        for k in 1..power.len() {
            if power[k].is_finite() {
                sum += power[k] * (u.t[k] - u.t[k - 1]) / 3600.0;
            }
            energy.push(sum);
        }
        energy.truncate(power.len());
        out.push(Series { name: "/_review/power_w".into(), t: u.t.clone(), v: power, labels: None });
        out.push(Series { name: "/_review/energy_wh".into(), t: u.t.clone(), v: energy, labels: None });
    }
    out
}

/// Series the pages need that the log doesn't hold directly.
pub fn derive(store: &mut Store) {
    for base in ["/vehicle_attitude/q", "/vehicle_attitude_setpoint/q_d"] {
        for s in quat_series(store, base) {
            store.insert(s);
        }
    }
    let mut made = Vec::new();
    if let Some(s) = store.get("/vehicle_status/nav_state") {
        made.push(Series { name: "/_review/flight_mode".into(), t: s.t.clone(), v: s.v.clone(), labels: Some(MODES.iter().map(|m| m.to_string()).collect()) });
    }
    for (name, topic) in [("offboard_rate", "/offboard_control_mode/position"), ("setpoint_rate", "/trajectory_setpoint/yaw")] {
        if let Some(s) = store.get(topic) {
            made.push(Series { name: format!("/_review/{name}"), t: s.t.clone(), v: sample_rate(s), labels: None });
        }
    }
    for (n, a, b, wrap) in [
        ("x", "vehicle_local_position_setpoint/x", "vehicle_local_position/x", false),
        ("y", "vehicle_local_position_setpoint/y", "vehicle_local_position/y", false),
        ("z", "vehicle_local_position_setpoint/z", "vehicle_local_position/z", false),
        ("vx", "vehicle_local_position_setpoint/vx", "vehicle_local_position/vx", false),
        ("vy", "vehicle_local_position_setpoint/vy", "vehicle_local_position/vy", false),
        ("vz", "vehicle_local_position_setpoint/vz", "vehicle_local_position/vz", false),
        ("roll", "vehicle_attitude_setpoint/roll_deg", "vehicle_attitude/roll_deg", true),
        ("pitch", "vehicle_attitude_setpoint/pitch_deg", "vehicle_attitude/pitch_deg", true),
        ("yaw", "vehicle_attitude_setpoint/yaw_deg", "vehicle_attitude/yaw_deg", true),
        ("rollrate", "vehicle_rates_setpoint/roll", "vehicle_angular_velocity/xyz[0]", false),
        ("pitchrate", "vehicle_rates_setpoint/pitch", "vehicle_angular_velocity/xyz[1]", false),
        ("yawrate", "vehicle_rates_setpoint/yaw", "vehicle_angular_velocity/xyz[2]", false),
    ] {
        made.extend(difference(store, &format!("/_review/err_{n}"), a, b, wrap));
    }
    if let (Some(o), Some(e)) = (resolve(store, "timesync_status/observed_offset"), resolve(store, "timesync_status/estimated_offset"))
        && let (Some(o), Some(e)) = (store.get(&o), store.get(&e))
    {
        made.push(Series { name: "/_review/timesync_error_ms".into(), t: o.t.clone(), v: o.v.iter().zip(&e.v).map(|(a, b)| (a - b) * 1e-3).collect(), labels: None });
    }
    if let Some(s) = store.get("/vehicle_visual_odometry/timestamp_sample").filter(|s| !s.is_empty()) {
        made.push(Series { name: "/_review/ev_latency_ms".into(), t: s.t.clone(), v: s.t.iter().zip(&s.v).map(|(t, u)| (t - u * 1e-6) * 1e3).collect(), labels: None });
        let mut gap = vec![f64::NAN];
        gap.extend(s.t.windows(2).map(|w| (w[1] - w[0]) * 1e3));
        made.push(Series { name: "/_review/ev_gap_ms".into(), t: s.t.clone(), v: gap, labels: None });
        if let Some(p) = param(store, "EKF2_EV_DELAY") {
            let ends = vec![s.t[0], s.t[s.len() - 1]];
            made.push(Series { name: "/_review/ev_delay_param_ms".into(), t: Arc::new(ends), v: vec![p, p], labels: None });
        }
    }
    if let Some(n) = resolve(store, "estimator_aid_src_ev_pos/time_last_fuse")
        && let Some(s) = store.get(&n)
    {
        made.push(Series {
            name: "/_review/ev_since_fuse_ms".into(),
            t: s.t.clone(),
            v: s.t.iter().zip(&s.v).map(|(t, u)| if *u > 0.0 { (t - u * 1e-6) * 1e3 } else { f64::NAN }).collect(),
            labels: None,
        });
    }
    made.extend(motor_series(store));
    made.extend(flight_series(store));
    for s in made {
        store.insert(s);
    }
    ev_delay(store);
    flag_events(store);
}

/// Each failsafe flag going on or off, and each mode change, in the message
/// list.
fn flag_events(store: &mut Store) {
    if store.meta.messages.iter().any(|m| m.1 == "FLAG") {
        return;
    }
    let mut events = Vec::new();
    for prefix in ["/failsafe_flags/", "/failure_detector_status/fd_"] {
        for (k, s) in store.series.range(prefix.to_string()..).take_while(|(k, _)| k.starts_with(prefix)) {
            let name = k.rsplit('/').next().unwrap_or(k);
            if name.starts_with("mode_req") || name == "timestamp" {
                continue;
            }
            let mut on = s.v.first().is_some_and(|v| *v != 0.0);
            if on {
                events.push((s.t[0], "FLAG".to_string(), format!("{name} ON")));
            }
            for (t, v) in s.t.iter().zip(&s.v) {
                if (*v != 0.0) != on {
                    on = !on;
                    events.push((*t, "FLAG".to_string(), format!("{name} {}", if on { "ON" } else { "off" })));
                }
            }
        }
    }
    if let Some(s) = store.get("/vehicle_status/nav_state").cloned() {
        for i in 1..s.len() {
            if s.v[i] != s.v[i - 1] {
                let m = MODES.get(s.v[i] as usize).copied().unwrap_or("?");
                events.push((s.t[i], "FLAG".to_string(), format!("flight mode {m}")));
            }
        }
    }
    store.meta.messages.extend(events);
    store.meta.messages.sort_by(|a, b| a.0.total_cmp(&b.0));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store(names: &[(&str, Vec<f64>)]) -> Store {
        let mut s = Store::default();
        for (n, v) in names {
            let t = (0..v.len()).map(|i| i as f64).collect();
            s.insert(Series { name: n.to_string(), t: Arc::new(t), v: v.clone(), labels: None });
        }
        s
    }

    #[test]
    fn only_pages_with_data_and_instances_resolve() {
        let mut s = store(&[("/vehicle_local_position/x", vec![1.0, 2.0]), ("/vehicle_local_position/y", vec![1.0, 3.0]), ("/sensor_baro.00/temperature", vec![30.0, 31.0])]);
        let pages = pages(&mut s);
        let names: Vec<&str> = pages.iter().map(|p| p.name).collect();
        assert_eq!(names, vec!["overview", "position", "system"]);
        assert_eq!(pages[2].plots[0].curves[0].name, "/sensor_baro.00/temperature");
        assert_eq!(pages[0].plots[0].mode, Mode::Xy);
    }

    #[test]
    fn older_field_names_are_scaled() {
        let mut s = store(&[("/vehicle_gps_position/alt", vec![5000.0, 6000.0])]);
        let pages = pages(&mut s);
        let c = &pages[0].plots[0].curves[0];
        assert_eq!(s.get(&c.name).unwrap().v, vec![5.0, 6.0]);
    }

    #[test]
    fn idle_channels_are_dropped() {
        let mut s = store(&[("/actuator_motors/control[0]", vec![0.1, 0.5]), ("/actuator_motors/control[1]", vec![f64::NAN, f64::NAN]), ("/actuator_motors/control[2]", vec![0.0, 0.0])]);
        let pages = pages(&mut s);
        assert_eq!(pages[0].plots[0].curves.len(), 1);
    }

    #[test]
    fn correlation_finds_a_known_delay() {
        let t: Vec<f64> = (0..8000).map(|i| i as f64 * 0.002).collect();
        let f = |x: f64| (x * 3.1).sin() * (x * 7.3).cos() + (x * 1.7).sin().powi(3);
        let early: Vec<f64> = t.iter().map(|x| f(*x)).collect();
        let late: Vec<f64> = t.iter().map(|x| f(*x - 0.034)).collect();
        let c = correlate((&t, &early), (&t, &late)).unwrap();
        let (lag, r) = peak(&c);
        assert_eq!(lag, 34.0);
        assert!(r > 0.99);
    }

    #[test]
    fn flags_become_events_and_lanes() {
        let mut s = store(&[
            ("/failsafe_flags/local_position_invalid", vec![0.0, 1.0, 0.0]),
            ("/failsafe_flags/geofence_breached", vec![0.0, 0.0, 0.0]),
            ("/failsafe_flags/mode_req_attitude", vec![1.0, 1.0, 1.0]),
            ("/vehicle_status/failsafe", vec![0.0, 1.0, 1.0]),
            ("/vehicle_status/nav_state", vec![14.0, 14.0, 5.0]),
        ]);
        derive(&mut s);
        derive(&mut s);
        let texts: Vec<&str> = s.meta.messages.iter().map(|m| m.2.as_str()).collect();
        assert_eq!(texts, vec!["local_position_invalid ON", "local_position_invalid off", "flight mode Return"]);
        let pages = pages(&mut s);
        let f = pages.iter().find(|p| p.name == "failsafe").unwrap();
        let names: Vec<&str> = f.plots[0].curves.iter().map(|c| c.label.as_str()).collect();
        assert!(names.contains(&"local_position_invalid"));
        assert!(!names.iter().any(|n| n.contains("geofence") || n.contains("mode_req")));
        assert_eq!(s.get("/_review/flight_mode").unwrap().text_at(9.0).as_deref(), Some("Return"));
    }

    #[test]
    fn quaternion_to_degrees() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let mut s = store(&[("/vehicle_attitude/q[0]", vec![h]), ("/vehicle_attitude/q[1]", vec![0.0]), ("/vehicle_attitude/q[2]", vec![0.0]), ("/vehicle_attitude/q[3]", vec![h])]);
        derive(&mut s);
        assert!((s.get("/vehicle_attitude/yaw_deg").unwrap().v[0] - 90.0).abs() < 1e-9);
    }
}
