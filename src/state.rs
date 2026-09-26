//! The shared, mutable world: everything the MAVLink receiver learns about
//! the vehicle, plus the event log.
//!
//! [`State`] is written from the receiver thread and the flight-log worker
//! and read from the UI thread, so it lives behind one `Arc<Mutex<_>>`
//! ([`Shared`]). UI-only navigation state is in `app::Session` instead and
//! needs no lock. Mirrors `lazypx4/state.py`.

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Instant;

use crate::config::*;

pub type Shared = Arc<Mutex<State>>;

/// Lock the shared state, recovering from a poisoned mutex (a panicking
/// thread must not take the whole console down with it).
pub fn lock(shared: &Shared) -> MutexGuard<'_, State> {
    shared.lock().unwrap_or_else(|e| e.into_inner())
}

static START: OnceLock<Instant> = OnceLock::new();

/// Monotonic seconds. Starts at 1.0 so that 0.0 can keep meaning "never",
/// exactly like the `last_*` timestamps of the Python version.
pub fn now() -> f64 {
    START.get_or_init(Instant::now).elapsed().as_secs_f64() + 1.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Info,
    Command,
    Warn,
    Error,
    Failsafe,
}

impl Level {
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Info => "INFO",
            Level::Command => "COMMAND",
            Level::Warn => "WARN",
            Level::Error => "ERROR",
            Level::Failsafe => "FAILSAFE",
        }
    }
}

#[derive(Debug, Clone)]
pub struct LogEvent {
    /// Local wall-clock time, "HH:MM:SS".
    pub time: String,
    pub level: Level,
    pub message: String,
}

#[derive(Debug, Clone)]
pub struct PendingArm {
    pub desired_armed: bool,
    pub sent_at: f64,
}

#[derive(Debug, Clone)]
pub struct CustomMode {
    pub custom_mode: u32,
    pub standard_mode: u32,
    pub properties: u32,
    pub name: String,
}

#[derive(Debug, Clone)]
pub struct FlightLogEntry {
    pub id: u16,
    pub num_logs: u16,
    pub time_utc: u32,
    pub size: u32,
}

impl FlightLogEntry {
    pub fn time_string(&self) -> String {
        if self.time_utc == 0 {
            return "--".into();
        }
        chrono::DateTime::from_timestamp(self.time_utc as i64, 0)
            .map(|t| t.format("%Y-%m-%d %H:%M:%S UTC").to_string())
            .unwrap_or_else(|| "--".into())
    }

    pub fn size_string(&self) -> String {
        size_string(self.size as u64)
    }
}

pub fn size_string(value: u64) -> String {
    if value >= 1024 * 1024 {
        format!("{:.1} MiB", value as f64 / (1024.0 * 1024.0))
    } else if value >= 1024 {
        format!("{:.1} KiB", value as f64 / 1024.0)
    } else {
        format!("{value} B")
    }
}

/// One waypoint-queue / coverage target.
#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    pub lat: f64,
    pub lon: f64,
    pub name: String,
}

#[derive(Debug, Clone)]
pub struct Parameter {
    pub name: String,
    pub value: f64,
    pub param_type: u8,
    pub index: i32,
    pub startup_value: f64,
    pub default_value: Option<f64>,
    pub pending: bool,
}

impl Parameter {
    pub fn changed_from_startup(&self) -> bool {
        (self.value - self.startup_value).abs() > PARAM_VALUE_EPSILON
    }

    pub fn changed_from_default(&self) -> Option<bool> {
        self.default_value
            .map(|d| (self.value - d).abs() > PARAM_VALUE_EPSILON)
    }

    /// The CHANGED view: differs from the firmware default when known, else
    /// from the value first read at start-up.
    pub fn is_changed(&self) -> bool {
        match self.changed_from_default() {
            Some(changed) => changed,
            None => self.changed_from_startup(),
        }
    }
}

#[derive(Debug, Default)]
pub struct State {
    pub target_system: u8,
    pub target_component: u8,
    pub vehicle_locked: bool,
    /// Set once streams have been configured for the locked vehicle.
    pub streams_configured: bool,

    pub connected: bool,
    pub last_heartbeat: f64,
    pub last_rx: f64,

    pub autopilot_unix_usec: u64,
    pub autopilot_boot_ms: u32,
    pub last_system_time: f64,

    pub armed: bool,
    pub mode: String,
    pub base_mode: u8,
    pub custom_mode: u32,
    pub system_status: u8,
    /// MAV_AUTOPILOT_* from HEARTBEAT; -1 until one arrives.
    pub autopilot: i32,

    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub last_position: f64,

    pub local_x: f64,
    pub local_y: f64,
    pub local_z: f64,
    pub local_pos_valid: bool,
    pub last_local_pos: f64,
    pub position_trail: VecDeque<(f64, f64)>,

    pub global_lat: f64,
    pub global_lon: f64,
    pub global_alt: f64,
    pub global_pos_valid: bool,

    // Sensor calibration.
    pub cal_active: bool,
    pub cal_type: String,
    pub cal_progress: i32,
    pub cal_result: String,
    pub cal_last_message: String,
    pub cal_started_at: f64,
    pub cal_sent_at: f64,
    pub cal_ack: String,
    pub cal_log: VecDeque<String>,

    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub roll_rate: f64,
    pub pitch_rate: f64,
    pub yaw_rate: f64,

    /// -1 means "unknown / not reported" for all three.
    pub battery: f64,
    pub voltage: f64,
    pub current: f64,
    /// BATTERY_STATUS id being followed; -1 until one is seen.
    pub battery_id: i32,

    pub thrust: f64,
    pub groundspeed: f64,
    pub airspeed: f64,
    pub climb_rate: f64,

    pub att_target_roll: f64,
    pub att_target_pitch: f64,
    pub att_target_yaw: f64,
    pub att_target_thrust: f64,
    pub att_target_roll_rate: f64,
    pub att_target_pitch_rate: f64,
    pub att_target_yaw_rate: f64,
    pub last_att_target: f64,

    pub pos_target_x: f64,
    pub pos_target_y: f64,
    pub pos_target_z: f64,
    pub pos_target_vx: f64,
    pub pos_target_vy: f64,
    pub pos_target_vz: f64,
    pub pos_target_yaw: f64,
    pub pos_target_yaw_rate: f64,
    pub pos_target_type_mask: u16,
    pub pos_target_frame: u32,
    pub last_pos_target: f64,

    pub nav_roll: f64,
    pub nav_pitch: f64,
    pub nav_bearing: f64,
    pub nav_target_bearing: f64,
    pub nav_wp_dist: f64,
    pub nav_alt_error: f64,
    pub nav_aspd_error: f64,
    pub nav_xtrack_error: f64,
    pub last_nav_output: f64,

    pub alt_amsl: f64,
    pub alt_relative: f64,
    pub alt_local: f64,
    pub alt_monotonic: f64,
    pub alt_terrain: f64,
    pub alt_bottom_clearance: f64,
    pub last_altitude: f64,
    pub vfr_alt: f64,

    pub baro_pressure: f64,
    pub baro_temp: f64,
    pub last_baro: f64,

    pub rangefinder_distance: f64,
    pub rangefinder_min: f64,
    pub rangefinder_max: f64,
    pub rangefinder_quality: i32,
    pub rangefinder_orientation: i32,
    pub last_rangefinder: f64,

    pub local_origin_set: bool,
    pub local_origin_lat: f64,
    pub local_origin_lon: f64,
    pub local_origin_alt: f64,

    pub home_set: bool,
    pub home_lat: f64,
    pub home_lon: f64,
    pub home_alt: f64,

    pub gps_fix: i32,
    pub gps_sats: i32,
    pub gps_hdop: f64,
    pub gps_vdop: f64,
    pub gps_alt: f64,
    pub gps_h_acc: f64,
    pub gps_v_acc: f64,
    pub gps_vel_acc: f64,
    pub gps_speed: f64,
    pub gps_cog: f64,
    pub gps_alt_ellipsoid: f64,
    /// Dual-antenna heading; -1.0 on single-antenna receivers.
    pub gps_heading: f64,
    pub gps_heading_acc: f64,
    pub last_gps: f64,

    pub gps2_fix: i32,
    pub gps2_sats: i32,
    pub gps2_hdop: f64,
    pub gps_dgps_age: f64,
    pub gps_dgps_numch: i32,
    pub last_gps2: f64,

    pub rtk_health: i32,
    pub rtk_rate: f64,
    pub rtk_nsats: i32,
    pub rtk_baseline_m: f64,
    pub rtk_iar_hypotheses: i32,
    pub last_gps_rtk: f64,

    pub ekf_flags: u32,
    pub ekf_status: String,
    pub last_ekf: f64,

    pub est_is_estimator_status: bool,
    pub est_vel_ratio: f64,
    pub est_pos_horiz_ratio: f64,
    pub est_pos_vert_ratio: f64,
    pub est_mag_ratio: f64,
    pub est_hagl_ratio: f64,
    pub est_tas_ratio: f64,
    pub est_pos_horiz_accuracy: f64,
    pub est_pos_vert_accuracy: f64,

    pub rc_received: bool,
    /// Percent, -1 when the receiver doesn't report it.
    pub rc_rssi: i32,
    pub rc_failsafe: bool,
    pub rc_channels: [u16; 18],
    pub last_rc: f64,

    pub imu: bool,
    pub mag: bool,
    pub baro: bool,
    pub gps_sensor: bool,
    pub sensors_present: u32,
    pub sensors_enabled: u32,
    pub sensors_health: u32,

    pub vibration_x: f64,
    pub vibration_y: f64,
    pub vibration_z: f64,
    pub clipping: [u32; 3],
    pub last_vibration: f64,

    /// Direction the wind blows FROM, degrees.
    pub wind_speed: f64,
    pub wind_direction: f64,
    pub wind_speed_z: f64,
    pub last_wind: f64,

    pub servo_outputs: [u16; 8],
    pub last_servo_output: f64,

    pub fence_breach_status: u8,
    pub fence_breach_count: u16,
    pub last_fence_status: f64,

    pub fw_version_text: String,
    pub fw_git_hash: String,
    pub autopilot_version_received: bool,

    pub last_preflight_fail: String,
    pub last_preflight_fail_time: f64,

    pub rx_messages: u64,
    pub rx_rate: f64,
    pub statustext_count: u64,
    pub event_count: u64,

    pub events: VecDeque<LogEvent>,
    pub warning_count: u32,
    pub error_count: u32,
    pub failsafe_count: u32,

    pub last_ack: String,
    pub last_status_text: String,
    pub last_status_level: Option<Level>,

    pub pending_arm: Option<PendingArm>,

    pub heartbeat_timeout_active: bool,
    pub position_timeout_active: bool,
    pub gps_timeout_active: bool,
    pub ekf_timeout_active: bool,
    pub rc_timeout_active: bool,
    pub battery_low_active: bool,
    pub battery_critical_active: bool,

    // Parameters.
    pub parameters: HashMap<String, Parameter>,
    /// Names sorted case-insensitively.
    pub parameter_order: Vec<String>,
    pub parameter_count: u32,
    pub parameters_requested_at: f64,
    pub parameters_complete: bool,
    pub parameter_defaults: HashMap<String, f64>,
    pub parameter_full_list_requested: bool,
    pub parameter_set_pending: Option<(String, f64, f64)>,

    // Standard Modes Protocol (AVAILABLE_MODES), keyed by mode_index.
    pub custom_modes: BTreeMap<u8, CustomMode>,
    pub custom_modes_total: u32,
    pub custom_modes_requested_at: f64,
    pub custom_modes_complete: bool,
    pub available_modes_seq: i32,
    pub current_standard_mode: u32,
    pub current_custom_mode_id: u32,

    // MAVLink shell.
    pub shell_active: bool,
    pub shell_lines: VecDeque<String>,
    pub shell_line: crate::mav::shell::ShellLine,
    /// The command just sent, while its echo is still expected back.
    pub shell_echo: Option<crate::mav::shell::EchoFilter>,

    // Flight logs.
    pub flight_logs: BTreeMap<u16, FlightLogEntry>,
    pub flight_log_list_requested_at: f64,
    pub flight_log_list_last_entry_at: f64,
    pub flight_log_list_complete: bool,

    pub dl_active: bool,
    pub dl_cancel: bool,
    pub dl_id: i32,
    pub dl_size: u32,
    pub dl_received: u64,
    pub dl_started_at: f64,
    pub dl_speed: f64,
    pub dl_path: String,
    pub dl_status: String,
    pub dl_error: String,

    // --- mission map ([n]) ---------------------------------------------
    pub map_range: f64,
    pub map_trail_enabled: bool,
    pub map_fit_kml: bool,
    pub map_lidar_enabled: bool,
    pub map_navpath_enabled: bool,
    /// Arrow-key pan of the view centre, metres N / E.
    pub map_pan: (f64, f64),
    /// [u]: keep the view centred on the vehicle (pan is relative to it).
    pub map_follow: bool,
    /// [B]: how LiDAR points are placed on the map.
    pub map_lidar_frame: LidarFrame,

    /// KML overlay ([o]) - local visualization only until [O] uploads it.
    pub kml_path: String,
    pub kml: Option<crate::geo::Kml>,
    pub kml_error: String,

    /// Waypoint queue ([W] / [C]): remaining targets, current first.
    pub wp_queue: VecDeque<Target>,
    /// Every target of the running queue, for drawing the whole path.
    pub wp_path: Vec<Target>,
    pub wp_queue_mode: String,
    pub wp_queue_total: usize,
    pub wp_queue_alt_amsl: f64,
    pub wp_queue_face_target: bool,
    /// "Facing target" mode: turning in place before the leg.
    pub wp_queue_rotating: bool,
    pub wp_queue_active: bool,
    pub wp_queue_status: String,
    pub wp_queue_phase_started_at: f64,

    /// Geofence upload ([O]): (lat, lon, vertex count of its polygon).
    pub fence_upload_active: bool,
    pub fence_upload_items: Vec<(f64, f64, u16)>,
    pub fence_upload_acked_seq: i32,
    pub fence_upload_started_at: f64,
    pub fence_upload_status: String,
    pub fence_upload_error: String,

    /// Satellite snapshot ([i]).
    pub map_download_active: bool,
    pub map_download_status: String,
    pub map_download_path: String,
    pub map_download_error: String,
    pub map_download_started_at: f64,
}

impl State {
    pub fn new() -> Self {
        State {
            mode: "UNKNOWN".into(),
            autopilot: -1,
            battery: -1.0,
            voltage: -1.0,
            current: -1.0,
            battery_id: -1,
            rangefinder_quality: -1,
            rangefinder_orientation: -1,
            gps_heading: -1.0,
            rtk_health: -1,
            ekf_status: "UNKNOWN".into(),
            rc_rssi: -1,
            available_modes_seq: -1,
            dl_id: -1,
            dl_status: "IDLE".into(),
            map_range: 30.0,
            map_follow: true,
            map_trail_enabled: true,
            fence_upload_status: "IDLE".into(),
            map_download_status: "IDLE".into(),
            ..Default::default()
        }
    }

    // --- event log -------------------------------------------------------

    pub fn add_log(&mut self, level: Level, message: impl AsRef<str>) {
        // A raw control character in a row would move the real cursor, so
        // collapse every run of them to one space (see eventlog.py).
        let mut text = String::new();
        let mut in_ctrl = false;
        for ch in message.as_ref().chars() {
            if ch.is_control() {
                if !in_ctrl {
                    text.push(' ');
                }
                in_ctrl = true;
            } else {
                text.push(ch);
                in_ctrl = false;
            }
        }
        let event = LogEvent {
            time: chrono::Local::now().format("%H:%M:%S").to_string(),
            level,
            message: text.trim().to_string(),
        };
        if self.events.len() >= MAX_LOG_EVENTS {
            self.events.pop_front();
        }
        self.events.push_back(event);
        match level {
            Level::Warn => self.warning_count += 1,
            Level::Error => self.error_count += 1,
            Level::Failsafe => self.failsafe_count += 1,
            _ => {}
        }
    }

    pub fn info(&mut self, m: impl AsRef<str>) {
        self.add_log(Level::Info, m)
    }
    pub fn command(&mut self, m: impl AsRef<str>) {
        self.add_log(Level::Command, m)
    }
    pub fn warn(&mut self, m: impl AsRef<str>) {
        self.add_log(Level::Warn, m)
    }
    pub fn error(&mut self, m: impl AsRef<str>) {
        self.add_log(Level::Error, m)
    }
    pub fn failsafe(&mut self, m: impl AsRef<str>) {
        self.add_log(Level::Failsafe, m)
    }

    /// A received parameter's finite numeric value.
    pub fn param_value(&self, name: &str) -> Option<f64> {
        self.parameters
            .get(name)
            .map(|p| p.value)
            .filter(|v| v.is_finite())
    }

    pub fn trail_push(&mut self, x: f64, y: f64) {
        let far_enough = match self.position_trail.back() {
            None => true,
            Some(&(px, py)) => (x - px).hypot(y - py) >= POSITION_TRAIL_MIN_SPACING_M,
        };
        if far_enough {
            if self.position_trail.len() >= POSITION_TRAIL_MAXLEN {
                self.position_trail.pop_front();
            }
            self.position_trail.push_back((x, y));
        }
    }
}

/// Map a PX4 STATUSTEXT to INFO / WARN / ERROR / FAILSAFE. Keyword matches
/// win over the numeric MAV_SEVERITY, so e.g. an "ARMING DENIED" notice at
/// severity 6 still surfaces as an error.
pub fn classify_statustext(severity: u32, text: &str) -> Level {
    let upper = text.to_uppercase();

    const FAILSAFE: &[&str] = &[
        "FAILSAFE", "RC LOSS", "RC_LOSS", "OFFBOARD LOST", "OFFBOARD_LOST",
        "OFFBOARD LOSS", "DATA LINK LOST", "DATALINK LOST", "BATTERY FAILSAFE",
        "GPS FAILURE",
    ];
    const ERROR: &[&str] = &[
        "PREFLIGHT FAIL", "PREFLIGHT FAILED", "ARMING DENIED", "ARM DENIED",
        "DISARM DENIED", "COMMAND DENIED", "REJECT", "FAILED", "ERROR", "CRITICAL",
    ];
    const WARN: &[&str] = &[
        "WARNING", "WARN", "BATTERY", "LOW BATTERY", "EKF", "GPS", "RC", "ARM",
        "DISARM", "CALIBRAT",
    ];

    if FAILSAFE.iter().any(|w| upper.contains(w)) {
        return Level::Failsafe;
    }
    if ERROR.iter().any(|w| upper.contains(w)) {
        return Level::Error;
    }
    if WARN.iter().any(|w| upper.contains(w)) {
        return Level::Warn;
    }
    match severity {
        0..=3 => Level::Error,
        4 => Level::Warn,
        _ => Level::Info,
    }
}

/// Frame the LiDAR cloud is expressed in, for the mission-map overlay.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum LidarFrame {
    /// Robot / sensor frame (FLU): rotated by the vehicle yaw and placed at
    /// the vehicle position.
    #[default]
    Body,
    /// Already in a world frame (ENU, REP-105 map/odom): drawn as-is.
    World,
}

impl LidarFrame {
    pub fn next(self) -> Self {
        match self {
            LidarFrame::Body => LidarFrame::World,
            LidarFrame::World => LidarFrame::Body,
        }
    }
}
