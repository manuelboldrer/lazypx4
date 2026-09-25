//! Tunables, protocol constants and lookup tables.
//!
//! Nothing in here talks to MAVLink or touches shared state. Values the
//! command line can override live on [`Settings`]; everything else is a fixed
//! constant. Mirrors `lazypx4/config.py`.

use std::time::Duration;

/// User-overridable settings, populated once at start-up (see `main.rs`).
#[derive(Debug, Clone)]
pub struct Settings {
    /// UDP port to listen on for the PX4 MAVLink connection.
    pub port: u16,
    /// Directory downloaded `.ulg` flight logs are written to.
    pub log_dir: String,
    /// Optional PX4 `parameters.json` - enables the "changed from default"
    /// view and replaces the bundled parameter descriptions.
    pub param_defaults_file: Option<String>,
    /// Refuse to start a flight-log transfer while armed unless set.
    pub allow_log_download_while_armed: bool,
    /// Directory satellite-image snapshots are written to.
    pub map_dir: String,
    /// Filesystem whose usage the dashboard HOST line shows.
    pub disk_path: String,
    /// Directory scanned for .px4 files on the flash-firmware screen.
    pub firmware_dir: String,
    /// px_uploader.py / upload_log.py / ecl_ekf live here.
    pub tools_dir: String,
    pub ulog_upload_server: String,
    /// .kml to load as the map overlay at start-up (like pressing [o]).
    pub kml_path: Option<String>,
    /// ROS 2 topics (only used by a `--features ros` build).
    pub lidar_topic: String,
    pub navpath_topic: String,
    pub camera_topics: [String; 2],
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    pub use_sim_time: bool,
}

pub const HEARTBEAT_TIMEOUT: f64 = 5.0;
pub const TELEMETRY_TIMEOUT: f64 = 5.0;

/// A "Preflight Fail" / "PREARM" STATUSTEXT is latched onto the dashboard.
/// PX4 re-broadcasts it (~1 Hz) while the check is still failing, so if none
/// has repeated within this long the banner clears.
pub const PREFLIGHT_FAIL_TIMEOUT: f64 = 5.0;

pub const FRAME_PERIOD: Duration = Duration::from_millis(100);

pub const MIN_TERMINAL_COLS: u16 = 60;
pub const MIN_TERMINAL_ROWS: u16 = 16;

pub const ARM_CONFIRM_TIMEOUT: f64 = 5.0;

pub const MAX_LOG_EVENTS: usize = 500;

pub const BATTERY_LOW: f64 = 30.0;
pub const BATTERY_CRITICAL: f64 = 15.0;

// Dashboard health-colour thresholds: (GOOD, OK).
pub const GPS_HDOP_GOOD: f64 = 1.5;
pub const GPS_HDOP_OK: f64 = 3.0;
pub const GPS_SATS_GOOD: f64 = 15.0;
pub const GPS_SATS_OK: f64 = 10.0;
pub const POS_ACC_GOOD: f64 = 1.0;
pub const POS_ACC_OK: f64 = 3.0;
pub const RC_SIGNAL_GOOD: f64 = 70.0;
pub const RC_SIGNAL_OK: f64 = 30.0;
pub const RANGEFINDER_SIGNAL_GOOD: f64 = 70.0;
pub const RANGEFINDER_SIGNAL_OK: f64 = 30.0;
pub const RX_RATE_GOOD: f64 = 20.0;
pub const RX_RATE_OK: f64 = 5.0;
pub const VIBRATION_GOOD: f64 = 15.0;
pub const VIBRATION_OK: f64 = 30.0;

pub const RC_PWM_MIN: i32 = 1000;
pub const RC_PWM_CENTER: i32 = 1500;
pub const RC_PWM_MAX: i32 = 2000;
pub const RC_PWM_DISPLAY_MIN: i32 = 800;
pub const RC_PWM_DISPLAY_MAX: i32 = 2200;

/// DISTANCE_SENSOR orientation for a straight-down sensor
/// (MAV_SENSOR_ROTATION_PITCH_270) - the one PX4 fuses for HAGL.
pub const DISTANCE_SENSOR_ORIENTATION_DOWN: i32 = 25;

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

pub const PARAM_REQUEST_TIMEOUT: f64 = 15.0;
pub const PARAM_SET_TIMEOUT: f64 = 3.0;
pub const PARAM_VALUE_EPSILON: f64 = 1e-6;

/// The KEY PARAMETERS panel at the top of the parameter screen.
pub const KEY_PARAMETER_COLUMNS: &[&[(&str, &[&str])]] = &[
    &[
        (
            "FLIGHT",
            &["MIS_TAKEOFF_ALT", "RTL_RETURN_ALT", "MPC_XY_CRUISE", "MPC_XY_VEL_MAX", "MPC_LAND_SPEED"],
        ),
        ("FAILSAFE", &["GF_ACTION", "NAV_DLL_ACT", "NAV_RCL_ACT", "COM_LOW_BAT_ACT"]),
    ],
    &[
        (
            "ESTIMATOR",
            &[
                "EKF2_HGT_REF", "EKF2_RNG_CTRL", "EKF2_OF_CTRL", "EKF2_RNG_NOISE",
                "EKF2_GPS_CTRL", "EKF2_GPS_CHECK", "EKF2_BARO_CTRL", "EKF2_MAG_TYPE",
            ],
        ),
        ("ROS 2 / DDS", &["UXRCE_DDS_DOM_ID", "UXRCE_DDS_KEY", "UXRCE_DDS_NS_IDX"]),
    ],
];

pub fn param_type_name(t: u8) -> String {
    match t {
        1 => "UINT8".into(),
        2 => "INT8".into(),
        3 => "UINT16".into(),
        4 => "INT16".into(),
        5 => "UINT32".into(),
        6 => "INT32".into(),
        7 => "UINT64".into(),
        8 => "INT64".into(),
        9 => "REAL32".into(),
        10 => "REAL64".into(),
        _ => format!("TYPE_{t}"),
    }
}

// ---------------------------------------------------------------------------
// Flight modes (Standard Modes Protocol)
// ---------------------------------------------------------------------------

pub const MAVLINK_MSG_ID_AVAILABLE_MODES: f32 = 435.0;
pub const AVAILABLE_MODES_TIMEOUT: f64 = 5.0;
pub const MODE_PAGE_SIZE: usize = 18;

// ---------------------------------------------------------------------------
// MAVLink shell / flight logs
// ---------------------------------------------------------------------------

pub const SHELL_MAX_LINES: usize = 1000;

pub const LOG_LIST_TIMEOUT: f64 = 5.0;
pub const LOG_LIST_QUIET_PERIOD: f64 = 0.60;
pub const LOG_CHUNK_SIZE: usize = 90;
pub const LOG_REQUEST_SIZE: u32 = 5000;
pub const LOG_CHUNK_TIMEOUT: f64 = 5.0;
pub const LOG_CHUNK_RETRIES: u32 = 5;

pub const POSITION_TRAIL_MIN_SPACING_M: f64 = 0.2;
pub const POSITION_TRAIL_MAXLEN: usize = 1000;

// ---------------------------------------------------------------------------
// Estimation / sensor stream setup
// ---------------------------------------------------------------------------

/// (message id, interval in microseconds) requested at connect.
pub const STREAM_MESSAGE_INTERVALS: &[(u32, u32)] = &[
    (2, 1_000_000),  // SYSTEM_TIME                1 Hz
    (30, 100_000),   // ATTITUDE                  10 Hz
    (32, 100_000),   // LOCAL_POSITION_NED        10 Hz
    (33, 200_000),   // GLOBAL_POSITION_INT        5 Hz
    (24, 200_000),   // GPS_RAW_INT                5 Hz
    (124, 500_000),  // GPS2_RAW                   2 Hz
    (127, 500_000),  // GPS_RTK                    2 Hz
    (128, 1_000_000), // GPS2_RTK                  1 Hz
    (29, 500_000),   // SCALED_PRESSURE            2 Hz
    (74, 200_000),   // VFR_HUD                    5 Hz
    (132, 200_000),  // DISTANCE_SENSOR            5 Hz
    (141, 200_000),  // ALTITUDE                   5 Hz
    (230, 400_000),  // ESTIMATOR_STATUS         2.5 Hz
    (62, 200_000),   // NAV_CONTROLLER_OUTPUT      5 Hz
    (83, 200_000),   // ATTITUDE_TARGET            5 Hz
    (85, 200_000),   // POSITION_TARGET_LOCAL_NED  5 Hz
    (241, 500_000),  // VIBRATION                  2 Hz
    (231, 1_000_000), // WIND_COV                  1 Hz
    (36, 200_000),   // SERVO_OUTPUT_RAW           5 Hz
    (162, 1_000_000), // FENCE_STATUS              1 Hz
];

/// Delay between the ~25 commands of `configure_streams`, so they don't
/// saturate a low-baud serial link behind the companion computer.
pub const CONFIGURE_STREAMS_PACING: Duration = Duration::from_millis(20);

pub const ESTIMATOR_PARAM_NAMES: &[&str] = &[
    "EKF2_HGT_REF",
    "EKF2_GPS_CTRL",
    "EKF2_BARO_CTRL",
    "EKF2_RNG_CTRL",
    "EKF2_EV_CTRL",
    "EKF2_OF_CTRL",
    "EKF2_AGP_CTRL",
    "EKF2_MAG_TYPE",
    "EKF2_MIN_RNG",
    "EKF2_RNG_A_HMAX",
];

pub fn ekf2_hgt_ref_name(v: i64) -> String {
    match v {
        0 => "BARO".into(),
        1 => "GPS".into(),
        2 => "RANGE".into(),
        3 => "EXT VISION".into(),
        _ => v.to_string(),
    }
}

// ---------------------------------------------------------------------------
// Geofence (GF_ACTION parameter)
// ---------------------------------------------------------------------------

pub fn gf_action_name(v: i64) -> &'static str {
    match v {
        0 => "NONE (disabled)",
        1 => "WARNING",
        2 => "HOLD",
        3 => "RETURN",
        4 => "TERMINATE",
        _ => "?",
    }
}

pub fn gf_action_from_letter(s: &str) -> Option<i64> {
    match s {
        "n" => Some(0),
        "w" => Some(1),
        "h" => Some(2),
        "r" => Some(3),
        "t" => Some(4),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Sensor calibration (MAV_CMD_PREFLIGHT_CALIBRATION)
// ---------------------------------------------------------------------------

pub fn calibration_params(kind: &str) -> Option<[f32; 7]> {
    Some(match kind {
        "gyro" => [1., 0., 0., 0., 0., 0., 0.],
        "mag" => [0., 1., 0., 0., 0., 0., 0.],
        "baro" => [0., 0., 1., 0., 0., 0., 0.],
        "accel" => [0., 0., 0., 0., 1., 0., 0.],
        "level" => [0., 0., 0., 0., 2., 0., 0.],
        _ => return None,
    })
}

pub fn calibration_label(kind: &str) -> &str {
    match kind {
        "gyro" => "gyroscope",
        "mag" => "magnetometer / compass",
        "baro" => "barometer",
        "accel" => "accelerometer",
        "level" => "level horizon",
        other => other,
    }
}

// ---------------------------------------------------------------------------
// COMMAND_ACK / GPS / EKF display tables
// ---------------------------------------------------------------------------

pub fn command_name(id: u32) -> String {
    match id {
        11 => "SET_MODE".into(),
        20 => "NAV_RETURN_TO_LAUNCH".into(),
        21 => "NAV_LAND".into(),
        22 => "NAV_TAKEOFF".into(),
        176 => "DO_SET_MODE".into(),
        179 => "DO_SET_HOME".into(),
        185 => "FLIGHT_TERMINATION".into(),
        192 => "DO_REPOSITION".into(),
        193 => "DO_PAUSE_CONTINUE".into(),
        241 => "PREFLIGHT_CALIBRATION".into(),
        246 => "PREFLIGHT_REBOOT_SHUTDOWN".into(),
        262 => "DO_SET_STANDARD_MODE".into(),
        400 => "COMPONENT_ARM_DISARM".into(),
        511 => "SET_MESSAGE_INTERVAL".into(),
        512 => "REQUEST_MESSAGE".into(),
        _ => format!("COMMAND_{id}"),
    }
}

pub fn ack_name(result: u32) -> String {
    match result {
        0 => "ACCEPTED".into(),
        1 => "TEMPORARILY_REJECTED".into(),
        2 => "DENIED".into(),
        3 => "UNSUPPORTED".into(),
        4 => "FAILED".into(),
        5 => "IN_PROGRESS".into(),
        6 => "CANCELLED".into(),
        _ => format!("RESULT_{result}"),
    }
}

pub fn gps_fix_name(fix: i32) -> String {
    match fix {
        0 => "NO GPS".into(),
        1 => "NO FIX".into(),
        2 => "2D FIX".into(),
        3 => "3D FIX".into(),
        4 => "DGPS".into(),
        5 => "RTK FLOAT".into(),
        6 => "RTK FIXED".into(),
        7 => "STATIC".into(),
        8 => "PPP".into(),
        _ => format!("FIX {fix}"),
    }
}

pub fn distance_orientation_name(o: i32) -> String {
    match o {
        o if o < 0 => "?".into(),
        0 => "forward".into(),
        24 => "up".into(),
        25 => "down".into(),
        o => format!("rot {o}"),
    }
}

/// SYS_STATUS sensor bits relevant to a pre-flight arming check.
pub const SYS_STATUS_SENSOR_LABELS: &[(u32, &str)] = &[
    (1, "GYRO"),
    (2, "ACCEL"),
    (4, "MAG"),
    (8, "BARO"),
    (32, "GPS"),
    (64, "OPT_FLOW"),
    (1024, "RATE_CTRL"),
    (2048, "ATT_CTRL"),
    (4096, "YAW_POS"),
    (8192, "ALT_CTRL"),
    (16384, "POS_CTRL"),
    (32768, "MOTORS"),
    (65536, "RC"),
    (33554432, "BATTERY"),
];

pub fn firmware_version_type_name(v: u32) -> &'static str {
    match v {
        0 => "dev",
        64 => "alpha",
        128 => "beta",
        192 => "rc",
        255 => "release",
        _ => "?",
    }
}

pub fn pos_target_frame_name(f: u32) -> String {
    match f {
        1 => "LOCAL_NED".into(),
        7 => "LOCAL_OFFSET_NED".into(),
        8 => "BODY_NED".into(),
        9 => "BODY_OFFSET_NED".into(),
        20 => "LOCAL_FRD".into(),
        21 => "LOCAL_FLU".into(),
        _ => format!("FRAME {f}"),
    }
}

// ---------------------------------------------------------------------------
// Mission map, jog, waypoint queue, fence upload, satellite snapshot
// ---------------------------------------------------------------------------

/// Manual zoom cap, also bounding how far loaded KML geometry can push the
/// view (a shared Google Earth export can hold far-away unrelated sites).
pub const MAP_RANGE_MAX_M: f64 = 5000.0;

pub const JOG_STEP_M: f64 = 1.0;
pub const JOG_STEP_MIN_M: f64 = 0.25;
pub const JOG_STEP_MAX_M: f64 = 10.0;
pub const JOG_YAW_STEP_DEG: f64 = 15.0;
/// Minimum spacing between nudges, so a held key doesn't flood the link.
pub const JOG_MIN_INTERVAL: f64 = 0.12;

pub const WP_QUEUE_ARRIVAL_M: f64 = 1.0;
pub const WP_QUEUE_POLL_S: f64 = 0.5;
pub const WP_QUEUE_LEG_TIMEOUT_S: f64 = 120.0;
pub const WP_QUEUE_YAW_TOLERANCE_DEG: f64 = 8.0;
pub const WP_QUEUE_ROTATE_TIMEOUT_S: f64 = 20.0;

pub const FENCE_UPLOAD_TIMEOUT: f64 = 15.0;

pub const GPS_PUBLISH_TOPIC: &str = "/fire_gps_loc";
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
pub const GPS_PUBLISH_FRAME_ID: &str = "gps";
pub const GPS_PUBLISH_FEEDBACK_TIMEOUT_S: f64 = 4.0;

pub const MAP_IMAGE_SIZE: u32 = 1024;
pub const MAP_IMAGE_MIN_SPAN_M: f64 = 120.0;
pub const MAP_IMAGE_PAD: f64 = 1.8;
pub const MAP_DOWNLOAD_TIMEOUT_S: u64 = 25;

pub const LIDAR_CAM_ROTATE_STEP: f64 = 5.0;
pub const CAMERA_LOW_BW_MAX_COLS: usize = 48;

pub const WIFI_SIGNAL_GOOD: f64 = 60.0;
pub const WIFI_SIGNAL_OK: f64 = 30.0;
