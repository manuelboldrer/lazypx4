//! Preflight go / no-go: every check the operator would otherwise collect
//! from four screens, folded into one verdict. Pure - reads State, the host
//! monitor and the settings.

use crate::config::*;
use crate::host::SystemStats;
use crate::state::{State, now};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Status {
    /// Not known yet (no data) - never blocks, but shown dim.
    Unknown,
    Pass,
    Warn,
    Fail,
}

pub struct Check {
    pub group: &'static str,
    pub name: &'static str,
    pub status: Status,
    pub detail: String,
}

/// SYS_STATUS: PX4 sets this bit in `health` (only - never in `present`)
/// when it could arm in the current mode (SYS_STATUS.hpp,
/// health_report.can_arm_mode_flags).
const MAV_SYS_STATUS_PREARM_CHECK: u32 = 0x1000_0000;

/// Calibration id parameters: zero means "never calibrated".
pub const CALIBRATION_PARAMS: &[&str] = &["CAL_ACC0_ID", "CAL_GYRO0_ID", "CAL_MAG0_ID", "SYS_HAS_MAG"];

fn check(group: &'static str, name: &'static str, status: Status, detail: impl Into<String>) -> Check {
    Check { group, name, status, detail: detail.into() }
}

pub fn checks(st: &State, sys: &SystemStats, settings: &Settings) -> Vec<Check> {
    use Status::*;
    let t = now();
    let mut out = Vec::new();

    // --- vehicle link / PX4's own verdict -------------------------------
    let hb_age = t - st.last_heartbeat;
    out.push(if !st.vehicle_locked {
        check("PX4", "Link", Fail, "no heartbeat yet")
    } else if !st.connected {
        check("PX4", "Link", Fail, format!("heartbeat lost {hb_age:.0} s ago"))
    } else {
        check("PX4", "Link", Pass, format!("sys {}  ·  {:.0} msg/s", st.target_system, st.rx_rate))
    });

    let prearm_known = st.sensors_present != 0;
    out.push(if !st.last_preflight_fail.is_empty() {
        check("PX4", "Arming checks", Fail, st.last_preflight_fail.clone())
    } else if prearm_known && st.sensors_health & MAV_SYS_STATUS_PREARM_CHECK == 0 {
        check("PX4", "Arming checks", Fail, "PX4 cannot arm in the current mode - see the event log [g]")
    } else if prearm_known {
        check("PX4", "Arming checks", Pass, "PX4 can arm in the current mode")
    } else {
        check("PX4", "Arming checks", Unknown, "not reported yet")
    });

    let bad: Vec<&str> = SYS_STATUS_SENSOR_LABELS
        .iter()
        .filter(|(bit, _)| st.sensors_present & st.sensors_enabled & bit != 0 && st.sensors_health & bit == 0)
        .map(|(_, label)| *label)
        .collect();
    out.push(if st.sensors_present == 0 {
        check("PX4", "Sensors", Unknown, "not reported")
    } else if bad.is_empty() {
        check("PX4", "Sensors", Pass, "all enabled sensors healthy")
    } else {
        check("PX4", "Sensors", Fail, format!("unhealthy: {}", bad.join(" ")))
    });

    let cal = |name: &str| st.param_value(name);
    let has_mag = cal("SYS_HAS_MAG").map(|v| v != 0.0).unwrap_or(true);
    let wanted: Vec<(&str, &str)> = [("CAL_ACC0_ID", "accel"), ("CAL_GYRO0_ID", "gyro"), ("CAL_MAG0_ID", "mag")]
        .into_iter()
        .filter(|(p, _)| has_mag || *p != "CAL_MAG0_ID")
        .collect();
    let missing: Vec<&str> = wanted.iter().filter(|(p, _)| cal(p) == Some(0.0)).map(|(_, n)| *n).collect();
    out.push(if wanted.iter().any(|(p, _)| cal(p).is_none()) {
        check("PX4", "Calibration", Unknown, "CAL_*_ID not loaded yet")
    } else if missing.is_empty() {
        check("PX4", "Calibration", Pass, format!("{} calibrated", wanted.iter().map(|(_, n)| *n).collect::<Vec<_>>().join(", ")))
    } else {
        check("PX4", "Calibration", Fail, format!("not calibrated: {} ([s])", missing.join(", ")))
    });

    // --- navigation ------------------------------------------------------
    let fix = gps_fix_name(st.gps_fix);
    out.push(if st.last_gps == 0.0 {
        check("NAV", "GPS", Unknown, "no GPS data")
    } else if st.gps_fix < 3 {
        check("NAV", "GPS", Fail, format!("{fix}, {} sats", st.gps_sats))
    } else if (st.gps_sats as f64) < GPS_SATS_OK || st.gps_hdop > GPS_HDOP_OK {
        check("NAV", "GPS", Warn, format!("{fix}, {} sats, HDOP {:.1}", st.gps_sats, st.gps_hdop))
    } else {
        check("NAV", "GPS", Pass, format!("{fix}, {} sats, HDOP {:.1}", st.gps_sats, st.gps_hdop))
    });
    if st.gps_fix >= 5 || st.last_gps_rtk > 0.0 {
        out.push(match st.gps_fix {
            6 => check("NAV", "RTK", Pass, "FIXED"),
            5 => check("NAV", "RTK", Warn, "FLOAT"),
            _ => check("NAV", "RTK", Warn, "RTK data but no RTK fix"),
        });
    }

    let (verdict, color) = crate::ui::dashboard::ekf_summary(st.ekf_flags, st.est_is_estimator_status, &st.ekf_status);
    out.push(match color {
        c if c == crate::ui::GREEN => check("NAV", "EKF", Pass, verdict),
        c if c == crate::ui::RED => check("NAV", "EKF", Fail, verdict),
        c if c == crate::ui::YELLOW => check("NAV", "EKF", Warn, verdict),
        _ => check("NAV", "EKF", Unknown, verdict),
    });

    out.push(if st.home_set {
        check("NAV", "Home", Pass, format!("{:.6}, {:.6}", st.home_lat, st.home_lon))
    } else {
        check("NAV", "Home", Warn, "not set - RTL has nowhere to go")
    });

    // --- power / airframe --------------------------------------------------
    out.push(if st.battery < 0.0 {
        check("POWER", "Battery", Unknown, "not reported")
    } else if st.battery <= settings.battery_critical {
        check("POWER", "Battery", Fail, format!("{:.0}% ({:.2} V)", st.battery, st.voltage))
    } else if st.battery <= settings.battery_low {
        check("POWER", "Battery", Warn, format!("{:.0}% ({:.2} V)", st.battery, st.voltage))
    } else {
        check("POWER", "Battery", Pass, format!("{:.0}% ({:.2} V)", st.battery, st.voltage))
    });

    if st.last_vibration > 0.0 {
        let (v, color) = crate::ui::dashboard::vibration_verdict(st);
        let status = if color == crate::ui::RED { Warn } else { Pass };
        out.push(check("POWER", "Vibration", status, v));
    }

    out.push(if !st.rc_received {
        check("POWER", "RC", Warn, "no RC - make sure NAV_RCL_ACT / COM_RC_IN_MODE allow it")
    } else if st.rc_failsafe {
        check("POWER", "RC", Fail, "RC failsafe")
    } else {
        check("POWER", "RC", Pass, if st.rc_rssi >= 0 { format!("RSSI {}%", st.rc_rssi) } else { "connected".into() })
    });

    if let Some(action) = st.param_value("GF_ACTION") {
        let a = action.round() as i64;
        out.push(check(
            "POWER",
            "Geofence",
            if a == 0 { Warn } else { Pass },
            format!("GF_ACTION = {}", gf_action_name(a)),
        ));
    }

    // --- companion computer ----------------------------------------------
    if sys.ok {
        out.push(if sys.disk_free_gb < 1.0 {
            check("HOST", "Disk", Fail, format!("{:.1} GB free on {}", sys.disk_free_gb, settings.disk_path))
        } else if sys.disk_free_gb < 5.0 {
            check("HOST", "Disk", Warn, format!("{:.1} GB free on {}", sys.disk_free_gb, settings.disk_path))
        } else {
            check("HOST", "Disk", Pass, format!("{:.0} GB free on {}", sys.disk_free_gb, settings.disk_path))
        });
        out.push(if sys.cpu_percent > 90.0 {
            check("HOST", "CPU", Warn, format!("{:.0}%", sys.cpu_percent))
        } else {
            check("HOST", "CPU", Pass, format!("{:.0}%  ·  RAM {:.0}%", sys.cpu_percent, sys.mem_percent))
        });
        let dds = sys.xrce_agent_running || sys.zenoh_running;
        out.push(check(
            "HOST",
            "ROS 2 bridge",
            if dds { Pass } else { Warn },
            match (sys.xrce_agent_running, sys.zenoh_running) {
                (true, true) => "uXRCE-DDS agent + Zenoh running",
                (true, false) => "uXRCE-DDS agent running",
                (false, true) => "Zenoh running",
                _ => "no uXRCE-DDS agent / Zenoh process found",
            },
        ));
        out.push(check(
            "HOST",
            "rosbag",
            if sys.rosbag_recording { Pass } else { Warn },
            if sys.rosbag_recording { "recording" } else { "not recording" },
        ));
    }
    out
}

/// (worst status, number of fails, number of warnings).
pub fn verdict(checks: &[Check]) -> (Status, usize, usize) {
    let fails = checks.iter().filter(|c| c.status == Status::Fail).count();
    let warns = checks.iter().filter(|c| c.status == Status::Warn).count();
    let worst = checks.iter().map(|c| c.status).max().unwrap_or(Status::Unknown);
    (worst, fails, warns)
}

pub fn verdict_text(checks: &[Check]) -> String {
    match verdict(checks) {
        (Status::Fail, f, w) => format!("NO-GO: {f} failing, {w} warning(s)"),
        (Status::Warn, _, w) => format!("GO with {w} warning(s)"),
        _ => "GO".into(),
    }
}
