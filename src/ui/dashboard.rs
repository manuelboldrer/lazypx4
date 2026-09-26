//! The default screen: link / arm / mode / battery and a live telemetry
//! summary. Mirrors `render/dashboard.py`.

use ratatui::style::{Color, Modifier, Style};
use ratatui::text::Line;

use super::*;
use crate::state::now;

pub fn gps_fix_display(fix: i32) -> (Color, String) {
    let name = gps_fix_name(fix);
    let color = match fix {
        f if f >= 3 => GREEN,
        2 => YELLOW,
        _ => RED,
    };
    (color, name)
}

fn rtk_text(fix: i32) -> Ln {
    match fix {
        6 => Ln::new().st("FIXED", Style::new().fg(GREEN).add_modifier(Modifier::BOLD)),
        5 => Ln::new().fg("FLOAT", YELLOW),
        _ => Ln::new().dim("no"),
    }
}

/// Condense the estimator flags into one coloured verdict.
pub fn ekf_summary(flags: u32, is_estimator_status: bool, status: &str) -> (String, Color) {
    if flags == 0 {
        return if status == "OK" { ("OK".into(), GREEN) } else { (status.into(), Color::DarkGray) };
    }
    let has_att = flags & 1 != 0;
    let has_vel_h = flags & 2 != 0;
    let has_pos_h = flags & (8 | 16) != 0;
    let (gps_glitch, accel_error) = if is_estimator_status {
        (flags & 1024 != 0, flags & 2048 != 0)
    } else {
        if flags & 1024 != 0 {
            return ("INITIALIZING".into(), YELLOW);
        }
        (false, false)
    };
    if !has_att {
        ("NO ATTITUDE".into(), RED)
    } else if accel_error {
        ("ACCEL ERROR".into(), RED)
    } else if !has_vel_h || !has_pos_h {
        ("DEAD RECKONING".into(), YELLOW)
    } else if gps_glitch {
        ("GPS GLITCH".into(), YELLOW)
    } else {
        ("OK".into(), GREEN)
    }
}

/// Whether the rangefinder is feeding the EKF: distinguishes "deliberately
/// off" from "should be fusing but isn't".
fn rangefinder_status(rng_ctrl: Option<f64>, ekf_flags: u32, orientation: i32, age: Option<f64>) -> (String, Option<Color>) {
    let Some(rng_ctrl) = rng_ctrl else {
        return ("EKF2_RNG_CTRL not loaded".into(), None);
    };
    if rng_ctrl.round() as i64 == 0 {
        return ("disabled (EKF2_RNG_CTRL=0)".into(), None);
    }
    match age {
        None => ("enabled, no sensor data".into(), Some(RED)),
        Some(a) if a > 3.0 => ("enabled, no sensor data".into(), Some(RED)),
        _ if orientation != DISTANCE_SENSOR_ORIENTATION_DOWN => ("enabled, sensor not facing down".into(), Some(YELLOW)),
        Some(a) if a > 1.0 => ("enabled, data stale".into(), Some(YELLOW)),
        _ if ekf_flags & 64 != 0 => ("fusing (HAGL valid)".into(), Some(GREEN)),
        _ => ("enabled, not fusing yet".into(), Some(YELLOW)),
    }
}

fn vibration_verdict(st: &State) -> (&'static str, Color) {
    if st.clipping.iter().any(|&c| c > 0) {
        return ("CLIPPING", RED);
    }
    let peak = st.vibration_x.max(st.vibration_y).max(st.vibration_z);
    if peak >= VIBRATION_OK {
        ("HIGH", RED)
    } else if peak >= VIBRATION_GOOD {
        ("ELEVATED", YELLOW)
    } else {
        ("OK", GREEN)
    }
}

pub use crate::geo::global_to_local;

/// UTC `HH:MM:SS.mmmZ` for a Unix-epoch timestamp in seconds.
fn format_clock(epoch_seconds: f64) -> String {
    let secs = epoch_seconds.floor() as i64;
    let millis = (((epoch_seconds - secs as f64) * 1000.0) as u32) % 1000;
    chrono::DateTime::from_timestamp(secs, 0)
        .map(|t| format!("{}.{millis:03}Z", t.format("%H:%M:%S")))
        .unwrap_or_else(|| "--".into())
}

fn dim_or(color: Option<Color>, text: String) -> Ln {
    match color {
        Some(c) => Ln::new().fg(text, c),
        None => Ln::new().dim(text),
    }
}

fn load_color(percent: f64) -> Color {
    if percent >= 90.0 {
        RED
    } else if percent >= 70.0 {
        YELLOW
    } else {
        GREEN
    }
}

/// The companion computer's HOST (load) and SVC (rosbag / zenoh / XRCE
/// agent) lines - see host.rs.
fn host_lines(ctx: &Ctx) -> Vec<Line<'static>> {
    let h = crate::host::lock(&ctx.app.sys).clone();
    let dot = |on: bool| Ln::new().fg("●", if on { GREEN } else { RED });
    let svc = Ln::new()
        .raw(" SVC:  rosbag ")
        .spans(dot(h.rosbag_recording).0)
        .raw("    zenoh ")
        .spans(dot(h.zenoh_running).0)
        .raw("    xrce-agent ")
        .spans(dot(h.xrce_agent_running).0)
        .line();
    if !h.ok {
        return vec![Ln::new().raw(" HOST: ").dim("n/a (not Linux / /proc unreadable)").line(), svc];
    }
    vec![
        Ln::new()
            .raw(" HOST: CPU ")
            .fg(format!("{:3.0}%", h.cpu_percent), load_color(h.cpu_percent))
            .raw("   RAM ")
            .fg(format!("{:3.0}%", h.mem_percent), load_color(h.mem_percent))
            .dim(format!(" {:.1}/{:.1}G", h.mem_used_gb, h.mem_total_gb))
            .raw("   DISK ")
            .fg(format!("{:3.0}%", h.disk_percent), load_color(h.disk_percent))
            .dim(format!(" {:.0}G free", h.disk_free_gb))
            .raw(format!("   load {:.2}", h.load1))
            .line(),
        svc,
    ]
}

pub fn draw(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let t = now();
    let mut lines: Vec<Line<'static>> = Vec::new();

    let link = if st.connected {
        Ln::new().fg("CONNECTED", GREEN)
    } else if st.vehicle_locked {
        Ln::new().fg("DISCONNECTED", RED)
    } else {
        Ln::new().fg("WAITING FOR HEARTBEAT", YELLOW)
    };
    lines.push(
        Ln::new()
            .raw(" LINK: ")
            .spans(link.0)
            .raw(format!(
                "    PORT: {}    SYS: {}    COMP: {}    LOCKED: {}",
                ctx.app.settings.port,
                st.target_system,
                st.target_component,
                if st.vehicle_locked { "YES" } else { "NO" }
            ))
            .line(),
    );

    // Fixed slots (blank when inactive) so nothing below jumps a row when a
    // condition flips.
    if st.autopilot_version_received {
        let mut l = Ln::new().raw(" FW: ").bold(st.fw_version_text.clone());
        if !st.fw_git_hash.is_empty() {
            l = l.raw(" ").dim(st.fw_git_hash.clone());
        }
        lines.push(l.line());
    } else {
        lines.push(blank());
    }

    let fcu = if st.autopilot_unix_usec == 0 {
        Ln::new().dim("no wall clock yet (needs GPS lock)")
    } else {
        Ln::new()
            .raw(format_clock(st.autopilot_unix_usec as f64 / 1e6))
            .dim(format!(" ({})", st.autopilot_unix_usec))
            .dim(format!(" ({})", age_text(st.last_system_time)))
    };
    let ros_text = {
        let ros = crate::host::lock(&ctx.app.ros);
        if !ros.available {
            Ln::new().dim("n/a")
        } else if ros.time_ns <= 0 {
            Ln::new().dim(if ros.sim_time { "waiting for /clock" } else { "waiting..." })
        } else {
            let mut l = Ln::new().raw(format_clock(ros.time_ns as f64 / 1e9)).dim(format!(" ({})", ros.time_ns / 1000));
            if ros.sim_time {
                l = l.dim(" [sim]");
            }
            l.dim(format!(" ({})", age_text(ros.last_time)))
        }
    };
    let mut time_line = Ln::new().raw(" TIME:  ROS ").spans(ros_text.0).raw("   FCU ").spans(fcu.0);
    if st.autopilot_boot_ms > 0 {
        time_line = time_line.dim(format!("   boot {:.1}s", st.autopilot_boot_ms as f64 / 1000.0));
    }
    lines.push(time_line.line());

    lines.push(Ln::new().raw(" MODE: ").bold(st.mode.clone()).line());

    let mut state_line = Ln::new().raw(" STATE: ").spans(vec![armed_span(st.armed)]);
    if let Some(p) = &st.pending_arm {
        let text = if p.desired_armed { " ARMING..." } else { " DISARMING..." };
        state_line = state_line.st(text, Style::new().fg(YELLOW).add_modifier(Modifier::BOLD));
    }
    lines.push(state_line.line());

    let alert = Style::new().fg(RED).add_modifier(Modifier::BOLD);
    if !st.last_preflight_fail.is_empty() {
        lines.push(Ln::new().raw(" ").st(format!("PREARM: {}", st.last_preflight_fail), alert).line());
    } else {
        lines.push(blank());
    }

    let timed_out: Vec<&str> = [
        (st.position_timeout_active, "POSITION"),
        (st.gps_timeout_active, "GPS"),
        (st.ekf_timeout_active, "EKF"),
        (st.rc_timeout_active, "RC"),
    ]
    .iter()
    .filter(|(a, _)| *a)
    .map(|(_, n)| *n)
    .collect();
    if !timed_out.is_empty() {
        lines.push(
            Ln::new()
                .raw(" ")
                .st(format!("ALERT: {} TELEMETRY TIMEOUT", timed_out.join(", ")), alert)
                .line(),
        );
    } else {
        lines.push(blank());
    }

    if st.fence_breach_status != 0 {
        lines.push(
            Ln::new()
                .raw(" ")
                .st(format!("ALERT: GEOFENCE BREACH (count {})", st.fence_breach_count), alert)
                .line(),
        );
    } else {
        lines.push(blank());
    }

    lines.extend(host_lines(ctx));

    lines.push(
        Ln::new()
            .raw(" FLIGHT: ")
            .key("a", "arm")
            .key("d", "disarm")
            .key("T", "takeoff")
            .key("L", "land")
            .key("R", "RTL")
            .key("h", "hold")
            .key("F", if ctx.st.mode == "OFFBOARD" { "exit offboard" } else { "offboard" })
            .key("m", "mode")
            .key("E", "EKF reset")
            .dim("goto/jog on the [n] map")
            .line(),
    );
    lines.push(
        Ln::new()
            .raw(" SAFETY: ")
            .st("[K]", Style::new().fg(RED).add_modifier(Modifier::BOLD))
            .raw("kill  ")
            .key("H", "set home")
            .key("G", "geofence")
            .line(),
    );

    // --- POSITION / VELOCITY -------------------------------------------
    lines.push(blank());
    lines.push(section("POSITION / VELOCITY", ""));

    let (lx, ly, lz) = (st.local_x, st.local_y, st.local_z);
    if st.local_pos_valid {
        lines.push(
            Ln::new()
                .raw(format!("   LOCAL (EKF):  N {lx:9.3}   E {ly:9.3}   D {lz:9.3} m"))
                .dim(format!("  {}", age_text(st.last_local_pos)))
                .line(),
        );
    } else {
        lines.push(Ln::new().raw("   LOCAL (EKF):  ").dim("no LOCAL_POSITION_NED / ODOMETRY yet").line());
    }

    let gps_local = (st.global_pos_valid && st.local_origin_set)
        .then(|| global_to_local(st.global_lat, st.global_lon, st.local_origin_lat, st.local_origin_lon))
        .filter(|(n, e)| n.is_finite() && e.is_finite());

    if let Some((gn, ge)) = gps_local {
        lines.push(
            Ln::new()
                .raw(format!("   GPS (global): N {gn:9.3}   E {ge:9.3}   Alt {:6.2} m", st.global_alt))
                .dim(format!("  {}", age_text(st.last_gps)))
                .line(),
        );
    } else if st.global_pos_valid {
        lines.push(
            Ln::new()
                .raw("   GPS (global): ")
                .dim(format!(
                    "{:.7}, {:.7}  (no local origin - can't grid it)",
                    st.global_lat, st.global_lon
                ))
                .line(),
        );
    } else {
        lines.push(Ln::new().raw("   GPS (global): ").dim("no GLOBAL_POSITION_INT yet").line());
    }

    match gps_local {
        Some((gn, ge)) if st.local_pos_valid => {
            let (dn, de) = (gn - lx, ge - ly);
            let dist = dn.hypot(de);
            let c = if dist < 0.5 { GREEN } else if dist < 2.0 { YELLOW } else { RED };
            lines.push(
                Ln::new()
                    .raw("   EKF <-> GPS offset: ")
                    .fg(format!("{dist:5.2} m"), c)
                    .raw(format!("   (N {dn:+.2}  E {de:+.2})"))
                    .line(),
            );
        }
        _ => lines.push(blank()),
    }

    let (vx, vy, vz) = (st.vx, st.vy, st.vz);
    lines.push(Line::from(format!("   VX: {vx:9.3}   VY: {vy:9.3}   VZ: {vz:9.3} m/s")));
    lines.push(Line::from(format!(
        "   |V|: {:8.3} m/s   Horiz: {:8.3} m/s   Ground: {:7.2} m/s   Climb: {:+7.2} m/s",
        (vx * vx + vy * vy + vz * vz).sqrt(),
        vx.hypot(vy),
        st.groundspeed,
        st.climb_rate
    )));

    // --- ATTITUDE / CONTROL ---------------------------------------------
    lines.push(blank());
    lines.push(section("ATTITUDE / CONTROL", ""));
    lines.push(Line::from(format!(
        "   Roll:  {:8.2} deg   Pitch: {:8.2} deg   Yaw:   {:8.2} deg",
        st.roll, st.pitch, st.yaw
    )));
    lines.push(Line::from(format!(
        "   Rates R/P/Y: {:+6.1} / {:+6.1} / {:+6.1} deg/s   Throttle: {:5.1} %   Thrust sp: {:.2}",
        st.roll_rate, st.pitch_rate, st.yaw_rate, st.thrust, st.att_target_thrust
    )));
    if st.last_nav_output > 0.0 {
        lines.push(Line::from(format!(
            "   Nav: WP dist {:7.1} m   Alt err {:+6.1} m   XTrack {:+6.1} m",
            st.nav_wp_dist, st.nav_alt_error, st.nav_xtrack_error
        )));
    }

    // --- ALTITUDE --------------------------------------------------------
    lines.push(blank());
    lines.push(section("ALTITUDE", ""));
    let amsl = if st.last_altitude > 0.0 { st.alt_amsl } else { st.vfr_alt };
    let agl = [
        st.alt_bottom_clearance,
        st.alt_terrain,
        if st.last_rangefinder > 0.0 { st.rangefinder_distance } else { 0.0 },
    ]
    .into_iter()
    .find(|v| *v != 0.0);
    let agl = match agl {
        Some(v) => Ln::new().raw(format!("{v:.2} m")),
        None => Ln::new().dim("--"),
    };
    let hgt_ref = match st.param_value("EKF2_HGT_REF") {
        Some(v) => Ln::new().raw(ekf2_hgt_ref_name(v.round() as i64)),
        None => Ln::new().dim("?"),
    };
    lines.push(Line::from(format!(
        "   AMSL: {amsl:8.2} m   Rel: {:8.2} m   Local: {:8.2} m",
        st.alt_relative, st.alt_local
    )));
    lines.push(
        Ln::new()
            .raw("   AGL: ")
            .spans(agl.0)
            .raw(format!("   Baro: {:.1} hPa   Height ref: ", st.baro_pressure))
            .spans(hgt_ref.0)
            .line(),
    );

    // --- RANGEFINDER -----------------------------------------------------
    lines.push(blank());
    lines.push(section("RANGEFINDER", ""));
    let rf_age = (st.last_rangefinder > 0.0).then_some(t - st.last_rangefinder);
    let (use_text, use_color) =
        rangefinder_status(st.param_value("EKF2_RNG_CTRL"), st.ekf_flags, st.rangefinder_orientation, rf_age);
    if st.last_rangefinder > 0.0 {
        let q = st.rangefinder_quality;
        let quality = if q >= 0 {
            Ln::new().fg(format!("{q}%"), graded(q as f64, RANGEFINDER_SIGNAL_GOOD, RANGEFINDER_SIGNAL_OK, true))
        } else {
            Ln::new().dim("n/a")
        };
        lines.push(
            Ln::new()
                .raw(format!(
                    "   Raw: {:.2} m   (range {:.2}-{:.2} m)   Facing: {}   Signal: ",
                    st.rangefinder_distance,
                    st.rangefinder_min,
                    st.rangefinder_max,
                    distance_orientation_name(st.rangefinder_orientation)
                ))
                .spans(quality.0)
                .dim(format!("  {}", age_text(st.last_rangefinder)))
                .line(),
        );
    } else {
        lines.push(dim_line("no DISTANCE_SENSOR messages received"));
    }
    lines.push(Ln::new().raw("   Used by EKF: ").spans(dim_or(use_color, use_text).0).line());

    // --- BATTERY ---------------------------------------------------------
    lines.push(blank());
    lines.push(section("BATTERY", ""));
    let level = if st.battery < 0.0 {
        Ln::new().raw("--")
    } else if st.battery <= BATTERY_CRITICAL {
        Ln::new().st(format!("{:.0}%", st.battery), Style::new().fg(RED).add_modifier(Modifier::BOLD))
    } else if st.battery <= BATTERY_LOW {
        Ln::new().fg(format!("{:.0}%", st.battery), YELLOW)
    } else {
        Ln::new().fg(format!("{:.0}%", st.battery), GREEN)
    };
    let volts = if st.voltage >= 0.0 { format!("{:.2} V", st.voltage) } else { "--".into() };
    let amps = if st.current >= 0.0 { format!("{:.2} A", st.current) } else { "--".into() };
    lines.push(
        Ln::new()
            .raw("   Level: ")
            .spans(level.0)
            .raw(format!("   Voltage: {volts}   Current: {amps}"))
            .line(),
    );

    // --- NAVIGATION / ESTIMATION -----------------------------------------
    lines.push(blank());
    lines.push(section("NAVIGATION / ESTIMATION", ""));
    let (fix_color, fix_name) = gps_fix_display(st.gps_fix);
    let lower_better = |v: f64, good: f64, ok: f64| -> Option<Color> {
        (v > 0.0).then(|| graded(v, good, ok, false))
    };
    lines.push(
        Ln::new()
            .raw("   GPS: ")
            .fg(fix_name, fix_color)
            .raw("   Sats: ")
            .fg(st.gps_sats.to_string(), graded(st.gps_sats as f64, GPS_SATS_GOOD, GPS_SATS_OK, true))
            .raw("   HDOP: ")
            .spans(dim_or(lower_better(st.gps_hdop, GPS_HDOP_GOOD, GPS_HDOP_OK), format!("{:.2}", st.gps_hdop)).0)
            .raw("   VDOP: ")
            .spans(dim_or(lower_better(st.gps_vdop, GPS_HDOP_GOOD, GPS_HDOP_OK), format!("{:.2}", st.gps_vdop)).0)
            .raw("   EPH: ")
            .spans(dim_or(lower_better(st.gps_h_acc, POS_ACC_GOOD, POS_ACC_OK), format!("{:.2} m", st.gps_h_acc)).0)
            .raw("   EPV: ")
            .spans(dim_or(lower_better(st.gps_v_acc, POS_ACC_GOOD, POS_ACC_OK), format!("{:.2} m", st.gps_v_acc)).0)
            .raw("   RTK: ")
            .spans(rtk_text(st.gps_fix).0)
            .line(),
    );
    if st.global_pos_valid {
        lines.push(Line::from(format!(
            "   Position: Lat {:.7}°   Lon {:.7}°   Alt {:.2} m AMSL",
            st.global_lat, st.global_lon, st.global_alt
        )));
    } else {
        lines.push(Ln::new().raw("   Position: ").dim("no global position").line());
    }

    let (verdict, verdict_color) = ekf_summary(st.ekf_flags, st.est_is_estimator_status, &st.ekf_status);
    let home = if st.home_set { Ln::new().fg("SET", GREEN) } else { Ln::new().fg("NOT SET", RED) };
    let fence = if st.last_fence_status > 0.0 {
        if st.fence_breach_status != 0 {
            Ln::new().st(format!("BREACH ({})", st.fence_breach_count), alert)
        } else {
            Ln::new().fg("OK", GREEN)
        }
    } else {
        Ln::new().dim("n/a")
    };
    lines.push(
        Ln::new()
            .raw("   EKF: ")
            .fg(verdict, verdict_color)
            .raw("   PosAcc H/V: ")
            .fg(
                format!("{:.2}", st.est_pos_horiz_accuracy),
                graded(st.est_pos_horiz_accuracy, POS_ACC_GOOD, POS_ACC_OK, false),
            )
            .raw("/")
            .fg(
                format!("{:.2}", st.est_pos_vert_accuracy),
                graded(st.est_pos_vert_accuracy, POS_ACC_GOOD, POS_ACC_OK, false),
            )
            .raw(" m   Home: ")
            .spans(home.0)
            .raw("   Fence: ")
            .spans(fence.0)
            .line(),
    );

    let rc_conn = if !st.rc_received {
        Ln::new().dim("NOT CONNECTED")
    } else if st.rc_timeout_active {
        Ln::new().st("NO SIGNAL", alert)
    } else {
        Ln::new().fg("CONNECTED", GREEN)
    };
    let rssi = if st.rc_received && st.rc_rssi >= 0 {
        Ln::new().fg(format!("{}%", st.rc_rssi), graded(st.rc_rssi as f64, RC_SIGNAL_GOOD, RC_SIGNAL_OK, true))
    } else {
        Ln::new().dim("--")
    };
    let rc_fs = if st.rc_failsafe { Ln::new().fg("YES", RED) } else { Ln::new().fg("NO", GREEN) };
    lines.push(
        Ln::new()
            .raw("   RC: ")
            .spans(rc_conn.0)
            .raw("   RSSI: ")
            .spans(rssi.0)
            .raw("   Failsafe: ")
            .spans(rc_fs.0)
            .line(),
    );

    // --- SENSORS ---------------------------------------------------------
    lines.push(blank());
    lines.push(section("SENSORS", ""));
    if st.sensors_present != 0 {
        let items: Vec<(&str, Option<Color>)> = SYS_STATUS_SENSOR_LABELS
            .iter()
            .filter(|(bit, _)| st.sensors_present & bit != 0)
            .map(|&(bit, label)| {
                let color = if st.sensors_enabled & bit == 0 {
                    None
                } else if st.sensors_health & bit != 0 {
                    Some(GREEN)
                } else {
                    Some(RED)
                };
                (label, color)
            })
            .collect();
        if items.is_empty() {
            lines.push(dim_line("none reported"));
        }
        for chunk in items.chunks(6) {
            let mut l = Ln::new().raw("   ");
            for (i, (label, color)) in chunk.iter().enumerate() {
                if i > 0 {
                    l = l.raw("   ");
                }
                l = l.spans(dim_or(*color, label.to_string()).0);
            }
            lines.push(l.line());
        }
    } else {
        let ok = |v: bool| if v { Ln::new().fg("OK", GREEN) } else { Ln::new().fg("FAIL", RED) };
        lines.push(
            Ln::new()
                .raw("   IMU: ")
                .spans(ok(st.imu).0)
                .raw("   MAG: ")
                .spans(ok(st.mag).0)
                .raw("   BARO: ")
                .spans(ok(st.baro).0)
                .raw("   GPS: ")
                .spans(ok(st.gps_sensor).0)
                .line(),
        );
    }
    if st.last_vibration > 0.0 {
        let (verdict, color) = vibration_verdict(st);
        let clip_total: u32 = st.clipping.iter().sum();
        let mut l = Ln::new()
            .raw("   Vibration: ")
            .fg(verdict, color)
            .raw(format!(
                "   X {:.1}  Y {:.1}  Z {:.1} m/s²",
                st.vibration_x, st.vibration_y, st.vibration_z
            ));
        if clip_total > 0 {
            l = l.raw("   clipping: ").fg(clip_total.to_string(), RED);
        }
        lines.push(l.line());
    } else {
        lines.push(Ln::new().raw("   Vibration: ").dim("no VIBRATION messages yet").line());
    }

    // --- SYSTEM ----------------------------------------------------------
    lines.push(blank());
    lines.push(section("SYSTEM", ""));
    let rx = if st.connected {
        Ln::new().fg(format!("{:.0} msg/s", st.rx_rate), graded(st.rx_rate, RX_RATE_GOOD, RX_RATE_OK, true))
    } else {
        Ln::new().dim(format!("{:.0} msg/s", st.rx_rate))
    };
    let count = |n: u32, c: Color| if n > 0 { Ln::new().fg(n.to_string(), c) } else { Ln::new().dim(n.to_string()) };
    lines.push(
        Ln::new()
            .raw("   MAVLink RX: ")
            .spans(rx.0)
            .raw("   Warnings: ")
            .spans(count(st.warning_count, YELLOW).0)
            .raw("   Errors: ")
            .spans(count(st.error_count, RED).0)
            .raw("   Failsafes: ")
            .spans(count(st.failsafe_count, RED).0)
            .line(),
    );
    if !st.parameters.is_empty() {
        let changed = st.parameters.values().filter(|p| p.is_changed()).count();
        lines.push(Line::from(format!("   Parameters: {}   Changed: {changed}", st.parameters.len())));
    }
    if !st.last_ack.is_empty() {
        lines.push(Line::from(format!("   Last ACK: {}", st.last_ack)));
    }
    if !st.last_status_text.is_empty() {
        let style = st.last_status_level.map(level_color).unwrap_or_default();
        lines.push(Ln::new().raw("   PX4: ").st(st.last_status_text.clone(), style).line());
    }

    lines.push(blank());
    lines.push(section("SCREENS", ""));
    lines.push(Line::from("   [m] MODE   [s] CALIBRATE   [n] MISSION (map, goto + jog)   [c] CONTROL"));
    lines.push(Line::from("   [p] PARAMETERS   [g] EVENT LOG   [l] FLIGHT LOGS   [t] NSH   [u] USB/NETWORK"));
    lines.push(Line::from("   [f] FLASH FIRMWARE   [?] ABOUT   [q] EXIT   [TAB] panels"));
    lines
}
