//! Page-style screens: control / RC, sensor calibration, about, and the
//! placeholder for screens not ported from the Python version yet.

use ratatui::style::{Color, Modifier, Style};
use ratatui::text::Line;

use super::*;
use crate::state::now;

const STICK_WIDTH: usize = 19;
const STICK_HEIGHT: usize = 9;
const BAR_WIDTH: usize = 32;

fn age(last: f64) -> Ln {
    if last == 0.0 {
        return Ln::new().dim("never");
    }
    let d = now() - last;
    let c = if d < 2.0 { GREEN } else if d < 10.0 { YELLOW } else { RED };
    Ln::new().fg(format!("{d:.1}s"), c)
}

fn section_with(title: &str, note: Ln) -> Line<'static> {
    let mut spans = section(title, "").spans;
    spans.push(ratatui::text::Span::raw(" "));
    spans.extend(note.0);
    Line::from(spans)
}

fn err(v: f64) -> Ln {
    let c = if v.abs() < 1.0 { GREEN } else if v.abs() < 5.0 { YELLOW } else { RED };
    Ln::new().fg(format!("{v:+.2}"), c)
}

/// Shortest signed angle target - actual, degrees.
fn ang_err(target: f64, actual: f64) -> Ln {
    err((target - actual + 180.0).rem_euclid(360.0) - 180.0)
}

/// -1..1 for a centre-sprung channel.
fn centered_frac(raw: u16) -> f64 {
    if raw == 0 {
        return 0.0;
    }
    ((raw as f64 - RC_PWM_CENTER as f64) / ((RC_PWM_MAX - RC_PWM_MIN) as f64 / 2.0)).clamp(-1.0, 1.0)
}

/// Throttle has no centre spring: bottom..top maps to -1..1.
fn throttle_frac(raw: u16) -> f64 {
    if raw == 0 {
        return -1.0;
    }
    let unit = ((raw as f64 - RC_PWM_MIN as f64) / (RC_PWM_MAX - RC_PWM_MIN) as f64).clamp(0.0, 1.0);
    unit * 2.0 - 1.0
}

fn stick_grid(fx: f64, fy: f64) -> Vec<Vec<(char, bool)>> {
    let (w, h) = (STICK_WIDTH, STICK_HEIGHT);
    let (cc, cr) = (w / 2, h / 2);
    let mut grid = vec![vec![('·', false); w]; h];
    grid[cr].fill(('─', false));
    for row in grid.iter_mut() {
        row[cc] = ('│', false);
    }
    grid[cr][cc] = ('┼', false);
    let col = (((fx + 1.0) / 2.0 * (w - 1) as f64).round() as usize).min(w - 1);
    let row = (((1.0 - fy) / 2.0 * (h - 1) as f64).round() as usize).min(h - 1);
    grid[row][col] = ('●', true);
    grid
}

fn grid_spans(row: &[(char, bool)], l: Ln) -> Ln {
    row.iter().fold(l, |l, &(c, dot)| {
        if dot {
            l.st(c.to_string(), Style::new().fg(GREEN).add_modifier(Modifier::BOLD))
        } else {
            l.dim(c.to_string())
        }
    })
}

fn bar(raw: u16) -> Ln {
    if raw == 0 {
        return Ln::new().dim("·".repeat(BAR_WIDTH)).raw("   ").dim("unused");
    }
    let frac = ((raw as f64 - RC_PWM_DISPLAY_MIN as f64) / (RC_PWM_DISPLAY_MAX - RC_PWM_DISPLAY_MIN) as f64).clamp(0.0, 1.0);
    let filled = (frac * BAR_WIDTH as f64).round() as usize;
    let pct = (raw as f64 - RC_PWM_MIN as f64) / (RC_PWM_MAX - RC_PWM_MIN) as f64 * 100.0;
    Ln::new()
        .fg("█".repeat(filled), GREEN)
        .dim("░".repeat(BAR_WIDTH - filled))
        .raw(format!("   {raw:4} us  {pct:5.1}%"))
}

/// POSITION_TARGET_TYPEMASK bits mark axes to *ignore*; invert that.
fn pos_target_active_axes(mask: u16) -> Vec<&'static str> {
    let groups: [(&str, u16); 5] = [("pos", 1 | 2 | 4), ("vel", 8 | 16 | 32), ("acc", 64 | 128 | 256), ("yaw", 1024), ("yawrate", 2048)];
    let mut active: Vec<&str> = groups.iter().filter(|(_, bits)| mask & bits != *bits).map(|(n, _)| *n).collect();
    if mask & 512 != 0 {
        active.push("force");
    }
    active
}

pub fn control(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let mut lines = vec![Ln::new().raw(" MODE: ").bold(st.mode.clone()).raw("    ").spans(vec![armed_span(st.armed)]).line()];

    lines.push(blank());
    lines.push(section_with("ATTITUDE CONTROL", Ln::new().dim("setpoint ").spans(age(st.last_att_target).0)));
    lines.push(Line::from(format!("   {:10}{:>10}{:>10}{:>10}", "", "roll", "pitch", "yaw")));
    lines.push(Line::from(format!("   {:<10}{:>10.2}{:>10.2}{:>10.2}  deg", "actual", st.roll, st.pitch, st.yaw)));
    lines.push(Line::from(format!(
        "   {:<10}{:>10.2}{:>10.2}{:>10.2}  deg",
        "target", st.att_target_roll, st.att_target_pitch, st.att_target_yaw
    )));
    lines.push(
        Ln::new()
            .raw("   error      roll ")
            .spans(ang_err(st.att_target_roll, st.roll).0)
            .raw("   pitch ")
            .spans(ang_err(st.att_target_pitch, st.pitch).0)
            .raw("   yaw ")
            .spans(ang_err(st.att_target_yaw, st.yaw).0)
            .raw("  deg")
            .line(),
    );
    lines.push(blank());
    lines.push(Line::from(format!(
        "   {:<10}{:>10.1}{:>10.1}{:>10.1}  deg/s",
        "rate now", st.roll_rate, st.pitch_rate, st.yaw_rate
    )));
    lines.push(Line::from(format!(
        "   {:<10}{:>10.1}{:>10.1}{:>10.1}  deg/s",
        "rate sp", st.att_target_roll_rate, st.att_target_pitch_rate, st.att_target_yaw_rate
    )));
    lines.push(Line::from(format!(
        "   throttle (VFR_HUD): {:5.1} %      thrust setpoint: {:.3}",
        st.thrust, st.att_target_thrust
    )));

    lines.push(blank());
    lines.push(section_with("POSITION / VELOCITY SETPOINT", age(st.last_pos_target)));
    if st.last_pos_target > 0.0 {
        let axes = pos_target_active_axes(st.pos_target_type_mask);
        let axes = if axes.is_empty() { Ln::new().dim("none") } else { Ln::new().raw(axes.join(", ")) };
        lines.push(
            Ln::new()
                .raw(format!("   frame: {}   controlling: ", pos_target_frame_name(st.pos_target_frame)))
                .spans(axes.0)
                .line(),
        );
        lines.push(Line::from(format!("   {:10}{:>11}{:>11}{:>11}", "", "x", "y", "z")));
        let row = |label: &str, a: f64, b: f64, c: f64, unit: &str| {
            Line::from(format!("   {label:<10}{a:>11.2}{b:>11.2}{c:>11.2}  {unit}"))
        };
        lines.push(row("pos now", st.x, st.y, st.z, "m"));
        lines.push(row("pos sp", st.pos_target_x, st.pos_target_y, st.pos_target_z, "m"));
        lines.push(row("vel now", st.vx, st.vy, st.vz, "m/s"));
        lines.push(row("vel sp", st.pos_target_vx, st.pos_target_vy, st.pos_target_vz, "m/s"));
        lines.push(Line::from(format!(
            "   yaw sp: {:+.1} deg   yaw-rate sp: {:+.1} deg/s",
            st.pos_target_yaw, st.pos_target_yaw_rate
        )));
    } else {
        lines.push(dim_line("no POSITION_TARGET_LOCAL_NED (manual / rate mode, or not streamed)"));
    }

    lines.push(blank());
    lines.push(section_with("GUIDANCE · NAV_CONTROLLER_OUTPUT", age(st.last_nav_output)));
    if st.last_nav_output > 0.0 {
        lines.push(
            Ln::new()
                .raw(format!("   WP distance: {:.1} m   altitude error: ", st.nav_wp_dist))
                .spans(err(st.nav_alt_error).0)
                .raw(" m   crosstrack: ")
                .spans(err(st.nav_xtrack_error).0)
                .raw(" m")
                .line(),
        );
        lines.push(Line::from(format!(
            "   nav bearing: {:.0} deg   target bearing: {:.0} deg   airspeed error: {:+.1} m/s",
            st.nav_bearing, st.nav_target_bearing, st.nav_aspd_error
        )));
    } else {
        lines.push(dim_line("not in an auto / mission mode"));
    }

    // --- RC --------------------------------------------------------------
    lines.push(blank());
    let bold_red = Style::new().fg(RED).add_modifier(Modifier::BOLD);
    let conn = if !st.rc_received {
        Ln::new().dim("NOT CONNECTED")
    } else if st.rc_timeout_active {
        Ln::new().st("NO SIGNAL", bold_red)
    } else {
        Ln::new().fg("CONNECTED", GREEN)
    };
    let fs = if st.rc_failsafe { Ln::new().st("FAILSAFE", bold_red) } else { Ln::new().fg("ok", GREEN) };
    let rssi = if st.rc_received && st.rc_rssi >= 0 {
        Ln::new().fg(format!("{}%", st.rc_rssi), graded(st.rc_rssi as f64, RC_SIGNAL_GOOD, RC_SIGNAL_OK, true))
    } else {
        Ln::new().dim("--")
    };
    lines.push(section_with("RC INPUT", age(st.last_rc)));
    lines.push(Ln::new().raw("   ").spans(conn.0).raw("   RSSI: ").spans(rssi.0).raw("   ").spans(fs.0).line());

    let ch = st.rc_channels;
    if !st.rc_received || ch.iter().all(|&c| c == 0) {
        lines.push(dim_line("no RC_CHANNELS received yet"));
    } else {
        lines.push(blank());
        lines.push(Line::from(format!("   {:<w$}   RIGHT: roll / pitch", "LEFT: yaw / throttle", w = STICK_WIDTH)));
        let left = stick_grid(centered_frac(ch[3]), throttle_frac(ch[2]));
        let right = stick_grid(centered_frac(ch[0]), centered_frac(ch[1]));
        for (l, r) in left.iter().zip(&right) {
            lines.push(grid_spans(r, grid_spans(l, Ln::new().raw("   ")).raw("   ")).line());
        }
        lines.push(Line::from(format!(
            "   CH1 roll: {:4}   CH2 pitch: {:4}   CH3 thr: {:4}   CH4 yaw: {:4}",
            ch[0], ch[1], ch[2], ch[3]
        )));
        lines.push(blank());
        let active = ch.iter().filter(|&&c| c > 0).count();
        lines.push(section("ALL CHANNELS", &format!("{active}/{} active", ch.len())));
        for (i, &raw) in ch.iter().enumerate() {
            lines.push(Ln::new().raw(format!("   CH{:<2} ", i + 1)).spans(bar(raw).0).line());
        }
    }

    lines.push(blank());
    lines.push(section_with("WIND", age(st.last_wind)));
    if st.last_wind > 0.0 {
        lines.push(Line::from(format!(
            "   Horizontal: {:5.1} m/s   from {:5.1} deg   Vertical: {:+5.1} m/s",
            st.wind_speed, st.wind_direction, st.wind_speed_z
        )));
    } else {
        lines.push(dim_line("no WIND / WIND_COV messages yet"));
    }

    lines.push(blank());
    lines.push(section_with("ACTUATOR OUTPUTS", age(st.last_servo_output)));
    if st.last_servo_output > 0.0 {
        let row = |range: std::ops::Range<usize>| {
            let cells: Vec<String> = range.map(|i| format!("S{} {:4}", i + 1, st.servo_outputs[i])).collect();
            Line::from(format!("   {}  us", cells.join("  ")))
        };
        lines.push(row(0..4));
        if st.servo_outputs[4..].iter().any(|&v| v > 0) {
            lines.push(row(4..8));
        }
    } else {
        lines.push(dim_line("no SERVO_OUTPUT_RAW messages yet"));
    }
    lines
}

pub fn calibration(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let t = now();
    let mut lines = vec![Ln::new().raw(" Vehicle: ").spans(vec![armed_span(st.armed)]).line()];
    lines.push(Line::from(format!(
        " Link: port {}   STATUSTEXT seen: {}   EVENT seen: {}",
        ctx.app.settings.port, st.statustext_count, st.event_count
    )));
    lines.push(blank());

    if st.cal_active {
        let label = calibration_label(&st.cal_type).to_uppercase();
        lines.push(
            Ln::new()
                .st(format!(" CALIBRATING: {label}"), Style::new().fg(YELLOW).add_modifier(Modifier::BOLD))
                .raw(format!("   {:.0}s", t - st.cal_started_at))
                .line(),
        );
        let width = 40usize;
        let filled = (st.cal_progress.clamp(0, 100) as usize * width) / 100;
        lines.push(
            Ln::new()
                .raw(" [")
                .fg("█".repeat(filled), GREEN)
                .dim("░".repeat(width - filled))
                .raw(format!("] {:3}%", st.cal_progress))
                .line(),
        );
        if !st.cal_ack.is_empty() {
            let c = if st.cal_ack == "ACCEPTED" || st.cal_ack == "IN_PROGRESS" { GREEN } else { RED };
            lines.push(Ln::new().raw(" PX4 command ACK: ").fg(st.cal_ack.clone(), c).line());
        } else {
            lines.push(Ln::new().dim(format!(" PX4 command ACK: waiting ({:.0}s)", t - st.cal_sent_at)).line());
        }
        lines.push(blank());
        lines.push(Ln::new().bold(" PX4 [cal] messages:").line());
        if st.cal_log.is_empty() {
            lines.push(dim_line("(none yet)"));
        }
        for m in &st.cal_log {
            lines.push(Line::from(format!("   {m}")));
        }
        lines.push(blank());
        if st.cal_log.is_empty() && !st.cal_ack.is_empty() {
            lines.push(Ln::new().fg(" PX4 accepted it but has sent no [cal] progress yet.", YELLOW).line());
            lines.push(Ln::new().dim(" If nothing arrives, check STATUSTEXT reaches this link ([r] re-requests it).").line());
        } else {
            lines.push(Ln::new().fg(" Follow the prompts above, then wait for 'calibration done'.", YELLOW).line());
        }
        lines.push(Ln::new().dim(" [x]/ESC cancel").line());
        return lines;
    }

    if !st.cal_result.is_empty() {
        let label = calibration_label(&st.cal_type).to_string();
        let l = match st.cal_result.as_str() {
            "DONE" => Ln::new().fg(format!(" Last calibration: DONE ({label})"), GREEN),
            "CANCELLED" => Ln::new().fg(" Last calibration: CANCELLED", YELLOW),
            r => Ln::new().fg(format!(" Last calibration: {r}"), RED),
        };
        lines.push(l.line());
        if !st.cal_last_message.is_empty() {
            lines.push(dim_line(st.cal_last_message.clone()));
        }
        lines.push(blank());
    }

    lines.push(Ln::new().bold(" Select a calibration (vehicle must be DISARMED):").line());
    lines.push(blank());
    for (key, name, hint) in [
        ("g", "Gyroscope", "keep still"),
        ("a", "Accelerometer", "6 orientations"),
        ("l", "Level horizon", "level and still"),
        ("c", "Compass / magnetometer", "rotate about all axes"),
        ("b", "Barometer", "keep still"),
    ] {
        lines.push(Ln::new().raw(format!("   [{key}]  {name:<24}")).dim(hint).line());
    }
    lines.push(blank());
    lines.push(Ln::new().dim(" Accel and compass need you to physically move the airframe;").line());
    lines.push(Ln::new().dim(" PX4 detects each position automatically and reports progress here.").line());
    lines.push(Ln::new().dim(" A reboot ([b] on the parameter screen) is recommended afterwards.").line());
    lines
}

const LOGO: [&str; 4] = [
    "▌  ▞▀▖▀▀▌▌ ▌▛▀▖▌ ▌▌ ▌",
    "▌  ▙▄▌ ▞ ▝▞ ▙▄▘▝▞ ▚▄▌",
    "▌  ▌ ▌▞   ▌ ▌  ▞▝▖  ▌",
    "▀▀▘▘ ▘▀▀▘ ▘ ▘  ▘ ▘  ▘",
];

pub fn about() -> Vec<Line<'static>> {
    let logo = Style::new().fg(CYAN).add_modifier(Modifier::BOLD);
    let link = Style::new().fg(Color::Magenta);
    let mut lines = vec![blank()];
    lines.extend(LOGO.iter().map(|row| Ln::new().raw("  ").st(*row, logo).line()));
    lines.push(blank());
    lines.push(Ln::new().raw("  ").dim("a lazygit-style terminal UI for PX4 over MAVLink - Rust / ratatui port").line());
    lines.push(blank());
    lines.push(section("AUTHOR", ""));
    lines.push(Ln::new().raw("   Manuel Boldrer  ").st("https://manuelboldrer.github.io/", link).line());
    lines.push(Line::from("   manuel.boldrer@gmail.com"));
    lines.push(blank());
    lines.push(section("ACKNOWLEDGMENTS", ""));
    lines.push(Line::from("   Saxion University of Applied Sciences"));
    lines.push(Line::from("   Smart Mechatronics and Robotics Group"));
    lines.push(
        Ln::new()
            .raw("   ")
            .st("https://www.saxion.edu/research/research-groups/smart-mechatronics-and-robotics", link)
            .line(),
    );
    lines.push(blank());
    lines.push(section("VERSION", ""));
    lines.push(Line::from(format!("   lazypx4-rs {}", env!("CARGO_PKG_VERSION"))));
    lines
}
