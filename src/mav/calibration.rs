//! Sensor calibration (MAV_CMD_PREFLIGHT_CALIBRATION).
//!
//! PX4 runs gyro / accel / level / mag / baro calibration on-board: we send
//! one command and follow the "[cal] ..." STATUSTEXT stream for progress and
//! the "place the vehicle nose-down" style prompts. Must be done DISARMED.

use mavlink::dialects::development::MavCmd;

use crate::config::{calibration_label, calibration_params};
use crate::link::Link;
use crate::mav::commands::vehicle_ready;
use crate::mav::handlers::push_bounded;
use crate::state::{self, Shared, State, now};

/// Fold a "[cal]" STATUSTEXT line into the calibration state.
pub fn handle_calibration_text(st: &mut State, text: &str) {
    let lower = text.to_lowercase();
    st.cal_last_message = text.to_string();
    push_bounded(&mut st.cal_log, text.to_string(), 14);

    // "progress <NN>"
    if let Some(pos) = lower.find("progress") {
        let digits: String = lower[pos + 8..]
            .trim_start_matches(|c: char| c.is_whitespace() || c == '<')
            .chars()
            .take_while(char::is_ascii_digit)
            .collect();
        if let Ok(p) = digits.parse::<i32>() {
            st.cal_progress = p.clamp(0, 100);
        }
    }

    if lower.contains("calibration started") {
        st.cal_active = true;
        st.cal_progress = 0;
        st.cal_result.clear();
        st.cal_started_at = now();
    } else if lower.contains("calibration done") {
        st.cal_active = false;
        st.cal_progress = 100;
        st.cal_result = "DONE".into();
    } else if lower.contains("calibration failed") {
        st.cal_active = false;
        st.cal_result = "FAILED".into();
    } else if lower.contains("calibration cancel") {
        st.cal_active = false;
        st.cal_result = "CANCELLED".into();
    }
}

pub fn send_calibration(link: &Link, shared: &Shared, kind: &str) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    let Some(params) = calibration_params(kind) else {
        st.error(format!("Unknown calibration type: {kind}"));
        return false;
    };
    if st.armed {
        st.error("Calibration refused: vehicle is ARMED");
        return false;
    }
    if st.cal_active {
        st.warn("A calibration is already running");
        return false;
    }
    if !link.command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, params) {
        st.error("Failed to send calibration command");
        return false;
    }

    let t = now();
    st.cal_active = true;
    st.cal_type = kind.to_string();
    st.cal_progress = 0;
    st.cal_result.clear();
    st.cal_started_at = t;
    st.cal_sent_at = t;
    st.cal_ack.clear();
    st.cal_last_message.clear();
    st.cal_log.clear();

    let (sys, comp) = link.target();
    st.command(format!(
        "PREFLIGHT_CALIBRATION SENT: {} (sys {sys} comp {comp})",
        calibration_label(kind)
    ));
    true
}

pub fn cancel_calibration(link: &Link, shared: &Shared) -> bool {
    let mut st = state::lock(shared);
    if !link.command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, [0.; 7]) {
        st.error("Failed to cancel calibration");
        return false;
    }
    st.cal_active = false;
    st.cal_result = "CANCELLED".into();
    st.command("Calibration CANCEL sent");
    true
}

/// A real calibration always produces a COMMAND_ACK or "[cal]" text within
/// seconds; if neither arrives the command did not land, so clear RUNNING.
pub fn check_calibration(st: &mut State) {
    if !st.cal_active || st.cal_sent_at == 0.0 {
        return;
    }
    if !st.cal_ack.is_empty() || !st.cal_log.is_empty() || now() - st.cal_sent_at < 25.0 {
        return;
    }
    st.cal_active = false;
    st.cal_result = "NO RESPONSE".into();
    st.error(
        "Calibration: PX4 sent no ACK and no [cal] messages in 25 s - \
         command was not accepted. Check [g] event log and PX4 firmware.",
    );
}
