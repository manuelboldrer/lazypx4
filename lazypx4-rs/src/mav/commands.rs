//! Link setup and flight-command senders: GCS heartbeat, stream rates,
//! arm/disarm, legacy mode set, hold, kill, set-home, reboot, takeoff, land
//! and RTL. Mirrors `mavlink/connection.py`, `commands.py` and `guided.py`.
//!
//! Every state-changing sender is gated by [`vehicle_ready`], logs what it
//! did, and returns a bool. Arm state is never inferred from an ACK - only a
//! HEARTBEAT confirms it (see the heartbeat handler and
//! [`check_pending_arm`]).

use mavlink::dialects::development::{self as mav, MavCmd, MavMessage};

use crate::config::*;
use crate::link::{Link, char_array};
use crate::px4mode;
use crate::state::{self, PendingArm, Shared, State, now};

/// True only with a locked identity and recent telemetry, so a command is
/// never sent to a vehicle whose link has gone stale.
pub fn vehicle_ready(st: &mut State, link: &Link) -> bool {
    if !link.has_peer() {
        st.error("No MAVLink connection");
        return false;
    }
    if !st.vehicle_locked {
        st.error("Vehicle identity is not locked");
        return false;
    }
    if st.last_rx <= 0.0 {
        st.error("No vehicle telemetry received");
        return false;
    }
    if now() - st.last_rx > HEARTBEAT_TIMEOUT {
        st.error("Vehicle telemetry is stale");
        return false;
    }
    true
}

/// Announce this console as a GCS. PX4 gates several streams (STATUSTEXT
/// among them, on some builds) on a live GCS heartbeat.
pub fn send_gcs_heartbeat(link: &Link) {
    link.send(&MavMessage::HEARTBEAT(mav::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mav::MavType::MAV_TYPE_GCS,
        autopilot: mav::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: mav::MavModeFlag::empty(),
        system_status: mav::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    }));
}

/// Ask PX4 for faster estimation/sensor streams than the defaults, plus the
/// one-shot messages (origin, home, firmware version). Blocking for ~0.5 s
/// because of the pacing, so it's run on a helper thread.
pub fn configure_streams(link: &Link) {
    let pace = || std::thread::sleep(CONFIGURE_STREAMS_PACING);

    for &(msg_id, interval_us) in STREAM_MESSAGE_INTERVALS {
        link.command_long(
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            [msg_id as f32, interval_us as f32, 0., 0., 0., 0., 0.],
        );
        pace();
    }

    // GPS_GLOBAL_ORIGIN, HOME_POSITION: emitted on change; AUTOPILOT_VERSION:
    // only on request.
    for msg_id in [49.0, 242.0, 148.0] {
        link.command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, [msg_id, 0., 0., 0., 0., 0., 0.]);
        pace();
    }

    // Some stream configs omit STATUSTEXT / EVENT, which hides calibration
    // "[cal]" prompts. Force both on (10 Hz cap; event-driven in practice).
    for msg_id in [253.0, 410.0] {
        link.command_long(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, [msg_id, 100_000.0, 0., 0., 0., 0., 0.]);
        pace();
    }
}

pub fn request_param(link: &Link, name: &str) {
    let (target_system, target_component) = link.target();
    link.send(&MavMessage::PARAM_REQUEST_READ(mav::PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system,
        target_component,
        param_id: char_array(name),
    }));
}

/// Individually read the EKF2_* aiding params shown on the dashboard.
pub fn request_estimator_params(link: &Link) {
    for name in ESTIMATOR_PARAM_NAMES {
        request_param(link, name);
    }
}

pub fn send_arm(link: &Link, shared: &Shared, arm: bool) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    let action = if arm { "ARM" } else { "DISARM" };

    if st.armed == arm {
        let actual = if st.armed { "ARMED" } else { "DISARMED" };
        st.command(format!("{action} requested, but vehicle already reports {actual}"));
        return true;
    }
    if st.pending_arm.is_some() {
        st.warn("ARM/DISARM command already pending");
        return false;
    }

    // param2 = 0: never force-arm.
    if !link.command_long(
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        [if arm { 1.0 } else { 0.0 }, 0., 0., 0., 0., 0., 0.],
    ) {
        st.error(format!("Could not send {action}"));
        return false;
    }
    st.pending_arm = Some(PendingArm { desired_armed: arm, sent_at: now() });
    st.command(format!("{action} command SENT"));
    st.command(format!("{action}: waiting for HEARTBEAT confirmation"));
    true
}

/// Give up on an ARM/DISARM no HEARTBEAT confirmed in time.
pub fn check_pending_arm(st: &mut State) {
    let Some(pending) = &st.pending_arm else { return };
    if now() - pending.sent_at < ARM_CONFIRM_TIMEOUT {
        return;
    }
    let desired = pending.desired_armed;
    st.pending_arm = None;
    let action = if desired { "ARM" } else { "DISARM" };

    if st.armed == desired {
        st.command(format!("{action} confirmed by vehicle state"));
    } else if !st.last_status_text.is_empty() {
        let last = st.last_status_text.clone();
        st.error(format!(
            "{action} was not confirmed within {ARM_CONFIRM_TIMEOUT:.1}s. Last PX4 status: {last}"
        ));
    } else {
        st.error(format!("{action} was not confirmed within {ARM_CONFIRM_TIMEOUT:.1}s"));
    }
}

/// Set a legacy (string-named) PX4 flight mode, as pymavlink's `set_mode`.
pub fn set_mode(link: &Link, st: &mut State, mode: &str) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    let Some((base, main, sub)) = px4mode::legacy_mode_params(mode) else {
        st.error(format!("PX4 does not report mode '{mode}'"));
        return false;
    };
    if !link.command_long(MavCmd::MAV_CMD_DO_SET_MODE, [base, main, sub, 0., 0., 0., 0.]) {
        st.error(format!("Failed to set mode {mode}"));
        return false;
    }
    st.command(format!("MODE command SENT: {mode}"));
    true
}

/// Best-effort "stop and loiter": a HOLD-family mode, else DO_PAUSE_CONTINUE.
pub fn send_hold(link: &Link, shared: &Shared) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    let modes = px4mode::legacy_mode_names();
    for requested in ["HOLD", "AUTO.LOITER", "LOITER"] {
        if modes.iter().any(|m| m == requested) && set_mode(link, &mut st, requested) {
            st.command(format!("HOLD using PX4 mode {requested}"));
            return true;
        }
    }
    if link.command_long(MavCmd::MAV_CMD_DO_PAUSE_CONTINUE, [0.; 7]) {
        st.command("HOLD command SENT using MAV_CMD_DO_PAUSE_CONTINUE");
        true
    } else {
        st.error("Failed to send HOLD");
        false
    }
}

/// Plain COMMAND_LONG with success/failure logging.
fn simple_command(link: &Link, shared: &Shared, cmd: MavCmd, params: [f32; 7], ok: &str, fail: &str) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    if link.command_long(cmd, params) {
        st.command(ok);
        true
    } else {
        st.error(fail);
        false
    }
}

/// Force-terminate the flight NOW - cuts motors even mid-flight. Not the
/// same as disarm (which PX4 can refuse in flight); this is the kill switch.
pub fn send_kill(link: &Link, shared: &Shared) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    if link.command_long(MavCmd::MAV_CMD_DO_FLIGHTTERMINATION, [1., 0., 0., 0., 0., 0., 0.]) {
        st.warn("KILL command SENT - flight termination");
        true
    } else {
        st.error("Failed to send KILL");
        false
    }
}

pub fn send_set_home_current(link: &Link, shared: &Shared) -> bool {
    simple_command(
        link, shared,
        MavCmd::MAV_CMD_DO_SET_HOME, [1., 0., 0., 0., 0., 0., 0.],
        "SET HOME command SENT: current position", "Failed to set home",
    )
}

pub fn send_reboot(link: &Link, shared: &Shared) -> bool {
    let sent = simple_command(
        link, shared,
        MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, [1., 0., 0., 0., 0., 0., 0.],
        "PX4 REBOOT command SENT", "Failed to send PX4 reboot",
    );
    if sent {
        state::lock(shared).warn("PX4 is rebooting; MAVLink connection will temporarily disappear");
    }
    sent
}

pub fn send_rtl(link: &Link, shared: &Shared) -> bool {
    simple_command(
        link, shared,
        MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.; 7],
        "RTL (RETURN_TO_LAUNCH) SENT", "Failed to send RTL",
    )
}

fn command_int(link: &Link, frame: mav::MavFrame, command: MavCmd, params: [f32; 4], x: i32, y: i32, z: f32) -> bool {
    let (target_system, target_component) = link.target();
    link.send(&MavMessage::COMMAND_INT(mav::COMMAND_INT_DATA {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        x,
        y,
        z,
        command,
        target_system,
        target_component,
        frame,
        current: 0,
        autocontinue: 0,
    }))
}

/// Auto-takeoff to `altitude_m` above the current altitude. The vehicle must
/// already be armed (we never auto-arm) and have a global position.
pub fn send_takeoff(link: &Link, shared: &Shared, altitude_m: f64) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    if !st.armed {
        st.error("Takeoff refused: vehicle is not armed (press [a] first)");
        return false;
    }
    if !st.global_pos_valid {
        st.error("Takeoff refused: no global position (need a GPS fix)");
        return false;
    }
    // PX4 reads NAV_TAKEOFF's param7 as AMSL regardless of the frame given,
    // so a relative altitude here would be "N m above sea level" and PX4
    // refuses with "Already higher than takeoff altitude".
    let target_alt = st.global_alt + altitude_m.max(0.0);
    let sent = command_int(
        link,
        mav::MavFrame::MAV_FRAME_GLOBAL,
        MavCmd::MAV_CMD_NAV_TAKEOFF,
        [0., 0., 0., f32::NAN],
        (st.global_lat * 1e7).round() as i32,
        (st.global_lon * 1e7).round() as i32,
        target_alt as f32,
    );
    if !sent {
        st.error("Failed to send takeoff");
        return false;
    }
    st.command(format!("TAKEOFF (NAV_TAKEOFF) SENT: climb to {altitude_m:.1} m"));
    true
}

/// Land at the current position.
pub fn send_land(link: &Link, shared: &Shared) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    let sent = if st.global_pos_valid {
        command_int(
            link,
            mav::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MavCmd::MAV_CMD_NAV_LAND,
            [0., 0., 0., f32::NAN],
            (st.global_lat * 1e7).round() as i32,
            (st.global_lon * 1e7).round() as i32,
            0.0,
        )
    } else {
        link.command_long(MavCmd::MAV_CMD_NAV_LAND, [0.; 7])
    };
    if !sent {
        st.error("Failed to send land");
        return false;
    }
    st.command("LAND (NAV_LAND) SENT");
    true
}
