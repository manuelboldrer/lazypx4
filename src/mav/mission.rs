//! Mission upload ([M] / `:mission upload`) of a loaded QGC .plan - the
//! MAVLink mission protocol with mission_type = MISSION. The fence upload
//! in guided.rs is the same protocol with mission_type = FENCE.

use mavlink::dialects::development::{self as mav, MavCmd, MavMessage};

use crate::config::*;
use crate::geo::PlanItem;
use crate::link::Link;
use crate::mav::commands::vehicle_ready;
use crate::state::{State, now};

pub fn start_mission_upload(link: &Link, st: &mut State) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    if st.mission_upload_active {
        st.warn("A mission upload is already in progress");
        return false;
    }
    let items = st.plan.as_ref().map(|p| p.items.clone()).unwrap_or_default();
    if items.is_empty() {
        st.error("No mission loaded - load a QGC .plan with [o] first");
        return false;
    }
    let (target_system, target_component) = link.target();
    let count = items.len();
    st.mission_upload_items = items;
    st.mission_upload_active = true;
    st.mission_upload_status = "UPLOADING".into();
    st.mission_upload_error.clear();
    st.mission_upload_acked_seq = -1;
    st.mission_upload_started_at = now();
    if !link.send(&MavMessage::MISSION_COUNT(mav::MISSION_COUNT_DATA {
        count: count as u16,
        target_system,
        target_component,
        mission_type: mav::MavMissionType::MAV_MISSION_TYPE_MISSION,
        opaque_id: 0,
    })) {
        st.mission_upload_active = false;
        st.mission_upload_status = "ERROR".into();
        st.mission_upload_error = "send failed".into();
        st.error("Failed to start the mission upload");
        return false;
    }
    st.command(format!("MISSION upload started: {count} item(s)"));
    true
}

fn item_message(item: &PlanItem, seq: u16, target: (u8, u8)) -> Option<MavMessage> {
    let p = |i: usize| item.params[i];
    // Unknown commands / frames can't be encoded by this dialect.
    let command: MavCmd = num_traits_from(item.command as u32)?;
    let frame: mav::MavFrame = num_traits_from(item.frame as u32)?;
    // Positions travel as degE7 integers; a positionless item keeps its
    // raw params (x / y are then plain numbers, not coordinates).
    let positional = item.frame != 2;
    let int = |v: f64, scale: f64| if v.is_finite() { (v * scale).round() as i32 } else { 0 };
    Some(MavMessage::MISSION_ITEM_INT(mav::MISSION_ITEM_INT_DATA {
        param1: p(0) as f32,
        param2: p(1) as f32,
        param3: p(2) as f32,
        param4: p(3) as f32,
        x: int(p(4), if positional { 1e7 } else { 1.0 }),
        y: int(p(5), if positional { 1e7 } else { 1.0 }),
        z: if p(6).is_finite() { p(6) as f32 } else { 0.0 },
        seq,
        command,
        target_system: target.0,
        target_component: target.1,
        frame,
        current: u8::from(seq == 0),
        autocontinue: u8::from(item.autocontinue),
        mission_type: mav::MavMissionType::MAV_MISSION_TYPE_MISSION,
    }))
}

fn num_traits_from<T: num_traits::FromPrimitive>(v: u32) -> Option<T> {
    T::from_u32(v)
}

pub fn handle_mission_request(st: &mut State, link: &Link, seq: u16) {
    if !st.mission_upload_active {
        return;
    }
    let Some(item) = st.mission_upload_items.get(seq as usize) else { return };
    let Some(msg) = item_message(item, seq, link.target()) else {
        let command = item.command;
        st.mission_upload_active = false;
        st.mission_upload_status = "ERROR".into();
        st.mission_upload_error = format!("item {seq}: command {command} not in this MAVLink dialect");
        st.error(format!("MISSION upload aborted: item {seq} has unknown command {command}"));
        return;
    };
    if link.send(&msg) {
        st.mission_upload_acked_seq = seq as i32;
    } else {
        st.error(format!("Failed to send mission item {seq}"));
    }
}

pub fn handle_mission_ack(st: &mut State, result: mav::MavMissionResult) {
    if !st.mission_upload_active {
        return;
    }
    st.mission_upload_active = false;
    let total = st.mission_upload_items.len();
    if result == mav::MavMissionResult::MAV_MISSION_ACCEPTED {
        st.mission_upload_status = "COMPLETE".into();
        st.mission_upload_error.clear();
        st.info(format!("MISSION uploaded and ACCEPTED by PX4 ({total} item(s)) - :mode mission to fly it"));
    } else {
        let name = format!("{result:?}").trim_start_matches("MAV_MISSION_").to_string();
        st.mission_upload_status = "ERROR".into();
        st.mission_upload_error = name.clone();
        st.error(format!("MISSION upload rejected by PX4: {name}"));
    }
}

pub fn check_mission_upload(st: &mut State) {
    if st.mission_upload_active && now() - st.mission_upload_started_at > FENCE_UPLOAD_TIMEOUT {
        st.mission_upload_active = false;
        st.mission_upload_status = "ERROR".into();
        st.mission_upload_error = "timed out waiting for the vehicle".into();
        st.error("MISSION upload timed out");
    }
}
