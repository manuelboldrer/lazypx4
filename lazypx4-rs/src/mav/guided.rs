//! Goto (MAV_CMD_DO_REPOSITION) in three frames plus the keyboard jog, the
//! KML waypoint queue and the geofence upload. Mirrors `mavlink/guided.py`
//! (the goto half), `mavlink/wp_queue.py` and `mavlink/fence.py`.
//!
//! Every goto variant ends in the same DO_REPOSITION command; only how the
//! absolute lat / lon / AMSL target is computed differs. They all need the
//! vehicle armed - PX4 does the mode switch itself.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use mavlink::dialects::development::{self as mav, MavCmd, MavMessage};

use crate::config::*;
use crate::geo;
use crate::link::Link;
use crate::mav::commands::vehicle_ready;
use crate::state::{self, Shared, State, Target, now};

/// MAV_DO_REPOSITION_FLAGS_CHANGE_MODE.
const REPOSITION_CHANGE_MODE: f32 = 1.0;

/// The one DO_REPOSITION send shared by every goto variant and the jog.
/// `yaw_deg` is an absolute compass heading, None = keep current. (This
/// command's yaw param is radians, unlike most MAVLink yaw params.)
fn send_reposition(link: &Link, lat: f64, lon: f64, amsl: f64, yaw_deg: Option<f64>) -> bool {
    let (target_system, target_component) = link.target();
    link.send(&MavMessage::COMMAND_INT(mav::COMMAND_INT_DATA {
        param1: -1.0, // ground speed: default
        param2: REPOSITION_CHANGE_MODE,
        param3: 0.0,
        param4: yaw_deg.map(|y| y.to_radians() as f32).unwrap_or(f32::NAN),
        x: (lat * 1e7).round() as i32,
        y: (lon * 1e7).round() as i32,
        z: amsl as f32,
        command: MavCmd::MAV_CMD_DO_REPOSITION,
        target_system,
        target_component,
        // x/y = 1e7 deg, z = AMSL (MAV_FRAME_GLOBAL_INT's modern name).
        frame: mav::MavFrame::MAV_FRAME_GLOBAL,
        current: 0,
        autocontinue: 0,
    }))
}

fn armed_or_refuse(st: &mut State, link: &Link) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    if !st.armed {
        st.error("Goto refused: vehicle is not armed");
        return false;
    }
    true
}

/// Body-relative: `forward` / `right` / `down` (positive = descend) metres
/// from the current position and heading. `quiet` skips the log line (the
/// jog logs its own).
pub fn goto_body(link: &Link, st: &mut State, forward: f64, right: f64, down: f64, yaw: Option<f64>, quiet: bool) -> bool {
    if !armed_or_refuse(st, link) {
        return false;
    }
    if !st.global_pos_valid {
        st.error("Goto refused: no global position (need a GPS fix)");
        return false;
    }
    let (c, s) = (st.yaw.to_radians().cos(), st.yaw.to_radians().sin());
    let (north, east) = (forward * c - right * s, forward * s + right * c);
    let (lat, lon) = geo::offset(st.global_lat, st.global_lon, north, east);
    let amsl = st.global_alt - down;
    if !send_reposition(link, lat, lon, amsl, yaw) {
        st.error("Failed to send goto");
        return false;
    }
    if !quiet {
        st.command(format!(
            "GOTO (DO_REPOSITION) SENT [relative]: fwd {forward:+.1} right {right:+.1} down {down:+.1} m -> {lat:.7}, {lon:.7} @ {amsl:.1} m MSL"
        ));
    }
    true
}

/// Absolute point in the local NED frame (the map grid's frame).
pub fn goto_local(link: &Link, st: &mut State, north: f64, east: f64, down: f64, yaw: Option<f64>) -> bool {
    if !armed_or_refuse(st, link) {
        return false;
    }
    if !st.local_origin_set {
        st.error("Goto (local) refused: no local origin yet (GPS_GLOBAL_ORIGIN)");
        return false;
    }
    let (lat, lon) = geo::offset(st.local_origin_lat, st.local_origin_lon, north, east);
    let amsl = st.local_origin_alt - down;
    if !send_reposition(link, lat, lon, amsl, yaw) {
        st.error("Failed to send goto");
        return false;
    }
    st.command(format!(
        "GOTO (DO_REPOSITION) SENT [local NED]: N {north:+.1} E {east:+.1} D {down:+.1} m -> {lat:.7}, {lon:.7} @ {amsl:.1} m MSL"
    ));
    true
}

pub fn goto_global(link: &Link, st: &mut State, lat: f64, lon: f64, amsl: f64, yaw: Option<f64>) -> bool {
    if !armed_or_refuse(st, link) {
        return false;
    }
    if !send_reposition(link, lat, lon, amsl, yaw) {
        st.error("Failed to send goto");
        return false;
    }
    st.command(format!("GOTO (DO_REPOSITION) SENT [global]: {lat:.7}, {lon:.7} @ {amsl:.1} m MSL"));
    true
}

// ---------------------------------------------------------------------------
// Waypoint queue ([W] / [C])
// ---------------------------------------------------------------------------

/// Tiny xorshift for "rand N" - no need for a crypto-grade RNG to pick
/// which waypoint to fly to next.
fn random_index(n: usize) -> usize {
    use std::sync::atomic::{AtomicU64, Ordering};
    static SEED: AtomicU64 = AtomicU64::new(0);
    let mut x = SEED.load(Ordering::Relaxed);
    if x == 0 {
        x = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(1) | 1;
    }
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    SEED.store(x, Ordering::Relaxed);
    (x % n as u64) as usize
}

/// KML waypoints -> targets for "single" (1-based index) / "sequence" /
/// "random" (count legs, sampled with replacement).
pub fn build_targets(waypoints: &[geo::Waypoint], mode: &str, index: Option<usize>, count: Option<usize>) -> Result<Vec<Target>, String> {
    let target = |i: usize| {
        let w = &waypoints[i];
        Target {
            lat: w.lat,
            lon: w.lon,
            name: if w.name.is_empty() { format!("waypoint {}", i + 1) } else { w.name.clone() },
        }
    };
    match mode {
        "single" => match index {
            Some(i) if (1..=waypoints.len()).contains(&i) => Ok(vec![target(i - 1)]),
            _ => Err(format!("waypoint number must be 1-{}", waypoints.len())),
        },
        "sequence" => Ok((0..waypoints.len()).map(target).collect()),
        "random" => {
            let n = count.filter(|c| *c >= 1).ok_or("waypoint count must be at least 1")?;
            if waypoints.is_empty() {
                return Err("no waypoints loaded".into());
            }
            Ok((0..n).map(|_| target(random_index(waypoints.len()))).collect())
        }
        other => Err(format!("unknown waypoint queue mode: {other}")),
    }
}

/// Activate the queue: lock in the current altitude and send the first leg
/// (or, facing-target mode, turn toward it first).
pub fn start_wp_queue(link: &Link, shared: &Shared, targets: Vec<Target>, mode: &str, face_target: bool) -> bool {
    let mut st = state::lock(shared);
    if !st.armed {
        st.error("Waypoint queue refused: vehicle is not armed");
        return false;
    }
    if !st.global_pos_valid {
        st.error("Waypoint queue refused: no global position (need a GPS fix)");
        return false;
    }
    let Some(first) = targets.first().cloned() else { return false };
    let (alt, lat, lon) = (st.global_alt, st.global_lat, st.global_lon);

    let ok = if face_target {
        let bearing = geo::bearing_deg(lat, lon, first.lat, first.lon);
        goto_global(link, &mut st, lat, lon, alt, Some(bearing))
    } else {
        goto_global(link, &mut st, first.lat, first.lon, alt, None)
    };
    if !ok {
        return false;
    }
    st.wp_path = targets.clone();
    st.wp_queue = targets.into();
    st.wp_queue_mode = mode.to_string();
    st.wp_queue_total = st.wp_path.len();
    st.wp_queue_alt_amsl = alt;
    st.wp_queue_face_target = face_target;
    st.wp_queue_rotating = face_target;
    st.wp_queue_active = true;
    st.wp_queue_status = if face_target { format!("rotating to face '{}'", first.name) } else { "en route".into() };
    st.wp_queue_phase_started_at = now();
    let n = st.wp_path.len();
    st.info(format!(
        "WAYPOINT QUEUE started ({mode}{}): {n} target(s), first '{}' @ {alt:.1} m MSL, arrival radius {WP_QUEUE_ARRIVAL_M:.1} m",
        if face_target { ", facing target" } else { "" },
        first.name
    ));
    true
}

pub fn cancel_wp_queue(st: &mut State, reason: &str) {
    if !st.wp_queue_active {
        return;
    }
    st.wp_queue_active = false;
    st.wp_queue_status = reason.to_string();
    st.wp_queue.clear();
    st.info(format!("WAYPOINT QUEUE cancelled: {reason}"));
}

/// Current leg reached (or abandoned): next target, or done.
fn advance(link: &Link, st: &mut State) {
    st.wp_queue.pop_front();
    let Some(next) = st.wp_queue.front().cloned() else {
        st.wp_queue_active = false;
        st.wp_queue_status = "complete".into();
        st.info("WAYPOINT QUEUE complete");
        return;
    };
    let (alt, lat, lon) = (st.wp_queue_alt_amsl, st.global_lat, st.global_lon);
    let ok = if st.wp_queue_face_target {
        let bearing = geo::bearing_deg(lat, lon, next.lat, next.lon);
        goto_global(link, st, lat, lon, alt, Some(bearing))
    } else {
        goto_global(link, st, next.lat, next.lon, alt, None)
    };
    if !ok {
        cancel_wp_queue(st, "goto send failed");
        return;
    }
    st.wp_queue_rotating = st.wp_queue_face_target;
    st.wp_queue_status = if st.wp_queue_rotating { format!("rotating to face '{}'", next.name) } else { "en route".into() };
    st.wp_queue_phase_started_at = now();
}

/// Background loop: arrival / heading / timeout checks for the active queue.
pub fn wp_queue_thread(link: Arc<Link>, shared: Shared, shutdown: Arc<AtomicBool>) {
    while crate::host::sleep_unless_shutdown(&shutdown, Duration::from_secs_f64(WP_QUEUE_POLL_S)) {
        let mut st = state::lock(&shared);
        if !st.wp_queue_active {
            continue;
        }
        if !st.armed {
            cancel_wp_queue(&mut st, "vehicle disarmed");
            continue;
        }
        let Some(target) = st.wp_queue.front().cloned() else {
            cancel_wp_queue(&mut st, "empty queue");
            continue;
        };
        if !st.global_pos_valid {
            continue;
        }
        let (lat, lon) = (st.global_lat, st.global_lon);
        let elapsed = now() - st.wp_queue_phase_started_at;

        if st.wp_queue_rotating {
            let desired = geo::bearing_deg(lat, lon, target.lat, target.lon);
            let error = (desired - st.yaw + 180.0).rem_euclid(360.0) - 180.0;
            let timed_out = elapsed > WP_QUEUE_ROTATE_TIMEOUT_S;
            if timed_out {
                st.error(format!("WAYPOINT QUEUE timed out turning to face '{}' - departing anyway", target.name));
            }
            if error.abs() <= WP_QUEUE_YAW_TOLERANCE_DEG || timed_out {
                // Depart nose-locked on the bearing it now faces.
                let alt = st.wp_queue_alt_amsl;
                if goto_global(&link, &mut st, target.lat, target.lon, alt, Some(desired)) {
                    st.wp_queue_rotating = false;
                    st.wp_queue_status = "en route".into();
                    st.wp_queue_phase_started_at = now();
                } else {
                    cancel_wp_queue(&mut st, "goto send failed");
                }
            }
            continue;
        }

        let dist = geo::distance_m(lat, lon, target.lat, target.lon);
        if dist <= WP_QUEUE_ARRIVAL_M {
            st.info(format!("WAYPOINT QUEUE arrived at '{}' ({dist:.1} m)", target.name));
            advance(&link, &mut st);
        } else if elapsed > WP_QUEUE_LEG_TIMEOUT_S {
            st.error(format!("WAYPOINT QUEUE timed out reaching '{}' - skipping", target.name));
            advance(&link, &mut st);
        }
    }
}

// ---------------------------------------------------------------------------
// Geofence upload ([O]) - mission protocol, mission_type = FENCE
// ---------------------------------------------------------------------------

/// Rings -> one inclusion-vertex item per unique vertex, each carrying its
/// polygon's vertex count (param1), as PX4 expects.
pub fn fence_items(rings: &[Vec<(f64, f64)>]) -> Vec<(f64, f64, u16)> {
    rings
        .iter()
        .map(|r| geo::open_ring(r))
        .filter(|r| r.len() >= 3)
        .flat_map(|r| r.iter().map(move |&(lat, lon)| (lat, lon, r.len() as u16)))
        .collect()
}

pub fn start_fence_upload(link: &Link, st: &mut State) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    if st.fence_upload_active {
        st.warn("A geofence upload is already in progress");
        return false;
    }
    let items = fence_items(st.kml.as_ref().map(|k| &k.fence_rings[..]).unwrap_or(&[]));
    if items.is_empty() {
        st.error("No fence polygon loaded - load one with [o] first");
        return false;
    }
    let (target_system, target_component) = link.target();
    let count = items.len();
    st.fence_upload_items = items;
    st.fence_upload_active = true;
    st.fence_upload_status = "UPLOADING".into();
    st.fence_upload_error.clear();
    st.fence_upload_acked_seq = -1;
    st.fence_upload_started_at = now();
    if !link.send(&MavMessage::MISSION_COUNT(mav::MISSION_COUNT_DATA {
        count: count as u16,
        target_system,
        target_component,
        mission_type: mav::MavMissionType::MAV_MISSION_TYPE_FENCE,
        opaque_id: 0,
    })) {
        st.fence_upload_active = false;
        st.fence_upload_status = "ERROR".into();
        st.fence_upload_error = "send failed".into();
        st.error("Failed to start geofence upload");
        return false;
    }
    st.command(format!("GEOFENCE upload started: {count} vertice(s)"));
    true
}

/// MISSION_REQUEST / MISSION_REQUEST_INT for the fence: send that item.
pub fn handle_mission_request(st: &mut State, link: &Link, seq: u16, mission_type: mav::MavMissionType) {
    if mission_type != mav::MavMissionType::MAV_MISSION_TYPE_FENCE || !st.fence_upload_active {
        return;
    }
    let Some(&(lat, lon, vertices)) = st.fence_upload_items.get(seq as usize) else { return };
    let (target_system, target_component) = link.target();
    let sent = link.send(&MavMessage::MISSION_ITEM_INT(mav::MISSION_ITEM_INT_DATA {
        param1: vertices as f32,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: (lat * 1e7).round() as i32,
        y: (lon * 1e7).round() as i32,
        z: 0.0,
        seq,
        command: MavCmd::MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
        target_system,
        target_component,
        frame: mav::MavFrame::MAV_FRAME_GLOBAL,
        current: 0,
        autocontinue: 0,
        mission_type: mav::MavMissionType::MAV_MISSION_TYPE_FENCE,
    }));
    if sent {
        st.fence_upload_acked_seq = seq as i32;
    } else {
        st.error(format!("Failed to send fence item {seq}"));
    }
}

pub fn handle_mission_ack(st: &mut State, result: mav::MavMissionResult, mission_type: mav::MavMissionType) {
    if mission_type != mav::MavMissionType::MAV_MISSION_TYPE_FENCE || !st.fence_upload_active {
        return;
    }
    st.fence_upload_active = false;
    let total = st.fence_upload_items.len();
    if result == mav::MavMissionResult::MAV_MISSION_ACCEPTED {
        st.fence_upload_status = "COMPLETE".into();
        st.fence_upload_error.clear();
        st.info(format!("GEOFENCE uploaded and ACCEPTED by PX4 ({total} vertice(s))"));
    } else {
        let name = format!("{result:?}").trim_start_matches("MAV_MISSION_").to_string();
        st.fence_upload_status = "ERROR".into();
        st.fence_upload_error = name.clone();
        st.error(format!("GEOFENCE upload rejected by PX4: {name}"));
    }
}

pub fn check_fence_upload(st: &mut State) {
    if st.fence_upload_active && now() - st.fence_upload_started_at > FENCE_UPLOAD_TIMEOUT {
        st.fence_upload_active = false;
        st.fence_upload_status = "ERROR".into();
        st.fence_upload_error = "timed out waiting for the vehicle".into();
        st.error("GEOFENCE upload timed out");
    }
}
