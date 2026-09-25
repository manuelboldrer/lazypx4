//! Standard Modes Protocol (AVAILABLE_MODES / CURRENT_MODE).
//!
//! This is how PX4 v1.15+ exposes modes a static table can't know about:
//! custom internal modes and PX4 ROS 2 "external" modes registered at
//! runtime by a companion computer. Mirrors `mavlink/modes.py`.

use mavlink::dialects::development::{self as mav, MavCmd};

use crate::config::*;
use crate::link::{Link, from_char_array};
use crate::mav::commands::{set_mode, vehicle_ready};
use crate::px4mode;
use crate::state::{self, CustomMode, Shared, State, now};

const MAV_MODE_PROPERTY_NOT_USER_SELECTABLE: u32 = 2;
const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;

#[derive(Debug, Clone)]
pub enum ModeKind {
    Legacy(String),
    Standard(u32),
    Custom(u32),
}

#[derive(Debug, Clone)]
pub struct ModeOption {
    pub label: String,
    pub kind: ModeKind,
    /// HEARTBEAT custom_mode this option corresponds to, when known - used
    /// to put the cursor on the mode currently flying.
    pub custom_mode: Option<u32>,
}

pub fn standard_mode_name(value: u32) -> String {
    match value {
        0 => "NON_STANDARD".into(),
        1 => "POSITION_HOLD".into(),
        2 => "ORBIT".into(),
        3 => "CRUISE".into(),
        4 => "ALTITUDE_HOLD".into(),
        5 => "SAFE_RECOVERY".into(),
        6 => "MISSION".into(),
        7 => "LAND".into(),
        8 => "TAKEOFF".into(),
        v => format!("STANDARD_{v}"),
    }
}

pub fn request_available_modes(link: &Link, shared: &Shared) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    st.custom_modes.clear();
    st.custom_modes_total = 0;
    st.custom_modes_complete = false;
    st.custom_modes_requested_at = now();

    // param2 = 0 -> emit AVAILABLE_MODES for all modes.
    if !link.command_long(
        MavCmd::MAV_CMD_REQUEST_MESSAGE,
        [MAVLINK_MSG_ID_AVAILABLE_MODES, 0., 0., 0., 0., 0., 0.],
    ) {
        st.custom_modes_requested_at = 0.0;
        st.error("Failed to request AVAILABLE_MODES");
        return false;
    }
    st.command("AVAILABLE_MODES request SENT");
    true
}

pub fn handle_available_modes(st: &mut State, d: &mav::AVAILABLE_MODES_DATA) {
    // PX4 leaves the name empty for the standard modes; show those by their
    // standard-mode name instead of an opaque MODE_<custom_mode>.
    let mut name = from_char_array(&d.mode_name[..]);
    if name.is_empty() {
        let standard = d.standard_mode as u32;
        name = if standard != 0 { standard_mode_name(standard) } else { format!("MODE_{}", d.custom_mode) };
    }
    st.custom_modes.insert(
        d.mode_index,
        CustomMode {
            custom_mode: d.custom_mode,
            standard_mode: d.standard_mode as u32,
            properties: d.properties.bits(),
            name,
        },
    );
    let number = d.number_modes as u32;
    if number > 0 {
        st.custom_modes_total = number;
        if st.custom_modes.len() as u32 >= number {
            st.custom_modes_complete = true;
            st.custom_modes_requested_at = 0.0;
        }
    }
}

/// Time out an AVAILABLE_MODES enumeration PX4 never finished.
pub fn check_available_modes_request(st: &mut State) {
    let requested_at = st.custom_modes_requested_at;
    if requested_at == 0.0 || st.custom_modes_complete {
        return;
    }
    let received = st.custom_modes.len() as u32;
    let total = st.custom_modes_total;
    if total > 0 && received >= total {
        st.custom_modes_complete = true;
        st.custom_modes_requested_at = 0.0;
        return;
    }
    if now() - requested_at > AVAILABLE_MODES_TIMEOUT {
        st.custom_modes_requested_at = 0.0;
        let total = if total > 0 { format!("/{total}") } else { String::new() };
        st.warn(format!("AVAILABLE_MODES request timed out. Received {received}{total}"));
    }
}

/// The selectable modes: the Standard Modes Protocol whenever it produced
/// anything (it knows custom / external modes), else the legacy list.
pub fn mode_options(st: &State) -> Vec<ModeOption> {
    if st.custom_modes.is_empty() {
        return px4mode::legacy_mode_names()
            .into_iter()
            .map(|m| ModeOption { label: m.clone(), kind: ModeKind::Legacy(m), custom_mode: None })
            .collect();
    }
    st.custom_modes
        .values()
        .filter(|e| e.properties & MAV_MODE_PROPERTY_NOT_USER_SELECTABLE == 0)
        .map(|e| {
            if e.standard_mode != 0 {
                let standard = standard_mode_name(e.standard_mode);
                let label = if e.name == standard {
                    format!("{standard} [STANDARD]")
                } else {
                    format!("{} [{standard}]", e.name)
                };
                ModeOption { label, kind: ModeKind::Standard(e.standard_mode), custom_mode: Some(e.custom_mode) }
            } else {
                ModeOption {
                    label: format!("{} [CUSTOM]", e.name),
                    kind: ModeKind::Custom(e.custom_mode),
                    custom_mode: Some(e.custom_mode),
                }
            }
        })
        .collect()
}

pub fn confirm_mode_option(link: &Link, shared: &Shared, option: &ModeOption) {
    let mut st = state::lock(shared);
    match &option.kind {
        ModeKind::Legacy(name) => {
            set_mode(link, &mut st, name);
        }
        ModeKind::Standard(value) => {
            if !vehicle_ready(&mut st, link) {
                return;
            }
            if link.command_long(MavCmd::MAV_CMD_DO_SET_STANDARD_MODE, [*value as f32, 0., 0., 0., 0., 0., 0.]) {
                st.command(format!("DO_SET_STANDARD_MODE command SENT: {}", option.label));
            } else {
                st.error(format!("Failed to set standard mode {}", option.label));
            }
        }
        ModeKind::Custom(value) => {
            if !vehicle_ready(&mut st, link) {
                return;
            }
            if link.command_long(
                MavCmd::MAV_CMD_DO_SET_MODE,
                [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, *value as f32, 0., 0., 0., 0., 0.],
            ) {
                st.command(format!("DO_SET_MODE command SENT: {}", option.label));
            } else {
                st.error(format!("Failed to set custom mode {}", option.label));
            }
        }
    }
}
