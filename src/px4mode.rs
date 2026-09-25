//! PX4 legacy flight-mode encoding - what pymavlink's `mode_mapping_px4`,
//! `interpret_px4_mode` and `set_mode_px4` do for the Python version.
//!
//! PX4 packs its mode into HEARTBEAT.custom_mode as
//! `main_mode << 16 | sub_mode << 24`.

const MANUAL: u32 = 1;
const ALTCTL: u32 = 2;
const POSCTL: u32 = 3;
const AUTO: u32 = 4;
const ACRO: u32 = 5;
const OFFBOARD: u32 = 6;
const STABILIZED: u32 = 7;
const RATTITUDE: u32 = 8;

const AUTO_TAKEOFF: u32 = 2;
const AUTO_LOITER: u32 = 3;
const AUTO_MISSION: u32 = 4;
const AUTO_RTL: u32 = 5;
const AUTO_LAND: u32 = 6;
const AUTO_RTGS: u32 = 7;
const AUTO_FOLLOW_TARGET: u32 = 8;

// MAV_MODE_FLAG bits.
const CUSTOM_MODE_ENABLED: u8 = 1;
const GUIDED_ENABLED: u8 = 8;
const STABILIZE_ENABLED: u8 = 16;
const AUTO_ENABLED: u8 = 4;
const MANUAL_INPUT_ENABLED: u8 = 64;
pub const SAFETY_ARMED: u8 = 128;

const MANUAL_FLAGS: u8 = CUSTOM_MODE_ENABLED | MANUAL_INPUT_ENABLED | STABILIZE_ENABLED;
const AUTO_FLAGS: u8 = CUSTOM_MODE_ENABLED | AUTO_ENABLED | GUIDED_ENABLED | STABILIZE_ENABLED;

/// (name, base_mode flags, main mode, sub mode), as pymavlink's `px4_map`.
const PX4_MODES: &[(&str, u8, u32, u32)] = &[
    ("MANUAL", MANUAL_FLAGS, MANUAL, 0),
    ("STABILIZED", MANUAL_FLAGS, STABILIZED, 0),
    ("ACRO", MANUAL_FLAGS, ACRO, 0),
    ("RATTITUDE", MANUAL_FLAGS, RATTITUDE, 0),
    ("ALTCTL", MANUAL_FLAGS, ALTCTL, 0),
    ("POSCTL", MANUAL_FLAGS, POSCTL, 0),
    ("LOITER", AUTO_FLAGS, AUTO, AUTO_LOITER),
    ("MISSION", AUTO_FLAGS, AUTO, AUTO_MISSION),
    ("RTL", AUTO_FLAGS, AUTO, AUTO_RTL),
    ("LAND", AUTO_FLAGS, AUTO, AUTO_LAND),
    ("RTGS", AUTO_FLAGS, AUTO, AUTO_RTGS),
    ("FOLLOWME", AUTO_FLAGS, AUTO, AUTO_FOLLOW_TARGET),
    ("OFFBOARD", AUTO_FLAGS, OFFBOARD, 0),
    ("TAKEOFF", AUTO_FLAGS, AUTO, AUTO_TAKEOFF),
];

/// Sorted legacy mode names (pymavlink's `sorted(mode_mapping().keys())`).
pub fn legacy_mode_names() -> Vec<String> {
    let mut names: Vec<String> = PX4_MODES.iter().map(|m| m.0.to_string()).collect();
    names.sort();
    names
}

/// DO_SET_MODE params (base_mode, main_mode, sub_mode) for a legacy name.
pub fn legacy_mode_params(name: &str) -> Option<(f32, f32, f32)> {
    PX4_MODES
        .iter()
        .find(|m| m.0 == name)
        .map(|&(_, flags, main, sub)| (flags as f32, main as f32, sub as f32))
}

/// Mode name from a HEARTBEAT - pymavlink's `mode_string_v10`.
pub fn mode_string(autopilot: u32, base_mode: u8, custom_mode: u32) -> String {
    const MAV_AUTOPILOT_PX4: u32 = 12;
    if autopilot == MAV_AUTOPILOT_PX4 {
        return interpret_px4_mode(base_mode, custom_mode).to_string();
    }
    if base_mode & CUSTOM_MODE_ENABLED == 0 {
        return format!("Mode(0x{base_mode:08x})");
    }
    format!("Mode({custom_mode})")
}

fn interpret_px4_mode(base_mode: u8, custom_mode: u32) -> &'static str {
    let main = (custom_mode & 0x00FF_0000) >> 16;
    let sub = (custom_mode & 0xFF00_0000) >> 24;

    if base_mode & MANUAL_INPUT_ENABLED != 0 {
        return match main {
            MANUAL => "MANUAL",
            ACRO => "ACRO",
            RATTITUDE => "RATTITUDE",
            STABILIZED => "STABILIZED",
            ALTCTL => "ALTCTL",
            POSCTL => "POSCTL",
            _ => "UNKNOWN",
        };
    }

    let auto_bits = AUTO_ENABLED | STABILIZE_ENABLED | GUIDED_ENABLED;
    if base_mode & auto_bits == auto_bits {
        if main == AUTO {
            return match sub {
                AUTO_MISSION => "MISSION",
                AUTO_TAKEOFF => "TAKEOFF",
                AUTO_LOITER => "LOITER",
                AUTO_FOLLOW_TARGET => "FOLLOWME",
                AUTO_RTL => "RTL",
                AUTO_LAND => "LAND",
                AUTO_RTGS => "RTGS",
                _ => "UNKNOWN",
            };
        }
        if main == OFFBOARD {
            return "OFFBOARD";
        }
    }
    "UNKNOWN"
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_every_legacy_mode() {
        for &(name, flags, main, sub) in PX4_MODES {
            let custom = (main << 16) | (sub << 24);
            assert_eq!(mode_string(12, flags, custom), name);
        }
    }
}
