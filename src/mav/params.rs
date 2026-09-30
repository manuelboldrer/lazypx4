//! PX4 parameter protocol: list, read, set.
//!
//! The awkward part is the float<->int reinterpretation. PARAM_VALUE /
//! PARAM_SET only carry a 4-byte float; for integer parameters PX4 does NOT
//! convert the integer to a numeric float - it reinterprets the raw bytes of
//! an int32/uint32 as a float ("union cast"). We must reverse exactly that,
//! or NaN/Inf bit patterns (common for big uint32 device ids) break.

use mavlink::dialects::development::{self as mav, MavMessage};

use crate::config::*;
use crate::link::{Link, char_array, from_char_array};
use crate::mav::commands::vehicle_ready;
use crate::state::{Parameter, State, now};

pub fn is_integer(t: u8) -> bool {
    (1..=8).contains(&t)
}

fn is_unsigned(t: u8) -> bool {
    matches!(t, 1 | 3 | 5 | 7)
}

pub fn decode(raw: f32, t: u8) -> f64 {
    if !is_integer(t) {
        return raw as f64;
    }
    let bits = raw.to_bits();
    if is_unsigned(t) { bits as f64 } else { bits as i32 as f64 }
}

pub fn encode(value: f64, t: u8) -> f32 {
    if !is_integer(t) {
        return value as f32;
    }
    let int_value = value.round() as i64;
    if is_unsigned(t) {
        f32::from_bits((int_value & 0xFFFF_FFFF) as u32)
    } else {
        f32::from_bits(int_value as i32 as u32)
    }
}

pub fn format_value(p: &Parameter) -> String {
    if is_integer(p.param_type) {
        if !p.value.is_finite() {
            return "N/A".into();
        }
        return format!("{}", p.value.round() as i64);
    }
    fmt_g(p.value, if p.param_type == 10 { 9 } else { 6 })
}

/// Python's `%.Ng`: N significant digits, trailing zeros trimmed.
pub fn fmt_g(value: f64, digits: usize) -> String {
    if !value.is_finite() {
        return "N/A".into();
    }
    if value == 0.0 {
        return "0".into();
    }
    let exp = value.abs().log10().floor() as i32;
    if exp < -4 || exp >= digits as i32 {
        let s = format!("{:.*e}", digits.saturating_sub(1), value);
        // Trim the mantissa's trailing zeros: 1.500000e3 -> 1.5e3.
        if let Some((mant, e)) = s.split_once('e') {
            let mant = if mant.contains('.') {
                mant.trim_end_matches('0').trim_end_matches('.')
            } else {
                mant
            };
            let e: i32 = e.parse().unwrap_or(0);
            return format!("{mant}e{}{:02}", if e < 0 { '-' } else { '+' }, e.abs());
        }
        return s;
    }
    let decimals = (digits as i32 - 1 - exp).max(0) as usize;
    let s = format!("{:.*}", decimals, value);
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        s
    }
}

pub fn parse_value(text: &str, p: &Parameter) -> Result<f64, String> {
    let text = text.trim();
    if text.is_empty() {
        return Err("Empty parameter value".into());
    }
    let value: f64 = text.parse().map_err(|_| format!("could not convert '{text}' to a number"))?;
    if !value.is_finite() {
        return Err("Parameter value must be finite".into());
    }
    if is_integer(p.param_type) && value.fract() != 0.0 {
        return Err("This parameter requires an integer value".into());
    }
    Ok(value)
}

pub fn handle_param_value(st: &mut State, d: &mav::PARAM_VALUE_DATA) {
    let name = from_char_array(&d.param_id[..]);
    if name.is_empty() {
        return;
    }
    let param_type = d.param_type as u8;
    let value = decode(d.param_value, param_type);
    let index = d.param_index as i32;
    let count = d.param_count as u32;

    let default_value = st.parameter_defaults.get(&name).copied();
    let parameter = st.parameters.entry(name.clone()).or_insert_with(|| Parameter {
        name: name.clone(),
        value,
        param_type,
        index,
        startup_value: value,
        default_value,
        pending: false,
    });
    parameter.value = value;
    if param_type != 0 {
        parameter.param_type = param_type;
    }
    parameter.index = index;
    if parameter.default_value.is_none() {
        parameter.default_value = default_value;
    }
    let is_int = is_integer(parameter.param_type);
    let formatted = format_value(parameter);

    if let Err(pos) = st
        .parameter_order
        .binary_search_by(|n| n.to_uppercase().cmp(&name.to_uppercase()))
    {
        st.parameter_order.insert(pos, name.clone());
    }

    if count > 0 {
        st.parameter_count = st.parameter_count.max(count);
    }

    if let Some((pending_name, expected, _)) = &st.parameter_set_pending
        && *pending_name == name {
            let tolerance = if is_int { 0.0 } else { PARAM_VALUE_EPSILON };
            if (value - expected).abs() <= tolerance {
                st.parameter_set_pending = None;
                if let Some(p) = st.parameters.get_mut(&name) {
                    p.pending = false;
                }
                st.command(format!("PARAMETER CONFIRMED: {name} = {formatted}"));
            }
        }

    if count > 0 && st.parameters.len() as u32 >= count {
        st.parameter_count = count;
        st.parameters_complete = true;
        st.parameters_requested_at = 0.0;
    }
}

pub fn request_parameters(link: &Link, st: &mut State) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    st.parameters.clear();
    st.parameter_order.clear();
    st.parameter_count = 0;
    st.parameters_complete = false;
    st.parameters_requested_at = now();
    st.parameter_full_list_requested = true;
    st.parameter_set_pending = None;

    let (target_system, target_component) = link.target();
    if !link.send(&MavMessage::PARAM_REQUEST_LIST(mav::PARAM_REQUEST_LIST_DATA {
        target_system,
        target_component,
    })) {
        st.parameters_requested_at = 0.0;
        st.error("Failed to request parameters");
        return false;
    }
    st.command("PARAM_REQUEST_LIST SENT");
    true
}

/// The parameters in the active ALL / CHANGED view, in display order.
pub fn visible_parameters(st: &State, changed_only: bool) -> Vec<&Parameter> {
    st.parameter_order
        .iter()
        .filter_map(|n| st.parameters.get(n))
        .filter(|p| !changed_only || p.is_changed())
        .collect()
}

pub fn send_parameter_set(link: &Link, st: &mut State, name: &str, value: f64) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    let Some(parameter) = st.parameters.get_mut(name) else {
        st.error(format!("Unknown PX4 parameter: {name}"));
        return false;
    };
    let param_type = if parameter.param_type == 0 { 9 } else { parameter.param_type };
    parameter.pending = true;

    let (target_system, target_component) = link.target();
    let sent = link.send(&MavMessage::PARAM_SET(mav::PARAM_SET_DATA {
        param_value: encode(value, param_type),
        target_system,
        target_component,
        param_id: char_array(name),
        param_type: num_param_type(param_type),
    }));

    if !sent {
        if let Some(p) = st.parameters.get_mut(name) {
            p.pending = false;
        }
        st.error(format!("Failed to set parameter {name}"));
        return false;
    }
    st.parameter_set_pending = Some((name.to_string(), value, now()));
    st.command(format!("PARAM_SET SENT: {name} = {}", fmt_g(value, 12)));
    true
}

fn num_param_type(t: u8) -> mav::MavParamType {
    use mav::MavParamType as T;
    match t {
        1 => T::MAV_PARAM_TYPE_UINT8,
        2 => T::MAV_PARAM_TYPE_INT8,
        3 => T::MAV_PARAM_TYPE_UINT16,
        4 => T::MAV_PARAM_TYPE_INT16,
        5 => T::MAV_PARAM_TYPE_UINT32,
        6 => T::MAV_PARAM_TYPE_INT32,
        7 => T::MAV_PARAM_TYPE_UINT64,
        8 => T::MAV_PARAM_TYPE_INT64,
        10 => T::MAV_PARAM_TYPE_REAL64,
        _ => T::MAV_PARAM_TYPE_REAL32,
    }
}

/// Time out an incomplete PARAM_REQUEST_LIST response.
pub fn check_parameter_request(st: &mut State) {
    let requested_at = st.parameters_requested_at;
    if requested_at == 0.0 || st.parameters_complete {
        return;
    }
    let received = st.parameters.len() as u32;
    let count = st.parameter_count;
    if count > 0 && received >= count {
        st.parameters_complete = true;
        st.parameters_requested_at = 0.0;
        return;
    }
    if now() - requested_at > PARAM_REQUEST_TIMEOUT {
        st.parameters_requested_at = 0.0;
        let total = if count > 0 { format!("/{count}") } else { String::new() };
        st.warn(format!("Parameter request timed out. Received {received}{total}"));
    }
}

/// Time out a PARAM_SET PX4 never echoed back.
pub fn check_pending_parameter(st: &mut State) {
    let Some((name, _, sent_at)) = st.parameter_set_pending.clone() else { return };
    if now() - sent_at <= PARAM_SET_TIMEOUT {
        return;
    }
    st.parameter_set_pending = None;
    if let Some(p) = st.parameters.get_mut(&name) {
        p.pending = false;
    }
    if st.param_load_total > 0 {
        st.param_load_failed.push(name.clone());
    }
    st.error(format!("PARAM_SET not confirmed by PX4: {name}"));
}

// ---------------------------------------------------------------------
// Parameter files: save / load
// ---------------------------------------------------------------------

/// Write every received parameter in QGroundControl's `.params` format
/// (`sysid compid NAME value type`, tab separated), which QGC, MAVProxy and
/// MAVSDK all read back.
pub fn save_parameters(st: &State, path: &std::path::Path, target: (u8, u8)) -> Result<usize, String> {
    use std::fmt::Write as _;
    let mut out = String::new();
    let _ = writeln!(out, "# Onboard parameters for Vehicle {}", target.0);
    let _ = writeln!(out, "# Saved by lazypx4 {}", chrono::Local::now().format("%Y-%m-%d %H:%M:%S"));
    let _ = writeln!(out, "#");
    let _ = writeln!(out, "# Vehicle-Id Component-Id Name Value Type");
    let mut n = 0;
    for p in st.parameter_order.iter().filter_map(|name| st.parameters.get(name)) {
        let value = if is_integer(p.param_type) {
            format!("{}", p.value.round() as i64)
        } else {
            // Shortest text that round-trips the f32 PX4 actually stores.
            format!("{}", p.value as f32)
        };
        let t = if p.param_type == 0 { 9 } else { p.param_type };
        let _ = writeln!(out, "{}\t{}\t{}\t{}\t{}", target.0, target.1, p.name, value, t);
        n += 1;
    }
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    std::fs::write(path, out).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(n)
}

/// `(NAME, value text)` pairs from a parameter file. Accepts, line by line:
/// QGC `.params` (`1 1 NAME value type`), MAVProxy / plain `NAME value` or
/// `NAME,value`, `param set NAME value` (NSH scripts), and YAML-style
/// `NAME: value` (e.g. a tmux session's custom_config.yaml - group headers
/// with no value are skipped). `#` starts a comment.
pub fn parse_param_file(text: &str) -> Vec<(String, String)> {
    let is_name = |s: &str| {
        !s.is_empty()
            && s.len() <= 16
            && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
            && s.chars().next().is_some_and(|c| c.is_ascii_alphabetic())
    };
    let is_number = |s: &str| s.parse::<f64>().is_ok_and(f64::is_finite);
    let mut out = Vec::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        let line = line.strip_prefix("param set ").unwrap_or(line);
        let fields: Vec<&str> = line
            .split(|c: char| c.is_whitespace() || c == ',' || c == ':')
            .filter(|f| !f.is_empty())
            .collect();
        let pair = match fields.as_slice() {
            // QGC: sysid compid NAME value [type]
            [a, b, name, value, ..] if is_number(a) && is_number(b) && is_name(name) => Some((*name, *value)),
            [name, value, ..] if is_name(name) => Some((*name, *value)),
            _ => None,
        };
        if let Some((name, value)) = pair
            && is_number(value) {
                out.push((name.to_string(), value.to_string()));
            }
    }
    out
}

/// Differences between two parameter sets (name -> value).
#[derive(Debug, Default, PartialEq)]
pub struct ParamDiff {
    /// (name, left, right) where both have it but the values differ.
    pub changed: Vec<(String, f64, f64)>,
    pub only_left: Vec<String>,
    pub only_right: Vec<String>,
    pub same: usize,
}

pub fn same_value(a: f64, b: f64) -> bool {
    (a - b).abs() <= PARAM_VALUE_EPSILON * a.abs().max(b.abs()).max(1.0)
}

pub fn diff_parameters(left: &[(String, f64)], right: &[(String, f64)]) -> ParamDiff {
    let l: std::collections::BTreeMap<&str, f64> = left.iter().map(|(n, v)| (n.as_str(), *v)).collect();
    let r: std::collections::BTreeMap<&str, f64> = right.iter().map(|(n, v)| (n.as_str(), *v)).collect();
    let mut d = ParamDiff::default();
    for (name, &a) in &l {
        match r.get(name) {
            Some(&b) if same_value(a, b) => d.same += 1,
            Some(&b) => d.changed.push((name.to_string(), a, b)),
            None => d.only_left.push(name.to_string()),
        }
    }
    d.only_right = r.keys().filter(|n| !l.contains_key(*n)).map(|n| n.to_string()).collect();
    d
}

/// A parsed parameter file as numbers.
pub fn file_values(entries: &[(String, String)]) -> Vec<(String, f64)> {
    entries.iter().filter_map(|(n, v)| Some((n.clone(), v.parse::<f64>().ok()?))).collect()
}

#[derive(Debug, Default)]
pub struct LoadPlan {
    pub changes: Vec<(String, f64)>,
    pub unchanged: usize,
    /// In the file but not on this vehicle / firmware.
    pub unknown: Vec<String>,
    /// Present but not a valid value for its type (e.g. 2.5 for an int).
    pub invalid: Vec<String>,
}

/// Compare a parsed file against the vehicle's current values.
pub fn plan_parameter_load(st: &State, entries: &[(String, String)]) -> LoadPlan {
    let mut plan = LoadPlan::default();
    for (name, text) in entries {
        let Some(p) = st.parameters.get(name) else {
            plan.unknown.push(name.clone());
            continue;
        };
        let Ok(value) = parse_value(text, p) else {
            plan.invalid.push(name.clone());
            continue;
        };
        let t = if p.param_type == 0 { 9 } else { p.param_type };
        // What PX4 will store (and echo back): floats go through f32.
        let value = decode(encode(value, t), t);
        if value == p.value {
            plan.unchanged += 1;
        } else if let Some(pos) = plan.changes.iter().position(|(n, _)| n == name) {
            plan.changes[pos].1 = value; // last occurrence in the file wins
        } else {
            plan.changes.push((name.clone(), value));
        }
    }
    plan
}

pub fn start_parameter_load(st: &mut State, changes: Vec<(String, f64)>) {
    st.param_load_total = changes.len();
    st.param_load_failed.clear();
    st.param_load_queue = changes.into();
    st.command(format!("PARAM LOAD: setting {} parameter(s)", st.param_load_total));
}

pub fn param_load_active(st: &State) -> bool {
    st.param_load_total > 0
}

/// Send the next queued PARAM_SET once the previous one is confirmed or has
/// timed out (so PX4 is never flooded and every value gets an echo check).
pub fn process_parameter_load(link: &Link, st: &mut State) {
    if st.param_load_total == 0 || st.parameter_set_pending.is_some() {
        return;
    }
    if let Some((name, value)) = st.param_load_queue.pop_front() {
        if !send_parameter_set(link, st, &name, value) {
            st.param_load_failed.push(name);
        }
        return;
    }
    let total = st.param_load_total;
    let failed = std::mem::take(&mut st.param_load_failed);
    st.param_load_total = 0;
    if failed.is_empty() {
        st.info(format!("PARAM LOAD done: {total}/{total} confirmed"));
    } else {
        st.warn(format!(
            "PARAM LOAD done: {}/{total} confirmed, failed: {}",
            total - failed.len(),
            failed.join(" ")
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integer_params_round_trip_through_the_float_bits() {
        for (value, t) in [(1.0, 6), (-5.0, 6), (4_294_967_295.0, 5), (123456789.0, 5), (7.0, 1)] {
            assert_eq!(decode(encode(value, t), t), value);
        }
        assert_eq!(decode(encode(2.5, 9), 9), 2.5);
    }

    #[test]
    fn parses_qgc_plain_nsh_and_yaml_param_files() {
        let text = "# Onboard parameters\n\
                    1\t1\tMPC_XY_VEL_MAX\t5\t9\n\
                    COM_OBL_RC_ACT 3\n\
                    RTL_TYPE,1\n\
                    param set NAV_DLL_ACT 0\n\
                    velocity_limits:\n  MPC_ACC_HOR_MAX: 40.0     #5 [m/s^2] comment\n\
                    launch:\n  PX4_GZ_WORLD: cylinder_forest\n";
        let got = parse_param_file(text);
        let names: Vec<(&str, &str)> = got.iter().map(|(n, v)| (n.as_str(), v.as_str())).collect();
        assert_eq!(
            names,
            [
                ("MPC_XY_VEL_MAX", "5"),
                ("COM_OBL_RC_ACT", "3"),
                ("RTL_TYPE", "1"),
                ("NAV_DLL_ACT", "0"),
                ("MPC_ACC_HOR_MAX", "40.0"),
            ]
        );
    }

    #[test]
    fn diffs_two_parameter_sets() {
        let v = |pairs: &[(&str, f64)]| pairs.iter().map(|(n, x)| (n.to_string(), *x)).collect::<Vec<_>>();
        let d = diff_parameters(&v(&[("A", 1.0), ("B", 2.0), ("C", 0.1)]), &v(&[("A", 1.0), ("B", 3.0), ("D", 4.0), ("C", 0.100000001)]));
        assert_eq!(d.changed, vec![("B".to_string(), 2.0, 3.0)]);
        assert_eq!(d.only_left, Vec::<String>::new());
        assert_eq!(d.only_right, vec!["D".to_string()]);
        assert_eq!(d.same, 2);
    }

    #[test]
    fn g_format_matches_python() {
        assert_eq!(fmt_g(2.5, 6), "2.5");
        assert_eq!(fmt_g(0.1, 6), "0.1");
        assert_eq!(fmt_g(100.0, 6), "100");
        assert_eq!(fmt_g(1234567.0, 6), "1.23457e+06");
        assert_eq!(fmt_g(0.00001, 6), "1e-05");
    }
}
