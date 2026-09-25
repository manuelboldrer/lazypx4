//! PX4 parameter metadata (descriptions, enum / bitmask meanings, units).
//!
//! The MAVLink parameter protocol only carries a name and a number. Like
//! QGroundControl we read PX4's `parameters.json` for what the number means:
//! a compact copy (`data/param_meta.json.xz`) is bundled into the binary, and
//! `--param-defaults FILE` can point at the exact firmware's file instead.
//!
//! Compact format: `{"px4": "v1.17.0", "params": {NAME: entry}}` where an
//! entry has any of `s` short description, `l` long description, `u` units,
//! `min` / `max` / `d` (default), `e` enum `[[value, text], ...]`, `b`
//! bitmask `[[bit, text], ...]` and `r` (reboot required).

use std::collections::HashMap;
use std::io::Read;

use serde_json::Value;

static BUNDLED: &[u8] = include_bytes!("../data/param_meta.json.xz");

#[derive(Debug, Default, Clone)]
pub struct Entry {
    pub short: String,
    pub long: String,
    pub units: String,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub default: Option<f64>,
    pub enum_values: Vec<(f64, String)>,
    pub bitmask: Vec<(u32, String)>,
    pub reboot: bool,
}

#[derive(Default)]
pub struct ParamMeta {
    entries: HashMap<String, Entry>,
    pub source: String,
}

fn num(v: Option<&Value>) -> Option<f64> {
    match v? {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse().ok(),
        _ => None,
    }
}

/// A string field, with the line breaks PX4's metadata carries folded to
/// spaces (a raw newline would vanish from a single terminal row).
fn text(v: Option<&Value>) -> String {
    v.and_then(Value::as_str)
        .unwrap_or_default()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

impl ParamMeta {
    /// Load `path` (PX4 `parameters.json`, plain or `.xz`, or our compact
    /// file), falling back to the bundled copy. Never fails - a bad file just
    /// leaves the descriptions off.
    pub fn load(path: Option<&str>) -> Self {
        if let Some(path) = path
            && let Ok(bytes) = std::fs::read(path)
                && let Some(meta) = Self::parse(&bytes, path.ends_with(".xz")) {
                    return ParamMeta { source: path.to_string(), ..meta };
                }
        Self::parse(BUNDLED, true)
            .map(|m| ParamMeta { source: "bundled".into(), ..m })
            .unwrap_or_default()
    }

    fn parse(bytes: &[u8], xz: bool) -> Option<Self> {
        let json = if xz {
            let mut out = String::new();
            xz2::read::XzDecoder::new(bytes).read_to_string(&mut out).ok()?;
            out
        } else {
            String::from_utf8(bytes.to_vec()).ok()?
        };
        let data: Value = serde_json::from_str(&json).ok()?;

        let mut entries = HashMap::new();
        if let Some(list) = data.get("parameters").and_then(Value::as_array) {
            // PX4's own parameters.json.
            for p in list {
                let Some(name) = p.get("name").and_then(Value::as_str) else { continue };
                let entry = Entry {
                    short: text(p.get("shortDesc")),
                    long: text(p.get("longDesc")),
                    units: text(p.get("units")),
                    min: num(p.get("min")),
                    max: num(p.get("max")),
                    default: num(p.get("default")),
                    enum_values: p
                        .get("values")
                        .and_then(Value::as_array)
                        .map(|vs| {
                            vs.iter()
                                .filter_map(|v| Some((num(v.get("value"))?, text(v.get("description")))))
                                .collect()
                        })
                        .unwrap_or_default(),
                    bitmask: p
                        .get("bitmask")
                        .and_then(Value::as_array)
                        .map(|bs| {
                            bs.iter()
                                .filter_map(|b| Some((num(b.get("index"))? as u32, text(b.get("description")))))
                                .collect()
                        })
                        .unwrap_or_default(),
                    reboot: p.get("rebootRequired").and_then(Value::as_bool).unwrap_or(false),
                };
                entries.insert(name.to_string(), entry);
            }
        } else if let Some(map) = data.get("params").and_then(Value::as_object) {
            for (name, e) in map {
                let pairs = |key: &str| -> Vec<(f64, String)> {
                    e.get(key)
                        .and_then(Value::as_array)
                        .map(|vs| {
                            vs.iter()
                                .filter_map(|pair| {
                                    let pair = pair.as_array()?;
                                    Some((num(pair.first())?, text(pair.get(1))))
                                })
                                .collect()
                        })
                        .unwrap_or_default()
                };
                let entry = Entry {
                    short: text(e.get("s")),
                    long: text(e.get("l")),
                    units: text(e.get("u")),
                    min: num(e.get("min")),
                    max: num(e.get("max")),
                    default: num(e.get("d")),
                    enum_values: pairs("e"),
                    bitmask: pairs("b").into_iter().map(|(b, t)| (b as u32, t)).collect(),
                    reboot: e.get("r").and_then(Value::as_bool).unwrap_or(false),
                };
                entries.insert(name.clone(), entry);
            }
        } else {
            return None;
        }
        Some(ParamMeta { entries, source: String::new() })
    }

    pub fn get(&self, name: &str) -> Option<&Entry> {
        self.entries.get(name)
    }

    /// name -> default value, for the "changed from default" view.
    pub fn defaults(&self) -> HashMap<String, f64> {
        self.entries
            .iter()
            .filter_map(|(k, e)| Some((k.clone(), e.default?)))
            .collect()
    }
}

impl Entry {
    pub fn enum_label(&self, value: f64) -> Option<&str> {
        self.enum_values
            .iter()
            .find(|(v, _)| (v - value).abs() < 1e-6)
            .map(|(_, t)| t.as_str())
    }

    /// Texts of the set bits, or None if this isn't a bitmask parameter.
    pub fn bitmask_labels(&self, value: f64) -> Option<Vec<&str>> {
        if self.bitmask.is_empty() || !value.is_finite() {
            return None;
        }
        let raw = value.round() as i64;
        Some(
            self.bitmask
                .iter()
                .filter(|(bit, _)| *bit < 63 && raw >> bit & 1 == 1)
                .map(|(_, t)| t.as_str())
                .collect(),
        )
    }

    /// Short read-out for the value column: the enum label, the set bitmask
    /// bits, or the unit.
    pub fn meaning(&self, value: f64) -> String {
        if !self.enum_values.is_empty() {
            return self.enum_label(value).unwrap_or("?").to_string();
        }
        if let Some(labels) = self.bitmask_labels(value) {
            return if labels.is_empty() { "none".into() } else { labels.join(", ") };
        }
        self.units.clone()
    }
}
