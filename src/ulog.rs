//! PX4 ULog reader for the flight-review pane (ported from lazyjagg):
//! flattens every logged topic into time series named the PlotJuggler way
//! (`/vehicle_attitude/q[0]`, a second instance as `/sensor_accel.01/x`).

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

/// One field over time; the time column is shared by a topic's fields.
#[derive(Clone, Debug, Default)]
pub struct Series {
    pub name: String,
    pub t: Arc<Vec<f64>>,
    pub v: Vec<f64>,
    /// Text of each value, for enums shown by name (flight mode).
    pub labels: Option<Vec<String>>,
}

impl Series {
    pub fn len(&self) -> usize {
        self.v.len()
    }

    pub fn is_empty(&self) -> bool {
        self.v.is_empty()
    }

    /// Index of the last sample at or before `t`.
    pub fn index_at(&self, t: f64) -> Option<usize> {
        self.t.partition_point(|&x| x <= t).checked_sub(1)
    }

    pub fn value_at(&self, t: f64) -> Option<f64> {
        self.index_at(t).map(|i| self.v[i])
    }

    /// Value at `t` for reading: the number, or its label.
    pub fn text_at(&self, t: f64) -> Option<String> {
        let i = self.index_at(t)?;
        Some(match &self.labels {
            Some(l) => l.get(self.v[i] as usize).cloned().unwrap_or_else(|| fmt_value(self.v[i])),
            None => fmt_value(self.v[i]),
        })
    }

    /// Sample range [first, last) inside [t0, t1], widened by one sample on
    /// each side so a line reaches the plot edges.
    pub fn visible(&self, t0: f64, t1: f64) -> (usize, usize) {
        let a = self.t.partition_point(|&x| x < t0).saturating_sub(1);
        let b = (self.t.partition_point(|&x| x <= t1) + 1).min(self.len());
        (a, b)
    }
}

/// "12.345", compact and stable in a narrow column.
pub fn fmt_value(v: f64) -> String {
    if !v.is_finite() {
        return if v.is_nan() { "nan".into() } else if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if v == 0.0 {
        return "0".into();
    }
    let a = v.abs();
    if !(1e-3..1e7).contains(&a) {
        return format!("{v:.3e}");
    }
    let s = format!("{v:.*}", (5 - (a.log10().floor() as i32 + 1)).clamp(0, 6) as usize);
    if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s }
}

/// What a log says besides its series.
#[derive(Clone, Debug, Default)]
pub struct Meta {
    pub info: Vec<(String, String)>,
    pub params: Vec<(String, String)>,
    /// (time, level, text)
    pub messages: Vec<(f64, String, String)>,
    /// Data gaps: (time, seconds).
    pub dropouts: Vec<(f64, f64)>,
    /// When logging began: topics last published before it shouldn't
    /// stretch the view.
    pub start: Option<f64>,
}

#[derive(Default)]
pub struct Loaded {
    pub series: Vec<Series>,
    pub meta: Meta,
    pub warnings: Vec<String>,
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
    Bool,
}

impl Kind {
    fn parse(s: &str) -> Option<(Kind, usize)> {
        Some(match s {
            "int8_t" => (Kind::I8, 1),
            "uint8_t" => (Kind::U8, 1),
            "int16_t" => (Kind::I16, 2),
            "uint16_t" => (Kind::U16, 2),
            "int32_t" => (Kind::I32, 4),
            "uint32_t" => (Kind::U32, 4),
            "int64_t" => (Kind::I64, 8),
            "uint64_t" => (Kind::U64, 8),
            "float" => (Kind::F32, 4),
            "double" => (Kind::F64, 8),
            "bool" => (Kind::Bool, 1),
            _ => return None,
        })
    }

    #[inline]
    fn read(self, b: &[u8]) -> f64 {
        match self {
            Kind::I8 => b[0] as i8 as f64,
            Kind::U8 => b[0] as f64,
            Kind::Bool => (b[0] != 0) as u8 as f64,
            Kind::I16 => i16::from_le_bytes([b[0], b[1]]) as f64,
            Kind::U16 => u16::from_le_bytes([b[0], b[1]]) as f64,
            Kind::I32 => i32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
            Kind::U32 => u32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
            Kind::I64 => i64::from_le_bytes(b[..8].try_into().unwrap()) as f64,
            Kind::U64 => u64::from_le_bytes(b[..8].try_into().unwrap()) as f64,
            Kind::F32 => f32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
            Kind::F64 => f64::from_le_bytes(b[..8].try_into().unwrap()),
        }
    }
}

struct Field {
    ty: String,
    array: usize,
    name: String,
}

struct Leaf {
    name: String,
    kind: Kind,
    offset: usize,
    size: usize,
}

struct Sub {
    topic: String,
    multi_id: u8,
    leaves: Vec<Leaf>,
    ts: Option<(Kind, usize, usize)>,
    t: Vec<f64>,
    cols: Vec<Vec<f64>>,
}

fn parse_format(text: &str) -> Option<(String, Vec<Field>)> {
    let (name, rest) = text.split_once(':')?;
    let mut fields = Vec::new();
    for item in rest.split(';').filter(|s| !s.is_empty()) {
        let (ty, field) = item.rsplit_once(' ')?;
        let (ty, array) = match ty.split_once('[') {
            Some((t, n)) => (t, n.trim_end_matches(']').parse().ok()?),
            None => (ty, 1),
        };
        fields.push(Field { ty: ty.to_string(), array, name: field.to_string() });
    }
    Some((name.to_string(), fields))
}

fn size_of(ty: &str, formats: &HashMap<String, Vec<Field>>, depth: usize) -> usize {
    if let Some((_, n)) = Kind::parse(ty) {
        return n;
    }
    if ty == "char" {
        return 1;
    }
    if depth > 16 {
        return 0;
    }
    formats.get(ty).map_or(0, |f| f.iter().map(|x| size_of(&x.ty, formats, depth + 1) * x.array).sum())
}

#[allow(clippy::too_many_arguments)]
fn flatten(fields: &[Field], formats: &HashMap<String, Vec<Field>>, prefix: &str, offset: &mut usize, out: &mut Vec<Leaf>, max_array: usize, depth: usize) {
    for f in fields {
        let elem = size_of(&f.ty, formats, depth);
        if f.name.starts_with("_padding") || f.ty == "char" {
            *offset += elem * f.array;
            continue;
        }
        for i in 0..f.array {
            let name = if f.array > 1 { format!("{prefix}/{}[{i}]", f.name) } else { format!("{prefix}/{}", f.name) };
            if i >= max_array {
                *offset += elem;
            } else if let Some((kind, size)) = Kind::parse(&f.ty) {
                out.push(Leaf { name, kind, offset: *offset, size });
                *offset += size;
            } else if let Some(nested) = formats.get(&f.ty).filter(|_| depth < 16) {
                flatten(nested, formats, &name, offset, out, max_array, depth + 1);
            } else {
                *offset += elem;
            }
        }
    }
}

fn scalar(ty: &str, b: &[u8]) -> String {
    if ty.starts_with("char") {
        return String::from_utf8_lossy(b).trim_end_matches('\0').to_string();
    }
    match Kind::parse(ty) {
        Some((k, n)) if b.len() >= n => fmt_value(k.read(b)),
        _ => format!("<{} bytes>", b.len()),
    }
}

fn key_value(p: &[u8]) -> Option<(String, String, &[u8])> {
    let n = *p.first()? as usize;
    let key = std::str::from_utf8(p.get(1..1 + n)?).ok()?;
    let (ty, name) = key.rsplit_once(' ')?;
    Some((ty.to_string(), name.to_string(), p.get(1 + n..)?))
}

fn level(c: u8) -> String {
    let i = if (b'0'..=b'7').contains(&c) { (c - b'0') as usize } else { 6 };
    ["EMERG", "ALERT", "CRIT", "ERROR", "WARN", "NOTICE", "INFO", "DEBUG"][i].to_string()
}

/// Parses a whole log; `progress` gets the percentage read.
pub fn load(bytes: &[u8], max_array: usize, progress: &AtomicU64) -> Result<Loaded, String> {
    if bytes.len() < 16 || &bytes[..7] != b"ULog\x01\x12\x35" {
        return Err("not a ULog file".into());
    }
    let mut out = Loaded::default();
    let header_time = u64::from_le_bytes(bytes[8..16].try_into().unwrap()) as f64 * 1e-6;
    out.meta.start = (header_time > 0.0).then_some(header_time);
    let mut formats: HashMap<String, Vec<Field>> = HashMap::new();
    let mut subs: BTreeMap<u16, Sub> = BTreeMap::new();
    let mut multi: BTreeMap<String, String> = BTreeMap::new();
    let mut last_time = 0.0f64;
    let mut pos = 16usize;
    let total = bytes.len().max(1) as u64;
    let mut in_data = false;

    while pos + 3 <= bytes.len() {
        let size = u16::from_le_bytes([bytes[pos], bytes[pos + 1]]) as usize;
        let kind = bytes[pos + 2];
        let Some(p) = bytes.get(pos + 3..pos + 3 + size) else {
            out.warnings.push(format!("log ends mid-message at byte {pos}"));
            break;
        };
        pos += 3 + size;
        match kind {
            b'F' => {
                if let Some((name, fields)) = std::str::from_utf8(p).ok().and_then(parse_format) {
                    formats.insert(name, fields);
                }
            }
            b'I' => {
                if let Some((ty, key, v)) = key_value(p) {
                    out.meta.info.push((key, scalar(&ty, v)));
                }
            }
            b'M' => {
                if let Some((ty, key, v)) = p.get(1..).and_then(key_value) {
                    let text = scalar(&ty, v);
                    multi.entry(key).or_default().push_str(&text);
                }
            }
            b'P' => {
                if let Some((ty, key, v)) = key_value(p) {
                    let val = scalar(&ty, v);
                    if in_data {
                        out.meta.messages.push((last_time, "PARAM".into(), format!("{key} = {val}")));
                    }
                    out.meta.params.push((key, val));
                }
            }
            b'A' if p.len() >= 3 => {
                let id = u16::from_le_bytes([p[1], p[2]]);
                let name = String::from_utf8_lossy(&p[3..]).to_string();
                let Some(fields) = formats.get(&name) else { continue };
                let mut leaves = Vec::new();
                flatten(fields, &formats, "", &mut 0, &mut leaves, max_array, 0);
                let ts = leaves.iter().position(|l| l.name == "/timestamp").map(|i| leaves.remove(i)).map(|l| (l.kind, l.offset, l.size));
                let cols = (0..leaves.len()).map(|_| Vec::new()).collect();
                subs.insert(id, Sub { topic: name, multi_id: p[0], leaves, ts, t: Vec::new(), cols });
                in_data = true;
            }
            b'D' if p.len() >= 2 => {
                let id = u16::from_le_bytes([p[0], p[1]]);
                let Some(sub) = subs.get_mut(&id) else { continue };
                let body = &p[2..];
                let Some((kind, off, n)) = sub.ts else { continue };
                let Some(raw) = body.get(off..off + n) else { continue };
                let t = kind.read(raw) * 1e-6;
                last_time = t;
                sub.t.push(t);
                for (leaf, col) in sub.leaves.iter().zip(sub.cols.iter_mut()) {
                    // A logger may cut trailing padding; keep columns aligned.
                    col.push(body.get(leaf.offset..leaf.offset + leaf.size).map_or(f64::NAN, |b| leaf.kind.read(b)));
                }
            }
            b'L' if p.len() >= 9 => {
                let t = u64::from_le_bytes(p[1..9].try_into().unwrap()) as f64 * 1e-6;
                out.meta.messages.push((t, level(p[0]), String::from_utf8_lossy(&p[9..]).trim().to_string()));
            }
            b'C' if p.len() >= 11 => {
                let t = u64::from_le_bytes(p[3..11].try_into().unwrap()) as f64 * 1e-6;
                out.meta.messages.push((t, level(p[0]), String::from_utf8_lossy(&p[11..]).trim().to_string()));
            }
            b'O' if p.len() >= 2 => {
                out.meta.dropouts.push((last_time, u16::from_le_bytes([p[0], p[1]]) as f64 * 1e-3));
            }
            _ => {}
        }
        if pos & 0xfffff < 64 {
            progress.store(pos as u64 * 100 / total, Ordering::Relaxed);
        }
    }
    out.meta.info.extend(multi);

    let mut instances: HashMap<String, usize> = HashMap::new();
    for s in subs.values().filter(|s| !s.t.is_empty()) {
        *instances.entry(s.topic.clone()).or_default() += 1;
    }
    for sub in subs.into_values() {
        if sub.t.is_empty() {
            continue;
        }
        let topic = if instances[&sub.topic] > 1 { format!("/{}.{:02}", sub.topic, sub.multi_id) } else { format!("/{}", sub.topic) };
        let t = Arc::new(sub.t);
        for (leaf, v) in sub.leaves.iter().zip(sub.cols) {
            out.series.push(Series { name: format!("{topic}{}", leaf.name), t: t.clone(), v, labels: None });
        }
    }
    out.meta.messages.sort_by(|a, b| a.0.total_cmp(&b.0));
    progress.store(100, Ordering::Relaxed);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn msg(kind: u8, body: &[u8]) -> Vec<u8> {
        let mut v = (body.len() as u16).to_le_bytes().to_vec();
        v.push(kind);
        v.extend_from_slice(body);
        v
    }

    #[test]
    fn parses_nested_arrays_and_padding() {
        let mut f = b"ULog\x01\x12\x35\x01".to_vec();
        f.extend_from_slice(&0u64.to_le_bytes());
        f.extend(msg(b'F', b"inner:float a;uint8_t[3] _padding0;"));
        f.extend(msg(b'F', b"topic:uint64_t timestamp;inner[2] in;int16_t v;"));
        let mut add = vec![0u8, 7, 0];
        add.extend_from_slice(b"topic");
        f.extend(msg(b'A', &add));
        let mut d = vec![7u8, 0];
        d.extend_from_slice(&2_000_000u64.to_le_bytes());
        d.extend_from_slice(&1.5f32.to_le_bytes());
        d.extend_from_slice(&[0; 3]);
        d.extend_from_slice(&2.5f32.to_le_bytes());
        d.extend_from_slice(&[0; 3]);
        d.extend_from_slice(&(-3i16).to_le_bytes());
        f.extend(msg(b'D', &d));
        let l = load(&f, 100, &AtomicU64::new(0)).unwrap();
        let get = |n: &str| l.series.iter().find(|s| s.name == n).unwrap_or_else(|| panic!("{n}"));
        assert_eq!(get("/topic/in[0]/a").v, vec![1.5]);
        assert_eq!(get("/topic/in[1]/a").v, vec![2.5]);
        assert_eq!(get("/topic/v").v, vec![-3.0]);
        assert_eq!(*get("/topic/v").t, vec![2.0]);
        assert_eq!(l.meta.start, None, "no header time in this file");
    }

    #[test]
    fn value_format() {
        assert_eq!(fmt_value(0.0), "0");
        assert_eq!(fmt_value(1.5), "1.5");
        assert_eq!(fmt_value(1234.5678), "1234.6");
        assert_eq!(fmt_value(1e9), "1.000e9");
    }
}
