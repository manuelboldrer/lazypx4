//! [w] ROS: `/rosout` with a filter, one-key std_msgs publishes and std_srvs
//! calls, and a browser of the topics / services in the graph. Used by the
//! full UI and by the fleet view (which sends a relative name to every
//! marked vehicle, under its `uav_<sys>` namespace).
//!
//! The ROS side is one node on its own thread ([`spawn`], behind the `ros`
//! cargo feature): it collects `/rosout`, polls the topic graph and runs
//! the publish / call requests the pane queues. Messages are built as JSON
//! and sent through r2r's untyped publishers / clients, so any type the
//! build generated bindings for works - the short names (`bool`,
//! `trigger`, ...) just cover the standard ones.

// Without the `ros` feature the node half is compiled out.
#![cfg_attr(not(feature = "ros"), allow(dead_code))]

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use ratatui::style::{Modifier, Style};
use ratatui::text::Line;

use crate::host::lock;
use crate::state::now;
use crate::ui::{CYAN, GREEN, Ln, RED, YELLOW, blank, dim_line, section};

/// `/rosout` lines kept.
const MAX_LOG: usize = 5000;
/// Result lines kept.
const MAX_FEEDBACK: usize = 200;
/// A service that has not answered by then is reported (and dropped).
pub const CALL_TIMEOUT_S: f64 = 5.0;
/// A fresh publisher waits this long for its subscribers to be discovered
/// before the one-shot message goes out anyway.
pub const DISCOVERY_WAIT_S: f64 = 1.5;

// ---------------------------------------------------------------------------
// Config: [[ros.action]]
// ---------------------------------------------------------------------------

/// One `[[ros.action]]`: a topic to publish or a service to call.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ActionCfg {
    pub name: String,
    pub topic: Option<String>,
    pub service: Option<String>,
    /// Short (`bool`, `float64`, `trigger`, `setbool`...) or full
    /// (`std_msgs/msg/Bool`); blank = looked up in the graph / guessed.
    #[serde(rename = "type")]
    pub ty: Option<String>,
    /// Fixed value / JSON; omitted = t / f for bools, ENTER asks.
    pub value: Option<String>,
    /// Overrides `[ros] confirm` for this one.
    pub confirm: Option<bool>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Action {
    pub name: String,
    pub service: bool,
    /// As given: relative names go under each target's namespace.
    pub path: String,
    /// Full type, or "" (topic: from the graph; service: guessed).
    pub ty: String,
    pub value: Option<String>,
    pub confirm: bool,
    /// Added this session (p / s / the browser), not from the config.
    pub recent: bool,
}

impl Action {
    pub fn from_cfg(cfg: &ActionCfg, confirm_default: bool) -> Result<Action, String> {
        let (service, path) = match (&cfg.topic, &cfg.service) {
            (Some(t), None) => (false, t.trim().to_string()),
            (None, Some(s)) => (true, s.trim().to_string()),
            _ => return Err(format!("[[ros.action]] \"{}\": give exactly one of topic / service", cfg.name)),
        };
        if path.is_empty() {
            return Err(format!("[[ros.action]] \"{}\": empty name", cfg.name));
        }
        let ty = full_type(cfg.ty.as_deref().unwrap_or(""), service);
        let name = if cfg.name.trim().is_empty() { path.clone() } else { cfg.name.trim().to_string() };
        Ok(Action { name, service, path, ty, value: cfg.value.clone(), confirm: cfg.confirm.unwrap_or(confirm_default), recent: false })
    }

    /// t / f make sense for it.
    pub fn is_bool(&self) -> bool {
        matches!(self.ty.as_str(), "std_msgs/msg/Bool" | "std_srvs/srv/SetBool")
    }

    /// Sends without asking for a value.
    fn needs_no_value(&self) -> bool {
        self.value.is_some() || matches!(self.ty.as_str(), "std_msgs/msg/Empty" | "std_srvs/srv/Empty" | "std_srvs/srv/Trigger") || (self.service && self.ty.is_empty())
    }

    fn kind(&self) -> &'static str {
        if self.service { "call" } else { "pub" }
    }
}

// ---------------------------------------------------------------------------
// Pure helpers: names, types, values
// ---------------------------------------------------------------------------

/// `bool` -> `std_msgs/msg/Bool`, `trigger` -> `std_srvs/srv/Trigger`; a
/// full `pkg/msg/Type` (or `pkg/Type`) passes through.
pub fn full_type(ty: &str, service: bool) -> String {
    let ty = ty.trim();
    if ty.is_empty() {
        return String::new();
    }
    if ty.contains('/') {
        let parts: Vec<&str> = ty.split('/').collect();
        return match parts.as_slice() {
            [pkg, name] => format!("{pkg}/{}/{name}", if service { "srv" } else { "msg" }),
            _ => ty.to_string(),
        };
    }
    let lower = ty.to_lowercase();
    if service {
        match lower.as_str() {
            "trigger" => "std_srvs/srv/Trigger".into(),
            "setbool" | "set_bool" | "bool" => "std_srvs/srv/SetBool".into(),
            "empty" => "std_srvs/srv/Empty".into(),
            _ => ty.to_string(),
        }
    } else {
        let name = match lower.as_str() {
            "bool" => "Bool",
            "byte" => "Byte",
            "char" => "Char",
            "int8" => "Int8",
            "int16" => "Int16",
            "int32" | "int" => "Int32",
            "int64" => "Int64",
            "uint8" => "UInt8",
            "uint16" => "UInt16",
            "uint32" => "UInt32",
            "uint64" => "UInt64",
            "float32" => "Float32",
            "float64" | "float" | "double" => "Float64",
            "string" | "str" => "String",
            "empty" => "Empty",
            _ => return ty.to_string(),
        };
        format!("std_msgs/msg/{name}")
    }
}

/// The short name shown in lists (`std_msgs/msg/Bool` -> `Bool`).
pub fn short_type(ty: &str) -> &str {
    ty.rsplit('/').next().unwrap_or(ty)
}

/// Relative names go under the namespace; `/abs` stays; `~/x` is relative.
pub fn resolve(name: &str, ns: Option<&str>) -> String {
    let name = name.trim();
    if name.starts_with('/') {
        return name.to_string();
    }
    let name = name.trim_start_matches("~/");
    match ns.map(|n| n.trim_matches('/')).filter(|n| !n.is_empty()) {
        Some(ns) => format!("/{ns}/{name}"),
        None => format!("/{name}"),
    }
}

pub fn parse_bool(text: &str) -> Option<bool> {
    match text.trim().to_lowercase().as_str() {
        "true" | "t" | "1" | "on" | "yes" | "y" => Some(true),
        "false" | "f" | "0" | "off" | "no" | "n" => Some(false),
        _ => None,
    }
}

/// A service with no type given: SetBool for a bool argument, Trigger
/// for none.
pub fn guess_service_type(value: &str) -> Result<String, String> {
    let v = value.trim();
    if v.is_empty() {
        Ok("std_srvs/srv/Trigger".into())
    } else if parse_bool(v).is_some() {
        Ok("std_srvs/srv/SetBool".into())
    } else {
        Err("service type? give it: s NAME trigger|setbool|empty|pkg/srv/Type [ARG]".into())
    }
}

/// The message / request for `value`: JSON as given, else the `data` field
/// of the std_msgs scalars, SetBool's `data`, or `{}` for Empty / Trigger.
pub fn to_json(ty: &str, value: &str) -> Result<serde_json::Value, String> {
    use serde_json::json;
    let v = value.trim();
    if v.starts_with('{') {
        return serde_json::from_str(v).map_err(|e| format!("bad JSON: {e}"));
    }
    let name = ty.strip_prefix("std_msgs/msg/").or_else(|| ty.strip_prefix("std_srvs/srv/")).unwrap_or("");
    let int = || -> Result<serde_json::Value, String> {
        v.parse::<i64>().map(|n| json!({ "data": n })).map_err(|_| format!("{} wants an integer, not '{v}'", short_type(ty)))
    };
    match name {
        "Bool" | "SetBool" => parse_bool(v).map(|b| json!({ "data": b })).ok_or_else(|| format!("{} wants true / false, not '{v}'", short_type(ty))),
        "Empty" | "Trigger" if ty.starts_with("std_") => Ok(json!({})),
        "String" => Ok(json!({ "data": v })),
        "Int8" | "Int16" | "Int32" | "Int64" | "UInt8" | "UInt16" | "UInt32" | "UInt64" | "Byte" => int(),
        "Char" => match v.chars().count() {
            1 => Ok(json!({ "data": v.chars().next().unwrap() as u32 })),
            _ => int(),
        },
        "Float32" | "Float64" => v.parse::<f64>().map(|x| json!({ "data": x })).map_err(|_| format!("{} wants a number, not '{v}'", short_type(ty))),
        _ if v.is_empty() => Ok(json!({})),
        _ => Err(format!("{ty}: give the message as JSON, e.g. {{\"field\": 1}}")),
    }
}

/// "success=true message=done" for a response object.
pub fn response_text(v: &serde_json::Value) -> (bool, String) {
    let ok = v.get("success").and_then(|s| s.as_bool()).unwrap_or(true);
    let text = match v.as_object() {
        Some(o) if o.is_empty() => "ok".to_string(),
        Some(o) => {
            // `success` first: it is what one looks for in a Trigger reply.
            let mut fields: Vec<(&String, &serde_json::Value)> = o.iter().filter(|(k, _)| k.as_str() != "structure_needs_at_least_one_member").collect();
            fields.sort_by_key(|(k, _)| k.as_str() != "success");
            fields
                .into_iter()
                .map(|(k, v)| match v {
                serde_json::Value::String(s) => format!("{k}={s:?}"),
                other => format!("{k}={other}"),
            })
                .collect::<Vec<_>>()
                .join(" ")
        }
        None => v.to_string(),
    };
    (ok, text)
}

/// `p` / `s` line: NAME [TYPE] [VALUE...]. The second word is a type when
/// it looks like one (a short std name or contains '/').
pub fn parse_line(line: &str, service: bool) -> Result<(String, String, String), String> {
    let line = line.trim();
    let (name, rest) = line.split_once(char::is_whitespace).map(|(a, b)| (a, b.trim())).unwrap_or((line, ""));
    if name.is_empty() {
        return Err(if service { "call: SERVICE [TYPE] [ARG]".into() } else { "publish: TOPIC [TYPE] VALUE".into() });
    }
    let (first, after) = rest.split_once(char::is_whitespace).map(|(a, b)| (a, b.trim())).unwrap_or((rest, ""));
    let looks_like_type = !first.is_empty() && !first.starts_with('{') && (first.contains('/') || full_type(first, service).contains('/'));
    // "bool" alone after a topic is a type with no value; for a service
    // "true" is a value, never a type.
    if looks_like_type && !(service && parse_bool(first).is_some()) {
        Ok((name.to_string(), full_type(first, service), after.to_string()))
    } else {
        Ok((name.to_string(), String::new(), rest.to_string()))
    }
}

// ---------------------------------------------------------------------------
// Shared state between the pane and the node thread
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct LogLine {
    /// ROS stamp, seconds.
    pub stamp: f64,
    pub level: u8,
    pub name: String,
    pub msg: String,
}

#[derive(Debug, Clone)]
pub struct Feedback {
    pub seq: u64,
    pub time: f64,
    /// None = in progress / info.
    pub ok: Option<bool>,
    pub text: String,
}

#[derive(Debug, Clone)]
pub struct Request {
    pub service: bool,
    /// Resolved (absolute).
    pub name: String,
    /// Full type or "".
    pub ty: String,
    pub value: String,
}

#[derive(Debug, Default)]
pub struct ConsoleState {
    pub available: bool,
    pub error: String,
    pub log: VecDeque<LogLine>,
    /// Lines received in total (the deque drops the oldest).
    pub received: u64,
    pub requests: Vec<Request>,
    pub feedback: VecDeque<Feedback>,
    next_seq: u64,
    /// (name, type) of every topic in the graph, sorted.
    pub topics: Vec<(String, String)>,
}

impl ConsoleState {
    pub fn push_log(&mut self, line: LogLine) {
        self.received += 1;
        self.log.push_back(line);
        while self.log.len() > MAX_LOG {
            self.log.pop_front();
        }
    }

    pub fn note(&mut self, ok: Option<bool>, text: impl Into<String>) {
        self.next_seq += 1;
        self.feedback.push_back(Feedback { seq: self.next_seq, time: now(), ok, text: text.into() });
        while self.feedback.len() > MAX_FEEDBACK {
            self.feedback.pop_front();
        }
    }
}

pub type Console = Arc<Mutex<ConsoleState>>;

pub fn new_console() -> Console {
    Arc::default()
}

/// Feedback lines newer than `*seen` (and moves `*seen` on).
pub fn take_new_feedback(console: &Console, seen: &mut u64) -> Vec<Feedback> {
    let c = lock(console);
    let out: Vec<Feedback> = c.feedback.iter().filter(|f| f.seq > *seen).cloned().collect();
    if let Some(last) = out.last() {
        *seen = last.seq;
    }
    out
}

#[cfg(not(feature = "ros"))]
pub fn spawn(console: Console, _shutdown: Arc<std::sync::atomic::AtomicBool>) -> Option<std::thread::JoinHandle<()>> {
    lock(&console).error = "built without ROS 2 support (rebuild with --features ros, or install/install.sh --ros)".into();
    None
}

#[cfg(feature = "ros")]
pub use imp::spawn;

// ---------------------------------------------------------------------------
// The pane: keys and drawing
// ---------------------------------------------------------------------------

/// Where a relative name goes: one per target vehicle.
#[derive(Debug, Clone)]
pub struct Target {
    pub label: String,
    pub ns: Option<String>,
}

const LEVELS: [(u8, &str); 5] = [(10, "DEBUG"), (20, "INFO"), (30, "WARN"), (40, "ERROR"), (50, "FATAL")];

fn level_name(level: u8) -> &'static str {
    LEVELS.iter().rev().find(|(l, _)| level >= *l).map(|(_, n)| *n).unwrap_or("DEBUG")
}

fn level_style(level: u8) -> Style {
    match level {
        l if l >= 50 => Style::new().fg(RED).add_modifier(Modifier::BOLD),
        l if l >= 40 => Style::new().fg(RED),
        l if l >= 30 => Style::new().fg(YELLOW),
        l if l >= 20 => Style::new(),
        _ => Style::new().add_modifier(Modifier::DIM),
    }
}

enum Prompt {
    /// `/`: live filter of the log (or the browser).
    Filter,
    /// `p` / `s` line.
    Line { service: bool, text: String },
    /// A value for the selected action.
    Value { action: Action, text: String },
    Confirm { text: String, buffer: String, requests: Vec<Request> },
}

/// One row of the graph browser.
#[derive(Debug, Clone)]
pub struct GraphEntry {
    pub service: bool,
    pub name: String,
    pub ty: String,
}

pub struct Browser {
    pub cursor: usize,
    pub all_types: bool,
    pub filter: String,
    services: Vec<(String, String)>,
    loading: bool,
    error: String,
    rx: Option<ServiceList>,
}

/// `ros2 service list -t` result: (name, type) rows or an error line.
type ServiceList = std::sync::mpsc::Receiver<Result<Vec<(String, String)>, String>>;

fn is_standard(ty: &str) -> bool {
    ty.starts_with("std_msgs/") || ty.starts_with("std_srvs/")
}

/// `ros2 service list -t` (r2r has no service-graph query), in a thread.
fn fetch_services(script: String) -> ServiceList {
    let (tx, rx) = std::sync::mpsc::channel();
    let _ = std::thread::Builder::new().name("RosServices".into()).spawn(move || {
        let out = std::process::Command::new("bash").arg("-c").arg(&script).stdin(std::process::Stdio::null()).output();
        let _ = tx.send(match out {
            Ok(o) if o.status.success() => Ok(String::from_utf8_lossy(&o.stdout)
                .lines()
                .filter_map(|l| {
                    let (name, ty) = l.trim().split_once(' ')?;
                    Some((name.to_string(), ty.trim().trim_matches(['[', ']']).to_string()))
                })
                .filter(|(n, _)| n.starts_with('/'))
                .collect()),
            Ok(o) => {
                let err = String::from_utf8_lossy(&o.stderr);
                Err(err.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("ros2 service list failed").trim().to_string())
            }
            Err(e) => Err(format!("could not run ros2 service list: {e}")),
        });
    });
    rx
}

/// One word of the `/rosout` filter: `word` includes, `!word` excludes.
struct FilterTerm {
    negate: bool,
    /// Lowercased text, used when the word is not a valid regex.
    text: String,
    re: Option<regex::Regex>,
}

impl FilterTerm {
    fn parse(word: &str) -> Option<Self> {
        let (negate, plain) = match word.strip_prefix('!') {
            Some(rest) => (true, rest),
            None => (false, word),
        };
        (!plain.is_empty()).then(|| FilterTerm {
            negate,
            text: plain.to_lowercase(),
            re: regex::RegexBuilder::new(plain).case_insensitive(true).build().ok(),
        })
    }

    fn hits(&self, hay: &str, hay_lower: &str) -> bool {
        match &self.re {
            Some(re) => re.is_match(hay),
            None => hay_lower.contains(&self.text),
        }
    }
}

pub struct RosPane {
    pub actions: Vec<Action>,
    pub action_cursor: usize,
    pub filter: String,
    filter_terms: Vec<FilterTerm>,
    pub min_level: u8,
    /// Only lines from the targets' namespaces.
    pub ns_only: bool,
    pub follow: bool,
    /// First filtered row shown when not following.
    pub top: usize,
    prompt: Option<Prompt>,
    /// Local errors (bad input) until the next key.
    note: Option<(bool, String)>,
    pub browser: Option<Browser>,
    confirm_default: bool,
    /// For `ros2 service list` (the [record] ros_setup).
    ros_setup: String,
    /// Log rows visible last frame (PGUP / PGDN step).
    page: std::cell::Cell<usize>,
    /// Filtered rows last frame.
    filtered_len: std::cell::Cell<usize>,
}

impl RosPane {
    /// The pane plus any `[[ros.action]]` problems to report.
    pub fn new(cfgs: &[ActionCfg], confirm_default: bool, ros_setup: &str) -> (Self, Vec<String>) {
        let mut errors = Vec::new();
        let actions = cfgs
            .iter()
            .filter_map(|c| match Action::from_cfg(c, confirm_default) {
                Ok(a) => Some(a),
                Err(e) => {
                    errors.push(e);
                    None
                }
            })
            .collect();
        let pane = RosPane {
            actions,
            action_cursor: 0,
            filter: String::new(),
            filter_terms: Vec::new(),
            min_level: 20,
            ns_only: false,
            follow: true,
            top: 0,
            prompt: None,
            note: None,
            browser: None,
            confirm_default,
            ros_setup: ros_setup.to_string(),
            page: std::cell::Cell::new(10),
            filtered_len: std::cell::Cell::new(0),
        };
        (pane, errors)
    }

    /// Keys are text right now (no remaps, no global hotkeys).
    pub fn typing(&self) -> bool {
        self.prompt.is_some()
    }

    /// Footer for the key bar.
    pub fn footer(&self) -> &'static str {
        match (&self.prompt, &self.browser) {
            (Some(_), _) => "ENTER submit · ESC cancel",
            (None, Some(_)) => "j/k select · ENTER add as action · [a] all types · / filter · [r] refresh · [l]/ESC back",
            _ => "[ ] action · ENTER send · t/f true/false · [p] publish · [s] call · [l] graph · / filter · [L] level · [v] ns · [c] clear",
        }
    }

    /// Space-separated words: a line is shown when it matches any plain
    /// word (or there are none) and none of the `!word`s.
    fn set_filter(&mut self, text: String) {
        self.filter_terms = text.split_whitespace().filter_map(FilterTerm::parse).collect();
        self.filter = text;
    }

    fn line_matches(&self, line: &LogLine, namespaces: &[String]) -> bool {
        if line.level < self.min_level {
            return false;
        }
        if self.ns_only && !namespaces.is_empty() {
            let hit = namespaces.iter().any(|ns| line.name == *ns || line.name.starts_with(&format!("{ns}.")) || line.name.contains(&format!(".{ns}.")));
            if !hit {
                return false;
            }
        }
        if self.filter_terms.is_empty() {
            return true;
        }
        let hay = format!("{} {}", line.name, line.msg);
        let lower = hay.to_lowercase();
        let mut any_include = false;
        let mut included = false;
        for t in &self.filter_terms {
            let hit = t.hits(&hay, &lower);
            if t.negate {
                if hit {
                    return false;
                }
            } else {
                any_include = true;
                included |= hit;
            }
        }
        included || !any_include
    }

    /// The service list fetch.
    pub fn poll(&mut self) {
        let Some(b) = self.browser.as_mut() else { return };
        let Some(rx) = b.rx.as_ref() else { return };
        match rx.try_recv() {
            Ok(r) => {
                b.rx = None;
                b.loading = false;
                match r {
                    Ok(s) => b.services = s,
                    Err(e) => b.error = e,
                }
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                b.rx = None;
                b.loading = false;
            }
        }
    }

    fn service_script(&self) -> String {
        let mut script = String::new();
        if !self.ros_setup.is_empty() {
            let path = crate::record::expand_home(&self.ros_setup).display().to_string();
            script += &format!("source '{}' || exit 1; ", path.replace('\'', "'\\''"));
        }
        script + "exec timeout 15 ros2 service list -t"
    }

    fn open_browser(&mut self) {
        self.browser = Some(Browser {
            cursor: 0,
            all_types: false,
            filter: String::new(),
            services: Vec::new(),
            loading: true,
            error: String::new(),
            rx: Some(fetch_services(self.service_script())),
        });
    }

    /// Topics (from the node's graph) + services (ros2 service list) as
    /// the browser shows them.
    fn graph_entries(&self, console: &Console) -> Vec<GraphEntry> {
        let Some(b) = &self.browser else { return Vec::new() };
        let topics = lock(console).topics.clone();
        let filter = b.filter.to_lowercase();
        topics
            .into_iter()
            .map(|(name, ty)| GraphEntry { service: false, name, ty })
            .chain(b.services.iter().map(|(name, ty)| GraphEntry { service: true, name: name.clone(), ty: ty.clone() }))
            .filter(|e| b.all_types || is_standard(&e.ty))
            .filter(|e| !e.name.starts_with("/rosout") && !e.name.ends_with("/describe_parameters") && !e.name.contains("/_action/"))
            .filter(|e| filter.is_empty() || e.name.to_lowercase().contains(&filter) || e.ty.to_lowercase().contains(&filter))
            .collect()
    }

    /// One request per distinct resolved name.
    fn requests_for(&self, action: &Action, value: &str, targets: &[Target]) -> Vec<Request> {
        let mut out: Vec<Request> = Vec::new();
        let names: Vec<String> = if targets.is_empty() {
            vec![resolve(&action.path, None)]
        } else {
            targets.iter().map(|t| resolve(&action.path, t.ns.as_deref())).collect()
        };
        for name in names {
            if !out.iter().any(|r| r.name == name) {
                out.push(Request { service: action.service, name, ty: action.ty.clone(), value: value.to_string() });
            }
        }
        out
    }

    /// Queue (or ask first) `action` with `value`.
    fn fire(&mut self, action: &Action, value: &str, console: &Console, targets: &[Target]) {
        // Catch a bad value here rather than on the node thread.
        if !action.ty.is_empty()
            && let Err(e) = to_json(&action.ty, value)
        {
            self.note = Some((false, e));
            return;
        }
        let requests = self.requests_for(action, value, targets);
        let shown = if value.trim().is_empty() { String::new() } else { format!(" {}", value.trim()) };
        let label = format!(
            "{}{shown} -> {}",
            if action.service { "call" } else { "publish" },
            requests.iter().map(|r| r.name.as_str()).collect::<Vec<_>>().join(", ")
        );
        if action.confirm {
            let text = format!("{label}? Type YES");
            self.prompt = Some(Prompt::Confirm { text, buffer: String::new(), requests });
        } else {
            send(console, requests);
        }
    }

    fn run_selected(&mut self, value: Option<&str>, console: &Console, targets: &[Target]) {
        let Some(action) = self.actions.get(self.action_cursor).cloned() else {
            self.note = Some((false, "no action yet - [p] publish, [s] call, [l] pick from the graph, or [[ros.action]] in lazypx4.toml".into()));
            return;
        };
        match value.map(str::to_string).or_else(|| action.value.clone()) {
            Some(v) => self.fire(&action, &v, console, targets),
            None if action.needs_no_value() => self.fire(&action, "", console, targets),
            None => self.prompt = Some(Prompt::Value { action, text: String::new() }),
        }
    }

    /// Add (or re-select) a session action.
    fn add_recent(&mut self, action: Action) {
        match self.actions.iter().position(|a| a.service == action.service && a.path == action.path && a.ty == action.ty && a.value == action.value) {
            Some(i) => self.action_cursor = i,
            None => {
                self.actions.push(action);
                self.action_cursor = self.actions.len() - 1;
            }
        }
    }

    fn submit_line(&mut self, service: bool, text: &str, console: &Console, targets: &[Target]) {
        let (name, mut ty, value) = match parse_line(text, service) {
            Ok(p) => p,
            Err(e) => {
                self.note = Some((false, e));
                return;
            }
        };
        if service && ty.is_empty() {
            match guess_service_type(&value) {
                Ok(t) => ty = t,
                Err(e) => {
                    self.note = Some((false, e));
                    return;
                }
            }
        }
        if !service && ty.is_empty() {
            // Take the type from the graph now, so the action shows it.
            let resolved = resolve(&name, targets.first().and_then(|t| t.ns.as_deref()));
            ty = lock(console).topics.iter().find(|(n, _)| *n == resolved).map(|(_, t)| t.clone()).unwrap_or_default();
        }
        // A bool without value stays reusable with t / f.
        let keep_value = !(value.is_empty() || ((ty == "std_msgs/msg/Bool" || ty == "std_srvs/srv/SetBool") && parse_bool(&value).is_some()));
        let action = Action {
            name: name.clone(),
            service,
            path: name,
            ty,
            value: keep_value.then(|| value.clone()),
            confirm: self.confirm_default,
            recent: true,
        };
        self.add_recent(action.clone());
        if value.is_empty() && !action.needs_no_value() {
            if action.is_bool() {
                self.note = Some((true, format!("added {} - t / f sends true / false", action.path)));
            } else {
                self.prompt = Some(Prompt::Value { action, text: String::new() });
            }
            return;
        }
        self.fire(&action, &value, console, targets);
    }

    /// A pane key (names as `app::key_name` + vim normalisation). False =
    /// not ours (the caller handles back / panels).
    pub fn key(&mut self, key: &str, console: &Console, targets: &[Target]) -> bool {
        self.note = None;
        if let Some(prompt) = self.prompt.as_mut() {
            match prompt {
                Prompt::Filter => {
                    let browsing = self.browser.is_some();
                    let current = match &self.browser {
                        Some(b) => b.filter.clone(),
                        None => self.filter.clone(),
                    };
                    let mut text = current.clone();
                    match key {
                        "ESC" => {
                            text.clear();
                            self.prompt = None;
                        }
                        "ENTER" => self.prompt = None,
                        "BACKSPACE" => {
                            text.pop();
                        }
                        k if crate::app::is_text(k) && text.chars().count() < 160 => text.push_str(k),
                        _ => {}
                    }
                    if text != current {
                        if browsing {
                            if let Some(b) = self.browser.as_mut() {
                                b.filter = text;
                                b.cursor = 0;
                            }
                        } else {
                            self.set_filter(text);
                            self.follow = true;
                        }
                    }
                }
                Prompt::Line { text, .. } | Prompt::Value { text, .. } => match key {
                    "ESC" => self.prompt = None,
                    "BACKSPACE" => {
                        text.pop();
                    }
                    "ENTER" => match self.prompt.take() {
                        Some(Prompt::Line { service, text }) => self.submit_line(service, &text, console, targets),
                        Some(Prompt::Value { action, text }) => self.fire(&action, &text, console, targets),
                        _ => {}
                    },
                    k if crate::app::is_text(k) && text.chars().count() < 512 => text.push_str(k),
                    _ => {}
                },
                Prompt::Confirm { buffer, .. } => match key {
                    "ESC" => self.prompt = None,
                    "BACKSPACE" => {
                        buffer.pop();
                    }
                    "ENTER" => {
                        if buffer.eq_ignore_ascii_case("YES") {
                            if let Some(Prompt::Confirm { requests, .. }) = self.prompt.take() {
                                send(console, requests);
                            }
                        } else {
                            buffer.clear();
                            self.note = Some((false, "type YES to confirm".into()));
                        }
                    }
                    k if crate::app::is_text(k) => {
                        buffer.push_str(&k.to_uppercase());
                        let n = buffer.chars().count();
                        if n > 3 {
                            *buffer = buffer.chars().skip(n - 3).collect();
                        }
                    }
                    _ => {}
                },
            }
            return true;
        }

        if self.browser.is_some() {
            return self.browser_key(key, console, targets);
        }

        let n = self.actions.len();
        let page = self.page.get().max(1);
        let len = self.filtered_len.get();
        match key {
            "[" => self.action_cursor = self.action_cursor.saturating_sub(1),
            "]" => self.action_cursor = (self.action_cursor + 1).min(n.saturating_sub(1)),
            "ENTER" => self.run_selected(None, console, targets),
            "t" | "f" => {
                let is_bool = self.actions.get(self.action_cursor).is_some_and(Action::is_bool);
                if is_bool {
                    self.run_selected(Some(if key == "t" { "true" } else { "false" }), console, targets);
                } else {
                    self.note = Some((false, "t / f: select a Bool topic or SetBool service first ([ / ])".into()));
                }
            }
            "x" => match self.actions.get(self.action_cursor) {
                Some(a) if a.recent => {
                    self.actions.remove(self.action_cursor);
                    self.action_cursor = self.action_cursor.min(self.actions.len().saturating_sub(1));
                }
                Some(_) => self.note = Some((false, "configured in lazypx4.toml ([[ros.action]]) - only session actions are removed".into())),
                None => {}
            },
            "p" => self.prompt = Some(Prompt::Line { service: false, text: String::new() }),
            "s" => self.prompt = Some(Prompt::Line { service: true, text: String::new() }),
            "l" => self.open_browser(),
            "/" => self.prompt = Some(Prompt::Filter),
            "L" => {
                let i = LEVELS.iter().position(|(l, _)| *l == self.min_level).unwrap_or(1);
                self.min_level = LEVELS[(i + 1) % LEVELS.len()].0;
            }
            "v" => self.ns_only = !self.ns_only,
            "c" => {
                lock(console).log.clear();
                self.follow = true;
            }
            "UP" | "PGUP" | "HOME" => {
                let step = if key == "UP" { 1 } else if key == "PGUP" { page } else { usize::MAX };
                let start = if self.follow { len.saturating_sub(page) } else { self.top };
                self.top = start.saturating_sub(step);
                self.follow = false;
            }
            "DOWN" | "PGDN" => {
                if !self.follow {
                    self.top += if key == "DOWN" { 1 } else { page };
                    if self.top + page >= len {
                        self.follow = true;
                    }
                }
            }
            "END" | "G" => self.follow = true,
            _ => return false,
        }
        true
    }

    fn browser_key(&mut self, key: &str, console: &Console, targets: &[Target]) -> bool {
        let entries = self.graph_entries(console);
        let page = self.page.get().max(1);
        let script = self.service_script();
        let Some(b) = self.browser.as_mut() else { return false };
        let n = entries.len();
        match key {
            "l" | "ESC" => self.browser = None,
            "UP" => b.cursor = b.cursor.saturating_sub(1),
            "DOWN" => b.cursor = (b.cursor + 1).min(n.saturating_sub(1)),
            "PGUP" => b.cursor = b.cursor.saturating_sub(page),
            "PGDN" => b.cursor = (b.cursor + page).min(n.saturating_sub(1)),
            "HOME" => b.cursor = 0,
            "END" => b.cursor = n.saturating_sub(1),
            "a" => {
                b.all_types = !b.all_types;
                b.cursor = 0;
            }
            "r" if !b.loading => {
                b.loading = true;
                b.error.clear();
                b.rx = Some(fetch_services(script));
            }
            "/" => self.prompt = Some(Prompt::Filter),
            "ENTER" => {
                let Some(e) = entries.get(b.cursor.min(n.saturating_sub(1))).cloned() else { return true };
                // Under a target's namespace -> relative, so it follows the
                // targets (every marked vehicle in the fleet view).
                let path = targets
                    .iter()
                    .filter_map(|t| t.ns.as_deref())
                    .find_map(|ns| e.name.strip_prefix(&format!("/{ns}/")).map(str::to_string))
                    .unwrap_or_else(|| e.name.clone());
                let action = Action { name: path.clone(), service: e.service, path, ty: e.ty.clone(), value: None, confirm: self.confirm_default, recent: true };
                let hint = if action.is_bool() {
                    "t / f sends true / false"
                } else if action.needs_no_value() {
                    "ENTER sends it"
                } else {
                    "ENTER asks for the value"
                };
                self.note = Some((true, format!("added {} ({}) - {hint}", action.path, short_type(&action.ty))));
                self.add_recent(action);
                self.browser = None;
            }
            _ => return false,
        }
        true
    }

    /// The whole pane, `height` rows.
    pub fn draw(&self, console: &Console, targets: &[Target], width: usize, height: usize) -> Vec<Line<'static>> {
        let c = lock(console);
        let mut lines = Vec::new();

        if !c.available {
            let why = if c.error.is_empty() { "starting the ROS 2 node..." } else { c.error.as_str() };
            lines.push(Ln::new().raw("   ").fg(why.to_string(), if c.error.is_empty() { YELLOW } else { RED }).line());
            lines.push(blank());
        }

        // Targets.
        let who: Vec<String> = targets
            .iter()
            .map(|t| match &t.ns {
                Some(ns) => format!("{} → /{ns}/", t.label),
                None => format!("{} → /", t.label),
            })
            .collect();
        let who = if who.is_empty() { "relative names → /".to_string() } else { format!("relative names: {}", who.join(" · ")) };

        // Actions.
        lines.push(section("ACTIONS", &clip(&format!("[ ] select · ENTER send · t/f · x remove · {who}"), width.saturating_sub(14))));
        if self.actions.is_empty() {
            lines.push(dim_line("none - [p] publish TOPIC [TYPE] VALUE · [s] call SERVICE [TYPE] [ARG] · [l] pick from the graph · or [[ros.action]] in lazypx4.toml"));
        }
        let max_rows = (height / 4).clamp(3, 10);
        let start = crate::ui::window_start(self.action_cursor, self.actions.len(), max_rows);
        let name_w = self.actions.iter().map(|a| a.name.chars().count()).max().unwrap_or(8).clamp(8, 24);
        let path_w = self.actions.iter().map(|a| a.path.chars().count()).max().unwrap_or(8).clamp(8, 36);
        for (i, a) in self.actions.iter().enumerate().skip(start).take(max_rows) {
            let selected = i == self.action_cursor;
            let style = if selected { Style::new().add_modifier(Modifier::BOLD | Modifier::REVERSED) } else { Style::new().add_modifier(Modifier::BOLD) };
            let ty = if a.ty.is_empty() { if a.service { "(guess)".to_string() } else { "(from graph)".to_string() } } else { short_type(&a.ty).to_string() };
            let how = match &a.value {
                Some(v) => format!("= {v}"),
                None if a.is_bool() => "t / f".into(),
                None if a.needs_no_value() => "ENTER".into(),
                None => "ENTER asks".into(),
            };
            let mut l = Ln::new()
                .fg(if selected { " ▸ " } else { "   " }, GREEN)
                .st(format!("{:<name_w$}", clip(&a.name, name_w)), style)
                .raw(" ")
                .fg(format!("{:<5}", a.kind()), if a.service { YELLOW } else { CYAN })
                .raw(format!("{:<path_w$} ", clip(&a.path, path_w)))
                .dim(format!("{ty:<12} "))
                .raw(how);
            if a.confirm {
                l = l.dim("  (YES)");
            }
            if a.recent {
                l = l.dim("  session");
            }
            lines.push(l.line());
        }

        // Last results.
        let mut results: Vec<Line<'static>> = Vec::new();
        if let Some((ok, text)) = &self.note {
            results.push(Ln::new().raw("   ").fg(clip(text, width.saturating_sub(4)), if *ok { GREEN } else { RED }).line());
        }
        // A "calling..." line only while it is the newest.
        let newest = c.feedback.back().map(|f| f.seq);
        let recent: Vec<&Feedback> = c.feedback.iter().rev().filter(|f| f.ok.is_some() || Some(f.seq) == newest).take(2usize.saturating_sub(results.len())).collect();
        for f in recent.into_iter().rev() {
            let (mark, color) = match f.ok {
                Some(true) => ("✓", GREEN),
                Some(false) => ("✗", RED),
                None => ("…", YELLOW),
            };
            let age = now() - f.time;
            let age = if age < 60.0 { format!("{age:.0}s") } else { format!("{:.0}m", age / 60.0) };
            results.push(Ln::new().raw("   ").fg(format!("{mark} "), color).dim(format!("{age:>4} ")).fg(clip(&f.text, width.saturating_sub(12)), color).line());
        }
        lines.extend(results);
        lines.push(blank());

        let prompt_rows = if self.prompt.is_some() { 2 } else { 0 };
        let rows = height.saturating_sub(lines.len() + 1 + prompt_rows).max(1);
        self.page.set(rows);

        if let Some(b) = &self.browser {
            drop(c);
            self.draw_browser(b, console, width, rows, &mut lines);
        } else {
            let namespaces: Vec<String> = targets.iter().filter_map(|t| t.ns.clone()).collect();
            let filtered: Vec<&LogLine> = c.log.iter().filter(|l| self.line_matches(l, &namespaces)).collect();
            self.filtered_len.set(filtered.len());
            let mut note = format!("{}/{} · level ≥ {}", filtered.len(), c.log.len(), level_name(self.min_level));
            if !self.filter_terms.is_empty() {
                let text = self.filter_terms.iter().any(|t| t.re.is_none());
                note += &format!(" · filter \"{}\"{}", self.filter.trim(), if text { " (text)" } else { "" });
            }
            if self.ns_only {
                note += &if namespaces.is_empty() { " · ns: none set".to_string() } else { format!(" · ns {}", namespaces.join(",")) };
            }
            note += if self.follow { " · FOLLOW" } else { " · paused (G follow)" };
            lines.push(section("ROSOUT", &clip(&note, width.saturating_sub(14))));
            let start = if self.follow { filtered.len().saturating_sub(rows) } else { self.top.min(filtered.len().saturating_sub(rows)) };
            let shown: Vec<&&LogLine> = filtered.iter().skip(start).take(rows).collect();
            if shown.is_empty() {
                lines.push(dim_line(if c.log.is_empty() { "no /rosout messages yet" } else { "nothing matches (/ filter · L level · v namespace)" }));
            }
            let name_w = 22.min(width / 4);
            // Highlight only a single literal include word.
            let includes: Vec<&FilterTerm> = self.filter_terms.iter().filter(|t| !t.negate).collect();
            let highlight_word = match includes.as_slice() {
                [t] if t.re.is_none() || !t.text.chars().any(|ch| "\\.^$|?*+()[]{}".contains(ch)) => t.text.as_str(),
                _ => "",
            };
            for l in shown {
                let time = chrono::DateTime::from_timestamp(l.stamp as i64, ((l.stamp.fract()) * 1e9) as u32)
                    .map(|t| t.with_timezone(&chrono::Local).format("%H:%M:%S%.3f").to_string())
                    .unwrap_or_default();
                let msg = l.msg.replace('\n', " ⏎ ");
                let msg = clip(&msg, width.saturating_sub(13 + 6 + name_w + 3));
                let style = level_style(l.level);
                let mut ln = Ln::new()
                    .dim(format!(" {time} "))
                    .st(format!("{:<5} ", level_name(l.level)), style)
                    .fg(format!("{:<name_w$} ", clip(&l.name, name_w)), CYAN);
                ln = if highlight_word.is_empty() { ln.st(msg, style) } else { ln.spans(crate::ui::highlight(&msg, highlight_word, style)) };
                lines.push(ln.line());
            }
        }

        // Prompt at the bottom of the pane.
        if let Some(p) = &self.prompt {
            while lines.len() + 2 < height {
                lines.push(blank());
            }
            let bar = Style::new().bg(crate::ui::BLUE).fg(crate::ui::CRUST).add_modifier(Modifier::BOLD);
            let (title, ask, text) = match p {
                Prompt::Filter => {
                    let t = self.browser.as_ref().map(|b| b.filter.clone()).unwrap_or_else(|| self.filter.clone());
                    (" FILTER  ", "space-separated regex/text on node + message · words = show any · !word = hide · e.g. nav ctrl !debug !heartbeat · ENTER keep · ESC clear".to_string(), t)
                }
                Prompt::Line { service: false, text } => (
                    " PUBLISH ",
                    "TOPIC [TYPE] VALUE   e.g.  cmd/enable bool true · /speed float64 2.5 · /note string hi · /x pkg/msg/T {\"a\":1}".to_string(),
                    text.clone(),
                ),
                Prompt::Line { service: true, text } => (
                    " CALL    ",
                    "SERVICE [TYPE] [ARG]   e.g.  start (Trigger) · arm true (SetBool) · reset empty · /x pkg/srv/T {\"a\":1}".to_string(),
                    text.clone(),
                ),
                Prompt::Value { action, text } => (" VALUE   ", format!("{} {} ({}):", action.kind(), action.path, if action.ty.is_empty() { "type from the graph" } else { &action.ty }), text.clone()),
                Prompt::Confirm { text, buffer, .. } => (" CONFIRM ", text.clone(), buffer.clone()),
            };
            lines.push(Ln::new().st(title, bar).raw("  ").raw(clip(&ask, width.saturating_sub(12))).line());
            lines.push(Ln::new().st(if matches!(p, Prompt::Confirm { .. }) { " YES ?   " } else { "   >     " }, bar).raw(format!("  {text}█")).line());
        }
        lines
    }

    fn draw_browser(&self, b: &Browser, console: &Console, width: usize, rows: usize, lines: &mut Vec<Line<'static>>) {
        let entries = self.graph_entries(console);
        let mut note = format!("{} shown · {}", entries.len(), if b.all_types { "all types" } else { "std_msgs / std_srvs only ([a] all)" });
        if !b.filter.is_empty() {
            note += &format!(" · filter \"{}\"", b.filter);
        }
        if b.loading {
            note += " · ros2 service list...";
        }
        lines.push(section("GRAPH", &clip(&note, width.saturating_sub(12))));
        if !b.error.is_empty() {
            lines.push(Ln::new().raw("   ").fg(clip(&format!("services: {}", b.error), width.saturating_sub(4)), RED).line());
        }
        if entries.is_empty() {
            lines.push(dim_line("nothing (is ROS up? [a] shows every type)"));
            return;
        }
        let rows = rows.saturating_sub(lines.len().min(1)).max(1);
        let cursor = b.cursor.min(entries.len() - 1);
        let start = crate::ui::window_start(cursor, entries.len(), rows);
        let name_w = entries.iter().map(|e| e.name.chars().count()).max().unwrap_or(10).min(width.saturating_sub(40).max(10));
        for (i, e) in entries.iter().enumerate().skip(start).take(rows) {
            let selected = i == cursor;
            let style = if selected { Style::new().add_modifier(Modifier::BOLD | Modifier::REVERSED) } else { Style::new() };
            lines.push(
                Ln::new()
                    .fg(if selected { " ▸ " } else { "   " }, GREEN)
                    .fg(if e.service { "srv   " } else { "topic " }, if e.service { YELLOW } else { CYAN })
                    .st(format!("{:<name_w$}", clip(&e.name, name_w)), style)
                    .raw("  ")
                    .dim(e.ty.clone())
                    .line(),
            );
        }
    }
}

/// Hand requests to the node thread.
pub fn send(console: &Console, requests: Vec<Request>) {
    let mut c = lock(console);
    if !c.available {
        let why = if c.error.is_empty() { "the ROS 2 node is not up yet".to_string() } else { c.error.clone() };
        c.note(Some(false), why);
        return;
    }
    c.requests.extend(requests);
}

fn clip(text: &str, width: usize) -> String {
    if text.chars().count() <= width {
        text.to_string()
    } else {
        let mut s: String = text.chars().take(width.saturating_sub(1)).collect();
        s.push('…');
        s
    }
}

// ---------------------------------------------------------------------------
// The node (ros feature)
// ---------------------------------------------------------------------------

#[cfg(feature = "ros")]
mod imp {
    use std::collections::HashMap;
    use std::rc::Rc;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    use futures::StreamExt;
    use futures::future::{AbortHandle, Abortable};
    use futures::task::LocalSpawnExt;
    use r2r::QosProfile;
    use r2r::rcl_interfaces::msg::Log;

    use super::{CALL_TIMEOUT_S, Console, DISCOVERY_WAIT_S, LogLine, Request, guess_service_type, response_text, to_json};
    use crate::host::lock;
    use crate::state::now;

    pub fn spawn(console: Console, shutdown: Arc<AtomicBool>) -> Option<std::thread::JoinHandle<()>> {
        std::thread::Builder::new()
            .name("RosConsole".into())
            .spawn(move || {
                if let Err(e) = run(&console, &shutdown) {
                    let mut c = lock(&console);
                    c.available = false;
                    c.error = format!("ROS 2 unavailable: {e}");
                }
            })
            .ok()
    }

    /// A one-shot publish waiting for its new publisher to be matched.
    struct Deferred {
        key: (String, String),
        json: serde_json::Value,
        since: f64,
    }

    /// A service call in flight.
    struct Call {
        name: String,
        deadline: f64,
        abort: AbortHandle,
        done: Arc<AtomicBool>,
    }

    fn run(console: &Console, shutdown: &AtomicBool) -> Result<(), String> {
        let ctx = r2r::Context::create().map_err(|e| e.to_string())?;
        // One per process: several lazypx4 on one ROS graph must not clash.
        let mut node = r2r::Node::create(ctx, &format!("lazypx4_console_{}", std::process::id()), "").map_err(|e| e.to_string())?;
        let mut pool = futures::executor::LocalPool::new();
        let spawner = pool.spawner();

        // Volatile + reliable matches every /rosout publisher (rclcpp /
        // rclpy publish it transient-local, which a volatile reader takes).
        let rosout = node.subscribe::<Log>("/rosout", QosProfile::default().keep_last(1000)).map_err(|e| e.to_string())?;
        {
            let console = console.clone();
            spawner
                .spawn_local(rosout.for_each(move |m| {
                    lock(&console).push_log(LogLine {
                        stamp: m.stamp.sec as f64 + m.stamp.nanosec as f64 * 1e-9,
                        level: m.level,
                        name: m.name,
                        msg: m.msg,
                    });
                    futures::future::ready(())
                }))
                .map_err(|e| e.to_string())?;
        }

        {
            let mut c = lock(console);
            c.available = true;
            c.error.clear();
        }

        let mut publishers: HashMap<(String, String), r2r::PublisherUntyped> = HashMap::new();
        let mut clients: HashMap<(String, String), Rc<r2r::ClientUntyped>> = HashMap::new();
        let mut deferred: Vec<Deferred> = Vec::new();
        let mut calls: Vec<Call> = Vec::new();
        let mut next_graph = 0.0;

        while !shutdown.load(Ordering::Relaxed) {
            node.spin_once(Duration::from_millis(50));
            pool.run_until_stalled();
            let t = now();

            if t >= next_graph {
                next_graph = t + 2.0;
                if let Ok(map) = node.get_topic_names_and_types() {
                    let mut topics: Vec<(String, String)> = map.into_iter().map(|(n, ts)| (n, ts.first().cloned().unwrap_or_default())).collect();
                    topics.sort();
                    lock(console).topics = topics;
                }
            }

            let requests = std::mem::take(&mut lock(console).requests);
            for r in requests {
                if r.service {
                    match start_call(&mut node, &spawner, &mut clients, console, &r) {
                        Ok(call) => calls.push(call),
                        Err(e) => lock(console).note(Some(false), format!("call {}: {e}", r.name)),
                    }
                } else if let Err(e) = start_publish(&mut node, &mut publishers, &mut deferred, console, &r) {
                    lock(console).note(Some(false), format!("publish {}: {e}", r.name));
                }
            }

            // Fresh publishers: send once matched, or after the wait.
            deferred.retain(|d| {
                let Some(p) = publishers.get(&d.key) else { return false };
                let subs = p.get_inter_process_subscription_count().unwrap_or(0);
                if subs == 0 && now() - d.since < DISCOVERY_WAIT_S {
                    return true;
                }
                report_publish(console, p, &d.key.0, &d.json);
                false
            });

            calls.retain(|c| {
                if c.done.load(Ordering::Relaxed) {
                    return false;
                }
                if now() > c.deadline {
                    c.abort.abort();
                    lock(console).note(Some(false), format!("call {}: no answer in {CALL_TIMEOUT_S:.0} s (is the service up?)", c.name));
                    return false;
                }
                true
            });
        }
        Ok(())
    }

    fn report_publish(console: &Console, p: &r2r::PublisherUntyped, name: &str, json: &serde_json::Value) {
        let subs = p.get_inter_process_subscription_count().unwrap_or(0);
        let shown = match json.get("data") {
            Some(d) if json.as_object().is_some_and(|o| o.len() == 1) => d.to_string(),
            _ => json.to_string(),
        };
        match p.publish(json.clone()) {
            Ok(()) => {
                let who = match subs {
                    0 => " (no subscriber)".to_string(),
                    1 => " (1 subscriber)".to_string(),
                    n => format!(" ({n} subscribers)"),
                };
                lock(console).note(Some(subs > 0), format!("published {shown} -> {name}{who}"));
            }
            Err(e) => lock(console).note(Some(false), format!("publish {name}: {e}")),
        }
    }

    fn start_publish(
        node: &mut r2r::Node,
        publishers: &mut HashMap<(String, String), r2r::PublisherUntyped>,
        deferred: &mut Vec<Deferred>,
        console: &Console,
        r: &Request,
    ) -> Result<(), String> {
        let ty = if r.ty.is_empty() {
            let found = lock(console).topics.iter().find(|(n, _)| *n == r.name).map(|(_, t)| t.clone());
            match found.or_else(|| node.get_topic_names_and_types().ok()?.get(&r.name)?.first().cloned()) {
                Some(t) => t,
                None => return Err("not in the graph - give the type (p TOPIC TYPE VALUE)".into()),
            }
        } else {
            r.ty.clone()
        };
        let json = to_json(&ty, &r.value)?;
        let key = (r.name.clone(), ty.clone());
        if let Some(p) = publishers.get(&key) {
            report_publish(console, p, &r.name, &json);
            return Ok(());
        }
        let p = node.create_publisher_untyped(&r.name, &ty, QosProfile::default()).map_err(|e| format!("{ty}: {e}"))?;
        publishers.insert(key.clone(), p);
        deferred.push(Deferred { key, json, since: now() });
        Ok(())
    }

    fn start_call(
        node: &mut r2r::Node,
        spawner: &futures::executor::LocalSpawner,
        clients: &mut HashMap<(String, String), Rc<r2r::ClientUntyped>>,
        console: &Console,
        r: &Request,
    ) -> Result<Call, String> {
        let ty = if r.ty.is_empty() { guess_service_type(&r.value)? } else { r.ty.clone() };
        let json = to_json(&ty, &r.value)?;
        let key = (r.name.clone(), ty.clone());
        if !clients.contains_key(&key) {
            let c = node.create_client_untyped(&r.name, &ty, QosProfile::services_default()).map_err(|e| format!("{ty}: {e}"))?;
            clients.insert(key.clone(), Rc::new(c));
        }
        let client = clients[&key].clone();
        let available = r2r::Node::is_available(client.as_ref()).map_err(|e| e.to_string())?;
        let done = Arc::new(AtomicBool::new(false));
        let (abort, reg) = AbortHandle::new_pair();
        let (console2, done2, name, shown) = (console.clone(), done.clone(), r.name.clone(), r.value.trim().to_string());
        lock(console).note(None, format!("calling {name}{}...", if shown.is_empty() { String::new() } else { format!(" {shown}") }));
        // The request goes out once the server is matched (a request sent
        // before that is dropped by the middleware).
        let fut = async move {
            let result = match available.await {
                Ok(()) => match client.request(json) {
                    Ok(f) => f.await,
                    Err(e) => Err(e),
                },
                Err(e) => Err(e),
            };
            let mut c = lock(&console2);
            match result {
                Ok(Ok(resp)) => {
                    let (ok, text) = response_text(&resp);
                    c.note(Some(ok), format!("{name}: {text}"));
                }
                Ok(Err(e)) | Err(e) => c.note(Some(false), format!("call {name}: {e}")),
            }
            done2.store(true, Ordering::Relaxed);
        };
        spawner
            .spawn_local(async move {
                let _ = Abortable::new(fut, reg).await;
            })
            .map_err(|e| e.to_string())?;
        Ok(Call { name: r.name.clone(), deadline: now() + CALL_TIMEOUT_S, abort, done })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_types_expand() {
        assert_eq!(full_type("bool", false), "std_msgs/msg/Bool");
        assert_eq!(full_type("Float64", false), "std_msgs/msg/Float64");
        assert_eq!(full_type("trigger", true), "std_srvs/srv/Trigger");
        assert_eq!(full_type("setbool", true), "std_srvs/srv/SetBool");
        assert_eq!(full_type("std_msgs/Bool", false), "std_msgs/msg/Bool");
        assert_eq!(full_type("geometry_msgs/msg/Twist", false), "geometry_msgs/msg/Twist");
        assert_eq!(full_type("", false), "");
    }

    #[test]
    fn names_resolve_under_the_namespace() {
        assert_eq!(resolve("cmd/enable", Some("uav_1")), "/uav_1/cmd/enable");
        assert_eq!(resolve("/abs", Some("uav_1")), "/abs");
        assert_eq!(resolve("~/x", Some("/uav_2/")), "/uav_2/x");
        assert_eq!(resolve("x", None), "/x");
    }

    #[test]
    fn values_become_json() {
        use serde_json::json;
        assert_eq!(to_json("std_msgs/msg/Bool", "on").unwrap(), json!({"data": true}));
        assert_eq!(to_json("std_srvs/srv/SetBool", "false").unwrap(), json!({"data": false}));
        assert_eq!(to_json("std_srvs/srv/Trigger", "").unwrap(), json!({}));
        assert_eq!(to_json("std_msgs/msg/Int32", "-4").unwrap(), json!({"data": -4}));
        assert_eq!(to_json("std_msgs/msg/Float64", "2.5").unwrap(), json!({"data": 2.5}));
        assert_eq!(to_json("std_msgs/msg/String", "hello world").unwrap(), json!({"data": "hello world"}));
        assert_eq!(to_json("geometry_msgs/msg/Point", "{\"x\": 1}").unwrap(), json!({"x": 1}));
        assert!(to_json("std_msgs/msg/Bool", "maybe").is_err());
        assert!(to_json("std_msgs/msg/Int32", "1.5").is_err());
        assert!(to_json("geometry_msgs/msg/Point", "1").is_err());
    }

    #[test]
    fn lines_split_into_name_type_value() {
        assert_eq!(parse_line("cmd/enable bool true", false).unwrap(), ("cmd/enable".into(), "std_msgs/msg/Bool".into(), "true".into()));
        assert_eq!(parse_line("/speed 2.5", false).unwrap(), ("/speed".into(), String::new(), "2.5".into()));
        assert_eq!(parse_line("/note string hi there", false).unwrap(), ("/note".into(), "std_msgs/msg/String".into(), "hi there".into()));
        assert_eq!(parse_line("arm true", true).unwrap(), ("arm".into(), String::new(), "true".into()));
        assert_eq!(parse_line("start trigger", true).unwrap(), ("start".into(), "std_srvs/srv/Trigger".into(), String::new()));
        assert_eq!(parse_line("/x pkg/msg/T {\"a\": 1}", false).unwrap(), ("/x".into(), "pkg/msg/T".into(), "{\"a\": 1}".into()));
        assert!(parse_line("", true).is_err());
    }

    #[test]
    fn service_types_are_guessed() {
        assert_eq!(guess_service_type("").unwrap(), "std_srvs/srv/Trigger");
        assert_eq!(guess_service_type("on").unwrap(), "std_srvs/srv/SetBool");
        assert!(guess_service_type("3").is_err());
    }

    #[test]
    fn responses_read_well() {
        let (ok, text) = response_text(&serde_json::json!({"success": false, "message": "busy"}));
        assert!(!ok);
        assert_eq!(text, "success=false message=\"busy\"");
        assert_eq!(response_text(&serde_json::json!({})), (true, "ok".to_string()));
    }

    #[test]
    fn requests_dedupe_absolute_names() {
        let (pane, errors) = RosPane::new(&[], true, "");
        assert!(errors.is_empty());
        let targets = vec![Target { label: "a".into(), ns: Some("uav_1".into()) }, Target { label: "b".into(), ns: Some("uav_2".into()) }];
        let rel = Action { name: "e".into(), service: false, path: "enable".into(), ty: "std_msgs/msg/Bool".into(), value: None, confirm: false, recent: false };
        let names: Vec<String> = pane.requests_for(&rel, "true", &targets).into_iter().map(|r| r.name).collect();
        assert_eq!(names, vec!["/uav_1/enable", "/uav_2/enable"]);
        let abs = Action { path: "/enable".into(), ..rel };
        assert_eq!(pane.requests_for(&abs, "true", &targets).len(), 1);
    }

    #[test]
    fn config_actions_need_one_target() {
        let bad = ActionCfg { name: "x".into(), topic: Some("a".into()), service: Some("b".into()), ..Default::default() };
        assert!(Action::from_cfg(&bad, true).is_err());
        let ok = ActionCfg { name: "go".into(), service: Some("start".into()), ty: Some("trigger".into()), confirm: Some(false), ..Default::default() };
        let a = Action::from_cfg(&ok, true).unwrap();
        assert!(a.service && !a.confirm && a.ty == "std_srvs/srv/Trigger");
    }
}

