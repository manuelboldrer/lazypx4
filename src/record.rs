//! [e] RECORD: everything about a flight in one folder, from one pane - a
//! compressed lazyrec. `R` opens a run (`<dir>/<date>_<time>/`) and starts
//! every included recorder, `S` stops them, `F` finishes the run (stop,
//! parameters, manifest.json, summary.md, rename).
//!
//! Recorders:
//! - mavlink tlog: lazypx4's own link, written by `Link` (no extra
//!   mavlink-router endpoint needed), both directions; one per vehicle in
//!   fleet mode, each `mavlink/<namespace>.tlog`
//! - tmux panes: scrollback + live output of every pane (`tmux pipe-pane`)
//! - rosbag: `ros2 bag record -a` minus the excludes, alert when it stalls
//! - system: host CPU / RAM / disk + vehicle state, one CSV row a second
//! - ping: one per `name=host`, alert on a run of lost replies
//! - screen: wf-recorder (wlroots Wayland) or ffmpeg x11grab
//! - `[[record.command]]`: any command, optional alert regex
//!
//! Child processes run in their own process group with a parent-death
//! signal, so a crash still stops them (ros2 bag gets SIGINT and closes the
//! bag cleanly); tmux keeps piping the panes without us.

use std::fs::File;
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use chrono::{DateTime, Local};
use regex::Regex;

/// ros2 bag gets this long after SIGINT to close the bag before SIGTERM.
const ROSBAG_STOP_GRACE: Duration = Duration::from_secs(20);
const CHILD_STOP_GRACE: Duration = Duration::from_secs(5);
/// Consecutive lost pings before an alert.
const PING_ALERT_LOST: u32 = 5;

// ---------------------------------------------------------------------------
// Settings ([record] in lazypx4.toml)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RecordConfig {
    /// The UAV namespace (e.g. "uav_1") in the run folder and file names;
    /// "" = $UAV_NAME / $PX4_UXRCE_DDS_NS, else uav_<system id>.
    pub namespace: String,
    /// Runs go to `<dir>/<date>_<time>[_name]/`.
    pub dir: String,
    /// `<state_dir>/<tmux session>` holds the active run directory, so the
    /// shell recorders (px4-record.sh, rosbag-record.sh, ...) write there too.
    pub state_dir: String,
    pub tlog: bool,
    pub tmux: bool,
    pub tmux_skip_windows: Vec<String>,
    pub tmux_strip_ansi: bool,
    pub rosbag: bool,
    /// Sourced before ros2 (e.g. ~/ros2_ws/install/setup.bash).
    pub ros_setup: String,
    /// Non-empty: record only these topics (the excludes are then ignored).
    pub rosbag_topics: Vec<String>,
    pub rosbag_exclude: Vec<String>,
    /// Exact topic names left out of the bag (the RECORD pane's [t] topic
    /// list toggles these; with `rosbag_topics` they are dropped from it).
    pub rosbag_exclude_topics: Vec<String>,
    pub rosbag_args: Vec<String>,
    /// Alert when the bag has not grown for this long (0 = never).
    pub rosbag_stall_s: u64,
    pub system: bool,
    /// "name=host" (or just "host").
    pub ping: Vec<String>,
    /// Part of `R`; `V` toggles it either way.
    pub screen: bool,
    /// Follow one window by title (regex), "" = whole screen.
    pub screen_window: String,
    pub screen_fps: u32,
    /// Own command instead of wf-recorder / ffmpeg; `$OUT` = file to write.
    pub screen_cmd: String,
    /// Arm / disarm / mode changes / alerts become markers by themselves.
    pub auto_markers: bool,
    /// Labels of the `1`-`9` quick markers.
    pub markers: Vec<String>,
    pub disk_low_gb: f64,
    pub command: Vec<CommandCfg>,
}

impl RecordConfig {
    /// `-a` mode: the patterns plus the excluded topics, as one regex.
    pub fn rosbag_exclude_regex(&self) -> Option<String> {
        let parts: Vec<String> = self
            .rosbag_exclude
            .iter()
            .cloned()
            .chain(self.rosbag_exclude_topics.iter().map(|t| format!("^{}$", regex::escape(t))))
            .collect();
        (!parts.is_empty()).then(|| format!("({})", parts.join("|")))
    }

    /// `rosbag_topics` mode: the listed topics not excluded.
    pub fn rosbag_topics_kept(&self) -> Vec<&String> {
        self.rosbag_topics.iter().filter(|t| !self.rosbag_exclude_topics.contains(t)).collect()
    }

    /// Why `topic` is left out of the bag, if it is: `Some(None)` for an
    /// exact exclude ([t] can undo it), `Some(Some(pattern))` for a
    /// `rosbag_exclude` pattern or a topic not in `rosbag_topics`.
    pub fn rosbag_excluded(&self, topic: &str) -> Option<Option<String>> {
        if self.rosbag_exclude_topics.iter().any(|t| t == topic) {
            return Some(None);
        }
        if !self.rosbag_topics.is_empty() {
            return (!self.rosbag_topics.iter().any(|t| t == topic)).then(|| Some("not in rosbag_topics".into()));
        }
        self.rosbag_exclude
            .iter()
            .find(|p| regex::Regex::new(p).is_ok_and(|re| re.is_match(topic)))
            .map(|p| Some(format!("pattern {p}")))
    }

    /// `ros2 topic list` with `ros_setup` sourced first.
    pub fn topic_list_script(&self) -> String {
        let mut script = String::new();
        if !self.ros_setup.is_empty() {
            script += &format!("source {} || exit 1; ", shell_quote(&expand_home(&self.ros_setup).display().to_string()));
        }
        script + "exec timeout 15 ros2 topic list"
    }
}

impl Default for RecordConfig {
    fn default() -> Self {
        RecordConfig {
            namespace: String::new(),
            dir: "~/logs".into(),
            state_dir: "~/.tmux-recording".into(),
            tlog: true,
            tmux: true,
            tmux_skip_windows: Vec::new(),
            tmux_strip_ansi: false,
            rosbag: true,
            ros_setup: String::new(),
            rosbag_topics: Vec::new(),
            rosbag_exclude: vec![".*image_raw.*".into()],
            rosbag_exclude_topics: Vec::new(),
            rosbag_args: Vec::new(),
            rosbag_stall_s: 30,
            system: true,
            ping: Vec::new(),
            screen: false,
            screen_window: String::new(),
            screen_fps: 5,
            screen_cmd: String::new(),
            auto_markers: true,
            markers: ["takeoff", "mission start", "mission end", "landing", "anomaly", "good", "bad"]
                .map(String::from)
                .to_vec(),
            disk_low_gb: 5.0,
            command: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CommandCfg {
    pub name: String,
    pub cmd: String,
    /// Regex; a matching output line raises an alert.
    pub alert: String,
    /// Part of `R`.
    pub enabled: bool,
}

impl Default for CommandCfg {
    fn default() -> Self {
        CommandCfg { name: String::new(), cmd: String::new(), alert: String::new(), enabled: true }
    }
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

pub fn expand_home(p: &str) -> PathBuf {
    match p.strip_prefix("~/").or_else(|| (p == "~").then_some("")) {
        Some(rest) => PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(rest),
        None => PathBuf::from(p),
    }
}

/// `~/...` for display.
pub fn tilde(p: &Path) -> String {
    let home = std::env::var("HOME").unwrap_or_default();
    match p.strip_prefix(&home) {
        Ok(rest) if !home.is_empty() => format!("~/{}", rest.display()),
        _ => p.display().to_string(),
    }
}

pub fn hms(secs: u64) -> String {
    format!("{:02}:{:02}:{:02}", secs / 3600, secs / 60 % 60, secs % 60)
}

/// What the shell recorders do to names: anything odd becomes `_`.
pub fn sanitize(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() || "-_.".contains(c) { c } else { '_' }).collect()
}

fn shell_quote(s: &str) -> String {
    format!("'{}'", s.replace('\'', r"'\''"))
}

fn which(name: &str) -> bool {
    std::env::var_os("PATH").is_some_and(|paths| std::env::split_paths(&paths).any(|d| d.join(name).is_file()))
}

fn path_size(p: &Path) -> u64 {
    let Ok(meta) = std::fs::symlink_metadata(p) else { return 0 };
    if !meta.is_dir() {
        return meta.len();
    }
    std::fs::read_dir(p).map(|d| d.filter_map(Result::ok).map(|e| path_size(&e.path())).sum()).unwrap_or(0)
}

/// Free bytes on the filesystem holding `path` (or its nearest parent).
pub fn disk_free(path: &Path) -> Option<u64> {
    let mut p = path.to_path_buf();
    while !p.exists() {
        p = p.parent()?.to_path_buf();
    }
    let c_path = std::ffi::CString::new(p.as_os_str().as_encoded_bytes()).ok()?;
    // SAFETY: valid NUL-terminated path and a writable, zeroed statvfs.
    let mut s: libc::statvfs = unsafe { std::mem::zeroed() };
    if unsafe { libc::statvfs(c_path.as_ptr(), &mut s) } != 0 {
        return None;
    }
    Some(s.f_bavail as u64 * s.f_frsize as u64)
}

fn append(path: &Path) -> std::io::Result<File> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::OpenOptions::new().create(true).append(true).open(path)
}

/// `bash -c script` in its own process group; `death` is sent to it if
/// lazypx4 dies first.
fn spawn_bash(script: &str, cwd: &Path, stdout: File, death: libc::c_int) -> std::io::Result<Child> {
    let stderr = stdout.try_clone()?;
    let mut cmd = Command::new("bash");
    cmd.arg("-c").arg(script).current_dir(cwd).stdin(Stdio::null()).stdout(stdout).stderr(stderr);
    // SAFETY: setpgid and prctl are async-signal-safe.
    unsafe {
        cmd.pre_exec(move || {
            libc::setpgid(0, 0);
            libc::prctl(libc::PR_SET_PDEATHSIG, death);
            Ok(())
        });
    }
    cmd.spawn()
}

fn signal_group(pid: u32, sig: libc::c_int) {
    // SAFETY: kill(2) on the group spawn_bash created.
    unsafe {
        libc::kill(-(pid as libc::pid_t), sig);
    }
}

// ---------------------------------------------------------------------------
// tmux
// ---------------------------------------------------------------------------

fn tmux(args: &[&str]) -> Option<String> {
    let out = Command::new("tmux").args(args).stderr(Stdio::null()).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).into_owned())
}

/// The tmux session lazypx4 runs in.
pub fn tmux_session() -> Option<String> {
    std::env::var_os("TMUX").filter(|v| !v.is_empty())?;
    tmux(&["display-message", "-p", "#S"]).map(|s| s.trim().to_string()).filter(|s| !s.is_empty())
}

struct Pane {
    id: String,
    window_index: String,
    window_name: String,
    pane_index: String,
}

fn tmux_panes(session: &str) -> Vec<Pane> {
    let Some(out) = tmux(&[
        "list-panes",
        "-s",
        "-t",
        &format!("={session}"),
        "-F",
        "#{window_index}|#{pane_index}|#{pane_id}|#{window_name}",
    ]) else {
        return Vec::new();
    };
    out.lines()
        .filter_map(|l| {
            let mut it = l.splitn(4, '|');
            Some(Pane {
                window_index: it.next()?.into(),
                pane_index: it.next()?.into(),
                id: it.next()?.into(),
                window_name: it.next()?.into(),
            })
        })
        .collect()
}

/// Header + scrollback into `file`, then every new byte appended live by
/// `tmux pipe-pane` (which keeps going without us).
fn tmux_start_pipe(session: &str, pane: &Pane, file: &Path, strip_ansi: bool) -> Result<(), String> {
    let header = format!(
        "============================================================\n\
         tmux session : {session}\nwindow       : {} ({})\npane         : {} ({})\nstarted      : {}\n\
         ============================================================\n\n",
        pane.window_index,
        pane.window_name,
        pane.pane_index,
        pane.id,
        Local::now().format("%Y-%m-%d %H:%M:%S %Z"),
    );
    if file.exists() {
        let mut f = append(file).map_err(|e| e.to_string())?;
        let _ = write!(f, "\n{header}");
    } else {
        let scrollback = tmux(&["capture-pane", "-t", &pane.id, "-p", "-S", "-"]).unwrap_or_default();
        std::fs::write(file, format!("{header}{scrollback}")).map_err(|e| e.to_string())?;
    }
    let target = shell_quote(&file.display().to_string());
    let pipe = if strip_ansi {
        format!(r"sed -u -E 's/\x1b\[[0-9;?]*[ -\/]*[@-~]//g; s/\x1b\][^\x07]*(\x07|\x1b\\)//g; s/\r//g' >> {target}")
    } else {
        format!("cat >> {target}")
    };
    tmux(&["pipe-pane", "-t", &pane.id]);
    tmux(&["pipe-pane", "-t", &pane.id, &pipe]).map(|_| ()).ok_or_else(|| format!("tmux pipe-pane failed for {}", pane.id))
}

// ---------------------------------------------------------------------------
// Recorders
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub enum Kind {
    /// Index into the vehicles handed to start / stop / poll.
    Tlog(usize),
    Tmux,
    Rosbag,
    System,
    Ping(String),
    Screen,
    Command(CommandCfg),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    Idle,
    Recording,
    Stopping,
    Done,
    Failed(String),
    Unavailable(String),
}

impl Status {
    pub fn label(&self) -> &'static str {
        match self {
            Status::Idle => "IDLE",
            Status::Recording => "REC",
            Status::Stopping => "STOPPING",
            Status::Done => "DONE",
            Status::Failed(_) => "FAILED",
            Status::Unavailable(_) => "N/A",
        }
    }
}

/// What the app should tell the user (event log; `alert` = banner too).
pub enum Note {
    Info(String),
    Warn(String),
    Alert(String),
}

/// One vehicle whose link can be recorded, and its namespace (`uav_1`)
/// for file names.
pub struct Veh<'a> {
    /// The recorder index it belongs to (fleet slot; 0 in the full UI).
    pub id: usize,
    pub link: &'a crate::link::Link,
    pub ns: String,
}

/// The namespace from the environment the tmux sessions set up
/// (`UAV_NAME`, `PX4_UXRCE_DDS_NS`), if any.
pub fn env_namespace() -> Option<String> {
    ["UAV_NAME", "UAV_NAMESPACE", "PX4_UXRCE_DDS_NS"]
        .iter()
        .find_map(|k| std::env::var(k).ok())
        .map(|v| v.trim().trim_matches('/').to_string())
        .filter(|v| !v.is_empty())
}

/// One vehicle's state, for the auto markers and the run's alert list.
pub struct VehState {
    pub ns: String,
    pub armed: bool,
    pub mode: String,
    /// Unacknowledged alerts (count) and the newest one.
    pub alerts: usize,
    pub newest_alert: Option<String>,
}

/// What a poll needs from the rest of lazypx4.
pub struct PollCtx<'a> {
    pub vehicles: &'a [Veh<'a>],
    /// One CSV row for the system recorder (header, row).
    pub system_row: Option<(&'static str, String)>,
}

pub struct Recorder {
    pub kind: Kind,
    pub name: String,
    /// Part of `R`.
    pub include: bool,
    pub status: Status,
    pub started: Option<Instant>,
    pub ran_for: Option<Duration>,
    pub output: Option<PathBuf>,
    pub bytes: u64,
    /// Bytes per second, smoothed.
    pub rate: f64,
    pub detail: String,
    panes: Vec<(String, PathBuf)>,
    topics: Vec<String>,
    log: Option<PathBuf>,
    log_offset: u64,
    alert_re: Option<Regex>,
    child: Option<Child>,
    writer: Option<BufWriter<File>>,
    rows: u64,
    stop_deadline: Option<Instant>,
    term_sent: bool,
    last_sample: Option<(Instant, u64)>,
    last_growth: Option<Instant>,
    stall_warned: bool,
    ping_lost_run: u32,
    ping_lost: u64,
    ping_ok: u64,
}

impl Recorder {
    fn new(kind: Kind, name: impl Into<String>, include: bool) -> Self {
        Recorder {
            kind,
            name: name.into(),
            include,
            status: Status::Idle,
            started: None,
            ran_for: None,
            output: None,
            bytes: 0,
            rate: 0.0,
            detail: String::new(),
            panes: Vec::new(),
            topics: Vec::new(),
            log: None,
            log_offset: 0,
            alert_re: None,
            child: None,
            writer: None,
            rows: 0,
            stop_deadline: None,
            term_sent: false,
            last_sample: None,
            last_growth: None,
            stall_warned: false,
            ping_lost_run: 0,
            ping_lost: 0,
            ping_ok: 0,
        }
    }

    pub fn active(&self) -> bool {
        matches!(self.status, Status::Recording | Status::Stopping)
    }

    pub fn available(&self) -> bool {
        !matches!(self.status, Status::Unavailable(_))
    }

    /// Still recording something quitting would cut short (tmux keeps
    /// piping the panes without us).
    pub fn owns_data(&self) -> bool {
        self.active() && !matches!(self.kind, Kind::Tmux)
    }

    fn unavailable(mut self, why: impl Into<String>) -> Self {
        self.status = Status::Unavailable(why.into());
        self
    }

    pub fn idle_detail(&self, cfg: &RecordConfig) -> String {
        match &self.kind {
            Kind::Tlog(_) => "mavlink/<namespace>.tlog from this link, both directions".into(),
            Kind::Tmux => "every pane: scrollback + live output".into(),
            Kind::Rosbag if !cfg.rosbag_topics.is_empty() => format!("{} topics · [t] topics", cfg.rosbag_topics_kept().len()),
            Kind::Rosbag => {
                let mut ex = cfg.rosbag_exclude.join(" ");
                if !cfg.rosbag_exclude_topics.is_empty() {
                    ex = format!("{ex} + {} topic(s)", cfg.rosbag_exclude_topics.len()).trim().to_string();
                }
                if ex.is_empty() { "-a · [t] topics".into() } else { format!("-a, excluding {ex} · [t] topics") }
            }
            Kind::System => "system.csv: host + vehicle, 1 Hz".into(),
            Kind::Ping(host) => format!("net/ping_{}.log  {host}", sanitize(&self.name["ping ".len()..])),
            Kind::Screen if !cfg.screen_cmd.is_empty() => "custom command".into(),
            Kind::Screen if !cfg.screen_window.is_empty() => format!("window /{}/, {} fps", cfg.screen_window, cfg.screen_fps),
            Kind::Screen => format!("whole screen, {} fps", cfg.screen_fps),
            Kind::Command(c) => c.cmd.clone(),
        }
    }

    // -----------------------------------------------------------------
    // start / stop
    // -----------------------------------------------------------------

    fn start(&mut self, cfg: &RecordConfig, run: &Run, tmux_session: Option<&str>, vehicles: &[Veh], out: &mut Vec<Note>) {
        if self.active() || !self.available() {
            return;
        }
        self.bytes = 0;
        self.rate = 0.0;
        self.rows = 0;
        self.detail.clear();
        self.ran_for = None;
        self.last_sample = None;
        self.last_growth = Some(Instant::now());
        self.stall_warned = false;
        self.term_sent = false;
        self.stop_deadline = None;
        self.ping_lost_run = 0;
        self.ping_lost = 0;
        self.ping_ok = 0;
        let dir = &run.dir;
        let result: Result<(), String> = match self.kind.clone() {
            Kind::Tlog(i) => match vehicles.iter().find(|v| v.id == i) {
                Some(v) => {
                    let path = dir.join(format!("mavlink/{}.tlog", sanitize(&v.ns)));
                    self.name = format!("tlog {}", v.ns);
                    if v.link.start_run_tlog(path.clone()) {
                        self.output = Some(path);
                        Ok(())
                    } else {
                        Err("not on a live link".into())
                    }
                }
                None => Err("vehicle not connected".into()),
            },
            Kind::Tmux => self.start_tmux(cfg, dir, tmux_session, out),
            Kind::Rosbag => self.start_rosbag(cfg, dir),
            Kind::System => {
                let path = dir.join(match &run.namespace {
                    Some(ns) => format!("system_{}.csv", sanitize(ns)),
                    None => "system.csv".into(),
                });
                let existed = path.exists();
                append(&path).map_err(|e| e.to_string()).map(|f| {
                    self.writer = Some(BufWriter::new(f));
                    self.rows = u64::from(existed);
                    self.output = Some(path);
                })
            }
            Kind::Ping(host) => {
                let file = dir.join(format!("net/ping_{}.log", sanitize(&self.name["ping ".len()..])));
                // -D: unix timestamps, -O: report each missing reply.
                self.start_child(&format!("exec ping -D -O -i 1 {}", shell_quote(&host)), &file, libc::SIGINT)
            }
            Kind::Screen => {
                let file = dir.join("screen/screen.log");
                let r = self.start_child(&screen_script(cfg), &file, libc::SIGINT);
                self.output = Some(dir.join("screen"));
                r
            }
            Kind::Command(c) => {
                let file = dir.join(format!("{}.log", sanitize(&c.name)));
                self.start_child(&c.cmd, &file, libc::SIGTERM)
            }
        };
        match result {
            Ok(()) => {
                self.status = Status::Recording;
                self.started = Some(Instant::now());
                let to = self.output.as_deref().map(tilde).unwrap_or_default();
                out.push(Note::Info(format!("REC {} -> {to}", self.name)));
            }
            Err(e) => {
                self.status = Status::Failed(e.clone());
                out.push(Note::Warn(format!("REC {}: {e}", self.name)));
            }
        }
    }

    fn start_child(&mut self, script: &str, file: &Path, death: libc::c_int) -> Result<(), String> {
        let cwd = file.parent().unwrap_or(Path::new(".")).to_path_buf();
        let log = append(file).map_err(|e| e.to_string())?;
        self.log_offset = std::fs::metadata(file).map(|m| m.len()).unwrap_or(0);
        self.child = Some(spawn_bash(script, &cwd, log, death).map_err(|e| format!("bash: {e}"))?);
        self.log = Some(file.to_path_buf());
        self.output = Some(file.to_path_buf());
        Ok(())
    }

    fn start_tmux(&mut self, cfg: &RecordConfig, dir: &Path, session: Option<&str>, out: &mut Vec<Note>) -> Result<(), String> {
        let session = session.ok_or("not inside tmux")?;
        self.panes.clear();
        self.output = Some(dir.join("tmux"));
        self.follow_panes(cfg, dir, session, out, false);
        if self.panes.is_empty() {
            return Err("no panes to record".into());
        }
        Ok(())
    }

    /// Pipe every pane not recorded yet; returns how many are still open.
    fn follow_panes(&mut self, cfg: &RecordConfig, dir: &Path, session: &str, out: &mut Vec<Note>, announce: bool) -> usize {
        // Our own pane is only TUI redraws.
        let own = std::env::var("TMUX_PANE").unwrap_or_default();
        let panes = tmux_panes(session);
        for pane in &panes {
            if pane.id == own || cfg.tmux_skip_windows.contains(&pane.window_name) || self.panes.iter().any(|(id, _)| *id == pane.id) {
                continue;
            }
            let file = dir.join("tmux").join(format!(
                "window-{}_{}_pane-{}.txt",
                pane.window_index,
                sanitize(&pane.window_name),
                pane.pane_index
            ));
            let _ = std::fs::create_dir_all(dir.join("tmux"));
            match tmux_start_pipe(session, pane, &file, cfg.tmux_strip_ansi) {
                Ok(()) => {
                    if announce {
                        out.push(Note::Info(format!("REC new tmux pane {} ({}) recorded", pane.id, pane.window_name)));
                    }
                    self.panes.push((pane.id.clone(), file));
                }
                Err(e) => out.push(Note::Warn(format!("REC pane {}: {e}", pane.id))),
            }
        }
        self.panes.iter().filter(|(id, _)| panes.iter().any(|p| p.id == *id)).count()
    }

    fn start_rosbag(&mut self, cfg: &RecordConfig, dir: &Path) -> Result<(), String> {
        let dir = dir.join("rosbag");
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        let mut script = String::new();
        if !cfg.ros_setup.is_empty() {
            script += &format!("source {} || exit 1; ", shell_quote(&expand_home(&cfg.ros_setup).display().to_string()));
        }
        let mut record = String::from("ros2 bag record");
        if cfg.rosbag_topics.is_empty() {
            record += " -a";
            if let Some(re) = cfg.rosbag_exclude_regex() {
                // Humble: --exclude, Jazzy+: --exclude-regex.
                script += "X=--exclude; ros2 bag record --help 2>/dev/null | grep -q -- --exclude-regex && X=--exclude-regex; ";
                record += &format!(" \"$X\" {}", shell_quote(&re));
            }
        } else {
            let kept = cfg.rosbag_topics_kept();
            if kept.is_empty() {
                return Err("every topic in [record] rosbag_topics is excluded ([t])".into());
            }
            for t in kept {
                record += &format!(" {}", shell_quote(t));
            }
        }
        for a in &cfg.rosbag_args {
            record += &format!(" {}", shell_quote(a));
        }
        // ros2 bag wants to create its own (new) output folder.
        record += " -o bag";
        script += &format!("echo \"+ {}\"; exec {record}", record.replace('"', "\\\""));
        let _ = std::fs::write(dir.join("command.sh"), format!("#!/bin/bash\ncd {}\n{script}\n", shell_quote(&dir.display().to_string())));
        let r = self.start_child(&script, &dir.join("record.log"), libc::SIGINT);
        self.output = Some(dir);
        self.topics.clear();
        r
    }

    fn stop(&mut self, vehicles: &[Veh], out: &mut Vec<Note>) {
        if self.status != Status::Recording {
            return;
        }
        match &self.kind {
            Kind::Tlog(i) => {
                if let Some(v) = vehicles.iter().find(|v| v.id == *i) {
                    v.link.stop_run_tlog();
                }
                self.finish(Status::Done);
            }
            Kind::Tmux => {
                for (pane, _) in &self.panes {
                    tmux(&["pipe-pane", "-t", pane]);
                }
                self.finish(Status::Done);
            }
            Kind::System => {
                if let Some(mut w) = self.writer.take() {
                    let _ = w.flush();
                }
                self.finish(Status::Done);
            }
            Kind::Rosbag | Kind::Ping(_) | Kind::Screen | Kind::Command(_) => match &self.child {
                Some(child) => {
                    signal_group(child.id(), libc::SIGINT);
                    let grace = if matches!(self.kind, Kind::Rosbag) { ROSBAG_STOP_GRACE } else { CHILD_STOP_GRACE };
                    self.stop_deadline = Some(Instant::now() + grace);
                    self.status = Status::Stopping;
                    out.push(Note::Info(format!("REC {} stopping...", self.name)));
                }
                None => self.finish(Status::Done),
            },
        }
    }

    fn finish(&mut self, status: Status) {
        self.ran_for = self.started.map(|s| s.elapsed());
        self.status = status;
        self.child = None;
        self.stop_deadline = None;
        self.rate = 0.0;
        self.refresh_size();
    }

    // -----------------------------------------------------------------
    // polling
    // -----------------------------------------------------------------

    fn refresh_size(&mut self) {
        self.bytes = match self.kind {
            Kind::Tmux => self.panes.iter().map(|(_, f)| path_size(f)).sum(),
            _ => self.output.as_deref().map(path_size).unwrap_or(0),
        };
    }

    /// Paths moved under us by a rename of the run folder.
    fn rebase(&mut self, old: &Path, new: &Path) {
        let fix = |p: &mut PathBuf| {
            if let Ok(rest) = p.strip_prefix(old) {
                *p = new.join(rest);
            }
        };
        self.output.as_mut().map(fix);
        self.log.as_mut().map(fix);
        for (_, f) in &mut self.panes {
            fix(f);
        }
    }

    fn poll(&mut self, cfg: &RecordConfig, run: &Run, tmux_session: Option<&str>, ctx: &PollCtx, out: &mut Vec<Note>) {
        if !self.active() {
            return;
        }
        if let (Kind::System, Some((header, row)), Some(w)) = (&self.kind, &ctx.system_row, self.writer.as_mut()) {
            if self.rows == 0 {
                let _ = writeln!(w, "{header}");
            }
            let _ = writeln!(w, "{row}");
            let _ = w.flush();
            self.rows += 1;
        }
        self.refresh_size();
        let now = Instant::now();
        if let Some((t, b)) = self.last_sample {
            let dt = now.duration_since(t).as_secs_f64();
            if dt > 0.0 {
                self.rate = 0.7 * self.rate + 0.3 * (self.bytes.saturating_sub(b) as f64 / dt);
            }
            if self.bytes > b {
                self.last_growth = Some(now);
                self.stall_warned = false;
            }
        }
        self.last_sample = Some((now, self.bytes));

        match self.kind.clone() {
            Kind::Tlog(i) => {
                // The namespace became known after the start (the vehicle
                // connected later): rename the file to match.
                if let (Some(v), Some(old)) = (ctx.vehicles.iter().find(|v| v.id == i), self.output.clone())
                    && self.name != format!("tlog {}", v.ns)
                {
                    let new = old.with_file_name(format!("{}.tlog", sanitize(&v.ns)));
                    if !new.exists() {
                        let moved = if old.exists() { std::fs::rename(&old, &new).is_ok() } else { v.link.start_run_tlog(new.clone()) };
                        if moved {
                            out.push(Note::Info(format!("REC {} -> {}", self.name, tilde(&new))));
                            self.name = format!("tlog {}", v.ns);
                            self.output = Some(new);
                        }
                    }
                }
                self.detail = match ctx.vehicles.iter().find(|v| v.id == i).and_then(|v| v.link.run_tlog_frames()) {
                    Some(0) | None => "waiting for MAVLink traffic".into(),
                    Some(n) => format!("{n} frames"),
                };
            }
            Kind::Tmux => {
                if let Some(session) = tmux_session {
                    let open = self.follow_panes(cfg, &run.dir, session, out, true);
                    self.detail = if open == self.panes.len() {
                        format!("{} panes", self.panes.len())
                    } else {
                        format!("{open}/{} panes open", self.panes.len())
                    };
                }
            }
            Kind::Rosbag => {
                self.read_bag_log();
                self.detail = format!("{} topics", self.topics.len());
                self.poll_child(out);
                let stall = cfg.rosbag_stall_s;
                if self.status == Status::Recording
                    && stall > 0
                    && !self.stall_warned
                    && self.last_growth.is_some_and(|t| t.elapsed() > Duration::from_secs(stall))
                {
                    self.stall_warned = true;
                    out.push(Note::Alert(format!("REC: rosbag has not grown for {stall}s - check ROS / disk")));
                }
            }
            Kind::System => self.detail = format!("{} rows", self.rows.saturating_sub(1)),
            Kind::Ping(host) => {
                self.read_ping(&host, out);
                self.poll_child(out);
            }
            Kind::Screen => {
                let files = self
                    .output
                    .as_deref()
                    .and_then(|d| std::fs::read_dir(d).ok())
                    .map(|d| d.filter_map(Result::ok).filter(|e| e.path().extension().is_some_and(|x| x == "mkv")).count())
                    .unwrap_or(0);
                self.detail = format!("{files} video file{}", if files == 1 { "" } else { "s" });
                self.poll_child(out);
            }
            Kind::Command(c) => {
                self.detail = c.cmd.clone();
                if self.alert_re.is_some() {
                    self.scan_alerts(out);
                }
                self.poll_child(out);
            }
        }
    }

    /// Whole new lines of the child's log since the last call.
    fn new_lines(&mut self) -> Vec<String> {
        let Some(path) = self.log.clone() else { return Vec::new() };
        let Ok(mut f) = File::open(&path) else { return Vec::new() };
        if f.seek(SeekFrom::Start(self.log_offset)).is_err() {
            return Vec::new();
        }
        let mut buf = Vec::new();
        let _ = f.take(1 << 20).read_to_end(&mut buf);
        let Some(end) = buf.iter().rposition(|b| *b == b'\n') else { return Vec::new() };
        self.log_offset += end as u64 + 1;
        String::from_utf8_lossy(&buf[..end]).lines().map(str::to_string).collect()
    }

    fn scan_alerts(&mut self, out: &mut Vec<Note>) {
        for line in self.new_lines() {
            if self.alert_re.as_ref().is_some_and(|re| re.is_match(&line)) {
                let line: String = line.trim().chars().take(160).collect();
                out.push(Note::Alert(format!("REC: {}: {line}", self.name)));
            }
        }
    }

    pub fn is_rosbag(&self) -> bool {
        matches!(self.kind, Kind::Rosbag)
    }

    /// The topics a running / finished rosbag subscribed to.
    pub fn bag_topics(&self) -> &[String] {
        &self.topics
    }

    fn read_bag_log(&mut self) {
        for line in self.new_lines() {
            if let Some(rest) = line.split("Subscribed to topic").nth(1)
                && let Some(topic) = rest.split('\'').nth(1)
                && !self.topics.iter().any(|t| t == topic)
            {
                self.topics.push(topic.to_string());
            }
        }
    }

    fn read_ping(&mut self, host: &str, out: &mut Vec<Note>) {
        let mut last_rtt = None;
        for line in self.new_lines() {
            if let Some(ms) = line.split("time=").nth(1).and_then(|r| r.split_whitespace().next()).and_then(|v| v.parse::<f64>().ok()) {
                last_rtt = Some(ms);
                self.ping_ok += 1;
                if self.ping_lost_run >= PING_ALERT_LOST {
                    out.push(Note::Info(format!("REC {}: {host} answering again", self.name)));
                }
                self.ping_lost_run = 0;
            } else if line.contains("no answer yet") || line.contains("Unreachable") {
                self.ping_lost += 1;
                self.ping_lost_run += 1;
                if self.ping_lost_run == PING_ALERT_LOST {
                    out.push(Note::Alert(format!("REC: {host} ({}) not answering pings", &self.name["ping ".len()..])));
                }
            }
        }
        let total = self.ping_ok + self.ping_lost;
        let loss = if total > 0 { format!(", {:.0}% lost", self.ping_lost as f64 * 100.0 / total as f64) } else { String::new() };
        if let Some(ms) = last_rtt {
            self.detail = format!("{ms:.1} ms{loss}");
        } else if self.ping_lost_run > 0 {
            self.detail = format!("no reply x{}{loss}", self.ping_lost_run);
        } else if total == 0 {
            self.detail = format!("pinging {host}");
        }
    }

    fn poll_child(&mut self, out: &mut Vec<Note>) {
        let Some(child) = self.child.as_mut() else { return };
        match child.try_wait() {
            Ok(Some(exit)) => {
                if self.status == Status::Stopping || exit.code() == Some(0) {
                    out.push(Note::Info(format!("REC {} stopped", self.name)));
                    self.finish(Status::Done);
                } else {
                    let why = match exit.code() {
                        Some(c) => format!("exited with code {c}"),
                        None => "killed by a signal".into(),
                    };
                    let tail = self
                        .log
                        .as_deref()
                        .and_then(|l| std::fs::read_to_string(l).ok())
                        .and_then(|t| t.lines().rev().find(|l| !l.trim().is_empty()).map(|l| l.chars().take(120).collect::<String>()))
                        .map(|l| format!(": {l}"))
                        .unwrap_or_default();
                    out.push(Note::Alert(format!("REC: {} {why}{tail}", self.name)));
                    self.finish(Status::Failed(why));
                }
            }
            Ok(None) => {
                if let Some(deadline) = self.stop_deadline
                    && Instant::now() > deadline
                {
                    let pid = child.id();
                    if !self.term_sent {
                        signal_group(pid, libc::SIGTERM);
                        self.term_sent = true;
                        self.stop_deadline = Some(Instant::now() + CHILD_STOP_GRACE);
                        out.push(Note::Warn(format!("REC {} ignored Ctrl-C, sent SIGTERM", self.name)));
                    } else {
                        signal_group(pid, libc::SIGKILL);
                        self.stop_deadline = None;
                        out.push(Note::Warn(format!("REC {} killed", self.name)));
                    }
                }
            }
            Err(_) => {}
        }
    }

    fn manifest(&self) -> serde_json::Value {
        let why = match &self.status {
            Status::Failed(w) | Status::Unavailable(w) => Some(w.clone()),
            _ => None,
        };
        serde_json::json!({
            "name": self.name,
            "status": self.status.label(),
            "reason": why,
            "output": self.output.as_ref().map(|p| p.display().to_string()),
            "bytes": self.bytes,
            "seconds": self.ran_for.or(self.started.map(|s| s.elapsed())).map(|d| d.as_secs()),
            "detail": self.detail,
            "topics": matches!(self.kind, Kind::Rosbag).then_some(&self.topics),
        })
    }
}

/// wf-recorder on wlroots Wayland, ffmpeg x11grab on X11; one
/// `screen_<time>.mkv` per start (mkv stays readable when cut off).
fn screen_script(cfg: &RecordConfig) -> String {
    let fps = cfg.screen_fps.max(1);
    let filter = "scale=w='min(1920,iw)':h=-2,pad=ceil(iw/2)*2:ceil(ih/2)*2";
    let mut s = String::from("OUT=\"screen_$(date +%H-%M-%S).mkv\"; ");
    if !cfg.screen_cmd.is_empty() {
        s += &cfg.screen_cmd;
    } else if std::env::var_os("WAYLAND_DISPLAY").is_some() {
        s += "G=; ";
        if !cfg.screen_window.is_empty() {
            s += &format!(
                "G=$(swaymsg -t get_tree 2>/dev/null | jq -r --arg re {} 'first(.. | objects | select(.pid? and .name? and (.name | test($re))) | .rect | \"\\(.x),\\(.y) \\(.width)x\\(.height)\")' 2>/dev/null); \
                 [ -n \"$G\" ] || echo 'window not found - recording the whole screen'; ",
                shell_quote(&cfg.screen_window)
            );
        }
        s += &format!(
            "exec wf-recorder -f \"$OUT\" -r {fps} -c libx264 -p preset=ultrafast -p crf=30 -x yuv420p -F {} ${{G:+-g \"$G\"}} </dev/null",
            shell_quote(filter)
        );
    } else {
        s += "D=${DISPLAY:-:0}; POS=; SIZE=; ";
        if !cfg.screen_window.is_empty() {
            s += &format!(
                "eval \"$(xdotool search --onlyvisible --name {} getwindowgeometry --shell 2>/dev/null | head -n 6)\"; \
                 if [ -n \"$WIDTH\" ]; then POS=\"+$X,$Y\"; SIZE=\"${{WIDTH}}x${{HEIGHT}}\"; \
                 else echo 'window not found - recording the whole screen'; fi; ",
                shell_quote(&cfg.screen_window)
            );
        }
        s += &format!(
            "exec ffmpeg -nostdin -hide_banner -loglevel warning -f x11grab -framerate {fps} ${{SIZE:+-video_size $SIZE}} -i \"$D$POS\" \
             -vf {} -c:v libx264 -preset ultrafast -crf 30 -pix_fmt yuv420p \"$OUT\"",
            shell_quote(filter)
        );
    }
    s
}

// ---------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------

pub struct Marker {
    pub wall: DateTime<Local>,
    pub elapsed_s: f64,
    pub label: String,
    pub text: String,
}

pub struct Run {
    pub dir: PathBuf,
    pub started_wall: DateTime<Local>,
    pub started: Instant,
    pub finished_wall: Option<DateTime<Local>>,
    pub markers: Vec<Marker>,
    pub alerts: Vec<(String, String)>,
    /// The UAV namespace in the folder / file names (`uav_1`, `fleet`).
    pub namespace: Option<String>,
    /// The Flight Review link of the upload, once done.
    pub review_url: Option<String>,
}

impl Run {
    fn create(base: &Path, namespace: Option<&str>) -> std::io::Result<Run> {
        let now = Local::now();
        let mut stamp = now.format("%Y-%m-%d_%H-%M-%S").to_string();
        if let Some(ns) = namespace {
            stamp = format!("{stamp}_{}", sanitize(ns));
        }
        let mut dir = base.join(&stamp);
        let mut n = 2;
        while dir.exists() {
            dir = base.join(format!("{stamp}_{n}"));
            n += 1;
        }
        std::fs::create_dir_all(&dir)?;
        Ok(Run {
            dir,
            started_wall: now,
            started: Instant::now(),
            finished_wall: None,
            markers: Vec::new(),
            alerts: Vec::new(),
            namespace: namespace.map(str::to_string),
            review_url: None,
        })
    }

    pub fn name(&self) -> String {
        self.dir.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
    }

    pub fn elapsed_s(&self) -> u64 {
        match self.finished_wall {
            Some(end) => (end - self.started_wall).num_seconds().max(0) as u64,
            None => self.started.elapsed().as_secs(),
        }
    }

    /// markers.csv (for scripts) and notes.md (for people).
    fn add_marker(&mut self, label: &str, text: &str) -> std::io::Result<&Marker> {
        let m = Marker {
            wall: Local::now(),
            elapsed_s: self.started.elapsed().as_secs_f64(),
            label: label.into(),
            text: text.into(),
        };
        let csv_path = self.dir.join("markers.csv");
        let new = !csv_path.exists();
        let mut csv = append(&csv_path)?;
        if new {
            writeln!(csv, "wall_time,unix_time_s,elapsed_s,label,text")?;
        }
        let quote = |s: &str| format!("\"{}\"", s.replace('"', "\"\""));
        writeln!(
            csv,
            "{},{:.3},{:.3},{},{}",
            m.wall.format("%Y-%m-%d %H:%M:%S%.3f"),
            m.wall.timestamp_millis() as f64 / 1000.0,
            m.elapsed_s,
            quote(label),
            quote(text)
        )?;
        let notes_path = self.dir.join("notes.md");
        let new = !notes_path.exists();
        let mut notes = append(&notes_path)?;
        if new {
            writeln!(notes, "# {}\n", self.name())?;
        }
        let what = if text.is_empty() { format!("**{label}**") } else { format!("**{label}** {text}") };
        writeln!(notes, "- `{}` (+{}) {what}", m.wall.format("%H:%M:%S%.3f"), hms(m.elapsed_s as u64))?;
        self.markers.push(m);
        Ok(self.markers.last().unwrap())
    }
}

// ---------------------------------------------------------------------------
// The whole pane's state
// ---------------------------------------------------------------------------

pub struct Recording {
    pub cfg: RecordConfig,
    pub recorders: Vec<Recorder>,
    pub run: Option<Run>,
    pub cursor: usize,
    pub tmux_session: Option<String>,
    pub disk_free: Option<u64>,
    /// Put into the next run's folder name / file names; set by the caller
    /// before `record_all` (it knows the vehicle).
    pub namespace: Option<String>,
    disk_warned: bool,
    last_poll: Instant,
    /// Vehicle state the auto markers compare against.
    last: std::collections::HashMap<String, (bool, String, usize)>,
}

impl Recording {
    /// `tlogs`: one label per vehicle link that can be recorded.
    pub fn new(cfg: RecordConfig, tlogs: &[String], replay: bool) -> Self {
        let tmux_session = tmux_session();
        let ros = cfg.ros_setup.is_empty() && !which("ros2");
        let mut v = Vec::new();
        for (i, label) in tlogs.iter().enumerate() {
            let tlog = Recorder::new(Kind::Tlog(i), format!("tlog {label}"), cfg.tlog);
            v.push(if replay { tlog.unavailable("replaying a .tlog") } else { tlog });
        }
        let tm = Recorder::new(Kind::Tmux, "tmux panes", cfg.tmux);
        v.push(if tmux_session.is_none() { tm.unavailable("not running inside tmux") } else { tm });
        let bag = Recorder::new(Kind::Rosbag, "rosbag", cfg.rosbag);
        v.push(if ros { bag.unavailable("ros2 not on PATH (source ROS or set [record] ros_setup)") } else { bag });
        v.push(Recorder::new(Kind::System, "system", cfg.system));
        for spec in &cfg.ping {
            let (name, host) = match spec.split_once('=') {
                Some((n, h)) => (n.trim().to_string(), h.trim().to_string()),
                None => (spec.trim().to_string(), spec.trim().to_string()),
            };
            let r = Recorder::new(Kind::Ping(host), format!("ping {name}"), true);
            v.push(if which("ping") { r } else { r.unavailable("needs ping") });
        }
        let screen = Recorder::new(Kind::Screen, "screen", cfg.screen);
        let wayland = std::env::var_os("WAYLAND_DISPLAY").is_some();
        let screen_missing = if !cfg.screen_cmd.is_empty() {
            None
        } else if wayland {
            (!which("wf-recorder")).then_some("needs wf-recorder")
        } else if std::env::var_os("DISPLAY").is_none() {
            Some("no display ($DISPLAY / $WAYLAND_DISPLAY)")
        } else {
            (!which("ffmpeg")).then_some("needs ffmpeg")
        };
        v.push(match screen_missing {
            Some(why) => screen.unavailable(why),
            None => screen,
        });
        for c in &cfg.command {
            let mut r = Recorder::new(Kind::Command(c.clone()), c.name.clone(), c.enabled);
            if c.name.is_empty() || c.cmd.is_empty() {
                r = r.unavailable("needs a name and a cmd");
            } else if !c.alert.is_empty() {
                match Regex::new(&c.alert) {
                    Ok(re) => r.alert_re = Some(re),
                    Err(e) => r = r.unavailable(format!("bad alert regex: {e}")),
                }
            }
            v.push(r);
        }
        let disk_free = disk_free(&expand_home(&cfg.dir));
        Recording {
            cfg,
            recorders: v,
            run: None,
            cursor: 0,
            tmux_session,
            disk_free,
            namespace: None,
            disk_warned: false,
            last_poll: Instant::now(),
            last: Default::default(),
        }
    }

    pub fn recording(&self) -> bool {
        self.recorders.iter().any(Recorder::active)
    }

    pub fn owns_data(&self) -> bool {
        self.recorders.iter().any(Recorder::owns_data)
    }

    /// The open (not finished) run, creating one if needed.
    fn ensure_run(&mut self, out: &mut Vec<Note>) -> bool {
        if self.run.as_ref().is_some_and(|r| r.finished_wall.is_none()) {
            return true;
        }
        match Run::create(&expand_home(&self.cfg.dir), self.namespace.as_deref()) {
            Ok(run) => {
                out.push(Note::Info(format!("REC new run {}", tilde(&run.dir))));
                self.run = Some(run);
                self.write_state_file();
                for r in &mut self.recorders {
                    if r.available() {
                        r.status = Status::Idle;
                        r.bytes = 0;
                        r.detail.clear();
                        r.ran_for = None;
                        r.started = None;
                    }
                }
                true
            }
            Err(e) => {
                out.push(Note::Warn(format!("REC: could not create a run in {}: {e}", self.cfg.dir)));
                false
            }
        }
    }

    /// The shell recorders (px4-record.sh & co.) look the run up here.
    fn write_state_file(&self) {
        let (Some(session), Some(run)) = (&self.tmux_session, &self.run) else { return };
        let dir = expand_home(&self.cfg.state_dir);
        if std::fs::create_dir_all(&dir).is_ok() {
            let _ = std::fs::write(dir.join(sanitize(session)), format!("{}\n", run.dir.display()));
        }
    }

    pub fn state_file(&self) -> Option<PathBuf> {
        self.tmux_session.as_ref().map(|s| expand_home(&self.cfg.state_dir).join(sanitize(s)))
    }

    /// `R`: open a run and start every included recorder.
    pub fn record_all(&mut self, vehicles: &[Veh], out: &mut Vec<Note>) {
        if !self.ensure_run(out) {
            return;
        }
        // A vehicle's tlog only when that vehicle is asked for (the fleet
        // records the selected ones).
        let which: Vec<usize> = (0..self.recorders.len())
            .filter(|&i| self.recorders[i].include)
            .filter(|&i| match self.recorders[i].kind {
                Kind::Tlog(id) => vehicles.iter().any(|v| v.id == id),
                _ => true,
            })
            .collect();
        if which.is_empty() {
            out.push(Note::Warn("REC: no recorder is included in [R] ([i] includes one)".into()));
        }
        self.start(&which, vehicles, out);
    }

    fn start(&mut self, which: &[usize], vehicles: &[Veh], out: &mut Vec<Note>) {
        let Some(run) = self.run.as_ref() else { return };
        for &i in which {
            self.recorders[i].start(&self.cfg, run, self.tmux_session.as_deref(), vehicles, out);
        }
    }

    /// ENTER: start / stop the selected recorder.
    pub fn toggle(&mut self, i: usize, vehicles: &[Veh], out: &mut Vec<Note>) {
        let Some(r) = self.recorders.get_mut(i) else { return };
        if let Status::Unavailable(why) = &r.status {
            out.push(Note::Warn(format!("REC {}: {why}", r.name)));
        } else if r.active() {
            r.stop(vehicles, out);
        } else if self.ensure_run(out) {
            self.start(&[i], vehicles, out);
        }
    }

    /// A vehicle's link was replaced (fleet ENTER and back): carry on
    /// writing its tlog (appended) on the new one.
    pub fn resume_tlog(&self, id: usize, link: &crate::link::Link) {
        for r in &self.recorders {
            if let (Kind::Tlog(i), Status::Recording, Some(path)) = (&r.kind, &r.status, &r.output)
                && *i == id
            {
                link.start_run_tlog(path.clone());
            }
        }
    }

    pub fn screen_index(&self) -> Option<usize> {
        self.recorders.iter().position(|r| matches!(r.kind, Kind::Screen))
    }

    pub fn stop_all(&mut self, vehicles: &[Veh], out: &mut Vec<Note>) {
        for r in &mut self.recorders {
            r.stop(vehicles, out);
        }
    }

    pub fn add_marker(&mut self, label: &str, text: &str, out: &mut Vec<Note>) {
        let Some(run) = self.run.as_mut().filter(|r| r.finished_wall.is_none()) else {
            out.push(Note::Warn("REC: no open run - [R] starts one".into()));
            return;
        };
        match run.add_marker(label, text) {
            Ok(m) => out.push(Note::Info(format!("REC marker +{} {label} {text}", hms(m.elapsed_s as u64)))),
            Err(e) => out.push(Note::Warn(format!("REC marker: {e}"))),
        }
    }

    /// `<run>` -> `<run>_<suffix>`. Open files keep writing (they hold
    /// descriptors, not paths); tmux pipes are restarted on the new path.
    pub fn rename(&mut self, suffix: &str, out: &mut Vec<Note>) {
        let suffix = sanitize(suffix.trim());
        let Some(run) = self.run.as_mut() else {
            out.push(Note::Warn("REC: nothing recorded yet".into()));
            return;
        };
        if suffix.is_empty() {
            return;
        }
        let new = PathBuf::from(format!("{}_{suffix}", run.dir.display()));
        if new.exists() {
            out.push(Note::Warn(format!("REC rename: {} already exists", tilde(&new))));
            return;
        }
        if let Err(e) = std::fs::rename(&run.dir, &new) {
            out.push(Note::Warn(format!("REC rename: {e}")));
            return;
        }
        let old = std::mem::replace(&mut run.dir, new.clone());
        for r in &mut self.recorders {
            r.rebase(&old, &new);
            // `cat >> file` reopened nothing - point the pipes at the new name.
            if matches!(r.kind, Kind::Tmux) && r.status == Status::Recording
                && let Some(session) = self.tmux_session.as_deref()
            {
                for p in tmux_panes(session) {
                    if let Some((_, f)) = r.panes.iter().find(|(id, _)| *id == p.id) {
                        let _ = tmux_start_pipe(session, &p, f, self.cfg.tmux_strip_ansi);
                    }
                }
            }
        }
        self.write_state_file();
        out.push(Note::Info(format!("REC renamed to {}", tilde(&new))));
    }

    /// Once a second: recorder health, disk space, auto markers.
    pub fn poll(&mut self, ctx: &PollCtx, vehicles: &[VehState], out: &mut Vec<Note>) {
        if self.last_poll.elapsed() < Duration::from_secs(1) {
            return;
        }
        self.last_poll = Instant::now();
        if let Some(run) = self.run.as_ref() {
            for r in &mut self.recorders {
                r.poll(&self.cfg, run, self.tmux_session.as_deref(), ctx, out);
            }
        }
        self.disk_free = disk_free(&expand_home(&self.cfg.dir));
        let low = self.disk_free.is_some_and(|f| (f as f64) < self.cfg.disk_low_gb * 1e9);
        if low && self.recording() && !self.disk_warned {
            self.disk_warned = true;
            let gb = self.disk_free.unwrap_or(0) as f64 / 1e9;
            out.push(Note::Alert(format!("REC: disk almost full - {gb:.1} GB left in {}", self.cfg.dir)));
        } else if !low {
            self.disk_warned = false;
        }

        // Auto markers from each vehicle: arm / disarm, mode changes, alerts.
        let open = self.run.as_ref().is_some_and(|r| r.finished_wall.is_none()) && self.recording();
        let many = vehicles.len() > 1;
        for v in vehicles {
            let who = if many { format!("{} ", v.ns) } else { String::new() };
            let new_alert = v.newest_alert.as_deref().filter(|_| self.last.get(&v.ns).is_some_and(|l| v.alerts > l.2));
            if open && self.cfg.auto_markers {
                if let Some((armed, mode, _)) = self.last.get(&v.ns).cloned() {
                    if armed != v.armed {
                        self.add_marker(if v.armed { "armed" } else { "disarmed" }, who.trim(), out);
                    }
                    if !mode.is_empty() && !v.mode.is_empty() && mode != v.mode {
                        self.add_marker("mode", &format!("{who}{mode} -> {}", v.mode), out);
                    }
                }
                if let Some(text) = new_alert.filter(|t| !t.starts_with("REC:")) {
                    self.add_marker("alert", &format!("{who}{text}"), out);
                }
            }
            if let (Some(run), Some(text)) = (self.run.as_mut(), new_alert)
                && run.finished_wall.is_none()
            {
                run.alerts.push((Local::now().format("%H:%M:%S").to_string(), format!("{who}{text}")));
            }
            self.last.insert(v.ns.clone(), (v.armed, v.mode.clone(), v.alerts));
        }
    }

    /// Stopping recorders still closing (rosbag, ...).
    pub fn stopping(&self) -> bool {
        self.recorders.iter().any(|r| r.status == Status::Stopping)
    }

    /// Mark the run finished (once nothing is left closing) and write
    /// manifest.json + summary.md.
    pub fn write_manifest(&mut self, extra: serde_json::Value) -> Option<PathBuf> {
        let run = self.run.as_mut()?;
        if run.finished_wall.is_none() && !self.recorders.iter().any(Recorder::owns_data) {
            run.finished_wall = Some(Local::now());
        }
        let run = self.run.as_ref()?;
        let markers: Vec<serde_json::Value> = run
            .markers
            .iter()
            .map(|m| serde_json::json!({ "time": m.wall.to_rfc3339(), "elapsed_s": (m.elapsed_s * 1000.0).round() / 1000.0, "label": m.label, "text": m.text }))
            .collect();
        let manifest = serde_json::json!({
            "tool": format!("lazypx4 {}", env!("CARGO_PKG_VERSION")),
            "run": run.name(),
            "namespace": run.namespace,
            "directory": run.dir.display().to_string(),
            "host": std::fs::read_to_string("/proc/sys/kernel/hostname").map(|s| s.trim().to_string()).unwrap_or_default(),
            "user": std::env::var("USER").unwrap_or_default(),
            "tmux_session": self.tmux_session,
            "started": run.started_wall.to_rfc3339(),
            "finished": run.finished_wall.map(|t| t.to_rfc3339()),
            "duration_s": run.elapsed_s(),
            "recorders": self.recorders.iter().map(Recorder::manifest).collect::<Vec<_>>(),
            "markers": markers,
            "alerts": run.alerts.iter().map(|(t, a)| serde_json::json!({ "time": t, "text": a })).collect::<Vec<_>>(),
            "flight_review": run.review_url,
            "vehicle": extra,
        });
        let path = run.dir.join("manifest.json");
        std::fs::write(&path, serde_json::to_string_pretty(&manifest).unwrap_or_default() + "\n").ok()?;
        let _ = std::fs::write(run.dir.join("summary.md"), self.summary(&manifest));
        Some(path)
    }

    fn summary(&self, manifest: &serde_json::Value) -> String {
        use std::fmt::Write as _;
        let Some(run) = self.run.as_ref() else { return String::new() };
        let mut s = String::new();
        let _ = writeln!(s, "# {}\n", run.name());
        let _ = writeln!(s, "- started {} · {} long", run.started_wall.format("%Y-%m-%d %H:%M:%S"), hms(run.elapsed_s()));
        if let Some(v) = manifest["vehicle"].as_object() {
            for (k, v) in v {
                let text = v.as_str().map(str::to_string).unwrap_or_else(|| v.to_string());
                if !text.is_empty() && text != "null" {
                    let _ = writeln!(s, "- {k}: {text}");
                }
            }
        }
        if let Some(url) = &run.review_url {
            let _ = writeln!(s, "- Flight Review: {url}");
        }
        let _ = writeln!(s, "\n## Recorders\n\n| recorder | status | size | detail |\n|---|---|---|---|");
        for r in &self.recorders {
            if r.started.is_some() || !matches!(r.status, Status::Idle) {
                let reason = match &r.status {
                    Status::Failed(w) | Status::Unavailable(w) => format!(" ({w})"),
                    _ => String::new(),
                };
                let _ = writeln!(s, "| {} | {}{reason} | {} | {} |", r.name, r.status.label(), crate::state::size_string(r.bytes), r.detail);
            }
        }
        if !run.markers.is_empty() {
            let _ = writeln!(s, "\n## Markers\n");
            for m in &run.markers {
                let _ = writeln!(s, "- `+{}` {} **{}** {}", hms(m.elapsed_s as u64), m.wall.format("%H:%M:%S"), m.label, m.text);
            }
        }
        if !run.alerts.is_empty() {
            let _ = writeln!(s, "\n## Alerts\n");
            for (t, a) in &run.alerts {
                let _ = writeln!(s, "- `{t}` {a}");
            }
        }
        s
    }

    /// On quit: stop everything and give the children their grace to
    /// close (rosbag writes its metadata on SIGINT).
    pub fn shutdown(&mut self, vehicles: &[Veh]) {
        if !self.recording() {
            return;
        }
        let mut notes = Vec::new();
        self.stop_all(vehicles, &mut notes);
        let deadline = Instant::now() + ROSBAG_STOP_GRACE + CHILD_STOP_GRACE;
        while self.stopping() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(100));
            for r in &mut self.recorders {
                r.poll_child(&mut notes);
            }
        }
        self.write_manifest(serde_json::Value::Null);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scripts_are_valid_bash() {
        let mut cfg = RecordConfig { screen_window: "QGround".into(), ..Default::default() };
        let mut scripts = vec![screen_script(&cfg)];
        cfg.screen_cmd = "wf-recorder -f \"$OUT\"".into();
        scripts.push(screen_script(&cfg));
        for s in scripts {
            let out = Command::new("bash").arg("-n").arg("-c").arg(&s).output().unwrap();
            assert!(out.status.success(), "{s}\n{}", String::from_utf8_lossy(&out.stderr));
        }
    }

    #[test]
    fn config_parses_commands_and_defaults() {
        let cfg: RecordConfig = toml::from_str("ping = [\"drone=10.0.0.5\"]\n[[command]]\nname = \"kernel\"\ncmd = \"dmesg -w\"\n").unwrap();
        assert!(cfg.tlog && cfg.command[0].enabled);
        assert_eq!(cfg.dir, "~/logs");
        assert_eq!(cfg.ping, vec!["drone=10.0.0.5"]);
    }

    #[test]
    fn markers_go_to_csv_and_notes() {
        let base = std::env::temp_dir().join(format!("lazypx4_rec_test_{}", std::process::id()));
        let mut run = Run::create(&base, Some("uav_1")).unwrap();
        assert!(run.name().ends_with("_uav_1"));
        run.add_marker("takeoff", "first \"try\"").unwrap();
        let csv = std::fs::read_to_string(run.dir.join("markers.csv")).unwrap();
        assert!(csv.lines().nth(1).unwrap().ends_with("\"takeoff\",\"first \"\"try\"\"\""));
        assert!(std::fs::read_to_string(run.dir.join("notes.md")).unwrap().contains("**takeoff** first \"try\""));
        let _ = std::fs::remove_dir_all(base);
    }

    #[test]
    fn names_are_sanitized() {
        assert_eq!(sanitize("a b/c:d"), "a_b_c_d");
        assert_eq!(hms(3725), "01:02:05");
    }
}
