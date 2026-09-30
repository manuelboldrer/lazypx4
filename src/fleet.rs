//! `--fleet PORTS`: one row per vehicle, one UDP port each, with the basic
//! flight commands on the selected (or SPACE-marked) vehicles.
//!
//! Every port runs the same receiver + periodic service as the full UI
//! (`receiver_thread` + `service_vehicle` on its own `State`), so commands,
//! their ACKs, the health latches and the alerts behave exactly as they do
//! there - and the GCS heartbeat keeps PX4's data-link-loss failsafe quiet
//! for every vehicle. ENTER stops a port's unit (freeing the port) and runs
//! a full lazypx4 on it in the foreground; quitting that returns here and
//! the unit restarts. The other vehicles are served the whole time.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::DefaultTerminal;
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Cell, Paragraph, Row, Table, TableState};

use crate::app::GotoTarget;
use crate::app_map::{Goto, parse_goto};
use crate::config::*;
use crate::link::Link;
use crate::mav::{commands, flightlog, modes};
use crate::state::{self, Level, Shared, State, now};
use crate::ui::{GREEN, RED, YELLOW};

/// "14560,14561" / "14560-14563" / a mix.
pub fn parse_ports(text: &str) -> Result<Vec<u16>, String> {
    let mut ports = Vec::new();
    for part in text.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let bad = || format!("--fleet: '{part}' is not a port or a range like 14560-14563");
        match part.split_once('-') {
            Some((a, b)) => {
                let (a, b): (u16, u16) = (a.trim().parse().map_err(|_| bad())?, b.trim().parse().map_err(|_| bad())?);
                if a > b || b - a > 64 {
                    return Err(bad());
                }
                ports.extend(a..=b);
            }
            None => ports.push(part.parse().map_err(|_| bad())?),
        }
    }
    ports.dedup();
    if ports.is_empty() {
        return Err("--fleet: no ports given (e.g. --fleet 14560-14563, or [fleet] ports in lazypx4.toml)".into());
    }
    Ok(ports)
}

// ---------------------------------------------------------------------------
// One vehicle: link + state + its two threads
// ---------------------------------------------------------------------------

struct Unit {
    link: Arc<Link>,
    shared: Shared,
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
    /// Flight-log chunks have nowhere to go here; kept so sends don't fail.
    _log_rx: Receiver<flightlog::Chunk>,
}

fn start_unit(port: u16, settings: &Settings) -> std::io::Result<Unit> {
    let link = Arc::new(Link::bind(port, settings.tlog_dir.as_deref())?);
    let shared: Shared = Arc::new(Mutex::new(State::new()));
    let stop = Arc::new(AtomicBool::new(false));
    let (log_tx, log_rx) = std::sync::mpsc::channel();

    let receiver = {
        let (link, shared, stop) = (link.clone(), shared.clone(), stop.clone());
        std::thread::Builder::new()
            .name(format!("FleetRx{port}"))
            .spawn(move || crate::mav::handlers::receiver_thread(link, shared, log_tx, stop))?
    };
    let service = {
        let (link, shared, stop, settings) = (link.clone(), shared.clone(), stop.clone(), settings.clone());
        std::thread::Builder::new().name(format!("FleetSvc{port}")).spawn(move || {
            let mut last_heartbeat = 0.0;
            while !stop.load(Ordering::Relaxed) {
                let t = now();
                if t - last_heartbeat >= 1.0 && link.has_peer() {
                    last_heartbeat = t;
                    commands::send_gcs_heartbeat(&link);
                }
                crate::app::service_vehicle(&link, &mut state::lock(&shared), &settings);
                std::thread::sleep(Duration::from_millis(100));
            }
        })?
    };
    Ok(Unit { link, shared, stop, threads: vec![receiver, service], _log_rx: log_rx })
}

fn stop_unit(unit: Unit) {
    unit.stop.store(true, Ordering::Relaxed);
    // recv() times out within 500 ms; then the socket is dropped.
    for t in unit.threads {
        let _ = t.join();
    }
}

struct Slot {
    port: u16,
    unit: Option<Unit>,
    error: String,
    /// The port is handed to a full lazypx4 right now.
    opened: bool,
    /// Last PARAM_REQUEST_READ for the preflight's CAL_* / GF_ACTION.
    params_requested_at: f64,
}

impl Slot {
    fn start(&mut self, settings: &Settings) {
        match start_unit(self.port, settings) {
            Ok(u) => {
                self.unit = Some(u);
                self.error.clear();
            }
            Err(e) => self.error = format!("port {}: {e}", self.port),
        }
    }

    fn label(&self) -> String {
        match self.unit.as_ref().map(|u| state::lock(&u.shared).target_system) {
            Some(sys) if sys != 0 => format!("sys {sys}"),
            _ => format!("port {}", self.port),
        }
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum Command {
    Arm,
    Disarm,
    Takeoff(f64),
    Land,
    Rtl,
    Hold,
    Kill,
    Goto(GotoTarget),
    /// Resolved per vehicle: each may offer different modes.
    Mode(String),
    /// Toggle per vehicle: enter OFFBOARD, or leave it for HOLD.
    Offboard,
    SetHome,
    EkfReset,
    Reboot,
    /// Arm keyboard jog on this (single) vehicle.
    Jog,
}

#[derive(Clone, Copy, PartialEq)]
enum InputKind {
    Takeoff,
    Goto,
    Mode,
}

enum Prompt {
    Input { kind: InputKind, text: String, targets: Vec<usize> },
    Confirm { text: String, buffer: String, command: Command, targets: Vec<usize> },
}

fn execute(unit: &Unit, command: &Command) {
    let (link, shared) = (&unit.link, &unit.shared);
    match command {
        Command::Arm => {
            commands::send_arm(link, shared, true);
        }
        Command::Disarm => {
            commands::send_arm(link, shared, false);
        }
        Command::Takeoff(alt) => {
            commands::send_takeoff(link, shared, *alt);
        }
        Command::Land => {
            commands::send_land(link, shared);
        }
        Command::Rtl => {
            commands::send_rtl(link, shared);
        }
        Command::Hold => {
            commands::send_hold(link, shared);
        }
        Command::Kill => {
            commands::send_kill(link, shared);
        }
        Command::Goto(target) => {
            crate::app::send_goto(link, &mut state::lock(shared), target.clone());
        }
        Command::Offboard => {
            let enter = state::lock(shared).mode != "OFFBOARD";
            commands::send_offboard(link, shared, enter);
        }
        Command::SetHome => {
            commands::send_set_home_current(link, shared);
        }
        Command::Reboot => {
            commands::send_reboot(link, shared);
        }
        Command::EkfReset => {
            let mut st = state::lock(shared);
            if !st.shell_active && !crate::mav::shell::claim_shell(link) {
                return st.error("Could not open MAVLink shell for EKF reset");
            }
            st.shell_active = true;
            crate::mav::shell::send_shell_raw(link, b"ekf stop\n");
            crate::mav::shell::send_shell_raw(link, b"ekf start\n");
            st.command("EKF reset: ekf stop / ekf start");
        }
        // Handled by the fleet (it owns the jog state).
        Command::Jog => {}
        Command::Mode(query) => {
            let options = modes::mode_options(&state::lock(shared));
            let labels: Vec<String> = options.iter().map(|o| o.label.clone()).collect();
            match crate::cmdline::find_mode(&labels, query) {
                Ok(i) => modes::confirm_mode_option(link, shared, &options[i]),
                Err(e) => state::lock(shared).error(e),
            }
        }
    }
}

/// "GO" / "GO 3w" / "NO-GO 2f" and its colour: the PREFLIGHT column and
/// the arm confirmation.
fn short_verdict(checks: &[crate::preflight::Check]) -> (String, Color) {
    match crate::preflight::verdict(checks) {
        (crate::preflight::Status::Fail, f, _) => (format!("NO-GO {f}f"), RED),
        (crate::preflight::Status::Warn, _, w) => (format!("GO {w}w"), YELLOW),
        _ => ("GO".into(), GREEN),
    }
}

/// What the lower pane shows when there's only room for one thing.
#[derive(Clone, Copy, PartialEq)]
enum Pane {
    Events,
    Preflight,
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

struct Fleet<'a> {
    slots: Vec<Slot>,
    settings: &'a Settings,
    table: TableState,
    marked: Vec<bool>,
    prompt: Option<Prompt>,
    note: String,
    jog: Option<Jog>,
    pane: Pane,
    /// Companion-computer stats - one host, shared by every row's preflight.
    sys: Arc<Mutex<crate::host::SystemStats>>,
}

/// Keyboard jog on one vehicle: every key is a DO_REPOSITION nudge.
struct Jog {
    row: usize,
    step: f64,
    last: f64,
}

impl Fleet<'_> {
    fn checks(&self, unit: &Unit) -> Vec<crate::preflight::Check> {
        let sys = crate::host::lock(&self.sys);
        crate::preflight::checks(&state::lock(&unit.shared), &sys, self.settings)
    }

    /// PARAM_REQUEST_READ the CAL_* ids + GF_ACTION the preflight needs
    /// (the fleet never loads the full list), every 10 s while missing.
    fn request_preflight_params(&mut self) {
        let t = now();
        for slot in &mut self.slots {
            let Some(unit) = &slot.unit else { continue };
            if t - slot.params_requested_at < 10.0 {
                continue;
            }
            let missing: Vec<&str> = {
                let st = state::lock(&unit.shared);
                if !st.vehicle_locked {
                    continue;
                }
                crate::preflight::CALIBRATION_PARAMS
                    .iter()
                    .copied()
                    .chain(["GF_ACTION"])
                    .filter(|p| st.param_value(p).is_none())
                    .collect()
            };
            if !missing.is_empty() {
                slot.params_requested_at = t;
                for p in missing {
                    commands::request_param(&unit.link, p);
                }
            }
        }
    }

    fn selected(&self) -> usize {
        self.table.selected().unwrap_or(0).min(self.slots.len() - 1)
    }

    /// Marked rows, else the selected one - only rows with a live unit.
    fn targets(&self) -> Vec<usize> {
        let marked: Vec<usize> = (0..self.slots.len()).filter(|&i| self.marked[i]).collect();
        let rows = if marked.is_empty() { vec![self.selected()] } else { marked };
        rows.into_iter().filter(|&i| self.slots[i].unit.is_some()).collect()
    }

    fn names(&self, targets: &[usize]) -> String {
        let labels: Vec<String> = targets.iter().map(|&i| self.slots[i].label()).collect();
        if targets.len() == 1 { labels[0].clone() } else { format!("{} vehicles ({})", targets.len(), labels.join(", ")) }
    }

    fn confirm(&mut self, what: &str, command: Command, targets: Vec<usize>) {
        let text = format!("{what} {}? Type YES", self.names(&targets));
        self.prompt = Some(Prompt::Confirm { text, buffer: String::new(), command, targets });
    }

    fn command_key(&mut self, c: char) {
        let targets = self.targets();
        if targets.is_empty() {
            self.note = "no vehicle there (port not open)".into();
            return;
        }
        match c {
            'a' => {
                let verdicts: Vec<String> = targets
                    .iter()
                    .map(|&i| {
                        let unit = self.slots[i].unit.as_ref().unwrap();
                        format!("{} {}", self.slots[i].label(), short_verdict(&self.checks(unit)).0)
                    })
                    .collect();
                let text = format!("ARM {}? Preflight: {}. Type YES", self.names(&targets), verdicts.join(" · "));
                self.prompt = Some(Prompt::Confirm { text, buffer: String::new(), command: Command::Arm, targets });
            }
            'd' => self.confirm("DISARM", Command::Disarm, targets),
            'L' => self.confirm("LAND", Command::Land, targets),
            'R' => self.confirm("RETURN TO LAUNCH", Command::Rtl, targets),
            'h' => self.confirm("HOLD", Command::Hold, targets),
            'K' => self.confirm("KILL MOTORS NOW (flight termination, even in flight) on", Command::Kill, targets),
            'T' => self.prompt = Some(Prompt::Input { kind: InputKind::Takeoff, text: String::new(), targets }),
            'm' => self.prompt = Some(Prompt::Input { kind: InputKind::Mode, text: String::new(), targets }),
            // Several vehicles to one absolute point would collide, and a
            // shared relative offset is rarely what's meant: one at a time.
            'g' if targets.len() > 1 => self.note = "goto: one vehicle at a time (ESC clears the marks)".into(),
            'g' => self.prompt = Some(Prompt::Input { kind: InputKind::Goto, text: String::new(), targets }),
            'F' => self.confirm("OFFBOARD on/off (in OFFBOARD -> HOLD, else -> OFFBOARD; needs streaming setpoints) for", Command::Offboard, targets),
            'H' => self.confirm("Set HOME to the current position of", Command::SetHome, targets),
            'E' => self.confirm("RESET EKF (ekf stop / ekf start) on", Command::EkfReset, targets),
            'b' => self.confirm("REBOOT PX4 on", Command::Reboot, targets),
            'x' if targets.len() > 1 => self.note = "jog: one vehicle at a time (ESC clears the marks)".into(),
            'x' => {
                let st = state::lock(&self.slots[targets[0]].unit.as_ref().unwrap().shared);
                let refuse = if !st.armed {
                    Some("jog: arm the vehicle first")
                } else if !st.global_pos_valid {
                    Some("jog: needs a GPS / global position")
                } else {
                    None
                };
                drop(st);
                match refuse {
                    Some(r) => self.note = r.into(),
                    None => self.confirm(
                        "Arm keyboard JOG (w/s up/down, k/j fwd/back, a/d left/right, h/l yaw, [/] step, x/ESC stop) on",
                        Command::Jog,
                        targets,
                    ),
                }
            }
            _ => {}
        }
    }

    /// A key while jog is armed. Everything is swallowed so a stray
    /// a/d/h can't turn into arm / disarm / hold.
    fn jog_key(&mut self, code: KeyCode) {
        let Some(jog) = self.jog.as_mut() else { return };
        let key = match code {
            KeyCode::Esc | KeyCode::Char('x') => {
                self.jog = None;
                self.note = "jog disarmed".into();
                return;
            }
            KeyCode::Char('[') => {
                jog.step = (jog.step / 2.0).clamp(JOG_STEP_MIN_M, JOG_STEP_MAX_M);
                return;
            }
            KeyCode::Char(']') => {
                jog.step = (jog.step * 2.0).clamp(JOG_STEP_MIN_M, JOG_STEP_MAX_M);
                return;
            }
            KeyCode::Char(c) => c.to_string(),
            _ => return,
        };
        let Some((f, r, d, yaw_sign)) = crate::app_map::jog_vector(&key) else { return };
        let t = now();
        if t - jog.last < JOG_MIN_INTERVAL {
            return;
        }
        jog.last = t;
        let Some(unit) = &self.slots[jog.row].unit else {
            self.jog = None;
            return;
        };
        let step = jog.step;
        let (f, r, d) = (f * step, r * step, d * step);
        let mut st = state::lock(&unit.shared);
        let yaw = (yaw_sign != 0.0).then(|| (st.yaw + yaw_sign * JOG_YAW_STEP_DEG).rem_euclid(360.0));
        if crate::mav::guided::goto_body(&unit.link, &mut st, f, r, d, yaw, true) {
            match yaw {
                Some(y) => st.command(format!("JOG yaw -> {y:.0}")),
                None => st.command(format!("JOG fwd {f:+.1} right {r:+.1} down {d:+.1} m")),
            }
        }
    }

    fn submit_input(&mut self, kind: InputKind, text: String, targets: Vec<usize>) {
        match kind {
            InputKind::Takeoff => {
                let alt = if text.trim().is_empty() { 2.5 } else { text.trim().parse::<f64>().unwrap_or(f64::NAN) };
                if !(alt.is_finite() && alt > 0.0) {
                    self.note = "takeoff: altitude must be a positive number".into();
                    return;
                }
                self.confirm(&format!("TAKEOFF to {alt:.1} m:"), Command::Takeoff(alt), targets);
            }
            InputKind::Mode => {
                if text.trim().is_empty() {
                    return;
                }
                self.confirm(&format!("Set mode {} on", text.trim()), Command::Mode(text.trim().to_string()), targets);
            }
            InputKind::Goto => match parse_goto(&text) {
                Ok(Goto::Target(prompt, target)) => {
                    let text = format!("{} - {}", self.names(&targets), prompt);
                    self.prompt = Some(Prompt::Confirm { text, buffer: String::new(), command: Command::Goto(target), targets });
                }
                Ok(Goto::Fire(_)) => self.note = "goto: the fire target needs the full UI (ENTER)".into(),
                Err(e) => self.note = e,
            },
        }
    }

    fn prompt_key(&mut self, code: KeyCode) {
        let Some(prompt) = self.prompt.as_mut() else { return };
        match (code, prompt) {
            (KeyCode::Esc, _) => self.prompt = None,
            (KeyCode::Backspace, Prompt::Input { text, .. } | Prompt::Confirm { buffer: text, .. }) => {
                text.pop();
            }
            (KeyCode::Char(c), Prompt::Input { text, .. }) if text.len() < 128 => text.push(c),
            (KeyCode::Char(c), Prompt::Confirm { buffer, .. }) => {
                buffer.push(c.to_ascii_uppercase());
                if buffer.len() > 3 {
                    buffer.remove(0);
                }
            }
            (KeyCode::Enter, Prompt::Input { .. }) => {
                if let Some(Prompt::Input { kind, text, targets }) = self.prompt.take() {
                    self.submit_input(kind, text, targets);
                }
            }
            (KeyCode::Enter, Prompt::Confirm { buffer, .. }) => {
                if buffer != "YES" {
                    buffer.clear();
                    self.note = "type YES to confirm".into();
                    return;
                }
                if let Some(Prompt::Confirm { command, targets, .. }) = self.prompt.take() {
                    if let Command::Jog = command {
                        self.table.select(Some(targets[0]));
                        self.jog = Some(Jog { row: targets[0], step: JOG_STEP_M, last: 0.0 });
                        self.note.clear();
                        return;
                    }
                    for &i in &targets {
                        if let Some(unit) = &self.slots[i].unit {
                            execute(unit, &command);
                        }
                    }
                    self.note = format!("sent to {} - replies below / in each row", self.names(&targets));
                }
            }
            _ => {}
        }
    }

    /// Free the port, run the full UI on it, then serve it again.
    fn open(&mut self, terminal: &mut DefaultTerminal, config: Option<&str>) -> std::io::Result<()> {
        let i = self.selected();
        let port = self.slots[i].port;
        if let Some(u) = self.slots[i].unit.take() {
            stop_unit(u);
        }
        self.slots[i].opened = true;
        let exe = std::env::current_exe().unwrap_or_else(|_| "lazypx4".into());
        let mut command = std::process::Command::new(exe);
        command.arg("--port").arg(port.to_string());
        if let Some(c) = config {
            command.arg("--config").arg(c);
        }
        let status = crate::suspend::run_foreground(terminal, command, "", false, || {})?;
        self.note = match status {
            Ok(s) if s.success() => format!("back from port {port}"),
            Ok(s) => format!("lazypx4 on port {port} exited with {s}"),
            Err(e) => format!("could not start lazypx4: {e}"),
        };
        self.slots[i].opened = false;
        self.slots[i].start(self.settings);
        Ok(())
    }
}

pub fn run(terminal: &mut DefaultTerminal, settings: &Settings, ports: &[u16], config: Option<&str>) -> std::io::Result<()> {
    let mut fleet = Fleet {
        slots: ports.iter().map(|&port| Slot { port, unit: None, error: String::new(), opened: false, params_requested_at: 0.0 }).collect(),
        settings,
        table: TableState::default().with_selected(Some(0)),
        marked: vec![false; ports.len()],
        prompt: None,
        note: String::new(),
        jog: None,
        pane: Pane::Events,
        sys: Arc::new(Mutex::new(crate::host::SystemStats::default())),
    };
    let sysmon_stop = Arc::new(AtomicBool::new(false));
    {
        let (sys, disk, sd) = (fleet.sys.clone(), settings.disk_path.clone(), sysmon_stop.clone());
        std::thread::Builder::new().name("SysMon".into()).spawn(move || crate::host::sysmon_thread(sys, disk, sd))?;
    }
    for slot in &mut fleet.slots {
        slot.start(settings);
    }

    loop {
        let bell = fleet
            .slots
            .iter()
            .filter_map(|s| s.unit.as_ref())
            .fold(false, |any, u| std::mem::take(&mut state::lock(&u.shared).alert_bell) | any);
        if bell && settings.alert_bell {
            use std::io::Write;
            let _ = std::io::stdout().write_all(b"\x07");
            let _ = std::io::stdout().flush();
        }
        fleet.request_preflight_params();
        terminal.draw(|f| draw(f, &mut fleet))?;

        if !event::poll(FRAME_PERIOD)? {
            continue;
        }
        let Event::Key(key) = event::read()? else { continue };
        if key.kind == KeyEventKind::Release {
            continue;
        }
        if key.code == KeyCode::Char('c') && key.modifiers.contains(KeyModifiers::CONTROL) {
            break;
        }
        if fleet.prompt.is_some() {
            fleet.prompt_key(key.code);
            continue;
        }
        if fleet.jog.is_some() {
            fleet.jog_key(key.code);
            continue;
        }
        fleet.note.clear();
        let (i, n) = (fleet.selected(), ports.len());
        match key.code {
            KeyCode::Char('q') => break,
            KeyCode::Char('j') | KeyCode::Down => fleet.table.select(Some((i + 1) % n)),
            KeyCode::Char('k') | KeyCode::Up => fleet.table.select(Some((i + n - 1) % n)),
            KeyCode::Char(' ') => {
                fleet.marked[i] = !fleet.marked[i];
                fleet.table.select(Some((i + 1) % n));
            }
            KeyCode::Esc => fleet.marked.iter_mut().for_each(|m| *m = false),
            KeyCode::Char('A') => {
                for u in fleet.slots.iter().filter_map(|s| s.unit.as_ref()) {
                    state::lock(&u.shared).alerts.clear();
                }
            }
            KeyCode::Enter => fleet.open(terminal, config)?,
            KeyCode::Char('y') => {
                fleet.pane = if fleet.pane == Pane::Events { Pane::Preflight } else { Pane::Events };
            }
            KeyCode::Char(c) => fleet.command_key(c),
            _ => {}
        }
    }
    sysmon_stop.store(true, Ordering::Relaxed);
    for slot in fleet.slots {
        if let Some(u) = slot.unit {
            stop_unit(u);
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

/// Everything a row needs, copied out under the lock.
struct RowData {
    sys: u8,
    locked: bool,
    heartbeat_age: f64,
    armed: bool,
    pending_arm: bool,
    mode: String,
    battery: f64,
    voltage: f64,
    battery_critical: bool,
    battery_low: bool,
    gps_fix: i32,
    sats: i32,
    alt: Option<f64>,
    alert: Option<String>,
    /// Newest non-INFO event: replies to commands, warnings.
    last: Option<(Level, String)>,
    verdict: (String, Color),
}

fn row_data(st: &State, sys: &crate::host::SystemStats, settings: &Settings) -> RowData {
    RowData {
        // Nothing to judge until a vehicle answers on the port.
        verdict: if st.vehicle_locked {
            short_verdict(&crate::preflight::checks(st, sys, settings))
        } else {
            ("--".into(), Color::DarkGray)
        },
        sys: st.target_system,
        locked: st.vehicle_locked,
        heartbeat_age: now() - st.last_heartbeat,
        armed: st.armed,
        pending_arm: st.pending_arm.is_some(),
        mode: st.mode.clone(),
        battery: st.battery,
        voltage: st.voltage,
        battery_critical: st.battery_critical_active,
        battery_low: st.battery_low_active,
        gps_fix: st.gps_fix,
        sats: st.gps_sats,
        alt: st.local_pos_valid.then_some(-st.local_z),
        alert: st.alerts.last().map(|(_, t)| t.clone()),
        last: st.events.iter().rev().find(|e| e.level != Level::Info).map(|e| (e.level, e.message.clone())),
    }
}

fn level_style(level: Level) -> Style {
    match level {
        Level::Failsafe | Level::Error => Style::new().fg(RED),
        Level::Warn => Style::new().fg(YELLOW),
        Level::Command => Style::new().fg(Color::Cyan),
        _ => Style::new().add_modifier(Modifier::DIM),
    }
}

fn draw(frame: &mut ratatui::Frame, fleet: &mut Fleet) {
    let prompt_rows = if fleet.prompt.is_some() { 2 } else { 0 };
    let table_rows = fleet.slots.len() as u16 + 3;
    let [body, detail, keys, prompt_area] = Layout::vertical([
        Constraint::Length(table_rows.min(frame.area().height.saturating_sub(6))),
        Constraint::Min(3),
        Constraint::Length(1),
        Constraint::Length(prompt_rows),
    ])
    .areas(frame.area());
    let flash = (now() * 2.0) as i64 % 2 == 0;
    let dim = Style::new().add_modifier(Modifier::DIM);

    let data: Vec<Option<RowData>> = {
        let sys = crate::host::lock(&fleet.sys);
        fleet.slots.iter().map(|s| s.unit.as_ref().map(|u| row_data(&state::lock(&u.shared), &sys, fleet.settings))).collect()
    };

    let rows: Vec<Row> = fleet
        .slots
        .iter()
        .zip(&data)
        .enumerate()
        .map(|(i, (slot, d))| {
            let mark = if fleet.marked[i] { Span::styled("●", Style::new().fg(Color::Cyan)) } else { Span::raw(" ") };
            let Some(d) = d else {
                let (link, style, text) = if slot.opened {
                    ("OPEN", Style::new().fg(Color::Cyan), "full lazypx4 running on this port".to_string())
                } else {
                    ("ERROR", Style::new().fg(RED), slot.error.clone())
                };
                return Row::new(vec![
                    Cell::from(mark),
                    Cell::from(slot.port.to_string()),
                    Cell::from("-"),
                    Cell::from(Span::styled(link, style)),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(""),
                    Cell::from(Span::styled(text, style)),
                ]);
            };
            let (link, link_style) = if !d.locked {
                ("waiting".to_string(), Style::new().fg(YELLOW))
            } else if d.heartbeat_age > HEARTBEAT_TIMEOUT {
                (format!("LOST {:.0}s", d.heartbeat_age), Style::new().fg(RED).add_modifier(Modifier::BOLD))
            } else {
                ("OK".to_string(), Style::new().fg(GREEN))
            };
            let armed = match (d.armed, d.pending_arm) {
                _ if !d.locked => Span::styled("--", dim),
                (_, true) => Span::styled("pending", Style::new().fg(YELLOW)),
                (true, _) => Span::styled("ARMED", Style::new().fg(RED).add_modifier(Modifier::BOLD)),
                _ => Span::styled("disarmed", Style::new().fg(GREEN)),
            };
            let battery = if d.battery < 0.0 {
                Span::styled("--", dim)
            } else {
                let color = if d.battery_critical { RED } else if d.battery_low { YELLOW } else { GREEN };
                Span::styled(format!("{:.0}% {:.1}V", d.battery, d.voltage), Style::new().fg(color))
            };
            let gps_color = match d.gps_fix {
                f if f >= 3 => GREEN,
                2 => YELLOW,
                _ => RED,
            };
            let text = match (&d.alert, &d.last) {
                (Some(a), _) => Span::styled(format!("⚠ {a}"), Style::new().fg(RED).add_modifier(Modifier::BOLD)),
                (None, Some((level, m))) => Span::styled(m.clone(), level_style(*level)),
                _ => Span::raw(""),
            };
            let row = Row::new(vec![
                Cell::from(mark),
                Cell::from(slot.port.to_string()),
                Cell::from(if d.locked { d.sys.to_string() } else { "-".into() }),
                Cell::from(Span::styled(link, link_style)),
                Cell::from(armed),
                Cell::from(Span::styled(d.verdict.0.clone(), Style::new().fg(d.verdict.1))),
                Cell::from(if d.locked { d.mode.clone() } else { String::new() }),
                Cell::from(battery),
                Cell::from(Span::styled(format!("{} {}", gps_fix_name(d.gps_fix), d.sats.max(0)), Style::new().fg(gps_color))),
                Cell::from(d.alt.map(|a| format!("{a:.1} m")).unwrap_or_else(|| "--".into())),
                Cell::from(text),
            ]);
            if d.alert.is_some() && flash { row.style(Style::new().bg(Color::Rgb(90, 0, 0))) } else { row }
        })
        .collect();

    let header = Row::new(["", "PORT", "SYS", "LINK", "ARMED", "PREFLIGHT", "MODE", "BATTERY", "GPS", "ALT", "LAST REPLY / WARNING / ALERT"])
        .style(Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD));
    let widths = [
        Constraint::Length(1),
        Constraint::Length(6),
        Constraint::Length(4),
        Constraint::Length(9),
        Constraint::Length(9),
        Constraint::Length(10),
        Constraint::Length(12),
        Constraint::Length(12),
        Constraint::Length(14),
        Constraint::Length(9),
        Constraint::Min(10),
    ];
    let alerts = data.iter().flatten().filter(|d| d.alert.is_some()).count();
    let marked = fleet.marked.iter().filter(|m| **m).count();
    let mut title = format!(" FLEET · {} vehicle(s) ", fleet.slots.len());
    if marked > 0 {
        title.push_str(&format!("· {marked} MARKED "));
    }
    if alerts > 0 {
        title.push_str(&format!("· {alerts} ALERT(S) "));
    }
    let widget = Table::new(rows, widths)
        .header(header)
        .row_highlight_style(Style::new().add_modifier(Modifier::REVERSED))
        .block(Block::bordered().border_type(BorderType::Rounded).border_style(Style::new().fg(GREEN)).title(Line::from(title).bold()));
    frame.render_stateful_widget(widget, body, &mut fleet.table);

    // The selected vehicle's recent events: command replies land here.
    let i = fleet.selected();
    let slot = &fleet.slots[i];
    let height = detail.height.saturating_sub(2) as usize;
    let lines: Vec<Line> = match &slot.unit {
        Some(u) => {
            let st = state::lock(&u.shared);
            st.events
                .iter()
                .rev()
                .take(height)
                .rev()
                .map(|e| {
                    Line::from(vec![
                        Span::styled(format!(" {} ", e.time), dim),
                        Span::styled(format!("{:<9}", e.level.as_str()), level_style(e.level)),
                        Span::raw(e.message.clone()),
                    ])
                })
                .collect()
        }
        None => vec![Line::styled(format!(" {}", if slot.opened { "open in the full UI" } else { &slot.error }), dim)],
    };
    let events = Paragraph::new(lines).block(
        Block::bordered()
            .border_type(BorderType::Rounded)
            .border_style(dim)
            .title(format!(" EVENTS · {} ", slot.label())),
    );
    let checks = match &slot.unit {
        Some(u) => crate::ui::preflight::check_lines(&fleet.checks(u)),
        None => vec![Line::styled(" no vehicle on this port right now", dim)],
    };
    let preflight = Paragraph::new(checks).block(
        Block::bordered()
            .border_type(BorderType::Rounded)
            .border_style(dim)
            .title(format!(" PREFLIGHT · {} ", slot.label())),
    );
    // Side by side when there's room, else [y] picks one.
    if detail.width >= 150 {
        let [left, right] = Layout::horizontal([Constraint::Min(40), Constraint::Length(84)]).areas(detail);
        frame.render_widget(events, left);
        frame.render_widget(preflight, right);
    } else if fleet.pane == Pane::Preflight {
        frame.render_widget(preflight, detail);
    } else {
        frame.render_widget(events, detail);
    }

    let mut bar = match &fleet.jog {
        Some(j) => vec![Span::styled(
            format!(
                " JOG {} · step {:.2} m · w/s up/down · k/j fwd/back · a/d left/right · h/l yaw · [/] step · x/ESC stop ",
                fleet.slots[j.row].label(),
                j.step
            ),
            Style::new().bg(YELLOW).fg(Color::Black).add_modifier(Modifier::BOLD),
        )],
        None => vec![Span::styled(
            " [a]rm [d]isarm [T]akeoff [L]and [R]TL [h]old [g]oto [x]jog [m]ode o[F]fboard [H]ome [E]KF [b]reboot [K]ill · [y] preflight · SPACE mark · ENTER full UI · [A]ck · q",
            dim,
        )],
    };
    if !fleet.note.is_empty() {
        bar.push(Span::styled(format!("   {}", fleet.note), Style::new().fg(YELLOW)));
    }
    frame.render_widget(Paragraph::new(Line::from(bar)), keys);

    if let Some(p) = &fleet.prompt {
        let bar = Style::new().bg(Color::Blue).fg(Color::White).add_modifier(Modifier::BOLD);
        let lines = match p {
            Prompt::Confirm { text, buffer, .. } => vec![
                Line::from(vec![Span::styled(" CONFIRM ", bar), Span::raw("  "), Span::raw(text.clone()).bold()]),
                Line::from(vec![Span::styled(" YES ?   ", bar), Span::raw(format!("  {buffer}█   ")), Span::styled("ENTER confirm · ESC cancel", dim)]),
            ],
            Prompt::Input { kind, text, targets } => {
                let who = fleet.names(targets);
                let ask = match kind {
                    InputKind::Takeoff => format!("Takeoff altitude in metres for {who} (blank = 2.5):"),
                    InputKind::Mode => format!("Flight mode for {who} (e.g. posctl, hold, mission, offboard):"),
                    InputKind::Goto => format!("Goto for {who}: [r] fwd right down [yaw] · l N E D [yaw] · g lat lon alt [yaw]   e.g. 10 0 -2"),
                };
                vec![
                    Line::from(vec![Span::styled(" INPUT   ", bar), Span::raw("  "), Span::raw(ask)]),
                    Line::from(vec![Span::styled("   >     ", bar), Span::raw(format!("  {text}█   ")), Span::styled("ENTER submit · ESC cancel", dim)]),
                ]
            }
        };
        frame.render_widget(Paragraph::new(lines), prompt_area);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_port_lists_and_ranges() {
        assert_eq!(parse_ports("14560,14562-14564"), Ok(vec![14560, 14562, 14563, 14564]));
        assert!(parse_ports("").is_err());
        assert!(parse_ports("14564-14560").is_err());
        assert!(parse_ports("abc").is_err());
    }
}
