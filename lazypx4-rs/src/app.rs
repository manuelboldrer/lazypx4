//! The application: UI session state, the main poll/render loop and the
//! keyboard controller. Mirrors `app.py` + `navigation.py`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::DefaultTerminal;

use crate::config::*;
use crate::link::Link;
use crate::mav::modes::{self, ModeOption};
use crate::mav::{calibration, commands, flightlog, params, shell};
use crate::parammeta::ParamMeta;
use crate::state::{self, Shared, State, now};
use crate::ui;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    Dashboard,
    Map,
    ModeSelect,
    Parameters,
    Log,
    FlightLogs,
    Shell,
    PointCloud,
    Camera,
    Host,
    Control,
    Calibration,
    Firmware,
    About,
}

/// Sidebar order: (screen, label, hotkey). The dashboard has no hotkey -
/// every screen's own hotkey toggles back to it.
pub const NAV_ITEMS: &[(Screen, &str, Option<char>)] = &[
    (Screen::Dashboard, "DASHBOARD", None),
    (Screen::Map, "MISSION", Some('n')),
    (Screen::ModeSelect, "MODE", Some('m')),
    (Screen::Parameters, "PARAMETERS", Some('p')),
    (Screen::Log, "EVENT LOG", Some('g')),
    (Screen::FlightLogs, "FLIGHT LOGS", Some('l')),
    (Screen::Shell, "NSH SHELL", Some('t')),
    (Screen::PointCloud, "LIDAR POINTS", Some('v')),
    (Screen::Camera, "CAMERA", Some('w')),
    (Screen::Host, "USB / NETWORK", Some('u')),
    (Screen::Control, "CONTROL", Some('c')),
    (Screen::Calibration, "CALIBRATE", Some('s')),
    (Screen::Firmware, "FLASH FIRMWARE", Some('f')),
    (Screen::About, "ABOUT", Some('?')),
];

impl Screen {
    pub fn title(self) -> &'static str {
        match self {
            Screen::Dashboard => "PX4 UAV · COMMAND / TELEMETRY",
            Screen::ModeSelect => "PX4 FLIGHT MODE SELECT",
            Screen::Log => "PX4 EVENT · WARNING · FAILSAFE LOG",
            Screen::FlightLogs => "PX4 FLIGHT LOGS",
            Screen::Firmware => "PX4 FIRMWARE FLASH",
            Screen::Parameters => "PX4 PARAMETERS",
            Screen::Control => "PX4 CONTROL / SETPOINTS · RC",
            Screen::Camera => "ROS CAMERA PREVIEW",
            Screen::Map => "PX4 MISSION · plan view, up = North",
            Screen::PointCloud => "LIDAR POINT CLOUD OVERVIEW",
            Screen::Calibration => "PX4 SENSOR CALIBRATION",
            Screen::Shell => "PX4 MAVLINK SHELL · NSH",
            Screen::Host => "HOST · USB / NETWORK",
            Screen::About => "ABOUT · LAZYPX4",
        }
    }

    /// Screens whose content is one scrollable page rather than a list with
    /// its own cursor - jk/arrows scroll the main panel there.
    pub fn scrolls_as_page(self) -> bool {
        !matches!(
            self,
            Screen::ModeSelect | Screen::Parameters | Screen::Log | Screen::FlightLogs | Screen::Shell
        )
    }

    fn searchable(self) -> bool {
        matches!(self, Screen::ModeSelect | Screen::Log | Screen::FlightLogs | Screen::Parameters)
    }
}

/// What a "type YES" confirmation runs.
#[derive(Debug, Clone)]
pub enum Action {
    Arm,
    Disarm,
    Hold,
    Reboot,
    Kill,
    SetHome,
    EkfReset,
    Takeoff(f64),
    Land,
    Rtl,
    SetMode(ModeOption),
    SetParam(String, f64),
    Calibrate(&'static str),
    Goto(GotoTarget),
    /// Fly a waypoint queue / coverage path; previewed on the map while the
    /// confirmation is up.
    StartQueue { targets: Vec<crate::state::Target>, mode: String, face_target: bool },
    UploadFence,
    ArmJog,
    FlashFirmware(std::path::PathBuf, String),
}

#[derive(Debug, Clone)]
pub enum GotoTarget {
    /// forward, right, down (m) from the current position and heading.
    Body(f64, f64, f64, Option<f64>),
    /// north, east, down (m) in the local NED frame.
    Local(f64, f64, f64, Option<f64>),
    /// lat, lon, AMSL.
    Global(f64, f64, f64, Option<f64>),
}

/// What a single-line text input feeds.
#[derive(Debug, Clone, Copy)]
pub enum InputKind {
    Takeoff,
    Geofence,
    Goto,
    Kml,
    WpQueue,
    Coverage,
    CameraTopic(usize),
    LidarTopic,
}

pub struct Confirm {
    pub text: String,
    pub buffer: String,
    pub action: Action,
}

pub struct Input {
    pub prompt: String,
    pub buffer: String,
    pub kind: InputKind,
}

/// UI navigation state - only the main thread touches it.
pub struct Session {
    pub screen: Screen,
    pub sidebar_focused: bool,
    pub nav_index: usize,

    /// Page scroll for screens without their own list (clamped by the
    /// renderer, which knows the content height).
    pub main_scroll: usize,
    pub main_scroll_screen: Screen,

    pub mode_index: usize,
    pub log_scroll: usize,
    /// Event-log rows visible last frame - set by the renderer.
    pub log_page: usize,
    pub shell_scroll: isize,
    pub shell_follow: bool,
    pub shell_input: crate::lineedit::LineEdit,

    pub param_index: usize,
    pub param_changed_only: bool,
    pub param_page: usize,
    pub param_edit: Option<(String, String)>,

    pub flight_log_index: usize,

    /// Keyboard jog on the map screen.
    pub jog_armed: bool,
    pub jog_step: f64,
    pub jog_last: f64,

    /// [v] point-cloud view.
    pub cloud_view: crate::ui::sensors::CloudView,
    pub cloud_prev_view: crate::ui::sensors::CloudView,
    pub cloud_range: f64,
    pub cloud_yaw: f64,
    pub cloud_pitch: f64,

    /// [f] flash-firmware lists.
    pub firmware_files: Vec<std::path::PathBuf>,
    pub firmware_index: usize,
    pub firmware_ports: Vec<String>,
    pub firmware_port_index: usize,

    pub confirm: Option<Confirm>,
    pub input: Option<Input>,
    pub search_active: bool,
    pub search_query: String,
}

impl Session {
    fn new() -> Self {
        Session {
            screen: Screen::Dashboard,
            sidebar_focused: false,
            nav_index: 0,
            main_scroll: 0,
            main_scroll_screen: Screen::Dashboard,
            mode_index: 0,
            log_scroll: 0,
            log_page: 20,
            shell_scroll: 0,
            shell_follow: true,
            shell_input: Default::default(),
            param_index: 0,
            param_changed_only: false,
            param_page: 18,
            param_edit: None,
            flight_log_index: 0,
            jog_armed: false,
            jog_step: JOG_STEP_M,
            jog_last: 0.0,
            cloud_view: crate::ui::sensors::CloudView::Top,
            cloud_prev_view: crate::ui::sensors::CloudView::Top,
            cloud_range: 10.0,
            cloud_yaw: 45.0,
            cloud_pitch: 30.0,
            firmware_files: Vec::new(),
            firmware_index: 0,
            firmware_ports: Vec::new(),
            firmware_port_index: 0,
            confirm: None,
            input: None,
            search_active: false,
            search_query: String::new(),
        }
    }
}

pub struct App {
    pub session: Session,
    pub shared: Shared,
    pub link: Arc<Link>,
    pub meta: ParamMeta,
    pub settings: Settings,
    pub ros: crate::ros::Ros,
    pub sys: Arc<Mutex<crate::host::SystemStats>>,
    pub net: Arc<Mutex<crate::host::NetStats>>,
    pub jobs: crate::jobs::Jobs,
    pub(crate) log_rx: Arc<Mutex<Receiver<flightlog::Chunk>>>,
    pub(crate) shutdown: Arc<AtomicBool>,
    ros_thread: Option<std::thread::JoinHandle<()>>,
    last_gcs_heartbeat: f64,
}

/// Key names, as the Python keyboard thread produces them.
fn key_name(ev: KeyEvent) -> Option<String> {
    let ctrl = ev.modifiers.contains(KeyModifiers::CONTROL);
    Some(match ev.code {
        KeyCode::Char(c) if ctrl => format!("CTRL_{}", c.to_ascii_uppercase()),
        KeyCode::Char(c) => c.to_string(),
        KeyCode::Esc => "ESC".into(),
        KeyCode::Enter => "ENTER".into(),
        KeyCode::Backspace => "BACKSPACE".into(),
        KeyCode::Tab => "TAB".into(),
        KeyCode::BackTab => "TAB".into(),
        KeyCode::Up => "UP".into(),
        KeyCode::Down => "DOWN".into(),
        KeyCode::Left => "LEFT".into(),
        KeyCode::Right => "RIGHT".into(),
        KeyCode::PageUp => "PGUP".into(),
        KeyCode::PageDown => "PGDN".into(),
        KeyCode::Home => "HOME".into(),
        KeyCode::End => "END".into(),
        KeyCode::Delete => "DELETE".into(),
        _ => return None,
    })
}

pub(crate) fn is_text(key: &str) -> bool {
    let mut chars = key.chars();
    matches!((chars.next(), chars.next()), (Some(c), None) if !c.is_control())
}

pub(crate) fn normalize_vim_key(key: String) -> String {
    match key.as_str() {
        "j" => "DOWN".into(),
        "k" => "UP".into(),
        "CTRL_F" | "CTRL_D" => "PGDN".into(),
        "CTRL_B" | "CTRL_U" => "PGUP".into(),
        _ => key,
    }
}

const STARTUP_NOTES: &[&str] = &[
    "Console ready",
    "ARM/DISARM are explicit commands",
    "Heartbeat is authoritative for actual ARM state",
    "[T] takeoff  [L] land  [R] return-to-launch  (or pick the mode with [m])",
    "Use [m] MODE for all other PX4 flight-mode changes",
    "Press [p] for PX4 parameters ([/] to search)",
    "Press [c] control/setpoints, [s] sensor calibration",
    "Press [l] for PX4 flight logs, [t] for the NSH shell",
    "Press [n] for the mission map: goto, jog, KML overlay, waypoint queue, fence upload",
    "Press [u] for USB / network, [f] to flash firmware, [v] / [w] for LiDAR / camera",
];

/// Default climb for [T] takeoff, matching PX4's MIS_TAKEOFF_ALT default.
const TAKEOFF_DEFAULT_ALT: f64 = 2.5;

impl App {
    pub fn new(settings: Settings) -> std::io::Result<Self> {
        let link = Arc::new(Link::bind(settings.port)?);
        let shared: Shared = Arc::new(Mutex::new(State::new()));
        let meta = ParamMeta::load(settings.param_defaults_file.as_deref());

        {
            let mut st = state::lock(&shared);
            // Like the Python version, only the --param-defaults file (the
            // firmware actually flashed) defines "changed from default"; the
            // bundled metadata may be from a different PX4 release.
            if let Some(path) = &settings.param_defaults_file {
                if meta.source == *path {
                    st.parameter_defaults = meta.defaults();
                    let n = st.parameter_defaults.len();
                    st.info(format!("Loaded {n} parameter defaults from {path}"));
                } else {
                    st.warn(format!("Could not read parameter defaults file: {path} - using bundled metadata"));
                }
            }
            st.info(format!("Listening for PX4 on udpin:0.0.0.0:{}", settings.port));
            st.info("Waiting for PX4 heartbeat...");
            for note in STARTUP_NOTES {
                st.info(note);
            }
            st.info(format!("Flight-log downloads go to {}", settings.log_dir));
        }

        let (log_tx, log_rx) = std::sync::mpsc::channel();
        let shutdown = Arc::new(AtomicBool::new(false));
        {
            let (link, shared, shutdown) = (link.clone(), shared.clone(), shutdown.clone());
            std::thread::Builder::new()
                .name("MAVLinkThread".into())
                .spawn(move || crate::mav::handlers::receiver_thread(link, shared, log_tx, shutdown))?;
        }

        // Background helpers: host / network monitors, the waypoint queue,
        // and (in a `ros` build) the ROS 2 node.
        let sys = Arc::new(Mutex::new(crate::host::SystemStats::default()));
        let net = Arc::new(Mutex::new(crate::host::NetStats::default()));
        {
            let (sys, disk, sd) = (sys.clone(), settings.disk_path.clone(), shutdown.clone());
            std::thread::Builder::new().name("SysMon".into()).spawn(move || crate::host::sysmon_thread(sys, disk, sd))?;
            let (net, sd) = (net.clone(), shutdown.clone());
            std::thread::Builder::new().name("NetMon".into()).spawn(move || crate::host::netmon_thread(net, sd))?;
            let (link, shared, sd) = (link.clone(), shared.clone(), shutdown.clone());
            std::thread::Builder::new()
                .name("WpQueue".into())
                .spawn(move || crate::mav::guided::wp_queue_thread(link, shared, sd))?;
        }
        let ros = crate::ros::new(&settings);
        let ros_thread = crate::ros::spawn(ros.clone(), shared.clone(), &settings, shutdown.clone());

        let mut app = App {
            session: Session::new(),
            shared,
            link,
            meta,
            settings,
            ros,
            sys,
            net,
            jobs: Default::default(),
            log_rx: Arc::new(Mutex::new(log_rx)),
            shutdown,
            ros_thread,
            last_gcs_heartbeat: 0.0,
        };
        if let Some(path) = app.settings.kml_path.clone() {
            app.load_kml(&path);
        }
        Ok(app)
    }

    pub fn run(mut self, terminal: &mut DefaultTerminal) -> std::io::Result<()> {
        let mut next_frame = Instant::now();
        while !self.shutdown.load(Ordering::Relaxed) {
            let timeout = next_frame.saturating_duration_since(Instant::now()).min(Duration::from_millis(20));
            if event::poll(timeout)? {
                // Drain everything queued so a burst of keys isn't rate
                // limited by the frame period.
                loop {
                    if let Event::Key(ev) = event::read()?
                        && ev.kind != KeyEventKind::Release {
                            // A quick ESC followed by a key arrives as Alt+key.
                            if ev.modifiers.contains(KeyModifiers::ALT) {
                                self.process_key("ESC".into());
                            }
                            if let Some(key) = key_name(ev) {
                                self.process_key(key);
                            }
                        }
                    if !event::poll(Duration::ZERO)? {
                        break;
                    }
                }
            }

            self.tick();

            if Instant::now() >= next_frame {
                terminal.draw(|frame| ui::draw(frame, &mut self))?;
                next_frame = Instant::now() + FRAME_PERIOD;
            }
        }

        // Hand the NSH console back to whatever PX4 had on it.
        if state::lock(&self.shared).shell_active {
            shell::release_shell(&self.link);
        }
        // Let the ROS node see `shutdown` and tear down in order (its spin
        // is 100 ms, so this is quick).
        if let Some(t) = self.ros_thread.take() {
            let _ = t.join();
        }
        Ok(())
    }

    fn quit(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }

    /// Periodic work: GCS heartbeat, stream setup on lock, timeouts, health.
    fn tick(&mut self) {
        let t = now();
        if t - self.last_gcs_heartbeat >= 1.0 && self.link.has_peer() {
            self.last_gcs_heartbeat = t;
            commands::send_gcs_heartbeat(&self.link);
        }

        let mut st = state::lock(&self.shared);

        // First lock (or a heartbeat recovery): announce ourselves and ask
        // for streams. Paced, so it runs on a helper thread.
        if st.vehicle_locked && !st.streams_configured {
            st.streams_configured = true;
            let link = self.link.clone();
            std::thread::spawn(move || {
                commands::send_gcs_heartbeat(&link);
                commands::configure_streams(&link);
                commands::request_estimator_params(&link);
            });
        }

        commands::check_pending_arm(&mut st);
        params::check_parameter_request(&mut st);
        params::check_pending_parameter(&mut st);
        modes::check_available_modes_request(&mut st);
        flightlog::check_flight_log_list(&self.link, &mut st);
        calibration::check_calibration(&mut st);
        crate::mav::guided::check_fence_upload(&mut st);
        update_health(&mut st);
        drop(st);

        // The [P] banner on the map expires like the dashboard's PREARM one.
        let mut ros = crate::host::lock(&self.ros);
        if !ros.gps_feedback.is_empty() && t - ros.gps_feedback_time > GPS_PUBLISH_FEEDBACK_TIMEOUT_S {
            ros.gps_feedback.clear();
        }
    }

    pub(crate) fn configure_streams_async(&self) {
        let link = self.link.clone();
        std::thread::spawn(move || commands::configure_streams(&link));
    }

    // -----------------------------------------------------------------
    // Confirmation / input / search prompts
    // -----------------------------------------------------------------

    pub(crate) fn request_confirmation(&mut self, text: impl Into<String>, action: Action) {
        self.session.confirm = Some(Confirm { text: text.into(), buffer: String::new(), action });
    }

    pub(crate) fn request_input(&mut self, prompt: impl Into<String>, kind: InputKind) {
        self.session.input = Some(Input { prompt: prompt.into(), buffer: String::new(), kind });
    }

    fn process_confirmation(&mut self, key: &str) {
        let Some(confirm) = self.session.confirm.as_mut() else { return };
        match key {
            "ESC" => self.session.confirm = None,
            "BACKSPACE" => {
                confirm.buffer.pop();
            }
            "ENTER" => {
                if confirm.buffer.eq_ignore_ascii_case("YES") {
                    let action = self.session.confirm.take().unwrap().action;
                    self.run_action(action);
                } else {
                    confirm.buffer.clear();
                    state::lock(&self.shared).warn("Type YES to confirm");
                }
            }
            k if is_text(k) => {
                confirm.buffer.push_str(&k.to_uppercase());
                let n = confirm.buffer.chars().count();
                if n > 3 {
                    confirm.buffer = confirm.buffer.chars().skip(n - 3).collect();
                }
            }
            _ => {}
        }
    }

    fn process_input(&mut self, key: &str) {
        let Some(input) = self.session.input.as_mut() else { return };
        match key {
            "ESC" => self.session.input = None,
            "ENTER" => {
                let input = self.session.input.take().unwrap();
                match input.kind {
                    InputKind::Takeoff => self.takeoff_submit(&input.buffer),
                    InputKind::Geofence => self.fence_submit(&input.buffer),
                    InputKind::Goto => self.goto_submit(&input.buffer),
                    InputKind::Kml => self.load_kml(&input.buffer),
                    InputKind::WpQueue => self.wp_queue_submit(&input.buffer),
                    InputKind::Coverage => self.coverage_submit(&input.buffer),
                    InputKind::CameraTopic(slot) => self.camera_topic_submit(slot, &input.buffer),
                    InputKind::LidarTopic => self.lidar_topic_submit(&input.buffer),
                }
            }
            "BACKSPACE" => {
                input.buffer.pop();
            }
            k if is_text(k)
                && input.buffer.chars().count() < 64 => {
                    input.buffer.push_str(k);
                }
            _ => {}
        }
    }

    fn process_search(&mut self, key: &str) {
        match key {
            "ESC" => {
                self.session.search_active = false;
                self.session.search_query.clear();
            }
            "ENTER" => {
                self.session.search_active = false;
                self.search_jump(0);
            }
            "BACKSPACE" => {
                self.session.search_query.pop();
                self.search_jump(0);
            }
            k if is_text(k) => {
                if self.session.search_query.chars().count() < 48 {
                    self.session.search_query.push_str(k);
                }
                self.search_jump(0);
            }
            _ => {}
        }
    }

    /// Rows the "/" search matches against on the active list screen.
    fn search_rows(&self) -> Vec<String> {
        let st = state::lock(&self.shared);
        match self.session.screen {
            Screen::ModeSelect => modes::mode_options(&st).into_iter().map(|o| o.label).collect(),
            Screen::Log => st
                .events
                .iter()
                .map(|e| format!("{} {} {}", e.time, e.level.as_str(), e.message))
                .collect(),
            Screen::FlightLogs => st
                .flight_logs
                .values()
                .map(|e| format!("{:06} {} {}", e.id, e.time_string(), e.size_string()))
                .collect(),
            Screen::Parameters => params::visible_parameters(&st, self.session.param_changed_only)
                .into_iter()
                .map(|p| format!("{} {}", p.name, params::format_value(p)))
                .collect(),
            _ => Vec::new(),
        }
    }

    /// Move the list cursor to a match: 0 = first at/after the cursor
    /// (while typing), >0 next, <0 previous. Wraps around.
    fn search_jump(&mut self, direction: i32) {
        if self.session.search_query.is_empty() {
            return;
        }
        let rows = self.search_rows();
        if rows.is_empty() {
            return;
        }
        let q = self.session.search_query.to_lowercase();
        let matches: Vec<usize> = rows
            .iter()
            .enumerate()
            .filter(|(_, r)| r.to_lowercase().contains(&q))
            .map(|(i, _)| i)
            .collect();
        if matches.is_empty() {
            return;
        }
        let last = rows.len() - 1;
        let cur = match self.session.screen {
            Screen::ModeSelect => self.session.mode_index,
            Screen::Log => self.session.log_scroll,
            Screen::FlightLogs => self.session.flight_log_index,
            Screen::Parameters => self.session.param_index,
            _ => 0,
        }
        .min(last);

        let target = match direction {
            0 => matches.iter().copied().find(|&m| m >= cur).unwrap_or(matches[0]),
            d if d > 0 => matches.iter().copied().find(|&m| m > cur).unwrap_or(matches[0]),
            _ => matches.iter().copied().rev().find(|&m| m < cur).unwrap_or(*matches.last().unwrap()),
        };

        match self.session.screen {
            Screen::ModeSelect => self.session.mode_index = target,
            Screen::Log => {
                self.session.log_scroll = target.min(rows.len().saturating_sub(self.session.log_page))
            }
            Screen::FlightLogs => self.session.flight_log_index = target,
            Screen::Parameters => self.session.param_index = target,
            _ => {}
        }
    }

    // -----------------------------------------------------------------
    // Actions
    // -----------------------------------------------------------------

    fn run_action(&mut self, action: Action) {
        let (link, shared) = (&self.link, &self.shared);
        match action {
            Action::Arm => {
                commands::send_arm(link, shared, true);
            }
            Action::Disarm => {
                commands::send_arm(link, shared, false);
            }
            Action::Hold => {
                commands::send_hold(link, shared);
            }
            Action::Reboot => {
                commands::send_reboot(link, shared);
            }
            Action::Kill => {
                commands::send_kill(link, shared);
            }
            Action::SetHome => {
                commands::send_set_home_current(link, shared);
            }
            Action::Takeoff(alt) => {
                commands::send_takeoff(link, shared, alt);
            }
            Action::Land => {
                commands::send_land(link, shared);
            }
            Action::Rtl => {
                commands::send_rtl(link, shared);
            }
            Action::SetMode(option) => modes::confirm_mode_option(link, shared, &option),
            Action::SetParam(name, value) => {
                params::send_parameter_set(link, &mut state::lock(shared), &name, value);
            }
            Action::Calibrate(kind) => {
                calibration::send_calibration(link, shared, kind);
            }
            Action::Goto(target) => {
                let mut st = state::lock(shared);
                match target {
                    GotoTarget::Body(f, r, d, y) => crate::mav::guided::goto_body(link, &mut st, f, r, d, y, false),
                    GotoTarget::Local(n, e, d, y) => crate::mav::guided::goto_local(link, &mut st, n, e, d, y),
                    GotoTarget::Global(la, lo, a, y) => crate::mav::guided::goto_global(link, &mut st, la, lo, a, y),
                };
            }
            Action::StartQueue { targets, mode, face_target } => {
                crate::mav::guided::start_wp_queue(link, shared, targets, &mode, face_target);
            }
            Action::UploadFence => {
                crate::mav::guided::start_fence_upload(link, &mut state::lock(shared));
            }
            Action::ArmJog => {
                self.session.jog_step = JOG_STEP_M;
                self.session.jog_armed = true;
                state::lock(shared).command("JOG armed - w/s up/down, k/j fwd/back, a/d strafe, h/l yaw, [/] step");
            }
            Action::FlashFirmware(fw, port) => {
                crate::jobs::start_firmware_flash(&self.jobs, shared, &self.settings.tools_dir, &fw, &port);
            }
            Action::EkfReset => {
                if !self.ensure_shell() {
                    state::lock(shared).error("Could not open MAVLink shell for EKF reset");
                    return;
                }
                shell::send_shell_raw(link, b"ekf stop\n");
                shell::send_shell_raw(link, b"ekf start\n");
                state::lock(shared).command("EKF reset: ekf stop / ekf start");
            }
        }
    }

    fn ensure_shell(&self) -> bool {
        let mut st = state::lock(&self.shared);
        if st.shell_active {
            return true;
        }
        if shell::claim_shell(&self.link) {
            st.shell_active = true;
            st.command("MAVLink shell opened (NSH)");
            true
        } else {
            false
        }
    }

    fn open_takeoff_input(&mut self) {
        {
            let mut st = state::lock(&self.shared);
            if !st.armed {
                st.warn("Takeoff: arm first with [a]");
                return;
            }
            if !st.global_pos_valid {
                st.warn("Takeoff needs a GPS / global position");
                return;
            }
        }
        self.request_input(
            format!("Takeoff altitude in metres (blank = {TAKEOFF_DEFAULT_ALT}):"),
            InputKind::Takeoff,
        );
    }

    fn takeoff_submit(&mut self, text: &str) {
        let text = text.trim();
        let altitude = if text.is_empty() {
            TAKEOFF_DEFAULT_ALT
        } else {
            match text.parse::<f64>() {
                Ok(a) if a.is_finite() => a,
                _ => {
                    state::lock(&self.shared).error("Takeoff: altitude must be a number");
                    return;
                }
            }
        };
        if altitude <= 0.0 {
            state::lock(&self.shared).error("Takeoff: altitude must be positive");
            return;
        }
        self.request_confirmation(
            format!("TAKEOFF and climb to {altitude:.1} m? Type YES"),
            Action::Takeoff(altitude),
        );
    }

    /// PX4 has no runtime geofence enable/disable (it answers
    /// MAV_CMD_DO_FENCE_ENABLE UNSUPPORTED) - GF_ACTION is the real switch.
    fn open_fence_input(&mut self) {
        let current = state::lock(&self.shared).param_value("GF_ACTION");
        let Some(current) = current else {
            commands::request_param(&self.link, "GF_ACTION");
            state::lock(&self.shared).warn("Reading GF_ACTION from PX4 - press [G] again in a moment");
            return;
        };
        self.request_input(
            format!(
                "Geofence action - n=none(disabled) w=warning h=hold r=return t=terminate:  (current: {})",
                gf_action_name(current.round() as i64)
            ),
            InputKind::Geofence,
        );
    }

    fn fence_submit(&mut self, text: &str) {
        let Some(action) = gf_action_from_letter(&text.trim().to_lowercase()) else {
            state::lock(&self.shared).error("Geofence: type one of n/w/h/r/t");
            return;
        };
        self.request_confirmation(
            format!("Set geofence action to {} (GF_ACTION={action})? Type YES", gf_action_name(action)),
            Action::SetParam("GF_ACTION".into(), action as f64),
        );
    }

    // -----------------------------------------------------------------
    // Screen openers
    // -----------------------------------------------------------------

    fn open_screen(&mut self, screen: Screen) {
        self.session.screen = screen;
        match screen {
            Screen::Log => {
                let total = state::lock(&self.shared).events.len();
                self.session.log_scroll = total.saturating_sub(self.session.log_page);
            }
            Screen::ModeSelect => {
                let (options, current, custom, have_custom, requested) = {
                    let st = state::lock(&self.shared);
                    (
                        modes::mode_options(&st),
                        st.mode.clone(),
                        st.custom_mode,
                        !st.custom_modes.is_empty(),
                        st.custom_modes_requested_at > 0.0,
                    )
                };
                self.session.mode_index = options
                    .iter()
                    .position(|o| match &o.kind {
                        modes::ModeKind::Legacy(m) => *m == current,
                        _ => o.custom_mode == Some(custom),
                    })
                    .unwrap_or(0);
                // Lazily fetch the full list (custom / external modes) the
                // first time the screen opens.
                if !have_custom && !requested && self.link.has_peer() {
                    modes::request_available_modes(&self.link, &self.shared);
                }
            }
            Screen::Parameters => {
                self.session.param_index = 0;
                self.session.param_edit = None;
                let mut st = state::lock(&self.shared);
                // The EKF2_* params fetched for the dashboard must not
                // suppress the first full load.
                if st.vehicle_locked && !st.parameter_full_list_requested {
                    params::request_parameters(&self.link, &mut st);
                }
            }
            Screen::FlightLogs => {
                let mut st = state::lock(&self.shared);
                if st.flight_logs.is_empty() && st.flight_log_list_requested_at == 0.0 && st.vehicle_locked {
                    let allow = self.settings.allow_log_download_while_armed;
                    flightlog::request_flight_logs(&self.link, &mut st, allow);
                }
            }
            Screen::Shell => {
                self.session.shell_follow = true;
                if !self.ensure_shell() {
                    state::lock(&self.shared).error("Could not open MAVLink shell");
                }
            }
            Screen::Control | Screen::Calibration | Screen::Map => self.configure_streams_async(),
            Screen::Firmware => self.refresh_firmware_lists(),
            _ => {}
        }
    }

    fn opener_for(key: &str) -> Option<Screen> {
        let c = key.chars().next()?;
        if key.len() != 1 {
            return None;
        }
        if c == 'r' {
            return Some(Screen::Control);
        }
        NAV_ITEMS.iter().find(|(_, _, hk)| *hk == Some(c)).map(|(s, _, _)| *s)
    }

    pub(crate) fn focus_sidebar(&mut self) {
        self.session.sidebar_focused = true;
        self.sync_nav_index();
    }

    fn sync_nav_index(&mut self) {
        if let Some(i) = NAV_ITEMS.iter().position(|(s, _, _)| *s == self.session.screen) {
            self.session.nav_index = i;
        }
    }

    /// jk/arrows/page keys scroll a page-style screen. True if consumed.
    pub(crate) fn scroll_main(&mut self, key: &str) -> bool {
        let s = &mut self.session.main_scroll;
        match key {
            "UP" => *s = s.saturating_sub(1),
            "DOWN" => *s += 1,
            "PGUP" => *s = s.saturating_sub(10),
            "PGDN" => *s += 10,
            "HOME" => *s = 0,
            "END" => *s = usize::MAX / 2,
            _ => return false,
        }
        true
    }

    // -----------------------------------------------------------------
    // Top-level key dispatch
    // -----------------------------------------------------------------

    fn process_key(&mut self, key: String) {
        let before = self.session.screen;
        self.dispatch_key(key);
        // A search belongs to the list it was typed on.
        if self.session.screen != before && !self.session.search_active {
            self.session.search_query.clear();
        }
    }

    fn dispatch_key(&mut self, key: String) {
        if self.session.confirm.is_some() || self.session.input.is_some() || self.session.search_active {
            if key == "CTRL_C" {
                self.quit();
            } else if self.session.confirm.is_some() {
                self.process_confirmation(&key);
            } else if self.session.input.is_some() {
                self.process_input(&key);
            } else {
                self.process_search(&key);
            }
            return;
        }

        // A committed shell swallows every key, Tab and letters included.
        if self.session.screen == Screen::Shell && !self.session.sidebar_focused {
            self.handle_shell_key(&key);
            return;
        }

        if key == "CTRL_C" {
            self.quit();
            return;
        }

        if key == "TAB" {
            if self.session.sidebar_focused {
                self.session.sidebar_focused = false;
            } else {
                self.focus_sidebar();
            }
            return;
        }

        if self.session.sidebar_focused {
            self.handle_sidebar_key(&key);
            return;
        }

        let param_editing = self.session.screen == Screen::Parameters && self.session.param_edit.is_some();

        if self.session.screen.searchable() && !param_editing {
            match key.as_str() {
                "/" => {
                    self.session.search_active = true;
                    return;
                }
                "n" => {
                    self.search_jump(1);
                    return;
                }
                "N" => {
                    self.search_jump(-1);
                    return;
                }
                _ => {}
            }
        }

        if key == "q" {
            let busy = {
                let st = state::lock(&self.shared);
                param_editing || st.dl_active || st.cal_active || self.session.jog_armed
            };
            if !busy {
                self.quit();
                return;
            }
        }

        // On the map j/k are jog keys while jog is armed - it normalises
        // them itself once jog is off.
        let key = if param_editing || self.session.screen == Screen::Map { key } else { normalize_vim_key(key) };

        match self.session.screen {
            Screen::Map => self.handle_map_key(key),
            Screen::PointCloud => self.handle_pointcloud_key(&key),
            Screen::Camera => self.handle_camera_key(&key),
            Screen::Host => self.handle_host_key(&key),
            Screen::Firmware => self.handle_firmware_key(&key),
            Screen::Parameters => self.handle_parameter_key(&key),
            Screen::Log => self.handle_log_key(&key),
            Screen::ModeSelect => self.handle_mode_key(&key),
            Screen::FlightLogs => self.handle_flight_log_key(&key),
            Screen::Calibration => self.handle_calibration_key(&key),
            Screen::Dashboard => self.handle_dashboard_key(&key),
            screen => {
                // Control, About and the not-yet-ported screens: own hotkey
                // toggles back, 'r' re-requests streams on Control.
                if NAV_ITEMS.iter().any(|(s, _, hk)| *s == screen && hk.map(|c| c.to_string()) == Some(key.clone()))
                {
                    self.session.screen = Screen::Dashboard;
                } else if key == "ESC" {
                    self.focus_sidebar();
                } else if screen == Screen::Control && key == "r" {
                    self.configure_streams_async();
                    state::lock(&self.shared).info("Re-requested control / setpoint streams");
                } else {
                    self.scroll_main(&key);
                }
            }
        }
    }

    fn handle_sidebar_key(&mut self, key: &str) {
        let n = NAV_ITEMS.len();
        match key {
            "ESC" => self.session.sidebar_focused = false,
            "UP" | "k" => {
                self.session.nav_index = (self.session.nav_index + n - 1) % n;
                self.session.screen = NAV_ITEMS[self.session.nav_index].0;
            }
            "DOWN" | "j" => {
                self.session.nav_index = (self.session.nav_index + 1) % n;
                self.session.screen = NAV_ITEMS[self.session.nav_index].0;
            }
            "ENTER" => {
                self.session.sidebar_focused = false;
                self.open_screen(NAV_ITEMS[self.session.nav_index].0);
            }
            "q" => self.quit(),
            _ => {}
        }
    }

    fn handle_dashboard_key(&mut self, key: &str) {
        if let Some(screen) = Self::opener_for(key) {
            self.open_screen(screen);
            return;
        }
        if self.scroll_main(key) || self.handle_flight_command_key(key) {
            return;
        }
        if key == "ESC" {
            self.focus_sidebar();
        }
    }

    /// Every flight command the dashboard exposes; each goes through its
    /// own "type YES" confirmation.
    pub(crate) fn handle_flight_command_key(&mut self, key: &str) -> bool {
        match key {
            "a" | "d" => {
                let arm = key == "a";
                {
                    let mut st = state::lock(&self.shared);
                    if st.pending_arm.is_some() {
                        st.warn("ARM/DISARM command already pending");
                        return true;
                    }
                    if st.armed == arm {
                        st.warn(if arm {
                            "ARM ignored: vehicle already ARMED"
                        } else {
                            "DISARM ignored: vehicle already DISARMED"
                        });
                        return true;
                    }
                }
                if arm {
                    self.request_confirmation("ARM vehicle? Type YES", Action::Arm);
                } else {
                    self.request_confirmation("DISARM vehicle? Type YES", Action::Disarm);
                }
            }
            "h" => self.request_confirmation("HOLD vehicle? Type YES", Action::Hold),
            "T" => self.open_takeoff_input(),
            "L" => self.request_confirmation("LAND here? Type YES", Action::Land),
            "R" => self.request_confirmation("RETURN TO LAUNCH? Type YES", Action::Rtl),
            "E" => self.request_confirmation("RESET EKF (ekf stop / ekf start)? Type YES", Action::EkfReset),
            "K" => self.request_confirmation(
                "KILL motors NOW? This force-stops motors immediately, even in flight - NOT the same as disarm. Type YES",
                Action::Kill,
            ),
            "H" => {
                if !state::lock(&self.shared).global_pos_valid {
                    state::lock(&self.shared).warn("Set home needs a GPS / global position");
                    return true;
                }
                self.request_confirmation("Set HOME to current position? Type YES", Action::SetHome);
            }
            "G" => self.open_fence_input(),
            _ => return false,
        }
        true
    }

    fn handle_mode_key(&mut self, key: &str) {
        match key {
            "m" => {
                self.session.screen = Screen::Dashboard;
                return;
            }
            "ESC" => {
                self.focus_sidebar();
                return;
            }
            "r" => {
                modes::request_available_modes(&self.link, &self.shared);
                return;
            }
            _ => {}
        }
        let options = modes::mode_options(&state::lock(&self.shared));
        if options.is_empty() {
            return;
        }
        let count = options.len();
        let i = &mut self.session.mode_index;
        *i %= count;
        match key {
            "UP" => *i = (*i + count - 1) % count,
            "DOWN" => *i = (*i + 1) % count,
            "PGUP" => *i = i.saturating_sub(MODE_PAGE_SIZE),
            "PGDN" => *i = (*i + MODE_PAGE_SIZE).min(count - 1),
            "HOME" => *i = 0,
            "END" => *i = count - 1,
            "ENTER" => {
                let option = options[*i].clone();
                self.session.screen = Screen::Dashboard;
                self.request_confirmation(format!("Set mode to {}? Type YES", option.label), Action::SetMode(option));
            }
            _ => {}
        }
    }

    fn handle_log_key(&mut self, key: &str) {
        let total = state::lock(&self.shared).events.len();
        let page = self.session.log_page.max(1);
        let maximum = total.saturating_sub(page);
        let s = &mut self.session.log_scroll;
        match key {
            "g" => self.session.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "c" => {
                let mut st = state::lock(&self.shared);
                st.events.clear();
                st.info("Event log cleared");
            }
            "UP" => *s = s.saturating_sub(1),
            "DOWN" => *s += 1,
            "PGUP" => *s = s.saturating_sub(page),
            "PGDN" => *s += page,
            "HOME" => *s = 0,
            "END" => *s = maximum,
            _ => {}
        }
        self.session.log_scroll = self.session.log_scroll.min(maximum);
    }

    fn handle_calibration_key(&mut self, key: &str) {
        let active = state::lock(&self.shared).cal_active;
        if key == "r" {
            self.configure_streams_async();
            let port = self.settings.port;
            state::lock(&self.shared).info(format!("Re-requested STATUSTEXT / EVENT / streams on port {port}"));
            return;
        }
        if active {
            if key == "ESC" || key == "x" {
                calibration::cancel_calibration(&self.link, &self.shared);
            }
            return;
        }
        let prompt = match key {
            "s" => {
                self.session.screen = Screen::Dashboard;
                return;
            }
            "ESC" => {
                self.focus_sidebar();
                return;
            }
            "g" => ("gyro", "Calibrate GYRO? Keep the vehicle completely still. Type YES"),
            "a" => ("accel", "Calibrate ACCEL? You must place the vehicle on all 6 sides when prompted. Type YES"),
            "l" => ("level", "Calibrate LEVEL HORIZON? Set the vehicle level and still. Type YES"),
            "c" => ("mag", "Calibrate COMPASS? You must rotate the vehicle about all axes when prompted. Type YES"),
            "b" => ("baro", "Calibrate BAROMETER? Keep the vehicle still. Type YES"),
            _ => {
                self.scroll_main(key);
                return;
            }
        };
        self.request_confirmation(prompt.1, Action::Calibrate(prompt.0));
    }

    /// The command line is edited locally and sent on ENTER (see
    /// lineedit.rs for why); PGUP/PGDN scroll the output.
    fn handle_shell_key(&mut self, key: &str) {
        let input = &mut self.session.shell_input;
        match key {
            // The NSH session stays open in the background.
            "ESC" => self.focus_sidebar(),
            "CTRL_C" => {
                input.clear();
                shell::send_shell_raw(&self.link, b"\x03");
            }
            "ENTER" => {
                let line = input.submit();
                shell::send_command(&self.link, &mut state::lock(&self.shared), &line);
                self.session.shell_follow = true;
            }
            "BACKSPACE" => input.backspace(),
            "DELETE" => input.delete(),
            "LEFT" => input.left(),
            "RIGHT" => input.right(),
            "HOME" | "CTRL_A" => input.home(),
            "END" | "CTRL_E" => input.end(),
            "UP" => input.history_prev(),
            "DOWN" => input.history_next(),
            "CTRL_L" => state::lock(&self.shared).shell_lines.clear(),
            // shell_scroll is an offset from the bottom (<= 0).
            "PGUP" => {
                if self.session.shell_follow {
                    self.session.shell_scroll = 0;
                }
                self.session.shell_scroll -= 10;
                self.session.shell_follow = false;
            }
            "PGDN" => {
                self.session.shell_scroll += 10;
                if self.session.shell_scroll >= 0 {
                    self.session.shell_follow = true;
                }
            }
            k if is_text(k) => {
                input.insert(k);
                self.session.shell_follow = true;
            }
            _ => {}
        }
    }

    fn handle_flight_log_key(&mut self, key: &str) {
        let mut st = state::lock(&self.shared);
        let count = st.flight_logs.len();

        if st.dl_active {
            if key == "ESC" || key == "x" {
                flightlog::cancel_download(&mut st);
            }
            return;
        }
        if crate::jobs::running(&self.jobs) && matches!(key, "ESC" | "x") {
            drop(st);
            crate::jobs::cancel(&self.jobs, &self.shared);
            return;
        }

        let i = &mut self.session.flight_log_index;
        match key {
            "l" => self.session.screen = Screen::Dashboard,
            "ESC" => {
                drop(st);
                self.focus_sidebar();
            }
            "r" => {
                let allow = self.settings.allow_log_download_while_armed;
                flightlog::request_flight_logs(&self.link, &mut st, allow);
                self.session.flight_log_index = 0;
            }
            // Both act on the last *completed* download, not the cursor row:
            // the point is to inspect a log already pulled off the vehicle.
            "u" | "a" => {
                let path = (st.dl_status == "COMPLETE" && std::path::Path::new(&st.dl_path).is_file()).then(|| st.dl_path.clone());
                drop(st);
                self.upload_or_analyze(key == "u", path);
            }
            "UP" if count > 0 => *i = (*i + count - 1) % count,
            "DOWN" if count > 0 => *i = (*i + 1) % count,
            "PGUP" => *i = i.saturating_sub(10),
            "PGDN" if count > 0 => *i = (*i + 10).min(count - 1),
            "HOME" => *i = 0,
            "END" if count > 0 => *i = count - 1,
            "ENTER" if count > 0 => {
                let entry = st.flight_logs.values().nth((*i).min(count - 1)).cloned().unwrap();
                drop(st);
                flightlog::start_download(
                    self.link.clone(),
                    self.shared.clone(),
                    self.log_rx.clone(),
                    entry,
                    self.settings.log_dir.clone(),
                    self.settings.allow_log_download_while_armed,
                );
            }
            _ => {}
        }
    }

    fn handle_parameter_key(&mut self, key: &str) {
        if let Some((name, buffer)) = self.session.param_edit.as_mut() {
            match key {
                "ESC" => self.session.param_edit = None,
                "BACKSPACE" => {
                    buffer.pop();
                }
                "ENTER" => {
                    let (name, buffer) = (name.clone(), buffer.clone());
                    self.submit_parameter_edit(&name, &buffer);
                }
                k if is_text(k)
                    && buffer.chars().count() < 64 => {
                        buffer.push_str(k);
                    }
                _ => {}
            }
            return;
        }

        let count = {
            let st = state::lock(&self.shared);
            params::visible_parameters(&st, self.session.param_changed_only).len()
        };
        let page = self.session.param_page.max(1);
        let i = &mut self.session.param_index;
        match key {
            "p" => self.session.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "UP" if count > 0 => *i = (*i + count - 1) % count,
            "DOWN" if count > 0 => *i = (*i + 1) % count,
            "PGUP" => *i = i.saturating_sub(page),
            "PGDN" if count > 0 => *i = (*i + page).min(count - 1),
            "HOME" => *i = 0,
            "END" if count > 0 => *i = count - 1,
            "ENTER" => self.start_parameter_edit(),
            "v" => {
                self.session.param_changed_only = !self.session.param_changed_only;
                self.session.param_index = 0;
            }
            "r" => {
                params::request_parameters(&self.link, &mut state::lock(&self.shared));
                self.session.param_index = 0;
            }
            "b" => self.request_confirmation("REBOOT PX4? Vehicle will restart. Type YES", Action::Reboot),
            _ => {}
        }
    }

    fn start_parameter_edit(&mut self) {
        let mut st = state::lock(&self.shared);
        let visible = params::visible_parameters(&st, self.session.param_changed_only);
        if visible.is_empty() {
            return;
        }
        let p = visible[self.session.param_index.min(visible.len() - 1)];
        if p.pending {
            let name = p.name.clone();
            st.warn(format!("{name} is already waiting for PX4 confirmation"));
            return;
        }
        self.session.param_edit = Some((p.name.clone(), params::format_value(p)));
    }

    fn submit_parameter_edit(&mut self, name: &str, buffer: &str) {
        let parsed = {
            let mut st = state::lock(&self.shared);
            let Some(p) = st.parameters.get(name) else {
                st.error(format!("Parameter disappeared: {name}"));
                self.session.param_edit = None;
                return;
            };
            match params::parse_value(buffer, p) {
                Ok(v) => v,
                Err(e) => {
                    st.error(format!("Invalid value for {name}: {e}"));
                    return;
                }
            }
        };
        self.session.param_edit = None;
        self.request_confirmation(
            format!("Set {name} to {}? Type YES", params::fmt_g(parsed, 12)),
            Action::SetParam(name.to_string(), parsed),
        );
    }
}

/// Latch link / telemetry / battery health and log only the transitions.
fn update_health(st: &mut State) {
    let t = now();

    let heartbeat_stale = st.last_heartbeat > 0.0 && t - st.last_heartbeat > HEARTBEAT_TIMEOUT;
    if heartbeat_stale && !st.heartbeat_timeout_active {
        st.heartbeat_timeout_active = true;
        st.connected = false;
        st.error("PX4 HEARTBEAT TIMEOUT");
    } else if !heartbeat_stale && st.heartbeat_timeout_active {
        st.heartbeat_timeout_active = false;
        st.connected = true;
        // PX4 may have rebooted: re-request streams on the next tick.
        st.streams_configured = false;
        st.info("PX4 HEARTBEAT recovered");
    }

    type Flag = fn(&mut State) -> &mut bool;
    let checks: [(Flag, f64, &str); 4] = [
        (|s| &mut s.position_timeout_active, st.last_position, "Position"),
        (|s| &mut s.gps_timeout_active, st.last_gps, "GPS"),
        (|s| &mut s.ekf_timeout_active, st.last_ekf, "EKF"),
        (|s| &mut s.rc_timeout_active, st.last_rc, "RC"),
    ];
    for (flag, last, name) in checks {
        let bad = last > 0.0 && t - last > TELEMETRY_TIMEOUT;
        let active = flag(st);
        if bad && !*active {
            *active = true;
            st.warn(format!("{name} telemetry timeout"));
        } else if !bad && *active {
            *active = false;
            st.info(format!("{name} telemetry recovered"));
        }
    }

    if !st.last_preflight_fail.is_empty() && t - st.last_preflight_fail_time > PREFLIGHT_FAIL_TIMEOUT {
        st.last_preflight_fail.clear();
        st.info("PREARM check cleared (PX4 stopped reporting it)");
    }

    if st.battery >= 0.0 {
        let battery = st.battery;
        if battery <= BATTERY_CRITICAL {
            if !st.battery_critical_active {
                st.battery_critical_active = true;
                st.failsafe(format!("CRITICAL BATTERY: {battery:.0}%"));
            }
        } else if st.battery_critical_active {
            st.battery_critical_active = false;
            st.info("Battery recovered above critical threshold");
        }

        if battery <= BATTERY_LOW && battery > BATTERY_CRITICAL {
            if !st.battery_low_active {
                st.battery_low_active = true;
                st.warn(format!("LOW BATTERY: {battery:.0}%"));
            }
        } else if st.battery_low_active {
            st.battery_low_active = false;
            st.info("Battery recovered above low threshold");
        }
    }
}
