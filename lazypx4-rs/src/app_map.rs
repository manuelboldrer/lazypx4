//! Key handling for the mission map (goto, jog, KML, waypoint queue,
//! coverage, fence upload, satellite, GPS publish), the ROS sensor screens,
//! USB / network, flash firmware and the flight-log tool jobs. Mirrors the
//! corresponding parts of `navigation.py`.

use crate::app::{Action, App, GotoTarget, InputKind, Screen, normalize_vim_key};
use crate::config::*;
use crate::geo;
use crate::host::lock;
use crate::mav::guided;
use crate::state::{self, Target, now};
use crate::ui::sensors::CloudView;

/// Trailing word that turns on "stop and face each target" for a queue.
const FACE_WORDS: &[&str] = &["heading", "hdg", "face", "yaw"];

impl App {
    // -----------------------------------------------------------------
    // Map screen
    // -----------------------------------------------------------------

    pub(crate) fn handle_map_key(&mut self, key: String) {
        match key.as_str() {
            "n" => {
                self.session.jog_armed = false;
                self.session.screen = Screen::Dashboard;
                return;
            }
            // ESC always disarms jog - a safety action first.
            "ESC" => {
                self.session.jog_armed = false;
                self.focus_sidebar();
                return;
            }
            "x" => {
                self.toggle_jog();
                return;
            }
            _ => {}
        }
        if self.session.jog_armed && self.jog_key(&key) {
            return;
        }
        let key = if self.session.jog_armed { key } else { normalize_vim_key(key) };
        if self.scroll_main(&key) {
            return;
        }
        let mut st = state::lock(&self.shared);
        match key.as_str() {
            "i" => {
                drop(st);
                crate::satellite::start(&self.shared, &self.settings.map_dir);
            }
            "g" => {
                drop(st);
                self.open_goto_input();
            }
            "o" => {
                drop(st);
                self.request_input("KML file path (waypoints + fence overlay, visualization only):", InputKind::Kml);
            }
            "O" => {
                drop(st);
                self.open_fence_upload_confirm();
            }
            "W" => {
                drop(st);
                self.open_queue_input(true);
            }
            "C" => {
                drop(st);
                self.open_queue_input(false);
            }
            "P" => {
                drop(st);
                self.request_gps_publish();
            }
            "+" | "=" => st.map_range = (st.map_range / 1.5).clamp(2.0, MAP_RANGE_MAX_M),
            "-" | "_" => st.map_range = (st.map_range * 1.5).clamp(2.0, MAP_RANGE_MAX_M),
            "0" => st.map_range = 30.0,
            "t" => st.map_trail_enabled = !st.map_trail_enabled,
            "f" => st.map_fit_kml = !st.map_fit_kml,
            "V" => st.map_lidar_enabled = !st.map_lidar_enabled,
            "N" => st.map_navpath_enabled = !st.map_navpath_enabled,
            "c" => st.position_trail.clear(),
            "r" => {
                drop(st);
                self.configure_streams_async();
            }
            // Same flight commands as the dashboard (only reached with jog
            // off, so a/d/h mean arm/disarm/hold here, never a nudge).
            _ => {
                drop(st);
                self.handle_flight_command_key(&key);
            }
        }
    }

    fn toggle_jog(&mut self) {
        if self.session.jog_armed {
            self.session.jog_armed = false;
            state::lock(&self.shared).command("JOG disarmed");
            return;
        }
        {
            let mut st = state::lock(&self.shared);
            if !st.armed {
                st.warn("Jog: arm the vehicle first with [a]");
                return;
            }
            if !st.global_pos_valid {
                st.warn("Jog needs a GPS / global position");
                return;
            }
        }
        self.request_confirmation(
            "Arm keyboard JOG? w/s/k/j/a/d/h/l then nudge the vehicle immediately. Type YES",
            Action::ArmJog,
        );
    }

    /// A jog key while jog is armed: one DO_REPOSITION nudge. True if the
    /// key was a jog key (consumed even when rate-limited).
    fn jog_key(&mut self, key: &str) -> bool {
        let s = &mut self.session;
        match key {
            "[" => {
                s.jog_step = (s.jog_step / 2.0).clamp(JOG_STEP_MIN_M, JOG_STEP_MAX_M);
                return true;
            }
            "]" => {
                s.jog_step = (s.jog_step * 2.0).clamp(JOG_STEP_MIN_M, JOG_STEP_MAX_M);
                return true;
            }
            _ => {}
        }
        // (forward, right, down, yaw sign); yaw is a compass heading, so
        // "yaw left" decreases it.
        let (f, r, d, yaw_sign) = match key {
            "k" => (1.0, 0.0, 0.0, 0.0),
            "j" => (-1.0, 0.0, 0.0, 0.0),
            "a" => (0.0, -1.0, 0.0, 0.0),
            "d" => (0.0, 1.0, 0.0, 0.0),
            "w" => (0.0, 0.0, -1.0, 0.0),
            "s" => (0.0, 0.0, 1.0, 0.0),
            "h" => (0.0, 0.0, 0.0, -1.0),
            "l" => (0.0, 0.0, 0.0, 1.0),
            _ => return false,
        };
        let t = now();
        if t - s.jog_last < JOG_MIN_INTERVAL {
            return true;
        }
        s.jog_last = t;
        let step = s.jog_step;
        let (f, r, d) = (f * step, r * step, d * step);
        let mut st = state::lock(&self.shared);
        let yaw = (yaw_sign != 0.0).then(|| (st.yaw + yaw_sign * JOG_YAW_STEP_DEG).rem_euclid(360.0));
        if guided::goto_body(&self.link, &mut st, f, r, d, yaw, true) {
            match yaw {
                Some(y) => st.command(format!("JOG yaw -> {y:.0}")),
                None => st.command(format!("JOG fwd {f:+.1} right {r:+.1} down {d:+.1} m")),
            }
        }
        true
    }

    fn open_goto_input(&mut self) {
        if !state::lock(&self.shared).armed {
            state::lock(&self.shared).warn("Goto: vehicle is not armed");
        }
        self.request_input(
            "Goto: [r] fwd right down [yaw] (default)  |  [l] N E D [yaw]  |  [g] lat lon alt [yaw]  |  \
             [f] fire [alt MSL] [yaw]   e.g.  10 0 -2   or   l 5 -3 -10   or   g 52.218650 6.886870 40   or   f",
            InputKind::Goto,
        );
    }

    pub(crate) fn goto_submit(&mut self, text: &str) {
        let text = text.replace(',', " ");
        let mut tokens: Vec<&str> = text.split_whitespace().collect();
        let err = |app: &Self, m: &str| state::lock(&app.shared).error(m);
        if tokens.is_empty() {
            return err(self, "Goto: expected numbers - see the prompt for the syntax");
        }
        let frame = match tokens[0].to_lowercase().as_str() {
            "r" | "rel" | "relative" => Some("relative"),
            "l" | "local" => Some("local"),
            "g" | "global" => Some("global"),
            "f" | "fire" => Some("fire"),
            _ => None,
        };
        if frame.is_some() {
            tokens.remove(0);
        }
        let Ok(nums) = tokens.iter().map(|t| t.parse::<f64>()).collect::<Result<Vec<f64>, _>>() else {
            return err(self, "Goto: expected numbers after the optional frame letter");
        };
        let frame = frame.unwrap_or("relative");
        if frame == "fire" {
            return self.goto_fire(&nums);
        }
        if nums.len() < 3 {
            return err(self, "Goto: need three numbers (plus an optional yaw)");
        }
        let yaw = nums.get(3).copied();
        let yaw_text = yaw.map(|y| format!("  yaw {y:.0}")).unwrap_or_default();
        let (a, b, c) = (nums[0], nums[1], nums[2]);
        let (prompt, target) = match frame {
            "relative" => (
                format!("GOTO [relative]  fwd {a:+.1}  right {b:+.1}  down {c:+.1} m{yaw_text}   ({:.1} m horizontal). Type YES", a.hypot(b)),
                GotoTarget::Body(a, b, c, yaw),
            ),
            "local" => (
                format!("GOTO [local NED]  N {a:+.1}  E {b:+.1}  D {c:+.1} m{yaw_text}   (absolute, from the local origin). Type YES"),
                GotoTarget::Local(a, b, c, yaw),
            ),
            _ => (format!("GOTO [global]  {a:.7}, {b:.7} @ {c:.1} m MSL{yaw_text}. Type YES"), GotoTarget::Global(a, b, c, yaw)),
        };
        self.request_confirmation(prompt, Action::Goto(target));
    }

    /// `f [alt] [yaw]`: fly over the last /fire_gps_loc fix. The fix's own
    /// altitude is the ground, so the default is the current altitude.
    fn goto_fire(&mut self, nums: &[f64]) {
        let (fire, fire_last) = {
            let r = lock(&self.ros);
            (r.fire, r.fire_last)
        };
        let mut st = state::lock(&self.shared);
        let Some((lat, lon, _)) = fire.filter(|_| fire_last > 0.0) else {
            return st.error(format!("Goto fire: no fix received on {GPS_PUBLISH_TOPIC} yet"));
        };
        if !(lat.is_finite() && lon.is_finite()) {
            return st.error("Goto fire: the last fire fix has no valid lat/lon");
        }
        let alt = match nums.first() {
            Some(a) => *a,
            None if st.global_pos_valid && st.global_alt.is_finite() => st.global_alt,
            None => return st.error("Goto fire: no global position to hold the altitude - give one: f <alt MSL>"),
        };
        let yaw = nums.get(1).copied();
        let distance = if st.global_pos_valid {
            format!("{:.1} m away, ", geo::distance_m(st.global_lat, st.global_lon, lat, lon))
        } else {
            String::new()
        };
        drop(st);
        let prompt = format!(
            "GOTO [fire]  {lat:.7}, {lon:.7} @ {alt:.1} m MSL{}{}   ({distance}fix {:.0}s old). Type YES",
            if nums.is_empty() { " (current alt)" } else { "" },
            yaw.map(|y| format!("  yaw {y:.0}")).unwrap_or_default(),
            now() - fire_last
        );
        self.request_confirmation(prompt, Action::Goto(GotoTarget::Global(lat, lon, alt, yaw)));
    }

    pub(crate) fn load_kml(&mut self, text: &str) {
        let path = text.trim();
        if path.is_empty() {
            return;
        }
        let path = match path.strip_prefix("~/") {
            Some(rest) => format!("{}/{rest}", std::env::var("HOME").unwrap_or_default()),
            None => path.to_string(),
        };
        let mut st = state::lock(&self.shared);
        match geo::parse_kml(&path) {
            Ok(kml) => {
                st.info(format!(
                    "KML loaded: {} - {} waypoint(s), {} fence ring(s)",
                    std::path::Path::new(&path).file_name().unwrap_or_default().to_string_lossy(),
                    kml.waypoints.len(),
                    kml.fence_rings.len()
                ));
                st.kml = Some(kml);
                st.kml_path = path;
                st.kml_error.clear();
            }
            Err(e) => {
                st.kml = None;
                st.error(format!("KML load failed: {e}"));
                st.kml_error = e;
            }
        }
    }

    fn open_fence_upload_confirm(&mut self) {
        let (rings, vertices) = {
            let mut st = state::lock(&self.shared);
            if st.fence_upload_active {
                return st.warn("A geofence upload is already in progress");
            }
            let rings = st.kml.as_ref().map(|k| k.fence_rings.len()).unwrap_or(0);
            if rings == 0 {
                return st.warn("Load a .kml with a fence polygon first ([o])");
            }
            let vertices = guided::fence_items(&st.kml.as_ref().unwrap().fence_rings).len();
            if vertices == 0 {
                return st.warn("Loaded .kml has no polygon with at least 3 vertices");
            }
            (rings, vertices)
        };
        self.request_confirmation(
            format!(
                "UPLOAD geofence to vehicle? {rings} ring(s), {vertices} vertice(s). This REPLACES PX4's active fence \
                 and takes effect immediately. Type YES"
            ),
            Action::UploadFence,
        );
    }

    /// [W] (waypoints) / [C] (coverage): a key press while a queue runs
    /// cancels it; otherwise ask for the parameters.
    fn open_queue_input(&mut self, waypoints: bool) {
        let label = if waypoints { "Waypoint queue" } else { "Coverage" };
        let count = {
            let mut st = state::lock(&self.shared);
            if st.wp_queue_active {
                guided::cancel_wp_queue(&mut st, "cancelled by user");
                return;
            }
            let count = st.kml.as_ref().map(|k| if waypoints { k.waypoints.len() } else { k.fence_rings.len() }).unwrap_or(0);
            if count == 0 {
                return st.error(format!("{label}: load a KML with {} first ([o])", if waypoints { "waypoints" } else { "a polygon" }));
            }
            if !st.armed {
                st.warn(format!("{label}: vehicle is not armed"));
            }
            count
        };
        if waypoints {
            self.request_input(
                format!("Waypoint queue ({count} loaded): <n> | seq | rand <N>  [+ heading]   e.g.  3   seq   rand 8 heading"),
                InputKind::WpQueue,
            );
        } else {
            self.request_input(
                "Area coverage of the KML polygon: <line spacing m> [angle deg, 0 = N-S lines]  [+ heading]   \
                 e.g.  5   or   5 90   (angle omitted = along the longest edge)",
                InputKind::Coverage,
            );
        }
    }

    fn split_face(text: &str) -> (Vec<String>, bool) {
        let mut tokens: Vec<String> = text.split_whitespace().map(str::to_string).collect();
        let face = tokens.last().is_some_and(|t| FACE_WORDS.contains(&t.to_lowercase().as_str()));
        if face {
            tokens.pop();
        }
        (tokens, face)
    }

    fn confirm_queue(&mut self, label: &str, mode: &str, targets: Vec<Target>, face_target: bool, summary: String) {
        let prompt = format!(
            "{label} [{mode}]  {} target(s) at the current altitude{}: {summary}. Type YES",
            targets.len(),
            if face_target { ", facing each target" } else { "" }
        );
        self.request_confirmation(prompt, Action::StartQueue { targets, mode: mode.into(), face_target });
    }

    pub(crate) fn wp_queue_submit(&mut self, text: &str) {
        let (tokens, face) = Self::split_face(text);
        let waypoints = state::lock(&self.shared).kml.as_ref().map(|k| k.waypoints.clone()).unwrap_or_default();
        let err = |app: &Self, m: String| state::lock(&app.shared).error(m);
        let Some(first) = tokens.first().map(|t| t.to_lowercase()) else {
            return err(self, "Waypoint queue: expected a waypoint number, 'seq', or 'rand <N>'".into());
        };
        let result = match first.as_str() {
            "seq" | "sequence" | "all" => guided::build_targets(&waypoints, "sequence", None, None).map(|t| ("sequence", t)),
            "rand" | "random" => match tokens.get(1).and_then(|n| n.parse::<usize>().ok()) {
                Some(n) => guided::build_targets(&waypoints, "random", None, Some(n)).map(|t| ("random", t)),
                None => return err(self, "Waypoint queue: 'rand' needs a count, e.g. rand 5".into()),
            },
            n => match n.parse::<usize>() {
                Ok(i) => guided::build_targets(&waypoints, "single", Some(i), None).map(|t| ("single", t)),
                Err(_) => return err(self, "Waypoint queue: expected a waypoint number, 'seq', or 'rand <N>'".into()),
            },
        };
        match result {
            Ok((mode, targets)) => {
                let mut names: Vec<&str> = targets.iter().take(5).map(|t| t.name.as_str()).collect();
                let more = targets.len().saturating_sub(5);
                let more_text = format!("... (+{more} more)");
                if more > 0 {
                    names.push(&more_text);
                }
                let summary = names.join(", ");
                self.confirm_queue("WAYPOINT QUEUE", mode, targets, face, summary);
            }
            Err(e) => err(self, format!("Waypoint queue: {e}")),
        }
    }

    pub(crate) fn coverage_submit(&mut self, text: &str) {
        let (tokens, face) = Self::split_face(text);
        let ring = state::lock(&self.shared).kml.as_ref().and_then(|k| k.fence_rings.first().cloned()).unwrap_or_default();
        let spacing = tokens.first().and_then(|t| t.parse::<f64>().ok());
        let angle = tokens.get(1).map(|t| t.parse::<f64>());
        let (Some(spacing), None | Some(Ok(_))) = (spacing, &angle) else {
            return state::lock(&self.shared).error("Coverage: expected a spacing in metres, e.g. 5 (optional angle: 5 90)");
        };
        let angle = angle.and_then(Result::ok);
        match geo::coverage_path(&ring, spacing, angle, 1.0) {
            Ok(path) => {
                let n = path.len();
                let targets: Vec<Target> = path
                    .into_iter()
                    .enumerate()
                    .map(|(i, (lat, lon))| Target { lat, lon, name: format!("cover {}/{n}", i + 1) })
                    .collect();
                let length: f64 = targets.windows(2).map(|w| geo::distance_m(w[0].lat, w[0].lon, w[1].lat, w[1].lon)).sum();
                self.confirm_queue("AREA COVERAGE", "coverage", targets, face, format!("{spacing} m lines, {length:.0} m of sweep"));
            }
            Err(e) => state::lock(&self.shared).error(format!("Coverage: {e}")),
        }
    }

    /// [P]: ask the ROS node to publish the vehicle position once.
    fn request_gps_publish(&mut self) {
        let valid = state::lock(&self.shared).global_pos_valid;
        let mut ros = lock(&self.ros);
        let failure = if !ros.available {
            Some(format!("GPS publish: {}", ros.error))
        } else if !valid {
            Some("GPS publish: no global position yet (need a GPS fix)".to_string())
        } else {
            None
        };
        drop(ros);
        match failure {
            Some(msg) => {
                state::lock(&self.shared).warn(&msg);
                crate::ros::set_feedback(&self.ros, msg, false);
            }
            None => {
                crate::ros::set_feedback(&self.ros, format!("publishing to {GPS_PUBLISH_TOPIC} ..."), true);
                ros = lock(&self.ros);
                ros.gps_publish_requested = true;
            }
        }
    }

    // -----------------------------------------------------------------
    // ROS sensor screens
    // -----------------------------------------------------------------

    pub(crate) fn handle_pointcloud_key(&mut self, key: &str) {
        let s = &mut self.session;
        match key {
            "v" => s.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "+" | "=" => s.cloud_range = (s.cloud_range / 1.5).clamp(1.0, 200.0),
            "-" | "_" => s.cloud_range = (s.cloud_range * 1.5).clamp(1.0, 200.0),
            "0" => s.cloud_range = 10.0,
            "1" => s.cloud_view = CloudView::Top,
            "2" => s.cloud_view = CloudView::Front,
            "3" => s.cloud_view = CloudView::Oblique,
            "c" => {
                if s.cloud_view == CloudView::Free {
                    s.cloud_view = s.cloud_prev_view;
                } else {
                    s.cloud_prev_view = s.cloud_view;
                    s.cloud_view = CloudView::Free;
                }
            }
            // hjkl rotate the free camera (j/k arrive normalised).
            "h" | "l" | "UP" | "DOWN" if s.cloud_view == CloudView::Free => match key {
                "h" => s.cloud_yaw = (s.cloud_yaw - LIDAR_CAM_ROTATE_STEP).rem_euclid(360.0),
                "l" => s.cloud_yaw = (s.cloud_yaw + LIDAR_CAM_ROTATE_STEP).rem_euclid(360.0),
                "UP" => s.cloud_pitch = (s.cloud_pitch + LIDAR_CAM_ROTATE_STEP).clamp(-90.0, 90.0),
                _ => s.cloud_pitch = (s.cloud_pitch - LIDAR_CAM_ROTATE_STEP).clamp(-90.0, 90.0),
            },
            "t" => {
                let current = lock(&self.ros).lidar_topic.clone();
                self.request_input(format!("Point-cloud topic (current: {current}):"), InputKind::LidarTopic);
            }
            _ => {
                self.scroll_main(key);
            }
        }
    }

    fn normalize_topic(text: &str) -> String {
        let t = text.trim();
        if t.is_empty() || t.starts_with('/') { t.to_string() } else { format!("/{t}") }
    }

    pub(crate) fn lidar_topic_submit(&mut self, text: &str) {
        let topic = Self::normalize_topic(text);
        if topic.is_empty() {
            return;
        }
        lock(&self.ros).lidar_topic = topic.clone();
        state::lock(&self.shared).command(format!("Point-cloud topic changed to {topic}"));
    }

    pub(crate) fn camera_topic_submit(&mut self, slot: usize, text: &str) {
        let topic = Self::normalize_topic(text);
        lock(&self.ros).cameras[slot].topic = topic.clone();
        state::lock(&self.shared).command(format!(
            "Camera {} topic changed to {}",
            slot + 1,
            if topic.is_empty() { "(none)" } else { &topic }
        ));
    }

    pub(crate) fn handle_camera_key(&mut self, key: &str) {
        match key {
            "w" => self.session.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "1" | "2" => {
                let slot = if key == "1" { 0 } else { 1 };
                let current = lock(&self.ros).cameras[slot].topic.clone();
                self.request_input(
                    format!(
                        "Camera {} topic (current: {}, blank to clear):",
                        slot + 1,
                        if current.is_empty() { "unset" } else { &current }
                    ),
                    InputKind::CameraTopic(slot),
                );
            }
            "b" => {
                let mut r = lock(&self.ros);
                r.camera_low_bw = !r.camera_low_bw;
            }
            _ => {
                self.scroll_main(key);
            }
        }
    }

    // -----------------------------------------------------------------
    // Host / firmware / flight-log jobs
    // -----------------------------------------------------------------

    pub(crate) fn handle_host_key(&mut self, key: &str) {
        match key {
            "u" => self.session.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "i" => {
                let mut st = state::lock(&self.shared);
                if crate::host::run_speedtest_async(&self.net) {
                    st.command("Speed test started (download/upload/ping to a public server)");
                } else {
                    st.warn("Speed test already running");
                }
            }
            _ => {
                self.scroll_main(key);
            }
        }
    }

    pub(crate) fn refresh_firmware_lists(&mut self) {
        let s = &mut self.session;
        s.firmware_files = crate::jobs::discover_firmware_files(&self.settings.firmware_dir);
        s.firmware_ports = crate::jobs::discover_serial_ports();
        s.firmware_index = s.firmware_index.min(s.firmware_files.len().saturating_sub(1));
        s.firmware_port_index = s.firmware_port_index.min(s.firmware_ports.len().saturating_sub(1));
    }

    pub(crate) fn handle_firmware_key(&mut self, key: &str) {
        if crate::jobs::running(&self.jobs) {
            if key == "ESC" {
                crate::jobs::cancel(&self.jobs, &self.shared);
            }
            return;
        }
        let s = &mut self.session;
        let (nf, np) = (s.firmware_files.len(), s.firmware_ports.len());
        match key {
            "f" => s.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "r" => self.refresh_firmware_lists(),
            "UP" if nf > 0 => s.firmware_index = (s.firmware_index + nf - 1) % nf,
            "DOWN" if nf > 0 => s.firmware_index = (s.firmware_index + 1) % nf,
            "LEFT" if np > 0 => s.firmware_port_index = (s.firmware_port_index + np - 1) % np,
            "RIGHT" if np > 0 => s.firmware_port_index = (s.firmware_port_index + 1) % np,
            "ENTER" => {
                let mut st = state::lock(&self.shared);
                if nf == 0 {
                    return st.warn(format!("No .px4 firmware files found in {}", self.settings.firmware_dir));
                }
                if np == 0 {
                    return st.warn("No serial ports detected - plug in the flight controller via USB");
                }
                if st.armed {
                    return st.warn("Refusing to flash firmware while the vehicle is ARMED");
                }
                drop(st);
                let fw = s.firmware_files[s.firmware_index.min(nf - 1)].clone();
                let port = s.firmware_ports[s.firmware_port_index.min(np - 1)].clone();
                let name = fw.file_name().unwrap_or_default().to_string_lossy().into_owned();
                self.request_confirmation(
                    format!("FLASH {name} to {port}? This reboots the flight controller. Type YES"),
                    Action::FlashFirmware(fw, port),
                );
            }
            _ => {}
        }
    }

    /// [u] upload / [a] EKF check on the last completed download.
    pub(crate) fn upload_or_analyze(&mut self, upload: bool, path: Option<String>) {
        if crate::jobs::running(&self.jobs) {
            return state::lock(&self.shared).warn("A background job is already running");
        }
        let Some(path) = path else {
            let what = if upload { "press u to upload it" } else { "press a to run the EKF check" };
            return state::lock(&self.shared).warn(format!("Download a flight log first (ENTER), then {what}"));
        };
        let path = std::path::Path::new(&path);
        let tools = &self.settings.tools_dir;
        if upload {
            crate::jobs::start_ulog_upload(&self.jobs, &self.shared, tools, &self.settings.ulog_upload_server, path);
        } else {
            crate::jobs::start_ecl_ekf_check(&self.jobs, &self.shared, tools, path);
        }
    }
}
