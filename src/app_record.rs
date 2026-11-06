//! [e] RECORD: the keys and the once-a-second glue between the recorders
//! (`record.rs`) and the rest of lazypx4 - event log / alerts, the vehicle
//! state for auto markers and system.csv, the PX4 log download into the run
//! and the Flight Review upload.

use std::path::PathBuf;

use crate::app::{Action, App, InputKind, Screen};
use crate::mav::flightlog;
use crate::record::{Note, PollCtx, Veh, hms, tilde};
use crate::state;

const SYSTEM_HEADER: &str = "time,unix_s,cpu_pct,load1,mem_pct,mem_used_gb,disk_free_gb,\
connected,armed,mode,battery_pct,voltage_v,alt_rel_m,lat,lon,groundspeed_ms,gps_fix,sats,rx_msgs_s";

/// What `p` / `u` are waiting for.
#[derive(Default)]
pub struct RecJobs {
    /// `p`: download the newest PX4 log into the run once the list is in.
    pub want_log: bool,
    /// The download we started (dl_path is checked for it).
    pub downloading: bool,
    /// The .ulg `u` is uploading.
    pub uploading: Option<PathBuf>,
    /// `F` ran while recorders were still closing: rewrite the manifest
    /// once they are done.
    pub finish_pending: bool,
    /// `t`: the rosbag topic list, open in place of the markers.
    pub topics: Option<TopicView>,
}

/// `t` in RECORD: every topic `ros2 topic list` (and the bag) knows, each
/// recorded or left out; SPACE toggles an exact exclude.
pub struct TopicView {
    pub topics: Vec<String>,
    pub cursor: usize,
    /// `ros2 topic list` still running.
    pub loading: bool,
    pub error: String,
    rx: Option<std::sync::mpsc::Receiver<Result<Vec<String>, String>>>,
}

impl TopicView {
    fn fetch(script: String) -> std::sync::mpsc::Receiver<Result<Vec<String>, String>> {
        let (tx, rx) = std::sync::mpsc::channel();
        let _ = std::thread::Builder::new().name("RosTopics".into()).spawn(move || {
            let out = std::process::Command::new("bash")
                .arg("-c")
                .arg(&script)
                .stdin(std::process::Stdio::null())
                .output();
            let _ = tx.send(match out {
                Ok(o) if o.status.success() => {
                    Ok(String::from_utf8_lossy(&o.stdout).lines().map(str::trim).filter(|l| l.starts_with('/')).map(String::from).collect())
                }
                Ok(o) => {
                    let err = String::from_utf8_lossy(&o.stderr);
                    let line = err.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("").trim().to_string();
                    Err(if line.is_empty() { format!("ros2 topic list failed ({})", o.status) } else { line })
                }
                Err(e) => Err(format!("could not run ros2 topic list: {e}")),
            });
        });
        rx
    }

    fn merge(&mut self, more: impl IntoIterator<Item = String>) {
        let current = self.topics.get(self.cursor).cloned();
        for t in more {
            if !self.topics.contains(&t) {
                self.topics.push(t);
            }
        }
        self.topics.sort();
        if let Some(c) = current {
            self.cursor = self.topics.iter().position(|t| *t == c).unwrap_or(0);
        }
    }
}

impl App {
    /// This vehicle's namespace: `[record] namespace` / `--namespace`, else
    /// `UAV_NAME` / `PX4_UXRCE_DDS_NS`, else `uav_<system id>` once one
    /// is heard.
    pub(crate) fn rec_namespace(&self) -> Option<String> {
        let configured = self.rec.cfg.namespace.trim().trim_matches('/');
        if !configured.is_empty() {
            return Some(configured.to_string());
        }
        crate::record::env_namespace().or_else(|| {
            let sys = state::lock(&self.shared).target_system;
            (sys != 0).then(|| format!("uav_{sys}"))
        })
    }

    /// The namespace the tlog file is named after.
    pub(crate) fn rec_tlog_ns(&self) -> String {
        self.rec_namespace().unwrap_or_else(|| format!("port_{}", self.settings.port))
    }

    pub(crate) fn rec_notes(&self, notes: Vec<Note>) {
        if notes.is_empty() {
            return;
        }
        let mut st = state::lock(&self.shared);
        for n in notes {
            match n {
                Note::Info(t) => st.info(t),
                Note::Warn(t) => st.warn(t),
                Note::Alert(t) => st.alert(t),
            }
        }
    }

    /// From `tick()`.
    pub(crate) fn rec_tick(&mut self) {
        let (row, armed, mode, alerts, newest) = {
            let st = state::lock(&self.shared);
            let sys = crate::host::lock(&self.sys);
            let now = chrono::Local::now();
            let row = format!(
                "{},{:.3},{:.1},{:.2},{:.1},{:.2},{:.1},{},{},{},{:.0},{:.2},{:.2},{:.7},{:.7},{:.2},{},{},{:.1}",
                now.format("%Y-%m-%d %H:%M:%S%.3f"),
                now.timestamp_millis() as f64 / 1000.0,
                sys.cpu_percent,
                sys.load1,
                sys.mem_percent,
                sys.mem_used_gb,
                sys.disk_free_gb,
                u8::from(st.connected),
                u8::from(st.armed),
                st.mode.replace(',', " "),
                st.battery,
                st.voltage,
                st.alt_relative,
                st.global_lat,
                st.global_lon,
                st.groundspeed,
                st.gps_fix,
                st.gps_sats,
                st.rx_rate,
            );
            (row, st.armed, st.mode.clone(), st.alerts.len(), st.alerts.last().map(|(_, t)| t.clone()))
        };
        let mut notes = Vec::new();
        let vehicles = [Veh { id: 0, link: &self.link, ns: self.rec_tlog_ns() }];
        let ctx = PollCtx { vehicles: &vehicles, system_row: Some((SYSTEM_HEADER, row)) };
        let states = [crate::record::VehState { ns: self.rec_tlog_ns(), armed, mode, alerts, newest_alert: newest }];
        self.rec.poll(&ctx, &states, &mut notes);
        self.rec_notes(notes);
        if self.rec_jobs.finish_pending && !self.rec.stopping() {
            self.rec_jobs.finish_pending = false;
            self.rec_write_manifest(false);
        }
        self.rec_poll_jobs();
        self.rec_poll_topics();
    }

    /// The topics the bag or the config already name, so the list is not
    /// empty while `ros2 topic list` runs (or when ROS is not up).
    fn rec_known_topics(&self) -> Vec<String> {
        let bag = self.rec.recorders.iter().flat_map(|r| r.bag_topics().iter().cloned());
        bag.chain(self.rec.cfg.rosbag_topics.iter().cloned()).chain(self.rec.cfg.rosbag_exclude_topics.iter().cloned()).collect()
    }

    fn rec_open_topics(&mut self) {
        let mut view = TopicView { topics: Vec::new(), cursor: 0, loading: true, error: String::new(), rx: None };
        view.merge(self.rec_known_topics());
        view.rx = Some(TopicView::fetch(self.rec.cfg.topic_list_script()));
        self.rec_jobs.topics = Some(view);
    }

    fn rec_poll_topics(&mut self) {
        let Some(view) = self.rec_jobs.topics.as_mut() else { return };
        let Some(rx) = view.rx.as_ref() else { return };
        match rx.try_recv() {
            Ok(result) => {
                view.rx = None;
                view.loading = false;
                match result {
                    Ok(topics) => view.merge(topics),
                    Err(e) => view.error = e,
                }
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                view.rx = None;
                view.loading = false;
            }
        }
    }

    /// Keys while the topic list is open.
    fn handle_topic_key(&mut self, key: &str) {
        let Some(view) = self.rec_jobs.topics.as_mut() else { return };
        let n = view.topics.len();
        match key {
            "t" | "ESC" => self.rec_jobs.topics = None,
            "UP" => view.cursor = view.cursor.saturating_sub(1),
            "DOWN" => view.cursor = (view.cursor + 1).min(n.saturating_sub(1)),
            "PGUP" => view.cursor = view.cursor.saturating_sub(10),
            "PGDN" => view.cursor = (view.cursor + 10).min(n.saturating_sub(1)),
            "HOME" => view.cursor = 0,
            "END" => view.cursor = n.saturating_sub(1),
            "r" if !view.loading => {
                view.loading = true;
                view.error.clear();
                view.rx = Some(TopicView::fetch(self.rec.cfg.topic_list_script()));
            }
            " " | "ENTER" | "i" => {
                let Some(topic) = view.topics.get(view.cursor).cloned() else { return };
                let cfg = &mut self.rec.cfg;
                let text = match cfg.rosbag_excluded(&topic) {
                    Some(Some(why)) => format!("REC: {topic} is left out by [record] ({why}) - edit lazypx4.toml to record it"),
                    Some(None) => {
                        cfg.rosbag_exclude_topics.retain(|t| *t != topic);
                        format!("REC: rosbag records {topic}")
                    }
                    None => {
                        cfg.rosbag_exclude_topics.push(topic.clone());
                        format!("REC: rosbag leaves out {topic}")
                    }
                };
                let restart = self.rec_rosbag_running();
                let more = if restart { " (from the next rosbag start - ENTER on rosbag twice restarts it)" } else { "" };
                state::lock(&self.shared).info(format!("{text}{more}"));
            }
            "a" => {
                let n = self.rec.cfg.rosbag_exclude_topics.len();
                self.rec.cfg.rosbag_exclude_topics.clear();
                state::lock(&self.shared).info(format!("REC: {n} topic exclude(s) cleared"));
            }
            _ => {}
        }
    }

    fn rec_rosbag_running(&self) -> bool {
        self.rec.recorders.iter().any(|r| r.active() && r.is_rosbag())
    }

    /// `p` / `u` follow-ups.
    fn rec_poll_jobs(&mut self) {
        let Some(dir) = self.rec.run.as_ref().map(|r| r.dir.clone()) else { return };
        if self.rec_jobs.want_log {
            let newest = {
                let st = state::lock(&self.shared);
                (st.flight_log_list_complete || !st.flight_logs.is_empty() && state::now() - st.flight_log_list_last_entry_at > 2.0)
                    .then(|| st.flight_logs.values().max_by_key(|e| e.id).cloned())
            };
            match newest {
                Some(Some(entry)) => {
                    self.rec_jobs.want_log = false;
                    self.rec_jobs.downloading = flightlog::start_download(
                        self.link.clone(),
                        self.shared.clone(),
                        self.log_rx.clone(),
                        entry,
                        dir.join("px4").display().to_string(),
                        self.settings.allow_log_download_while_armed,
                    );
                }
                Some(None) => {
                    self.rec_jobs.want_log = false;
                    state::lock(&self.shared).warn("REC: PX4 reports no flight logs");
                }
                None => {
                    let st = state::lock(&self.shared);
                    if st.flight_log_list_requested_at > 0.0 && state::now() - st.flight_log_list_requested_at > 15.0 {
                        drop(st);
                        self.rec_jobs.want_log = false;
                        state::lock(&self.shared).warn("REC: no flight-log list from PX4");
                    }
                }
            }
        }
        if self.rec_jobs.downloading && !state::lock(&self.shared).dl_active {
            self.rec_jobs.downloading = false;
        }
        if let Some(ulog) = self.rec_jobs.uploading.clone() {
            let j = crate::host::lock(&self.jobs);
            if !j.active {
                let url = (j.status == "SUCCESS" && j.result.starts_with("http")).then(|| j.result.clone());
                drop(j);
                self.rec_jobs.uploading = None;
                if let Some(url) = url {
                    let _ = std::fs::write(dir.join("flight_review_link.txt"), format!("{url}\n{}\n", ulog.display()));
                    if let Some(run) = self.rec.run.as_mut() {
                        run.review_url = Some(url);
                    }
                    self.rec_write_manifest(false);
                }
            }
        }
    }

    pub(crate) fn rec_record_all(&mut self) {
        let mut notes = Vec::new();
        let new = self.rec.run.as_ref().is_none_or(|r| r.finished_wall.is_some());
        self.rec.namespace = self.rec_namespace();
        let vehicles = [Veh { id: 0, link: &self.link, ns: self.rec_tlog_ns() }];
        self.rec.record_all(&vehicles, &mut notes);
        self.rec_notes(notes);
        if new && self.rec.run.is_some() {
            self.rec_save_params("params_start.txt");
        }
    }

    pub(crate) fn rec_stop_all(&mut self) {
        let mut notes = Vec::new();
        let vehicles = [Veh { id: 0, link: &self.link, ns: self.rec_tlog_ns() }];
        self.rec.stop_all(&vehicles, &mut notes);
        self.rec_notes(notes);
    }

    /// `F`: stop, parameters, manifest + summary, then offer a name.
    pub(crate) fn rec_finish(&mut self) {
        let Some(run) = self.rec.run.as_ref() else {
            state::lock(&self.shared).warn("REC: nothing recorded yet");
            return;
        };
        let name = run.name();
        self.rec_stop_all();
        self.rec_save_params("params_end.txt");
        self.rec_write_manifest(true);
        self.rec_jobs.finish_pending = self.rec.stopping();
        self.request_input(format!("Append to the run name {name}_  (ENTER on empty keeps it)"), InputKind::RecRename);
    }

    pub(crate) fn rec_write_manifest(&mut self, announce: bool) {
        let extra = {
            let st = state::lock(&self.shared);
            serde_json::json!({
                "system_id": st.target_system,
                "firmware": st.fw_version_text,
                "firmware_git": st.fw_git_hash,
                "mode_at_end": st.mode,
                "battery_at_end_pct": (st.battery >= 0.0).then_some(st.battery),
                "lazypx4_tlog": self.link.tlog_path().map(|p| p.display().to_string()),
                "udp_port": self.settings.port,
            })
        };
        let path = self.rec.write_manifest(extra);
        let mut st = state::lock(&self.shared);
        match path {
            Some(p) if announce => {
                let more = if self.rec.stopping() { " (rewritten once the recorders still closing are done)" } else { "" };
                st.info(format!("REC manifest.json + summary.md written: {}{more}", tilde(p.parent().unwrap_or(&p))));
            }
            None if announce => st.warn("REC: could not write manifest.json"),
            _ => {}
        }
    }

    /// The vehicle's parameters as they are now (QGC .params), when loaded.
    fn rec_save_params(&self, name: &str) {
        let Some(dir) = self.rec.run.as_ref().map(|r| r.dir.join("px4")) else { return };
        let name = format!("{}_{name}", crate::record::sanitize(&self.rec_namespace().unwrap_or_else(|| "px4".into())));
        let st = state::lock(&self.shared);
        if st.parameters.is_empty() {
            return;
        }
        let _ = crate::mav::params::save_parameters(&st, &dir.join(name), self.link.target());
    }

    fn rec_download_log(&mut self) {
        if self.rec.run.is_none() {
            state::lock(&self.shared).warn("REC: no run yet - [R] starts one");
            return;
        }
        let mut st = state::lock(&self.shared);
        if st.dl_active {
            st.warn("A flight-log download is already active");
            return;
        }
        let allow = self.settings.allow_log_download_while_armed;
        if flightlog::request_flight_logs(&self.link, &mut st, allow) {
            st.info("REC: fetching the PX4 log list, then the newest log into the run");
            drop(st);
            self.rec_jobs.want_log = true;
        }
    }

    /// The newest .ulg in the run, else the last download anywhere.
    fn rec_newest_ulog(&self) -> Option<PathBuf> {
        let dir = self.rec.run.as_ref()?.dir.join("px4");
        let in_run = std::fs::read_dir(dir).ok().and_then(|d| {
            d.filter_map(Result::ok)
                .map(|e| e.path())
                .filter(|p| p.extension().is_some_and(|x| x == "ulg"))
                .max_by_key(|p| p.metadata().and_then(|m| m.modified()).ok())
        });
        in_run.or_else(|| {
            let p = state::lock(&self.shared).dl_path.clone();
            (!p.is_empty()).then(|| PathBuf::from(p))
        })
    }

    fn rec_upload(&mut self) {
        let Some(ulog) = self.rec_newest_ulog() else {
            state::lock(&self.shared).warn("REC: no .ulg to upload - [p] downloads the newest PX4 log first");
            return;
        };
        if crate::jobs::start_ulog_upload(&self.jobs, &self.shared, &self.settings.ulog_upload_server, &ulog) {
            self.rec_jobs.uploading = Some(ulog);
        }
    }

    pub(crate) fn rec_marker(&mut self, label: &str, text: &str) {
        let mut notes = Vec::new();
        self.rec.add_marker(label, text, &mut notes);
        self.rec_notes(notes);
    }

    /// `M` / `:mark`: "label: note", or just a note.
    pub(crate) fn rec_marker_text(&mut self, text: &str) {
        let text = text.trim();
        if text.is_empty() {
            return;
        }
        let (label, note) = match text.split_once(':') {
            Some((l, t)) if !l.trim().is_empty() && !l.contains(' ') => (l.trim().to_string(), t.trim().to_string()),
            _ => ("note".to_string(), text.to_string()),
        };
        self.rec_marker(&label, &note);
    }

    pub(crate) fn rec_rename(&mut self, text: &str) {
        if text.trim().is_empty() {
            return;
        }
        let mut notes = Vec::new();
        self.rec.rename(text, &mut notes);
        self.rec_notes(notes);
        if self.rec.run.as_ref().is_some_and(|r| r.finished_wall.is_some()) {
            self.rec_write_manifest(false);
        }
    }

    /// `q` while recording asks first.
    pub(crate) fn rec_guard_quit(&mut self) -> bool {
        if !self.rec.owns_data() {
            return false;
        }
        self.request_confirmation("Recording is running - stop it (rosbag closes cleanly) and quit? Type YES", Action::QuitRecording);
        true
    }

    pub(crate) fn handle_record_key(&mut self, key: &str) {
        if self.rec_jobs.topics.is_some() {
            self.handle_topic_key(key);
            return;
        }
        let n = self.rec.recorders.len();
        match key {
            "t" => self.rec_open_topics(),
            "e" => self.session.screen = Screen::Dashboard,
            "ESC" => self.focus_sidebar(),
            "UP" => self.rec.cursor = self.rec.cursor.saturating_sub(1),
            "DOWN" => self.rec.cursor = (self.rec.cursor + 1).min(n.saturating_sub(1)),
            "HOME" => self.rec.cursor = 0,
            "END" => self.rec.cursor = n.saturating_sub(1),
            "R" => self.rec_record_all(),
            "S" => self.rec_stop_all(),
            "F" => self.rec_finish(),
            "ENTER" => {
                let mut notes = Vec::new();
                let new = self.rec.run.as_ref().is_none_or(|r| r.finished_wall.is_some());
                self.rec.namespace = self.rec_namespace();
                let vehicles = [Veh { id: 0, link: &self.link, ns: self.rec_tlog_ns() }];
                self.rec.toggle(self.rec.cursor, &vehicles, &mut notes);
                self.rec_notes(notes);
                if new && self.rec.run.is_some() {
                    self.rec_save_params("params_start.txt");
                }
            }
            "i" | " " => {
                if let Some(r) = self.rec.recorders.get_mut(self.rec.cursor) {
                    r.include = !r.include;
                }
            }
            "V" => match self.rec.screen_index() {
                Some(i) => {
                    let mut notes = Vec::new();
                    self.rec.namespace = self.rec_namespace();
                    let vehicles = [Veh { id: 0, link: &self.link, ns: self.rec_tlog_ns() }];
                    self.rec.toggle(i, &vehicles, &mut notes);
                    self.rec_notes(notes);
                }
                None => state::lock(&self.shared).warn("REC: no screen recorder"),
            },
            "M" => self.request_input("Marker: \"label: note\" or just a note", InputKind::RecMarker),
            "E" => {
                let name = self.rec.run.as_ref().map(|r| r.name()).unwrap_or_default();
                self.request_input(format!("Append to the run name {name}_"), InputKind::RecRename);
            }
            "p" => self.rec_download_log(),
            "u" => self.rec_upload(),
            "x" if self.rec_jobs.downloading => {
                flightlog::cancel_download(&mut state::lock(&self.shared));
            }
            "x" if self.rec_jobs.uploading.is_some() => {
                crate::jobs::cancel(&self.jobs, &self.shared);
            }
            "Y" => {
                if let Some(run) = self.rec.run.as_ref() {
                    let path = run.dir.display().to_string();
                    let ok = crate::jobs::copy_to_clipboard(&path);
                    let mut st = state::lock(&self.shared);
                    if ok {
                        st.info(format!("REC path copied: {path}"));
                    } else {
                        st.warn("Could not copy to the clipboard (wl-copy / xclip)");
                    }
                }
            }
            k if k.len() == 1 && k.as_bytes()[0].is_ascii_digit() && k != "0" => {
                let i = (k.as_bytes()[0] - b'1') as usize;
                match self.rec.cfg.markers.get(i).cloned() {
                    Some(label) => self.rec_marker(&label, ""),
                    None => state::lock(&self.shared).warn(format!("REC: no quick marker {k} ([record] markers)")),
                }
            }
            _ => {}
        }
    }

    /// `:rec start|stop|finish|status` and `:mark TEXT`.
    pub(crate) fn rec_command(&mut self, args: &str) -> Result<(), String> {
        match args.trim() {
            "" | "status" => {
                let text = match self.rec.run.as_ref() {
                    Some(r) if self.rec.recording() => format!("REC {} - {} - {}", hms(r.elapsed_s()), tilde(&r.dir), self.rec_active_names()),
                    Some(r) => format!("Not recording - last run {}", tilde(&r.dir)),
                    None => "Not recording ([e] RECORD, or :rec start)".into(),
                };
                state::lock(&self.shared).info(text);
            }
            "start" => self.rec_record_all(),
            "stop" => self.rec_stop_all(),
            "finish" => self.rec_finish(),
            a if a.starts_with("name ") => self.rec_rename(&a["name ".len()..]),
            _ => return Err("usage: :rec start|stop|finish|status|name SUFFIX".into()),
        }
        Ok(())
    }

    fn rec_active_names(&self) -> String {
        let names: Vec<&str> = self.rec.recorders.iter().filter(|r| r.active()).map(|r| r.name.as_str()).collect();
        names.join(", ")
    }
}
