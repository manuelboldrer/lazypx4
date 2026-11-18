//! [l] FLIGHT LOGS batch work: SPACE marks logs, then ENTER downloads and
//! [u] uploads all of them one after another (a log that isn't on disk yet
//! is downloaded first, then uploaded), [d] deletes their local copies and
//! [E] erases every log on the vehicle (MAVLink's log protocol has no
//! per-log delete, only LOG_ERASE).

use std::collections::{BTreeSet, VecDeque};
use std::path::{Path, PathBuf};

use mavlink::dialects::development::{self as mav, MavMessage};

use crate::app::{Action, App};
use crate::mav::flightlog;
use crate::state::{self, FlightLogEntry, State};

#[derive(Default)]
pub struct LogBatch {
    /// SPACE-marked log ids.
    pub selected: BTreeSet<u16>,
    /// Logs still to download, in order.
    pub downloads: VecDeque<FlightLogEntry>,
    /// The download this batch started and is waiting on.
    pub current: Option<u16>,
    /// Ids to upload once their download completes.
    pub upload_after: BTreeSet<u16>,
    /// .ulg files still to upload.
    pub uploads: VecDeque<PathBuf>,
    /// An upload this batch started is running in the job slot.
    pub uploading: Option<PathBuf>,
}

impl LogBatch {
    pub fn busy(&self) -> bool {
        self.current.is_some() || !self.downloads.is_empty() || self.uploading.is_some() || !self.uploads.is_empty()
    }
}

/// Where a log is (or would be) saved.
pub fn local_path(log_dir: &str, entry: &FlightLogEntry) -> PathBuf {
    Path::new(log_dir).join(flightlog::flight_log_filename(entry))
}

/// The log is already fully on disk.
pub fn downloaded(log_dir: &str, entry: &FlightLogEntry) -> bool {
    std::fs::metadata(local_path(log_dir, entry)).is_ok_and(|m| m.is_file() && (entry.size == 0 || m.len() == entry.size as u64))
}

impl App {
    /// The marked logs (list order), else the cursor row.
    fn log_targets(&self, st: &State) -> Vec<FlightLogEntry> {
        let logs = st.flight_logs_sorted();
        if logs.is_empty() {
            return Vec::new();
        }
        let sel = &self.log_batch.selected;
        if sel.is_empty() {
            return vec![logs[self.session.flight_log_index.min(logs.len() - 1)].clone()];
        }
        logs.into_iter().filter(|e| sel.contains(&e.id)).cloned().collect()
    }

    /// SPACE: mark / unmark the cursor row and step down.
    pub(crate) fn toggle_log_selection(&mut self) {
        let st = state::lock(&self.shared);
        let logs = st.flight_logs_sorted();
        if logs.is_empty() {
            return;
        }
        let i = self.session.flight_log_index.min(logs.len() - 1);
        let id = logs[i].id;
        let count = logs.len();
        drop(st);
        if !self.log_batch.selected.remove(&id) {
            self.log_batch.selected.insert(id);
        }
        self.session.flight_log_index = (i + 1).min(count - 1);
    }

    /// `*`: mark every log, or clear the marks when all are marked.
    pub(crate) fn toggle_all_logs(&mut self) {
        let st = state::lock(&self.shared);
        let all: BTreeSet<u16> = st.flight_logs.keys().copied().collect();
        drop(st);
        self.log_batch.selected = if self.log_batch.selected == all { BTreeSet::new() } else { all };
    }

    /// ENTER: download the marked logs (skipping ones already on disk), or
    /// the cursor row.
    pub(crate) fn download_logs(&mut self) {
        let st = state::lock(&self.shared);
        let targets = self.log_targets(&st);
        drop(st);
        if targets.is_empty() {
            return;
        }
        let dir = self.settings.log_dir.clone();
        let batch = !self.log_batch.selected.is_empty();
        let (have, need): (Vec<_>, Vec<_>) = targets.into_iter().partition(|e| batch && downloaded(&dir, e));
        let mut st = state::lock(&self.shared);
        if !have.is_empty() {
            st.info(format!("{} marked log(s) already downloaded - skipped", have.len()));
        }
        if need.is_empty() {
            return;
        }
        if batch {
            st.command(format!("Queued {} flight-log download(s)", need.len()));
        }
        drop(st);
        self.log_batch.downloads.extend(need);
        self.log_batch_tick();
    }

    /// [u] with marks: upload each marked log, downloading it first when needed.
    pub(crate) fn upload_marked_logs(&mut self) {
        let st = state::lock(&self.shared);
        let targets = self.log_targets(&st);
        drop(st);
        let dir = self.settings.log_dir.clone();
        let (mut ups, mut dls) = (0, 0);
        for e in targets {
            if downloaded(&dir, &e) {
                self.log_batch.uploads.push_back(local_path(&dir, &e));
                ups += 1;
            } else {
                self.log_batch.upload_after.insert(e.id);
                self.log_batch.downloads.push_back(e);
                dls += 1;
            }
        }
        let note = if dls > 0 { format!(" ({dls} to download first)") } else { String::new() };
        state::lock(&self.shared).command(format!("Queued {} Flight Review upload(s){note}", ups + dls));
        self.log_batch_tick();
    }

    /// [d]: delete the local copies of the marked logs (or the cursor row).
    pub(crate) fn confirm_delete_local_logs(&mut self) {
        let st = state::lock(&self.shared);
        let targets = self.log_targets(&st);
        drop(st);
        let dir = &self.settings.log_dir;
        let paths: Vec<PathBuf> = targets.iter().map(|e| local_path(dir, e)).filter(|p| p.is_file()).collect();
        if paths.is_empty() {
            return state::lock(&self.shared).warn("No local copy of the selected log(s) to delete");
        }
        let what = match paths.as_slice() {
            [one] => one.file_name().unwrap_or_default().to_string_lossy().into_owned(),
            many => format!("{} downloaded logs", many.len()),
        };
        self.request_confirmation(format!("DELETE {what} from {dir}? Type YES"), Action::DeleteLocalLogs(paths));
    }

    /// [E]: erase every log on the vehicle.
    pub(crate) fn confirm_erase_vehicle_logs(&mut self) {
        let mut st = state::lock(&self.shared);
        if st.dl_active || self.log_batch.busy() {
            return st.warn("Wait for the flight-log transfers to finish before erasing");
        }
        if st.armed {
            return st.warn("Erasing flight logs is disabled while vehicle is ARMED");
        }
        let n = st.flight_logs.len();
        drop(st);
        let count = if n > 0 { format!("ALL {n} logs") } else { "ALL logs".into() };
        self.request_confirmation(
            format!("ERASE {count} on the vehicle's SD card (LOG_ERASE, cannot be undone)? Type YES"),
            Action::EraseVehicleLogs,
        );
    }

    pub(crate) fn delete_local_logs(&mut self, paths: Vec<PathBuf>) {
        let mut st = state::lock(&self.shared);
        let mut deleted = 0;
        for p in &paths {
            match std::fs::remove_file(p) {
                Ok(()) => deleted += 1,
                Err(e) => st.error(format!("Could not delete {}: {e}", p.display())),
            }
            if st.dl_path == p.display().to_string() {
                st.dl_path.clear();
                st.dl_status.clear();
            }
        }
        st.info(format!("Deleted {deleted} local flight log(s)"));
    }

    pub(crate) fn erase_vehicle_logs(&mut self) {
        let mut st = state::lock(&self.shared);
        if !self.link.has_peer() {
            return st.error("No MAVLink connection");
        }
        let (target_system, target_component) = self.link.target();
        if !self.link.send(&MavMessage::LOG_ERASE(mav::LOG_ERASE_DATA { target_system, target_component })) {
            return st.error("Failed to send LOG_ERASE");
        }
        st.flight_logs.clear();
        st.flight_log_list_complete = false;
        st.command("LOG_ERASE sent - all vehicle logs erased; [r] to reload the list");
        drop(st);
        self.log_batch.selected.clear();
        self.session.flight_log_index = 0;
    }

    /// Cancel the queued batch work (the running transfer is cancelled by the caller).
    pub(crate) fn clear_log_batch(&mut self) {
        let b = &mut self.log_batch;
        let n = b.downloads.len() + b.uploads.len();
        b.downloads.clear();
        b.uploads.clear();
        b.upload_after.clear();
        if n > 0 {
            state::lock(&self.shared).warn(format!("Dropped {n} queued flight-log transfer(s)"));
        }
    }

    /// Start the next queued download / upload once the slot is free.
    pub(crate) fn log_batch_tick(&mut self) {
        let dir = self.settings.log_dir.clone();

        // Download finished?
        if let Some(id) = self.log_batch.current {
            let st = state::lock(&self.shared);
            if !st.dl_active {
                let (status, path) = (st.dl_status.clone(), st.dl_path.clone());
                drop(st);
                self.log_batch.current = None;
                let wanted_upload = self.log_batch.upload_after.remove(&id);
                match status.as_str() {
                    "COMPLETE" if wanted_upload => self.log_batch.uploads.push_back(PathBuf::from(path)),
                    "CANCELLED" => self.clear_log_batch(),
                    _ => {}
                }
            }
        }
        if self.log_batch.current.is_none() && !self.log_batch.downloads.is_empty() && !state::lock(&self.shared).dl_active {
            let entry = self.log_batch.downloads.pop_front().expect("non-empty");
            let id = entry.id;
            if flightlog::start_download(
                self.link.clone(),
                self.shared.clone(),
                self.log_rx.clone(),
                entry,
                dir,
                self.settings.allow_log_download_while_armed,
            ) {
                self.log_batch.current = Some(id);
            } else {
                // Armed, or the log dir can't be made: the rest would fail too.
                self.log_batch.upload_after.remove(&id);
                self.clear_log_batch();
            }
        }

        // Upload finished?
        if self.log_batch.uploading.is_some() {
            let j = crate::host::lock(&self.jobs);
            if !j.active {
                let cancelled = j.status == "CANCELLED";
                drop(j);
                self.log_batch.uploading = None;
                if cancelled {
                    self.clear_log_batch();
                }
            }
        }
        if self.log_batch.uploading.is_none()
            && !crate::jobs::running(&self.jobs)
            && self.rec_jobs.uploading.is_none()
            && let Some(path) = self.log_batch.uploads.pop_front()
            && crate::jobs::start_ulog_upload(&self.jobs, &self.shared, &self.settings.ulog_upload_server, &path, &self.review_links_dir())
        {
            self.log_batch.uploading = Some(path);
        }
    }
}
