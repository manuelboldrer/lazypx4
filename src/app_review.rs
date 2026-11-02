//! [v] FLIGHT REVIEW: open a .ulg (downloaded logs, or any file through the
//! browser) and page through the Flight Review plots lazyjagg shows, drawn
//! in braille inside the panel. Parsing runs on a worker thread; telemetry
//! keeps flowing meanwhile.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::time::{Instant, SystemTime};

use crate::app::{App, InputKind, Screen};
use crate::plot::{Bounds, Raster};
use crate::review::{self, Page, Store};
use crate::state;

/// Longest array kept element by element (as lazyjagg).
const MAX_ARRAY: usize = 100;

pub struct LogFile {
    pub path: PathBuf,
    pub size: u64,
    pub modified: Option<SystemTime>,
}

pub struct Loading {
    pub path: PathBuf,
    pub progress: Arc<AtomicU64>,
    pub started: Instant,
    rx: Receiver<Result<ReviewLog, String>>,
}

/// A rendered plot, kept until its view or size changes.
pub struct Cached {
    pub key: (u64, u64, u64, u16, u16),
    pub raster: Raster,
    pub bounds: Bounds,
}

pub struct ReviewLog {
    pub path: PathBuf,
    pub store: Store,
    pub pages: Vec<Page>,
    pub page: usize,
    /// Focused plot on the page.
    pub focus: usize,
    pub maximized: bool,
    /// Whole log, and the time window shown.
    pub span: (f64, f64),
    pub view: (f64, f64),
    pub tracker: f64,
    /// Bottom message list instead of the one-line status.
    pub show_messages: bool,
    pub warnings: Vec<String>,
    pub cache: HashMap<(usize, usize), Cached>,
}

impl ReviewLog {
    /// Seconds since the log started, for labels.
    pub fn rel(&self, t: f64) -> f64 {
        t - self.span.0
    }

    pub fn info(&self, key: &str) -> Option<&str> {
        self.store.meta.info.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }

    fn set_view(&mut self, a: f64, b: f64) {
        let (lo, hi) = self.span;
        let width = (b - a).clamp(0.05_f64.min(hi - lo), (hi - lo).max(1e-3));
        let a = a.clamp(lo, (hi - width).max(lo));
        self.view = (a, a + width);
        self.tracker = self.tracker.clamp(self.view.0, self.view.1);
    }

    fn zoom(&mut self, factor: f64) {
        let (a, b) = self.view;
        let width = (b - a) * factor;
        let centre = if self.tracker >= a && self.tracker <= b { self.tracker } else { (a + b) / 2.0 };
        let frac = if b > a { (centre - a) / (b - a) } else { 0.5 };
        self.set_view(centre - width * frac, centre + width * (1.0 - frac));
    }

    fn pan(&mut self, frac: f64) {
        let (a, b) = self.view;
        let d = (b - a) * frac;
        self.set_view(a + d, b + d);
        self.tracker = (self.tracker + d).clamp(self.view.0, self.view.1);
    }

    fn move_tracker(&mut self, frac: f64) {
        let (a, b) = self.view;
        self.tracker = (self.tracker + (b - a) * frac).clamp(self.span.0, self.span.1);
        // Drag the window along when the tracker leaves it.
        if self.tracker > b {
            self.set_view(self.tracker - (b - a), self.tracker);
        } else if self.tracker < a {
            self.set_view(self.tracker, self.tracker + (b - a));
        }
    }

    /// Tracker to the next (`dir` > 0) or previous log message.
    fn jump_message(&mut self, dir: i32) -> Option<String> {
        let msgs = &self.store.meta.messages;
        let eps = 1e-6;
        let m = if dir > 0 {
            msgs.iter().find(|m| m.0 > self.tracker + eps && m.1 != "PARAM")
        } else {
            msgs.iter().rev().find(|m| m.0 < self.tracker - eps && m.1 != "PARAM")
        }?
        .clone();
        self.tracker = m.0.clamp(self.span.0, self.span.1);
        let (a, b) = self.view;
        if self.tracker < a || self.tracker > b {
            let w = b - a;
            self.set_view(self.tracker - w / 2.0, self.tracker + w / 2.0);
        }
        Some(format!("[{}] {}", m.1, m.2))
    }
}

#[derive(Default)]
pub struct ReviewState {
    pub files: Vec<LogFile>,
    pub index: usize,
    pub dir: PathBuf,
    pub loading: Option<Loading>,
    pub log: Option<ReviewLog>,
    pub error: String,
    /// List rows visible last frame (PGUP/PGDN) - set by the renderer.
    pub page_rows: usize,
}

impl ReviewState {
    /// .ulg files in `dir` (and its subfolders one level down), newest first.
    pub fn scan(&mut self, dir: &Path) {
        self.dir = dir.to_path_buf();
        let mut files = Vec::new();
        let walk = |d: &Path, files: &mut Vec<LogFile>, sub: &mut Vec<PathBuf>| {
            let Ok(entries) = std::fs::read_dir(d) else { return };
            for e in entries.flatten() {
                let path = e.path();
                let Ok(meta) = e.metadata() else { continue };
                if meta.is_dir() {
                    sub.push(path);
                } else if path.extension().is_some_and(|x| x.eq_ignore_ascii_case("ulg")) {
                    files.push(LogFile { path, size: meta.len(), modified: meta.modified().ok() });
                }
            }
        };
        let mut subdirs = Vec::new();
        walk(dir, &mut files, &mut subdirs);
        for s in subdirs {
            walk(&s, &mut files, &mut Vec::new());
        }
        files.sort_by(|a, b| b.modified.cmp(&a.modified).then_with(|| a.path.cmp(&b.path)));
        self.files = files;
        self.index = self.index.min(self.files.len().saturating_sub(1));
    }

    /// Hand a finished load over; returns a line for the event log.
    fn poll(&mut self) -> Option<(bool, String)> {
        let loading = self.loading.as_ref()?;
        match loading.rx.try_recv() {
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                let name = file_name(&loading.path);
                self.loading = None;
                self.error = format!("{name}: the loader stopped");
                Some((false, self.error.clone()))
            }
            Ok(Ok(log)) => {
                let secs = loading.started.elapsed().as_secs_f64();
                let text = format!("Flight review: {} loaded ({} pages, {:.1}s)", file_name(&log.path), log.pages.len(), secs);
                self.loading = None;
                self.error.clear();
                self.log = Some(log);
                Some((true, text))
            }
            Ok(Err(e)) => {
                let name = file_name(&loading.path);
                self.loading = None;
                self.error = format!("{name}: {e}");
                Some((false, format!("Flight review: {}", self.error)))
            }
        }
    }
}

pub fn file_name(p: &Path) -> String {
    p.file_name().map_or_else(|| p.display().to_string(), |n| n.to_string_lossy().to_string())
}

/// Parse, derive and lay out a log (on the worker thread).
fn load_log(path: &Path, progress: &AtomicU64) -> Result<ReviewLog, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let loaded = crate::ulog::load(&bytes, MAX_ARRAY, progress)?;
    drop(bytes);
    let mut store = Store::default();
    for s in loaded.series {
        store.insert(s);
    }
    store.meta = loaded.meta;
    if !review::available(&store) {
        return Err("no PX4 vehicle topics in this log".into());
    }
    review::derive(&mut store);
    let pages = review::pages(&mut store);
    if pages.is_empty() {
        return Err("nothing to plot in this log".into());
    }
    let (lo, hi) = store.time_span().ok_or("the log holds no samples")?;
    // Topics last published before logging began shouldn't stretch the view.
    let lo = store.meta.start.filter(|s| *s > lo && *s < hi).unwrap_or(lo);
    let hi = if hi > lo { hi } else { lo + 1.0 };
    Ok(ReviewLog {
        path: path.to_path_buf(),
        store,
        pages,
        page: 0,
        focus: 0,
        maximized: false,
        span: (lo, hi),
        view: (lo, hi),
        tracker: lo,
        show_messages: false,
        warnings: loaded.warnings,
        cache: HashMap::new(),
    })
}

impl App {
    /// Where downloaded logs are, as a path.
    pub(crate) fn review_dir(&self) -> PathBuf {
        let dir = PathBuf::from(&self.settings.log_dir);
        if dir.is_absolute() { dir } else { std::env::current_dir().map(|c| c.join(&dir)).unwrap_or(dir) }
    }

    /// Opening the screen: list the downloaded logs.
    pub(crate) fn review_opened(&mut self) {
        if self.review.log.is_none() {
            let dir = self.review_dir();
            self.review.scan(&dir);
        }
    }

    /// Start parsing `path` in the background.
    pub(crate) fn review_open_path(&mut self, path: &str) {
        let path = path.trim();
        let path = PathBuf::from(match path.strip_prefix("~/") {
            Some(rest) => format!("{}/{rest}", std::env::var("HOME").unwrap_or_default()),
            None => path.to_string(),
        });
        if !path.is_file() {
            return state::lock(&self.shared).warn(format!("Flight review: no such file {}", path.display()));
        }
        if self.review.loading.is_some() {
            return state::lock(&self.shared).warn("Flight review: a log is already loading");
        }
        let progress = Arc::new(AtomicU64::new(0));
        let (tx, rx) = std::sync::mpsc::channel();
        let (p, prog) = (path.clone(), progress.clone());
        std::thread::Builder::new()
            .name("ulog-review".into())
            .spawn(move || {
                let _ = tx.send(load_log(&p, &prog));
            })
            .ok();
        state::lock(&self.shared).info(format!("Flight review: loading {}", file_name(&path)));
        self.review.error.clear();
        self.review.loading = Some(Loading { path, progress, started: Instant::now(), rx });
        self.session.screen = Screen::Review;
    }

    /// The last completed download, straight into the review pane.
    pub(crate) fn review_last_download(&mut self, path: Option<String>) {
        match path {
            Some(p) => self.review_open_path(&p),
            None => {
                self.session.screen = Screen::Review;
                self.review_opened();
            }
        }
    }

    pub(crate) fn review_poll(&mut self) {
        if let Some((ok, text)) = self.review.poll() {
            let mut st = state::lock(&self.shared);
            if ok { st.info(text) } else { st.warn(text) }
        }
    }

    pub(crate) fn handle_review_key(&mut self, key: &str) {
        match key {
            "v" => return self.session.screen = Screen::Dashboard,
            "ESC" => return self.focus_sidebar(),
            "o" => return self.request_file("Open a PX4 .ulg log", InputKind::ReviewLog),
            _ => {}
        }
        if self.review.log.is_some() {
            self.handle_review_plot_key(key);
        } else {
            self.handle_review_list_key(key);
        }
    }

    fn handle_review_list_key(&mut self, key: &str) {
        let r = &mut self.review;
        let count = r.files.len();
        let page = r.page_rows.max(1);
        match key {
            "r" => {
                let dir = self.review_dir();
                self.review.scan(&dir);
            }
            "UP" if count > 0 => r.index = (r.index + count - 1) % count,
            "DOWN" if count > 0 => r.index = (r.index + 1) % count,
            "PGUP" => r.index = r.index.saturating_sub(page),
            "PGDN" if count > 0 => r.index = (r.index + page).min(count - 1),
            "HOME" => r.index = 0,
            "END" if count > 0 => r.index = count - 1,
            "ENTER" if count > 0 => {
                let path = r.files[r.index.min(count - 1)].path.display().to_string();
                self.review_open_path(&path);
            }
            _ => {}
        }
    }

    fn handle_review_plot_key(&mut self, key: &str) {
        let Some(log) = self.review.log.as_mut() else { return };
        let pages = log.pages.len();
        let plots = log.pages[log.page].plots.len();
        let mut said = None;
        match key {
            "]" | "PGDN" => {
                log.page = (log.page + 1) % pages;
                log.focus = 0;
            }
            "[" | "PGUP" => {
                log.page = (log.page + pages - 1) % pages;
                log.focus = 0;
            }
            k if k.len() == 1 && k.as_bytes()[0].is_ascii_digit() && k != "0" => {
                let i = (k.as_bytes()[0] - b'1') as usize;
                if i < pages {
                    log.page = i;
                    log.focus = 0;
                }
            }
            "DOWN" => log.focus = (log.focus + 1) % plots,
            "UP" => log.focus = (log.focus + plots - 1) % plots,
            "ENTER" | "z" => log.maximized = !log.maximized,
            "LEFT" | "h" => log.move_tracker(-0.01),
            "RIGHT" | "l" => log.move_tracker(0.01),
            "H" => log.move_tracker(-0.1),
            "L" => log.move_tracker(0.1),
            "<" | "," => log.pan(-0.25),
            ">" | "." => log.pan(0.25),
            "+" | "=" => log.zoom(0.5),
            "-" | "_" => log.zoom(2.0),
            "0" => {
                let (a, b) = log.span;
                log.set_view(a, b);
            }
            "HOME" => log.tracker = log.view.0,
            "END" => log.tracker = log.view.1,
            "n" => said = Some(log.jump_message(1).unwrap_or_else(|| "no later message".into())),
            "N" => said = Some(log.jump_message(-1).unwrap_or_else(|| "no earlier message".into())),
            "m" => log.show_messages = !log.show_messages,
            "x" => {
                self.review.log = None;
                let dir = self.review_dir();
                self.review.scan(&dir);
            }
            _ => {}
        }
        if let Some(text) = said {
            self.review.error = text;
        } else if key != "n" && key != "N" {
            self.review.error.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn log(span: (f64, f64)) -> ReviewLog {
        ReviewLog {
            path: PathBuf::new(),
            store: Store::default(),
            pages: Vec::new(),
            page: 0,
            focus: 0,
            maximized: false,
            span,
            view: span,
            tracker: span.0,
            show_messages: false,
            warnings: Vec::new(),
            cache: HashMap::new(),
        }
    }

    #[test]
    fn zoom_stays_inside_the_log() {
        let mut l = log((10.0, 110.0));
        l.tracker = 60.0;
        l.zoom(0.5);
        assert_eq!(l.view, (35.0, 85.0));
        l.pan(10.0);
        assert_eq!(l.view, (60.0, 110.0));
        l.zoom(8.0);
        assert_eq!(l.view, (10.0, 110.0));
    }

    #[test]
    fn tracker_drags_the_window() {
        let mut l = log((0.0, 100.0));
        l.set_view(0.0, 10.0);
        l.tracker = 9.5;
        l.move_tracker(0.1);
        assert!((l.tracker - 10.5).abs() < 1e-9);
        assert!((l.view.1 - 10.5).abs() < 1e-9 && (l.view.0 - 0.5).abs() < 1e-9);
    }
}

#[cfg(test)]
pub fn load_for_test(path: &Path) -> Result<ReviewLog, String> {
    load_log(path, &AtomicU64::new(0))
}
