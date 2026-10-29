//! Ranger-style file browser for the "load a file" prompts ([o] .kml /
//! .plan, [l] / [D] parameter files): hjkl to walk the tree, `/` to
//! filter, a preview of the file under the cursor. TAB drops back to the
//! typed path prompt, pre-filled with the current directory.

use std::path::{Path, PathBuf};

use crate::app::InputKind;

pub struct Entry {
    pub name: String,
    pub dir: bool,
}

pub struct FilePicker {
    pub title: String,
    pub kind: InputKind,
    pub dir: PathBuf,
    pub entries: Vec<Entry>,
    pub index: usize,
    pub filter: String,
    pub filtering: bool,
    pub show_hidden: bool,
    /// Lower-case extensions listed; empty or `all_files` lists everything.
    pub exts: &'static [&'static str],
    pub all_files: bool,
    pub error: String,
}

pub enum Outcome {
    Stay,
    Cancel,
    Pick(PathBuf),
    /// Switch to the typed prompt, starting from this text.
    Type(String),
}

impl FilePicker {
    /// Opens on `start` (falls back to its parent, then the cwd), with the
    /// cursor on `select` if it is listed.
    pub fn new(title: impl Into<String>, kind: InputKind, exts: &'static [&'static str], start: &Path, select: Option<&str>) -> Self {
        let dir = [Some(start), start.parent()]
            .into_iter()
            .flatten()
            .find(|d| d.is_dir())
            .map(Path::to_path_buf)
            .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")));
        let mut p = FilePicker {
            title: title.into(),
            kind,
            dir: std::fs::canonicalize(&dir).unwrap_or(dir),
            entries: Vec::new(),
            index: 0,
            filter: String::new(),
            filtering: false,
            show_hidden: false,
            exts,
            all_files: false,
            error: String::new(),
        };
        p.reload(select);
        p
    }

    fn listed(&self, name: &str, dir: bool) -> bool {
        if name.starts_with('.') && !self.show_hidden {
            return false;
        }
        if !self.filter.is_empty() && !name.to_lowercase().contains(&self.filter.to_lowercase()) {
            return false;
        }
        dir || self.all_files || self.exts.is_empty() || {
            let lower = name.to_lowercase();
            self.exts.iter().any(|e| lower.ends_with(&format!(".{e}")))
        }
    }

    /// Re-read `dir`: directories first, then files, each sorted by name.
    pub fn reload(&mut self, select: Option<&str>) {
        self.entries.clear();
        self.error.clear();
        match std::fs::read_dir(&self.dir) {
            Ok(rd) => {
                for e in rd.flatten() {
                    let name = e.file_name().to_string_lossy().to_string();
                    // Follows symlinks, so a link to a directory walks like one.
                    let dir = e.path().is_dir();
                    if self.listed(&name, dir) {
                        self.entries.push(Entry { name, dir });
                    }
                }
            }
            Err(e) => self.error = e.to_string(),
        }
        self.entries.sort_by(|a, b| b.dir.cmp(&a.dir).then_with(|| a.name.to_lowercase().cmp(&b.name.to_lowercase())));
        self.index = select
            .and_then(|s| self.entries.iter().position(|e| e.name == s))
            .unwrap_or(0);
    }

    pub fn current(&self) -> Option<&Entry> {
        self.entries.get(self.index)
    }

    pub fn current_path(&self) -> Option<PathBuf> {
        self.current().map(|e| self.dir.join(&e.name))
    }

    fn cd(&mut self, dir: PathBuf, select: Option<String>) {
        self.dir = dir;
        self.filter.clear();
        self.filtering = false;
        self.reload(select.as_deref());
    }

    fn parent(&mut self) {
        if let Some(parent) = self.dir.parent().map(Path::to_path_buf) {
            let from = self.dir.file_name().map(|n| n.to_string_lossy().to_string());
            self.cd(parent, from);
        }
    }

    fn open(&mut self) -> Outcome {
        let Some(path) = self.current_path() else { return Outcome::Stay };
        if self.current().is_some_and(|e| e.dir) {
            self.cd(path, None);
            Outcome::Stay
        } else {
            Outcome::Pick(path)
        }
    }

    fn step(&mut self, delta: isize) {
        let n = self.entries.len();
        if n > 0 {
            self.index = (self.index as isize + delta).clamp(0, n as isize - 1) as usize;
        }
    }

    /// `key` as App::dispatch_key names it; `page` = rows visible.
    pub fn key(&mut self, key: &str, page: usize) -> Outcome {
        if self.filtering {
            match key {
                "ESC" => {
                    self.filter.clear();
                    self.filtering = false;
                    self.reload(None);
                }
                "ENTER" => {
                    self.filtering = false;
                    // A single match is what you were looking for.
                    if self.entries.len() == 1 {
                        return self.open();
                    }
                }
                "BACKSPACE" => {
                    if self.filter.pop().is_none() {
                        self.filtering = false;
                    }
                    self.reload(None);
                }
                "UP" | "DOWN" => self.step(if key == "UP" { -1 } else { 1 }),
                k if crate::app::is_text(k) && self.filter.len() < 64 => {
                    self.filter.push_str(k);
                    self.reload(None);
                }
                _ => {}
            }
            return Outcome::Stay;
        }
        let page = page.max(1) as isize;
        match key {
            "ESC" | "q" => return Outcome::Cancel,
            "j" | "DOWN" => self.step(1),
            "k" | "UP" => self.step(-1),
            "PGDN" | "CTRL_D" | "CTRL_F" => self.step(page),
            "PGUP" | "CTRL_U" | "CTRL_B" => self.step(-page),
            "g" | "HOME" => self.index = 0,
            "G" | "END" => self.index = self.entries.len().saturating_sub(1),
            "h" | "LEFT" | "BACKSPACE" => self.parent(),
            "l" | "RIGHT" | "ENTER" => return self.open(),
            "~" => self.cd(PathBuf::from(std::env::var("HOME").unwrap_or_else(|_| "/".into())), None),
            "/" => self.filtering = true,
            "." => {
                self.show_hidden = !self.show_hidden;
                let keep = self.current().map(|e| e.name.clone());
                self.reload(keep.as_deref());
            }
            "a" => {
                self.all_files = !self.all_files;
                let keep = self.current().map(|e| e.name.clone());
                self.reload(keep.as_deref());
            }
            "TAB" | ":" => return Outcome::Type(format!("{}/", self.dir.display())),
            _ => {}
        }
        Outcome::Stay
    }

    /// The first `rows` lines of what the cursor is on: a directory's
    /// listing, or a text file's head.
    pub fn preview(&self, rows: usize) -> Vec<String> {
        let Some(path) = self.current_path() else { return Vec::new() };
        if path.is_dir() {
            let Ok(rd) = std::fs::read_dir(&path) else { return vec!["(unreadable)".into()] };
            let mut names: Vec<String> = rd
                .flatten()
                .map(|e| {
                    let n = e.file_name().to_string_lossy().to_string();
                    if e.path().is_dir() { format!("{n}/") } else { n }
                })
                .filter(|n| self.show_hidden || !n.starts_with('.'))
                .collect();
            names.sort_by_key(|n| (!n.ends_with('/'), n.to_lowercase()));
            if names.is_empty() {
                names.push("(empty)".into());
            }
            names.truncate(rows);
            return names;
        }
        // Bounded read: a stray multi-GB log must not stall the UI.
        use std::io::Read;
        let mut buf = Vec::new();
        let read = std::fs::File::open(&path).and_then(|f| f.take(16 * 1024).read_to_end(&mut buf));
        if let Err(e) = read {
            return vec![format!("({e})")];
        }
        if buf.contains(&0) {
            return vec![format!("(binary, {})", human_size(path.metadata().map(|m| m.len()).unwrap_or(0)))];
        }
        String::from_utf8_lossy(&buf).lines().take(rows).map(|l| l.replace('\t', "    ")).collect()
    }
}

pub fn human_size(bytes: u64) -> String {
    match bytes {
        b if b < 1024 => format!("{b} B"),
        b if b < 1024 * 1024 => format!("{:.1} KB", b as f64 / 1024.0),
        b => format!("{:.1} MB", b as f64 / (1024.0 * 1024.0)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree(tag: &str) -> PathBuf {
        let root = std::env::temp_dir().join(format!("lazypx4_picker_{tag}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(root.join("sub")).unwrap();
        std::fs::create_dir_all(root.join(".hidden")).unwrap();
        for f in ["b.params", "a.kml", "notes.txt", "sub/x.params"] {
            std::fs::write(root.join(f), "SYS_AUTOSTART 4001\n").unwrap();
        }
        root
    }

    fn names(p: &FilePicker) -> Vec<&str> {
        p.entries.iter().map(|e| e.name.as_str()).collect()
    }

    #[test]
    fn lists_dirs_first_and_filters_by_extension() {
        let root = tree("lists_di");
        let mut p = FilePicker::new("t", InputKind::ParamLoad, &["params"], &root, None);
        assert_eq!(names(&p), ["sub", "b.params"]);
        p.key("a", 10);
        assert_eq!(names(&p), ["sub", "a.kml", "b.params", "notes.txt"]);
        p.key(".", 10);
        assert_eq!(names(&p), [".hidden", "sub", "a.kml", "b.params", "notes.txt"]);
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn walks_into_and_out_of_directories() {
        let root = tree("walks_in");
        let mut p = FilePicker::new("t", InputKind::ParamLoad, &["params"], &root, None);
        assert!(matches!(p.key("l", 10), Outcome::Stay));
        assert_eq!(names(&p), ["x.params"]);
        assert!(matches!(p.key("ENTER", 10), Outcome::Pick(path) if path.ends_with("sub/x.params")));
        p.key("h", 10);
        // Back up lands on the directory we came from.
        assert_eq!(p.current().unwrap().name, "sub");
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn filter_narrows_and_a_single_match_opens() {
        let root = tree("filter_n");
        let mut p = FilePicker::new("t", InputKind::Kml, &["kml", "plan"], &root, None);
        for k in ["/", "A", "."] {
            p.key(k, 10);
        }
        assert_eq!(names(&p), ["a.kml"]);
        assert!(matches!(p.key("ENTER", 10), Outcome::Pick(path) if path.ends_with("a.kml")));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn starts_on_a_files_directory_and_selects_it() {
        let root = tree("starts_o");
        let p = FilePicker::new("t", InputKind::ParamLoad, &["params"], &root.join("b.params"), Some("b.params"));
        assert_eq!(p.current().unwrap().name, "b.params");
        let _ = std::fs::remove_dir_all(root);
    }
}
