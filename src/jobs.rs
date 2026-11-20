//! Background subprocess jobs: firmware flashing (`px_uploader.py`), the
//! .ulog upload to Flight Review v2 (`curl`) and the ecl_ekf health check
//! (`process_logdata_ekf.py`). Mirrors `jobs.py` + `pxtools.py`.
//!
//! Flashing and the EKF check stay real PX4 tool scripts run under Python - they're manual,
//! occasional actions, and reimplementing a bootloader protocol or the
//! ecl_ekf analysis buys nothing. One job slot is shared by all three.

use std::collections::VecDeque;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};

use crate::host::lock;
use crate::state::{self, Shared};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobKind {
    Firmware,
    UlogUpload,
    EclEkf,
}

#[derive(Debug, Default)]
pub struct JobState {
    pub kind: Option<JobKind>,
    pub active: bool,
    pub cancel: bool,
    pub status: String,
    pub error: String,
    pub result: String,
    pub lines: VecDeque<String>,
    /// Running process, for cancel (the worker owns the Child itself).
    pid: Option<u32>,
}

pub type Jobs = Arc<Mutex<JobState>>;

/// Called with the job's full output once it exited 0; its Ok value becomes
/// the result line, an Err fails the job.
type OnSuccess = Box<dyn FnOnce(&str, &Shared) -> Result<String, String> + Send>;

pub fn running(jobs: &Jobs) -> bool {
    lock(jobs).active
}

/// The interpreter for a Tools/ script: an active virtualenv, the repo's
/// `.venv` next to the tools directory (where `install/install.sh` puts
/// `Tools/requirements.txt`), else `python3` from PATH.
fn tool_python(tools_dir: &str) -> PathBuf {
    if let Ok(venv) = std::env::var("VIRTUAL_ENV") {
        let p = Path::new(&venv).join("bin/python");
        if p.exists() {
            return p;
        }
    }
    let repo_venv = Path::new(tools_dir).join("../.venv/bin/python");
    if repo_venv.exists() {
        return repo_venv;
    }
    PathBuf::from("python3")
}

fn start(jobs: &Jobs, shared: &Shared, kind: JobKind, mut cmd: Command, label: String, on_success: Option<OnSuccess>) -> bool {
    {
        let mut j = lock(jobs);
        if j.active {
            state::lock(shared).warn(format!("{label}: another background job is already running"));
            return false;
        }
        *j = JobState { kind: Some(kind), active: true, status: "RUNNING".into(), ..Default::default() };
    }

    let child = match cmd.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn() {
        Ok(c) => c,
        Err(e) => {
            let mut j = lock(jobs);
            j.active = false;
            j.status = "ERROR".into();
            j.error = e.to_string();
            state::lock(shared).error(format!("{label} failed to start: {e}"));
            return false;
        }
    };
    lock(jobs).pid = Some(child.id());
    state::lock(shared).command(format!("{label} started"));

    let (jobs, shared) = (jobs.clone(), shared.clone());
    std::thread::spawn(move || run(jobs, shared, child, label, on_success));
    true
}

fn run(jobs: Jobs, shared: Shared, mut child: Child, label: String, on_success: Option<OnSuccess>) {
    let (stdout, stderr) = (child.stdout.take(), child.stderr.take());
    // stderr is merged line-by-line alongside stdout, like STDOUT=STDERR.
    let err_jobs = jobs.clone();
    let err_thread = stderr.map(|s| std::thread::spawn(move || pump(s, &err_jobs)));
    let mut output = stdout.map(|s| pump(s, &jobs)).unwrap_or_default();
    if let Some(t) = err_thread {
        output.extend(t.join().unwrap_or_default());
    }
    let status = child.wait();

    let finish = |status: &str, error: String, result: String| {
        let mut j = lock(&jobs);
        j.active = false;
        j.pid = None;
        j.status = status.into();
        j.error = error;
        j.result = result;
    };

    if lock(&jobs).cancel {
        finish("CANCELLED", String::new(), String::new());
        state::lock(&shared).warn(format!("{label} cancelled"));
        return;
    }
    match status {
        Ok(s) if s.success() => {}
        other => {
            let error = output.last().cloned().unwrap_or_else(|| match other {
                Ok(s) => format!("exit code {}", s.code().unwrap_or(-1)),
                Err(e) => e.to_string(),
            });
            state::lock(&shared).error(format!("{label} failed: {error}"));
            finish("ERROR", error, String::new());
            return;
        }
    }
    let full = output.join("\n");
    let result = match on_success {
        None => Ok(String::new()),
        Some(f) => f(&full, &shared),
    };
    match result {
        Ok(result) => {
            let suffix = if result.is_empty() { String::new() } else { format!(" - {result}") };
            state::lock(&shared).info(format!("{label}: done{suffix}"));
            finish("SUCCESS", String::new(), result);
        }
        Err(e) => {
            state::lock(&shared).error(format!("{label}: {e}"));
            finish("ERROR", e, String::new());
        }
    }
}

/// Read a pipe to EOF, splitting on CR as well as LF (px_uploader draws its
/// progress bar with CR), appending each line to the job's live tail.
fn pump(mut reader: impl Read, jobs: &Jobs) -> Vec<String> {
    let mut all = Vec::new();
    let mut buf = Vec::new();
    let mut chunk = [0u8; 1024];
    let emit = |buf: &mut Vec<u8>, all: &mut Vec<String>| {
        let line = String::from_utf8_lossy(buf).trim().to_string();
        buf.clear();
        if !line.is_empty() {
            let mut j = lock(jobs);
            if j.lines.len() >= 300 {
                j.lines.pop_front();
            }
            j.lines.push_back(line.clone());
            all.push(line);
        }
    };
    while let Ok(n) = reader.read(&mut chunk) {
        if n == 0 {
            break;
        }
        for &b in &chunk[..n] {
            if b == b'\n' || b == b'\r' {
                emit(&mut buf, &mut all);
            } else {
                buf.push(b);
            }
        }
    }
    emit(&mut buf, &mut all);
    all
}

pub fn cancel(jobs: &Jobs, shared: &Shared) -> bool {
    let pid = {
        let mut j = lock(jobs);
        if !j.active {
            return false;
        }
        j.cancel = true;
        j.pid
    };
    if let Some(pid) = pid {
        // SAFETY: plain signal to the child we spawned; it is only reaped
        // by the worker's wait(), after which pid is cleared.
        unsafe {
            libc::kill(pid as libc::pid_t, libc::SIGTERM);
        }
    }
    state::lock(shared).command("Job cancellation requested");
    true
}

// ---------------------------------------------------------------------------
// Firmware flashing
// ---------------------------------------------------------------------------

/// `.px4` files in the firmware directory, newest first.
pub fn discover_firmware_files(dir: &str) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else { return Vec::new() };
    let mut files: Vec<(std::time::SystemTime, PathBuf)> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_file() && p.extension().is_some_and(|e| e == "px4"))
        .map(|p| (p.metadata().and_then(|m| m.modified()).unwrap_or(std::time::UNIX_EPOCH), p))
        .collect();
    files.sort_by_key(|f| std::cmp::Reverse(f.0));
    files.into_iter().map(|(_, p)| p).collect()
}

/// Companion-computer UARTs wired to the FC (Jetson ttyTHS*, Raspberry Pi
/// ttyAMA*); flashing over one goes through the bootloader's serial port.
const UART_PREFIXES: [&str; 2] = ["ttyTHS", "ttyAMA"];

pub fn is_uart_port(port: &str) -> bool {
    let name = port.rsplit('/').next().unwrap_or(port);
    UART_PREFIXES.iter().any(|p| name.starts_with(p))
}

/// Ports a flight controller could be on: USB serial first, then companion
/// UARTs (never a PC's generic /dev/ttyS* UARTs).
pub fn discover_serial_ports() -> Vec<String> {
    let mut ports: Vec<String> = Vec::new();
    for (dir, prefixes) in [
        ("/dev/serial/by-id", &[""][..]),
        ("/dev", &["ttyACM", "ttyUSB", UART_PREFIXES[0], UART_PREFIXES[1]][..]),
    ] {
        if let Ok(entries) = std::fs::read_dir(dir) {
            for e in entries.filter_map(Result::ok) {
                let name = e.file_name().to_string_lossy().into_owned();
                if prefixes.iter().any(|p| name.starts_with(p)) {
                    ports.push(format!("{dir}/{name}"));
                }
            }
        }
    }
    ports.sort_by(|a, b| (is_uart_port(a), a).cmp(&(is_uart_port(b), b)));
    ports.dedup();
    ports
}

/// Baud rates for flashing over a true serial port (ignored over USB).
pub struct FlashBauds {
    pub bootloader: u32,
    /// Comma-separated; px_uploader sends its reboot-to-bootloader at these.
    pub flightstack: String,
}

/// Parse "57600,921600" into bauds, skipping junk.
pub fn parse_bauds(list: &str) -> Vec<u32> {
    list.split(',').filter_map(|b| b.trim().parse().ok()).collect()
}

/// [f] over a companion UART. px_uploader reboots PX4 into its bootloader
/// through the UART itself, and that bootloader then waits forever for a
/// host, so first make sure PX4 is really on the other end: listen for its
/// HEARTBEAT, and refuse to flash without one. Runs off the UI thread.
pub fn start_uart_firmware_flash(jobs: &Jobs, shared: &Shared, tools_dir: &str, firmware: &Path, port: &str, bauds: FlashBauds) {
    let (jobs, shared) = (jobs.clone(), shared.clone());
    let (tools_dir, firmware, port) = (tools_dir.to_string(), firmware.to_path_buf(), port.to_string());
    state::lock(&shared).info(format!("Checking {port} for a PX4 heartbeat before flashing..."));
    std::thread::spawn(move || {
        let candidates = parse_bauds(&bauds.flightstack);
        match crate::uartprobe::find_heartbeat_baud(&port, &candidates, std::time::Duration::from_millis(2500)) {
            Ok(baud) => {
                state::lock(&shared).info(format!("PX4 heartbeat on {port} at {baud} baud"));
                let bauds = FlashBauds { bootloader: bauds.bootloader, flightstack: baud.to_string() };
                start_firmware_flash(&jobs, &shared, &tools_dir, &firmware, &port, &bauds);
            }
            Err(e) => state::lock(&shared).error(format!(
                "Not flashing: {e}. Without it the FC can't be confirmed on this UART, and a \
                 reboot into the bootloader there could leave it stuck until power-cycled. \
                 Check the wiring / MAVLink on that port, or flash over USB."
            )),
        }
    });
}

/// The /dev/serial/by-id name changes when PX4 reboots into its bootloader
/// (`..._PX4_FMU_v6X.x_0-if00` -> `..._PX4_BL_FMU_v6X.x_0-if00`), so the path
/// picked while PX4 runs is gone by the time px_uploader looks for the
/// bootloader. Hand it a glob (it expands comma-separated patterns) matching
/// both names, plus the original path.
fn uploader_port_pattern(port: &str) -> String {
    if port.starts_with("/dev/serial/by-id/") && port.contains("PX4_") {
        let glob = port.replace("PX4_BL_", "PX4_").replacen("PX4_", "PX4_*", 1);
        format!("{glob},{port}")
    } else {
        port.to_string()
    }
}

pub fn start_firmware_flash(jobs: &Jobs, shared: &Shared, tools_dir: &str, firmware: &Path, port: &str, bauds: &FlashBauds) -> bool {
    let uploader = Path::new(tools_dir).join("px_uploader.py");
    if !uploader.is_file() {
        state::lock(shared).error(format!("px_uploader.py not found under {tools_dir}"));
        return false;
    }
    let mut cmd = Command::new(tool_python(tools_dir));
    cmd.arg(&uploader).arg("--port").arg(uploader_port_pattern(port));
    if is_uart_port(port) {
        cmd.arg("--baud-bootloader").arg(bauds.bootloader.to_string());
        cmd.arg("--baud-flightstack").arg(&bauds.flightstack);
    }
    cmd.arg(firmware);
    let name = firmware.file_name().unwrap_or_default().to_string_lossy();
    start(jobs, shared, JobKind::Firmware, cmd, format!("Firmware flash ({name} -> {port})"), None)
}

// ---------------------------------------------------------------------------
// .ulog upload / ecl_ekf check
// ---------------------------------------------------------------------------

/// Best-effort copy to the clipboard: wl-copy (Wayland), else xclip.
pub(crate) fn copy_to_clipboard(text: &str) -> bool {
    for (bin, args) in [("wl-copy", &[][..]), ("xclip", &["-selection", "clipboard"][..])] {
        let Ok(mut child) = Command::new(bin).args(args).stdin(Stdio::piped()).stdout(Stdio::null()).stderr(Stdio::null()).spawn() else {
            continue;
        };
        if let Some(mut stdin) = child.stdin.take() {
            use std::io::Write;
            let _ = stdin.write_all(text.as_bytes());
        }
        if child.wait().is_ok_and(|s| s.success()) {
            return true;
        }
    }
    false
}

/// Upload a .ulg to a Flight Review v2 server (`POST <server>/api/upload`)
/// with curl, then copy the log's page URL (`<server>/log/<id>`) and save it
/// under `links_dir`.
pub fn start_ulog_upload(jobs: &Jobs, shared: &Shared, server: &str, log: &Path, links_dir: &Path) -> bool {
    let mut file_arg = std::ffi::OsString::from("file=@");
    file_arg.push(log);
    let mut cmd = Command::new("curl");
    cmd.args(["--silent", "--show-error", "--fail-with-body", "-X", "POST"])
        .arg("-F")
        .arg(file_arg)
        .args(["-F", "source=lazypx4", "-F", "is_public=false"])
        .arg(format!("{server}/api/upload"));

    let base = server.to_string();
    let (log_path, links_dir) = (log.to_path_buf(), links_dir.to_path_buf());
    let on_success: OnSuccess = Box::new(move |output, shared| {
        let reply: serde_json::Value = output
            .lines()
            .find_map(|line| serde_json::from_str(line.trim()).ok())
            .ok_or("Flight Review did not return a JSON reply")?;
        let id = reply["id"].as_str().ok_or("Flight Review reply has no log id")?;
        let url = format!("{base}/log/{id}");
        let mut st = state::lock(shared);
        if copy_to_clipboard(&url) {
            st.info(format!("Log report URL copied to clipboard: {url}"));
        } else {
            st.warn("Could not copy the URL to the clipboard (wl-copy / xclip)");
        }
        let token = reply["delete_token"].as_str();
        if let Some(token) = token {
            st.info(format!("Delete token for {id}: {token}"));
        }
        match save_review_link(&links_dir, &log_path, &url, token) {
            Ok(file) => st.info(format!("Flight Review link saved to {}", file.display())),
            Err(e) => st.warn(format!("Could not save the Flight Review link in {}: {e}", links_dir.display())),
        }
        Ok(url)
    });
    let name = log.file_name().unwrap_or_default().to_string_lossy();
    start(jobs, shared, JobKind::UlogUpload, cmd, format!("Uploading {name} to {server}"), Some(on_success))
}

/// `<links_dir>/<log name>.txt` (URL, log, delete token) plus a line in
/// `<links_dir>/flight_review_links.csv`. Returns the .txt path.
fn save_review_link(links_dir: &Path, log: &Path, url: &str, token: Option<&str>) -> std::io::Result<PathBuf> {
    use std::io::Write;
    std::fs::create_dir_all(links_dir)?;
    let stem = log.file_stem().unwrap_or_default().to_string_lossy();
    let log_abs = std::fs::canonicalize(log).unwrap_or_else(|_| log.to_path_buf());
    let time = chrono::Local::now().format("%Y-%m-%d %H:%M:%S").to_string();
    let token = token.unwrap_or("");

    let file = links_dir.join(format!("{stem}.txt"));
    std::fs::write(&file, format!("url: {url}\nlog: {}\nuploaded: {time}\ndelete_token: {token}\n", log_abs.display()))?;

    let index = links_dir.join("flight_review_links.csv");
    let new = !index.exists();
    let mut csv = std::fs::OpenOptions::new().create(true).append(true).open(index)?;
    if new {
        writeln!(csv, "uploaded,log,url,delete_token")?;
    }
    writeln!(csv, "{time},{},{url},{token}", log_abs.display())?;
    Ok(file)
}

pub fn start_ecl_ekf_check(jobs: &Jobs, shared: &Shared, tools_dir: &str, log: &Path) -> bool {
    let script = Path::new(tools_dir).join("ecl_ekf/process_logdata_ekf.py");
    if !script.is_file() {
        state::lock(shared).error(format!("process_logdata_ekf.py not found under {tools_dir}"));
        return false;
    }
    let mut cmd = Command::new(tool_python(tools_dir));
    cmd.arg(&script).arg(log);
    let report = format!("{}-0.pdf", log.display());
    let on_success: OnSuccess = Box::new(move |output, _| {
        let mut verdict = if output.contains("Minor anomalies detected") {
            "WARNING - minor anomalies detected".to_string()
        } else if output.contains("No anomalies detected") {
            "PASS - no anomalies detected".to_string()
        } else {
            "completed".to_string()
        };
        if Path::new(&report).is_file() {
            verdict += &format!(" - report: {report}");
        }
        Ok(verdict)
    });
    let name = log.file_name().unwrap_or_default().to_string_lossy();
    start(jobs, shared, JobKind::EclEkf, cmd, format!("EKF health-check on {name}"), Some(on_success))
}
