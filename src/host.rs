//! Companion-computer monitoring - the machine lazypx4 runs on, not the
//! vehicle. Mirrors `sysmon.py` (CPU / RAM / disk load and service checks for
//! the dashboard's HOST / SVC lines) and `netmon.py` (interfaces, Wi-Fi,
//! neighbours, USB devices, Tailscale, on-demand speed test for the [u]
//! screen).
//!
//! Linux only: /proc and /sys are read directly; `ip`, `nmcli`, `dmesg` and
//! `tailscale` enrich it when present, and everything degrades quietly
//! without them.

use std::collections::HashMap;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

// ---------------------------------------------------------------------------
// System stats (sysmon.py)
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Clone)]
pub struct SystemStats {
    /// /proc was readable at least once.
    pub ok: bool,
    pub cpu_percent: f64,
    pub load1: f64,
    pub mem_percent: f64,
    pub mem_used_gb: f64,
    pub mem_total_gb: f64,
    pub disk_percent: f64,
    pub disk_free_gb: f64,
    pub rosbag_recording: bool,
    pub zenoh_running: bool,
    pub xrce_agent_running: bool,
}

/// (total, idle) jiffies from /proc/stat.
fn read_cpu_times() -> Option<(u64, u64)> {
    let text = std::fs::read_to_string("/proc/stat").ok()?;
    let values: Vec<u64> = text.lines().next()?.split_whitespace().skip(1).filter_map(|v| v.parse().ok()).collect();
    let idle = values.get(3)? + values.get(4).copied().unwrap_or(0); // idle + iowait
    Some((values.iter().sum(), idle))
}

fn read_mem_gb() -> Option<(f64, f64, f64)> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    let info: HashMap<&str, f64> = text
        .lines()
        .filter_map(|l| {
            let (k, rest) = l.split_once(':')?;
            Some((k, rest.split_whitespace().next()?.parse().ok()?))
        })
        .collect();
    let total = *info.get("MemTotal")?;
    let avail = info.get("MemAvailable").or(info.get("MemFree")).copied().unwrap_or(0.0);
    if total <= 0.0 {
        return None;
    }
    let used = (total - avail).max(0.0);
    let gb = 1.0 / (1024.0 * 1024.0);
    Some((used * gb, total * gb, 100.0 * used / total))
}

/// (free GiB, used percent) of the filesystem holding `path`.
fn disk_usage(path: &str) -> Option<(f64, f64)> {
    let c_path = std::ffi::CString::new(path).ok()?;
    let mut s: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: c_path is a valid NUL-terminated string and `s` is a properly
    // sized, writable statvfs.
    if unsafe { libc::statvfs(c_path.as_ptr(), &mut s) } != 0 {
        return None;
    }
    let frsize = s.f_frsize as f64;
    let total = s.f_blocks as f64 * frsize;
    let free = s.f_bavail as f64 * frsize;
    let used = total - s.f_bfree as f64 * frsize;
    let percent = if total > 0.0 { 100.0 * used / total } else { 0.0 };
    Some((free / (1u64 << 30) as f64, percent))
}

fn cmdlines() -> Vec<String> {
    let Ok(dir) = std::fs::read_dir("/proc") else { return Vec::new() };
    dir.filter_map(Result::ok)
        .filter(|e| e.file_name().to_string_lossy().bytes().all(|b| b.is_ascii_digit()))
        .filter_map(|e| std::fs::read(e.path().join("cmdline")).ok())
        .filter(|raw| !raw.is_empty())
        .map(|raw| String::from_utf8_lossy(&raw).replace('\0', " ").to_lowercase())
        .collect()
}

fn detect_services() -> (bool, bool, bool) {
    let mut rosbag = false;
    let mut zenoh = false;
    let mut xrce = false;
    for cmd in cmdlines() {
        rosbag |= cmd.contains("record") && (cmd.contains("bag") || cmd.contains("rosbag2"));
        zenoh |= cmd.contains("zenoh");
        // PX4's MicroXRCEAgent, a source build of micro-xrce-dds-agent, ...
        xrce |= cmd.contains("xrce");
    }
    (rosbag, zenoh, xrce)
}

fn load_average() -> f64 {
    std::fs::read_to_string("/proc/loadavg")
        .ok()
        .and_then(|s| s.split_whitespace().next()?.parse().ok())
        .unwrap_or(0.0)
}

pub fn sysmon_thread(stats: Arc<Mutex<SystemStats>>, disk_path: String, shutdown: Arc<AtomicBool>) {
    let mut previous = read_cpu_times();
    while sleep_unless_shutdown(&shutdown, Duration::from_secs(2)) {
        let now_cpu = read_cpu_times();
        let mem = read_mem_gb();
        let disk = disk_usage(&disk_path);
        let (rosbag, zenoh, xrce) = detect_services();

        let mut s = lock(&stats);
        if let (Some(p), Some(c)) = (previous, now_cpu) {
            s.ok = true;
            let total = c.0.saturating_sub(p.0) as f64;
            let idle = c.1.saturating_sub(p.1) as f64;
            s.cpu_percent = if total > 0.0 { (100.0 * (total - idle) / total).clamp(0.0, 100.0) } else { 0.0 };
        }
        s.load1 = load_average();
        if let Some((used, total, pct)) = mem {
            (s.mem_used_gb, s.mem_total_gb, s.mem_percent) = (used, total, pct);
        }
        if let Some((free, pct)) = disk {
            (s.disk_free_gb, s.disk_percent) = (free, pct);
        }
        (s.rosbag_recording, s.zenoh_running, s.xrce_agent_running) = (rosbag, zenoh, xrce);
        previous = now_cpu.or(previous);
    }
}

/// Sleep in small steps; false once shutdown is requested.
pub fn sleep_unless_shutdown(shutdown: &AtomicBool, total: Duration) -> bool {
    let deadline = Instant::now() + total;
    while Instant::now() < deadline {
        if shutdown.load(Ordering::Relaxed) {
            return false;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    !shutdown.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// Network / USB (netmon.py)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default)]
pub struct NetInterface {
    pub name: String,
    pub wifi: bool,
    pub up: bool,
    pub carrier: bool,
    /// -1 = unknown / not applicable.
    pub speed_mbps: i64,
    pub addrs: Vec<String>,
    pub ssid: String,
    /// -1 = not connected / no nmcli.
    pub signal_percent: i32,
    /// Live throughput from the interface's own counters - deliberately not
    /// an internet test, which would burn data on the flight radio.
    pub rx_bps: f64,
    pub tx_bps: f64,
    pub rx_packets: u64,
    pub tx_packets: u64,
    pub errors: [u64; 5], // rx_errors, tx_errors, rx_dropped, tx_dropped, collisions
}

#[derive(Debug, Clone, Default)]
pub struct UsbDevice {
    pub location: String,
    pub vendor_id: String,
    pub product_id: String,
    pub manufacturer: String,
    pub product: String,
    pub speed_mbps: f64,
    pub max_power_ma: u32,
}

#[derive(Debug, Clone, Default)]
pub struct Neighbor {
    pub ip: String,
    pub mac: String,
    pub iface: String,
    pub state: String,
}

#[derive(Debug, Clone, Default)]
pub struct Tailscale {
    pub available: bool,
    pub backend_state: String,
    pub hostname: String,
    pub ips: Vec<String>,
    pub exit_node: String,
    pub peers_total: usize,
    pub peers_online: usize,
}

#[derive(Debug, Clone, Default)]
pub struct SpeedTest {
    pub running: bool,
    pub completed: bool,
    pub error: String,
    pub download_mbps: f64,
    pub upload_mbps: f64,
    pub ping_ms: f64,
    pub server: String,
}

#[derive(Debug, Clone, Default)]
pub struct NetStats {
    pub ok: bool,
    pub interfaces: Vec<NetInterface>,
    pub neighbors_ok: bool,
    pub neighbors: Vec<Neighbor>,
    pub usb_ok: bool,
    pub usb_devices: Vec<UsbDevice>,
    pub usb_warnings_available: bool,
    pub usb_warnings: Vec<String>,
    pub tailscale: Tailscale,
    pub speedtest: SpeedTest,
}

const SYS_NET: &str = "/sys/class/net";
const SYS_USB: &str = "/sys/bus/usb/devices";
/// Loopback, container and tunnel interfaces - not a physical link.
const SKIP_IFACE_PREFIXES: &[&str] = &["lo", "docker", "br-", "veth", "virbr", "tun", "tap"];

fn read_text(path: &str) -> String {
    std::fs::read_to_string(path).map(|s| s.trim().to_string()).unwrap_or_default()
}

/// stdout of a command, when it exits 0 (`keep_on_error`: regardless).
fn run(args: &[&str], keep_on_error: bool) -> Option<String> {
    let out = Command::new(args[0]).args(&args[1..]).stdin(std::process::Stdio::null()).output().ok()?;
    (keep_on_error || out.status.success()).then(|| String::from_utf8_lossy(&out.stdout).into_owned())
}

fn skip_iface(name: &str) -> bool {
    SKIP_IFACE_PREFIXES.iter().any(|p| name.starts_with(p))
}

fn addrs_by_iface() -> HashMap<String, Vec<String>> {
    let mut map: HashMap<String, Vec<String>> = HashMap::new();
    for line in run(&["ip", "-o", "addr", "show"], false).unwrap_or_default().lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 4 && (parts[2] == "inet" || parts[2] == "inet6") {
            map.entry(parts[1].to_string()).or_default().push(parts[3].to_string());
        }
    }
    map
}

fn wifi_by_iface() -> HashMap<String, (String, i32)> {
    let mut map = HashMap::new();
    let out = run(&["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL,DEVICE", "dev", "wifi"], false).unwrap_or_default();
    for line in out.lines() {
        let parts: Vec<&str> = line.split(':').collect();
        if parts.len() < 4 || parts[0] != "yes" {
            continue;
        }
        let n = parts.len();
        map.insert(parts[n - 1].to_string(), (parts[n - 3].to_string(), parts[n - 2].parse().unwrap_or(-1)));
    }
    map
}

fn read_interfaces(prev: &mut HashMap<String, (Instant, u64, u64)>) -> Option<Vec<NetInterface>> {
    let mut names: Vec<String> = std::fs::read_dir(SYS_NET)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();

    let addrs = addrs_by_iface();
    let wifi = wifi_by_iface();
    let now = Instant::now();
    let mut out = Vec::new();

    for name in names {
        if skip_iface(&name) {
            continue;
        }
        let base = format!("{SYS_NET}/{name}");
        let is_wifi = std::path::Path::new(&format!("{base}/wireless")).is_dir()
            || std::path::Path::new(&format!("{base}/phy80211")).is_dir();
        let is_eth = !is_wifi && std::path::Path::new(&format!("{base}/device")).is_dir();
        if !is_wifi && !is_eth {
            continue;
        }
        let counter = |f: &str| read_text(&format!("{base}/statistics/{f}")).parse::<u64>().unwrap_or(0);
        let (rx_bytes, tx_bytes) = (counter("rx_bytes"), counter("tx_bytes"));
        let (mut rx_bps, mut tx_bps) = (0.0, 0.0);
        if let Some((t, prx, ptx)) = prev.get(&name) {
            let dt = now.duration_since(*t).as_secs_f64();
            if dt > 0.0 {
                rx_bps = rx_bytes.saturating_sub(*prx) as f64 * 8.0 / dt;
                tx_bps = tx_bytes.saturating_sub(*ptx) as f64 * 8.0 / dt;
            }
        }
        prev.insert(name.clone(), (now, rx_bytes, tx_bytes));

        let (ssid, signal) = wifi.get(&name).cloned().unwrap_or((String::new(), -1));
        out.push(NetInterface {
            wifi: is_wifi,
            up: read_text(&format!("{base}/operstate")) == "up",
            carrier: read_text(&format!("{base}/carrier")) == "1",
            speed_mbps: if is_eth { read_text(&format!("{base}/speed")).parse().unwrap_or(-1) } else { -1 },
            addrs: addrs.get(&name).cloned().unwrap_or_default(),
            ssid,
            signal_percent: signal,
            rx_bps,
            tx_bps,
            rx_packets: counter("rx_packets"),
            tx_packets: counter("tx_packets"),
            errors: [
                counter("rx_errors"),
                counter("tx_errors"),
                counter("rx_dropped"),
                counter("tx_dropped"),
                counter("collisions"),
            ],
            name,
        });
    }
    Some(out)
}

/// The kernel neighbour (ARP/NDP) table - devices this host has talked to;
/// passive, never an active scan.
fn read_neighbors() -> Option<Vec<Neighbor>> {
    let out = run(&["ip", "neigh", "show"], false)?;
    let mut list: Vec<Neighbor> = out
        .lines()
        .filter_map(|line| {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 3 || parts[1] != "dev" || skip_iface(parts[2]) {
                return None;
            }
            let mac = parts.iter().position(|p| *p == "lladdr").and_then(|i| parts.get(i + 1)).unwrap_or(&"");
            Some(Neighbor {
                ip: parts[0].into(),
                mac: mac.to_string(),
                iface: parts[2].into(),
                state: parts.last().unwrap().to_uppercase(),
            })
        })
        .collect();
    list.sort_by(|a, b| a.ip.cmp(&b.ip));
    list.truncate(24);
    Some(list)
}

fn read_usb_devices() -> Option<Vec<UsbDevice>> {
    let mut entries: Vec<String> = std::fs::read_dir(SYS_USB)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.file_name().to_string_lossy().into_owned())
        // "1-8:1.0" are interfaces and "usbN" root hubs, not peripherals.
        .filter(|e| !e.contains(':') && !e.starts_with("usb"))
        .collect();
    entries.sort();
    Some(
        entries
            .into_iter()
            .map(|entry| {
                let base = format!("{SYS_USB}/{entry}");
                let text = |f: &str| read_text(&format!("{base}/{f}"));
                UsbDevice {
                    vendor_id: text("idVendor"),
                    product_id: text("idProduct"),
                    manufacturer: text("manufacturer"),
                    product: text("product"),
                    speed_mbps: text("speed").parse().unwrap_or(0.0),
                    max_power_ma: text("bMaxPower").trim_end_matches("mA").parse().unwrap_or(0),
                    location: entry,
                }
            })
            .collect(),
    )
}

/// Recent kernel USB errors/warnings, or None when dmesg is restricted.
fn read_usb_warnings() -> Option<Vec<String>> {
    let out = run(&["dmesg", "--ctime", "--level=err,warn"], false)?;
    let lines: Vec<String> = out
        .lines()
        .filter(|l| {
            let l = l.to_lowercase();
            ["usb", "xhci", "ehci", "ohci"].iter().any(|k| l.contains(k))
        })
        .map(str::to_string)
        .collect();
    Some(lines[lines.len().saturating_sub(6)..].to_vec())
}

fn read_tailscale() -> Option<Tailscale> {
    // `tailscale status` exits non-zero for normal states (logged out).
    let out = run(&["tailscale", "status", "--json"], true)?;
    let Ok(data) = serde_json::from_str::<serde_json::Value>(&out) else {
        return Some(Tailscale::default());
    };
    let peers = data.get("Peer").and_then(|p| p.as_object()).cloned().unwrap_or_default();
    let str_of = |v: &serde_json::Value, k: &str| v.get(k).and_then(|x| x.as_str()).unwrap_or_default().to_string();
    Some(Tailscale {
        available: true,
        backend_state: data.get("BackendState").and_then(|v| v.as_str()).unwrap_or("Unknown").into(),
        hostname: data.get("Self").map(|s| str_of(s, "HostName")).unwrap_or_default(),
        ips: data
            .get("TailscaleIPs")
            .and_then(|v| v.as_array())
            .map(|a| a.iter().filter_map(|x| x.as_str().map(String::from)).collect())
            .unwrap_or_default(),
        exit_node: peers
            .values()
            .find(|p| p.get("ExitNode").and_then(|v| v.as_bool()).unwrap_or(false))
            .map(|p| str_of(p, "HostName"))
            .unwrap_or_default(),
        peers_total: peers.len(),
        peers_online: peers.values().filter(|p| p.get("Online").and_then(|v| v.as_bool()).unwrap_or(false)).count(),
    })
}

pub fn netmon_thread(stats: Arc<Mutex<NetStats>>, shutdown: Arc<AtomicBool>) {
    let mut prev = HashMap::new();
    loop {
        let interfaces = read_interfaces(&mut prev);
        let neighbors = read_neighbors();
        let usb = read_usb_devices();
        let warnings = read_usb_warnings();
        let tailscale = read_tailscale();
        {
            let mut s = lock(&stats);
            if let Some(i) = interfaces {
                s.ok = true;
                s.interfaces = i;
            }
            if let Some(n) = neighbors {
                s.neighbors_ok = true;
                s.neighbors = n;
            }
            if let Some(u) = usb {
                s.usb_ok = true;
                s.usb_devices = u;
            }
            if let Some(w) = warnings {
                s.usb_warnings_available = true;
                s.usb_warnings = w;
            }
            if let Some(t) = tailscale {
                s.tailscale = t;
            }
        }
        if !sleep_unless_shutdown(&shutdown, Duration::from_secs(3)) {
            return;
        }
    }
}

/// A real download/upload/ping test. Only ever started by a keypress: it
/// burns real bandwidth, possibly on the flight radio. False if one runs.
pub fn run_speedtest_async(stats: &Arc<Mutex<NetStats>>) -> bool {
    {
        let mut s = lock(stats);
        if s.speedtest.running {
            return false;
        }
        s.speedtest = SpeedTest { running: true, ..Default::default() };
    }
    let stats = stats.clone();
    std::thread::spawn(move || {
        let mut result = SpeedTest { completed: true, ..Default::default() };
        match Command::new("speedtest").args(["--json", "--secure"]).stdin(std::process::Stdio::null()).output() {
            Err(_) => result.error = "'speedtest' not found (pip install speedtest-cli)".into(),
            Ok(out) if !out.status.success() || out.stdout.is_empty() => {
                let err = String::from_utf8_lossy(&out.stderr);
                result.error = err.lines().last().unwrap_or("speedtest failed").chars().take(200).collect();
            }
            Ok(out) => match serde_json::from_slice::<serde_json::Value>(&out.stdout) {
                Ok(d) => {
                    let f = |k: &str| d.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
                    result.download_mbps = f("download") / 1e6;
                    result.upload_mbps = f("upload") / 1e6;
                    result.ping_ms = f("ping");
                    if let Some(s) = d.get("server") {
                        let g = |k: &str| s.get(k).and_then(|v| v.as_str()).unwrap_or_default();
                        result.server = format!("{} ({})", g("sponsor"), g("name"));
                    }
                }
                Err(e) => result.error = format!("couldn't parse speedtest output: {e}"),
            },
        }
        lock(&stats).speedtest = result;
    });
    true
}
