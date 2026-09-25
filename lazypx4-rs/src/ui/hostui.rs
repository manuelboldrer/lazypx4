//! [u] USB / network (`render/hostinfo.py`), [f] flash firmware
//! (`render/firmware.py`) and the shared background-job panel
//! (`render/jobpanel.py`).

use ratatui::style::{Modifier, Style};
use ratatui::text::Line;

use super::*;
use crate::host::lock;
use crate::jobs::JobKind;

fn rate(bps: f64) -> String {
    if bps >= 1e6 {
        format!("{:.2} Mbps", bps / 1e6)
    } else if bps >= 1e3 {
        format!("{:.1} Kbps", bps / 1e3)
    } else {
        format!("{bps:.0} bps")
    }
}

pub fn host(ctx: &Ctx) -> Vec<Line<'static>> {
    let net = lock(&ctx.app.net).clone();
    let mut lines = vec![section("NETWORK", "")];

    if !net.ok {
        lines.push(dim_line("n/a (not Linux / /sys unreadable)"));
    } else if net.interfaces.is_empty() {
        lines.push(dim_line("no Wi-Fi / Ethernet interfaces found"));
    }
    for i in &net.interfaces {
        let state = if !i.up {
            Ln::new().fg("DOWN", RED)
        } else if !i.carrier {
            Ln::new().fg("UP (no carrier)", YELLOW)
        } else {
            Ln::new().fg("UP", GREEN)
        };
        let addrs = if i.addrs.is_empty() { Ln::new().dim("no IP") } else { Ln::new().raw(i.addrs.join(", ")) };
        lines.push(
            Ln::new()
                .raw(format!("   {:<10} ", i.name))
                .dim(if i.wifi { "(wifi)" } else { "(eth)" })
                .raw("   ")
                .spans(state.0)
                .raw("   IP: ")
                .spans(addrs.0)
                .line(),
        );
        if i.wifi {
            if i.signal_percent >= 0 {
                let ssid = if i.ssid.is_empty() { Ln::new().dim("(hidden)") } else { Ln::new().bold(i.ssid.clone()) };
                lines.push(
                    Ln::new()
                        .raw("      SSID: ")
                        .spans(ssid.0)
                        .raw("   signal: ")
                        .fg(format!("{}%", i.signal_percent), graded(i.signal_percent as f64, WIFI_SIGNAL_GOOD, WIFI_SIGNAL_OK, true))
                        .line(),
                );
            } else if i.up {
                lines.push(Ln::new().raw("      ").dim("not associated / nmcli not available").line());
            }
        } else if i.speed_mbps > 0 {
            lines.push(
                Ln::new()
                    .raw("      link speed: ")
                    .fg(format!("{} Mbps", i.speed_mbps), if i.speed_mbps >= 100 { GREEN } else { YELLOW })
                    .line(),
            );
        }
        if i.up {
            lines.push(Line::from(format!(
                "      RX {} ({} pkts)   TX {} ({} pkts)",
                rate(i.rx_bps),
                i.rx_packets,
                rate(i.tx_bps),
                i.tx_packets
            )));
            let [rxe, txe, rxd, txd, col] = i.errors;
            if rxe + txe + rxd + txd + col > 0 {
                lines.push(
                    Ln::new()
                        .fg(format!("      errors: rx {rxe} tx {txe}   dropped: rx {rxd} tx {txd}   collisions: {col}"), RED)
                        .line(),
                );
            }
        }
    }

    lines.push(blank());
    lines.push(section("DEVICES ON NETWORK", "from the ARP table, not an active scan"));
    if !net.neighbors_ok {
        lines.push(dim_line("n/a ('ip' not available)"));
    } else if net.neighbors.is_empty() {
        lines.push(dim_line("no other devices seen yet"));
    }
    for n in &net.neighbors {
        let state = match n.state.as_str() {
            "REACHABLE" | "PERMANENT" => Ln::new().fg(n.state.clone(), GREEN),
            "STALE" | "DELAY" | "PROBE" => Ln::new().fg(n.state.clone(), YELLOW),
            "FAILED" | "INCOMPLETE" => Ln::new().fg(n.state.clone(), RED),
            other => Ln::new().dim(other.to_string()),
        };
        let mac = if n.mac.is_empty() { "?" } else { &n.mac };
        lines.push(
            Ln::new()
                .raw(format!("   {:<16} ", n.ip))
                .dim(format!("{mac:<19} {:<9} ", n.iface))
                .spans(state.0)
                .line(),
        );
    }

    lines.push(blank());
    lines.push(section("INTERNET SPEED TEST", "[i] to run"));
    let sp = &net.speedtest;
    if sp.running {
        lines.push(Ln::new().raw("   ").fg("running... (download / upload / ping, ~15-30s)", YELLOW).line());
    } else if !sp.completed {
        lines.push(dim_line("not run yet - press [i]"));
    } else if !sp.error.is_empty() {
        lines.push(Ln::new().raw("   ").fg(format!("failed: {}", sp.error), RED).line());
    } else {
        lines.push(
            Ln::new()
                .raw("   Download: ")
                .fg(format!("{:.1} Mbps", sp.download_mbps), GREEN)
                .raw("   Upload: ")
                .fg(format!("{:.1} Mbps", sp.upload_mbps), GREEN)
                .raw(format!("   Ping: {:.0} ms", sp.ping_ms))
                .line(),
        );
        if !sp.server.is_empty() {
            lines.push(dim_line(format!("server: {}", sp.server)));
        }
    }

    lines.push(blank());
    lines.push(section("TAILSCALE", ""));
    let ts = &net.tailscale;
    if !ts.available {
        lines.push(dim_line("not installed / `tailscale` not on PATH"));
    } else {
        let state = match ts.backend_state.as_str() {
            "Running" => Ln::new().fg("Running", GREEN),
            "NeedsLogin" | "NeedsMachineAuth" => Ln::new().fg(ts.backend_state.clone(), YELLOW),
            "" | "Stopped" => Ln::new().fg("Stopped", RED),
            other => Ln::new().dim(other.to_string()),
        };
        let ips = if ts.ips.is_empty() { Ln::new().dim("no IP") } else { Ln::new().raw(ts.ips.join(", ")) };
        lines.push(
            Ln::new()
                .raw("   State: ")
                .spans(state.0)
                .raw("   Host: ")
                .bold(if ts.hostname.is_empty() { "?".to_string() } else { ts.hostname.clone() })
                .raw("   IP: ")
                .spans(ips.0)
                .line(),
        );
        if ts.backend_state == "Running" {
            let mut l = Ln::new().raw("   Peers online: ");
            let peers = format!("{}/{}", ts.peers_online, ts.peers_total);
            l = if ts.peers_online > 0 { l.fg(peers, GREEN) } else { l.dim(peers) };
            if !ts.exit_node.is_empty() {
                l = l.raw("   Exit node: ").bold(ts.exit_node.clone());
            }
            lines.push(l.line());
        }
    }

    lines.push(blank());
    lines.push(section("USB DEVICES", ""));
    if !net.usb_ok {
        lines.push(dim_line("n/a (not Linux / /sys/bus/usb unreadable)"));
    } else if net.usb_devices.is_empty() {
        lines.push(dim_line("no USB devices enumerated"));
    }
    for d in &net.usb_devices {
        let name = if !d.product.is_empty() {
            Ln::new().raw(d.product.clone())
        } else if !d.manufacturer.is_empty() {
            Ln::new().raw(d.manufacturer.clone())
        } else {
            Ln::new().dim("(unnamed device)")
        };
        let ids = if d.vendor_id.is_empty() { "----:----".to_string() } else { format!("{}:{}", d.vendor_id, d.product_id) };
        lines.push(Ln::new().raw(format!("   {:<8} ", d.location)).dim(ids).raw("  ").spans(name.0).line());
        let m = d.speed_mbps;
        let speed = if m >= 5000.0 {
            Ln::new().st(format!("{m:.0} Mbps (SuperSpeed)"), Style::new().fg(GREEN).add_modifier(Modifier::BOLD))
        } else if m >= 480.0 {
            Ln::new().fg(format!("{m:.0} Mbps (High Speed)"), GREEN)
        } else if m >= 12.0 {
            Ln::new().dim(format!("{m:.1} Mbps (Full Speed)"))
        } else if m > 0.0 {
            Ln::new().dim(format!("{m:.1} Mbps (Low Speed)"))
        } else {
            Ln::new().dim("unknown speed")
        };
        let power = if d.max_power_ma > 0 { Ln::new().raw(format!("{} mA", d.max_power_ma)) } else { Ln::new().dim("?") };
        lines.push(Ln::new().raw("      ").spans(speed.0).raw("   power: ").spans(power.0).line());
    }

    lines.push(blank());
    lines.push(section("USB KERNEL LOG", if net.usb_warnings_available { "" } else { "n/a" }));
    if !net.usb_warnings_available {
        lines.push(dim_line("dmesg not accessible (needs root / dmesg_restrict=0)"));
    } else if net.usb_warnings.is_empty() {
        lines.push(Ln::new().raw("   ").fg("no recent USB errors/warnings", GREEN).line());
    }
    for w in &net.usb_warnings {
        let lower = w.to_lowercase();
        let bad = ["error", "fail", "reset", "disconnect", "over-current", "overcurrent"].iter().any(|k| lower.contains(k));
        lines.push(Ln::new().raw("   ").fg(w.trim().to_string(), if bad { RED } else { YELLOW }).line());
    }
    lines
}

/// The latest job of `kind`, or nothing if none has run this session.
pub fn job_panel(ctx: &Ctx, kind: JobKind) -> Vec<Line<'static>> {
    let j = lock(&ctx.app.jobs);
    if j.kind != Some(kind) {
        return Vec::new();
    }
    let color = match j.status.as_str() {
        "RUNNING" | "CANCELLED" => YELLOW,
        "SUCCESS" => GREEN,
        "ERROR" => RED,
        _ => ratatui::style::Color::DarkGray,
    };
    let mut lines = vec![blank(), Ln::new().st(format!("Job: {}", j.status), Style::new().fg(color).add_modifier(Modifier::BOLD)).line()];
    if !j.error.is_empty() {
        lines.push(Ln::new().fg(format!("  {}", j.error), RED).line());
    }
    if !j.result.is_empty() {
        lines.push(Ln::new().fg(format!("  {}", j.result), GREEN).line());
    }
    let skip = j.lines.len().saturating_sub(8);
    lines.extend(j.lines.iter().skip(skip).map(|l| Ln::new().dim(format!("  {l}")).line()));
    lines
}

pub fn firmware(ctx: &Ctx) -> Vec<Line<'static>> {
    let s = &ctx.app.session;
    let port = if s.firmware_ports.is_empty() {
        Ln::new().fg("no serial port detected - plug in the flight controller", RED)
    } else {
        let i = s.firmware_port_index.min(s.firmware_ports.len() - 1);
        let mut l = Ln::new().fg(s.firmware_ports[i].clone(), GREEN);
        if s.firmware_ports.len() > 1 {
            l = l.dim(format!("   ({}/{}, LEFT/RIGHT to change)", i + 1, s.firmware_ports.len()));
        }
        l
    };
    let mut lines = vec![
        Ln::new().raw(" Target port: ").spans(port.0).line(),
        Line::from(format!(" Firmware directory: {}", ctx.app.settings.firmware_dir)),
        blank(),
        Ln::new().dim(format!(" {:>3} {:<40} {:>10}  {:<20}", "", "FILE", "SIZE", "MODIFIED")).line(),
        Ln::new().dim(format!(" {}", "─".repeat(74))).line(),
    ];
    if s.firmware_files.is_empty() {
        lines.push(Ln::new().dim(format!(" No .px4 firmware files found in {}", ctx.app.settings.firmware_dir)).line());
    }
    let cursor = s.firmware_index.min(s.firmware_files.len().saturating_sub(1));
    for (i, path) in s.firmware_files.iter().enumerate() {
        let meta = std::fs::metadata(path).ok();
        let size = meta.as_ref().map(|m| crate::state::size_string(m.len())).unwrap_or_else(|| "--".into());
        let mtime = meta
            .and_then(|m| m.modified().ok())
            .map(|t| chrono::DateTime::<chrono::Local>::from(t).format("%Y-%m-%d %H:%M:%S").to_string())
            .unwrap_or_else(|| "--".into());
        let name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
        let selected = i == cursor;
        let text = format!(" {} {name:<40} {size:>10}  {mtime:<20}", if selected { ">" } else { " " });
        lines.push(if selected { Line::styled(text, Style::new().fg(GREEN).add_modifier(Modifier::BOLD)) } else { Line::from(text) });
    }
    lines.extend(job_panel(ctx, JobKind::Firmware));
    if lock(&ctx.app.jobs).active {
        lines.push(blank());
        lines.push(Ln::new().st("Flashing in progress - ESC cancels", Style::new().fg(YELLOW).add_modifier(Modifier::BOLD)).line());
    }
    lines
}
