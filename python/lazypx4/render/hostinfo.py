"""The [u] screen: host USB devices and network (Wi-Fi/Ethernet/IP) health.

A companion-computer sanity check, not vehicle telemetry - "is the telemetry
radio's USB dongle actually enumerated", "did Wi-Fi drop", "does eth0 have an
IP". See :mod:`lazypx4.netmon` for how the data is gathered.
"""

from __future__ import annotations

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW, ui_section
from ..config import WIFI_SIGNAL_GOOD, WIFI_SIGNAL_OK
from ..netmon import stats as net
from .chrome import graded_color

_WARN_LINE_RE_KEYWORDS = ("error", "fail", "reset", "disconnect", "over-current", "overcurrent")


def _iface_state_text(iface):
    if not iface.up:
        return RED + "DOWN" + RESET
    if not iface.carrier:
        return YELLOW + "UP (no carrier)" + RESET
    return GREEN + "UP" + RESET


def _usb_speed_text(mbps):
    if mbps >= 5000:
        return GREEN + BOLD + f"{mbps:.0f} Mbps (SuperSpeed)" + RESET
    if mbps >= 480:
        return GREEN + f"{mbps:.0f} Mbps (High Speed)" + RESET
    if mbps >= 12:
        return DIM + f"{mbps:.1f} Mbps (Full Speed)" + RESET
    if mbps > 0:
        return DIM + f"{mbps:.1f} Mbps (Low Speed)" + RESET
    return DIM + "unknown speed" + RESET


def _format_rate(bps):
    if bps >= 1e6:
        return f"{bps / 1e6:.2f} Mbps"
    if bps >= 1e3:
        return f"{bps / 1e3:.1f} Kbps"
    return f"{bps:.0f} bps"


def _tailscale_state_text(backend_state):
    if backend_state == "Running":
        return GREEN + "Running" + RESET
    if backend_state in ("NeedsLogin", "NeedsMachineAuth"):
        return YELLOW + backend_state + RESET
    if backend_state in ("Stopped", ""):
        return RED + (backend_state or "Stopped") + RESET
    return DIM + backend_state + RESET


def _neighbor_state_text(state):
    if state in ("REACHABLE", "PERMANENT"):
        return GREEN + state + RESET
    if state in ("STALE", "DELAY", "PROBE"):
        return YELLOW + state + RESET
    if state in ("FAILED", "INCOMPLETE"):
        return RED + state + RESET
    return DIM + state + RESET


def draw_host_screen():
    with net.lock:
        ok = net.ok
        interfaces = list(net.interfaces)
        usb_ok = net.usb_ok
        usb_devices = list(net.usb_devices)
        usb_warnings_available = net.usb_warnings_available
        usb_warnings = list(net.usb_warnings)
        tailscale = net.tailscale
        speedtest = net.speedtest
        neighbors_ok = net.neighbors_ok
        neighbors = list(net.neighbors)

    lines = []

    lines.append(ui_section("NETWORK"))

    if not ok:
        lines.append("   " + DIM + "n/a (not Linux / /sys unreadable)" + RESET)
    elif not interfaces:
        lines.append("   " + DIM + "no Wi-Fi / Ethernet interfaces found" + RESET)
    else:
        for iface in interfaces:
            addrs = ", ".join(iface.addrs) if iface.addrs else DIM + "no IP" + RESET
            kind_label = "wifi" if iface.kind == "wifi" else "eth"

            lines.append(
                f"   {iface.name:<10} {DIM}({kind_label}){RESET}"
                f"   {_iface_state_text(iface)}"
                f"   IP: {addrs}"
            )

            if iface.kind == "wifi":
                if iface.signal_percent >= 0:
                    sig_color = graded_color(iface.signal_percent, WIFI_SIGNAL_GOOD, WIFI_SIGNAL_OK)
                    ssid_text = iface.ssid or DIM + "(hidden)" + RESET
                    lines.append(
                        f"      SSID: {BOLD}{ssid_text}{RESET}"
                        f"   signal: {sig_color}{iface.signal_percent}%{RESET}"
                    )
                elif iface.up:
                    lines.append("      " + DIM + "not associated / nmcli not available" + RESET)
            elif iface.kind == "ethernet" and iface.speed_mbps > 0:
                speed_color = GREEN if iface.speed_mbps >= 100 else YELLOW
                lines.append(f"      link speed: {speed_color}{iface.speed_mbps} Mbps{RESET}")

            if iface.up:
                lines.append(
                    f"      RX {_format_rate(iface.rx_bps)} ({iface.rx_packets} pkts)"
                    f"   TX {_format_rate(iface.tx_bps)} ({iface.tx_packets} pkts)"
                )
                problems = (
                    iface.rx_errors + iface.tx_errors
                    + iface.rx_dropped + iface.tx_dropped + iface.collisions
                )
                if problems:
                    lines.append(
                        f"      {RED}errors: rx {iface.rx_errors} tx {iface.tx_errors}"
                        f"   dropped: rx {iface.rx_dropped} tx {iface.tx_dropped}"
                        f"   collisions: {iface.collisions}{RESET}"
                    )

    lines.append("")
    lines.append(ui_section("DEVICES ON NETWORK", "from the ARP table, not an active scan"))

    if not neighbors_ok:
        lines.append("   " + DIM + "n/a ('ip' not available)" + RESET)
    elif not neighbors:
        lines.append("   " + DIM + "no other devices seen yet" + RESET)
    else:
        for n in neighbors:
            mac_display = n.mac or "?"
            lines.append(
                f"   {n.ip:<16} {DIM}{mac_display:<19}{RESET}"
                f" {DIM}{n.iface:<9}{RESET} {_neighbor_state_text(n.state)}"
            )

    lines.append("")
    lines.append(ui_section("INTERNET SPEED TEST", "[i] to run"))

    if speedtest.running:
        lines.append("   " + YELLOW + "running... (download / upload / ping, ~15-30s)" + RESET)
    elif not speedtest.completed:
        lines.append("   " + DIM + "not run yet - press [i]" + RESET)
    elif speedtest.error:
        lines.append("   " + RED + f"failed: {speedtest.error}" + RESET)
    else:
        lines.append(
            f"   Download: {GREEN}{speedtest.download_mbps:.1f} Mbps{RESET}"
            f"   Upload: {GREEN}{speedtest.upload_mbps:.1f} Mbps{RESET}"
            f"   Ping: {speedtest.ping_ms:.0f} ms"
        )
        if speedtest.server:
            lines.append("   " + DIM + f"server: {speedtest.server}" + RESET)

    lines.append("")
    lines.append(ui_section("TAILSCALE"))

    if not tailscale.available:
        lines.append("   " + DIM + "not installed / `tailscale` not on PATH" + RESET)
    else:
        ips = ", ".join(tailscale.ips) if tailscale.ips else DIM + "no IP" + RESET
        host_text = tailscale.hostname or DIM + "?" + RESET
        lines.append(
            f"   State: {_tailscale_state_text(tailscale.backend_state)}"
            f"   Host: {BOLD}{host_text}{RESET}"
            f"   IP: {ips}"
        )

        if tailscale.backend_state == "Running":
            peer_color = (
                GREEN if tailscale.peers_online else DIM
            )
            exit_text = (
                f"   Exit node: {BOLD}{tailscale.exit_node}{RESET}" if tailscale.exit_node else ""
            )
            lines.append(
                f"   Peers online: {peer_color}{tailscale.peers_online}/{tailscale.peers_total}{RESET}"
                + exit_text
            )

    lines.append("")
    lines.append(ui_section("USB DEVICES"))

    if not usb_ok:
        lines.append("   " + DIM + "n/a (not Linux / /sys/bus/usb unreadable)" + RESET)
    elif not usb_devices:
        lines.append("   " + DIM + "no USB devices enumerated" + RESET)
    else:
        for dev in usb_devices:
            name = dev.product or dev.manufacturer or DIM + "(unnamed device)" + RESET
            ids = f"{dev.vendor_id}:{dev.product_id}" if dev.vendor_id else "----:----"
            power = f"{dev.max_power_ma} mA" if dev.max_power_ma else DIM + "?" + RESET

            lines.append(f"   {dev.location:<8} {DIM}{ids}{RESET}  {name}")
            lines.append(
                f"      {_usb_speed_text(dev.speed_mbps)}   power: {power}"
            )

    lines.append("")
    lines.append(ui_section("USB KERNEL LOG", "" if usb_warnings_available else "n/a"))

    if not usb_warnings_available:
        lines.append("   " + DIM + "dmesg not accessible (needs root / dmesg_restrict=0)" + RESET)
    elif not usb_warnings:
        lines.append("   " + GREEN + "no recent USB errors/warnings" + RESET)
    else:
        for line in usb_warnings:
            lower = line.lower()
            color = RED if any(k in lower for k in _WARN_LINE_RE_KEYWORDS) else YELLOW
            lines.append(f"   {color}{line.strip()}{RESET}")

    lines.append("")
    lines.append("[i] speed test    [u] back    [ESC] panels")

    return lines
