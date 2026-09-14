"""Host network / USB monitor: interfaces, IPs, Wi-Fi link, USB devices.

Like :mod:`lazypx4.sysmon`, this watches the machine lazypx4 runs on (a UAV
companion computer), not the vehicle - a physical-layer sanity check ("is the
telemetry radio's USB dongle actually there", "did the Wi-Fi drop", "does eth0
have an IP"). Linux only (reads ``/sys``); ``nmcli``/``ip`` are used when
present for the details ``/sys`` alone can't give (SSID, IP addresses) but
everything degrades gracefully without them. Delete this module and the [u]
USB / NETWORK screen and nothing else changes.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field

from .eventlog import log_warn
from .state import shutdown_event

SAMPLE_INTERVAL = 3.0

_SYS_NET = "/sys/class/net"
_SYS_USB = "/sys/bus/usb/devices"

#: Virtual/container interfaces that aren't useful on a "is the physical link
#: healthy" pane - loopback, Docker/bridge networking, veth pairs, tunnels.
_SKIP_IFACE_PREFIXES = ("lo", "docker", "br-", "veth", "virbr", "tun", "tap")

_SUBPROCESS_TIMEOUT = 2.0


@dataclass
class NetInterface:
    name: str
    kind: str = "other"          # "wifi" | "ethernet" | "other"
    up: bool = False
    carrier: bool = False
    speed_mbps: int = -1         # -1 = unknown/not applicable
    addrs: list = field(default_factory=list)   # ["192.168.1.5/24", ...]
    ssid: str = ""
    signal_percent: int = -1     # -1 = n/a (not connected / no nmcli)

    # Live throughput, bits/sec - the interface's own byte counters sampled
    # SAMPLE_INTERVAL apart. Deliberately not a real internet bandwidth test:
    # that would burn real data on whatever link this runs over, which in
    # flight may be the same radio carrying MAVLink/video.
    rx_bps: float = 0.0
    tx_bps: float = 0.0

    # Cumulative packet counters (/sys/class/net/<iface>/statistics/*).
    rx_packets: int = 0
    tx_packets: int = 0
    rx_errors: int = 0
    tx_errors: int = 0
    rx_dropped: int = 0
    tx_dropped: int = 0
    collisions: int = 0


@dataclass
class UsbDevice:
    location: str                # sysfs bus-port path, e.g. "1-8.2"
    vendor_id: str = ""
    product_id: str = ""
    manufacturer: str = ""
    product: str = ""
    speed_mbps: float = 0.0
    max_power_ma: int = 0


@dataclass
class TailscaleStatus:
    available: bool = False      # the `tailscale` CLI ran successfully
    backend_state: str = ""      # "Running", "Stopped", "NeedsLogin", ...
    hostname: str = ""
    ips: list = field(default_factory=list)
    exit_node: str = ""          # active exit node's name, or "" if none
    peers_total: int = 0
    peers_online: int = 0


@dataclass
class SpeedTestResult:
    """A real internet bandwidth test - unlike everything else in this
    module, this is never run automatically (see run_speedtest_async())."""
    running: bool = False
    completed: bool = False
    error: str = ""
    download_mbps: float = 0.0
    upload_mbps: float = 0.0
    ping_ms: float = 0.0
    server: str = ""
    updated: float = 0.0


@dataclass
class NeighborDevice:
    """A device this host has actually talked to on the LAN (from the kernel
    ARP/NDP neighbor table) - passive, no active scanning of the network."""
    ip: str
    mac: str = ""
    iface: str = ""
    state: str = "UNKNOWN"       # REACHABLE, STALE, DELAY, PROBE, FAILED, ...


@dataclass
class NetStats:
    ok: bool = False             # /sys/class/net was readable at least once
    interfaces: list = field(default_factory=list)   # [NetInterface]

    neighbors_ok: bool = False
    neighbors: list = field(default_factory=list)    # [NeighborDevice]

    usb_ok: bool = False         # /sys/bus/usb/devices was readable
    usb_devices: list = field(default_factory=list)  # [UsbDevice]
    usb_warnings: list = field(default_factory=list)  # recent dmesg USB lines
    usb_warnings_available: bool = False

    tailscale: TailscaleStatus = field(default_factory=TailscaleStatus)

    speedtest: SpeedTestResult = field(default_factory=SpeedTestResult)

    updated: float = 0.0

    lock: threading.Lock = field(default_factory=threading.Lock)


stats = NetStats()


# ---------------------------------------------------------------------------
# Network interfaces (/sys/class/net, enriched with `ip`/`nmcli` when present)
# ---------------------------------------------------------------------------


def _read_text(path):
    try:
        with open(path, encoding="ascii") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def _run(args):
    try:
        result = subprocess.run(
            args, capture_output=True, text=True, timeout=_SUBPROCESS_TIMEOUT, check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout if result.returncode == 0 else None


def _run_raw(args):
    """Like :func:`_run`, but keeps stdout regardless of exit code - some
    tools (``tailscale status``) exit non-zero for a perfectly normal state
    (logged out, backend stopped) while still printing valid JSON."""
    try:
        result = subprocess.run(
            args, capture_output=True, text=True, timeout=_SUBPROCESS_TIMEOUT, check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout


_IP_ADDR_RE = re.compile(r"^\d+:\s+(\S+)\s+inet6?\s+(\S+)")


def _read_addrs_by_iface():
    """{iface: ["192.168.1.5/24", ...]} from `ip -o addr show`, or {}."""
    out = _run(["ip", "-o", "addr", "show"])
    if out is None:
        return {}

    addrs = {}
    for line in out.splitlines():
        match = _IP_ADDR_RE.match(line)
        if match:
            addrs.setdefault(match.group(1), []).append(match.group(2))
    return addrs


def _read_wifi_by_iface():
    """{iface: (ssid, signal_percent)} for the active Wi-Fi connection(s)."""
    out = _run(["nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL,DEVICE", "dev", "wifi"])
    if out is None:
        return {}

    wifi = {}
    for line in out.splitlines():
        parts = line.split(":")
        if len(parts) < 4 or parts[0] != "yes":
            continue
        ssid, signal, device = parts[-3], parts[-2], parts[-1]
        try:
            wifi[device] = (ssid, int(signal))
        except ValueError:
            wifi[device] = (ssid, -1)
    return wifi


def _iface_kind(name):
    if os.path.isdir(f"{_SYS_NET}/{name}/wireless") or os.path.isdir(f"{_SYS_NET}/{name}/phy80211"):
        return "wifi"
    if os.path.isdir(f"{_SYS_NET}/{name}/device"):
        return "ethernet"
    return "other"


_COUNTER_FIELDS = (
    "rx_bytes", "tx_bytes", "rx_packets", "tx_packets",
    "rx_errors", "tx_errors", "rx_dropped", "tx_dropped", "collisions",
)


def _read_iface_counters(name):
    counters = {}
    for field_name in _COUNTER_FIELDS:
        try:
            counters[field_name] = int(_read_text(f"{_SYS_NET}/{name}/statistics/{field_name}") or 0)
        except ValueError:
            counters[field_name] = 0
    return counters


#: Previous sample's (timestamp, rx_bytes, tx_bytes) per interface, so
#: _read_interfaces() can turn cumulative byte counters into a live rate.
#: Only ever touched from the netmon background thread, so it needs no lock
#: (unlike the public `stats` object, which the render thread also reads).
_prev_counters = {}


def _read_interfaces():
    try:
        names = sorted(os.listdir(_SYS_NET))
    except OSError:
        return None

    addrs_by_iface = _read_addrs_by_iface()
    wifi_by_iface = _read_wifi_by_iface()
    now = time.monotonic()

    interfaces = []
    for name in names:
        if name.startswith(_SKIP_IFACE_PREFIXES):
            continue

        kind = _iface_kind(name)
        if kind == "other":
            continue

        operstate = _read_text(f"{_SYS_NET}/{name}/operstate")
        carrier = _read_text(f"{_SYS_NET}/{name}/carrier") == "1"

        speed_mbps = -1
        if kind == "ethernet":
            raw_speed = _read_text(f"{_SYS_NET}/{name}/speed")
            try:
                speed_mbps = int(raw_speed)
            except ValueError:
                speed_mbps = -1

        ssid, signal_percent = wifi_by_iface.get(name, ("", -1))

        counters = _read_iface_counters(name)
        rx_bps = tx_bps = 0.0
        previous = _prev_counters.get(name)
        if previous is not None:
            prev_time, prev_rx, prev_tx = previous
            dt = now - prev_time
            if dt > 0:
                # max(0, ...): a counter reset (interface bounced) would
                # otherwise show as a bogus huge negative rate.
                rx_bps = max(0.0, (counters["rx_bytes"] - prev_rx) * 8 / dt)
                tx_bps = max(0.0, (counters["tx_bytes"] - prev_tx) * 8 / dt)
        _prev_counters[name] = (now, counters["rx_bytes"], counters["tx_bytes"])

        interfaces.append(NetInterface(
            name=name,
            kind=kind,
            up=operstate == "up",
            carrier=carrier,
            speed_mbps=speed_mbps,
            addrs=addrs_by_iface.get(name, []),
            ssid=ssid,
            signal_percent=signal_percent,
            rx_bps=rx_bps,
            tx_bps=tx_bps,
            rx_packets=counters["rx_packets"],
            tx_packets=counters["tx_packets"],
            rx_errors=counters["rx_errors"],
            tx_errors=counters["tx_errors"],
            rx_dropped=counters["rx_dropped"],
            tx_dropped=counters["tx_dropped"],
            collisions=counters["collisions"],
        ))

    return interfaces


# ---------------------------------------------------------------------------
# LAN neighbors (kernel ARP/NDP table - passive, no active network scanning)
# ---------------------------------------------------------------------------


def _parse_neigh_line(line):
    parts = line.split()
    if len(parts) < 3 or parts[1] != "dev":
        return None

    ip, iface, state = parts[0], parts[2], parts[-1].upper()
    mac = ""
    if "lladdr" in parts:
        idx = parts.index("lladdr")
        if idx + 1 < len(parts):
            mac = parts[idx + 1]

    return ip, iface, mac, state


_MAX_NEIGHBORS = 24


def _read_neighbors():
    out = _run(["ip", "neigh", "show"])
    if out is None:
        return None

    neighbors = []
    for line in out.splitlines():
        parsed = _parse_neigh_line(line)
        if parsed is None:
            continue
        ip, iface, mac, state = parsed
        if iface.startswith(_SKIP_IFACE_PREFIXES):
            continue
        neighbors.append(NeighborDevice(ip=ip, mac=mac, iface=iface, state=state))

    neighbors.sort(key=lambda n: n.ip)
    return neighbors[:_MAX_NEIGHBORS]


# ---------------------------------------------------------------------------
# USB devices (/sys/bus/usb/devices)
# ---------------------------------------------------------------------------


def _read_usb_devices():
    try:
        entries = os.listdir(_SYS_USB)
    except OSError:
        return None

    devices = []
    for entry in sorted(entries):
        # Interface sub-nodes look like "1-8:1.0"; root hubs are "usb1",
        # "usb2", ... - neither is a plugged-in peripheral.
        if ":" in entry or entry.startswith("usb"):
            continue

        base = f"{_SYS_USB}/{entry}"
        try:
            speed_mbps = float(_read_text(f"{base}/speed") or 0)
        except ValueError:
            speed_mbps = 0.0

        try:
            max_power_ma = int((_read_text(f"{base}/bMaxPower") or "0mA").rstrip("mA"))
        except ValueError:
            max_power_ma = 0

        devices.append(UsbDevice(
            location=entry,
            vendor_id=_read_text(f"{base}/idVendor"),
            product_id=_read_text(f"{base}/idProduct"),
            manufacturer=_read_text(f"{base}/manufacturer"),
            product=_read_text(f"{base}/product"),
            speed_mbps=speed_mbps,
            max_power_ma=max_power_ma,
        ))

    return devices


_DMESG_USB_RE = re.compile(r"usb|xhci|ehci|ohci", re.IGNORECASE)
_MAX_USB_WARNINGS = 6


def _read_usb_warnings():
    """Recent kernel USB reset/disconnect/error lines, or None if dmesg is
    unavailable (unreadable without root on many distros - degrade quietly)."""
    out = _run(["dmesg", "--ctime", "--level=err,warn"])
    if out is None:
        return None

    lines = [line for line in out.splitlines() if _DMESG_USB_RE.search(line)]
    return lines[-_MAX_USB_WARNINGS:]


# ---------------------------------------------------------------------------
# Tailscale (best-effort; `tailscale` may not be installed - that's fine)
# ---------------------------------------------------------------------------


def _read_tailscale():
    out = _run_raw(["tailscale", "status", "--json"])
    if out is None:
        return None

    try:
        data = json.loads(out)
    except ValueError:
        return TailscaleStatus(available=False)

    self_node = data.get("Self") or {}
    peers = data.get("Peer") or {}
    exit_node = ""
    for peer in peers.values():
        if peer.get("ExitNode"):
            exit_node = peer.get("HostName", "")
            break

    return TailscaleStatus(
        available=True,
        backend_state=data.get("BackendState", "Unknown"),
        hostname=self_node.get("HostName", ""),
        ips=data.get("TailscaleIPs") or [],
        exit_node=exit_node,
        peers_total=len(peers),
        peers_online=sum(1 for p in peers.values() if p.get("Online")),
    )


# ---------------------------------------------------------------------------
# Internet speed test - on-demand only (see run_speedtest_async()). Everything
# else in this module polls automatically every SAMPLE_INTERVAL; this does
# not, because it burns real bandwidth on whatever link this runs over -
# possibly the same radio carrying MAVLink/video in flight.
# ---------------------------------------------------------------------------

_SPEEDTEST_TIMEOUT = 60.0


def _run_speedtest():
    result = SpeedTestResult(completed=True)

    try:
        proc = subprocess.run(
            ["speedtest", "--json", "--secure"],
            capture_output=True, text=True, timeout=_SPEEDTEST_TIMEOUT, check=False,
        )
    except FileNotFoundError:
        result.error = "'speedtest' not found (pip install speedtest-cli)"
        proc = None
    except subprocess.SubprocessError as exc:
        result.error = str(exc)[:200]
        proc = None

    if proc is not None:
        if proc.returncode != 0 or not proc.stdout.strip():
            result.error = (proc.stderr or "speedtest failed").strip().splitlines()[-1][:200]
        else:
            try:
                data = json.loads(proc.stdout)
                result.download_mbps = data.get("download", 0.0) / 1e6
                result.upload_mbps = data.get("upload", 0.0) / 1e6
                result.ping_ms = data.get("ping", 0.0)
                server = data.get("server") or {}
                result.server = f"{server.get('sponsor', '')} ({server.get('name', '')})".strip()
            except (ValueError, AttributeError) as exc:
                result.error = f"couldn't parse speedtest output: {exc}"

    result.updated = time.monotonic()
    with stats.lock:
        stats.speedtest = result


def run_speedtest_async():
    """Kick off a real download/upload/ping test in the background.

    Never called automatically - only from a user keypress. Returns False
    (does nothing) if one is already running.
    """
    with stats.lock:
        if stats.speedtest.running:
            return False
        stats.speedtest = SpeedTestResult(running=True)

    threading.Thread(target=_run_speedtest, daemon=True, name="SpeedTestThread").start()
    return True


# ---------------------------------------------------------------------------
# Background thread
# ---------------------------------------------------------------------------


def _sample():
    interfaces = _read_interfaces()
    neighbors = _read_neighbors()
    usb_devices = _read_usb_devices()
    usb_warnings = _read_usb_warnings()
    tailscale = _read_tailscale()

    with stats.lock:
        if interfaces is not None:
            stats.ok = True
            stats.interfaces = interfaces
        if neighbors is not None:
            stats.neighbors_ok = True
            stats.neighbors = neighbors
        if usb_devices is not None:
            stats.usb_ok = True
            stats.usb_devices = usb_devices
        if usb_warnings is not None:
            stats.usb_warnings_available = True
            stats.usb_warnings = usb_warnings
        if tailscale is not None:
            stats.tailscale = tailscale
        stats.updated = time.monotonic()


def netmon_thread():
    if shutil.which("ip") is None:
        log_warn("netmon: 'ip' not found - interface IP addresses won't be shown")

    warned = False
    while not shutdown_event.is_set():
        try:
            _sample()
        except Exception as exc:  # never let host monitoring take down the app
            if not warned:
                warned = True
                log_warn(f"Network/USB monitor error: {exc}")
        time.sleep(SAMPLE_INTERVAL)
