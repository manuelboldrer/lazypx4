"""The [f] flash-firmware screen: pick a .px4 file and a serial port, then
flash it to the flight controller via Tools/px_uploader.py."""

from __future__ import annotations

import os
import time

from ..ansi import BOLD, DIM, GREEN, RED, RESET, YELLOW
from ..config import settings
from ..state import state
from .jobpanel import job_panel_lines


def _file_size_string(path):
    try:
        size = os.path.getsize(path)
    except OSError:
        return "--"
    if size >= 1024 * 1024:
        return f"{size / (1024 * 1024):.1f} MiB"
    if size >= 1024:
        return f"{size / 1024:.1f} KiB"
    return f"{size} B"


def _file_mtime_string(path):
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(path)))
    except OSError:
        return "--"


def draw_firmware_screen():
    with state.lock:
        files = list(state.firmware_files)
        file_index = state.firmware_index
        ports = list(state.firmware_ports)
        port_index = state.firmware_port_index
        job_active = state.job_active and state.job_kind == "firmware"

    lines = []

    if ports:
        port_index = min(port_index, len(ports) - 1)
        port_text = GREEN + ports[port_index] + RESET
        if len(ports) > 1:
            port_text += DIM + f"   ({port_index + 1}/{len(ports)}, LEFT/RIGHT to change)" + RESET
    else:
        port_text = RED + "no serial port detected - plug in the flight controller" + RESET

    lines.append(f" Target port: {port_text}")
    lines.append(f" Firmware directory: {settings.firmware_dir}")
    lines.append("")
    lines.append(DIM + f" {'':>3} {'FILE':<40} {'SIZE':>10}  {'MODIFIED':<20}" + RESET)
    lines.append(DIM + " " + "─" * 74 + RESET)

    if not files:
        lines.append(DIM + f" No .px4 firmware files found in {settings.firmware_dir}" + RESET)
    else:
        file_index = min(file_index, len(files) - 1)
        for i, path in enumerate(files):
            selected = i == file_index
            marker = ">" if selected else " "
            name = os.path.basename(path)
            line = (
                f" {marker} {name:<40} {_file_size_string(path):>10}"
                f"  {_file_mtime_string(path):<20}"
            )
            if selected:
                lines.append(BOLD + GREEN + line + RESET)
            else:
                lines.append(line)

    lines.extend(job_panel_lines("firmware"))

    lines.append("")
    if job_active:
        lines.append(BOLD + YELLOW + "Flashing in progress - ESC cancels" + RESET)
    else:
        lines.append("UP/DOWN select firmware    LEFT/RIGHT select port    r = refresh")
        lines.append("ENTER = flash selected firmware    f = back    ESC = panels")

    return lines
