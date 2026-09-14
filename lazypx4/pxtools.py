"""Wraps the standalone PX4 scripts under ``settings.tools_dir`` as
background jobs (see :mod:`lazypx4.jobs`):

* ``px_uploader.py`` - flash a ``.px4`` firmware file to the flight
  controller over serial (flash-firmware screen, [f]).
* ``upload_log.py`` - upload a downloaded ``.ulog`` to the PX4 flight-review
  web server and copy the resulting plot URL to the clipboard (flight-logs
  screen, [u]).
* ``ecl_ekf/process_logdata_ekf.py`` - run the ecl_ekf health-check on a
  downloaded ``.ulog`` and report Pass/Warning/Fail (flight-logs screen,
  [a]).

Each of these is a real, independent PX4 tool script rather than something
reimplemented here - lazypx4 shells out to it under a real Python
interpreter (see :func:`_tool_python`) and interprets its output.
"""

from __future__ import annotations

import glob
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

from .config import settings
from .eventlog import log_error, log_info, log_warn
from .jobs import start_job


def _tool_python():
    """The Python interpreter to run a ``Tools/`` script under.

    ``sys.executable`` is only a Python interpreter when lazypx4 itself is
    running as an ordinary Python process (a checkout's ``.venv``, or a
    plain ``pip install``) - there it is exactly right, since that's the
    environment the ``tools`` extra in pyproject.toml installs
    pyserial/requests/pyulog/etc. into. A PyInstaller ``--onefile`` build's
    ``sys.executable`` is the frozen lazypx4 binary itself: handing it a
    script path as an argument does not run that script, it just re-launches
    the lazypx4 UI ignoring the extra arguments. Frozen builds fall back to
    a real ``python3`` off PATH instead, which is where these scripts'
    "pip install ..." hints already point a user who is missing a module.
    """
    if not getattr(sys, "frozen", False):
        return sys.executable
    return shutil.which("python3") or shutil.which("python") or "python3"


def _git_email():
    try:
        output = subprocess.check_output(
            ["git", "config", "--global", "user.email"],
            stderr=subprocess.DEVNULL, timeout=2,
        )
        return output.decode("utf-8").strip()
    except Exception:
        return ""


def copy_to_clipboard(text):
    """Best-effort copy of ``text`` to the Wayland clipboard via wl-copy."""
    try:
        subprocess.run(
            ["wl-copy"], input=text.encode("utf-8"), timeout=3, check=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        return True
    except Exception as exc:
        log_warn(f"Could not copy to clipboard (wl-copy): {exc}")
        return False


# ---------------------------------------------------------------------------
# Firmware flashing (px_uploader.py)
# ---------------------------------------------------------------------------


def discover_firmware_files():
    directory = Path(settings.firmware_dir)
    if not directory.is_dir():
        return []

    files = sorted(
        (p for p in directory.glob("*.px4") if p.is_file()),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return [str(p) for p in files]


def discover_serial_ports():
    """USB serial ports a flight controller could plausibly be on.

    A PX4 board always shows up as a USB CDC-ACM (or USB-serial-bridge)
    device, never as one of a PC's onboard ``/dev/ttyS*`` UARTs - so those
    are filtered out rather than left to bury the one real port in a wall of
    ports nothing will ever answer on.
    """
    ports = []
    try:
        from serial.tools import list_ports
        ports = [
            info.device for info in list_ports.comports()
            if info.vid is not None or "USB" in (info.hwid or "")
        ]
    except Exception:
        ports = []

    if not ports:
        ports = sorted(set(
            glob.glob("/dev/serial/by-id/*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("/dev/ttyUSB*")
        ))

    return ports


def start_firmware_flash(firmware_path, port):
    uploader = os.path.join(settings.tools_dir, "px_uploader.py")
    if not os.path.isfile(uploader):
        log_error(f"px_uploader.py not found under {settings.tools_dir}")
        return False

    cmd = [_tool_python(), uploader, "--port", port, firmware_path]
    label = f"Firmware flash ({os.path.basename(firmware_path)} -> {port})"
    return start_job("firmware", cmd, label)


# ---------------------------------------------------------------------------
# .ulog web upload (upload_log.py)
# ---------------------------------------------------------------------------

_URL_RE = re.compile(r"URL:\s*(\S+)")


def start_ulog_upload(log_path):
    script = os.path.join(settings.tools_dir, "upload_log.py")
    if not os.path.isfile(script):
        log_error(f"upload_log.py not found under {settings.tools_dir}")
        return False

    email = _git_email()
    cmd = [
        _tool_python(), script, "--quiet",
        "--server", settings.ulog_upload_server,
        "--description", "", "--feedback", "",
        "--source", "lazypx4",
    ]
    if email:
        cmd += ["--email", email]
    cmd.append(log_path)

    def _on_success(output_text):
        match = _URL_RE.search(output_text)
        if not match:
            raise RuntimeError("upload_log.py did not report a plot URL")
        url = match.group(1)
        if copy_to_clipboard(url):
            log_info(f"Log report URL copied to clipboard: {url}")
        return url

    label = f"Uploading {os.path.basename(log_path)} to {settings.ulog_upload_server}"
    return start_job("ulog_upload", cmd, label, on_success=_on_success)


# ---------------------------------------------------------------------------
# ecl_ekf health-check (ecl_ekf/process_logdata_ekf.py)
# ---------------------------------------------------------------------------


def start_ecl_ekf_check(log_path):
    script = os.path.join(settings.tools_dir, "ecl_ekf", "process_logdata_ekf.py")
    if not os.path.isfile(script):
        log_error(f"process_logdata_ekf.py not found under {settings.tools_dir}")
        return False

    cmd = [_tool_python(), script, log_path]

    def _on_success(output_text):
        if "Minor anomalies detected" in output_text:
            verdict = "WARNING - minor anomalies detected"
        elif "No anomalies detected" in output_text:
            verdict = "PASS - no anomalies detected"
        else:
            verdict = "completed"

        report_pdf = f"{log_path}-0.pdf"
        if os.path.isfile(report_pdf):
            verdict += f" - report: {report_pdf}"
        return verdict

    label = f"EKF health-check on {os.path.basename(log_path)}"
    return start_job("ecl_ekf", cmd, label, on_success=_on_success)
