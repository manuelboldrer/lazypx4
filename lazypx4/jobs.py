"""Generic background subprocess runner for one-shot utility actions.

Firmware flashing (``px_uploader.py``), the .ulog web upload
(``upload_log.py``) and the ecl_ekf health-check
(``process_logdata_ekf.py``) all just run a standalone script to completion
and report a result - none of them are something the vehicle link depends
on, and only one is ever meaningfully in flight at a time. Routing all three
through one job runner (and one ``state.job_*`` slot, see :mod:`lazypx4.state`)
keeps their screens' code identical instead of three near-duplicate copies of
the same "run a subprocess, stream its output, show a status line" logic.
"""

from __future__ import annotations

import subprocess
import threading
import time

from .eventlog import log_command, log_error, log_info, log_warn
from .state import state

_ACTIVE_PROC = None
_ACTIVE_PROC_LOCK = threading.Lock()


def job_running():
    with state.lock:
        return state.job_active


def start_job(kind, cmd, label, on_success=None, cwd=None):
    """Run ``cmd`` (an argv list) in the background under ``label``.

    ``on_success(output_text)`` runs on the worker thread once the process
    exits with status 0, and its return value (if any) becomes
    ``state.job_result``. Raising from it fails the job with that exception's
    message - used by callers that only know the run truly succeeded once
    they can parse the process's own output (e.g. the uploaded log's URL).
    """
    with state.lock:
        if state.job_active:
            log_warn(f"{label}: another background job is already running")
            return False

        state.job_active = True
        state.job_kind = kind
        state.job_cancel = False
        state.job_lines.clear()
        state.job_status = "RUNNING"
        state.job_error = ""
        state.job_result = ""
        state.job_started_at = time.monotonic()

    threading.Thread(
        target=_run_job, args=(cmd, on_success, cwd, label),
        daemon=True, name=f"Job-{kind}",
    ).start()

    log_command(f"{label} started")
    return True


def cancel_job():
    with state.lock:
        if not state.job_active:
            return False
        state.job_cancel = True

    with _ACTIVE_PROC_LOCK:
        proc = _ACTIVE_PROC
    if proc is not None:
        try:
            proc.terminate()
        except Exception:
            pass

    log_command("Job cancellation requested")
    return True


def _append_line(line):
    if not line:
        return
    with state.lock:
        state.job_lines.append(line)


def _run_job(cmd, on_success, cwd, label):
    global _ACTIVE_PROC

    output_lines = []

    try:
        proc = subprocess.Popen(
            cmd, cwd=cwd,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
    except Exception as exc:
        with state.lock:
            state.job_active = False
            state.job_status = "ERROR"
            state.job_error = str(exc)
        log_error(f"{label} failed to start: {exc}")
        return

    with _ACTIVE_PROC_LOCK:
        _ACTIVE_PROC = proc

    try:
        buf = bytearray()
        while True:
            chunk = proc.stdout.read(1)
            if not chunk:
                break
            if chunk in (b"\r", b"\n"):
                line = buf.decode("utf-8", errors="replace").strip()
                buf.clear()
                if line:
                    _append_line(line)
                    output_lines.append(line)
            else:
                buf.extend(chunk)

        if buf:
            line = buf.decode("utf-8", errors="replace").strip()
            if line:
                _append_line(line)
                output_lines.append(line)

        returncode = proc.wait(timeout=15)
    except Exception as exc:
        try:
            proc.kill()
        except Exception:
            pass
        with state.lock:
            state.job_active = False
            state.job_status = "ERROR"
            state.job_error = str(exc)
        log_error(f"{label} failed: {exc}")
        return
    finally:
        with _ACTIVE_PROC_LOCK:
            _ACTIVE_PROC = None

    with state.lock:
        cancelled = state.job_cancel

    if cancelled:
        with state.lock:
            state.job_active = False
            state.job_status = "CANCELLED"
        log_warn(f"{label} cancelled")
        return

    if returncode != 0:
        error_text = output_lines[-1] if output_lines else f"exit code {returncode}"
        with state.lock:
            state.job_active = False
            state.job_status = "ERROR"
            state.job_error = error_text
        log_error(f"{label} failed: {error_text}")
        return

    full_output = "\n".join(output_lines)
    result_text = ""

    if on_success is not None:
        try:
            result_text = on_success(full_output) or ""
        except Exception as exc:
            with state.lock:
                state.job_active = False
                state.job_status = "ERROR"
                state.job_error = str(exc)
            log_error(f"{label}: {exc}")
            return

    with state.lock:
        state.job_active = False
        state.job_status = "SUCCESS"
        state.job_result = result_text

    log_info(f"{label}: done" + (f" - {result_text}" if result_text else ""))
