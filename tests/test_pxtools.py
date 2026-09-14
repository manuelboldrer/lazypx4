"""Tests for the generic job runner (lazypx4.jobs) and the PX4 tool-script
wrappers (lazypx4.pxtools) behind the [f] flash-firmware and the [u]/[a]
flight-log-screen actions - no real serial port, network or ulog required."""

from __future__ import annotations

import sys
import time

import pytest

from lazypx4 import jobs, navigation, pxtools
from lazypx4.config import settings
from lazypx4.state import session, state


@pytest.fixture(autouse=True)
def _reset_job_state():
    with state.lock:
        state.job_active = False
        state.job_kind = ""
        state.job_cancel = False
        state.job_lines.clear()
        state.job_status = "IDLE"
        state.job_error = ""
        state.job_result = ""
        state.firmware_files = []
        state.firmware_index = 0
        state.firmware_ports = []
        state.firmware_port_index = 0
    yield
    with state.lock:
        state.job_active = False
        state.job_cancel = False


def _wait_until_done(timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with state.lock:
            if not state.job_active:
                return
        time.sleep(0.02)
    raise AssertionError("job did not finish in time")


# ---------------------------------------------------------------------------
# lazypx4.jobs
# ---------------------------------------------------------------------------


def test_start_job_streams_output_and_reports_success():
    cmd = [sys.executable, "-c", "print('line one'); print('line two')"]
    assert jobs.start_job("test", cmd, "unit test job")
    _wait_until_done()

    with state.lock:
        assert state.job_status == "SUCCESS"
        assert list(state.job_lines) == ["line one", "line two"]


def test_start_job_refuses_when_one_already_running():
    slow_cmd = [sys.executable, "-c", "import time; time.sleep(0.2)"]
    assert jobs.start_job("test", slow_cmd, "slow job")
    assert jobs.job_running() is True
    assert jobs.start_job("test", [sys.executable, "-c", "print('x')"], "second job") is False
    _wait_until_done()


def test_start_job_nonzero_exit_is_reported_as_error():
    cmd = [sys.executable, "-c", "print('boom'); raise SystemExit(1)"]
    assert jobs.start_job("test", cmd, "failing job")
    _wait_until_done()

    with state.lock:
        assert state.job_status == "ERROR"
        assert state.job_error


def test_on_success_return_value_becomes_job_result():
    cmd = [sys.executable, "-c", "print('hello')"]
    assert jobs.start_job("test", cmd, "job with callback", on_success=lambda out: out.upper())
    _wait_until_done()

    with state.lock:
        assert state.job_status == "SUCCESS"
        assert state.job_result == "HELLO"


def test_on_success_exception_fails_the_job():
    def _boom(_output):
        raise RuntimeError("post-processing exploded")

    cmd = [sys.executable, "-c", "print('ok')"]
    assert jobs.start_job("test", cmd, "job with bad callback", on_success=_boom)
    _wait_until_done()

    with state.lock:
        assert state.job_status == "ERROR"
        assert "post-processing exploded" in state.job_error


def test_cancel_job_with_nothing_running_returns_false():
    assert jobs.cancel_job() is False


# ---------------------------------------------------------------------------
# lazypx4.pxtools - firmware discovery
# ---------------------------------------------------------------------------


def test_discover_firmware_files_lists_px4_files_newest_first(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "firmware_dir", str(tmp_path))

    old = tmp_path / "old.px4"
    old.write_bytes(b"old")
    new = tmp_path / "new.px4"
    new.write_bytes(b"new")
    (tmp_path / "ignored.txt").write_bytes(b"nope")

    import os
    now = time.time()
    os.utime(old, (now - 100, now - 100))
    os.utime(new, (now, now))

    files = pxtools.discover_firmware_files()
    assert files == [str(new), str(old)]


def test_discover_firmware_files_missing_dir_returns_empty(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "firmware_dir", str(tmp_path / "does-not-exist"))
    assert pxtools.discover_firmware_files() == []


def test_discover_serial_ports_never_raises():
    # No real hardware in CI/dev sandboxes - this just must not blow up, and
    # must return a list (possibly empty).
    assert isinstance(pxtools.discover_serial_ports(), list)


# ---------------------------------------------------------------------------
# lazypx4.pxtools - missing script guards
# ---------------------------------------------------------------------------


def test_start_firmware_flash_missing_script_fails_cleanly(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "tools_dir", str(tmp_path))
    assert pxtools.start_firmware_flash("fw.px4", "/dev/ttyACM0") is False


def test_start_ulog_upload_missing_script_fails_cleanly(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "tools_dir", str(tmp_path))
    assert pxtools.start_ulog_upload("log.ulg") is False


def test_start_ecl_ekf_check_missing_script_fails_cleanly(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "tools_dir", str(tmp_path))
    assert pxtools.start_ecl_ekf_check("log.ulg") is False


def test_url_regex_extracts_plot_url():
    match = pxtools._URL_RE.search(
        "Uploading foo.ulg...\nURL: https://logs.px4.io/plot_app?log=abc-123\n"
    )
    assert match.group(1) == "https://logs.px4.io/plot_app?log=abc-123"


# ---------------------------------------------------------------------------
# navigation.py wiring
# ---------------------------------------------------------------------------


def test_upload_last_flight_log_without_a_download_warns(monkeypatch):
    with state.lock:
        state.flight_log_download_status = "IDLE"
        state.flight_log_download_path = ""
        before = len(state.events)

    navigation.upload_last_flight_log()

    with state.lock:
        assert len(state.events) == before + 1
        assert "Download a flight log first" in state.events[-1].message


def test_analyze_last_flight_log_without_a_download_warns():
    with state.lock:
        state.flight_log_download_status = "IDLE"
        state.flight_log_download_path = ""
        before = len(state.events)

    navigation.analyze_last_flight_log()

    with state.lock:
        assert len(state.events) == before + 1
        assert "Download a flight log first" in state.events[-1].message


def test_upload_last_flight_log_uses_completed_download(tmp_path, monkeypatch):
    log_path = tmp_path / "px4_log_000001.ulg"
    log_path.write_bytes(b"not a real ulog")

    with state.lock:
        state.flight_log_download_status = "COMPLETE"
        state.flight_log_download_path = str(log_path)

    called = {}

    def _fake_start_ulog_upload(path):
        called["path"] = path
        return True

    monkeypatch.setattr(navigation, "start_ulog_upload", _fake_start_ulog_upload)

    navigation.upload_last_flight_log()

    assert called["path"] == str(log_path)


def test_refresh_firmware_lists_populates_state(monkeypatch):
    monkeypatch.setattr(navigation, "discover_firmware_files", lambda: ["/tmp/a.px4", "/tmp/b.px4"])
    monkeypatch.setattr(navigation, "discover_serial_ports", lambda: ["/dev/ttyACM0"])

    navigation.refresh_firmware_lists()

    with state.lock:
        assert state.firmware_files == ["/tmp/a.px4", "/tmp/b.px4"]
        assert state.firmware_ports == ["/dev/ttyACM0"]


def test_handle_firmware_key_enter_without_files_warns(monkeypatch):
    monkeypatch.setattr(navigation, "discover_firmware_files", lambda: [])
    monkeypatch.setattr(navigation, "discover_serial_ports", lambda: [])
    navigation.refresh_firmware_lists()

    with state.lock:
        before = len(state.events)

    navigation.handle_firmware_key("ENTER")

    with state.lock:
        assert len(state.events) == before + 1
        assert "No .px4 firmware files found" in state.events[-1].message


def test_handle_firmware_key_enter_while_armed_refuses(monkeypatch):
    monkeypatch.setattr(navigation, "discover_firmware_files", lambda: ["/tmp/a.px4"])
    monkeypatch.setattr(navigation, "discover_serial_ports", lambda: ["/dev/ttyACM0"])
    navigation.refresh_firmware_lists()

    with state.lock:
        state.armed = True
        before = len(state.events)

    try:
        navigation.handle_firmware_key("ENTER")
        with state.lock:
            assert len(state.events) == before + 1
            assert "ARMED" in state.events[-1].message
            assert session.confirm_active is False
    finally:
        with state.lock:
            state.armed = False


def test_handle_firmware_key_enter_opens_confirmation(monkeypatch):
    monkeypatch.setattr(navigation, "discover_firmware_files", lambda: ["/tmp/a.px4"])
    monkeypatch.setattr(navigation, "discover_serial_ports", lambda: ["/dev/ttyACM0"])
    navigation.refresh_firmware_lists()

    with state.lock:
        state.armed = False

    navigation.handle_firmware_key("ENTER")

    assert session.confirm_active is True
    assert "a.px4" in session.confirm_text
    assert "/dev/ttyACM0" in session.confirm_text

    navigation.cancel_confirmation()
