"""Offline smoke tests - no MAVLink connection, no real terminal."""

from __future__ import annotations

import io
from contextlib import redirect_stdout

import pytest

from lazypx4 import cli, eventlog
from lazypx4.mavlink import handlers, parameters
from lazypx4.models import Parameter
from lazypx4.render import DRAW_FUNCTIONS, draw
from lazypx4.state import session, state


class FakeMsg:
    """Minimal stand-in for a decoded pymavlink message."""

    def __init__(self, msg_type, src_system=1, src_component=1, **fields):
        self._type = msg_type
        self._src_system = src_system
        self._src_component = src_component
        self.__dict__.update(fields)

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return self._src_system

    def get_srcComponent(self):
        return self._src_component


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def test_cli_parses_and_applies_settings():
    parser = cli._build_parser()
    args = parser.parse_args(["--port", "14999", "--log-dir", "/tmp/x", "--map-dir", "/tmp/y"])
    assert args.port == 14999
    assert args.log_dir == "/tmp/x"
    assert args.map_dir == "/tmp/y"
    assert args.allow_log_download_while_armed is False


def test_cli_version_exits(capsys):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--version"])
    assert exc.value.code == 0
    assert "lazypx4" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# STATUSTEXT classification
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "severity, text, expected",
    [
        (6, "Ready to fly", "INFO"),
        (6, "ARMING DENIED: preflight checks", "ERROR"),
        (4, "Low battery, return home", "WARN"),
        (2, "RC LOSS - failsafe triggered", "FAILSAFE"),
        (1, "some critical thing", "ERROR"),
    ],
)
def test_classify_statustext(severity, text, expected):
    assert eventlog.classify_statustext(severity, text) == expected


# ---------------------------------------------------------------------------
# Parameter float<->int reinterpretation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("param_type", [5, 6])  # UINT32, INT32
@pytest.mark.parametrize("value", [0, 1, 42, 123456, 2_000_000_000])
def test_param_int_roundtrip(param_type, value):
    wire = parameters.encode_number_as_param_float(value, param_type)
    back = parameters.decode_param_float_as_number(wire, param_type)
    assert int(round(back)) == value


def test_param_real32_passthrough():
    wire = parameters.encode_number_as_param_float(3.5, 9)
    assert parameters.decode_param_float_as_number(wire, 9) == pytest.approx(3.5)


def test_parameter_format_and_parse():
    p = Parameter(name="COM_RC_IN_MODE", value=1.0, param_type=6)
    assert parameters.parameter_format_value(p) == "1"
    assert parameters.parse_parameter_value("2", p) == 2.0
    with pytest.raises(ValueError):
        parameters.parse_parameter_value("2.5", p)


# ---------------------------------------------------------------------------
# Telemetry handlers fold messages into state
# ---------------------------------------------------------------------------


def test_handle_heartbeat_locks_and_tracks_mode():
    with state.lock:
        state.vehicle_locked = False
        state.armed = False

    msg = FakeMsg(
        "HEARTBEAT",
        autopilot=12,      # MAV_AUTOPILOT_PX4, not INVALID
        base_mode=0,
        custom_mode=0,
        system_status=4,
    )
    handlers.handle_heartbeat(msg)

    with state.lock:
        assert state.vehicle_locked is True
        assert state.target_system == 1


def test_handle_attitude_converts_to_degrees():
    import math

    handlers.handle_attitude(FakeMsg("ATTITUDE", roll=math.pi / 2, pitch=0.0, yaw=0.0))
    with state.lock:
        assert state.roll == pytest.approx(90.0)


def test_handle_statustext_logs_event():
    with state.lock:
        before = len(state.events)
    handlers.handle_statustext(FakeMsg("STATUSTEXT", severity=6, text="hello world"))
    with state.lock:
        assert len(state.events) == before + 1
        assert "hello world" in state.events[-1].message


# ---------------------------------------------------------------------------
# Every screen renders without raising
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("screen", sorted(DRAW_FUNCTIONS))
def test_screen_renders(screen):
    session.screen = screen
    buf = io.StringIO()
    with redirect_stdout(buf):
        draw()
    # draw() swallows screen errors and falls back to the dashboard, so also
    # assert it did not silently log a render failure.
    with state.lock:
        recent = [e.message for e in list(state.events)[-5:]]
    assert not any("failed to render" in m for m in recent), recent


def test_dispatch_table_is_complete():
    from lazypx4.mavlink import receiver

    for name, fn in receiver._DISPATCH.items():
        assert callable(fn), name
