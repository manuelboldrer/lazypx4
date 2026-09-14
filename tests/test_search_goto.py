"""Tests for the "/" search, mode-list paging and the "goto" command."""

from __future__ import annotations

import math
import time

import pytest

from lazypx4 import navigation, search
from lazypx4.mavlink import guided
from lazypx4.models import Parameter
from lazypx4.render.chrome import highlight_matches
from lazypx4.state import session, state


# ---------------------------------------------------------------------------
# highlight_matches
# ---------------------------------------------------------------------------


def test_highlight_wraps_each_match_case_insensitively():
    out = highlight_matches("EKF2_GPS_CTRL and ekf2_baro", "ekf2")
    assert out.count("\033[7m") == 2
    assert out.count("\033[27m") == 2
    # the visible text is unchanged
    assert out.replace("\033[7m", "").replace("\033[27m", "") == "EKF2_GPS_CTRL and ekf2_baro"


def test_highlight_noop_without_query():
    assert highlight_matches("anything", "") == "anything"


# ---------------------------------------------------------------------------
# search over the parameter list
# ---------------------------------------------------------------------------


def _load_params(names):
    with state.lock:
        state.parameters.clear()
        state.parameter_order.clear()
        for n in names:
            state.parameters[n] = Parameter(name=n, value=1.0, param_type=6)
        state.parameter_order.extend(names)
        state.parameter_index = 0
        state.parameter_view = "ALL"


def test_search_jump_moves_parameter_cursor():
    _load_params(["COM_RC_IN_MODE", "EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "MPC_XY_VEL_MAX"])
    session.screen = "parameters"
    session.search_query = "baro"

    search.jump(0)
    with state.lock:
        assert state.parameter_order[state.parameter_index] == "EKF2_BARO_CTRL"

    # n from there wraps back to the only match
    search.jump(1)
    with state.lock:
        assert state.parameter_order[state.parameter_index] == "EKF2_BARO_CTRL"

    session.search_query = "ekf2"
    with state.lock:
        state.parameter_index = 0
    search.jump(1)  # first EKF2 after index 0
    with state.lock:
        first = state.parameter_order[state.parameter_index]
    search.jump(1)  # next EKF2
    with state.lock:
        second = state.parameter_order[state.parameter_index]
    assert {first, second} == {"EKF2_GPS_CTRL", "EKF2_BARO_CTRL"}


def test_search_footer_reports_match_count():
    _load_params(["EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "MPC_XY_VEL_MAX"])
    session.screen = "parameters"
    session.search_active = False
    session.search_query = "ekf2"
    assert "2 match" in search.footer_line()
    session.search_query = "nope"
    assert "no matches" in search.footer_line()
    session.search_query = ""
    assert search.footer_line() == ""


def test_process_search_input_builds_query_and_clears_on_esc():
    _load_params(["EKF2_GPS_CTRL"])
    session.screen = "parameters"
    session.search_active = True
    session.search_query = ""
    for ch in "ekf":
        navigation.process_search(ch)
    assert session.search_query == "ekf"
    navigation.process_search("ESC")
    assert session.search_query == "" and session.search_active is False


# ---------------------------------------------------------------------------
# goto  (MAV_CMD_DO_REPOSITION)
# ---------------------------------------------------------------------------


class _FakeMav:
    def __init__(self):
        self.command_int = []
        self.command_long = []

    def command_int_send(self, *args):
        self.command_int.append(args)

    def command_long_send(self, *args):
        self.command_long.append(args)


class _FakeLink:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = _FakeMav()


def _ready_state(**overrides):
    with state.lock:
        state.vehicle_locked = True
        state.last_rx = time.monotonic()
        state.armed = True
        state.global_pos_valid = True
        state.global_lat = 47.0
        state.global_lon = 8.0
        state.global_alt = 500.0   # AMSL
        state.z = 12.0             # relative alt
        state.yaw = 0.0
        state.jog_armed = False
        state.jog_step = 1.0
        for k, v in overrides.items():
            setattr(state, k, v)


def test_goto_forward_moves_north_and_keeps_altitude():
    _ready_state(yaw=0.0)
    link = _FakeLink()
    assert guided.send_goto_body(link, 100.0, 0.0, 0.0) is True

    (args,) = link.mav.command_int
    frame, command = args[2], args[3]
    x, y, z = args[10], args[11], args[12]
    from pymavlink import mavutil

    assert command == mavutil.mavlink.MAV_CMD_DO_REPOSITION
    assert frame == mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    assert x == pytest.approx(int(round((47.0 + 100.0 / 111320.0) * 1e7)), abs=2)
    assert y == pytest.approx(int(round(8.0 * 1e7)), abs=2)
    assert z == pytest.approx(500.0)  # down = 0 -> unchanged AMSL, not the ground


def test_goto_forward_moves_east_when_heading_east_and_can_climb():
    _ready_state(yaw=90.0)
    link = _FakeLink()
    guided.send_goto_body(link, 100.0, 0.0, -5.0)  # also climb 5 m

    (args,) = link.mav.command_int
    x, y, z = args[10], args[11], args[12]
    assert x == pytest.approx(int(round(47.0 * 1e7)), abs=3)
    assert y > int(round(8.0 * 1e7))
    assert z == pytest.approx(505.0)


def test_goto_refused_without_global_position():
    _ready_state(global_pos_valid=False)
    link = _FakeLink()
    assert guided.send_goto_body(link, 10.0, 0.0, 0.0) is False
    assert link.mav.command_int == []


def test_goto_submit_parses_and_opens_confirmation():
    _ready_state()
    session.link = _FakeLink()
    session.confirm_active = False
    navigation._goto_submit("10 -5 0")
    assert session.confirm_active is True
    assert "fwd +10.0" in session.confirm_text and "right -5.0" in session.confirm_text
    navigation.cancel_confirmation()


def test_goto_submit_rejects_garbage():
    session.confirm_active = False
    navigation._goto_submit("not numbers")
    assert session.confirm_active is False


# ---------------------------------------------------------------------------
# takeoff / land / RTL
# ---------------------------------------------------------------------------


def test_takeoff_climbs_above_current_altitude():
    _ready_state(z=0.0)
    link = _FakeLink()
    assert guided.send_takeoff(link, 5.0) is True
    (args,) = link.mav.command_int
    from pymavlink import mavutil

    assert args[3] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert args[12] == pytest.approx(5.0)  # relative target altitude


def test_takeoff_refused_when_disarmed():
    _ready_state(armed=False)
    link = _FakeLink()
    assert guided.send_takeoff(link, 5.0) is False
    assert link.mav.command_int == []


def test_land_and_rtl_send_commands():
    _ready_state()
    link = _FakeLink()

    assert guided.send_land(link) is True
    assert guided.send_rtl(link) is True

    from pymavlink import mavutil

    sent_int = [a[3] for a in link.mav.command_int]
    sent_long = [a[2] for a in link.mav.command_long]
    assert mavutil.mavlink.MAV_CMD_NAV_LAND in sent_int  # land: global position present
    assert mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH in sent_long


def test_takeoff_submit_default_altitude_opens_confirmation():
    _ready_state()
    session.link = _FakeLink()
    session.confirm_active = False
    navigation._takeoff_submit("")
    assert session.confirm_active is True
    assert f"{navigation.TAKEOFF_DEFAULT_ALT:.1f} m" in session.confirm_text
    navigation.cancel_confirmation()


# ---------------------------------------------------------------------------
# keyboard jog
# ---------------------------------------------------------------------------


def _armed_jog():
    _ready_state()
    session.link = _FakeLink()
    session.screen = "map"
    session.jog_armed = True
    session.jog_step = 1.0
    navigation._jog_last_nudge = 0.0


def test_jog_forward_sends_one_reposition_step():
    _armed_jog()
    assert navigation._jog_key("k") is True
    (args,) = session.link.mav.command_int
    from pymavlink import mavutil

    assert args[3] == mavutil.mavlink.MAV_CMD_DO_REPOSITION
    # heading north, 1 m forward -> latitude increases by ~1 m
    assert args[10] == pytest.approx(int(round((47.0 + 1.0 / 111320.0) * 1e7)), abs=2)


def test_jog_up_climbs_and_step_scales():
    _armed_jog()
    navigation._jog_key("]")                  # step 1 -> 2 m
    assert session.jog_step == 2.0
    navigation._jog_key("w")                  # up 2 m
    (args,) = session.link.mav.command_int
    assert args[12] == pytest.approx(500.0 + 2.0)  # AMSL + 2


def test_jog_yaw_only_keeps_position():
    _armed_jog()
    navigation._jog_key("h")                  # yaw left
    (args,) = session.link.mav.command_int
    assert args[10] == pytest.approx(int(round(47.0 * 1e7)), abs=2)   # no move
    assert args[11] == pytest.approx(int(round(8.0 * 1e7)), abs=2)
    # yaw param is in radians (MAV_CMD_DO_REPOSITION spec), not degrees
    assert args[9] == pytest.approx(math.radians(345.0))  # h -> yaw - 15 -> 345 (counter-clockwise/left)

    navigation._jog_last_nudge = 0.0
    navigation._jog_key("l")                  # yaw right -> yaw + 15 (clockwise/right)
    assert session.link.mav.command_int[-1][9] == pytest.approx(math.radians(15.0))


def test_jog_throttles_rapid_presses():
    _armed_jog()
    navigation._jog_key("k")
    navigation._jog_key("k")                  # immediately after -> throttled
    assert len(session.link.mav.command_int) == 1


def test_jog_key_not_consumed_when_not_a_move():
    _armed_jog()
    assert navigation._jog_key("t") is False   # 't' is a map key, not jog


def test_toggle_jog_off_is_immediate():
    _armed_jog()
    session.confirm_active = False
    navigation.toggle_jog()
    assert session.jog_armed is False
    assert session.confirm_active is False


def test_toggle_jog_on_requires_armed_and_confirms():
    _ready_state(armed=False)
    session.jog_armed = False
    session.confirm_active = False
    navigation.toggle_jog()
    assert session.confirm_active is False      # blocked: not armed

    _ready_state(armed=True)
    session.jog_armed = False
    navigation.toggle_jog()
    assert session.confirm_active is True       # asks "type YES"
    navigation.cancel_confirmation()
