"""Tests for the [n] position map's trail: sampling spacing/cap
(lazypx4.mavlink.handlers._store_local_position) and its Braille-packed
rendering (lazypx4.render.mapview)."""

from __future__ import annotations

from lazypx4.config import POSITION_TRAIL_MAXLEN, POSITION_TRAIL_MIN_SPACING_M
from lazypx4.mavlink.handlers import _store_local_position
from lazypx4.state import state


def _reset_trail():
    with state.lock:
        state.position_trail.clear()
        state.local_pos_valid = False


def test_trail_skips_points_closer_than_min_spacing():
    _reset_trail()
    with state.lock:
        _store_local_position(0.0, 0.0, 0.0, 1.0)
        _store_local_position(POSITION_TRAIL_MIN_SPACING_M * 0.5, 0.0, 0.0, 1.1)

    with state.lock:
        assert list(state.position_trail) == [(0.0, 0.0)]


def test_trail_keeps_points_at_or_past_min_spacing():
    _reset_trail()
    with state.lock:
        _store_local_position(0.0, 0.0, 0.0, 1.0)
        _store_local_position(POSITION_TRAIL_MIN_SPACING_M, 0.0, 0.0, 1.1)

    with state.lock:
        assert list(state.position_trail) == [(0.0, 0.0), (POSITION_TRAIL_MIN_SPACING_M, 0.0)]


def test_trail_is_capped_at_maxlen():
    _reset_trail()
    with state.lock:
        for i in range(POSITION_TRAIL_MAXLEN + 50):
            _store_local_position(i * POSITION_TRAIL_MIN_SPACING_M * 2, 0.0, 0.0, float(i))

    with state.lock:
        assert len(state.position_trail) == POSITION_TRAIL_MAXLEN
        # the oldest points were dropped, not the newest
        assert state.position_trail[-1][0] > state.position_trail[0][0]


def test_map_screen_renders_a_dense_trail_as_braille():
    import math

    from lazypx4.render.mapview import draw_map_screen
    from lazypx4.state import session

    _reset_trail()
    with state.lock:
        state.local_pos_valid = True
        state.map_trail_enabled = True
        state.map_range = 15.0
        state.local_x = state.local_y = state.local_z = 0.0
        for i in range(300):
            t = i * 0.05
            n, e = 8.0 * math.sin(t), 8.0 * math.cos(t) - 8.0
            state.position_trail.append((n, e))

    session.screen = "map"
    lines = draw_map_screen()

    # A Braille cell (U+2800-U+28FF) somewhere in the grid means the dense
    # trail got packed into sub-cell dots rather than collapsing to nothing
    # or raising - the actual glyphs vary with the grid's exact size/zoom.
    body = "".join(lines)
    assert any(0x2800 <= ord(ch) <= 0x28FF for ch in body)
