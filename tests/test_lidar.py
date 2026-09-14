"""Tests for the LiDAR point-cloud overview (lazypx4.lidar) and the [v]
screen's navigation wiring. Most of this is built entirely from fake
PointCloud2-shaped objects and needs no rclpy/sensor_msgs/ROS environment;
the one test that does (runtime topic re-subscription) skips itself where
none is sourced."""

from __future__ import annotations

import struct
import time

import pytest

from lazypx4 import lidar, navigation
from lazypx4.state import session, state


class FakeField:
    def __init__(self, name, offset, datatype):
        self.name = name
        self.offset = offset
        self.datatype = datatype


class FakeCloud:
    """Minimal stand-in for a decoded sensor_msgs/msg/PointCloud2."""

    def __init__(self, points):
        # x, y, z as float32, packed little-endian, one point per row.
        self.fields = [
            FakeField("x", 0, 7),
            FakeField("y", 4, 7),
            FakeField("z", 8, 7),
        ]
        self.point_step = 12
        self.width = len(points)
        self.height = 1
        self.is_bigendian = False
        self.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in points)


@pytest.fixture(autouse=True)
def _reset_lidar_state():
    with state.lock:
        state.lidar_supported = False
        state.lidar_frame_id = ""
        state.lidar_point_count = 0
        state.lidar_rate_hz = 0.0
        state.lidar_last_received = 0.0
        state.lidar_sample_x = []
        state.lidar_sample_y = []
        state.lidar_sample_z = []
        state.lidar_range_min = 0.0
        state.lidar_range_max = 0.0
        state.lidar_view_range = 10.0
        state.lidar_view_mode = "top"
    yield


# ---------------------------------------------------------------------------
# _decode_points
# ---------------------------------------------------------------------------


def test_decode_points_basic():
    points = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (-1.0, -2.0, -3.0)]
    cloud = FakeCloud(points)

    count, xs, ys, zs = lidar._decode_points(cloud, max_points=100)

    assert count == 3
    assert xs == pytest.approx([1.0, 4.0, -1.0])
    assert ys == pytest.approx([2.0, 5.0, -2.0])
    assert zs == pytest.approx([3.0, 6.0, -3.0])


def test_decode_points_decimates_to_max_points():
    points = [(float(i), float(i), float(i)) for i in range(1000)]
    cloud = FakeCloud(points)

    count, xs, ys, zs = lidar._decode_points(cloud, max_points=100)

    assert count == 1000          # the message's real point count is kept
    assert len(xs) <= 100         # but the returned sample is capped
    assert len(xs) == len(ys) == len(zs)


def test_decode_points_drops_non_finite():
    points = [(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0), (float("inf"), 1.0, 1.0)]
    cloud = FakeCloud(points)

    count, xs, ys, zs = lidar._decode_points(cloud, max_points=100)

    assert count == 3
    assert xs == pytest.approx([1.0])
    assert ys == pytest.approx([2.0])
    assert zs == pytest.approx([3.0])


def test_decode_points_missing_xyz_fields_returns_empty():
    cloud = FakeCloud([(1.0, 2.0, 3.0)])
    cloud.fields = [FakeField("intensity", 0, 7)]

    assert lidar._decode_points(cloud) == (0, [], [], [])


def test_decode_points_empty_cloud():
    cloud = FakeCloud([])
    assert lidar._decode_points(cloud) == (0, [], [], [])


def test_decode_points_bigendian():
    points = [(1.5, -2.5, 3.5)]
    cloud = FakeCloud(points)
    cloud.is_bigendian = True
    cloud.data = b"".join(struct.pack(">fff", x, y, z) for x, y, z in points)

    count, xs, ys, zs = lidar._decode_points(cloud, max_points=10)

    assert count == 1
    assert xs == pytest.approx([1.5])
    assert ys == pytest.approx([-2.5])
    assert zs == pytest.approx([3.5])


def test_decode_points_without_numpy_returns_empty(monkeypatch):
    monkeypatch.setattr(lidar, "np", None)
    cloud = FakeCloud([(1.0, 2.0, 3.0)])
    assert lidar._decode_points(cloud) == (0, [], [], [])


# ---------------------------------------------------------------------------
# navigation.py wiring
# ---------------------------------------------------------------------------


def test_open_pointcloud_screen_sets_screen():
    navigation.open_pointcloud_screen()
    assert session.screen == "pointcloud"


def test_pointcloud_zoom_keys_clamp_range():
    with state.lock:
        state.lidar_view_range = 10.0

    navigation.handle_pointcloud_key("+")
    with state.lock:
        assert state.lidar_view_range == pytest.approx(10.0 / 1.5)

    navigation.handle_pointcloud_key("0")
    with state.lock:
        assert state.lidar_view_range == pytest.approx(10.0)

    navigation.handle_pointcloud_key("-")
    with state.lock:
        assert state.lidar_view_range == pytest.approx(15.0)


def test_pointcloud_v_key_returns_to_dashboard():
    session.screen = "pointcloud"
    navigation.handle_pointcloud_key("v")
    assert session.screen == "dashboard"


def test_pointcloud_screen_renders_with_sample_points():
    with state.lock:
        state.lidar_supported = True
        state.lidar_frame_id = "lidar_frame"
        state.lidar_point_count = 500
        state.lidar_sample_x = [1.0, -1.0, 0.0]
        state.lidar_sample_y = [0.5, -0.5, 0.0]
        state.lidar_sample_z = [0.0, 0.0, 0.0]
        state.lidar_range_min = 0.5
        state.lidar_range_max = 1.5

    from lazypx4.render.pointcloud import draw_pointcloud_screen

    lines = draw_pointcloud_screen()
    assert any("lidar_frame" in line for line in lines)
    assert any("Points/msg: 500" in line for line in lines)
    assert any("Height Z:" in line for line in lines)


# ---------------------------------------------------------------------------
# render/pointcloud.py - Z color gradient
# ---------------------------------------------------------------------------


def test_z_color_is_low_at_min_and_high_at_max():
    from lazypx4.render.pointcloud import _COLOR_BANDS, _color_for

    assert _color_for(0.0, 0.0, 10.0) == _COLOR_BANDS[0]
    assert _color_for(10.0, 0.0, 10.0) == _COLOR_BANDS[-1]
    assert _color_for(5.0, 0.0, 10.0) not in (_COLOR_BANDS[0], _COLOR_BANDS[-1])


def test_z_color_flat_range_does_not_divide_by_zero():
    from lazypx4.render.pointcloud import _COLOR_BANDS, _color_for

    assert _color_for(3.0, 3.0, 3.0) == _COLOR_BANDS[0]


def test_pointcloud_screen_colors_tallest_point_per_cell():
    with state.lock:
        state.lidar_supported = True
        state.lidar_frame_id = "lidar_frame"
        state.lidar_view_mode = "top"
        # Two points that land in the same Braille character cell at a
        # small view range - the taller one (z=5.0) must win that cell's
        # color, even though both dots are set within the one glyph.
        state.lidar_sample_x = [1.0, 1.0]
        state.lidar_sample_y = [1.0, 1.0]
        state.lidar_sample_z = [0.0, 5.0]
        state.lidar_range_min = 1.0
        state.lidar_range_max = 1.5
        state.lidar_view_range = 2.0

    from lazypx4.render.pointcloud import _color_for, draw_pointcloud_screen

    lines = draw_pointcloud_screen()
    tall_color = _color_for(5.0, 0.0, 5.0)
    assert any(tall_color in line for line in lines)


# ---------------------------------------------------------------------------
# render/pointcloud.py - top vs. front view switching
# ---------------------------------------------------------------------------


def test_pointcloud_view_keys_switch_mode():
    with state.lock:
        state.lidar_view_mode = "top"

    navigation.handle_pointcloud_key("2")
    with state.lock:
        assert state.lidar_view_mode == "front"

    navigation.handle_pointcloud_key("1")
    with state.lock:
        assert state.lidar_view_mode == "top"


def test_pointcloud_screen_renders_in_front_view():
    with state.lock:
        state.lidar_supported = True
        state.lidar_frame_id = "lidar_frame"
        state.lidar_view_mode = "front"
        state.lidar_sample_x = [1.0, 2.0, -1.0]
        state.lidar_sample_y = [0.5, -0.5, 0.0]
        state.lidar_sample_z = [0.2, -0.2, 1.0]
        state.lidar_range_min = 0.5
        state.lidar_range_max = 2.0

    from lazypx4.render.pointcloud import draw_pointcloud_screen

    lines = draw_pointcloud_screen()
    assert any("Front" in line for line in lines)
    assert any("Depth X" in line for line in lines)


def test_pointcloud_key_3_switches_to_oblique_view():
    with state.lock:
        state.lidar_view_mode = "top"

    navigation.handle_pointcloud_key("3")
    with state.lock:
        assert state.lidar_view_mode == "oblique"


def test_pointcloud_screen_renders_in_oblique_view():
    with state.lock:
        state.lidar_supported = True
        state.lidar_frame_id = "lidar_frame"
        state.lidar_view_mode = "oblique"
        state.lidar_sample_x = [1.0, 2.0, -1.0]
        state.lidar_sample_y = [0.5, -0.5, 0.0]
        state.lidar_sample_z = [0.2, -0.2, 1.0]
        state.lidar_range_min = 0.5
        state.lidar_range_max = 2.0

    from lazypx4.render.pointcloud import draw_pointcloud_screen

    lines = draw_pointcloud_screen()
    assert any("oblique" in line for line in lines)
    assert any("Perp. axis" in line for line in lines)


# ---------------------------------------------------------------------------
# navigation.py - changing the point-cloud topic at runtime
# ---------------------------------------------------------------------------


def test_pointcloud_topic_key_opens_input_prompt(monkeypatch):
    from lazypx4.config import settings

    monkeypatch.setattr(settings, "lidar_topic", "/livox/points")
    navigation.cancel_confirmation()
    session.input_active = False

    navigation.handle_pointcloud_key("t")

    assert session.input_active is True
    assert "/livox/points" in session.input_prompt


def test_pointcloud_topic_submit_updates_settings(monkeypatch):
    from lazypx4.config import settings

    monkeypatch.setattr(settings, "lidar_topic", "/livox/points")

    navigation._pointcloud_topic_submit("/other/points")

    assert settings.lidar_topic == "/other/points"


def test_pointcloud_topic_submit_adds_leading_slash(monkeypatch):
    from lazypx4.config import settings

    monkeypatch.setattr(settings, "lidar_topic", "/livox/points")

    navigation._pointcloud_topic_submit("cloud")

    assert settings.lidar_topic == "/cloud"


def test_pointcloud_topic_submit_ignores_blank_input(monkeypatch):
    from lazypx4.config import settings

    monkeypatch.setattr(settings, "lidar_topic", "/livox/points")

    navigation._pointcloud_topic_submit("   ")

    assert settings.lidar_topic == "/livox/points"


@pytest.mark.skipif(not lidar.lidar_available(), reason="no ROS 2 environment sourced")
def test_lidar_thread_resubscribes_on_topic_change():
    """End-to-end against a real rclpy node/executor (this sandbox has ROS 2
    sourced): the thread's own loop must notice settings.lidar_topic
    changing and re-subscribe in place, without restarting the thread."""
    import threading

    from lazypx4.config import settings
    from lazypx4.state import shutdown_event

    original_topic = settings.lidar_topic
    settings.lidar_topic = "/lazypx4_test/points_a"

    thread = threading.Thread(target=lidar.lidar_thread, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with state.lock:
                if state.lidar_supported:
                    break
            time.sleep(0.05)
        with state.lock:
            assert state.lidar_supported is True

        with state.lock:
            state.lidar_frame_id = "stale_from_topic_a"

        settings.lidar_topic = "/lazypx4_test/points_b"

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with state.lock:
                if state.lidar_frame_id != "stale_from_topic_a":
                    break
            time.sleep(0.05)

        with state.lock:
            # _subscribe() resets this to "" on every re-subscribe, whether
            # or not a message has arrived on the new topic yet.
            assert state.lidar_frame_id == ""
    finally:
        shutdown_event.set()
        thread.join(timeout=5)
        shutdown_event.clear()  # this Event is a process-wide singleton
        settings.lidar_topic = original_topic
