"""The shared, mutable world: vehicle state, UI session, and the queues.

Two singletons:

* :data:`state` - everything the MAVLink receiver learns about the vehicle.
  It is written from the receiver thread and the flight-log worker thread and
  read from the main/render thread, so every access is guarded by
  ``state.lock`` (a re-entrant lock).

* :data:`session` - UI navigation state (which screen is active, scroll
  offsets, the confirmation prompt) plus the live MAVLink connection. Only
  the main thread touches it, so it needs no lock.
"""

from __future__ import annotations

import queue
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

from .config import MAX_LOG_EVENTS, POSITION_TRAIL_MAXLEN, SHELL_MAX_LINES
from .models import PendingArm


@dataclass
class State:
    target_system: int = 0
    target_component: int = 0
    vehicle_locked: bool = False

    connected: bool = False
    last_heartbeat: float = 0.0
    last_rx: float = 0.0

    # Autopilot wall clock (SYSTEM_TIME). PX4 reports 0 until it has a wall-
    # clock source (usually GPS lock), so unix_usec == 0 means "not yet known"
    # rather than the epoch.
    autopilot_unix_usec: int = 0
    autopilot_boot_ms: int = 0
    last_system_time: float = 0.0

    # ROS 2 clock, sampled by a standalone rclpy node (lazypx4.rosclock) so it
    # can be shown next to the autopilot's own clock. ``ros_clock_supported``
    # is False when rclpy could not be imported (no ROS 2 environment
    # sourced); ``ros_time_ns`` stays 0 while sim time is enabled but no
    # /clock publisher has been seen yet.
    ros_clock_supported: bool = False
    ros_time_ns: int = 0
    ros_time_sim: bool = False
    last_ros_time: float = 0.0

    armed: bool = False
    mode: str = "UNKNOWN"
    base_mode: int = 0
    custom_mode: int = 0
    system_status: int = 0
    available_modes: list = field(default_factory=list)

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    last_position: float = 0.0

    # Dedicated local NED position (LOCAL_POSITION_NED / ODOMETRY only),
    # kept separate from x/y so the plan-view map is never fed lat/lon.
    local_x: float = 0.0
    local_y: float = 0.0
    local_z: float = 0.0
    local_pos_valid: bool = False
    last_local_pos: float = 0.0
    position_trail: deque = field(default_factory=lambda: deque(maxlen=POSITION_TRAIL_MAXLEN))

    # Global position (GLOBAL_POSITION_INT) for the map's lat/lon readout.
    global_lat: float = 0.0
    global_lon: float = 0.0
    global_alt: float = 0.0
    global_pos_valid: bool = False

    map_range: float = 30.0
    map_trail_enabled: bool = True

    # Satellite-image snapshot download.
    map_download_active: bool = False
    map_download_status: str = "IDLE"
    map_download_path: str = ""
    map_download_error: str = ""
    map_download_started_at: float = 0.0

    # Sensor calibration (MAV_CMD_PREFLIGHT_CALIBRATION + "[cal]" STATUSTEXT).
    cal_active: bool = False
    cal_type: str = ""
    cal_progress: int = 0
    cal_result: str = ""
    cal_last_message: str = ""
    cal_last_message_at: float = 0.0
    cal_started_at: float = 0.0
    cal_sent_at: float = 0.0
    cal_ack: str = ""
    cal_ack_at: float = 0.0
    cal_statustext_at_start: int = 0
    cal_event_at_start: int = 0
    cal_log: deque = field(default_factory=lambda: deque(maxlen=14))

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Body angular rates from ATTITUDE (deg/s).
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    battery: float = -1.0
    voltage: float = 0.0
    current: float = 0.0

    thrust: float = 0.0

    groundspeed: float = 0.0
    airspeed: float = 0.0
    climb_rate: float = 0.0

    # Attitude / rate setpoints (ATTITUDE_TARGET).
    att_target_roll: float = 0.0
    att_target_pitch: float = 0.0
    att_target_yaw: float = 0.0
    att_target_thrust: float = 0.0
    att_target_roll_rate: float = 0.0
    att_target_pitch_rate: float = 0.0
    att_target_yaw_rate: float = 0.0
    last_att_target: float = 0.0

    # Position / velocity setpoints (POSITION_TARGET_LOCAL_NED).
    pos_target_x: float = 0.0
    pos_target_y: float = 0.0
    pos_target_z: float = 0.0
    pos_target_vx: float = 0.0
    pos_target_vy: float = 0.0
    pos_target_vz: float = 0.0
    pos_target_yaw: float = 0.0
    pos_target_yaw_rate: float = 0.0
    pos_target_type_mask: int = 0
    pos_target_frame: int = 0
    last_pos_target: float = 0.0

    # Guidance errors (NAV_CONTROLLER_OUTPUT).
    nav_roll: float = 0.0
    nav_pitch: float = 0.0
    nav_bearing: float = 0.0
    nav_target_bearing: float = 0.0
    nav_wp_dist: float = 0.0
    nav_alt_error: float = 0.0
    nav_aspd_error: float = 0.0
    nav_xtrack_error: float = 0.0
    last_nav_output: float = 0.0

    # Altitude (ALTITUDE message; falls back to VFR_HUD alt).
    alt_amsl: float = 0.0
    alt_relative: float = 0.0
    alt_local: float = 0.0
    alt_monotonic: float = 0.0
    alt_terrain: float = 0.0
    alt_bottom_clearance: float = 0.0
    last_altitude: float = 0.0
    vfr_alt: float = 0.0

    # Barometer (SCALED_PRESSURE).
    baro_pressure: float = 0.0
    baro_temp: float = 0.0
    last_baro: float = 0.0

    # Downward-facing rangefinder (DISTANCE_SENSOR).
    rangefinder_distance: float = 0.0
    rangefinder_min: float = 0.0
    rangefinder_max: float = 0.0
    rangefinder_quality: int = -1
    rangefinder_orientation: int = -1
    last_rangefinder: float = 0.0

    # Local-position origin (GPS_GLOBAL_ORIGIN).
    local_origin_set: bool = False
    local_origin_lat: float = 0.0
    local_origin_lon: float = 0.0
    local_origin_alt: float = 0.0

    # Home / launch position (HOME_POSITION). Used as the reference for the
    # [g] goto on the map screen.
    home_set: bool = False
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0

    gps_fix: int = 0
    gps_sats: int = 0
    gps_hdop: float = 0.0
    gps_vdop: float = 0.0
    gps_alt: float = 0.0
    gps_h_acc: float = 0.0
    gps_v_acc: float = 0.0
    gps_vel_acc: float = 0.0
    gps_speed: float = 0.0
    gps_cog: float = 0.0
    gps_alt_ellipsoid: float = 0.0
    # Dual-antenna GPS heading (GPS_RAW_INT.yaw/hdg_acc) - stays -1.0 ("n/a")
    # on single-antenna receivers, which never populate this field.
    gps_heading: float = -1.0
    gps_heading_acc: float = 0.0
    last_gps: float = 0.0

    gps2_fix: int = 0
    gps2_sats: int = 0
    gps2_hdop: float = 0.0
    gps_dgps_age: float = 0.0
    gps_dgps_numch: int = 0
    last_gps2: float = 0.0

    # RTK detail (GPS_RTK / GPS2_RTK). Populated only when the GNSS receiver
    # is fed RTCM corrections (from a GCS / NTRIP link - this console does not
    # inject them itself).
    rtk_health: int = -1
    rtk_rate: float = 0.0
    rtk_nsats: int = 0
    rtk_baseline_m: float = 0.0
    rtk_iar_hypotheses: int = 0
    last_gps_rtk: float = 0.0

    ekf_flags: int = 0
    ekf_status: str = "UNKNOWN"
    last_ekf: float = 0.0

    # Estimator health detail (ESTIMATOR_STATUS / EKF_STATUS_REPORT).
    est_is_estimator_status: bool = False
    est_vel_ratio: float = 0.0
    est_pos_horiz_ratio: float = 0.0
    est_pos_vert_ratio: float = 0.0
    est_mag_ratio: float = 0.0
    est_hagl_ratio: float = 0.0
    est_tas_ratio: float = 0.0
    est_pos_horiz_accuracy: float = 0.0
    est_pos_vert_accuracy: float = 0.0
    ekf_vel_variance: float = 0.0
    ekf_pos_horiz_variance: float = 0.0
    ekf_pos_vert_variance: float = 0.0
    ekf_compass_variance: float = 0.0
    ekf_terrain_variance: float = 0.0

    rc_received: bool = False
    rc_rssi: int = 0
    rc_lq: int = 0
    rc_failsafe: bool = False
    rc_channels: list = field(default_factory=lambda: [0] * 18)
    last_rc: float = 0.0

    imu: bool = False
    mag: bool = False
    baro: bool = False
    gps_sensor: bool = False

    # Full SYS_STATUS sensor bitmasks (onboard_control_sensors_{present,
    # enabled,health}) - the 4 booleans above are a legacy subset of `health`
    # kept for the older call sites; render.chrome decodes these three for
    # the full per-sensor arming-health list.
    sensors_present: int = 0
    sensors_enabled: int = 0
    sensors_health: int = 0
    last_sys_status: float = 0.0

    # IMU vibration (VIBRATION message) - excessive levels or clipping mean
    # unbalanced props / a loose FC mount, worth catching before flight.
    vibration_x: float = 0.0
    vibration_y: float = 0.0
    vibration_z: float = 0.0
    clipping_0: int = 0
    clipping_1: int = 0
    clipping_2: int = 0
    last_vibration: float = 0.0

    # Flashed firmware (AUTOPILOT_VERSION, requested once at connect).
    fw_version_text: str = ""
    fw_git_hash: str = ""
    autopilot_version_received: bool = False

    # Most recent "Preflight Fail" / "PREARM" STATUSTEXT - PX4's own arming
    # gate reason, otherwise buried in the scrolling event log. Cleared on a
    # successful arm (see handle_heartbeat).
    last_preflight_fail: str = ""
    last_preflight_fail_time: float = 0.0

    rx_messages: int = 0
    rx_rate: float = 0.0

    statustext_count: int = 0
    event_count: int = 0

    events: deque = field(default_factory=lambda: deque(maxlen=MAX_LOG_EVENTS))

    warning_count: int = 0
    error_count: int = 0
    failsafe_count: int = 0

    last_ack: str = ""
    last_ack_time: float = 0.0

    last_status_text: str = ""
    last_status_level: str = ""

    pending_arm: Optional[PendingArm] = None

    heartbeat_timeout_active: bool = False
    position_timeout_active: bool = False
    gps_timeout_active: bool = False
    ekf_timeout_active: bool = False
    rc_timeout_active: bool = False

    battery_low_active: bool = False
    battery_critical_active: bool = False

    parameters: dict = field(default_factory=dict)
    parameter_order: list = field(default_factory=list)
    parameter_count: int = 0
    parameters_received: int = 0
    parameters_requested_at: float = 0.0
    parameters_complete: bool = False
    parameter_defaults: dict = field(default_factory=dict)
    parameter_full_list_requested: bool = False
    estimator_params_requested_at: float = 0.0
    parameter_view: str = "ALL"
    parameter_index: int = 0
    parameter_page: int = 0
    parameter_edit_name: str = ""
    parameter_edit_buffer: str = ""
    parameter_edit_active: bool = False
    parameter_set_pending: bool = False
    parameter_set_name: str = ""
    parameter_set_value: Optional[float] = None
    parameter_set_sent_at: float = 0.0

    # Flight modes (Standard Modes Protocol / AVAILABLE_MODES).
    custom_modes: dict = field(default_factory=dict)
    custom_modes_order: list = field(default_factory=list)
    custom_modes_total: int = 0
    custom_modes_received: int = 0
    custom_modes_requested_at: float = 0.0
    custom_modes_complete: bool = False

    available_modes_seq: int = -1

    current_standard_mode: int = 0
    current_custom_mode_id: int = 0

    # MAVLink shell (NSH console).
    shell_active: bool = False
    shell_lines: deque = field(default_factory=lambda: deque(maxlen=SHELL_MAX_LINES))
    shell_partial: str = ""
    shell_last_activity: float = 0.0

    # PX4 flight-log download.
    flight_logs: dict = field(default_factory=dict)
    flight_log_order: list = field(default_factory=list)
    flight_log_list_requested_at: float = 0.0
    flight_log_list_last_entry_at: float = 0.0
    flight_log_list_complete: bool = False
    flight_log_index: int = 0

    flight_log_download_active: bool = False
    flight_log_download_cancel: bool = False
    flight_log_download_id: int = -1
    flight_log_download_size: int = 0
    flight_log_download_received: int = 0
    flight_log_download_offset: int = 0
    flight_log_download_retry: int = 0
    flight_log_download_started_at: float = 0.0
    flight_log_download_speed: float = 0.0
    flight_log_download_path: str = ""
    flight_log_download_status: str = "IDLE"
    flight_log_download_error: str = ""

    # Generic background job runner - firmware flashing (px_uploader.py),
    # .ulog web upload (upload_log.py) and the ecl_ekf health-check
    # (process_logdata_ekf.py) all share this one slot instead of three
    # near-duplicate sets of fields, since they are manual, occasional
    # actions and only one of them is ever meaningfully running at a time.
    # See lazypx4.jobs.
    job_active: bool = False
    job_kind: str = ""
    job_cancel: bool = False
    job_lines: deque = field(default_factory=lambda: deque(maxlen=300))
    job_status: str = "IDLE"
    job_error: str = ""
    job_result: str = ""
    job_started_at: float = 0.0

    # Flash-firmware screen: discovered .px4 files / serial ports and the
    # cursor position into each list.
    firmware_files: list = field(default_factory=list)
    firmware_index: int = 0
    firmware_ports: list = field(default_factory=list)
    firmware_port_index: int = 0

    # LiDAR point-cloud overview ([v] screen) - see lazypx4.lidar. Runs its
    # own rclpy node independent of the ROS clock's, so `lidar_supported`
    # tracks separately from `ros_clock_supported`.
    lidar_supported: bool = False
    lidar_frame_id: str = ""
    lidar_point_count: int = 0
    lidar_rate_hz: float = 0.0
    lidar_last_received: float = 0.0
    lidar_sample_x: list = field(default_factory=list)
    lidar_sample_y: list = field(default_factory=list)
    lidar_sample_z: list = field(default_factory=list)
    lidar_range_min: float = 0.0
    lidar_range_max: float = 0.0
    lidar_view_range: float = 10.0
    #: "top" (bird's eye, X/Y), "front" (elevation, Y/Z), "oblique"
    #: (45-degree, both at once) or "free" (freely rotated with hjkl) - see
    #: lazypx4.render.pointcloud._VIEWS and _free_view().
    lidar_view_mode: str = "top"
    #: The mode [c] restores on toggling the free camera back off.
    lidar_prev_view_mode: str = "top"
    #: Free camera orientation (degrees), rotated with hjkl while
    #: `lidar_view_mode == "free"` - see lazypx4.render.pointcloud._free_view().
    lidar_cam_yaw: float = 45.0
    lidar_cam_pitch: float = 30.0

    lock: threading.RLock = field(default_factory=threading.RLock)


@dataclass
class Session:
    """UI navigation state and the live MAVLink connection (main thread only)."""

    #: The active ``mavutil`` connection, set once :func:`connect` succeeds.
    link: object = None

    #: Name of the visible screen; keys of ``render.DRAW_FUNCTIONS``.
    screen: str = "dashboard"

    # Lazygit/lazydocker-style panel layout: which box has keyboard focus
    # ("main", the default - all existing per-screen keys behave exactly as
    # before) or "sidebar" (arrows/jk move `nav_index`, ENTER opens that
    # panel). Toggled with TAB; see navigation.py's `_handle_sidebar_key`.
    nav_focus: str = "main"
    nav_index: int = 0

    # Vertical scroll for a screen with no navigable list of its own
    # (dashboard, estimation, control, calibration) whose content is taller
    # than the main panel - jk/UP/DOWN move it, chrome.draw_frame() clamps it
    # and resets it to 0 whenever `main_scroll_screen` no longer matches the
    # active screen (i.e. you switched screens since it was last set).
    main_scroll: int = 0
    main_scroll_screen: str = ""

    mode_index: int = 0
    log_scroll: int = 0
    shell_scroll: int = 0
    shell_follow: bool = True

    #: Keyboard jog on the map screen: armed state and current step (metres).
    jog_armed: bool = False
    jog_step: float = 1.0

    # Confirmation prompt ("type YES") overlaid on the current screen.
    confirm_active: bool = False
    confirm_text: str = ""
    confirm_buffer: str = ""
    confirm_callback: Optional[Callable[[], None]] = None

    # Generic single-line text input (used by "goto"), overlaid like the
    # confirmation prompt. ``input_callback`` is called with the entered text.
    input_active: bool = False
    input_prompt: str = ""
    input_buffer: str = ""
    input_callback: Optional[Callable[[str], None]] = None

    # Incremental search ("/"): highlight matches on the active list screen and
    # jump the cursor between them with n / N. ``search_active`` is the typing
    # state; ``search_query`` is the committed term.
    search_active: bool = False
    search_query: str = ""

    # Set by the SIGWINCH handler; consumed once by draw_lines() to force a
    # full-screen clear on the next frame. Without it, shrinking the terminal
    # can leave stale characters from the previous, larger frame on screen.
    resize_pending: bool = False


state = State()
session = Session()

shutdown_event = threading.Event()
key_queue: "queue.Queue" = queue.Queue()

# LOG_DATA packets are handed straight from the receiver thread to the active
# flight-log download worker through this queue - never through ``state`` or
# its lock (see lazypx4.mavlink.flightlog for why).
flight_log_queue: "queue.Queue" = queue.Queue()
