"""Tunables, protocol constants and lookup tables.

Nothing in here talks to MAVLink or touches shared state. Values that the
command-line interface can override live on the :data:`settings` object;
everything else is a fixed constant.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil


# ---------------------------------------------------------------------------
# Runtime-configurable settings (see lazypx4.cli)
# ---------------------------------------------------------------------------


@dataclass
class Settings:
    """User-overridable settings, populated once at start-up."""

    #: UDP port to listen on for the PX4 MAVLink connection.
    port: int = 14560

    #: Directory downloaded ``.ulg`` flight logs are written to.
    log_dir: str = "./px4_logs"

    #: Directory satellite-image snapshots are written to.
    map_dir: str = "./px4_maps"

    #: Filesystem path whose usage is shown in the dashboard HOST block.
    disk_path: str = "/"

    #: Optional JSON file of PX4 parameter defaults (enables the
    #: "changed from default" view on the parameter screen).
    param_defaults_file: Optional[str] = None

    #: Downloading a closed flight log is read-only, but LOG_REQUEST_LIST /
    #: LOG_REQUEST_DATA are intended for log retrieval rather than live
    #: logging. The conservative default is to refuse a transfer while armed.
    allow_log_download_while_armed: bool = False

    #: Directory scanned for ``.px4`` firmware files on the flash-firmware
    #: screen ([f]).
    firmware_dir: str = "./px4_firmware"

    #: Directory holding the standalone PX4 scripts the flash-firmware and
    #: flight-log screens shell out to: ``px_uploader.py`` (firmware
    #: flashing), ``upload_log.py`` (.ulog web upload) and
    #: ``ecl_ekf/process_logdata_ekf.py`` (EKF health-check report). Relative
    #: to the current working directory, same as ``log_dir``/``map_dir``/
    #: ``firmware_dir`` above - lazypx4 is normally run from inside the
    #: ``lazypx4/`` project directory (as a checkout, or as the standalone
    #: binary from ``build_binary.sh`` copied onto ``$PATH`` but still run
    #: from there), which is exactly where ``Tools/`` sits one level up.
    tools_dir: str = "../Tools"

    #: Server ``upload_log.py`` posts a downloaded .ulog to for the
    #: flight-log screen's [u] "upload to web" action.
    ulog_upload_server: str = "https://logs.px4.io"

    #: ROS 2 topic (sensor_msgs/PointCloud2) the [v] point-cloud screen
    #: subscribes to - see :mod:`lazypx4.lidar`.
    lidar_topic: str = "/livox/points"

    #: V4L2 device the [w] camera screen captures from - see
    #: :mod:`lazypx4.camera`. Never opened until the user turns the feed on.
    camera_device: str = "/dev/video0"


settings = Settings()


# ---------------------------------------------------------------------------
# Timing / layout
# ---------------------------------------------------------------------------

HEARTBEAT_TIMEOUT = 3.0
TELEMETRY_TIMEOUT = 3.0

REFRESH_HZ = 10.0

# Below this the fixed-width layout can't fit; show a "too small"
# placeholder instead of letting lines wrap and scroll the alt screen.
MIN_TERMINAL_COLS = 60
MIN_TERMINAL_ROWS = 16

ARM_CONFIRM_TIMEOUT = 5.0

MAX_LOG_EVENTS = 500
LOG_PAGE_SIZE = 25

BATTERY_LOW = 30.0
BATTERY_CRITICAL = 15.0

# ---------------------------------------------------------------------------
# Dashboard health-colour thresholds
#
# Each pair is (GOOD, OK): at/beyond GOOD a value renders green, between OK
# and GOOD it renders yellow, and past OK it renders red. "Lower is better"
# metrics (HDOP, position accuracy) compare the other way - see call sites.
# ---------------------------------------------------------------------------

GPS_HDOP_GOOD = 1.5
GPS_HDOP_OK = 3.0

GPS_SATS_GOOD = 8
GPS_SATS_OK = 6

# GPS / estimator horizontal & vertical position accuracy, metres.
POS_ACC_GOOD = 1.0
POS_ACC_OK = 3.0

# RC link signal strength / quality, percent.
RC_SIGNAL_GOOD = 70
RC_SIGNAL_OK = 30

# RC_CHANNELS raw PWM microsecond range - the near-universal RC convention
# (1000 = stick/lever full one way, 1500 = centre, 2000 = full the other way).
# A channel can overtravel slightly past 1000/2000 depending on transmitter
# endpoint calibration, hence the wider display clamp on the [r] RC screen.
RC_PWM_MIN = 1000
RC_PWM_CENTER = 1500
RC_PWM_MAX = 2000
RC_PWM_DISPLAY_MIN = 800
RC_PWM_DISPLAY_MAX = 2200

# Rangefinder signal quality, percent.
RANGEFINDER_SIGNAL_GOOD = 70
RANGEFINDER_SIGNAL_OK = 30

# Wi-Fi signal strength, percent (NetworkManager's 0-100 scale).
WIFI_SIGNAL_GOOD = 60
WIFI_SIGNAL_OK = 30

# MAVLink receive rate, msg/s. A healthy PX4 link normally streams tens of
# messages/second; below this it's either a thin link or streams haven't
# been (re-)requested yet.
RX_RATE_GOOD = 20.0
RX_RATE_OK = 5.0

# VIBRATION.vibration_{x,y,z} magnitude, m/s^2 rms - PX4/QGroundControl's
# rule-of-thumb bands for "fine" vs "marginal, check prop balance / mounting"
# vs "bad". Any nonzero clipping count is treated as bad regardless of this.
VIBRATION_GOOD = 15.0
VIBRATION_OK = 30.0

# DISTANCE_SENSOR orientation code for a straight-down-facing sensor
# (MAV_SENSOR_ROTATION_PITCH_270), i.e. the one PX4 fuses for HAGL/terrain.
DISTANCE_SENSOR_ORIENTATION_DOWN = 25


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

# Number of parameters visible on one terminal page.
PARAM_PAGE_SIZE = 18

# Time allowed for a complete PARAM_REQUEST_LIST response.
PARAM_REQUEST_TIMEOUT = 15.0

# Time allowed for PX4 to echo a PARAM_SET.
PARAM_SET_TIMEOUT = 3.0

# Numeric comparison tolerance.
PARAM_VALUE_EPSILON = 1e-6

PARAM_TYPE_NAMES = {
    1: "UINT8",
    2: "INT8",
    3: "UINT16",
    4: "INT16",
    5: "UINT32",
    6: "INT32",
    7: "UINT64",
    8: "INT64",
    9: "REAL32",
    10: "REAL64",
}


# ---------------------------------------------------------------------------
# Flight modes (Standard Modes Protocol)
# ---------------------------------------------------------------------------

# MAVLink "Standard Modes Protocol" message IDs.
#
# PX4 v1.15+ can enumerate ALL of its modes (including custom /
# PX4-ROS2-external modes that are NOT in pymavlink's static
# mode_mapping_px4 table) via these messages. Older PX4 simply
# never answers the request, and we fall back to the legacy
# hardcoded mode_mapping() list.
MAVLINK_MSG_ID_AVAILABLE_MODES = 435
MAVLINK_MSG_ID_CURRENT_MODE = 436
MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR = 437

AVAILABLE_MODES_TIMEOUT = 5.0

# Rows shown on one page of the mode-select list.
MODE_PAGE_SIZE = 18


# ---------------------------------------------------------------------------
# Keyboard jog / nudge control ([w] on the dashboard)
# ---------------------------------------------------------------------------

# Each key press nudges the vehicle by one step via MAV_CMD_DO_REPOSITION.
JOG_STEP_M = 1.0
JOG_STEP_MIN_M = 0.25
JOG_STEP_MAX_M = 10.0
JOG_YAW_STEP_DEG = 15.0

# Minimum spacing between nudges, so holding a key does not flood the link.
JOG_MIN_INTERVAL = 0.12


# ---------------------------------------------------------------------------
# MAVLink shell (NSH console)
# ---------------------------------------------------------------------------
#
# PX4 exposes its NuttShell over MAVLink using SERIAL_CONTROL packets
# targeted at the "shell" virtual device - this is exactly the mechanism
# behind QGroundControl's "MAVLink Console" widget. Only one client can hold
# it exclusively at a time.

SHELL_DEVICE = getattr(mavutil.mavlink, "SERIAL_CONTROL_DEV_SHELL", 10)
SHELL_FLAG_RESPOND = getattr(mavutil.mavlink, "SERIAL_CONTROL_FLAG_RESPOND", 1)
SHELL_FLAG_EXCLUSIVE = getattr(mavutil.mavlink, "SERIAL_CONTROL_FLAG_EXCLUSIVE", 2)

SHELL_MAX_LINES = 1000


# ---------------------------------------------------------------------------
# Flight-log transfer (MAVLink LOG_* messages)
# ---------------------------------------------------------------------------

FLIGHT_LOG_PAGE_SIZE = 18

LOG_LIST_TIMEOUT = 5.0
LOG_LIST_QUIET_PERIOD = 0.60

# LOG_DATA carries at most 90 bytes, but PX4 can satisfy one LOG_REQUEST_DATA
# with a much larger range and stream many LOG_DATA packets back-to-back.
# Requesting a window amortizes the request/response overhead and is
# dramatically faster than asking for 90 bytes at a time.
LOG_CHUNK_SIZE = 90
LOG_REQUEST_SIZE = 5000
LOG_CHUNK_TIMEOUT = 5.0
LOG_CHUNK_RETRIES = 5


# ---------------------------------------------------------------------------
# Position map trail (position map -> [t])
# ---------------------------------------------------------------------------
#
# The map screen packs the trail into Braille characters (8 sub-cell dots
# each - see lazypx4.render.chrome.braille_glyph), so it can usefully show
# a much finer trail than one "." per character cell could - which is why
# this samples much closer together than the on-screen grid's own cell size
# (typically ~0.5-2 m at a normal zoom) would otherwise justify.
POSITION_TRAIL_MIN_SPACING_M = 0.2
POSITION_TRAIL_MAXLEN = 1000


# ---------------------------------------------------------------------------
# Local camera preview ([w] screen - see lazypx4.camera)
# ---------------------------------------------------------------------------
#
# Never started automatically (opening a V4L2 device and running ffmpeg is
# real CPU/USB cost) - the [w] screen asks for confirmation before capture
# starts. Once on, a single long-lived ffmpeg process streams raw frames
# continuously (rather than being re-spawned per frame, which would reopen
# the device - and pay its negotiation/settling time - every single frame
# and cap effective fps far below what the device can actually do). Two
# presets trade detail for less data to move and decode each frame: "full"
# is plausible over a local/SSH-on-LAN session, "low bandwidth" is meant for
# a slow link (a telemetry radio or a thin cellular tether) and also
# switches the renderer from ANSI truecolor half-blocks to a colourless
# ASCII ramp, which is what actually cuts the bytes written to the terminal.
CAMERA_START_TIMEOUT = 5.0
CAMERA_FRAME_TIMEOUT = 3.0
CAMERA_RETRY_INTERVAL = 1.0

# ffmpeg -input_format candidates to try, in order, until one actually
# produces frames. Without pinning this, ffmpeg/V4L2 may negotiate a raw
# (uncompressed) capture mode that many USB webcams only support at a
# handful of fps due to USB bandwidth, even though the same device can
# stream full-rate in a compressed format - MJPEG is the near-universal one,
# so it's tried first. `None` means "let ffmpeg auto-select" and is the
# last-resort fallback for devices MJPEG doesn't work on.
CAMERA_INPUT_FORMATS = ("mjpeg", None)

CAMERA_FULL_WIDTH = 256
CAMERA_FULL_HEIGHT = 144
CAMERA_FULL_FPS = 30

# [j]/[k] on the camera screen scale the "full" preset's resolution up/down
# at runtime (aspect ratio held fixed at CAMERA_FULL_HEIGHT/CAMERA_FULL_WIDTH),
# clamped to this range. The upper end covers a typical webcam's max (most
# top out around 1080p) - requesting more than a given device supports just
# makes the V4L2 driver clamp to its own max, same as any other size it
# doesn't support exactly (see the "driver changed the video from X to Y"
# case ffmpeg logs). Low-bandwidth resolution is intentionally not
# adjustable - its whole point is a fixed, small, predictable frame size for
# a slow link.
CAMERA_FULL_WIDTH_MIN = 64
CAMERA_FULL_WIDTH_MAX = 1920
CAMERA_FULL_RES_STEP = 1.25

CAMERA_LOW_BW_WIDTH = 64
CAMERA_LOW_BW_HEIGHT = 36
CAMERA_LOW_BW_FPS = 5


# ---------------------------------------------------------------------------
# Satellite-image snapshots (position map -> [i])
# ---------------------------------------------------------------------------
#
# Pulled from Esri "World Imagery" (no API key). The companion needs internet
# access; the image and a JSON sidecar are written to ``settings.map_dir``.

MAP_IMAGE_SIZE = 1024
MAP_IMAGE_MIN_SPAN_M = 120.0
MAP_IMAGE_PAD = 1.8
MAP_DOWNLOAD_TIMEOUT = 25.0


# ---------------------------------------------------------------------------
# Estimation / sensor stream setup
# ---------------------------------------------------------------------------
#
# PX4 streams most of what the estimation view needs on the default MAVLink
# stream, but at low rates. Ask for a handful of them a bit faster, and read
# the EKF2_* aiding parameters that describe which sensors the estimator is
# actually fusing and which height source it treats as primary.

STREAM_MESSAGE_INTERVALS = {
    2: 1000000,     # SYSTEM_TIME                1 Hz
    30: 100000,     # ATTITUDE                  10 Hz
    32: 100000,     # LOCAL_POSITION_NED        10 Hz
    33: 200000,     # GLOBAL_POSITION_INT        5 Hz
    24: 200000,     # GPS_RAW_INT                5 Hz
    124: 500000,    # GPS2_RAW                   2 Hz
    127: 500000,    # GPS_RTK                    2 Hz
    128: 1000000,   # GPS2_RTK                   1 Hz
    29: 500000,     # SCALED_PRESSURE            2 Hz
    74: 200000,     # VFR_HUD                    5 Hz
    132: 200000,    # DISTANCE_SENSOR            5 Hz
    141: 200000,    # ALTITUDE                   5 Hz
    230: 400000,    # ESTIMATOR_STATUS         2.5 Hz
    62: 200000,     # NAV_CONTROLLER_OUTPUT      5 Hz
    83: 200000,     # ATTITUDE_TARGET            5 Hz
    85: 200000,     # POSITION_TARGET_LOCAL_NED  5 Hz
    241: 500000,    # VIBRATION                  2 Hz
}

ESTIMATOR_PARAM_NAMES = (
    "EKF2_HGT_REF",
    "EKF2_GPS_CTRL",
    "EKF2_BARO_CTRL",
    "EKF2_RNG_CTRL",
    "EKF2_EV_CTRL",
    "EKF2_OF_CTRL",
    "EKF2_AGP_CTRL",
    "EKF2_MAG_TYPE",
    "EKF2_MIN_RNG",
    "EKF2_RNG_A_HMAX",
)

EKF2_HGT_REF_NAMES = {0: "BARO", 1: "GPS", 2: "RANGE", 3: "EXT VISION"}
EKF2_RNG_CTRL_NAMES = {0: "DISABLED", 1: "CONDITIONAL", 2: "ALWAYS"}


# ---------------------------------------------------------------------------
# Sensor calibration (MAV_CMD_PREFLIGHT_CALIBRATION)
# ---------------------------------------------------------------------------

MAV_CMD_PREFLIGHT_CALIBRATION = getattr(
    mavutil.mavlink, "MAV_CMD_PREFLIGHT_CALIBRATION", 241
)

# param1..param7 of MAV_CMD_PREFLIGHT_CALIBRATION.
CALIBRATION_PARAMS = {
    "gyro":  (1, 0, 0, 0, 0, 0, 0),
    "mag":   (0, 1, 0, 0, 0, 0, 0),
    "baro":  (0, 0, 1, 0, 0, 0, 0),
    "accel": (0, 0, 0, 0, 1, 0, 0),
    "level": (0, 0, 0, 0, 2, 0, 0),
}

CALIBRATION_LABELS = {
    "gyro": "gyroscope",
    "mag": "magnetometer / compass",
    "baro": "barometer",
    "accel": "accelerometer",
    "level": "level horizon",
}


# ---------------------------------------------------------------------------
# COMMAND_ACK / GPS / EKF display tables
# ---------------------------------------------------------------------------

COMMAND_NAMES = {
    11: "SET_MODE",
    20: "NAV_RETURN_TO_LAUNCH",
    21: "NAV_LAND",
    22: "NAV_TAKEOFF",
    176: "DO_SET_MODE",
    192: "DO_REPOSITION",
    193: "DO_PAUSE_CONTINUE",
    241: "PREFLIGHT_CALIBRATION",
    246: "PREFLIGHT_REBOOT_SHUTDOWN",
    262: "DO_SET_STANDARD_MODE",
    400: "COMPONENT_ARM_DISARM",
    511: "SET_MESSAGE_INTERVAL",
    512: "REQUEST_MESSAGE",
}

ACK_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}

GPS_FIX_NAMES = {
    0: "NO GPS",
    1: "NO FIX",
    2: "2D FIX",
    3: "3D FIX",
    4: "DGPS",
    5: "RTK FLOAT",
    6: "RTK FIXED",
    7: "STATIC",
    8: "PPP",
}

# ESTIMATOR_STATUS_FLAGS (PX4).
ESTIMATOR_STATUS_FLAG_LABELS = (
    (1, "ATT"),
    (2, "VEL_H"),
    (4, "VEL_V"),
    (8, "POS_H_REL"),
    (16, "POS_H_ABS"),
    (32, "POS_V_ABS"),
    (64, "POS_V_AGL"),
    (128, "CONST_POS"),
    (256, "PRED_POS_H_REL"),
    (512, "PRED_POS_H_ABS"),
    (1024, "GPS_GLITCH"),
    (2048, "ACCEL_ERROR"),
)

# EKF_STATUS_FLAGS (EKF_STATUS_REPORT).
EKF_STATUS_REPORT_FLAG_LABELS = (
    (1, "ATT"),
    (2, "VEL_H"),
    (4, "VEL_V"),
    (8, "POS_H_REL"),
    (16, "POS_H_ABS"),
    (32, "POS_V_ABS"),
    (64, "POS_V_AGL"),
    (128, "CONST_POS"),
    (256, "PRED_POS_H_REL"),
    (512, "PRED_POS_H_ABS"),
    (1024, "UNINITIALIZED"),
)

DISTANCE_SENSOR_ORIENTATION_NAMES = {
    0: "forward",
    24: "up",
    25: "down",
}

# SYS_STATUS onboard_control_sensors_{present,enabled,health} bits, restricted
# to the ones relevant to a pre-flight arming check on a typical multirotor
# (the full MAV_SYS_STATUS_SENSOR enum has ~30 bits, most airframe-specific).
SYS_STATUS_SENSOR_LABELS = (
    (1, "GYRO"),
    (2, "ACCEL"),
    (4, "MAG"),
    (8, "BARO"),
    (32, "GPS"),
    (64, "OPT_FLOW"),
    (1024, "RATE_CTRL"),
    (2048, "ATT_CTRL"),
    (4096, "YAW_POS"),
    (8192, "ALT_CTRL"),
    (16384, "POS_CTRL"),
    (32768, "MOTORS"),
    (65536, "RC"),
    (33554432, "BATTERY"),
)

# AUTOPILOT_VERSION.flight_sw_version's low byte (FIRMWARE_VERSION_TYPE).
FIRMWARE_VERSION_TYPE_NAMES = {
    0: "dev",
    64: "alpha",
    128: "beta",
    192: "rc",
    255: "release",
}

POS_TARGET_FRAME_NAMES = {
    1: "LOCAL_NED",
    7: "LOCAL_OFFSET_NED",
    8: "BODY_NED",
    9: "BODY_OFFSET_NED",
    20: "LOCAL_FRD",
    21: "LOCAL_FLU",
}
