"""Static data for the lazygit/lazydocker-style panel layout.

:data:`NAV_ITEMS` drives the sidebar panel list rendered by
:mod:`lazypx4.render.chrome` and the sidebar-focus key handling in
:mod:`lazypx4.navigation` - both import this instead of hard-coding the
screen order twice. :data:`SCREEN_TITLES` supplies the title painted into the
main panel's border, replacing the per-screen ``ui_header`` banner each
screen used to draw inline.

Each entry is ``(screen_key, label, hotkey)``; ``hotkey`` is the key already
bound in :data:`lazypx4.navigation._SCREEN_OPENERS` to open that screen, or
``None`` for the dashboard, which every screen returns to via ESC instead of
a dedicated open key.
"""

from __future__ import annotations

NAV_ITEMS = [
    ("dashboard", "DASHBOARD", None),
    ("map", "MISSION", "n"),
    ("mode_select", "MODE", "m"),
    ("parameters", "PARAMETERS", "p"),
    ("log", "EVENT LOG", "g"),
    ("flight_logs", "FLIGHT LOGS", "l"),
    ("shell", "NSH SHELL", "t"),
    ("pointcloud", "LIDAR POINTS", "v"),
    ("camera", "CAMERA", "w"),
    ("host", "USB / NETWORK", "u"),
    ("control", "CONTROL", "c"),
    ("calibration", "CALIBRATE", "s"),
    ("firmware", "FLASH FIRMWARE", "f"),
    ("about", "ABOUT", "?"),
]

SCREEN_TITLES = {
    "dashboard": "PX4 UAV · COMMAND / TELEMETRY",
    "mode_select": "PX4 FLIGHT MODE SELECT",
    "log": "PX4 EVENT · WARNING · FAILSAFE LOG",
    "flight_logs": "PX4 FLIGHT LOGS",
    "firmware": "PX4 FIRMWARE FLASH",
    "parameters": "PX4 PARAMETERS",
    "control": "PX4 CONTROL / SETPOINTS · RC",
    "camera": "ROS CAMERA PREVIEW",
    "map": "PX4 MISSION · plan view, up = North",
    "pointcloud": "LIDAR POINT CLOUD OVERVIEW",
    "calibration": "PX4 SENSOR CALIBRATION",
    "shell": "PX4 MAVLINK SHELL · NSH",
    "host": "HOST · USB / NETWORK",
    "about": "ABOUT · LAZYPX4",
}
