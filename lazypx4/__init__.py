"""lazypx4 - a lazygit-style terminal UI for PX4 over MAVLink."""

import os
import sys

# A normal (non-frozen) `python3` run folds the PYTHONPATH environment variable into sys.path
# itself, which is how lazypx4.rosclock/lazypx4.lidar find rclpy from a sourced ROS 2 install
# without it ever being a pip dependency. PyInstaller's frozen bootloader does NOT do this -- its
# sys.path is limited to its own bundle -- so without this, `import rclpy` in the frozen binary
# fails even in a shell where ROS 2 has been sourced and $PYTHONPATH correctly includes it. This
# must run before any lazypx4 submodule's own `try: import rclpy` (hence here, in the package's
# own __init__, which always runs first), and only when frozen: a non-frozen run already has these
# via the interpreter's normal startup, and re-adding them would be harmless but pointless.
if getattr(sys, "frozen", False):
    for _path in os.environ.get("PYTHONPATH", "").split(os.pathsep):
        if _path and _path not in sys.path:
            sys.path.append(_path)

__version__ = "0.1.0"
