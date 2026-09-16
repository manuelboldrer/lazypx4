"""The [?] screen: project logo, author/contact, version.

Static - no vehicle or host state to read, unlike every other screen in
:mod:`lazypx4.render`.
"""

from __future__ import annotations

from .. import __version__
from ..ansi import BOLD, CYAN, DIM, MAGENTA, RESET, ui_section

_LOGO = (
    "▌  ▞▀▖▀▀▌▌ ▌▛▀▖▌ ▌▌ ▌",
    "▌  ▙▄▌ ▞ ▝▞ ▙▄▘▝▞ ▚▄▌",
    "▌  ▌ ▌▞   ▌ ▌  ▞▝▖  ▌",
    "▀▀▘▘ ▘▀▀▘ ▘ ▘  ▘ ▘  ▘",
)

_AUTHOR_URL = "https://manuelboldrer.github.io/"
_GROUP_URL = "https://www.saxion.edu/research/research-groups/smart-mechatronics-and-robotics"


def draw_about_screen():
    lines = []

    lines.append("")
    for row in _LOGO:
        lines.append("  " + BOLD + CYAN + row + RESET)
    lines.append("")
    lines.append("  " + DIM + "a lazygit-style terminal UI for PX4 over MAVLink" + RESET)
    lines.append("")

    lines.append(ui_section("AUTHOR"))
    lines.append("   Manuel Boldrer  " + MAGENTA + _AUTHOR_URL + RESET)
    lines.append("   manuel.boldrer@gmail.com")
    lines.append("")

    lines.append(ui_section("ACKNOWLEDGMENTS"))
    lines.append("   Saxion University of Applied Sciences")
    lines.append("   Smart Mechatronics and Robotics Group")
    lines.append("   " + MAGENTA + _GROUP_URL + RESET)
    lines.append("")

    lines.append(ui_section("VERSION"))
    lines.append(f"   lazypx4 {__version__}")
    lines.append("")

    lines.append("[?] back    [ESC] panels")

    return lines
