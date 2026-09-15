"""The [?] screen: project logo, author/contact, sponsor link, version.

Static - no vehicle or host state to read, unlike every other screen in
:mod:`lazypx4.render`.
"""

from __future__ import annotations

from .. import __version__
from ..ansi import BOLD, CYAN, DIM, MAGENTA, RESET, ui_section

_LOGO = (
    "╻  ┏━┓╺━┓╻ ╻┏━┓╻ ╻╻ ╻",
    "┃  ┣━┫┏━┛┗┳┛┣━┛┏╋┛┗━┫",
    "┗━╸╹ ╹┗━╸ ╹ ╹  ╹ ╹  ╹",
)

_SPONSOR_URL = "https://github.com/sponsors/manuelboldrer"


def draw_about_screen():
    lines = []

    lines.append("")
    for row in _LOGO:
        lines.append("  " + BOLD + CYAN + row + RESET)
    lines.append("")
    lines.append("  " + DIM + "a lazygit-style terminal UI for PX4 over MAVLink" + RESET)
    lines.append("")

    lines.append(ui_section("AUTHOR"))
    lines.append("   Manuel Boldrer")
    lines.append("   manuel.boldrer@gmail.com")
    lines.append("")

    lines.append(ui_section("SPONSOR"))
    lines.append("   " + MAGENTA + _SPONSOR_URL + RESET)
    lines.append("")

    lines.append(ui_section("ACKNOWLEDGMENTS"))
    lines.append("   Saxion University of Applied Sciences")
    lines.append("   Smart Mechatronics and Robotics Group")
    lines.append("")

    lines.append(ui_section("VERSION"))
    lines.append(f"   lazypx4 {__version__}")
    lines.append("")

    lines.append("[?] back    [ESC] panels")

    return lines
