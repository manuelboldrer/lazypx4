"""Command-line entry point: parse args, apply settings, run the app."""

from __future__ import annotations

import argparse
import sys

from . import __version__
from .config import settings


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="lazypx4",
        description="A lazygit-style terminal UI for PX4 over MAVLink.",
    )
    parser.add_argument(
        "--version", action="version", version=f"%(prog)s {__version__}",
    )
    parser.add_argument(
        "-p", "--port", type=int, default=settings.port, metavar="N",
        help=f"UDP port to listen for the PX4 MAVLink connection on (default: {settings.port})",
    )
    parser.add_argument(
        "--log-dir", default=settings.log_dir, metavar="DIR",
        help=f"directory for downloaded .ulg flight logs (default: {settings.log_dir})",
    )
    parser.add_argument(
        "--map-dir", default=settings.map_dir, metavar="DIR",
        help=f"directory for satellite-image snapshots (default: {settings.map_dir})",
    )
    parser.add_argument(
        "--disk-path", default=settings.disk_path, metavar="PATH",
        help=f"filesystem whose usage the dashboard HOST block shows "
             f"(default: {settings.disk_path})",
    )
    parser.add_argument(
        "--param-defaults", default=None, metavar="FILE",
        help="JSON file of PX4 parameter defaults, enabling the "
             "'changed from default' parameter view",
    )
    parser.add_argument(
        "--allow-log-download-while-armed", action="store_true",
        help="permit starting a flight-log download while the vehicle is armed "
             "(off by default)",
    )
    parser.add_argument(
        "--firmware-dir", default=settings.firmware_dir, metavar="DIR",
        help="directory scanned for .px4 firmware files on the "
             f"flash-firmware screen (default: {settings.firmware_dir})",
    )
    parser.add_argument(
        "--tools-dir", default=settings.tools_dir, metavar="DIR",
        help="directory containing px_uploader.py / upload_log.py / ecl_ekf "
             f"(default: {settings.tools_dir})",
    )
    parser.add_argument(
        "--lidar-topic", default=settings.lidar_topic, metavar="TOPIC",
        help="ROS 2 sensor_msgs/PointCloud2 topic for the [v] point-cloud "
             f"screen (default: {settings.lidar_topic})",
    )
    parser.add_argument(
        "--camera-topic", default=settings.camera_topic_1, metavar="TOPIC",
        help="ROS 2 sensor_msgs/Image or CompressedImage topic for the "
             f"[w] camera screen's first slot (default: {settings.camera_topic_1})",
    )
    parser.add_argument(
        "--camera-topic-2", default=settings.camera_topic_2, metavar="TOPIC",
        help="a second ROS 2 image topic for the [w] camera screen, shown "
             "alongside the first (default: unset)",
    )
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)

    settings.port = args.port
    settings.log_dir = args.log_dir
    settings.map_dir = args.map_dir
    settings.disk_path = args.disk_path
    settings.param_defaults_file = args.param_defaults
    settings.allow_log_download_while_armed = args.allow_log_download_while_armed
    settings.firmware_dir = args.firmware_dir
    settings.tools_dir = args.tools_dir
    settings.lidar_topic = args.lidar_topic
    settings.camera_topic_1 = args.camera_topic
    settings.camera_topic_2 = args.camera_topic_2

    if not sys.stdin.isatty():
        print("lazypx4 needs an interactive terminal to run.", file=sys.stderr)
        return 1

    # Imported here so that --help / --version work without a MAVLink stack.
    from .app import run

    run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
