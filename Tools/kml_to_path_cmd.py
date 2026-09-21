#!/usr/bin/env python3
"""Print a `ros2 topic pub` command that publishes a nav_msgs/Path on
/navsat_utm_path passing through KML waypoints, to test lazypx4's [N] overlay.

The path is in the local map frame (x = East, y = North, metres from the PX4
local origin), projected with the same equirectangular formula the lazypx4 map
uses, so the path lands exactly on the numbered KML markers. Pass the origin
the map shows on its "Origin H" line.

    Tools/kml_to_path_cmd.py ../kml/Dome.kml --origin 52.2186000 6.8869000
    Tools/kml_to_path_cmd.py ../kml/Dome.kml --origin 52.2186 6.8869 --waypoints 1 2 3 --z 10
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from lazypx4.kml import parse_kml  # noqa: E402

EARTH_RADIUS_M = 6378137.0  # keep in sync with lazypx4.render.chrome.global_to_local


def global_to_local(lat, lon, lat0, lon0):
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return north, east


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kml")
    ap.add_argument("--origin", nargs=2, type=float, required=True, metavar=("LAT", "LON"))
    ap.add_argument("--waypoints", nargs="+", type=int, help="1-based waypoint numbers, in order (default: all)")
    ap.add_argument("--z", type=float, default=10.0, help="pose height in metres (default 10)")
    ap.add_argument("--spacing", type=float, default=0.0, help="insert points every this many metres (default: off)")
    ap.add_argument("--topic", default="/navsat_utm_path")
    args = ap.parse_args()

    wps = parse_kml(args.kml)["waypoints"]
    picks = args.waypoints or list(range(1, len(wps) + 1))
    pts = []  # (east, north)
    for i in picks:
        n, e = global_to_local(wps[i - 1][0], wps[i - 1][1], *args.origin)
        pts.append((e, n))

    if args.spacing > 0:
        dense = [pts[0]]
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            steps = max(1, int(math.hypot(bx - ax, by - ay) / args.spacing))
            dense += [(ax + (bx - ax) * k / steps, ay + (by - ay) * k / steps) for k in range(1, steps + 1)]
        pts = dense

    poses = ",\n    ".join(
        f"{{pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {args.z}}}}}}}" for x, y in pts
    )
    print(f"""ros2 topic pub -r 1 {args.topic} nav_msgs/msg/Path "{{
  header: {{frame_id: map}},
  poses: [
    {poses}
  ]
}}\"""")


if __name__ == "__main__":
    main()
