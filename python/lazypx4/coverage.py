"""Area coverage ("lawnmower") path generation for a KML fence polygon.

:func:`coverage_path` turns a polygon ring into an ordered list of
``(lat, lon)`` waypoints that sweeps parallel, evenly spaced lines across it,
alternating direction on each pass (boustrophedon), so a vehicle flying them
in order covers the whole area with the fewest turns. Pure geometry - the
waypoint queue ([C] on the map screen, see :mod:`lazypx4.mavlink.wp_queue`) is what
actually flies the result.

Works in a flat local metre frame around the polygon's centroid, which is
accurate to well under a centimetre per kilometre for field-sized areas. Concave
polygons are handled (a scan line crossing the polygon several times yields one
segment per crossing) but the segments are simply visited in scan order, so the
vehicle may cut across the outside of a concave notch between them.
"""

from __future__ import annotations

import math

#: WGS-84 equatorial radius, metres.
_EARTH_RADIUS_M = 6378137.0

#: Refuse to generate more waypoints than this - almost certainly a spacing typo.
MAX_COVERAGE_WAYPOINTS = 500


def _to_local(ring, lat0, lon0):
    """(lat, lon) ring -> [(east, north)] metres around (lat0, lon0)."""
    k_lon = math.cos(math.radians(lat0))
    return [
        (
            math.radians(lon - lon0) * _EARTH_RADIUS_M * k_lon,
            math.radians(lat - lat0) * _EARTH_RADIUS_M,
        )
        for lat, lon in ring
    ]


def _to_global(east, north, lat0, lon0):
    k_lon = math.cos(math.radians(lat0))
    return (
        lat0 + math.degrees(north / _EARTH_RADIUS_M),
        lon0 + math.degrees(east / (_EARTH_RADIUS_M * k_lon)),
    )


def _longest_edge_heading(points):
    """Compass heading (0-180, 0 = north) of the polygon's longest edge."""
    best_len = -1.0
    best_heading = 0.0
    for (e1, n1), (e2, n2) in zip(points, points[1:] + points[:1]):
        length = math.hypot(e2 - e1, n2 - n1)
        if length > best_len:
            best_len = length
            best_heading = math.degrees(math.atan2(e2 - e1, n2 - n1)) % 180.0
    return best_heading


def coverage_path(ring, spacing_m, angle_deg=None, end_inset_m=1.0):
    """Lawnmower waypoints covering ``ring`` (``[(lat, lon), ...]``).

    ``spacing_m``   distance between adjacent sweep lines, metres (> 0).
    ``angle_deg``   compass heading of the sweep lines (0 = north-south lines,
                    90 = east-west). ``None`` aligns them with the polygon's
                    longest edge, which minimises the number of turns.
    ``end_inset_m`` each line is shortened by this much at both ends so the
                    vehicle doesn't fly exactly along the boundary/fence.

    The first line sits half a spacing inside the boundary. Returns
    ``[(lat, lon), ...]`` (two per sweep segment). Raises :class:`ValueError`
    on a bad spacing, a degenerate polygon, no line fitting inside it, or more
    than :data:`MAX_COVERAGE_WAYPOINTS` waypoints.
    """
    if not spacing_m or spacing_m <= 0:
        raise ValueError("spacing must be greater than 0 m")

    ring = list(ring)
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring = ring[:-1]  # KML rings repeat the first vertex to close
    if len(ring) < 3:
        raise ValueError("polygon needs at least 3 vertices")

    lat0 = sum(lat for lat, _lon in ring) / len(ring)
    lon0 = sum(lon for _lat, lon in ring) / len(ring)
    points = _to_local(ring, lat0, lon0)

    if angle_deg is None:
        angle_deg = _longest_edge_heading(points)

    # Rotate so the sweep lines run along +u; lines are stacked along v.
    # Heading h -> unit vector (east, north) = (sin h, cos h) is the u axis.
    a = math.radians(angle_deg)
    ue, un = math.sin(a), math.cos(a)
    ve, vn = un, -ue  # 90 degrees clockwise of u
    rot = [(e * ue + n * un, e * ve + n * vn) for e, n in points]

    v_min = min(v for _u, v in rot)
    v_max = max(v for _u, v in rot)
    if v_max - v_min <= 0:
        raise ValueError("polygon has no area")

    segments = []  # one (v, [(u_start, u_end), ...]) per sweep line
    v = v_min + spacing_m / 2.0
    while v < v_max:
        crossings = []
        for (u1, v1), (u2, v2) in zip(rot, rot[1:] + rot[:1]):
            if (v1 <= v < v2) or (v2 <= v < v1):
                crossings.append(u1 + (v - v1) / (v2 - v1) * (u2 - u1))
        crossings.sort()

        line = []
        for u_start, u_end in zip(crossings[0::2], crossings[1::2]):
            u_start += end_inset_m
            u_end -= end_inset_m
            if u_end > u_start:
                line.append((u_start, u_end))
        if line:
            segments.append((v, line))
        v += spacing_m

    if not segments:
        raise ValueError("no sweep line fits inside the polygon - try a smaller spacing")

    path = []
    for i, (v, line) in enumerate(segments):
        if i % 2:  # every other pass runs back the way it came
            line = [(u_end, u_start) for u_start, u_end in reversed(line)]
        for u_from, u_to in line:
            for u in (u_from, u_to):
                path.append(_to_global(u * ue + v * ve, u * un + v * vn, lat0, lon0))

    if len(path) > MAX_COVERAGE_WAYPOINTS:
        raise ValueError(
            f"{len(path)} waypoints (max {MAX_COVERAGE_WAYPOINTS}) - use a larger spacing"
        )

    return path
