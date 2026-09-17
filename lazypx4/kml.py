"""Minimal, dependency-free .kml parsing for the map screen's KML overlay.

Only the handful of KML elements a geofence/waypoint export actually uses are
understood: ``Point`` (a waypoint), ``LineString`` (an ordered waypoint path)
and ``Polygon`` (a fence boundary, taken from its outer ring). Anything else
in the file - styles, folders, extended data - is ignored. This uses the
standard library's ``xml.etree.ElementTree`` rather than a real KML library,
since the point is to load a file instantly with nothing extra to install.

This is a *visualization* overlay only - nothing here is ever uploaded to the
vehicle. See :mod:`lazypx4.render.mapview`.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET


class KmlError(Exception):
    pass


def _local_tag(element):
    tag = element.tag
    return tag.rsplit("}", 1)[-1] if "}" in tag else tag


def _find_first(element, tag_name):
    for child in element.iter():
        if _local_tag(child) == tag_name:
            return child
    return None


def _parse_coordinates(text):
    """"lon,lat[,alt] lon,lat[,alt] ..." -> [(lat, lon, alt), ...]."""
    points = []

    for chunk in text.split():
        parts = chunk.split(",")
        if len(parts) < 2:
            continue
        try:
            lon = float(parts[0])
            lat = float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
        except ValueError:
            continue
        points.append((lat, lon, alt))

    return points


def parse_kml(path):
    """Parse a .kml file into ``{"waypoints": [...], "fence_rings": [...]}``.

    ``waypoints`` is ``[(lat, lon, alt, name), ...]`` in document order - one
    entry per ``Point`` placemark, plus every vertex of every ``LineString``
    in order. ``fence_rings`` is ``[[(lat, lon), ...], ...]``, one ring per
    ``Polygon``'s outer boundary - most files have exactly one, but every
    ring found is kept so multiple inclusion/exclusion zones still show up.

    Raises :class:`KmlError` on anything unreadable, unparseable, or with
    nothing usable in it.
    """
    try:
        tree = ET.parse(path)
    except (ET.ParseError, OSError) as exc:
        raise KmlError(f"could not read {path}: {exc}") from exc

    root = tree.getroot()
    waypoints = []
    fence_rings = []

    for placemark in root.iter():
        if _local_tag(placemark) != "Placemark":
            continue

        name_el = _find_first(placemark, "name")
        name = (name_el.text or "").strip() if name_el is not None else ""

        point_el = _find_first(placemark, "Point")
        if point_el is not None:
            coords_el = _find_first(point_el, "coordinates")
            if coords_el is not None and coords_el.text:
                points = _parse_coordinates(coords_el.text)
                if points:
                    lat, lon, alt = points[0]
                    waypoints.append((lat, lon, alt, name))

        line_el = _find_first(placemark, "LineString")
        if line_el is not None:
            coords_el = _find_first(line_el, "coordinates")
            if coords_el is not None and coords_el.text:
                for i, (lat, lon, alt) in enumerate(_parse_coordinates(coords_el.text), start=1):
                    waypoints.append((lat, lon, alt, f"{name} {i}".strip()))

        for polygon_el in placemark.iter():
            if _local_tag(polygon_el) != "Polygon":
                continue

            outer_el = _find_first(polygon_el, "outerBoundaryIs")
            if outer_el is None:
                continue

            ring_el = _find_first(outer_el, "LinearRing")
            if ring_el is None:
                continue

            coords_el = _find_first(ring_el, "coordinates")
            if coords_el is None or not coords_el.text:
                continue

            ring = [(lat, lon) for lat, lon, _alt in _parse_coordinates(coords_el.text)]
            if len(ring) >= 3:
                fence_rings.append(ring)

    if not waypoints and not fence_rings:
        raise KmlError("no Point/LineString waypoints or Polygon fence found in this file")

    return {"waypoints": waypoints, "fence_rings": fence_rings}
