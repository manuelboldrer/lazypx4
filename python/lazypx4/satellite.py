"""Satellite-image snapshot for the position map ([n] -> [i]).

Downloads a Web-Mercator tile from Esri "World Imagery" (no API key) covering
the vehicle and its home point (and, if one is loaded, the [o] KML overlay's
fence/waypoints too), annotates it with pins, the fence boundary, waypoint
markers and a scale bar if Pillow is installed, and writes the image plus a
JSON sidecar to ``settings.map_dir``. Runs on a background thread; needs
internet access.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime, timezone

from .config import (
    MAP_DOWNLOAD_TIMEOUT,
    MAP_IMAGE_MIN_SPAN_M,
    MAP_IMAGE_PAD,
    MAP_IMAGE_SIZE,
    MAP_RANGE_MAX_M,
    settings,
)
from .eventlog import log_command, log_error, log_info, log_warn
from .state import state
from .util import clamp


def _web_mercator(lat, lon):
    radius = 6378137.0
    x = radius * math.radians(lon)
    y = radius * math.log(
        math.tan(math.pi / 4.0 + math.radians(clamp(lat, -85.0, 85.0)) / 2.0)
    )
    return x, y


def start_satellite_download():
    with state.lock:
        if state.map_download_active:
            log_warn("A satellite-image download is already running")
            return False

        if not state.global_pos_valid:
            state.map_download_status = "ERROR"
            state.map_download_error = (
                "no GPS position (need a GPS fix / GLOBAL_POSITION_INT)"
            )
            state.map_download_path = ""
            log_error("Satellite image: no GPS position available")
            return False

        robot_lat = state.global_lat
        robot_lon = state.global_lon

        if state.local_origin_set:
            home_lat = state.local_origin_lat
            home_lon = state.local_origin_lon
        else:
            home_lat, home_lon = robot_lat, robot_lon

        kml_waypoints = list(state.kml_waypoints) if state.kml_loaded else []
        kml_fence_rings = [list(ring) for ring in state.kml_fence_rings] if state.kml_loaded else []

        state.map_download_active = True
        state.map_download_status = "STARTING"
        state.map_download_error = ""
        state.map_download_path = ""
        state.map_download_started_at = time.monotonic()

    thread = threading.Thread(
        target=_satellite_download_worker,
        args=(robot_lat, robot_lon, home_lat, home_lon, kml_waypoints, kml_fence_rings),
        daemon=True,
        name="SatelliteMap",
    )
    thread.start()

    kml_note = ""
    if kml_waypoints or kml_fence_rings:
        kml_note = (
            f" + KML overlay ({len(kml_waypoints)} waypoint(s), "
            f"{len(kml_fence_rings)} fence ring(s))"
        )

    log_command(
        f"Satellite image requested: robot {robot_lat:.6f},{robot_lon:.6f} "
        f"home {home_lat:.6f},{home_lon:.6f}" + kml_note
    )
    return True


def _satellite_download_worker(robot_lat, robot_lon, home_lat, home_lon, kml_waypoints, kml_fence_rings):
    import urllib.error
    import urllib.request

    status = "ERROR"
    error_text = ""
    final_path = ""

    try:
        os.makedirs(settings.map_dir, exist_ok=True)

        rx, ry = _web_mercator(robot_lat, robot_lon)
        hx, hy = _web_mercator(home_lat, home_lon)

        # The frame must fit the KML overlay too, not just home/robot - same
        # centre+span approach, generalized from two points to however many
        # there are (with just home/robot, this reduces to exactly the old
        # calculation). A KML file can carry placemarks from more than one
        # site (e.g. a shared Google Earth project); a waypoint or fence
        # vertex tens or hundreds of km from home/robot would otherwise blow
        # the span out so far the imagery zooms out past the point of being
        # useful, so only fold in KML points within MAP_RANGE_MAX_M of the
        # home/robot area.
        ref_cx, ref_cy = (rx + hx) / 2.0, (ry + hy) / 2.0

        points_xy = [(rx, ry), (hx, hy)]
        for lat, lon, _alt, _name in kml_waypoints:
            px, py = _web_mercator(lat, lon)
            if math.hypot(px - ref_cx, py - ref_cy) <= MAP_RANGE_MAX_M:
                points_xy.append((px, py))
        for ring in kml_fence_rings:
            for lat, lon in ring:
                px, py = _web_mercator(lat, lon)
                if math.hypot(px - ref_cx, py - ref_cy) <= MAP_RANGE_MAX_M:
                    points_xy.append((px, py))

        xs = [p[0] for p in points_xy]
        ys = [p[1] for p in points_xy]

        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0

        span = max(max(xs) - min(xs), max(ys) - min(ys)) * MAP_IMAGE_PAD
        span = max(span, MAP_IMAGE_MIN_SPAN_M)
        half = span / 2.0

        minx, maxx = cx - half, cx + half
        miny, maxy = cy - half, cy + half

        url = (
            "https://server.arcgisonline.com/arcgis/rest/services/"
            "World_Imagery/MapServer/export?"
            f"bbox={minx:.2f},{miny:.2f},{maxx:.2f},{maxy:.2f}"
            "&bboxSR=3857&imageSR=3857"
            f"&size={MAP_IMAGE_SIZE},{MAP_IMAGE_SIZE}"
            "&format=png&transparent=false&f=image"
        )

        with state.lock:
            state.map_download_status = "DOWNLOADING"

        request = urllib.request.Request(url, headers={"User-Agent": "lazypx4"})
        with urllib.request.urlopen(request, timeout=MAP_DOWNLOAD_TIMEOUT) as response:
            data = response.read()

        if not data or len(data) < 512:
            raise RuntimeError("imagery server returned no image")

        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(settings.map_dir, f"px4_map_{stamp}")
        final_path = base + ".png"

        def to_pixel(x, y):
            px = (x - minx) / (maxx - minx) * MAP_IMAGE_SIZE
            py = (maxy - y) / (maxy - miny) * MAP_IMAGE_SIZE
            return px, py

        home_px = to_pixel(hx, hy)
        robot_px = to_pixel(rx, ry)

        kml_waypoints_px = [
            (to_pixel(*_web_mercator(lat, lon)), lat, lon, alt, name)
            for lat, lon, alt, name in kml_waypoints
        ]
        kml_fence_rings_px = [
            [to_pixel(*_web_mercator(lat, lon)) for lat, lon in ring]
            for ring in kml_fence_rings
        ]

        # Web-Mercator distances are inflated by 1/cos(latitude); undo that
        # for real-world ground distance and scale.
        mercator_scale = max(0.05, math.cos(math.radians((robot_lat + home_lat) / 2.0)))
        distance_m = math.hypot(rx - hx, ry - hy) * mercator_scale
        ground_res = (span / MAP_IMAGE_SIZE) * mercator_scale

        annotated = False
        try:
            import io as _io

            from PIL import Image, ImageDraw

            image = Image.open(_io.BytesIO(data)).convert("RGB")
            draw = ImageDraw.Draw(image)

            def marker(point, color, label):
                px, py = point
                radius = 11
                draw.line([px - radius - 5, py, px + radius + 5, py], fill=color, width=2)
                draw.line([px, py - radius - 5, px, py + radius + 5], fill=color, width=2)
                draw.ellipse(
                    [px - radius, py - radius, px + radius, py + radius],
                    outline=color, width=3,
                )
                draw.text((px + radius + 4, py - radius - 6), label, fill=color)

            metres_per_px = ground_res
            bar_m = 25.0
            while bar_m / metres_per_px < 90.0 and bar_m < span * mercator_scale:
                bar_m *= 2.0
            bar_px = bar_m / metres_per_px
            y_bar = MAP_IMAGE_SIZE - 28
            draw.line([20, y_bar, 20 + bar_px, y_bar], fill=(255, 235, 0), width=4)
            draw.text((20, y_bar - 16), f"{bar_m:.0f} m", fill=(255, 235, 0))

            fence_color = (255, 220, 0)
            for ring_px in kml_fence_rings_px:
                if len(ring_px) >= 2:
                    draw.line(ring_px + [ring_px[0]], fill=fence_color, width=3)

            waypoint_color = (230, 60, 220)
            for i, (point, _lat, _lon, _alt, name) in enumerate(kml_waypoints_px, start=1):
                wx, wy = point
                wp_radius = 7
                label = name or f"WP{i}"
                draw.ellipse(
                    [wx - wp_radius, wy - wp_radius, wx + wp_radius, wy + wp_radius],
                    outline=waypoint_color, width=2,
                )
                draw.text((wx + wp_radius + 3, wy - wp_radius - 4), f"{i}:{label}", fill=waypoint_color)

            marker(home_px, (255, 70, 70), "HOME")
            marker(robot_px, (80, 170, 255), f"ROBOT  {distance_m:.0f} m")

            image.save(final_path)
            annotated = True
        except Exception:
            # Pillow not installed (or failed): keep the raw imagery, the
            # JSON sidecar still records where the pins belong.
            with open(final_path, "wb") as handle:
                handle.write(data)

        sidecar = {
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "source": "Esri World Imagery (server.arcgisonline.com)",
            "annotated": annotated,
            "image": os.path.basename(final_path),
            "image_size_px": [MAP_IMAGE_SIZE, MAP_IMAGE_SIZE],
            "bbox_epsg3857": [minx, miny, maxx, maxy],
            "ground_resolution_m_per_px": round(ground_res, 3),
            "home": {
                "lat": home_lat,
                "lon": home_lon,
                "pixel": [round(home_px[0], 1), round(home_px[1], 1)],
            },
            "robot": {
                "lat": robot_lat,
                "lon": robot_lon,
                "pixel": [round(robot_px[0], 1), round(robot_px[1], 1)],
                "distance_from_home_m": round(distance_m, 1),
            },
        }

        if kml_waypoints_px or kml_fence_rings_px:
            sidecar["kml"] = {
                "waypoints": [
                    {
                        "name": name,
                        "lat": lat,
                        "lon": lon,
                        "alt": alt,
                        "pixel": [round(point[0], 1), round(point[1], 1)],
                    }
                    for point, lat, lon, alt, name in kml_waypoints_px
                ],
                "fence_rings": [
                    [[round(px, 1), round(py, 1)] for px, py in ring_px]
                    for ring_px in kml_fence_rings_px
                ],
            }

        with open(base + ".json", "w", encoding="utf-8") as handle:
            json.dump(sidecar, handle, indent=2)

        status = "COMPLETE"

    except urllib.error.URLError as exc:
        error_text = f"network error: {getattr(exc, 'reason', exc)}"
    except Exception as exc:
        error_text = str(exc)

    with state.lock:
        state.map_download_active = False
        state.map_download_status = status
        state.map_download_error = error_text
        if status == "COMPLETE":
            state.map_download_path = final_path

    if status == "COMPLETE":
        note = "annotated" if final_path.endswith(".png") else ""
        if kml_waypoints or kml_fence_rings:
            note = (note + " + kml overlay").strip()
        log_info(f"Satellite image saved: {final_path} {note}".rstrip())
    else:
        log_error(f"Satellite image download failed: {error_text}")
