//! Satellite-image snapshot for the mission map ([i]). Mirrors
//! `satellite.py`: one Web-Mercator export from Esri "World Imagery" (no API
//! key) covering the vehicle, home and any loaded KML overlay, annotated
//! with pins, the fence, waypoint markers and a scale bar, plus a JSON
//! sidecar recording where everything is. Background thread; needs
//! internet.

use std::f64::consts::PI;
use std::path::Path;

use image::{Rgb, RgbImage};

use crate::config::*;
use crate::geo;
use crate::state::{self, Shared, now};

fn web_mercator(lat: f64, lon: f64) -> (f64, f64) {
    let r = geo::EARTH_RADIUS_M;
    (r * lon.to_radians(), r * (PI / 4.0 + lat.clamp(-85.0, 85.0).to_radians() / 2.0).tan().ln())
}

pub fn start(shared: &Shared, map_dir: &str) -> bool {
    let job = {
        let mut st = state::lock(shared);
        if st.map_download_active {
            st.warn("A satellite-image download is already running");
            return false;
        }
        if !st.global_pos_valid {
            st.map_download_status = "ERROR".into();
            st.map_download_error = "no GPS position (need a GPS fix / GLOBAL_POSITION_INT)".into();
            st.map_download_path.clear();
            st.error("Satellite image: no GPS position available");
            return false;
        }
        let robot = (st.global_lat, st.global_lon);
        let home = if st.local_origin_set { (st.local_origin_lat, st.local_origin_lon) } else { robot };
        let kml = st.kml.clone().unwrap_or_default();
        st.map_download_active = true;
        st.map_download_status = "STARTING".into();
        st.map_download_error.clear();
        st.map_download_path.clear();
        st.map_download_started_at = now();
        let note = if kml.waypoints.is_empty() && kml.fence_rings.is_empty() {
            String::new()
        } else {
            format!(" + KML overlay ({} waypoint(s), {} fence ring(s))", kml.waypoints.len(), kml.fence_rings.len())
        };
        st.command(format!(
            "Satellite image requested: robot {:.6},{:.6} home {:.6},{:.6}{note}",
            robot.0, robot.1, home.0, home.1
        ));
        (robot, home, kml)
    };
    let (shared, map_dir) = (shared.clone(), map_dir.to_string());
    std::thread::spawn(move || {
        let (robot, home, kml) = job;
        let result = download(&shared, &map_dir, robot, home, &kml);
        let mut st = state::lock(&shared);
        st.map_download_active = false;
        match result {
            Ok(path) => {
                st.map_download_status = "COMPLETE".into();
                st.info(format!("Satellite image saved: {path}"));
                st.map_download_path = path;
            }
            Err(e) => {
                st.map_download_status = "ERROR".into();
                st.error(format!("Satellite image download failed: {e}"));
                st.map_download_error = e;
            }
        }
    });
    true
}

fn download(shared: &Shared, map_dir: &str, robot: (f64, f64), home: (f64, f64), kml: &geo::Kml) -> Result<String, String> {
    std::fs::create_dir_all(map_dir).map_err(|e| e.to_string())?;
    let (rx, ry) = web_mercator(robot.0, robot.1);
    let (hx, hy) = web_mercator(home.0, home.1);

    // Frame everything, but ignore KML geometry more than MAP_RANGE_MAX_M
    // from the robot/home area (multi-site exports would zoom out too far).
    let (ref_x, ref_y) = ((rx + hx) / 2.0, (ry + hy) / 2.0);
    let mut pts = vec![(rx, ry), (hx, hy)];
    let near = |p: (f64, f64)| (p.0 - ref_x).hypot(p.1 - ref_y) <= MAP_RANGE_MAX_M;
    pts.extend(kml.waypoints.iter().map(|w| web_mercator(w.lat, w.lon)).filter(|p| near(*p)));
    pts.extend(kml.fence_rings.iter().flatten().map(|&(la, lo)| web_mercator(la, lo)).filter(|p| near(*p)));
    let (min_x, max_x) = pts.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.0), b.max(p.0)));
    let (min_y, max_y) = pts.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.1), b.max(p.1)));
    let (cx, cy) = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
    let span = ((max_x - min_x).max(max_y - min_y) * MAP_IMAGE_PAD).max(MAP_IMAGE_MIN_SPAN_M);
    let half = span / 2.0;
    let (bx0, bx1, by0, by1) = (cx - half, cx + half, cy - half, cy + half);
    let size = MAP_IMAGE_SIZE;

    let url = format!(
        "https://server.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/export?\
         bbox={bx0:.2},{by0:.2},{bx1:.2},{by1:.2}&bboxSR=3857&imageSR=3857&size={size},{size}\
         &format=png&transparent=false&f=image"
    );
    state::lock(shared).map_download_status = "DOWNLOADING".into();
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(std::time::Duration::from_secs(MAP_DOWNLOAD_TIMEOUT_S)))
        .build()
        .into();
    let data = agent
        .get(&url)
        .header("User-Agent", "lazypx4")
        .call()
        .map_err(|e| format!("network error: {e}"))?
        .body_mut()
        .read_to_vec()
        .map_err(|e| format!("network error: {e}"))?;
    if data.len() < 512 {
        return Err("imagery server returned no image".into());
    }

    let stamp = chrono::Local::now().format("%Y%m%d_%H%M%S");
    let base = Path::new(map_dir).join(format!("px4_map_{stamp}"));
    let png_path = base.with_extension("png");

    let to_px = |(x, y): (f64, f64)| ((x - bx0) / (bx1 - bx0) * size as f64, (by1 - y) / (by1 - by0) * size as f64);
    let home_px = to_px((hx, hy));
    let robot_px = to_px((rx, ry));
    // Web-Mercator inflates distances by 1/cos(lat); undo for ground metres.
    let merc = ((robot.0 + home.0) / 2.0).to_radians().cos().max(0.05);
    let distance = (rx - hx).hypot(ry - hy) * merc;
    let ground_res = span / size as f64 * merc;

    let mut img = image::load_from_memory(&data).map_err(|e| format!("bad image: {e}"))?.to_rgb8();
    annotate(&mut img, kml, &to_px, &web_mercator, home_px, robot_px, distance, ground_res, span * merc);
    img.save(&png_path).map_err(|e| e.to_string())?;

    let r1 = |v: f64| (v * 10.0).round() / 10.0;
    let mut sidecar = serde_json::json!({
        "captured_utc": chrono::Utc::now().to_rfc3339(),
        "source": "Esri World Imagery (server.arcgisonline.com)",
        "annotated": true,
        "image": png_path.file_name().unwrap().to_string_lossy(),
        "image_size_px": [size, size],
        "bbox_epsg3857": [bx0, by0, bx1, by1],
        "ground_resolution_m_per_px": (ground_res * 1000.0).round() / 1000.0,
        "home": {"lat": home.0, "lon": home.1, "pixel": [r1(home_px.0), r1(home_px.1)]},
        "robot": {"lat": robot.0, "lon": robot.1, "pixel": [r1(robot_px.0), r1(robot_px.1)],
                  "distance_from_home_m": r1(distance)},
    });
    if !kml.waypoints.is_empty() || !kml.fence_rings.is_empty() {
        sidecar["kml"] = serde_json::json!({
            "waypoints": kml.waypoints.iter().map(|w| {
                let p = to_px(web_mercator(w.lat, w.lon));
                serde_json::json!({"name": w.name, "lat": w.lat, "lon": w.lon, "alt": w.alt, "pixel": [r1(p.0), r1(p.1)]})
            }).collect::<Vec<_>>(),
            "fence_rings": kml.fence_rings.iter().map(|r| r.iter().map(|&(la, lo)| {
                let p = to_px(web_mercator(la, lo));
                [r1(p.0), r1(p.1)]
            }).collect::<Vec<_>>()).collect::<Vec<_>>(),
        });
    }
    std::fs::write(base.with_extension("json"), serde_json::to_string_pretty(&sidecar).unwrap()).map_err(|e| e.to_string())?;
    Ok(png_path.display().to_string())
}

// ---------------------------------------------------------------------------
// Minimal raster drawing: thick lines, rings, 8x8 bitmap text.
// ---------------------------------------------------------------------------

fn plot(img: &mut RgbImage, x: i64, y: i64, c: Rgb<u8>) {
    if x >= 0 && y >= 0 && (x as u32) < img.width() && (y as u32) < img.height() {
        img.put_pixel(x as u32, y as u32, c);
    }
}

fn line(img: &mut RgbImage, (x0, y0): (f64, f64), (x1, y1): (f64, f64), c: Rgb<u8>, width: i64) {
    let steps = ((x1 - x0).abs().max((y1 - y0).abs()).ceil() as i64).max(1);
    let r = width / 2;
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let (x, y) = ((x0 + (x1 - x0) * t).round() as i64, (y0 + (y1 - y0) * t).round() as i64);
        for dx in -r..=r {
            for dy in -r..=r {
                plot(img, x + dx, y + dy, c);
            }
        }
    }
}

fn ring(img: &mut RgbImage, (cx, cy): (f64, f64), radius: f64, c: Rgb<u8>, width: f64) {
    let n = (radius * 8.0).max(24.0) as i64;
    for i in 0..n {
        let a = i as f64 / n as f64 * 2.0 * PI;
        let mut rr = radius - width / 2.0;
        while rr <= radius + width / 2.0 {
            plot(img, (cx + rr * a.cos()).round() as i64, (cy + rr * a.sin()).round() as i64, c);
            rr += 0.5;
        }
    }
}

/// 8x8 bitmap text (ASCII), drawn with a 1 px dark outline for contrast.
fn text(img: &mut RgbImage, x: f64, y: f64, s: &str, c: Rgb<u8>) {
    use font8x8::UnicodeFonts;
    let (x0, y0) = (x.round() as i64, y.round() as i64);
    for pass in [Rgb([0, 0, 0]), c] {
        let outline = pass == Rgb([0, 0, 0]);
        for (i, ch) in s.chars().enumerate() {
            let Some(glyph) = font8x8::BASIC_FONTS.get(ch) else { continue };
            for (row, bits) in glyph.iter().enumerate() {
                for col in 0..8 {
                    if bits >> col & 1 == 1 {
                        let (px, py) = (x0 + i as i64 * 8 + col, y0 + row as i64);
                        if outline {
                            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                                plot(img, px + dx, py + dy, pass);
                            }
                        } else {
                            plot(img, px, py, pass);
                        }
                    }
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn annotate(
    img: &mut RgbImage,
    kml: &geo::Kml,
    to_px: &dyn Fn((f64, f64)) -> (f64, f64),
    merc: &dyn Fn(f64, f64) -> (f64, f64),
    home_px: (f64, f64),
    robot_px: (f64, f64),
    distance: f64,
    metres_per_px: f64,
    ground_span: f64,
) {
    // Scale bar: 25 m doubled until it's at least 90 px.
    let size = img.height() as f64;
    let mut bar_m = 25.0;
    while bar_m / metres_per_px < 90.0 && bar_m < ground_span {
        bar_m *= 2.0;
    }
    let yellow = Rgb([255, 235, 0]);
    let y_bar = size - 28.0;
    line(img, (20.0, y_bar), (20.0 + bar_m / metres_per_px, y_bar), yellow, 4);
    text(img, 20.0, y_bar - 16.0, &format!("{bar_m:.0} m"), yellow);

    let fence = Rgb([255, 220, 0]);
    for r in &kml.fence_rings {
        let pts: Vec<(f64, f64)> = r.iter().map(|&(la, lo)| to_px(merc(la, lo))).collect();
        for (a, b) in pts.iter().zip(pts.iter().cycle().skip(1)).take(pts.len()) {
            line(img, *a, *b, fence, 3);
        }
    }
    let wp = Rgb([230, 60, 220]);
    for (i, w) in kml.waypoints.iter().enumerate() {
        let p = to_px(merc(w.lat, w.lon));
        ring(img, p, 7.0, wp, 2.0);
        let label = if w.name.is_empty() { format!("WP{}", i + 1) } else { w.name.clone() };
        text(img, p.0 + 10.0, p.1 - 11.0, &format!("{}:{label}", i + 1), wp);
    }
    let marker = |img: &mut RgbImage, (x, y): (f64, f64), c: Rgb<u8>, label: &str| {
        let r = 11.0;
        line(img, (x - r - 5.0, y), (x + r + 5.0, y), c, 2);
        line(img, (x, y - r - 5.0), (x, y + r + 5.0), c, 2);
        ring(img, (x, y), r, c, 3.0);
        text(img, x + r + 4.0, y - r - 6.0, label, c);
    };
    marker(img, home_px, Rgb([255, 70, 70]), "HOME");
    marker(img, robot_px, Rgb([80, 170, 255]), &format!("ROBOT  {distance:.0} m"));
}
