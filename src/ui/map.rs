//! The [n] mission map: an ASCII plan view (up = North). Mirrors
//! `render/mapview.py`.
//!
//! The vehicle is drawn twice when the data is there: the green heading
//! arrow is the EKF local solution, the cyan `⊕` the raw GNSS fix projected
//! through GPS_GLOBAL_ORIGIN - watching them converge is the quickest read on
//! an RTK bring-up. Layers, lowest priority first: axes, trail (Braille),
//! KML fence (yellow dots), planned queue path (cyan Braille), ROS navpath
//! (blue Braille), LiDAR scan (height-coloured Braille), then the markers.

use std::collections::HashMap;

use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};

use super::*;
use crate::geo;
use crate::state::{LidarFrame, Target, now};

pub const BRAILLE_BITS: [[u8; 4]; 2] = [[0x01, 0x02, 0x04, 0x40], [0x08, 0x10, 0x20, 0x80]];

pub fn braille(bits: u8) -> char {
    char::from_u32(0x2800 + bits as u32).unwrap_or(' ')
}

/// Low-to-high colour bands for height / depth, as LiDAR viewers do.
pub const BANDS: [Color; 5] = [Color::Blue, Color::Cyan, Color::Green, Color::Yellow, Color::Red];

pub fn band_color(v: f64, lo: f64, hi: f64) -> Color {
    if hi <= lo {
        return BANDS[0];
    }
    let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
    BANDS[((t * BANDS.len() as f64) as usize).min(BANDS.len() - 1)]
}

pub fn heading_arrow(yaw_deg: f64) -> char {
    if !yaw_deg.is_finite() {
        return '?';
    }
    ['↑', '↗', '→', '↘', '↓', '↙', '←', '↖'][((yaw_deg.rem_euclid(360.0) / 45.0 + 0.5) as usize) % 8]
}

#[derive(Clone, Copy, PartialEq)]
enum Layer {
    Empty,
    Axis,
    Trail,
    Fence,
    Path,
    Marker,
}

#[derive(Clone)]
struct Cell {
    ch: char,
    style: Style,
    layer: Layer,
}

/// Height of a terminal row in column widths; a Braille dot is then square.
pub const CELL_ASPECT: f64 = 2.0;

/// North half-extent / East half-extent of a `w` x `h` grid drawn to scale.
fn n_over_e(w: usize, h: usize) -> f64 {
    (h / 2) as f64 * CELL_ASPECT / ((w / 2) as f64).max(1.0)
}

/// A character grid over the local N/E plane, with Braille sub-cells.
/// Drawn to scale: `range` is the East half-extent, `range_n` the North one.
struct Grid {
    w: usize,
    h: usize,
    cells: Vec<Cell>,
    center: (f64, f64),
    range: f64,
    range_n: f64,
}

impl Grid {
    fn new(w: usize, h: usize, center: (f64, f64), range: f64) -> Self {
        let range_n = range * n_over_e(w, h);
        Grid { w, h, cells: vec![Cell { ch: ' ', style: Style::new(), layer: Layer::Empty }; w * h], center, range, range_n }
    }

    fn cell_of(&self, n: f64, e: f64) -> Option<(usize, usize)> {
        let (hw, hh) = ((self.w / 2) as f64, (self.h / 2) as f64);
        let col = ((e - self.center.1) / self.range * hw).round() + hw;
        let row = hh - ((n - self.center.0) / self.range_n * hh).round();
        (col.is_finite() && row.is_finite() && col >= 0.0 && row >= 0.0 && (col as usize) < self.w && (row as usize) < self.h)
            .then_some((row as usize, col as usize))
    }

    /// Place a glyph; `only_over` limits which layers it may replace.
    fn put(&mut self, n: f64, e: f64, ch: char, style: Style, layer: Layer, only_over: &[Layer]) {
        if let Some((r, c)) = self.cell_of(n, e) {
            let cell = &mut self.cells[r * self.w + c];
            if only_over.is_empty() || only_over.contains(&cell.layer) {
                *cell = Cell { ch, style, layer };
            }
        }
    }

    fn marker(&mut self, n: f64, e: f64, ch: char, style: Style) {
        self.put(n, e, ch, style, Layer::Marker, &[]);
    }

    /// Dot position in the 2x4-per-cell Braille raster.
    fn dot_of(&self, n: f64, e: f64) -> Option<(usize, usize)> {
        let (dw, dh) = ((self.w * 2) as f64, (self.h * 4) as f64);
        let col = ((e - self.center.1) / self.range * dw / 2.0).round() + dw / 2.0;
        let row = dh / 2.0 - ((n - self.center.0) / self.range_n * dh / 2.0).round();
        (col.is_finite() && row.is_finite() && col >= 0.0 && row >= 0.0 && col < dw && row < dh).then_some((row as usize, col as usize))
    }

    /// Accumulate points into Braille cells: cell -> (bits, max value).
    fn braille_points(&self, pts: impl Iterator<Item = (f64, f64, f64)>) -> HashMap<(usize, usize), (u8, f64)> {
        let mut out: HashMap<(usize, usize), (u8, f64)> = HashMap::new();
        for (n, e, v) in pts {
            if let Some((dr, dc)) = self.dot_of(n, e) {
                let entry = out.entry((dr / 4, dc / 2)).or_insert((0, f64::MIN));
                entry.0 |= BRAILLE_BITS[dc % 2][dr % 4];
                entry.1 = entry.1.max(v);
            }
        }
        out
    }

    /// Densely sample a polyline into Braille dots.
    fn braille_polyline(&self, pts: &[(f64, f64)]) -> HashMap<(usize, usize), (u8, f64)> {
        let dot = (self.range / self.w.max(1) as f64).max(0.001);
        let samples = pts.windows(2).flat_map(|seg| {
            let ((an, ae), (bn, be)) = (seg[0], seg[1]);
            let steps = ((bn - an).hypot(be - ae) / dot).clamp(1.0, 2000.0) as usize;
            (0..=steps).map(move |i| {
                let t = i as f64 / steps as f64;
                (an + (bn - an) * t, ae + (be - ae) * t, 0.0)
            })
        });
        self.braille_points(samples)
    }

    fn paint_braille(&mut self, cells: HashMap<(usize, usize), (u8, f64)>, style: impl Fn(f64) -> Style, layer: Layer, only_over: &[Layer]) {
        for ((r, c), (bits, v)) in cells {
            let cell = &mut self.cells[r * self.w + c];
            if only_over.contains(&cell.layer) {
                *cell = Cell { ch: braille(bits), style: style(v), layer };
            }
        }
    }

    fn lines(&self) -> Vec<Line<'static>> {
        self.cells
            .chunks(self.w)
            .map(|row| {
                let mut spans = vec![Span::raw("  ")];
                spans.extend(row.iter().map(|c| Span::styled(c.ch.to_string(), c.style)));
                Line::from(spans)
            })
            .collect()
    }
}

fn wrap_keys(keys: &[&str], width: usize, sep: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    for k in keys {
        let cand = if cur.is_empty() { k.to_string() } else { format!("{cur}{sep}{k}") };
        if !cur.is_empty() && cand.chars().count() > width {
            out.push(std::mem::replace(&mut cur, k.to_string()));
        } else {
            cur = cand;
        }
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

const MAP_KEYS: &[&str] = &[
    "[g] goto", "[W] wp queue", "[C] coverage", "[P] publish gps", "[x] jog", "[o] load kml",
    "[O] upload fence", "[+]/[-] zoom", "[arrows] pan", "[u] follow", "[0] reset", "[f] fit kml", "[t] trail", "[c] clear",
    "[V] lidar", "[B] lidar frame", "[N] navpath", "[i] sat", "[n] back", "ESC panels",
];

fn baseline_text(m: f64) -> String {
    if m <= 0.0 {
        "n/a".into()
    } else if m >= 1000.0 {
        format!("{:.2} km", m / 1000.0)
    } else {
        format!("{m:.0} m")
    }
}

fn offset_verdict(delta: f64) -> Ln {
    let c = if delta.abs() < 0.5 { GREEN } else if delta.abs() < 2.0 { YELLOW } else { RED };
    Ln::new().fg(format!("{delta:+.2} m"), c)
}

pub fn draw(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let session = &ctx.app.session;
    let ros = crate::host::lock(&ctx.app.ros);
    let t = now();
    let bold = |c: Color| Style::new().fg(c).add_modifier(Modifier::BOLD);

    let (lx, ly, lz) = (st.local_x, st.local_y, st.local_z);
    let valid = st.local_pos_valid;
    let origin = st.local_origin_set.then_some((st.local_origin_lat, st.local_origin_lon));
    let local = |lat: f64, lon: f64| origin.and_then(|o| geo::to_local(lat, lon, o));

    let gps_local = if st.global_pos_valid { local(st.global_lat, st.global_lon) } else { None };
    let gps_offset = gps_local.filter(|_| valid).map(|(n, e)| (n - lx, e - ly));
    let home_local = if st.home_set { local(st.home_lat, st.home_lon) } else { None };

    let kml = st.kml.as_ref();
    let waypoints_local: Vec<Option<(f64, f64)>> =
        kml.map(|k| k.waypoints.iter().map(|w| local(w.lat, w.lon)).collect()).unwrap_or_default();
    let rings_local: Vec<Vec<(f64, f64)>> = kml
        .map(|k| {
            k.fence_rings
                .iter()
                .map(|r| r.iter().filter_map(|&(la, lo)| local(la, lo)).collect::<Vec<_>>())
                .filter(|r| r.len() >= 3)
                .collect()
        })
        .unwrap_or_default();

    // The path awaiting its YES is previewed; else the running queue's.
    let preview: Option<&Vec<Target>> = match &session.confirm {
        Some(crate::app::Confirm { action: crate::app::Action::StartQueue { targets, .. }, .. }) => Some(targets),
        _ => None,
    };
    let path_targets: Vec<Target> = preview.cloned().unwrap_or_else(|| if st.wp_queue_active { st.wp_path.clone() } else { Vec::new() });
    let path_local: Vec<(f64, f64)> = path_targets.iter().filter_map(|p| local(p.lat, p.lon)).collect();
    let path_len: f64 = path_targets.windows(2).map(|w| geo::distance_m(w[0].lat, w[0].lon, w[1].lat, w[1].lon)).sum();
    let queue_target_local = if st.wp_queue_active { st.wp_queue.front().and_then(|q| local(q.lat, q.lon)) } else { None };

    let fire = ros.fire.filter(|_| ros.fire_last > 0.0);
    let fire_local = fire.and_then(|(la, lo, _)| local(la, lo));
    let fire_rel = match (fire, fire_local) {
        (Some((_, _, alt)), Some((fnn, fe))) if valid => {
            let fd = if alt.is_finite() { -(alt - st.local_origin_alt) } else { 0.0 };
            let (rn, re, rd) = (fnn - lx, fe - ly, fd - lz);
            Some((rn, re, rd, (rn * rn + re * re + rd * rd).sqrt()))
        }
        _ => None,
    };
    let navpath: &[(f64, f64)] = if st.map_navpath_enabled { &ros.navpath_points } else { &[] };

    let mut lines: Vec<Line<'static>> = Vec::new();

    if !ros.gps_feedback.is_empty() {
        let bg = if ros.gps_feedback_ok { GREEN } else { RED };
        lines.push(Line::from(Span::styled(
            format!(" [P] GPS {} ", ros.gps_feedback),
            Style::new().bg(bg).fg(Color::Black).add_modifier(Modifier::BOLD),
        )));
    }
    let arm = if st.armed { Ln::new().fg("ARMED", GREEN) } else { Ln::new().dim("DISARMED") };
    lines.push(Ln::new().raw(" MODE: ").bold(st.mode.clone()).raw("   ").spans(arm.0).line());

    if session.jog_armed {
        lines.push(Line::from(Span::styled(
            format!(
                " JOG ARMED   step {:.2} m / {JOG_YAW_STEP_DEG:.0}°   w/s up·down  k/j fwd·back  a/d strafe  h/l yaw  [/] step  x off ",
                session.jog_step
            ),
            Style::new().bg(RED).fg(Color::White).add_modifier(Modifier::BOLD),
        )));
        if !st.last_ack.is_empty() {
            lines.push(Ln::new().dim(format!(" last command: {}", st.last_ack)).line());
        }
        if !st.armed {
            lines.push(Ln::new().fg(" vehicle not armed - nudges will be refused", YELLOW).line());
        }
    } else {
        lines.push(
            Ln::new()
                .raw(" FLIGHT: ")
                .key("a", "arm")
                .key("d", "disarm")
                .key("T", "takeoff")
                .key("L", "land")
                .key("R", "RTL")
                .key("h", "hold")
                .st("[K]", bold(RED))
                .raw("kill  ")
                .key("H", "home")
                .key("G", "fence")
                .line(),
        );
    }

    // --- read-out blocks shared by both layouts ---------------------------
    let gnss_lines = || -> Vec<Line<'static>> {
        let mut out = Vec::new();
        let (fc, fname) = dashboard::gps_fix_display(st.gps_fix);
        out.push(
            Ln::new()
                .raw(" GNSS      ")
                .fg(fname, fc)
                .raw(format!("   sats {}   EPH {:.3} m   EPV {:.3} m", st.gps_sats, st.gps_h_acc, st.gps_v_acc))
                .dim(if st.last_gps > 0.0 { format!("   {}", age_text(st.last_gps)) } else { String::new() })
                .line(),
        );
        if st.global_pos_valid {
            out.push(Line::from(format!(
                " Global    {:.7}, {:.7}   alt {:.1} m (MSL)",
                st.global_lat, st.global_lon, st.global_alt
            )));
        }
        if st.last_gps_rtk > 0.0 {
            let health = match st.rtk_health {
                1 => Ln::new().fg("OK", GREEN),
                0 => Ln::new().fg("unhealthy", YELLOW),
                _ => Ln::new().dim("?"),
            };
            let corr = if st.gps_dgps_age > 0.0 { format!("   corr age {:.1}s", st.gps_dgps_age) } else { String::new() };
            out.push(
                Ln::new()
                    .raw(" RTK       health ")
                    .spans(health.0)
                    .raw(format!(
                        "   corrections {:.1} Hz   base {}   nsat {}{corr}",
                        st.rtk_rate,
                        baseline_text(st.rtk_baseline_m),
                        st.rtk_nsats
                    ))
                    .dim(format!("   {}", age_text(st.last_gps_rtk)))
                    .line(),
            );
        } else {
            out.push(Ln::new().raw(" RTK       ").dim("no GPS_RTK - receiver is not being fed RTCM corrections").line());
        }
        if let Some((dn, de)) = gps_offset {
            let d = dn.hypot(de);
            let c = if d < 0.5 { GREEN } else if d < 2.0 { YELLOW } else { RED };
            out.push(
                Ln::new()
                    .raw(" EKF↔GNSS  ")
                    .fg(format!("{d:.2} m"), c)
                    .raw(format!(" offset   (N {dn:+.2}  E {de:+.2})   "))
                    .fg("⊕", CYAN)
                    .raw(" = GNSS fix on the grid")
                    .line(),
            );
        } else if valid && st.global_pos_valid && origin.is_none() {
            out.push(Ln::new().raw(" EKF↔GNSS  ").dim("no local origin yet - cannot place the GNSS fix on the grid").line());
        }
        out
    };

    // Three heights above ground from independent sources; on flat ground
    // they should agree, and the offsets show which one is off.
    let altitude_lines = || -> Vec<Line<'static>> {
        let mut out = Vec::new();
        let ground = kml.and_then(|k| k.ground_alt);
        let mut h_site = None;
        if st.global_pos_valid {
            if let Some(g) = ground {
                let h = st.global_alt - g;
                h_site = Some(h);
                out.push(
                    Ln::new()
                        .raw(format!(" ALTITUDE  EKF {:.1} m AMSL   KML site {g:.1} m   ", st.global_alt))
                        .bold(format!("{h:+.2} m"))
                        .raw(" above site")
                        .line(),
                );
            } else {
                out.push(
                    Ln::new()
                        .raw(format!(" ALTITUDE  EKF {:.1} m AMSL   ", st.global_alt))
                        .dim(if kml.is_some() {
                            "no altitude in the KML - no site reference"
                        } else {
                            "[o] load a KML with altitude for a site reference"
                        })
                        .line(),
                );
            }
        } else {
            out.push(Ln::new().raw(" ALTITUDE  ").dim("no global position yet").line());
        }
        if valid && st.global_pos_valid && origin.is_some() {
            let (h_local, h_origin) = (-lz, st.global_alt - st.local_origin_alt);
            out.push(
                Ln::new()
                    .raw(format!("   local/global  -D {h_local:+.2} m   AMSL-origin {h_origin:+.2} m   diff "))
                    .spans(offset_verdict(h_local - h_origin).0)
                    .line(),
            );
        }
        let rng_age = (st.last_rangefinder > 0.0).then_some(t - st.last_rangefinder);
        let rd = st.rangefinder_distance;
        match rng_age {
            None => out.push(Ln::new().raw("   rangefinder   ").dim("no DISTANCE_SENSOR data").line()),
            Some(a) if a > 2.0 => out.push(Ln::new().raw("   rangefinder   ").fg(format!("stale ({a:.0}s ago)"), YELLOW).line()),
            _ if !(st.rangefinder_min..=st.rangefinder_max).contains(&rd) => out.push(
                Ln::new()
                    .raw("   rangefinder   ")
                    .fg(format!("{rd:.2} m out of range"), YELLOW)
                    .dim(format!("   (valid {:.2}-{:.2} m)", st.rangefinder_min, st.rangefinder_max))
                    .line(),
            ),
            _ => {
                let q = if st.rangefinder_quality >= 0 { format!("   q {}", st.rangefinder_quality) } else { String::new() };
                let reference = h_site.map(|h| (h, "above site")).or(valid.then_some((-lz, "local -D")));
                let mut l = Ln::new().raw(format!("   rangefinder   {rd:.2} m{q}"));
                if let Some((r, name)) = reference {
                    l = l.raw(format!("   vs {r:+.2} m {name}   diff ")).spans(offset_verdict(rd - r).0);
                }
                out.push(l.line());
            }
        }
        out
    };

    let satellite_lines = || -> Vec<Line<'static>> {
        if st.map_download_active {
            vec![Ln::new().fg(format!(" Satellite image: {} ({:.0}s)...", st.map_download_status, t - st.map_download_started_at), YELLOW).line()]
        } else if st.map_download_status == "COMPLETE" && !st.map_download_path.is_empty() {
            vec![Ln::new().fg(format!(" Satellite image saved: {}", st.map_download_path), GREEN).line()]
        } else if st.map_download_status == "ERROR" && !st.map_download_error.is_empty() {
            vec![Ln::new().fg(format!(" Satellite image failed: {}", st.map_download_error), RED).line()]
        } else {
            Vec::new()
        }
    };

    let kml_name = std::path::Path::new(&st.kml_path).file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();

    if !valid {
        lines.push(blank());
        lines.push(Ln::new().fg(" Waiting for LOCAL_POSITION_NED / ODOMETRY ...", RED).line());
        lines.push(blank());
        lines.push(Ln::new().dim(" PX4 only publishes a local position once the estimator has one").line());
        lines.push(Ln::new().dim(" (needs GPS lock, or flow/vision). Global / GNSS detail below.").line());
        lines.push(blank());
        lines.extend(gnss_lines());
        lines.extend(altitude_lines());
        let sat = satellite_lines();
        if !sat.is_empty() {
            lines.push(blank());
            lines.extend(sat);
        }
        if kml.is_some() {
            lines.push(blank());
            lines.push(Ln::new().raw(format!(" KML       {kml_name}   ")).dim("loaded").line());
        } else if !st.kml_error.is_empty() {
            lines.push(blank());
            lines.push(Ln::new().raw(" KML       ").fg(format!("load failed: {}", st.kml_error), RED).line());
        }
        lines.push(blank());
        let keys = ["[i] satellite image", "[o] load kml", "[O] upload fence", "[n] back", "ESC panels"];
        lines.extend(wrap_keys(&keys, ctx.width, "   ").into_iter().map(Line::from));
        return lines;
    }

    // --- the grid --------------------------------------------------------
    let key_lines = wrap_keys(MAP_KEYS, ctx.width.saturating_sub(1), "   ");
    let grid_w = ((ctx.width.saturating_sub(3)) | 1).clamp(21, 103);
    let grid_h = ((ctx.height.saturating_sub(17 + key_lines.len())) | 1).clamp(11, 35);

    let target = (st.last_pos_target > 0.0 && t - st.last_pos_target < 5.0)
        .then_some((st.pos_target_x, st.pos_target_y))
        .filter(|(x, y)| x.is_finite() && y.is_finite());

    let in_range = |p: &(f64, f64)| p.0.abs() <= MAP_RANGE_MAX_M && p.1.abs() <= MAP_RANGE_MAX_M;
    let mut geometry: Vec<(f64, f64)> = waypoints_local.iter().flatten().copied().collect();
    geometry.extend(rings_local.iter().flatten().copied());
    geometry.extend(path_local.iter().copied());
    geometry.extend(navpath.iter().copied());
    geometry.extend(fire_local);
    geometry.retain(in_range);

    // To scale: a North extent needs 1/k times the East half-range.
    let k = n_over_e(grid_w, grid_h);
    let (center, range) = if st.map_fit_kml && !geometry.is_empty() {
        // [f]: centre on the geometry (plus the live markers) itself.
        let mut all = vec![(lx, ly)];
        all.extend(&geometry);
        all.extend(target);
        all.extend(gps_local);
        let (n0, n1) = all.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.0), b.max(p.0)));
        let (e0, e1) = all.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.1), b.max(p.1)));
        let c = ((n0 + n1) / 2.0, (e0 + e1) / 2.0);
        let r = all.iter().map(|p| ((p.0 - c.0).abs() / k).max((p.1 - c.1).abs())).fold(0.0, f64::max) * 1.08;
        (c, r.max(2.0))
    } else if st.map_follow || st.map_pan != (0.0, 0.0) {
        // [u] follow and/or arrow-key pan: a plain window at the set range.
        let base = if st.map_follow { (lx, ly) } else { (0.0, 0.0) };
        ((base.0 + st.map_pan.0, base.1 + st.map_pan.1), st.map_range.max(2.0))
    } else {
        // Origin-centred; the range always reaches the vehicle, GNSS fix,
        // setpoint and (nearby) KML geometry.
        let mut r = st.map_range;
        let mut reach = |p: (f64, f64)| r = r.max(p.0.abs() / k * 1.08).max(p.1.abs() * 1.08);
        reach((lx, ly));
        target.into_iter().chain(gps_local).chain(geometry.iter().copied()).for_each(&mut reach);
        ((0.0, 0.0), r.max(2.0))
    };

    let mut grid = Grid::new(grid_w, grid_h, center, range);
    let (hw, hh) = ((grid_w / 2) as f64, (grid_h / 2) as f64);
    let range_n = grid.range_n;
    let dim = Style::new().add_modifier(Modifier::DIM);

    // Axes through the local origin.
    for c in 0..grid_w {
        grid.put(0.0, center.1 + (c as f64 - hw) / hw * range, '─', Style::new(), Layer::Axis, &[Layer::Empty]);
    }
    for r in 0..grid_h {
        grid.put(center.0 + (hh - r as f64) / hh * range_n, 0.0, '│', Style::new(), Layer::Axis, &[Layer::Empty, Layer::Axis]);
    }
    grid.put(0.0, 0.0, '┼', Style::new(), Layer::Axis, &[]);

    if st.map_trail_enabled && !st.position_trail.is_empty() {
        let cells = grid.braille_points(st.position_trail.iter().map(|&(n, e)| (n, e, 0.0)));
        grid.paint_braille(cells, |_| dim, Layer::Trail, &[Layer::Empty]);
    }

    if !rings_local.is_empty() {
        let cell_size = (range / hw.max(hh)).max(0.01);
        for ring in &rings_local {
            for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)).take(ring.len()) {
                let steps = ((b.0 - a.0).hypot(b.1 - a.1) / cell_size).clamp(1.0, 400.0) as usize;
                for i in 0..=steps {
                    let f = i as f64 / steps as f64;
                    grid.put(a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f, '.', Style::new().fg(YELLOW), Layer::Fence, &[Layer::Empty]);
                }
            }
        }
    }

    if path_local.len() >= 2 {
        let cells = grid.braille_polyline(&path_local);
        grid.paint_braille(cells, |_| Style::new().fg(CYAN), Layer::Path, &[Layer::Empty, Layer::Fence]);
        grid.marker(path_local[0].0, path_local[0].1, 'S', bold(GREEN));
        let last = path_local[path_local.len() - 1];
        grid.marker(last.0, last.1, 'E', bold(RED));
    }

    if navpath.len() >= 2 {
        let cells = grid.braille_polyline(navpath);
        grid.paint_braille(cells, |_| Style::new().fg(Color::Blue), Layer::Path, &[Layer::Empty, Layer::Fence]);
        // A round trip ends on its start: draw `e` first so `s` stays.
        let (s, e) = (navpath[0], navpath[navpath.len() - 1]);
        grid.marker(e.0, e.1, 'e', bold(RED));
        grid.marker(s.0, s.1, 's', bold(GREEN));
    }

    // LiDAR scan, lowest priority. Body frame (x fwd, y left): rotated by
    // yaw and placed at the vehicle, assuming a centred, body-aligned mount.
    // World frame (ENU map/odom): x east, y north, drawn as-is.
    let lidar_fresh = ros.lidar_last > 0.0 && t - ros.lidar_last < 2.0;
    let (z_lo, z_hi) = ros.lidar_points.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p[2]), b.max(p[2])));
    let lidar_frame = st.map_lidar_frame;
    if st.map_lidar_enabled && lidar_fresh {
        let (c, s) = (st.yaw.to_radians().cos(), st.yaw.to_radians().sin());
        let world = lidar_frame == LidarFrame::World;
        let pts = ros.lidar_points.iter().map(|p| {
            if world { (p[1], p[0], p[2]) } else { (lx + p[0] * c + p[1] * s, ly + p[0] * s - p[1] * c, p[2]) }
        });
        let cells = grid.braille_points(pts);
        grid.paint_braille(cells, |z| Style::new().fg(band_color(z, z_lo, z_hi)), Layer::Trail, &[Layer::Empty]);
    }

    for (i, w) in waypoints_local.iter().enumerate() {
        if let Some((n, e)) = w {
            let digit = char::from_digit(((i + 1) % 10) as u32, 10).unwrap();
            grid.marker(*n, *e, digit, bold(Color::Magenta));
        }
    }
    if let Some((n, e)) = queue_target_local {
        grid.marker(n, e, '◎', bold(Color::White));
    }
    match home_local {
        Some((n, e)) => grid.marker(n, e, 'H', Style::new().fg(YELLOW)),
        // No HOME_POSITION yet: the origin usually coincides with home.
        None if origin.is_some() => grid.marker(0.0, 0.0, 'H', dim),
        None => {}
    }
    if let Some((x, y)) = target {
        grid.marker(x, y, '*', Style::new());
    }
    if let Some((n, e)) = fire_local.filter(in_range) {
        grid.marker(n, e, 'F', bold(RED));
    }
    if let Some((n, e)) = gps_local {
        grid.marker(n, e, '⊕', Style::new().fg(CYAN));
    }
    grid.marker(lx, ly, heading_arrow(st.yaw), bold(GREEN));

    let mut header = Ln::new()
        .raw(format!(" range E +/-{range:.1} m  N +/-{range_n:.1} m      "))
        .dim(format!("cell ~ {:4.1} m x {:4.1} m (to scale)", range / hw, range_n / hh));
    if !st.map_fit_kml && (st.map_follow || st.map_pan != (0.0, 0.0)) {
        let what = if st.map_follow { "[u] follow" } else { "panned" };
        header = header.raw("   ").fg(what, GREEN).raw(format!("  centre N {:+.1} E {:+.1}", center.0, center.1));
    }
    if st.map_fit_kml {
        header = if geometry.is_empty() {
            header.raw("   ").dim("[f] fit:kml (no geometry loaded)")
        } else {
            header.raw("   ").fg("[f] fit:kml", GREEN).raw(format!("  centre N {:+.1} E {:+.1}", center.0, center.1))
        };
    }
    lines.push(header.line());
    lines.push(blank());
    lines.extend(grid.lines());
    lines.push(blank());

    lines.push(
        Ln::new()
            .raw(" Local ")
            .fg(heading_arrow(st.yaw).to_string(), GREEN)
            .raw(format!(
                "  N {lx:+8.2}   E {ly:+8.2}   D {lz:+8.2} m   Yaw {:6.1}°   Speed {:.2} m/s",
                st.yaw,
                st.vx.hypot(st.vy)
            ))
            .line(),
    );
    if let Some((x, y)) = target {
        lines.push(Line::from(format!(" Target *  N {x:+8.2}   E {y:+8.2} m   dist {:.2} m", (x - lx).hypot(y - ly))));
    }
    if let Some((la, lo)) = origin {
        lines.push(Line::from(format!(" Origin H  {la:.7}, {lo:.7}")));
    }
    if st.home_set {
        let mut l = Ln::new().raw(" Home      ").fg("H", YELLOW).raw(format!("  {:.7}, {:.7}", st.home_lat, st.home_lon));
        if home_local.is_none() {
            l = l.raw("  ").dim("(no local origin yet)");
        }
        lines.push(l.line());
    }
    lines.extend(gnss_lines());
    lines.extend(altitude_lines());

    if let Some(k) = kml {
        if origin.is_some() {
            lines.push(
                Ln::new()
                    .raw(format!(" KML       {kml_name}   waypoints ("))
                    .st("#", bold(Color::Magenta))
                    .raw(format!("): {}   fence (", k.waypoints.len()))
                    .fg(".", YELLOW)
                    .raw(format!("): {} ring(s)", k.fence_rings.len()))
                    .line(),
            );
        } else {
            lines.push(
                Ln::new()
                    .raw(format!(" KML       {kml_name}   "))
                    .dim("loaded, but no local origin yet - cannot place it on the grid")
                    .line(),
            );
        }
        for (i, (w, l)) in k.waypoints.iter().zip(&waypoints_local).enumerate() {
            let label: String = if w.name.is_empty() { format!("waypoint {}", i + 1) } else { w.name.clone() };
            let label: String = label.chars().take(16).collect();
            let local_text = match l {
                Some((n, e)) => Ln::new().raw(format!("N {n:+8.2}  E {e:+8.2} m")),
                None => Ln::new().dim("out of range"),
            };
            lines.push(
                Ln::new()
                    .raw("   ")
                    .st(((i + 1) % 10).to_string(), bold(Color::Magenta))
                    .raw(format!(" {label:<16} local "))
                    .spans(local_text.0)
                    .raw(format!("   global {:.7}, {:.7}   alt {:.1} m", w.lat, w.lon, w.alt))
                    .line(),
            );
        }
    } else if !st.kml_error.is_empty() {
        lines.push(Ln::new().raw(" KML       ").fg(format!("load failed: {}", st.kml_error), RED).line());
    }

    if !path_local.is_empty() {
        let mut l = Ln::new()
            .raw(if preview.is_some() { " PREVIEW   " } else { " PATH      " })
            .fg("⣿", CYAN)
            .raw(format!(" {} waypoint(s), {path_len:.0} m   ", path_targets.len()))
            .st("S", bold(GREEN))
            .raw(" start  ")
            .st("E", bold(RED))
            .raw(" end");
        if preview.is_some() {
            l = l.raw("   ").fg("awaiting YES", YELLOW);
        }
        lines.push(l.line());
    } else if preview.is_some() {
        lines.push(Ln::new().raw(" PREVIEW   ").dim("no local origin yet - cannot draw the path").line());
    }

    if st.wp_queue_active {
        if let Some(q) = st.wp_queue.front() {
            let done = st.wp_queue_total - st.wp_queue.len() + 1;
            let dist = queue_target_local.map(|(n, e)| format!("   dist {:.1} m", (n - lx).hypot(e - ly))).unwrap_or_default();
            lines.push(
                Ln::new()
                    .raw(" WP QUEUE  ")
                    .st("◎", bold(Color::White))
                    .raw(format!(
                        " [{}]  {done}/{} -> {}{dist}{}   ",
                        st.wp_queue_mode,
                        st.wp_queue_total,
                        q.name,
                        if st.wp_queue_face_target { "  facing target" } else { "" }
                    ))
                    .fg(st.wp_queue_status.clone(), GREEN)
                    .raw("   [W]/[C] cancel")
                    .line(),
            );
        }
    } else if !st.wp_queue_status.is_empty() && !st.wp_queue_mode.is_empty() {
        lines.push(Ln::new().raw(" WP QUEUE  ").dim(st.wp_queue_status.clone()).line());
    }

    if st.map_lidar_enabled {
        let note = if !ros.available {
            Ln::new().dim("no ROS 2 - LiDAR unavailable")
        } else if !lidar_fresh {
            Ln::new().fg("no recent scan", YELLOW).raw(format!(" on {}", ros.lidar_topic))
        } else {
            let mut l = Ln::new().raw(format!("{} pts   height {z_lo:.1} m ", ros.lidar_points.len()));
            for c in BANDS {
                l = l.fg("█", c);
            }
            let how = match lidar_frame {
                LidarFrame::World => "world ENU, drawn as-is",
                LidarFrame::Body => "robot frame, at vehicle, rotated by yaw",
            };
            let frame = if ros.lidar_frame_id.is_empty() { "--" } else { &ros.lidar_frame_id };
            l.raw(format!(" {z_hi:.1} m   ")).dim(format!("frame {frame} -> [B] {how}"))
        };
        lines.push(Ln::new().raw(" LIDAR     ").spans(note.0).line());
    }

    if let Some((la, lo, alt)) = fire {
        let mut l = Ln::new().raw(" FIRE      ").st("F", bold(RED)).raw(format!("  {la:.7}, {lo:.7}"));
        match fire_local {
            None => l = l.raw("   ").dim("no local origin yet - cannot place it on the grid"),
            Some((n, e)) => {
                l = l
                    .raw(format!("   alt {alt:.1} m   from origin H: N {n:+8.2}  E {e:+8.2} m"))
                    .dim(format!("   received {:.0}s ago", t - ros.fire_last));
            }
        }
        lines.push(l.line());
        if fire_local.is_some_and(|p| !in_range(&p)) {
            lines.push(Ln::new().fg(format!("   more than {:.0} km from the origin - not drawn on the grid", MAP_RANGE_MAX_M / 1000.0), YELLOW).line());
        }
        if let Some((rn, re, rd, d)) = fire_rel {
            lines.push(Line::from(format!("   Fire rel. UAV (NED)  N {rn:+8.2}   E {re:+8.2}   D {rd:+8.2} m   dist {d:.1} m")));
        }
    } else if ros.available {
        let note = if ros.fire_publishers.is_empty() {
            Ln::new().dim("no publisher seen")
        } else {
            Ln::new().fg("published but nothing received yet", YELLOW).raw(" - ").dim(ros.fire_publishers.join(", "))
        };
        lines.push(Ln::new().raw(" FIRE      ").spans(note.0).raw(format!(" on {GPS_PUBLISH_TOPIC}")).line());
    }

    if st.map_navpath_enabled {
        let note = if !ros.available {
            Ln::new().dim(format!("no ROS 2 - path unavailable ({})", ros.error))
        } else if ros.navpath_last == 0.0 && !ros.navpath_publishers.is_empty() {
            Ln::new()
                .fg("published but nothing received yet", YELLOW)
                .raw(" - ")
                .dim(ros.navpath_publishers.join(", "))
                .raw(format!(" on {}", ros.navpath_topic))
        } else if ros.navpath_last == 0.0 {
            Ln::new().dim("no publisher seen").raw(format!(" on {}", ros.navpath_topic))
        } else {
            let closed = navpath.len() >= 2 && navpath[0] == navpath[navpath.len() - 1];
            Ln::new()
                .fg("⣿", Color::Blue)
                .raw(format!(" {} pose(s)   ", navpath.len()))
                .st("s", bold(GREEN))
                .raw(" start  ")
                .st("e", bold(RED))
                .raw(if closed { " end (= start)   " } else { " end   " })
                .dim(format!("received {:.0}s ago (ENU map frame)", t - ros.navpath_last))
        };
        lines.push(Ln::new().raw(" NAVPATH   ").spans(note.0).line());
    }

    let total = st.fence_upload_items.len();
    if st.fence_upload_active {
        lines.push(
            Ln::new()
                .raw(" FENCE UP  ")
                .fg("uploading", YELLOW)
                .raw(format!("   vertex {}/{total}", st.fence_upload_acked_seq + 1))
                .line(),
        );
    } else if st.fence_upload_status == "COMPLETE" {
        lines.push(Ln::new().raw(" FENCE UP  ").fg("ACCEPTED by PX4", GREEN).raw(format!("   {total} vertice(s)")).line());
    } else if st.fence_upload_status == "ERROR" {
        lines.push(Ln::new().raw(" FENCE UP  ").fg(format!("failed: {}", st.fence_upload_error), RED).line());
    }

    lines.push(blank());
    let trail = if st.map_trail_enabled { format!("on ({})", st.position_trail.len()) } else { "off".into() };
    lines.push(Line::from(format!(
        " data age {:.1}s    trail {trail}",
        if st.last_local_pos > 0.0 { t - st.last_local_pos } else { 0.0 }
    )));
    lines.extend(satellite_lines());
    lines.extend(key_lines.into_iter().map(Line::from));
    lines
}
