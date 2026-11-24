//! The [n] mission map: an ASCII plan view (up = North). Mirrors
//! `render/mapview.py`.
//!
//! The vehicle is drawn twice when the data is there: the green heading
//! arrow is the EKF local solution, the cyan `⊕` the raw GNSS fix projected
//! through GPS_GLOBAL_ORIGIN - watching them converge is the quickest read on
//! an RTK bring-up. Layers, lowest priority first: axes, trail (Braille),
//! KML fence (yellow dots), planned queue path (cyan Braille), then the
//! markers.
//! Under all of it, once [i] fetched one, the satellite snapshot as
//! truecolour half-blocks (two image pixels per cell, glyph cells darkened
//! behind their glyph) - or, on a kitty-graphics terminal, the image itself
//! at full resolution under the cells it fully covers (see crate::kitty).

use std::collections::HashMap;
use std::sync::Arc;

use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};

use super::*;
use crate::geo;
use crate::kitty;
use crate::satellite::SatImage;
use crate::state::{MapView, Target, now};

pub const BRAILLE_BITS: [[u8; 4]; 2] = [[0x01, 0x02, 0x04, 0x40], [0x08, 0x10, 0x20, 0x80]];

pub fn braille(bits: u8) -> char {
    char::from_u32(0x2800 + bits as u32).unwrap_or(' ')
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

/// Background colour of the box (north, east, half-north, half-east) metres.
type BgFn<'a> = &'a dyn Fn(f64, f64, f64, f64) -> Option<(u8, u8, u8)>;

type V3 = [f64; 3];

fn dot3(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Horizontal field of view of the 3D camera.
const FOV_H_DEG: f64 = 60.0;

/// A pinhole camera over the local N/E/Up frame, projecting onto the
/// grid's Braille raster (2 x 4 square dots per cell).
#[derive(Clone, Copy)]
struct Camera {
    pos: V3,
    fwd: V3,
    right: V3,
    up: V3,
    /// Focal length in dots, and the raster half-extents (cols, rows).
    f: f64,
    half: (f64, f64),
    near: f64,
}

impl Camera {
    /// Looking at `target` from `tilt_deg` above the horizon, toward
    /// `az_deg` (from North), from far enough that `range` metres either
    /// side of the target fill the width.
    fn new(target: V3, range: f64, az_deg: f64, tilt_deg: f64, w: usize, h: usize) -> Self {
        let (az, el) = (az_deg.to_radians(), tilt_deg.clamp(5.0, 89.0).to_radians());
        let fwd = [el.cos() * az.cos(), el.cos() * az.sin(), -el.sin()];
        let right = [-az.sin(), az.cos(), 0.0];
        let up = [el.sin() * az.cos(), el.sin() * az.sin(), el.cos()];
        let half = (w as f64, h as f64 * 2.0);
        let tan = (FOV_H_DEG / 2.0).to_radians().tan();
        let d = range / tan;
        let pos = [target[0] - fwd[0] * d, target[1] - fwd[1] * d, target[2] - fwd[2] * d];
        Camera { pos, fwd, right, up, f: half.0 / tan, half, near: d * 0.02 }
    }

    /// Raster dot (col, row) of a point, if it is in front of the camera.
    fn project(&self, p: V3) -> Option<(f64, f64)> {
        let v = [p[0] - self.pos[0], p[1] - self.pos[1], p[2] - self.pos[2]];
        let z = dot3(v, self.fwd);
        (z > self.near).then(|| (self.half.0 + self.f * dot3(v, self.right) / z, self.half.1 - self.f * dot3(v, self.up) / z))
    }

    /// Where the ray through raster dot (col, row) meets the ground (Up =
    /// 0), and roughly how many metres one dot covers there.
    fn ground_at(&self, col: f64, row: f64) -> Option<(f64, f64, f64)> {
        let (x, y) = (col - self.half.0, self.half.1 - row);
        let dir: V3 = std::array::from_fn(|i| self.fwd[i] * self.f + self.right[i] * x + self.up[i] * y);
        let t = -self.pos[2] / dir[2];
        if !(dir[2] < 0.0 && t > 0.0 && t.is_finite()) {
            return None;
        }
        let len = dot3(dir, dir).sqrt();
        // One dot across, stretched along the ground by the grazing angle.
        let across = t * len / self.f;
        let grazing = (-dir[2] / len).max(0.05);
        Some((self.pos[0] + t * dir[0], self.pos[1] + t * dir[1], across / grazing.sqrt()))
    }
}

/// A character grid over the local N/E plane, with Braille sub-cells.
/// Drawn to scale: `range` is the East half-extent, `range_n` the North one.
/// With a `cam` it is a perspective view instead (see [`Camera`]); points
/// without a height are then on the ground.
struct Grid {
    w: usize,
    h: usize,
    cells: Vec<Cell>,
    center: (f64, f64),
    range: f64,
    range_n: f64,
    cam: Option<Camera>,
}

impl Grid {
    fn new(w: usize, h: usize, center: (f64, f64), range: f64) -> Self {
        let range_n = range * n_over_e(w, h);
        Grid { w, h, cells: vec![Cell { ch: ' ', style: Style::new(), layer: Layer::Empty }; w * h], center, range, range_n, cam: None }
    }

    fn with_camera(mut self, cam: Camera) -> Self {
        self.cam = Some(cam);
        self
    }

    /// Raster dot (col, row) of a point `u` metres up, unrounded.
    fn dot3_f(&self, n: f64, e: f64, u: f64) -> Option<(f64, f64)> {
        self.cam.and_then(|c| c.project([n, e, u])).filter(|(x, y)| {
            x.is_finite() && y.is_finite() && *x >= 0.0 && *y >= 0.0 && *x < (self.w * 2) as f64 && *y < (self.h * 4) as f64
        })
    }

    /// Cell of a point `u` metres up (the plan view ignores `u`).
    fn cell_of3(&self, n: f64, e: f64, u: f64) -> Option<(usize, usize)> {
        if self.cam.is_none() {
            return self.cell_of(n, e);
        }
        self.dot3_f(n, e, u).map(|(x, y)| (y as usize / 4, x as usize / 2))
    }

    fn cell_of(&self, n: f64, e: f64) -> Option<(usize, usize)> {
        if self.cam.is_some() {
            return self.cell_of3(n, e, 0.0);
        }
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

    /// A marker `u` metres up (on the ground in the plan view).
    fn marker3(&mut self, n: f64, e: f64, u: f64, ch: char, style: Style) {
        if let Some((r, c)) = self.cell_of3(n, e, u) {
            self.cells[r * self.w + c] = Cell { ch, style, layer: Layer::Marker };
        }
    }

    /// Text starting at cell (r, c), clipped to the grid, over `only_over`.
    fn label(&mut self, r: usize, c: usize, text: &str, style: Style, only_over: &[Layer]) {
        for (i, ch) in text.chars().enumerate() {
            if c + i >= self.w || r >= self.h {
                break;
            }
            let cell = &mut self.cells[r * self.w + c + i];
            if only_over.contains(&cell.layer) {
                *cell = Cell { ch, style, layer: Layer::Marker };
            }
        }
    }

    /// Dot position in the 2x4-per-cell Braille raster.
    fn dot_of(&self, n: f64, e: f64) -> Option<(usize, usize)> {
        if self.cam.is_some() {
            return self.dot3_f(n, e, 0.0).map(|(x, y)| (y as usize, x as usize));
        }
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
        let pts: Vec<V3> = pts.iter().map(|&(n, e)| [n, e, 0.0]).collect();
        self.braille_polyline3(&pts)
    }

    /// Densely sample a polyline of (N, E, Up) points into Braille dots.
    fn braille_polyline3(&self, pts: &[V3]) -> HashMap<(usize, usize), (u8, f64)> {
        // Finer in perspective: the near side is magnified.
        let dot = (self.range / self.w.max(1) as f64 / if self.cam.is_some() { 3.0 } else { 1.0 }).max(0.001);
        let mut out: HashMap<(usize, usize), (u8, f64)> = HashMap::new();
        for seg in pts.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
            let steps = (len / dot).clamp(1.0, 4000.0) as usize;
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let p: V3 = std::array::from_fn(|k| a[k] + (b[k] - a[k]) * t);
                let d = match self.cam {
                    Some(_) => self.dot3_f(p[0], p[1], p[2]).map(|(x, y)| (y as usize, x as usize)),
                    None => self.dot_of(p[0], p[1]),
                };
                if let Some((dr, dc)) = d {
                    let entry = out.entry((dr / 4, dc / 2)).or_insert((0, 0.0));
                    entry.0 |= BRAILLE_BITS[dc % 2][dr % 4];
                }
            }
        }
        out
    }

    fn paint_braille(&mut self, cells: HashMap<(usize, usize), (u8, f64)>, style: impl Fn(f64) -> Style, layer: Layer, only_over: &[Layer]) {
        for ((r, c), (bits, v)) in cells {
            let cell = &mut self.cells[r * self.w + c];
            if only_over.contains(&cell.layer) {
                *cell = Cell { ch: braille(bits), style: style(v), layer };
            }
        }
    }

    /// The block of cells lying entirely inside the snapshot, and the
    /// snapshot pixels behind it, for drawing the image itself under them.
    fn cover(&self, img: &SatImage, origin: (f64, f64)) -> Option<Cover> {
        // A perspective view is no rectangle of the image.
        if self.cam.is_some() {
            return None;
        }
        let (hw, hh) = ((self.w / 2) as f64, (self.h / 2) as f64);
        let (dn, de) = (self.range_n / hh.max(1.0), self.range / hw.max(1.0));
        // Outer edges of the grid (cells are centred on their n/e).
        let (n_top, e_left) = (self.center.0 + (hh + 0.5) * dn, self.center.1 - (hw + 0.5) * de);
        let (n_bot, e_right) = (n_top - self.h as f64 * dn, e_left + self.w as f64 * de);
        let (x0, y0) = img.pixel_of(local_to_global(origin, n_top, 0.0).0, local_to_global(origin, 0.0, e_left).1);
        let (x1, y1) = img.pixel_of(local_to_global(origin, n_bot, 0.0).0, local_to_global(origin, 0.0, e_right).1);
        let (sx, sy) = ((x1 - x0) / self.w as f64, (y1 - y0) / self.h as f64);
        if !(sx > 0.0 && sy > 0.0) {
            return None;
        }
        let (iw, ih) = (img.rgb().width() as f64, img.rgb().height() as f64);
        let span = |p0: f64, step: f64, size: f64, cells: usize| {
            ((-p0 / step).ceil().clamp(0.0, cells as f64) as usize, ((size - p0) / step).floor().clamp(0.0, cells as f64) as usize)
        };
        let (c0, c1) = span(x0, sx, iw, self.w);
        let (r0, r1) = span(y0, sy, ih, self.h);
        (c1 > c0 && r1 > r0).then(|| Cover {
            rows: (r0, r1),
            cols: (c0, c1),
            src: [x0 + c0 as f64 * sx, y0 + r0 as f64 * sy, (c1 - c0) as f64 * sx, (r1 - r0) as f64 * sy],
        })
    }

    /// `bg(n, e, half_n, half_e)` is the background colour of that box;
    /// empty cells inside `cover` keep the default background instead.
    fn lines(&self, bg: Option<BgFn>, cover: Option<&Cover>) -> Vec<Line<'static>> {
        let (hw, hh) = ((self.w / 2) as f64, (self.h / 2) as f64);
        let (dn, de) = (self.range_n / hh.max(1.0), self.range / hw.max(1.0));
        let rgb = |(r, g, b): (u8, u8, u8)| Color::Rgb(r, g, b);
        self.cells
            .chunks(self.w)
            .enumerate()
            .map(|(r, row)| {
                let mut spans = vec![Span::raw("  ")];
                spans.extend(row.iter().enumerate().map(|(c, cell)| {
                    let Some(bg) = bg else { return Span::styled(cell.ch.to_string(), cell.style) };
                    let (top, bot) = match self.cam {
                        Some(cam) => {
                            let at = |row: f64| cam.ground_at((2 * c + 1) as f64, row).and_then(|(n, e, m)| bg(n, e, m, m));
                            (at((4 * r + 1) as f64), at((4 * r + 3) as f64))
                        }
                        None => {
                            let n = self.center.0 + (hh - r as f64) / hh * self.range_n;
                            let e = self.center.1 + (c as f64 - hw) / hw * self.range;
                            (bg(n + dn / 4.0, e, dn / 4.0, de / 2.0), bg(n - dn / 4.0, e, dn / 4.0, de / 2.0))
                        }
                    };
                    let covered = cover.is_some_and(|k| (k.rows.0..k.rows.1).contains(&r) && (k.cols.0..k.cols.1).contains(&c));
                    match (cell.layer, top, bot) {
                        (Layer::Empty, _, _) if covered => Span::styled(" ", Style::new().bg(Color::Reset)),
                        (_, None, None) => Span::styled(cell.ch.to_string(), cell.style),
                        (Layer::Empty, Some(t), Some(b)) => Span::styled("▀", Style::new().fg(rgb(t)).bg(rgb(b))),
                        (Layer::Empty, Some(t), None) => Span::styled("▀", Style::new().fg(rgb(t))),
                        (Layer::Empty, None, Some(b)) => Span::styled("▄", Style::new().fg(rgb(b))),
                        (_, t, b) => {
                            // Darkened so the glyph stays readable on top.
                            let (t, b) = (t.or(b).unwrap(), b.or(t).unwrap());
                            let mix = |x: u8, y: u8| ((x as u16 + y as u16) * 3 / 10) as u8;
                            let style = cell.style.bg(Color::Rgb(mix(t.0, b.0), mix(t.1, b.1), mix(t.2, b.2)));
                            Span::styled(cell.ch.to_string(), if cell.style.fg.is_none() { style.fg(TEXT) } else { style })
                        }
                    }
                }));
                Line::from(spans)
            })
            .collect()
    }
}

/// Local N/E metres around `origin` -> lat/lon (inverse of
/// geo::global_to_local).
fn local_to_global((olat, olon): (f64, f64), n: f64, e: f64) -> (f64, f64) {
    let r = geo::EARTH_RADIUS_M;
    let cos0 = olat.to_radians().cos().max(0.05);
    (olat + (n / r).to_degrees(), olon + (e / (r * cos0)).to_degrees())
}

/// Satellite background in a local N/E frame around `origin`.
fn sat_background(img: &SatImage, origin: (f64, f64)) -> impl Fn(f64, f64, f64, f64) -> Option<(u8, u8, u8)> + '_ {
    move |n: f64, e: f64, half_n: f64, half_e: f64| {
        let (lat, lon) = local_to_global(origin, n, e);
        img.sample(lat, lon, half_n, half_e)
    }
}

/// Grid cells (rows r0..r1, cols c0..c1) drawn with the image itself, and
/// the snapshot pixel rectangle [x, y, w, h] stretched over them.
pub struct Cover {
    rows: (usize, usize),
    cols: (usize, usize),
    src: [f64; 4],
}

/// The satellite image under a map grid whose top-left cell is at content
/// `line`, column `col` of a panel.
pub struct MapImage {
    pub line: usize,
    pub col: usize,
    cover: Cover,
    image: Arc<SatImage>,
}

impl MapImage {
    /// The kitty placement for a panel whose content starts at screen
    /// (`x`, `y`), scrolled by `scroll` lines and `height` lines tall.
    pub fn placement(&self, x: u16, y: u16, scroll: usize, height: usize) -> Option<kitty::Placement> {
        let Cover { rows: (r0, r1), cols: (c0, c1), src: [sx, sy, sw, sh] } = self.cover;
        let per_row = sh / (r1 - r0) as f64;
        // Content lines of the covered rows, clipped to the visible window.
        let (top, bot) = ((self.line + r0).max(scroll), (self.line + r1).min(scroll + height));
        if bot <= top {
            return None;
        }
        let py = sy + (top - self.line - r0) as f64 * per_row;
        let ph = (bot - top) as f64 * per_row;
        let (x0, y0) = (sx.round() as u32, py.round() as u32);
        let src = [x0, y0, ((sx + sw).round() as u32).saturating_sub(x0).max(1), ((py + ph).round() as u32).saturating_sub(y0).max(1)];
        Some(kitty::Placement {
            image: self.image.clone(),
            col: x + (self.col + c0) as u16,
            row: y + (top - scroll) as u16,
            cols: (c1 - c0) as u16,
            rows: (bot - top) as u16,
            src,
        })
    }
}

/// One vehicle on the fleet map.
pub struct FleetPin {
    pub lat: f64,
    pub lon: f64,
    pub yaw: f64,
    /// Drawn one cell East of the heading arrow.
    pub label: char,
    pub style: Style,
    pub home: Option<(f64, f64)>,
}

/// The fleet view's map: every vehicle (arrow + label) and its home `H`,
/// plus the start-up KML overlay (fence dots, numbered waypoints), framed
/// together and divided by `zoom`, over the satellite snapshot if there is
/// one. Returns the rows (`w` x `h` cells after the 2-column margin), the
/// East half-extent in metres and, on a kitty-graphics terminal, the image
/// to place under them.
pub fn fleet_map(
    w: usize,
    h: usize,
    pins: &[FleetPin],
    kml: Option<&geo::Kml>,
    zoom: f64,
    sat: Option<&Arc<SatImage>>,
) -> (Vec<Line<'static>>, f64, Option<MapImage>) {
    let (w, h) = (w.max(4), h.max(2));
    let points: Vec<(f64, f64)> = pins.iter().flat_map(|p| std::iter::once((p.lat, p.lon)).chain(p.home)).collect();
    let Some(&origin) = points.first() else { return (Vec::new(), 0.0, None) };
    let mut local: Vec<(f64, f64)> = points.iter().filter_map(|&(la, lo)| geo::to_local(la, lo, origin)).collect();

    // KML geometry far from the vehicles (multi-site exports) is not framed.
    let to_local = |&(la, lo): &(f64, f64)| geo::to_local(la, lo, origin);
    let near = |p: &(f64, f64)| local.iter().any(|v| (p.0 - v.0).hypot(p.1 - v.1) <= MAP_RANGE_MAX_M);
    let waypoints: Vec<(f64, f64)> =
        kml.map(|k| k.waypoints.iter().filter_map(|w| to_local(&(w.lat, w.lon))).collect()).unwrap_or_default();
    let rings: Vec<Vec<(f64, f64)>> = kml
        .map(|k| k.fence_rings.iter().map(|r| r.iter().filter_map(to_local).collect::<Vec<_>>()).filter(|r| r.len() >= 3).collect())
        .unwrap_or_default();
    let geometry: Vec<(f64, f64)> = waypoints.iter().chain(rings.iter().flatten()).copied().filter(near).collect();
    local.extend(geometry);
    let (min_n, max_n) = local.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.0), b.max(p.0)));
    let (min_e, max_e) = local.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.1), b.max(p.1)));
    let center = ((min_n + max_n) / 2.0, (min_e + max_e) / 2.0);
    let fit = ((max_e - min_e) / 2.0).max((max_n - min_n) / 2.0 / n_over_e(w, h)) * 1.25;
    let range = (fit.max(10.0) / zoom).clamp(1.0, MAP_RANGE_MAX_M);

    let mut grid = Grid::new(w, h, center, range);
    let east_cell = range / (w / 2) as f64;
    let cell_size = (range / ((w / 2).max(h / 2)) as f64).max(0.01);
    for ring in &rings {
        for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)).take(ring.len()) {
            let steps = ((b.0 - a.0).hypot(b.1 - a.1) / cell_size).clamp(1.0, 400.0) as usize;
            for i in 0..=steps {
                let f = i as f64 / steps as f64;
                grid.put(a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f, '.', Style::new().fg(YELLOW), Layer::Fence, &[Layer::Empty]);
            }
        }
    }
    for (i, &(n, e)) in waypoints.iter().enumerate() {
        let digit = char::from_digit(((i + 1) % 10) as u32, 10).unwrap();
        grid.marker(n, e, digit, Style::new().fg(MAUVE).add_modifier(Modifier::BOLD));
    }
    for p in pins {
        if let Some((n, e)) = p.home.and_then(|(la, lo)| geo::to_local(la, lo, origin)) {
            grid.marker(n, e, 'H', Style::new().fg(YELLOW));
        }
    }
    for p in pins {
        if let Some((n, e)) = geo::to_local(p.lat, p.lon, origin) {
            grid.marker(n, e, heading_arrow(p.yaw), p.style.add_modifier(Modifier::BOLD));
            grid.marker(n, e + east_cell, p.label, p.style);
        }
    }
    let bg = sat.map(|img| sat_background(img, origin));
    let image = sat.filter(|_| kitty::supported()).and_then(|img| {
        grid.cover(img, origin).map(|cover| MapImage { line: 0, col: 2, cover, image: img.clone() })
    });
    (grid.lines(bg.as_ref().map(|f| f as BgFn), image.as_ref().map(|m| &m.cover)), range, image)
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
    "[g] goto", "[W] wp queue", "[C] coverage", "[x] jog", "[o] load kml",
    "[O] upload fence", "[+]/[-] zoom", "[arrows] pan", "[u] follow", "[0] reset", "[f] fit kml", "[t] trail", "[c] clear",
    "[v] plan/3D/chase", "[V] tilt", "[<]/[>] rotate", "[i] sat", "[I] sat on/off", "[n] back", "ESC panels",
];

/// 1-2-5 spacing for the 3D ground grid: about two lines per half-range.
fn grid_step(range: f64) -> f64 {
    let raw = (range / 2.0).max(0.5);
    let mag = 10f64.powf(raw.log10().floor());
    [1.0, 2.0, 5.0, 10.0].into_iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag)
}

fn compass_point(az: f64) -> &'static str {
    ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][((az.rem_euclid(360.0) / 45.0 + 0.5) as usize) % 8]
}

/// The 3D views' depth reference: a dim ground grid around the view
/// centre, the origin's N/E axes brighter, and an `N` at the North end.
fn ground_grid(grid: &mut Grid, center: (f64, f64), range: f64) {
    let step = grid_step(range);
    let ext = range * 3.0;
    let (n0, n1) = (((center.0 - ext) / step).floor() as i64, ((center.0 + ext) / step).ceil() as i64);
    let (e0, e1) = (((center.1 - ext) / step).floor() as i64, ((center.1 + ext) / step).ceil() as i64);
    let (nlo, nhi, elo, ehi) = (n0 as f64 * step, n1 as f64 * step, e0 as f64 * step, e1 as f64 * step);
    let dim = Style::new().fg(OVERLAY0).add_modifier(Modifier::DIM);
    let mut lines: Vec<[V3; 2]> = Vec::new();
    for i in n0..=n1 {
        let n = i as f64 * step;
        lines.push([[n, elo, 0.0], [n, ehi, 0.0]]);
    }
    for i in e0..=e1 {
        let e = i as f64 * step;
        lines.push([[nlo, e, 0.0], [nhi, e, 0.0]]);
    }
    for l in &lines {
        let cells = grid.braille_polyline3(l);
        grid.paint_braille(cells, |_| dim, Layer::Axis, &[Layer::Empty]);
    }
    // The origin axes, where they cross the grid.
    for l in [[[0.0, elo, 0.0], [0.0, ehi, 0.0]], [[nlo, 0.0, 0.0], [nhi, 0.0, 0.0]]] {
        if (nlo..=nhi).contains(&l[0][0]) || (elo..=ehi).contains(&l[0][1]) {
            let cells = grid.braille_polyline3(&l);
            grid.paint_braille(cells, |_| Style::new(), Layer::Axis, &[Layer::Empty, Layer::Axis]);
        }
    }
    grid.marker3(nhi, center.1, 0.0, 'N', Style::new().fg(RED).add_modifier(Modifier::BOLD));
}

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


    let mut lines: Vec<Line<'static>> = Vec::new();

    let arm = if st.armed { Ln::new().fg("ARMED", GREEN) } else { Ln::new().dim("DISARMED") };
    lines.push(Ln::new().raw(" MODE: ").bold(st.mode.clone()).raw("   ").spans(arm.0).line());

    if session.jog_armed {
        lines.push(Line::from(Span::styled(
            format!(
                " JOG ARMED   step {:.2} m / {JOG_YAW_STEP_DEG:.0}°   w/s up·down  k/j fwd·back  a/d strafe  h/l yaw  [/] step  x off ",
                session.jog_step
            ),
            Style::new().bg(RED).fg(CRUST).add_modifier(Modifier::BOLD),
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
            let shown = match (&st.map_satellite, st.map_satellite_hidden) {
                (Some(_), false) => Ln::new().dim("   drawn under the grid · [I] hide"),
                (Some(_), true) => Ln::new().dim("   hidden · [I] show"),
                _ => Ln::new(),
            };
            vec![Ln::new().fg(format!(" Satellite image saved: {}", st.map_download_path), GREEN).spans(shown.0).line()]
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

    // Height above the local origin (the 3D views' ground plane).
    let alt = if lz.is_finite() { -lz } else { 0.0 };
    let view = st.map_view;
    let az = view.azimuth(st.map_az, st.yaw);
    let following = st.map_follow && !st.map_fit_kml;
    let mut grid = Grid::new(grid_w, grid_h, center, range);
    if view != MapView::Plan {
        // Following: orbit the vehicle itself; else a point on the ground.
        let target_u = if following { alt.max(0.0) } else { 0.0 };
        grid = grid.with_camera(Camera::new([center.0, center.1, target_u], range, az, st.map_tilt, grid_w, grid_h));
    }
    let (hw, hh) = ((grid_w / 2) as f64, (grid_h / 2) as f64);
    let range_n = grid.range_n;
    let dim = Style::new().add_modifier(Modifier::DIM);

    if view == MapView::Plan {
        // Axes through the local origin.
        for c in 0..grid_w {
            grid.put(0.0, center.1 + (c as f64 - hw) / hw * range, '─', Style::new(), Layer::Axis, &[Layer::Empty]);
        }
        for r in 0..grid_h {
            grid.put(center.0 + (hh - r as f64) / hh * range_n, 0.0, '│', Style::new(), Layer::Axis, &[Layer::Empty, Layer::Axis]);
        }
        grid.put(0.0, 0.0, '┼', Style::new(), Layer::Axis, &[]);
    } else {
        ground_grid(&mut grid, center, range);
    }

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


    for (i, w) in waypoints_local.iter().enumerate() {
        if let Some((n, e)) = w {
            let digit = char::from_digit(((i + 1) % 10) as u32, 10).unwrap();
            grid.marker(*n, *e, digit, bold(MAUVE));
        }
    }
    if let Some((n, e)) = queue_target_local {
        grid.marker(n, e, '◎', bold(TEXT));
    }
    match home_local {
        Some((n, e)) => grid.marker(n, e, 'H', Style::new().fg(YELLOW)),
        // No HOME_POSITION yet: the origin usually coincides with home.
        None if origin.is_some() => grid.marker(0.0, 0.0, 'H', dim),
        None => {}
    }
    if view == MapView::Plan {
        if let Some((x, y)) = target {
            grid.marker(x, y, '*', Style::new());
        }
        if let Some((n, e)) = gps_local {
            grid.marker(n, e, '⊕', Style::new().fg(CYAN));
        }
        grid.marker(lx, ly, heading_arrow(st.yaw), bold(GREEN));
    } else {
        // The vehicle up in the air, a stalk down to its shadow on the
        // ground: the shadow is what lines up with the KML / path / home.
        let stalk = grid.braille_polyline3(&[[lx, ly, 0.0], [lx, ly, alt]]);
        grid.paint_braille(stalk, |_| Style::new().fg(GREEN).add_modifier(Modifier::DIM), Layer::Path, &[
            Layer::Empty,
            Layer::Axis,
            Layer::Trail,
            Layer::Fence,
        ]);
        grid.marker3(lx, ly, 0.0, '×', Style::new().fg(GREEN));
        if let Some((x, y)) = target {
            let tu = if st.pos_target_z.is_finite() { -st.pos_target_z } else { alt };
            grid.marker3(x, y, tu, '*', Style::new());
        }
        if let Some((n, e)) = gps_local {
            grid.marker3(n, e, alt, '⊕', Style::new().fg(CYAN));
        }
        // Heading arrow as the heading looks on screen.
        let yaw = st.yaw.to_radians();
        let reach = range * 0.1;
        let arrow = match (grid.dot3_f(lx, ly, alt), grid.cam.and_then(|c| c.project([lx + reach * yaw.cos(), ly + reach * yaw.sin(), alt]))) {
            (Some(a), Some(b)) if st.yaw.is_finite() => heading_arrow((b.0 - a.0).atan2(a.1 - b.1).to_degrees()),
            _ => heading_arrow(st.yaw),
        };
        grid.marker3(lx, ly, alt, arrow, bold(GREEN));
        if let Some((r, c)) = grid.cell_of3(lx, ly, alt) {
            grid.label(r, c + 1, &format!(" {alt:.1}m"), Style::new().fg(GREEN), &[Layer::Empty, Layer::Axis, Layer::Trail, Layer::Path]);
        }
    }

    let mut header = if view == MapView::Plan {
        Ln::new()
            .raw(format!(" range E +/-{range:.1} m  N +/-{range_n:.1} m      "))
            .dim(format!("cell ~ {:4.1} m x {:4.1} m (to scale)", range / hw, range_n / hh))
    } else {
        let name = if view == MapView::Chase { "3D chase" } else { "3D" };
        Ln::new()
            .fg(format!(" {name}"), GREEN)
            .raw(format!("  tilt {:.0}°  looking {} ({az:03.0}°)   ±{range:.0} m   ", st.map_tilt, compass_point(az)))
            .dim({
                let step = grid_step(range);
                if step >= 1.0 { format!("grid {step:.0} m") } else { format!("grid {step:.1} m") }
            })
    };
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
    let sat = st.map_satellite.as_ref().filter(|_| !st.map_satellite_hidden).zip(origin);
    let sat_bg = sat.map(|(img, origin)| sat_background(img, origin));
    let image = sat.filter(|_| kitty::supported()).and_then(|(img, origin)| {
        grid.cover(img, origin).map(|cover| MapImage { line: lines.len(), col: 2, cover, image: img.clone() })
    });
    lines.extend(grid.lines(sat_bg.as_ref().map(|f| f as BgFn), image.as_ref().map(|m| &m.cover)));
    *ctx.map_image.borrow_mut() = image;
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
                    .st("#", bold(MAUVE))
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
                    .st(((i + 1) % 10).to_string(), bold(MAUVE))
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
                    .st("◎", bold(TEXT))
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


    let total = st.mission_upload_items.len();
    if st.mission_upload_active {
        lines.push(
            Ln::new()
                .raw(" MISSION   ")
                .fg("uploading", YELLOW)
                .raw(format!("   item {}/{total}", st.mission_upload_acked_seq + 1))
                .line(),
        );
    } else if st.mission_upload_status == "COMPLETE" {
        lines.push(Ln::new().raw(" MISSION   ").fg("ACCEPTED by PX4", GREEN).raw(format!("   {total} item(s)")).line());
    } else if st.mission_upload_status == "ERROR" {
        lines.push(Ln::new().raw(" MISSION   ").fg(format!("failed: {}", st.mission_upload_error), RED).line());
    } else if let Some(plan) = &st.plan {
        lines.push(Ln::new().raw(" MISSION   ").dim(format!("{} item(s) loaded from .plan - [M] upload", plan.items.len())).line());
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fleet_map_draws_every_vehicle_and_home() {
        let pin = |lat: f64, label, home| FleetPin { lat, lon: 11.0, yaw: 0.0, label, style: Style::new(), home };
        // ~110 m apart North-South, the second with its home beside it.
        let pins = [pin(46.0, '1', None), pin(46.001, '2', Some((46.0005, 11.0)))];
        let (lines, range, _) = fleet_map(60, 20, &pins, None, 1.0, None);
        let text: String = lines.iter().flat_map(|l| l.spans.iter().map(|s| s.content.to_string())).collect();
        assert_eq!(lines.len(), 20);
        assert_eq!(text.matches('↑').count(), 2);
        assert!(text.contains('1') && text.contains('2') && text.contains('H'));
        assert!(range > 10.0);
    }

    #[test]
    fn fleet_map_draws_the_kml_overlay() {
        let pins = [FleetPin { lat: 46.0, lon: 11.0, yaw: 0.0, label: 'a', style: Style::new(), home: None }];
        let wp = |lat: f64, lon: f64| geo::Waypoint { lat, lon, alt: 0.0, name: String::new() };
        let kml = geo::Kml {
            // ~200 m North of the vehicle: framed. ~50 km away: ignored.
            waypoints: vec![wp(46.0018, 11.0), wp(46.45, 11.0)],
            fence_rings: vec![vec![(45.9995, 10.9995), (45.9995, 11.0005), (46.0005, 11.0005), (46.0005, 10.9995)]],
            ground_alt: None,
        };
        let (lines, range, _) = fleet_map(60, 20, &pins, Some(&kml), 1.0, None);
        let text: String = lines.iter().flat_map(|l| l.spans.iter().map(|s| s.content.to_string())).collect();
        assert!(text.contains('1') && text.contains('.'));
        assert!(range > 100.0 && range < 1000.0);
    }

    /// A 20 m square fence around the origin, home at the origin and the
    /// vehicle 10 m up over its corner, seen tilted from the South.
    fn tilted_scene(az: f64, tilt: f64) -> Grid {
        let (w, h) = (61, 21);
        let mut grid = Grid::new(w, h, (0.0, 0.0), 30.0).with_camera(Camera::new([0.0, 0.0, 0.0], 30.0, az, tilt, w, h));
        ground_grid(&mut grid, (0.0, 0.0), 30.0);
        let ring = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0), (-10.0, -10.0)];
        let cells = grid.braille_polyline(&ring);
        grid.paint_braille(cells, |_| Style::new().fg(YELLOW), Layer::Fence, &[Layer::Empty, Layer::Axis]);
        grid.marker(0.0, 0.0, 'H', Style::new());
        let stalk = grid.braille_polyline3(&[[10.0, 10.0, 0.0], [10.0, 10.0, 10.0]]);
        grid.paint_braille(stalk, |_| Style::new(), Layer::Path, &[Layer::Empty, Layer::Axis, Layer::Fence]);
        grid.marker3(10.0, 10.0, 10.0, 'V', Style::new());
        grid
    }

    #[test]
    fn tilted_view_puts_the_near_side_low_and_height_up() {
        let grid = tilted_scene(0.0, 45.0);
        // Looking North: the home sits mid-screen, a point South of it is
        // lower, North higher; the vehicle is above its ground shadow.
        let (hr, hc) = grid.cell_of(0.0, 0.0).unwrap();
        assert!((hr as i64 - 10).abs() <= 1 && (hc as i64 - 30).abs() <= 1);
        assert!(grid.cell_of(-10.0, 0.0).unwrap().0 > hr);
        assert!(grid.cell_of(10.0, 0.0).unwrap().0 < hr);
        let (gr, gc) = grid.cell_of(10.0, 10.0).unwrap();
        let (vr, vc) = grid.cell_of3(10.0, 10.0, 10.0).unwrap();
        assert!(vr < gr && vc.abs_diff(gc) <= 1);
        // Perspective: the far edge of the fence is narrower than the near.
        let width = |n| grid.cell_of(n, 10.0).unwrap().1 - grid.cell_of(n, -10.0).unwrap().1;
        assert!(width(10.0) < width(-10.0));
        // Behind the camera is not drawn.
        assert!(grid.cell_of(-500.0, 0.0).is_none());
    }

    #[test]
    fn tilted_view_turns_with_the_azimuth() {
        // Looking East, North is on the left.
        let grid = tilted_scene(90.0, 45.0);
        let (_, hc) = grid.cell_of(0.0, 0.0).unwrap();
        assert!(grid.cell_of(10.0, 0.0).unwrap().1 < hc);
    }

    #[test]
    fn camera_ray_hits_the_ground_where_it_projects() {
        let cam = Camera::new([5.0, -3.0, 0.0], 30.0, 30.0, 45.0, 61, 21);
        let (x, y) = cam.project([12.0, 4.0, 0.0]).unwrap();
        let (n, e, m) = cam.ground_at(x, y).unwrap();
        assert!((n - 12.0).abs() < 1e-6 && (e - 4.0).abs() < 1e-6 && m > 0.0);
        // Straight ahead at the horizon and above it there is no ground.
        assert!(cam.ground_at(61.0, -100.0).is_none());
    }

    #[test]
    fn kitty_cover_is_the_cells_inside_the_snapshot() {
        let origin = (46.0, 11.0);
        let (cx, cy) = crate::satellite::web_mercator(origin.0, origin.1);
        let img = SatImage::new(image::RgbImage::new(100, 100), [cx - 500.0, cy - 500.0, cx + 500.0, cy + 500.0]);

        // Well inside: every cell, centred on the image.
        let k = Grid::new(21, 11, (0.0, 0.0), 100.0).cover(&img, origin).unwrap();
        assert_eq!((k.rows, k.cols), ((0, 11), (0, 21)));
        let [x, y, w, h] = k.src;
        assert!((x + w / 2.0 - 50.0).abs() < 0.5 && (y + h / 2.0 - 50.0).abs() < 0.5);

        // Wider than the image: only the middle cells, all within it.
        let k = Grid::new(21, 11, (0.0, 0.0), 2000.0).cover(&img, origin).unwrap();
        assert!(k.cols.0 > 0 && k.cols.1 < 21 && k.rows.0 > 0 && k.rows.1 < 11);
        let [x, y, w, h] = k.src;
        assert!(x >= 0.0 && y >= 0.0 && x + w <= 100.0 + 1e-9 && y + h <= 100.0 + 1e-9);

        // Entirely off it.
        assert!(Grid::new(21, 11, (5000.0, 0.0), 100.0).cover(&img, origin).is_none());
    }
}
