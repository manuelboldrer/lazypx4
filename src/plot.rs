//! Drawing a flight-review plot (ported from lazyjagg's raster): range
//! finding, min/max decimation (a million samples cost one column per
//! pixel), lines, steps, flag lanes and ticks on a small pixel buffer that
//! is then shown as braille cells.

use std::sync::Arc;

use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::Color;

use crate::review::{Mode, Plot, Store};
use crate::ulog::Series;

pub const GRID: u32 = 0x363a4f;

pub fn rgb(c: u32) -> Color {
    Color::Rgb((c >> 16) as u8, (c >> 8) as u8, c as u8)
}

pub struct Raster {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u32>,
}

impl Raster {
    pub fn new(w: usize, h: usize) -> Raster {
        Raster { w, h, px: vec![0; w * h] }
    }

    #[inline]
    fn set(&mut self, x: i64, y: i64, c: u32) {
        if x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h {
            self.px[y as usize * self.w + x as usize] = c;
        }
    }

    fn vline(&mut self, x: i64, y0: i64, y1: i64, c: u32) {
        if x < 0 || x >= self.w as i64 || self.h == 0 {
            return;
        }
        let (a, b) = (y0.min(y1).max(0), y0.max(y1).min(self.h as i64 - 1));
        for y in a..=b {
            self.px[y as usize * self.w + x as usize] = c;
        }
    }

    fn line(&mut self, x0: i64, y0: i64, x1: i64, y1: i64, c: u32) {
        // Far off-screen coordinates (deep zoom) would loop for ages.
        const LIM: i64 = 1 << 20;
        if x0.abs().max(x1.abs()).max(y0.abs()).max(y1.abs()) > LIM {
            let Some((a, b)) = clip(x0 as f64, y0 as f64, x1 as f64, y1 as f64, self.w as f64, self.h as f64) else { return };
            return self.line(a.0 as i64, a.1 as i64, b.0 as i64, b.1 as i64, c);
        }
        let (dx, dy) = ((x1 - x0).abs(), -(y1 - y0).abs());
        let (sx, sy) = (if x0 < x1 { 1 } else { -1 }, if y0 < y1 { 1 } else { -1 });
        let (mut x, mut y, mut err) = (x0, y0, dx + dy);
        loop {
            self.set(x, y, c);
            if x == x1 && y == y1 {
                break;
            }
            if (sx > 0 && x >= self.w as i64) || (sx < 0 && x < 0) || (sy > 0 && dx == 0 && y >= self.h as i64) {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }
}

/// Liang-Barsky segment clip to [0,w]x[0,h].
fn clip(x0: f64, y0: f64, x1: f64, y1: f64, w: f64, h: f64) -> Option<((f64, f64), (f64, f64))> {
    let (dx, dy) = (x1 - x0, y1 - y0);
    let (mut a, mut b) = (0.0f64, 1.0f64);
    for (p, q) in [(-dx, x0), (dx, w - x0), (-dy, y0), (dy, h - y0)] {
        if p == 0.0 {
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                a = a.max(r);
            } else {
                b = b.min(r);
            }
            if a > b {
                return None;
            }
        }
    }
    Some(((x0 + a * dx, y0 + a * dy), (x0 + b * dx, y0 + b * dy)))
}

/// A curve ready to draw.
pub enum Trace {
    Time(Arc<Series>),
    Points(Vec<f64>, Vec<f64>),
}

pub struct Prepared {
    pub trace: Trace,
    pub color: u32,
}

/// Data ranges a plot shows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub x0: f64,
    pub x1: f64,
    pub y0: f64,
    pub y1: f64,
}

impl Bounds {
    #[inline]
    fn px(&self, x: f64, w: usize) -> f64 {
        (x - self.x0) / (self.x1 - self.x0) * (w as f64 - 1.0)
    }

    #[inline]
    fn py(&self, y: f64, h: usize) -> f64 {
        (1.0 - (y - self.y0) / (self.y1 - self.y0)) * (h as f64 - 1.0)
    }

    /// Where `x` falls across the plot, 0 = left, 1 = right.
    pub fn x_frac(&self, x: f64) -> f64 {
        self.px(x, 2)
    }

    /// Where `y` falls down the plot, 0 = top, 1 = bottom.
    pub fn y_frac(&self, y: f64) -> f64 {
        self.py(y, 2)
    }
}

fn pad(lo: f64, hi: f64) -> (f64, f64) {
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return (0.0, 1.0);
    }
    if hi - lo < 1e-300 || (hi - lo) < lo.abs().max(hi.abs()) * 1e-12 {
        let d = if lo == 0.0 { 1.0 } else { lo.abs() * 0.1 };
        return (lo - d, hi + d);
    }
    let m = (hi - lo) * 0.06;
    (lo - m, hi + m)
}

fn extend(range: &mut (f64, f64), v: f64) {
    if v.is_finite() {
        range.0 = range.0.min(v);
        range.1 = range.1.max(v);
    }
}

/// The curves of `plot` for the time window [t0, t1].
pub fn prepare(plot: &Plot, store: &Store, t0: f64, t1: f64) -> Vec<Prepared> {
    let mut out = Vec::new();
    for c in &plot.curves {
        let Some(y) = store.get(&c.name).cloned() else { continue };
        let trace = match plot.mode {
            Mode::Time | Mode::Flags => Trace::Time(y),
            Mode::Fft => {
                let (f, m) = spectrum(&y, t0, t1, 1 << 16);
                Trace::Points(f.get(1..).unwrap_or(&[]).to_vec(), m.get(1..).unwrap_or(&[]).to_vec())
            }
            Mode::Xy => {
                let Some(x) = c.x.as_ref().and_then(|n| store.get(n)).cloned() else { continue };
                let (a, b) = if store.aux.contains(&y.name) { (0, y.len()) } else { y.visible(t0, t1) };
                let stride = ((b - a) / 50_000).max(1);
                let (mut xs, mut ys) = (Vec::new(), Vec::new());
                let mut i = a;
                while i < b {
                    if let Some(v) = x.value_at(y.t[i]) {
                        xs.push(v);
                        ys.push(y.v[i]);
                    }
                    i += stride;
                }
                Trace::Points(xs, ys)
            }
        };
        out.push(Prepared { trace, color: c.color });
    }
    out
}

/// The ranges a plot shows for time window [t0, t1]; `aspect` is the
/// plot's pixel width / height, for equal-axis XY plots.
pub fn bounds_for(plot: &Plot, traces: &[Prepared], t0: f64, t1: f64, aspect: f64) -> Bounds {
    let mut yr = (f64::INFINITY, f64::NEG_INFINITY);
    let mut xr = (f64::INFINITY, f64::NEG_INFINITY);
    for p in traces {
        match &p.trace {
            Trace::Time(s) => {
                let (a, b) = s.visible(t0, t1);
                for &v in &s.v[a..b] {
                    extend(&mut yr, v);
                }
            }
            Trace::Points(x, y) => {
                for &v in x {
                    extend(&mut xr, v);
                }
                for &v in y {
                    extend(&mut yr, v);
                }
            }
        }
    }
    let (mut y0, mut y1) = pad(yr.0, yr.1);
    let (mut x0, mut x1) = match plot.mode {
        Mode::Time | Mode::Flags => (t0, t1),
        _ => pad(xr.0, xr.1),
    };
    if plot.equal && plot.mode == Mode::Xy && aspect > 0.0 {
        let per_px = ((x1 - x0) / aspect).max(y1 - y0);
        let (cx, cy) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
        (x0, x1) = (cx - per_px * aspect / 2.0, cx + per_px * aspect / 2.0);
        (y0, y1) = (cy - per_px / 2.0, cy + per_px / 2.0);
    }
    Bounds { x0, x1, y0, y1 }
}

fn draw_grid(r: &mut Raster, f: &Bounds, xs: &[f64], ys: &[f64]) {
    let (w, h) = (r.w, r.h);
    for &t in xs {
        let x = f.px(t, w).round() as i64;
        for y in (0..h as i64).step_by((h / 24).max(4)) {
            r.set(x, y, GRID);
        }
    }
    for &t in ys {
        let y = f.py(t, h).round() as i64;
        for x in (0..w as i64).step_by((w / 48).max(4)) {
            r.set(x, y, GRID);
        }
    }
}

/// A time series with one screen column per pixel at most.
fn draw_time(r: &mut Raster, s: &Series, f: &Bounds, steps: bool, color: u32) {
    let (a, b) = s.visible(f.x0, f.x1);
    if a >= b {
        return;
    }
    let (w, h) = (r.w, r.h);
    let mut last: Option<(i64, i64)> = None;
    let mut col: Option<(i64, i64, i64, i64, i64)> = None; // x, first, min, max, last

    let flush = |r: &mut Raster, col: &(i64, i64, i64, i64, i64), last: &mut Option<(i64, i64)>| {
        let &(x, first, lo, hi, end) = col;
        if let Some((px, py)) = *last {
            if steps {
                r.line(px, py, x, py, color);
                r.line(x, py, x, first, color);
            } else {
                r.line(px, py, x, first, color);
            }
        } else {
            r.set(x, first, color);
        }
        if lo != hi {
            r.vline(x, lo, hi, color);
        }
        *last = Some((x, end));
    };

    for i in a..b {
        let v = s.v[i];
        if !v.is_finite() {
            if let Some(c) = col.take() {
                flush(r, &c, &mut last);
            }
            last = None; // a gap
            continue;
        }
        let x = f.px(s.t[i], w).round() as i64;
        let y = f.py(v, h).round() as i64;
        match &mut col {
            Some(c) if c.0 == x => {
                c.2 = c.2.min(y);
                c.3 = c.3.max(y);
                c.4 = y;
            }
            _ => {
                if let Some(c) = col.take() {
                    flush(r, &c, &mut last);
                }
                col = Some((x, y, y, y, y));
            }
        }
    }
    if let Some(c) = col.take() {
        flush(r, &c, &mut last);
    }
}

fn draw_points(r: &mut Raster, x: &[f64], y: &[f64], f: &Bounds, color: u32) {
    let (w, h) = (r.w, r.h);
    let mut last = None;
    for (&a, &b) in x.iter().zip(y) {
        if !a.is_finite() || !b.is_finite() {
            last = None;
            continue;
        }
        let (px, py) = (f.px(a, w).round() as i64, f.py(b, h).round() as i64);
        match last {
            Some((lx, ly)) => r.line(lx, ly, px, py, color),
            None => r.set(px, py, color),
        }
        last = Some((px, py));
    }
}

/// One band per series, filled while it is non-zero.
fn draw_lanes(r: &mut Raster, traces: &[Prepared], f: &Bounds) {
    let n = traces.len().max(1);
    let (w, h) = (r.w, r.h);
    for (lane, p) in traces.iter().enumerate() {
        let Trace::Time(s) = &p.trace else { continue };
        let top = lane * h / n;
        // A pixel row of air between lanes when there is room.
        let bottom = ((lane + 1) * h / n).saturating_sub(if h / n > 3 { 2 } else { 1 }).max(top);
        let (a, b) = s.visible(f.x0, f.x1);
        for i in a..b {
            if s.v[i] == 0.0 || s.v[i].is_nan() {
                continue;
            }
            let end = if i + 1 < s.len() { s.t[i + 1] } else { s.t[i] };
            let (x0, x1) = (f.px(s.t[i].max(f.x0), w) as i64, f.px(end.min(f.x1), w) as i64);
            for x in x0.max(0)..=x1.max(x0).min(w as i64 - 1) {
                r.vline(x, top as i64, bottom as i64, p.color);
            }
        }
    }
}

/// Grid at the given ticks, and every curve.
pub fn render(plot: &Plot, traces: &[Prepared], f: &Bounds, r: &mut Raster, xs: &[f64], ys: &[f64]) {
    if plot.mode == Mode::Flags {
        draw_grid(r, f, xs, &[]);
        draw_lanes(r, traces, f);
        return;
    }
    draw_grid(r, f, xs, ys);
    for p in traces {
        match &p.trace {
            Trace::Time(s) => draw_time(r, s, f, plot.steps, p.color),
            Trace::Points(x, y) => draw_points(r, x, y, f, p.color),
        }
    }
}

/// 1, 2 or 5 times a power of ten, at or above `raw`.
fn nice_step(raw: f64) -> f64 {
    let mag = 10f64.powf(raw.log10().floor());
    let frac = raw / mag;
    mag * if frac <= 1.0 {
        1.0
    } else if frac <= 2.0 {
        2.0
    } else if frac <= 5.0 {
        5.0
    } else {
        10.0
    }
}

/// Round tick positions in [lo, hi].
pub fn ticks(lo: f64, hi: f64, n: usize) -> Vec<f64> {
    let span = hi - lo;
    if !(span.is_finite() && span > 0.0) {
        return Vec::new();
    }
    let step = nice_step(span / n.max(1) as f64);
    let mut out = Vec::new();
    let mut k = (lo / step).ceil();
    while k * step <= hi && out.len() < 64 {
        out.push(k * step);
        k += 1.0;
    }
    out
}

/// Ticks that are round once `offset` is subtracted (relative time).
pub fn ticks_offset(lo: f64, hi: f64, n: usize, offset: f64) -> Vec<f64> {
    ticks(lo - offset, hi - offset, n).into_iter().map(|t| t + offset).collect()
}

/// A tick label: decimals enough to tell neighbours apart.
pub fn tick_label(v: f64, step: f64) -> String {
    let v = if v.abs() < step.abs() * 1e-6 || v == 0.0 { 0.0 } else { v };
    if v != 0.0 && (v.abs() >= 1e6 || v.abs() < 1e-3) {
        return format!("{v:.1e}");
    }
    let d = if step > 0.0 { (-step.log10().floor()).clamp(0.0, 6.0) as usize } else { 0 };
    format!("{v:.d$}")
}

/// Dots of each cell; the colour with the most dots wins it, curves over
/// grid. `r` must be `area` x (2, 4) pixels.
pub fn braille(r: &Raster, area: Rect, buf: &mut Buffer) {
    const BITS: [[u32; 4]; 2] = [[0x01, 0x02, 0x04, 0x40], [0x08, 0x10, 0x20, 0x80]];
    for cy in 0..area.height as usize {
        for cx in 0..area.width as usize {
            let mut bits = 0u32;
            let mut votes: [(u32, u8); 8] = [(0, 0); 8];
            let mut curve = false;
            for (dx, col) in BITS.iter().enumerate() {
                for (dy, bit) in col.iter().enumerate() {
                    let c = r.px[(cy * 4 + dy) * r.w + cx * 2 + dx];
                    if c == 0 {
                        continue;
                    }
                    bits |= bit;
                    curve |= c != GRID;
                    if let Some(v) = votes.iter_mut().find(|v| v.0 == c || v.1 == 0) {
                        v.0 = c;
                        v.1 += 1;
                    }
                }
            }
            if bits == 0 {
                continue;
            }
            let color = votes.iter().filter(|v| v.1 > 0 && (!curve || v.0 != GRID)).max_by_key(|v| v.1).map_or(GRID, |v| v.0);
            let ch = char::from_u32(0x2800 + bits).unwrap_or(' ');
            buf[(area.x + cx as u16, area.y + cy as u16)].set_char(ch).set_fg(rgb(color));
        }
    }
}

// ---------------------------------------------------------------------
// Spectrum
// ---------------------------------------------------------------------

/// In-place iterative radix-2 FFT; `re.len()` must be a power of two.
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -std::f64::consts::TAU / len as f64;
        let (wr, wi) = (ang.cos(), ang.sin());
        for start in (0..n).step_by(len) {
            let (mut cr, mut ci) = (1.0, 0.0);
            for k in 0..len / 2 {
                let (a, b) = (start + k, start + k + len / 2);
                let (tr, ti) = (re[b] * cr - im[b] * ci, re[b] * ci + im[b] * cr);
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
                (cr, ci) = (cr * wr - ci * wi, cr * wi + ci * wr);
            }
        }
        len <<= 1;
    }
}

/// Magnitude spectrum of the samples of `s` inside [t0, t1], resampled onto
/// their median period (the latest power-of-two run of samples, at most
/// `max_points`), mean removed and Hann windowed. Returns (Hz, magnitude).
pub fn spectrum(s: &Series, t0: f64, t1: f64, max_points: usize) -> (Vec<f64>, Vec<f64>) {
    let (a, b) = s.visible(t0, t1);
    if b < a + 16 {
        return (Vec::new(), Vec::new());
    }
    let mut dts: Vec<f64> = s.t[a..b].windows(2).map(|w| w[1] - w[0]).filter(|d| *d > 0.0).collect();
    if dts.is_empty() {
        return (Vec::new(), Vec::new());
    }
    dts.sort_by(f64::total_cmp);
    let dt = dts[dts.len() / 2];
    let (start, end) = (s.t[a].max(t0), s.t[b - 1].min(t1));
    let avail = (((end - start) / dt) as usize + 1).min(max_points);
    if avail < 16 {
        return (Vec::new(), Vec::new());
    }
    let n = 1usize << (usize::BITS - 1 - avail.leading_zeros());
    // Centre the run in the window.
    let start = start + ((avail - n) as f64 * dt) / 2.0;
    let mut re = Vec::with_capacity(n);
    let mut i = a;
    for k in 0..n {
        let at = start + k as f64 * dt;
        while i + 1 < b && s.t[i + 1] <= at {
            i += 1;
        }
        let v = if i + 1 < b && s.t[i + 1] > s.t[i] {
            let f = ((at - s.t[i]) / (s.t[i + 1] - s.t[i])).clamp(0.0, 1.0);
            s.v[i] + (s.v[i + 1] - s.v[i]) * f
        } else {
            s.v[i]
        };
        re.push(if v.is_finite() { v } else { 0.0 });
    }
    let mean = re.iter().sum::<f64>() / n as f64;
    for (k, v) in re.iter_mut().enumerate() {
        let w = 0.5 - 0.5 * (std::f64::consts::TAU * k as f64 / (n - 1) as f64).cos();
        *v = (*v - mean) * w;
    }
    let mut im = vec![0.0; n];
    fft(&mut re, &mut im);
    let bins = n / 2;
    let norm = 4.0 / n as f64; // 2 for one side, 2 for the Hann gain
    let freq = (0..bins).map(|k| k as f64 / (n as f64 * dt)).collect();
    let mag = (0..bins).map(|k| re[k].hypot(im[k]) * norm).collect();
    (freq, mag)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::review::Curve;

    fn plot(mode: Mode) -> Plot {
        Plot { title: String::new(), mode, steps: false, equal: false, curves: Vec::<Curve>::new() }
    }

    #[test]
    fn ticks_are_round() {
        assert_eq!(ticks(0.0, 10.0, 5), vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
        assert!(ticks(1.0, 1.0, 5).is_empty());
    }

    #[test]
    fn huge_series_fills_columns() {
        let t: Vec<f64> = (0..2_000_000).map(|i| i as f64).collect();
        let v: Vec<f64> = t.iter().map(|x| x.sin()).collect();
        let s = Arc::new(Series { name: "s".into(), t: Arc::new(t), v, labels: None });
        let p = plot(Mode::Time);
        let tr = vec![Prepared { trace: Trace::Time(s), color: 0xff }];
        let f = bounds_for(&p, &tr, 0.0, 2e6, 1.0);
        let mut r = Raster::new(200, 80);
        render(&p, &tr, &f, &mut r, &[], &[]);
        let columns = (0..200).filter(|x| (0..80).any(|y| r.px[y * 200 + x] == 0xff)).count();
        assert!(columns >= 199, "{columns}");
    }

    #[test]
    fn equal_axes_keep_the_scale() {
        let p = Plot { equal: true, ..plot(Mode::Xy) };
        let tr = vec![Prepared { trace: Trace::Points(vec![0.0, 10.0], vec![0.0, 1.0]), color: 1 }];
        let f = bounds_for(&p, &tr, 0.0, 1.0, 2.0);
        assert!(((f.x1 - f.x0) / (f.y1 - f.y0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn spectrum_finds_a_tone() {
        let t: Vec<f64> = (0..4000).map(|i| i as f64 / 1000.0).collect();
        let v: Vec<f64> = t.iter().map(|x| (std::f64::consts::TAU * 80.0 * x).sin()).collect();
        let s = Series { name: "s".into(), t: Arc::new(t), v, labels: None };
        let (f, m) = spectrum(&s, 0.0, 4.0, 1 << 16);
        let k = (0..m.len()).max_by(|a, b| m[*a].total_cmp(&m[*b])).unwrap();
        assert!((f[k] - 80.0).abs() < 1.0, "{}", f[k]);
        assert!((m[k] - 1.0).abs() < 0.2, "{}", m[k]);
    }
}
