//! ROS 2 sensor screens: [v] LiDAR point cloud (`render/pointcloud.py`) and
//! [w] camera preview (`render/camera.py`).

use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};

use super::map::{BANDS, BRAILLE_BITS, band_color, braille};
use super::*;
use crate::state::now;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CloudView {
    Top,
    Front,
    Oblique,
    Free,
}

/// Sensor-frame point -> (screen horizontal, screen vertical, depth).
type Project = Box<dyn Fn([f64; 3]) -> (f64, f64, f64)>;

/// (label, depth label, projection).
fn projection(view: CloudView, yaw_deg: f64, pitch_deg: f64) -> (String, &'static str, Project) {
    let s45 = std::f64::consts::FRAC_1_SQRT_2;
    match view {
        CloudView::Top => ("Top-down (X fwd / Y left)".into(), "Height Z", Box::new(|[x, y, z]| (-y, x, z))),
        CloudView::Front => ("Front (Y left / Z up)".into(), "Depth X", Box::new(|[x, y, z]| (-y, z, x))),
        // Forward and up both push a point up the screen at 45 degrees.
        CloudView::Oblique => (
            "45 deg oblique (Y left / X+Z diagonal)".into(),
            "Perp. axis",
            Box::new(move |[x, y, z]| (-y, x * s45 + z * s45, x * s45 - z * s45)),
        ),
        // Yaw about Z, then pitch between level (0 = front) and down (90 = top).
        CloudView::Free => {
            let (cy, sy) = (yaw_deg.to_radians().cos(), yaw_deg.to_radians().sin());
            let (cp, sp) = (pitch_deg.to_radians().cos(), pitch_deg.to_radians().sin());
            (
                format!("Free camera (yaw {yaw_deg:.0} deg / pitch {pitch_deg:.0} deg)"),
                "View axis",
                Box::new(move |[x, y, z]| {
                    let fwd = x * cy + y * sy;
                    (x * sy - y * cy, fwd * sp + z * cp, fwd * cp - z * sp)
                }),
            )
        }
    }
}

fn age(last: f64, fresh: f64, stale: f64) -> Ln {
    if last == 0.0 {
        return Ln::new().dim("never");
    }
    let a = now() - last;
    Ln::new().fg(format!("{a:.1}s ago"), if a < fresh { GREEN } else if a < stale { YELLOW } else { RED })
}

pub fn pointcloud(ctx: &Ctx) -> Vec<Line<'static>> {
    let ros = crate::host::lock(&ctx.app.ros);
    let s = &ctx.app.session;
    if !ros.available {
        return vec![
            Ln::new().dim(format!(" ROS 2 unavailable: {}", ros.error)).line(),
            Ln::new().dim(format!(" Would subscribe to {}", ros.lidar_topic)).line(),
        ];
    }
    let (label, depth_label, project) = projection(s.cloud_view, s.cloud_yaw, s.cloud_pitch);
    let range = s.cloud_range.max(0.5);

    let mut lines = vec![
        Line::from(format!(" Topic: {}   Frame: {}", ros.lidar_topic, if ros.lidar_frame_id.is_empty() { "--" } else { &ros.lidar_frame_id })),
        Ln::new()
            .raw(" Last message: ")
            .spans(age(ros.lidar_last, 1.0, 5.0).0)
            .raw(format!("   Rate: {:.1} Hz   Points/msg: {}", ros.lidar_rate_hz, ros.lidar_point_count))
            .line(),
    ];
    let projected: Vec<(f64, f64, f64)> = ros.lidar_points.iter().map(|&p| project(p)).collect();
    let (d_lo, d_hi) = projected.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.2), b.max(p.2)));
    if projected.is_empty() {
        lines.push(Ln::new().dim(" No points received yet").line());
    } else {
        lines.push(Line::from(format!(" Range (sample): {:.2} - {:.2} m", ros.lidar_range.0, ros.lidar_range.1)));
        let mut l = Ln::new().raw(format!(" {depth_label}: {d_lo:.2} m "));
        for c in BANDS {
            l = l.fg("█", c);
        }
        lines.push(l.raw(format!(" {d_hi:.2} m")).line());
    }
    lines.push(blank());
    lines.push(Line::from(format!(" {label}   range +/-{range:.0} m")));

    let gw = ((ctx.width.saturating_sub(2)) | 1).clamp(21, 103);
    let gh = ((ctx.height.saturating_sub(9)) | 1).clamp(9, 35);
    let (dw, dh) = ((gw * 2) as f64, (gh * 4) as f64);
    let dot = |h: f64, v: f64| {
        let col = (h / range * dw / 2.0).round() + dw / 2.0;
        let row = dh / 2.0 - (v / range * dh / 2.0).round();
        (col >= 0.0 && row >= 0.0 && col < dw && row < dh).then_some((row as usize, col as usize))
    };
    // Per cell: dot bits and the largest depth seen (most salient wins).
    let mut cells = vec![(0u8, f64::MIN); gw * gh];
    for &(h, v, d) in &projected {
        if let Some((r, c)) = dot(h, v) {
            let cell = &mut cells[(r / 4) * gw + c / 2];
            cell.0 |= BRAILLE_BITS[c % 2][r % 4];
            cell.1 = cell.1.max(d);
        }
    }
    let origin = dot(0.0, 0.0).map(|(r, c)| (r / 4, c / 2));
    for row in 0..gh {
        let mut spans = vec![Span::raw("  ")];
        for col in 0..gw {
            let (bits, d) = cells[row * gw + col];
            if origin == Some((row, col)) {
                spans.push(Span::styled("+", Style::new().add_modifier(Modifier::BOLD)));
            } else if bits == 0 {
                spans.push(Span::raw(" "));
            } else {
                spans.push(Span::styled(braille(bits).to_string(), Style::new().fg(band_color(d, d_lo, d_hi))));
            }
        }
        lines.push(Line::from(spans));
    }
    lines
}

const ASCII_RAMP: &[u8] = b" .:-=+*#%@";

/// Grid cells fitting a frame into `max_w` x `max_h` keeping aspect; each
/// cell is one pixel wide and two tall (half blocks).
fn fit(fw: u32, fh: u32, max_w: usize, max_h: usize) -> (usize, usize) {
    if fw == 0 || fh == 0 || max_w == 0 || max_h == 0 {
        return (0, 0);
    }
    let rows_per_col = fh as f64 / (2.0 * fw as f64);
    let mut w = max_w;
    let mut h = (w as f64 * rows_per_col).round() as usize;
    if h > max_h {
        h = max_h;
        w = (h as f64 / rows_per_col).round() as usize;
    }
    (w.clamp(1, max_w), h.clamp(1, max_h))
}

fn sample(count: usize, size: u32) -> Vec<usize> {
    (0..count).map(|i| ((i as u64 * size as u64 / count as u64) as usize).min(size as usize - 1)).collect()
}

fn render_slot(slot: &crate::ros::CameraSlot, index: usize, low_bw: bool, max_w: usize, max_h: usize) -> Vec<Line<'static>> {
    let mut lines = vec![Line::from(format!(
        " [{}] {}{}",
        index + 1,
        slot.topic,
        if slot.kind == "compressed" { " (compressed)" } else { "" }
    ))];
    if !slot.error.is_empty() {
        lines.push(Ln::new().raw("   ").fg(format!("error: {}", slot.error), RED).line());
        return lines;
    }
    lines.push(
        Ln::new()
            .raw("   Last frame: ")
            .spans(age(slot.last_frame_at, 2.0, 6.0).0)
            .raw(format!("   Rate: {:.1} fps   Size: {}x{}", slot.fps, slot.width, slot.height))
            .line(),
    );
    if !slot.note.is_empty() {
        lines.push(Ln::new().raw("   ").fg(slot.note.clone(), YELLOW).line());
    }
    if slot.rgb.is_empty() {
        lines.push(dim_line("waiting for a frame..."));
        return lines;
    }
    let body_h = max_h.saturating_sub(lines.len()).max(1);
    let px = |x: usize, y: usize| {
        let i = (y * slot.width as usize + x) * 3;
        (slot.rgb[i], slot.rgb[i + 1], slot.rgb[i + 2])
    };
    if low_bw {
        // Colourless brightness ramp, capped width - fewer bytes to redraw
        // over a thin link.
        let (gw, gh) = fit(slot.width, slot.height, max_w.min(CAMERA_LOW_BW_MAX_COLS), body_h);
        let (xs, ys) = (sample(gw, slot.width), sample(gh, slot.height));
        for &y in &ys {
            let row: String = xs
                .iter()
                .map(|&x| {
                    let (r, g, b) = px(x, y);
                    let luma = 0.299 * r as f64 + 0.587 * g as f64 + 0.114 * b as f64;
                    ASCII_RAMP[(luma / 255.0 * (ASCII_RAMP.len() - 1) as f64) as usize] as char
                })
                .collect();
            lines.push(Line::from(row));
        }
    } else {
        // Truecolor half blocks: top pixel in the foreground, bottom in the
        // background of '▀'.
        let (gw, gh) = fit(slot.width, slot.height, max_w, body_h);
        let (xs, ys) = (sample(gw, slot.width), sample(gh * 2, slot.height));
        for row in 0..gh {
            let spans: Vec<Span> = xs
                .iter()
                .map(|&x| {
                    let (tr, tg, tb) = px(x, ys[row * 2]);
                    let (br, bg, bb) = px(x, ys[row * 2 + 1]);
                    Span::styled("▀", Style::new().fg(Color::Rgb(tr, tg, tb)).bg(Color::Rgb(br, bg, bb)))
                })
                .collect();
            lines.push(Line::from(spans));
        }
    }
    lines
}

pub fn camera(ctx: &Ctx) -> Vec<Line<'static>> {
    let ros = crate::host::lock(&ctx.app.ros);
    let mut lines = vec![section("CAMERA", "")];
    if !ros.available {
        lines.push(dim_line(format!("ROS 2 unavailable: {}", ros.error)));
        return lines;
    }
    let mode = if ros.camera_low_bw { Ln::new().fg("LOW BANDWIDTH", YELLOW) } else { Ln::new().fg("FULL", GREEN) };
    lines.push(Ln::new().raw("   Render: ").spans(mode.0).line());
    lines.push(blank());
    let configured: Vec<(usize, &crate::ros::CameraSlot)> = ros.cameras.iter().enumerate().filter(|(_, s)| !s.topic.is_empty()).collect();
    if configured.is_empty() {
        lines.push(dim_line("no camera topics set - [1]/[2] to subscribe to a sensor_msgs/Image or CompressedImage topic"));
        return lines;
    }
    let avail = ctx.height.saturating_sub(lines.len()).max(configured.len());
    let pane_h = avail / configured.len();
    for (i, (index, slot)) in configured.iter().enumerate() {
        if i > 0 {
            lines.push(blank());
        }
        lines.extend(render_slot(slot, *index, ros.camera_low_bw, ctx.width, pane_h.saturating_sub(usize::from(i > 0))));
    }
    lines
}
