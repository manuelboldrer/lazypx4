//! [v] FLIGHT REVIEW: the log list, or a page of plots for the loaded log
//! (lazyjagg's Flight Review pages, drawn as braille).

use ratatui::Frame;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::{Modifier, Style};
use ratatui::text::Line;
use ratatui::widgets::{Block, BorderType, Paragraph, Widget};

use super::{CRUST, GREEN, Ln, MAUVE, OVERLAY0, RED, SURFACE1, TEXT, YELLOW, window_start};
use crate::app_review::{Cached, ReviewLog, ReviewState, file_name};
use crate::picker::human_size;
use crate::plot::{self, Raster, rgb};
use crate::review::Mode;

const Y_GUTTER: u16 = 8;

pub fn draw(frame: &mut Frame, r: &mut ReviewState, area: Rect) {
    if let Some(l) = &r.loading {
        let pct = l.progress.load(std::sync::atomic::Ordering::Relaxed).min(100);
        let width = area.width.saturating_sub(8).min(60) as usize;
        let done = width * pct as usize / 100;
        let lines = vec![
            Line::default(),
            Ln::new().raw("   Loading ").bold(file_name(&l.path)).dim(format!("  {:.1}s", l.started.elapsed().as_secs_f64())).line(),
            Line::default(),
            Ln::new().raw("   ").fg("█".repeat(done), GREEN).fg("░".repeat(width - done), SURFACE1).raw(format!(" {pct}%")).line(),
            Line::default(),
            Ln::new().raw("   ").dim("parsing, then deriving the review series; telemetry keeps running").line(),
        ];
        frame.render_widget(Paragraph::new(lines), area);
        return;
    }
    match r.log.as_mut() {
        Some(log) => {
            let error = r.error.clone();
            draw_log(frame, log, &error, area);
        }
        None => draw_list(frame, r, area),
    }
}

fn draw_list(frame: &mut Frame, r: &mut ReviewState, area: Rect) {
    let mut lines = vec![
        Ln::new().raw(" ").dim("Downloaded logs in ").fg(r.dir.display().to_string(), TEXT).line(),
        Ln::new().raw(" ").dim("ENTER plots the Flight Review pages · [o] opens any .ulg from disk").line(),
        Line::default(),
    ];
    if !r.error.is_empty() {
        lines.insert(2, Ln::new().raw(" ").fg(r.error.clone(), RED).line());
    }
    if r.files.is_empty() {
        lines.push(Ln::new().raw("   ").dim("No .ulg files here yet. Download one on ").bold("[l]").dim(" FLIGHT LOGS, or press ").bold("[o]").dim(" to browse.").line());
        frame.render_widget(Paragraph::new(lines), area);
        r.page_rows = 1;
        return;
    }
    lines.push(Ln::new().st(format!("   {:<44} {:>10}  {}", "FILE", "SIZE", "MODIFIED"), Style::new().fg(OVERLAY0).add_modifier(Modifier::BOLD)).line());
    let rows = (area.height as usize).saturating_sub(lines.len()).max(1);
    r.page_rows = rows;
    let start = window_start(r.index, r.files.len(), rows);
    for (i, f) in r.files.iter().enumerate().skip(start).take(rows) {
        let when = f.modified.map(|m| chrono::DateTime::<chrono::Local>::from(m).format("%Y-%m-%d %H:%M").to_string()).unwrap_or_default();
        let rel = f.path.strip_prefix(&r.dir).unwrap_or(&f.path).display().to_string();
        let text = format!(" {} {:<44} {:>10}  {when}", if i == r.index { "▶" } else { " " }, cut(&rel, 44), human_size(f.size));
        let style = if i == r.index { Style::new().bg(GREEN).fg(CRUST).add_modifier(Modifier::BOLD) } else { Style::new() };
        lines.push(Line::styled(text, style));
    }
    frame.render_widget(Paragraph::new(lines), area);
}

/// `s` cut to `n` characters with an ellipsis.
fn cut(s: &str, n: usize) -> String {
    if s.chars().count() <= n {
        return s.to_string();
    }
    let mut out: String = s.chars().take(n.saturating_sub(1)).collect();
    out.push('…');
    out
}

/// The end of `s` when it is longer than `n`.
fn cut_left(s: &str, n: usize) -> String {
    let count = s.chars().count();
    if count <= n {
        return s.to_string();
    }
    let mut out = String::from("…");
    out.extend(s.chars().skip(count - n.saturating_sub(1)));
    out
}

fn clock(secs: f64) -> String {
    let s = secs.max(0.0) as u64;
    format!("{:02}:{:02}", s / 60, s % 60)
}

fn level_color(level: &str) -> Style {
    match level {
        "EMERG" | "ALERT" | "CRIT" | "ERROR" => Style::new().fg(RED),
        "WARN" => Style::new().fg(YELLOW),
        "FLAG" => Style::new().fg(MAUVE),
        "PARAM" | "DEBUG" => Style::new().fg(OVERLAY0),
        _ => Style::new().fg(TEXT),
    }
}

fn draw_log(frame: &mut Frame, log: &mut ReviewLog, note: &str, area: Rect) {
    if area.height < 8 || area.width < 40 {
        return;
    }
    let msg_rows: u16 = if log.show_messages { 8.min(area.height / 3) } else { 1 };
    let header = Rect::new(area.x, area.y, area.width, 1);
    let tabs = Rect::new(area.x, area.y + 1, area.width, 1);
    let plots = Rect::new(area.x, area.y + 2, area.width, area.height - 2 - msg_rows);
    let bottom = Rect::new(area.x, area.bottom() - msg_rows, area.width, msg_rows);

    // Header: file, vehicle, firmware, length, window.
    let (a, b) = log.view;
    let mut head = Ln::new().raw(" ").bold(file_name(&log.path));
    if let Some(sys) = log.info("sys_name") {
        head = head.dim(" · ").raw(sys.to_string());
    }
    if let Some(ver) = log.info("ver_sw_release_str").or(log.info("ver_sw")) {
        head = head.dim(" · ").raw(cut(ver, 16));
    }
    head = head.dim(" · ").raw(format!("length {}", clock(log.span.1 - log.span.0)));
    let zoomed = (a, b) != log.span;
    head = head.dim(" · view ").fg(format!("{}–{}", clock(log.rel(a)), clock(log.rel(b))), if zoomed { YELLOW } else { TEXT });
    if !log.warnings.is_empty() {
        head = head.dim(" · ").fg(cut(&log.warnings[0], 40), YELLOW);
    }
    frame.render_widget(Paragraph::new(head.line()), header);

    // Page tabs.
    // Page tabs, scrolled so the current one is always in sight.
    let labels: Vec<String> = log.pages.iter().enumerate().map(|(i, p)| if i < 9 { format!(" {} {} ", i + 1, p.name) } else { format!(" {} ", p.name) }).collect();
    let room = (area.width as usize).saturating_sub(4);
    let mut first = 0;
    while first < log.page && labels[first..=log.page].iter().map(|l| l.chars().count()).sum::<usize>() > room {
        first += 1;
    }
    let mut tab_line = Ln::new().fg(if first > 0 { "‹" } else { " " }, OVERLAY0);
    let mut used = 0;
    for (i, label) in labels.iter().enumerate().skip(first) {
        used += label.chars().count();
        if used > room {
            tab_line = tab_line.fg("›", OVERLAY0);
            break;
        }
        tab_line = if i == log.page { tab_line.st(label.clone(), Style::new().bg(GREEN).fg(CRUST).add_modifier(Modifier::BOLD)) } else { tab_line.fg(label.clone(), OVERLAY0) };
    }
    frame.render_widget(Paragraph::new(tab_line.line()), tabs);

    // Plot grid: one per row for a few plots, then two or three.
    let n = log.pages[log.page].plots.len();
    log.focus = log.focus.min(n.saturating_sub(1));
    let mut rects = Vec::new();
    if log.maximized {
        rects.push((log.focus, plots));
    } else {
        let per_row = match n {
            0..=3 => 1,
            4..=6 => 2,
            _ => 3,
        };
        let rows = n.div_ceil(per_row).max(1);
        for row in 0..rows {
            let y0 = plots.y + (plots.height as usize * row / rows) as u16;
            let y1 = plots.y + (plots.height as usize * (row + 1) / rows) as u16;
            let ids: Vec<usize> = (row * per_row..((row + 1) * per_row).min(n)).collect();
            for (k, id) in ids.iter().enumerate() {
                let x0 = plots.x + (plots.width as usize * k / ids.len()) as u16;
                let x1 = plots.x + (plots.width as usize * (k + 1) / ids.len()) as u16;
                rects.push((*id, Rect::new(x0, y0, x1 - x0, y1 - y0)));
            }
        }
    }
    let page = log.page;
    for (id, r) in rects {
        if r.width >= 16 && r.height >= 5 {
            draw_plot(frame.buffer_mut(), log, page, id, r);
        }
    }

    draw_bottom(frame, log, note, bottom);
}

fn draw_plot(buf: &mut Buffer, log: &mut ReviewLog, page: usize, id: usize, r: Rect) {
    let plot = log.pages[page].plots[id].clone();
    let focused = log.focus == id;
    let block = Block::bordered()
        .border_type(BorderType::Rounded)
        .border_style(Style::new().fg(if focused { GREEN } else { SURFACE1 }))
        .title(Line::from(format!(" {}: {} ", id + 1, cut(&plot.title, r.width.saturating_sub(8) as usize))).style(Style::new().fg(if focused { GREEN } else { TEXT })));
    let inner = block.inner(r);
    block.render(r, buf);
    if inner.width <= Y_GUTTER * 3 || inner.height < 3 {
        return;
    }
    let gutter = match plot.mode {
        Mode::Flags => plot.curves.iter().map(|c| c.label.chars().count() as u16 + 3).max().unwrap_or(0).clamp(Y_GUTTER, inner.width / 3),
        _ => Y_GUTTER,
    };
    let chart = Rect::new(inner.x + gutter, inner.y, inner.width - gutter, inner.height - 1);
    let (t0, t1) = log.view;
    let timed = matches!(plot.mode, Mode::Time | Mode::Flags);
    let origin = log.span.0;
    let xn = (chart.width as usize / 14).clamp(2, 12);
    let yn = (chart.height as usize / 3).clamp(2, 8);
    let axes = |b: &plot::Bounds| {
        let xs = if timed { plot::ticks_offset(b.x0, b.x1, xn, origin) } else { plot::ticks(b.x0, b.x1, xn) };
        (xs, plot::ticks(b.y0, b.y1, yn))
    };

    let key = (t0.to_bits(), t1.to_bits(), 0, chart.width, chart.height);
    if log.cache.get(&(page, id)).is_none_or(|c| c.key != key) {
        let traces = plot::prepare(&plot, &log.store, t0, t1);
        let (w, h) = (chart.width as usize * 2, chart.height as usize * 4);
        let bounds = plot::bounds_for(&plot, &traces, t0, t1, w as f64 / h as f64);
        let (xs, ys) = axes(&bounds);
        let mut raster = Raster::new(w, h);
        plot::render(&plot, &traces, &bounds, &mut raster, &xs, &ys);
        log.cache.insert((page, id), Cached { key, raster, bounds });
    }
    let cached = &log.cache[&(page, id)];
    let bounds = cached.bounds;
    plot::braille(&cached.raster, chart, buf);
    let (xs, ys) = axes(&bounds);

    // Y ticks.
    let dim = Style::new().fg(OVERLAY0);
    if plot.mode != Mode::Flags {
        let step = if ys.len() > 1 { ys[1] - ys[0] } else { 0.0 };
        for t in ys {
            let row = chart.y + (bounds.y_frac(t) * (chart.height - 1) as f64).round() as u16;
            let label = plot::tick_label(t, step);
            buf.set_string(inner.x, row, format!("{:>w$}", cut(&label, 7), w = Y_GUTTER as usize - 1), dim);
        }
    }
    // X ticks.
    let step = if xs.len() > 1 { xs[1] - xs[0] } else { 0.0 };
    let base = inner.bottom() - 1;
    let mut free_from = chart.x;
    for t in xs {
        let col = chart.x + (bounds.x_frac(t) * (chart.width - 1) as f64).round() as u16;
        let label = plot::tick_label(if timed { t - origin } else { t }, step);
        let n = label.chars().count() as u16;
        let at = col.saturating_sub(n / 2).max(chart.x);
        if at >= free_from && at + n <= chart.right() {
            buf.set_string(at, base, &label, dim);
            free_from = at + n + 1;
        }
    }
    let unit = match plot.mode {
        Mode::Fft => "Hz",
        Mode::Xy if plot.equal => "m",
        Mode::Xy => "",
        _ => "s",
    };
    buf.set_string(inner.x + 1, base, unit, dim);

    // Tracker.
    let tracker = log.tracker;
    if timed && tracker >= bounds.x0 && tracker <= bounds.x1 {
        let col = chart.x + (bounds.x_frac(tracker) * (chart.width - 1) as f64).round() as u16;
        for y in chart.top()..chart.bottom() {
            buf[(col, y)].set_char('│').set_fg(YELLOW);
        }
    }

    if plot.mode == Mode::Flags {
        let n = plot.curves.len().max(1) as u32;
        for (i, c) in plot.curves.iter().enumerate() {
            let row = chart.y + (i as u32 * chart.height as u32 / n) as u16;
            if i > 0 && row == chart.y + ((i as u32 - 1) * chart.height as u32 / n) as u16 {
                continue; // More lanes than rows: the picture still shows them.
            }
            let on = log.store.get(&c.name).and_then(|s| s.value_at(tracker)).is_some_and(|v| v != 0.0 && !v.is_nan());
            let (mark, color) = if on { ("● ", rgb(c.color)) } else { ("  ", OVERLAY0) };
            let label = cut_left(&c.label, gutter.saturating_sub(3) as usize);
            buf.set_string(inner.x, row, format!("{mark}{label}"), Style::new().fg(color));
        }
        return;
    }

    // Legend, top right, with values at the tracker.
    let mut rows = Vec::new();
    for c in &plot.curves {
        let y = log.store.get(&c.name);
        let value = if plot.mode == Mode::Fft { None } else { y.and_then(|s| s.text_at(tracker)) };
        if plot.mode == Mode::Xy
            && let (Some(ys), Some(xs)) = (y, c.x.as_ref().and_then(|n| log.store.get(n)))
            && !log.store.aux.contains(&ys.name)
            && let (Some(px), Some(py)) = (xs.value_at(tracker), ys.value_at(tracker))
            && px >= bounds.x0
            && px <= bounds.x1
            && py >= bounds.y0
            && py <= bounds.y1
        {
            let col = chart.x + (bounds.x_frac(px) * (chart.width - 1) as f64).round() as u16;
            let row = chart.y + (bounds.y_frac(py) * (chart.height - 1) as f64).round() as u16;
            buf[(col, row)].set_char('●').set_fg(YELLOW);
        }
        if plot.mode == Mode::Xy && c.label.is_empty() {
            continue;
        }
        rows.push((c.label.clone(), c.color, value));
    }
    let width = rows.iter().map(|r| r.0.chars().count() + r.2.as_ref().map_or(0, |v| v.chars().count().min(12) + 1) + 3).max().unwrap_or(0).min(chart.width as usize * 2 / 3).max(8);
    let shown = (chart.height as usize).min(rows.len());
    let x = chart.right().saturating_sub(width as u16);
    for (i, (label, color, value)) in rows.iter().take(shown).enumerate() {
        let y = chart.y + i as u16;
        let value = value.as_deref().map(|v| cut(v, 12)).unwrap_or_default();
        let name = cut_left(label, width.saturating_sub(value.chars().count() + 4));
        buf.set_string(x, y, " ".repeat(width), Style::new().bg(CRUST));
        buf.set_string(x, y, "━ ", Style::new().fg(rgb(*color)).bg(CRUST));
        buf.set_string(x + 2, y, &name, Style::new().fg(TEXT).bg(CRUST));
        let vx = (x + width as u16).saturating_sub(value.chars().count() as u16 + 1);
        buf.set_string(vx, y, &value, Style::new().fg(GREEN).bg(CRUST));
    }
}

/// Tracker time, mode and the latest message; or the message list.
fn draw_bottom(frame: &mut Frame, log: &ReviewLog, note: &str, area: Rect) {
    let tracker = log.tracker;
    let msgs = &log.store.meta.messages;
    let last = msgs.partition_point(|m| m.0 <= tracker);
    if area.height <= 1 {
        let mut l = Ln::new().raw(" ").fg(format!("t {:.2}s", log.rel(tracker)), YELLOW);
        if let Some(mode) = log.store.get("/_review/flight_mode").and_then(|s| s.text_at(tracker)) {
            l = l.dim(" · ").raw(mode);
        }
        if let Some(armed) = crate::review::resolve(&log.store, "vehicle_status/arming_state").and_then(|n| log.store.get(&n)).and_then(|s| s.value_at(tracker)) {
            l = if armed == 2.0 { l.dim(" · ").fg("ARMED", GREEN) } else { l.dim(" · disarmed") };
        }
        let text = if !note.is_empty() {
            Some((note.to_string(), Style::new().fg(MAUVE)))
        } else {
            msgs[..last].iter().rev().find(|m| m.1 != "PARAM" && m.1 != "DEBUG").map(|m| (format!("{:.1}s [{}] {}", log.rel(m.0), m.1, m.2), level_color(&m.1)))
        };
        if let Some((text, style)) = text {
            l = l.dim(" · ").st(cut(&text, area.width.saturating_sub(40) as usize), style);
        }
        frame.render_widget(Paragraph::new(l.line()), area);
        return;
    }
    let rows = area.height as usize - 1;
    let start = last.saturating_sub(rows / 2 + 1).min(msgs.len().saturating_sub(rows));
    let mut lines = vec![Ln::new().fg("── messages ", OVERLAY0).dim(format!("{} · n/N jump · m hide", msgs.len())).line()];
    for (i, m) in msgs.iter().enumerate().skip(start).take(rows) {
        let current = i + 1 == last;
        let mut style = level_color(&m.1);
        if current {
            style = style.add_modifier(Modifier::BOLD | Modifier::REVERSED);
        }
        lines.push(Line::styled(cut(&format!(" {:>8.2}s {:<6} {}", log.rel(m.0), m.1, m.2), area.width as usize), style));
    }
    frame.render_widget(Paragraph::new(lines), area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;

    /// On a real log named by LAZYPX4_TEST_ULOG: every page draws at a few
    /// sizes and zooms without panicking.
    #[test]
    fn real_log_draws_every_page() {
        let Ok(path) = std::env::var("LAZYPX4_TEST_ULOG") else { return };
        let mut r = ReviewState::default();
        let started = std::time::Instant::now();
        r.log = Some(crate::app_review::load_for_test(std::path::Path::new(&path)).unwrap());
        eprintln!("loaded in {:.2}s", started.elapsed().as_secs_f64());
        let names: Vec<_> = r.log.as_ref().unwrap().pages.iter().map(|p| (p.name, p.plots.len())).collect();
        eprintln!("pages {names:?}");
        for (w, h) in [(200, 60), (100, 30), (60, 12)] {
            let mut term = Terminal::new(TestBackend::new(w, h)).unwrap();
            for page in 0..names.len() {
                for (zoom, msgs, max) in [(false, false, false), (true, true, false), (false, false, true)] {
                    let log = r.log.as_mut().unwrap();
                    log.page = page;
                    log.show_messages = msgs;
                    log.maximized = max;
                    log.tracker = (log.span.0 + log.span.1) / 2.0;
                    log.view = if zoom { (log.tracker - 0.5, log.tracker + 0.5) } else { log.span };
                    term.draw(|f| draw(f, &mut r, f.area())).unwrap();
                }
            }
        }
        let mut term = Terminal::new(TestBackend::new(160, 50)).unwrap();
        let log = r.log.as_mut().unwrap();
        log.page = 14;
        log.view = log.span;
        log.maximized = false;
        log.show_messages = false;
        term.draw(|f| draw(f, &mut r, f.area())).unwrap();
        let buf = term.backend().buffer();
        let text: String = (0..buf.area.height).map(|y| (0..buf.area.width).map(|x| buf[(x, y)].symbol().to_string()).collect::<String>() + "\n").collect();
        eprintln!("{text}");
    }
}
