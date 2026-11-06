//! [e] RECORD: run header, the recorder list, markers and the finish keys
//! on one page.

use ratatui::style::{Modifier, Style};
use ratatui::text::Line;

use super::*;
use crate::record::{Status, hms, tilde};

fn status_style(s: &Status) -> Style {
    match s {
        Status::Recording => Style::new().fg(RED).add_modifier(Modifier::BOLD),
        Status::Stopping => Style::new().fg(YELLOW),
        Status::Done => Style::new().fg(GREEN),
        Status::Failed(_) => Style::new().fg(RED),
        Status::Idle | Status::Unavailable(_) => Style::new().add_modifier(Modifier::DIM),
    }
}

fn clip(text: &str, width: usize) -> String {
    if text.chars().count() <= width {
        text.to_string()
    } else {
        let mut s: String = text.chars().take(width.saturating_sub(1)).collect();
        s.push('…');
        s
    }
}

pub fn draw(ctx: &Ctx) -> Vec<Line<'static>> {
    let app = ctx.app;
    let rec = &app.rec;
    let width = ctx.width;
    let mut lines = Vec::new();

    // Run header.
    let free = rec.disk_free.map(|f| format!("{:.0} GB free", f as f64 / 1e9)).unwrap_or_default();
    let free_color = if rec.disk_free.is_some_and(|f| (f as f64) < rec.cfg.disk_low_gb * 1e9) { RED } else { GREEN };
    match &rec.run {
        Some(run) => {
            let state = if rec.recording() {
                let blink = (crate::state::now() * 2.0) as i64 % 2 == 0;
                Ln::new().st(" ● REC ", Style::new().fg(if blink { RED } else { CRUST }).bg(if blink { CRUST } else { RED }).add_modifier(Modifier::BOLD))
            } else if run.finished_wall.is_some() {
                Ln::new().st(" FINISHED ", Style::new().fg(CRUST).bg(GREEN).add_modifier(Modifier::BOLD))
            } else {
                Ln::new().st(" STOPPED ", Style::new().fg(CRUST).bg(YELLOW).add_modifier(Modifier::BOLD))
            };
            lines.push(
                state
                    .raw(format!("  {}  ", hms(run.elapsed_s())))
                    .bold(clip(&tilde(&run.dir), width.saturating_sub(40)))
                    .raw("   ")
                    .fg(free, free_color)
                    .line(),
            );
        }
        None => lines.push(
            Ln::new()
                .st(" NOT RECORDING ", Style::new().add_modifier(Modifier::REVERSED))
                .raw(format!("  runs go to {}/<date>_<time>/   ", rec.cfg.dir))
                .fg(free, free_color)
                .line(),
        ),
    }
    let mut info = Ln::new().raw("   ");
    match (&rec.tmux_session, rec.state_file()) {
        (Some(s), Some(f)) => info = info.dim(format!("tmux session {s} · run dir also in {}", tilde(&f))),
        _ => info = info.dim("not inside tmux - no pane recording"),
    }
    if let Some(p) = app.link.tlog_path() {
        info = info.dim(format!(" · session tlog {}", p.display()));
    }
    lines.push(info.line());
    lines.push(blank());

    // Recorders.
    lines.push(section("RECORDERS", "j/k select · ENTER start/stop · i include in [R] · V screen · t rosbag topics"));
    let name_w = rec.recorders.iter().map(|r| r.name.chars().count()).max().unwrap_or(10).clamp(10, 20);
    for (i, r) in rec.recorders.iter().enumerate() {
        let selected = i == rec.cursor;
        let pointer = if selected { "▸" } else { " " };
        let include = if !r.available() {
            Ln::new().dim("[-]")
        } else if r.include {
            Ln::new().fg("[x]", GREEN)
        } else {
            Ln::new().dim("[ ]")
        };
        let time = match (r.status == Status::Recording, r.started, r.ran_for) {
            (true, Some(s), _) => hms(s.elapsed().as_secs()),
            (false, _, Some(d)) => hms(d.as_secs()),
            _ => String::new(),
        };
        let size = if r.bytes > 0 || r.active() { crate::state::size_string(r.bytes) } else { String::new() };
        let rate = if r.status == Status::Recording && r.rate > 0.0 {
            format!("{}/s", crate::state::size_string(r.rate as u64))
        } else {
            String::new()
        };
        let detail = match &r.status {
            Status::Failed(why) | Status::Unavailable(why) => why.clone(),
            _ if r.detail.is_empty() || r.status == Status::Idle => r.idle_detail(&rec.cfg),
            _ => r.detail.clone(),
        };
        let used = 4 + 4 + name_w + 1 + 9 + 9 + 11 + 12;
        let name_style = if selected { Style::new().add_modifier(Modifier::BOLD | Modifier::REVERSED) } else { Style::new().add_modifier(Modifier::BOLD) };
        lines.push(
            Ln::new()
                .fg(format!(" {pointer} "), GREEN)
                .spans(include.0)
                .raw(" ")
                .st(format!("{:<name_w$}", r.name), name_style)
                .raw(" ")
                .st(format!("{:<9}", r.status.label()), status_style(&r.status))
                .raw(format!("{time:<9}{size:>11}{rate:>12}  "))
                .dim(clip(&detail, width.saturating_sub(used)))
                .line(),
        );
    }
    lines.push(blank());

    if let Some(view) = app.rec_jobs.topics.as_ref() {
        draw_topics(ctx, view, &mut lines);
        return lines;
    }

    // Markers.
    let quick: Vec<String> = rec.cfg.markers.iter().take(9).enumerate().map(|(i, m)| format!("{} {m}", i + 1)).collect();
    lines.push(section("MARKERS", &clip(&format!("{} · M note{}", quick.join(" · "), if rec.cfg.auto_markers { " · auto: arm, mode, alerts" } else { "" }), width.saturating_sub(16))));
    let tail_rows = ctx.height.saturating_sub(lines.len() + 4).max(1);
    match rec.run.as_ref().map(|r| &r.markers) {
        Some(markers) if !markers.is_empty() => {
            let skip = markers.len().saturating_sub(tail_rows);
            for m in &markers[skip..] {
                lines.push(
                    Ln::new()
                        .raw(format!("   +{}  ", hms(m.elapsed_s as u64)))
                        .dim(m.wall.format("%H:%M:%S  ").to_string())
                        .fg(m.label.clone(), CYAN)
                        .raw(format!(" {}", clip(&m.text, width.saturating_sub(34))))
                        .line(),
                );
            }
        }
        _ => lines.push(dim_line("no markers yet")),
    }
    lines.push(blank());

    // Finish / PX4 log / upload.
    let st = ctx.st;
    let mut job = Ln::new().raw("   ");
    if app.rec_jobs.want_log {
        job = job.fg("fetching the PX4 log list...", YELLOW);
    } else if app.rec_jobs.downloading && st.dl_active {
        let pct = if st.dl_size > 0 { st.dl_received as f64 * 100.0 / st.dl_size as f64 } else { 0.0 };
        job = job.fg(format!("downloading PX4 log {} into the run: {pct:.0}%  ([x] cancel)", st.dl_id), YELLOW);
    } else if app.rec_jobs.uploading.is_some() {
        job = job.fg("uploading to Flight Review...  ([x] cancel)", YELLOW);
    } else if let Some(url) = rec.run.as_ref().and_then(|r| r.review_url.clone()) {
        job = job.raw("Flight Review: ").fg(url, GREEN);
    } else {
        job = job.dim("[p] newest PX4 log -> <run>/px4/ · [u] upload it to Flight Review");
    }
    lines.push(section("FINISH", "[F] stop + parameters + manifest.json + summary.md + name · [E] rename · [Y] copy path"));
    lines.push(job.line());
    lines
}

/// `t`: the rosbag topic list in place of the markers / finish sections.
fn draw_topics(ctx: &Ctx, view: &crate::app_record::TopicView, lines: &mut Vec<Line<'static>>) {
    let rec = &ctx.app.rec;
    let width = ctx.width;
    let in_bag: Vec<&String> = rec.recorders.iter().filter(|r| r.is_rosbag()).flat_map(|r| r.bag_topics()).collect();
    let excluded = view.topics.iter().filter(|t| rec.cfg.rosbag_excluded(t).is_some()).count();
    let title = format!("ROSBAG TOPICS {}/{} recorded", view.topics.len() - excluded, view.topics.len());
    lines.push(section(&title, "j/k · SPACE record / leave out · a clear excludes · r refresh · t/ESC back"));
    if view.loading {
        lines.push(Ln::new().raw("   ").fg("ros2 topic list...", YELLOW).line());
    } else if !view.error.is_empty() {
        lines.push(Ln::new().raw("   ").fg(clip(&view.error, width.saturating_sub(4)), RED).line());
    }
    if view.topics.is_empty() {
        if !view.loading {
            lines.push(dim_line("no topics (is ROS up? [record] ros_setup)"));
        }
        return;
    }
    let rows = ctx.height.saturating_sub(lines.len() + 2).max(3);
    let start = view.cursor.saturating_sub(rows / 2).min(view.topics.len().saturating_sub(rows));
    let name_w = view.topics.iter().map(|t| t.chars().count()).max().unwrap_or(10).min(width.saturating_sub(30).max(10));
    for (i, topic) in view.topics.iter().enumerate().skip(start).take(rows) {
        let selected = i == view.cursor;
        let pointer = if selected { "▸" } else { " " };
        let (mark, why) = match rec.cfg.rosbag_excluded(topic) {
            None => (Ln::new().fg("[x]", GREEN), String::new()),
            Some(None) => (Ln::new().fg("[ ]", RED), "left out".to_string()),
            Some(Some(why)) => (Ln::new().dim("[-]"), why),
        };
        let live = if in_bag.contains(&topic) { "● in the bag" } else { "" };
        let style = if selected { Style::new().add_modifier(Modifier::BOLD | Modifier::REVERSED) } else { Style::new() };
        lines.push(
            Ln::new()
                .fg(format!(" {pointer} "), GREEN)
                .spans(mark.0)
                .raw(" ")
                .st(format!("{:<name_w$}", clip(topic, name_w)), style)
                .raw("  ")
                .fg(live, RED)
                .raw(" ")
                .dim(why)
                .line(),
        );
    }
}
