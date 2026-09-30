//! [y] preflight go / no-go (the checks live in `preflight.rs`).

use ratatui::style::{Color, Modifier, Style};
use ratatui::text::Line;

use super::*;
use crate::preflight::{self, Status};

pub fn draw(ctx: &Ctx) -> Vec<Line<'static>> {
    let sys = crate::host::lock(&ctx.app.sys);
    let checks = preflight::checks(ctx.st, &sys, &ctx.app.settings);
    drop(sys);
    let mut lines = vec![blank()];
    lines.extend(check_lines(&checks));
    lines.push(blank());
    lines.push(dim_line("   Unknown rows never block. Warnings are advice; PX4's own arming checks have the final say."));
    lines
}

/// Verdict banner + one row per check, grouped - shared with the fleet.
pub fn check_lines(checks: &[preflight::Check]) -> Vec<Line<'static>> {
    let (worst, _, _) = preflight::verdict(checks);
    let banner = match worst {
        Status::Fail => Style::new().bg(RED).fg(Color::White),
        Status::Warn => Style::new().bg(YELLOW).fg(Color::Black),
        _ => Style::new().bg(GREEN).fg(Color::Black),
    }
    .add_modifier(Modifier::BOLD);

    let mut lines = vec![Ln::new().raw("   ").st(format!("  {}  ", preflight::verdict_text(checks)), banner).line()];
    let mut group = "";
    for c in checks {
        if c.group != group {
            group = c.group;
            lines.push(blank());
            lines.push(section(group, ""));
        }
        let (mark, color) = match c.status {
            Status::Pass => ("PASS", Some(GREEN)),
            Status::Warn => ("WARN", Some(YELLOW)),
            Status::Fail => ("FAIL", Some(RED)),
            Status::Unknown => (" -- ", None),
        };
        let mark = match color {
            Some(color) => Ln::new().st(mark, Style::new().fg(color).add_modifier(Modifier::BOLD)),
            None => Ln::new().dim(mark),
        };
        lines.push(Ln::new().raw("   ").spans(mark.0).raw(format!("  {:<16}", c.name)).raw(c.detail.clone()).line());
    }
    lines
}
