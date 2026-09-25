//! List screens with their own cursor: flight modes, parameters, event log,
//! flight logs and the NSH shell. Each windows itself to the panel height.

use ratatui::style::{Modifier, Style};
use ratatui::text::{Line, Span};

use super::*;
use crate::mav::{modes, params};
use crate::parammeta::Entry;
use crate::state::now;

fn selected_style() -> Style {
    Style::new().fg(GREEN).add_modifier(Modifier::BOLD)
}

pub fn modes(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let options = modes::mode_options(st);
    let query = &ctx.app.session.search_query;

    let source = if st.custom_modes.is_empty() {
        if st.custom_modes_requested_at > 0.0 {
            Ln::new().fg("legacy list (waiting for AVAILABLE_MODES...)", YELLOW)
        } else {
            Ln::new().dim("legacy PX4 list (no Standard Modes Protocol reply)")
        }
    } else {
        let total = if st.custom_modes_total > 0 { st.custom_modes_total.to_string() } else { "?".into() };
        Ln::new().fg(format!("Standard Modes Protocol ({}/{total})", st.custom_modes.len()), GREEN)
    };
    let mut lines = vec![
        Ln::new().raw(" Current: ").bold(st.mode.clone()).raw("    ").spans(vec![armed_span(st.armed)]).line(),
        Ln::new().raw(" Source: ").spans(source.0).line(),
        blank(),
    ];

    if options.is_empty() {
        lines.push(dim_line("no modes known yet"));
        return lines;
    }
    let cursor = ctx.app.session.mode_index % options.len();
    let height = ctx.height.saturating_sub(lines.len()).max(1);
    let start = window_start(cursor, options.len(), height);
    for (i, option) in options.iter().enumerate().skip(start).take(height) {
        let selected = i == cursor;
        let style = if selected { selected_style() } else { Style::new() };
        let marker = if selected { " > " } else { "   " };
        lines.push(
            Ln::new()
                .st(marker, style)
                .spans(highlight(&option.label, query, style))
                .line(),
        );
    }
    lines
}

/// Description / allowed values / range of the selected parameter, the way
/// QGC shows it. `expanded` (while editing) lists every option.
fn detail_lines(p: &crate::state::Parameter, entry: Option<&Entry>, expanded: bool) -> Vec<Line<'static>> {
    let Some(e) = entry else {
        return vec![dim_line("no description available for this parameter")];
    };
    let mut out = Vec::new();
    let mut head = Ln::new().raw(" ").bold(e.short.clone());
    if e.reboot {
        head = head.raw("  ").fg("(reboot required)", YELLOW);
    }
    out.push(head.line());

    let on = Style::new().fg(GREEN).add_modifier(Modifier::BOLD);
    if !e.enum_values.is_empty() {
        let items: Vec<Span> = e
            .enum_values
            .iter()
            .map(|(v, t)| {
                let text = format!("{} ({t})", params::fmt_g(*v, 6));
                if (v - p.value).abs() < 1e-6 { Span::styled(text, on) } else { Span::raw(text) }
            })
            .collect();
        if expanded {
            out.extend(items.into_iter().map(|s| Line::from(vec![Span::raw("   "), s])));
        } else {
            let mut l = Ln::new().raw("  ");
            for s in items {
                l = l.raw(" ").spans(vec![s]).raw(" ");
            }
            out.push(l.line());
        }
    } else if !e.bitmask.is_empty() {
        let raw = if p.value.is_finite() { p.value.round() as i64 } else { 0 };
        if expanded {
            for (bit, text) in &e.bitmask {
                let set = *bit < 63 && raw >> bit & 1 == 1;
                let mark = if set { Ln::new().fg("[x]", GREEN) } else { Ln::new().dim("[ ]") };
                out.push(Ln::new().raw(format!("   bit {bit} ")).spans(mark.0).raw(format!(" {text}")).line());
            }
        } else {
            let mut l = Ln::new().raw("  ");
            for (bit, text) in &e.bitmask {
                let set = *bit < 63 && raw >> bit & 1 == 1;
                l = l.raw(" ");
                l = if set { l.fg(bit.to_string(), GREEN) } else { l.dim(bit.to_string()) };
                l = l.raw(format!(" {text} "));
            }
            out.push(l.line());
        }
    } else {
        let mut facts = Vec::new();
        if e.min.is_some() || e.max.is_some() {
            let f = |v: Option<f64>| v.map(|v| params::fmt_g(v, 6)).unwrap_or_else(|| "..".into());
            facts.push(format!("range {} .. {}", f(e.min), f(e.max)));
        }
        if !e.units.is_empty() {
            facts.push(format!("unit {}", e.units));
        }
        if !facts.is_empty() {
            out.push(Line::from(format!("   {}", facts.join("   "))));
        }
    }
    if let Some(d) = e.default {
        out.push(dim_line(format!("default {}", params::fmt_g(d, 6))));
    }
    if expanded && !e.long.is_empty() {
        out.push(Ln::new().raw(" ").dim(e.long.clone()).line());
    }
    out
}

/// One KEY PARAMETERS cell: `NAME  value meaning`.
fn key_cell(ctx: &Ctx, name: &str, col_width: usize) -> Line<'static> {
    let label = format!("{name:<18} ");
    let Some(p) = ctx.st.parameters.get(name) else {
        return Ln::new().raw(label).dim("--").line();
    };
    let number = params::format_value(p);
    let meaning = ctx.app.meta.get(name).map(|e| e.meaning(p.value)).unwrap_or_default();
    let room = col_width.saturating_sub(label.len() + number.len() + 1);
    let meaning: String = meaning.chars().take(room).collect();
    Ln::new().raw(label).fg(number, GREEN).raw(" ").dim(meaning).line()
}

fn key_panel(ctx: &Ctx) -> Vec<Line<'static>> {
    let col_width = ctx.width.saturating_sub(4) / 2;
    if col_width < 34 {
        return Vec::new();
    }
    let columns: Vec<Vec<Line<'static>>> = KEY_PARAMETER_COLUMNS
        .iter()
        .map(|groups| {
            let mut cells = Vec::new();
            for (heading, names) in groups.iter() {
                if !cells.is_empty() {
                    cells.push(blank());
                }
                cells.push(Line::styled(heading.to_string(), Style::new().fg(CYAN).add_modifier(Modifier::BOLD)));
                cells.extend(names.iter().map(|n| key_cell(ctx, n, col_width)));
            }
            cells
        })
        .collect();

    let rows = columns.iter().map(Vec::len).max().unwrap_or(0);
    let mut out = vec![Ln::new().dim(" KEY PARAMETERS").line()];
    for r in 0..rows {
        let mut spans = vec![Span::raw("  ")];
        for (ci, col) in columns.iter().enumerate() {
            let cell = col.get(r).cloned().unwrap_or_default();
            let used = cell.width();
            let cell_spans: Vec<Span> = cell.spans;
            spans.extend(cell_spans);
            if ci + 1 < columns.len() {
                spans.push(Span::raw(" ".repeat(col_width.saturating_sub(used) + 2)));
            }
        }
        out.push(Line::from(spans));
    }
    out.push(blank());
    out
}

pub fn parameters(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let session = &ctx.app.session;
    let visible = params::visible_parameters(st, session.param_changed_only);
    let query = &session.search_query;

    let status = if st.parameters_complete {
        Ln::new().fg("COMPLETE", GREEN)
    } else if st.parameters_requested_at > 0.0 {
        Ln::new().fg(format!("LOADING ({:.1}s)", now() - st.parameters_requested_at), YELLOW)
    } else if !st.parameters.is_empty() {
        Ln::new().fg("PARTIAL", YELLOW)
    } else {
        Ln::new().dim("NOT LOADED")
    };
    let expected = if st.parameter_count > 0 { st.parameter_count.to_string() } else { "--".into() };
    let view = if session.param_changed_only { "CHANGED" } else { "ALL" };

    let mut lines = vec![
        Ln::new()
            .raw(" Status: ")
            .spans(status.0)
            .raw(format!("    Received: {}/{expected}    View: ", st.parameters.len()))
            .bold(view)
            .line(),
    ];
    match &st.parameter_set_pending {
        Some((name, _, _)) => lines.push(Ln::new().fg(format!(" Pending PARAM_SET: {name}"), YELLOW).line()),
        None => lines.push(blank()),
    }

    let cursor = session.param_index.min(visible.len().saturating_sub(1));
    let editing = session.param_edit.as_ref();
    let detail = visible
        .get(cursor)
        .map(|p| detail_lines(p, ctx.app.meta.get(&p.name), editing.is_some()))
        .unwrap_or_default();

    let edit_rows = if editing.is_some() { 2 } else { 0 };
    let changed_note = if session.param_changed_only { 2 } else { 0 };
    let fixed = lines.len() + 1 + detail.len() + edit_rows + changed_note;
    let mut panel = key_panel(ctx);
    if !panel.is_empty() && ctx.height.saturating_sub(fixed + panel.len()) < 6 {
        panel.clear();
    }
    let page_size = ctx.height.saturating_sub(fixed + panel.len()).clamp(3, 40);
    ctx.page.set(page_size);
    lines.extend(panel);

    if visible.is_empty() {
        lines.push(Ln::new().fg(" No parameters in the current view.", RED).line());
    } else {
        let start = (cursor / page_size) * page_size;
        let name_w = 34usize;
        let mean_room = ctx.width.saturating_sub(4 + name_w + 1 + 12 + 1 + 8 + 9);
        for (i, p) in visible.iter().enumerate().skip(start).take(page_size) {
            let selected = i == cursor;
            let base = if selected { selected_style() } else { Style::new() };
            let status = if p.pending {
                Ln::new().fg(" PENDING", YELLOW)
            } else {
                match p.changed_from_default() {
                    Some(true) => Ln::new().fg(" CHANGED", YELLOW),
                    None if p.changed_from_startup() => Ln::new().fg(" CHANGED*", YELLOW),
                    _ => Ln::new(),
                }
            };
            let name: String = p.name.chars().take(name_w).collect();
            let meaning: String = ctx
                .app
                .meta
                .get(&p.name)
                .map(|e| e.meaning(p.value))
                .unwrap_or_default()
                .chars()
                .take(mean_room)
                .collect();
            let value: String = params::format_value(p).chars().take(12).collect();
            lines.push(
                Ln::new()
                    .st(if selected { "  > " } else { "    " }, base)
                    .spans(highlight(&format!("{name:<name_w$}"), query, base))
                    .st(format!(" {value:>12} {:<8}", config_type_name(p.param_type)), base)
                    .st(format!("{meaning:<mean_room$}"), if selected { base } else { Style::new().fg(CYAN) })
                    .spans(status.0)
                    .line(),
            );
        }
    }

    lines.push(blank());
    lines.extend(detail);

    if let Some((name, buffer)) = editing {
        lines.push(Ln::new().bold(format!(" Edit {name}: ")).raw(format!("{buffer}█")).line());
        lines.push(Ln::new().dim(" ENTER = apply    ESC = cancel").line());
    }
    if session.param_changed_only {
        lines.push(blank());
        if ctx.app.settings.param_defaults_file.is_some() {
            lines.push(Ln::new().dim(" CHANGED = different from the firmware's default values.").line());
        } else {
            lines.push(Ln::new().dim(" * CHANGED = different from the value first loaded by this console.").line());
        }
    }
    lines
}

fn config_type_name(t: u8) -> String {
    crate::config::param_type_name(t)
}

pub fn event_log(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let query = &ctx.app.session.search_query;
    let mut lines = vec![
        Ln::new()
            .raw(format!(" {} events   ", st.events.len()))
            .fg(format!("warnings {}", st.warning_count), YELLOW)
            .raw("   ")
            .fg(format!("errors {}", st.error_count), RED)
            .raw("   ")
            .st(format!("failsafes {}", st.failsafe_count), Style::new().fg(RED).add_modifier(Modifier::BOLD))
            .line(),
        blank(),
    ];
    let page = ctx.height.saturating_sub(lines.len()).max(1);
    ctx.page.set(page);
    let start = ctx.app.session.log_scroll.min(st.events.len().saturating_sub(page));
    for e in st.events.iter().skip(start).take(page) {
        let style = level_color(e.level);
        lines.push(
            Ln::new()
                .dim(format!(" {} ", e.time))
                .st(format!("{:<8} ", e.level.as_str()), style)
                .spans(highlight(&e.message, query, style))
                .line(),
        );
    }
    lines
}

pub fn flight_logs(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let session = &ctx.app.session;
    let query = &session.search_query;

    let list_status = if st.flight_log_list_requested_at > 0.0 {
        Ln::new().fg(format!("LOADING ({:.1}s)", now() - st.flight_log_list_requested_at), YELLOW)
    } else if st.flight_log_list_complete {
        Ln::new().fg("COMPLETE", GREEN)
    } else {
        Ln::new().dim("NOT LOADED")
    };
    let mut lines = vec![
        Ln::new()
            .raw(" List: ")
            .spans(list_status.0)
            .raw(format!("    Logs: {}    Save to: {}", st.flight_logs.len(), ctx.app.settings.log_dir))
            .line(),
    ];

    // Download status.
    let dl = if st.dl_active {
        let pct = if st.dl_size > 0 { st.dl_received as f64 * 100.0 / st.dl_size as f64 } else { 0.0 };
        Ln::new().fg(
            format!(
                " Downloading {:06}: {} / {} ({pct:.1}%)  {:.1} KiB/s  {}",
                st.dl_id,
                crate::state::size_string(st.dl_received),
                crate::state::size_string(st.dl_size as u64),
                st.dl_speed / 1024.0,
                st.dl_status
            ),
            YELLOW,
        )
    } else {
        match st.dl_status.as_str() {
            "COMPLETE" => Ln::new().fg(format!(" Saved: {}", st.dl_path), GREEN),
            "ERROR" => Ln::new().fg(format!(" Download failed: {}", st.dl_error), RED),
            "CANCELLED" => Ln::new().fg(" Download cancelled", YELLOW),
            _ => Ln::new(),
        }
    };
    lines.push(dl.line());
    if st.dl_active {
        let width = 50usize;
        let filled = if st.dl_size > 0 {
            ((st.dl_received as f64 / st.dl_size as f64).min(1.0) * width as f64) as usize
        } else {
            0
        };
        lines.push(Ln::new().raw(" [").fg("█".repeat(filled), GREEN).dim("░".repeat(width - filled)).raw("]").line());
    }
    lines.push(blank());
    lines.push(Ln::new().dim(format!("    {:<8}{:<26}{:>12}", "ID", "DATE", "SIZE")).line());

    // Upload / EKF-check job status goes below the list, rows reserved.
    let mut jobs = super::hostui::job_panel(ctx, crate::jobs::JobKind::UlogUpload);
    jobs.extend(super::hostui::job_panel(ctx, crate::jobs::JobKind::EclEkf));

    if st.flight_logs.is_empty() {
        lines.push(dim_line(if st.flight_log_list_requested_at > 0.0 {
            "waiting for LOG_ENTRY..."
        } else {
            "no logs - [r] to request the list"
        }));
        lines.extend(jobs);
        return lines;
    }
    let count = st.flight_logs.len();
    let cursor = session.flight_log_index.min(count - 1);
    let height = ctx.height.saturating_sub(lines.len() + jobs.len()).max(1);
    let start = window_start(cursor, count, height);
    for (i, e) in st.flight_logs.values().enumerate().skip(start).take(height) {
        let selected = i == cursor;
        let style = if selected { selected_style() } else { Style::new() };
        let row = format!("{:06}  {:<26}{:>12}", e.id, e.time_string(), e.size_string());
        lines.push(
            Ln::new()
                .st(if selected { "  > " } else { "    " }, style)
                .spans(highlight(&row, query, style))
                .line(),
        );
    }
    lines.extend(jobs);
    lines
}

pub fn shell(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let session = &ctx.app.session;
    let status = if st.shell_active { Ln::new().fg("OPEN", GREEN) } else { Ln::new().fg("CLOSED", RED) };
    let mut lines = vec![Ln::new().raw(" NSH: ").spans(status.0).line()];

    // NSH's unfinished line (normally the prompt) followed by the local
    // input being edited, with the cursor in it.
    let mut prompt: String = st.shell_line.chars.iter().collect();
    if st.shell_echo.as_ref().is_some_and(|f| f.hides_partial(&prompt)) {
        prompt.clear();
    }
    let input = &session.shell_input;
    let mut chars = input.text.clone();
    chars.push(' ');
    let before: String = chars[..input.cursor].iter().collect();
    let at: String = chars[input.cursor].to_string();
    let after: String = chars[input.cursor + 1..].iter().collect();
    let edit_line = Ln::new()
        .raw(format!(" {prompt}{before}"))
        .st(at, Style::new().add_modifier(Modifier::REVERSED))
        .raw(after)
        .line();

    let mut all: Vec<Line<'static>> = st.shell_lines.iter().map(|l| Line::from(format!(" {l}"))).collect();
    all.push(edit_line);

    let height = ctx.height.saturating_sub(lines.len()).max(1);
    let max_start = all.len().saturating_sub(height);
    let start = if session.shell_follow {
        max_start
    } else {
        (max_start as isize + session.shell_scroll).clamp(0, max_start as isize) as usize
    };
    lines.extend(all.into_iter().skip(start).take(height));
    lines
}
