//! The file-browser popup: listing on the left, preview on the right.

use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Modifier, Style};
use ratatui::text::Line;
use ratatui::widgets::{Block, BorderType, Clear, Paragraph};

use super::{BLUE, GREEN, Ln, OVERLAY0, RED, SURFACE1, YELLOW, base_style, window_start};
use crate::picker::{FilePicker, human_size};

/// Centered over `area`; returns the listing's visible rows (PGUP/PGDN).
pub fn draw(frame: &mut Frame, area: Rect, p: &FilePicker) -> usize {
    let width = 110.min(area.width.saturating_sub(4));
    let height = 30.min(area.height.saturating_sub(2));
    let [_, mid, _] = Layout::horizontal([Constraint::Fill(1), Constraint::Length(width), Constraint::Fill(1)]).areas(area);
    let [_, popup, _] = Layout::vertical([Constraint::Fill(1), Constraint::Length(height), Constraint::Fill(1)]).areas(mid);
    frame.render_widget(Clear, popup);

    let keys = if p.filtering {
        " type to filter · ENTER done · ESC clear ".to_string()
    } else {
        format!(
            " hjkl move · / filter · . hidden · a {} · ~ home · TAB type path · ESC cancel ",
            if p.all_files { "matching only" } else { "all files" }
        )
    };
    let block = Block::bordered()
        .border_type(BorderType::Rounded)
        .border_style(Style::new().fg(GREEN))
        .title(format!(" {} ", p.title))
        .title_bottom(Line::from(keys).right_aligned())
        .style(base_style());
    let inner = block.inner(popup);
    frame.render_widget(block, popup);

    let [header, body] = Layout::vertical([Constraint::Length(2), Constraint::Min(1)]).areas(inner);
    let mut head = Ln::new().fg(format!(" {}/", p.dir.display().to_string().trim_end_matches('/')), BLUE);
    if p.filtering || !p.filter.is_empty() {
        head = head.raw("   ").fg(format!("/{}{}", p.filter, if p.filtering { "█" } else { "" }), YELLOW);
    }
    let sub = if !p.error.is_empty() {
        Ln::new().raw(" ").fg(p.error.clone(), RED)
    } else if !p.all_files && !p.exts.is_empty() {
        Ln::new().raw(" ").dim(format!("showing {}  ({} entries)", p.exts.iter().map(|e| format!("*.{e}")).collect::<Vec<_>>().join(" "), p.entries.len()))
    } else {
        Ln::new().raw(" ").dim(format!("all files  ({} entries)", p.entries.len()))
    };
    frame.render_widget(Paragraph::new(vec![head.line(), sub.line()]), header);

    let [list_area, gap, preview_area] =
        Layout::horizontal([Constraint::Percentage(45), Constraint::Length(1), Constraint::Fill(1)]).areas(body);
    let rows = list_area.height as usize;
    let lw = list_area.width as usize;
    let start = window_start(p.index, p.entries.len(), rows);
    let mut lines = Vec::new();
    for (i, e) in p.entries.iter().enumerate().skip(start).take(rows) {
        let label = if e.dir { format!(" {}/", e.name) } else { format!(" {}", e.name) };
        let size = if e.dir {
            String::new()
        } else {
            p.dir.join(&e.name).metadata().map(|m| human_size(m.len())).unwrap_or_default()
        };
        let pad = lw.saturating_sub(label.chars().count() + size.len() + 1);
        let text: String = format!("{label}{}{size} ", " ".repeat(pad)).chars().take(lw).collect();
        let mut style = if e.dir { Style::new().fg(BLUE).add_modifier(Modifier::BOLD) } else { Style::new() };
        if i == p.index {
            style = style.bg(SURFACE1).add_modifier(Modifier::BOLD);
        }
        lines.push(Ln::new().st(text, style).line());
    }
    if p.entries.is_empty() {
        lines.push(Ln::new().dim(if p.filter.is_empty() { " (nothing here - [a] all files, [.] hidden)" } else { " (no match)" }).line());
    }
    frame.render_widget(Paragraph::new(lines), list_area);

    let sep: Vec<Line> = (0..gap.height).map(|_| Ln::new().fg("│", OVERLAY0).line()).collect();
    frame.render_widget(Paragraph::new(sep), gap);
    let pw = preview_area.width as usize;
    let preview: Vec<Line> = p
        .preview(preview_area.height as usize)
        .into_iter()
        .map(|l| Ln::new().dim(format!(" {}", l.chars().take(pw.saturating_sub(1)).collect::<String>())).line())
        .collect();
    frame.render_widget(Paragraph::new(preview), preview_area);
    rows
}
