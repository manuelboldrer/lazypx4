//! Rendering: the lazygit-style frame (sidebar + main panel + key bar +
//! prompt) and the per-screen content builders in the submodules.
//!
//! Each screen builds a `Vec<Line>` while holding the state lock once per
//! frame; page-style screens are scrolled by the frame, list screens window
//! themselves around their cursor.

mod dashboard;
mod hostui;
mod lists;
mod map;
mod other;
pub mod sensors;

use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Paragraph};

use crate::app::{App, NAV_ITEMS, Screen};
use crate::config::*;
use crate::state::{self, State};

pub const GREEN: Color = Color::Green;
pub const YELLOW: Color = Color::Yellow;
pub const RED: Color = Color::Red;
pub const CYAN: Color = Color::Cyan;

/// Small span builder so screens read like the Python f-strings they port.
#[derive(Default)]
pub struct Ln(Vec<Span<'static>>);

impl Ln {
    pub fn new() -> Self {
        Ln(Vec::new())
    }
    pub fn raw(mut self, s: impl Into<String>) -> Self {
        self.0.push(Span::raw(s.into()));
        self
    }
    pub fn st(mut self, s: impl Into<String>, style: Style) -> Self {
        self.0.push(Span::styled(s.into(), style));
        self
    }
    pub fn fg(self, s: impl Into<String>, color: Color) -> Self {
        self.st(s, Style::new().fg(color))
    }
    pub fn dim(self, s: impl Into<String>) -> Self {
        self.st(s, Style::new().add_modifier(Modifier::DIM))
    }
    pub fn bold(self, s: impl Into<String>) -> Self {
        self.st(s, Style::new().add_modifier(Modifier::BOLD))
    }
    /// `[k]` hotkey hint followed by its label.
    pub fn key(self, k: &str, label: &str) -> Self {
        self.bold(format!("[{k}]")).raw(format!("{label}  "))
    }
    pub fn spans(mut self, spans: Vec<Span<'static>>) -> Self {
        self.0.extend(spans);
        self
    }
    pub fn line(self) -> Line<'static> {
        Line::from(self.0)
    }
}

impl From<Ln> for Line<'static> {
    fn from(l: Ln) -> Self {
        l.line()
    }
}

pub fn blank() -> Line<'static> {
    Line::default()
}

pub fn dim_line(text: impl Into<String>) -> Line<'static> {
    Ln::new().raw("   ").dim(text).line()
}

/// "── TITLE ─── note" section header.
pub fn section(title: &str, note: &str) -> Line<'static> {
    let mut l = Ln::new()
        .fg("── ", CYAN)
        .st(title.to_string(), Style::new().fg(CYAN).add_modifier(Modifier::BOLD))
        .fg(" ──", CYAN);
    if !note.is_empty() {
        l = l.raw(" ").dim(note.to_string());
    }
    l.line()
}

/// Green/yellow/red for `value` against a (good, ok) threshold pair.
pub fn graded(value: f64, good: f64, ok: f64, higher_is_better: bool) -> Color {
    let (g, o) = if higher_is_better {
        (value >= good, value >= ok)
    } else {
        (value <= good, value <= ok)
    };
    if g {
        GREEN
    } else if o {
        YELLOW
    } else {
        RED
    }
}

pub fn level_color(level: state::Level) -> Style {
    match level {
        state::Level::Info => Style::new(),
        state::Level::Command => Style::new().fg(CYAN),
        state::Level::Warn => Style::new().fg(YELLOW),
        state::Level::Error => Style::new().fg(RED),
        state::Level::Failsafe => Style::new().fg(RED).add_modifier(Modifier::BOLD),
    }
}

pub fn armed_span(armed: bool) -> Span<'static> {
    if armed {
        Span::styled(" ARMED ", Style::new().bg(GREEN).fg(Color::Black).add_modifier(Modifier::BOLD))
    } else {
        Span::styled(" DISARMED ", Style::new().add_modifier(Modifier::DIM))
    }
}

pub fn age_text(last: f64) -> String {
    if last > 0.0 {
        format!("{:.1}s ago", state::now() - last)
    } else {
        "never".into()
    }
}

/// Split `text` into spans, reversing every case-insensitive match of
/// `query` (reverse video composes with whatever colour the row has).
pub fn highlight(text: &str, query: &str, base: Style) -> Vec<Span<'static>> {
    if query.is_empty() {
        return vec![Span::styled(text.to_string(), base)];
    }
    let lower = text.to_lowercase();
    let needle = query.to_lowercase();
    // Byte offsets only line up when lowercasing kept the length.
    if lower.len() != text.len() {
        return vec![Span::styled(text.to_string(), base)];
    }
    let mut out = Vec::new();
    let mut i = 0;
    while let Some(j) = lower[i..].find(&needle).map(|j| j + i) {
        if j > i {
            out.push(Span::styled(text[i..j].to_string(), base));
        }
        out.push(Span::styled(
            text[j..j + needle.len()].to_string(),
            base.add_modifier(Modifier::REVERSED),
        ));
        i = j + needle.len();
    }
    if i < text.len() {
        out.push(Span::styled(text[i..].to_string(), base));
    }
    out
}

/// Keep a list cursor visible: the first row to draw for a window of
/// `height` rows over `total`, cursor roughly centred.
pub fn window_start(cursor: usize, total: usize, height: usize) -> usize {
    if total <= height || height == 0 {
        return 0;
    }
    cursor.saturating_sub(height / 2).min(total - height)
}

pub struct Ctx<'a> {
    pub st: &'a State,
    pub app: &'a App,
    /// Rows / columns of the main panel's inside.
    pub height: usize,
    pub width: usize,
    /// Rows per page a list screen settled on this frame (PGUP/PGDN step).
    pub page: std::cell::Cell<usize>,
}

pub fn draw(frame: &mut Frame, app: &mut App) {
    let area = frame.area();
    if area.width < MIN_TERMINAL_COLS || area.height < MIN_TERMINAL_ROWS {
        frame.render_widget(
            Paragraph::new(vec![
                Line::from(format!("Terminal too small ({}x{}).", area.width, area.height)),
                Line::from(format!("Resize to at least {MIN_TERMINAL_COLS}x{MIN_TERMINAL_ROWS}.")),
                Line::from("Tip: shrink your terminal's font (Ctrl -) to fit more columns/rows."),
            ]),
            area,
        );
        return;
    }

    let prompt_rows = if app.session.confirm.is_some() || app.session.input.is_some() {
        2
    } else if app.session.search_active || !app.session.search_query.is_empty() && app.session.screen_is_searchable() {
        1
    } else {
        0
    };
    let [body, keybar, prompt] = Layout::vertical([
        Constraint::Min(1),
        Constraint::Length(1),
        Constraint::Length(prompt_rows),
    ])
    .areas(area);

    // Narrow terminals drop the sidebar, except while it has focus - TAB
    // must never move the keyboard into something you can't see.
    let use_sidebar = area.width >= 100 || app.session.sidebar_focused;
    let (sidebar, main) = if use_sidebar {
        let [s, m] = Layout::horizontal([Constraint::Length(24), Constraint::Min(1)]).areas(body);
        (Some(s), m)
    } else {
        (None, body)
    };

    let st = state::lock(&app.shared);

    if let Some(sidebar) = sidebar {
        draw_sidebar(frame, app, &st, sidebar);
    }

    let focused = !app.session.sidebar_focused;
    let border = if focused { Style::new().fg(GREEN) } else { Style::new().add_modifier(Modifier::DIM) };
    let block = Block::bordered()
        .border_type(BorderType::Rounded)
        .border_style(border)
        .title(Line::from(format!(" {} ", app.session.screen.title())).bold());
    let inner = block.inner(main);
    frame.render_widget(block, main);

    let screen = app.session.screen;
    let (lines, footer, page) = {
        let ctx = Ctx {
            st: &st,
            app,
            height: inner.height as usize,
            width: inner.width as usize,
            page: std::cell::Cell::new(0),
        };
        let (lines, footer) = build_screen(&ctx, screen);
        (lines, footer, ctx.page.get())
    };

    let mut scroll_note = String::new();
    if screen.scrolls_as_page() {
        if app.session.main_scroll_screen != screen {
            app.session.main_scroll_screen = screen;
            app.session.main_scroll = 0;
        }
        let max_scroll = lines.len().saturating_sub(inner.height as usize);
        app.session.main_scroll = app.session.main_scroll.min(max_scroll);
        if max_scroll > 0 {
            scroll_note = format!("  ·  j/k scroll {}/{}", app.session.main_scroll, max_scroll);
        }
        frame.render_widget(Paragraph::new(lines).scroll((app.session.main_scroll as u16, 0)), inner);
    } else {
        frame.render_widget(Paragraph::new(lines), inner);
    }
    draw_keybar(frame, keybar, &footer, &scroll_note);
    draw_prompt(frame, app, prompt);
    drop(st);

    match screen {
        Screen::Log => app.session.log_page = page.max(1),
        Screen::Parameters => app.session.param_page = page.max(1),
        _ => {}
    }
}

impl crate::app::Session {
    pub fn screen_is_searchable(&self) -> bool {
        matches!(self.screen, Screen::ModeSelect | Screen::Log | Screen::FlightLogs | Screen::Parameters)
    }
}

fn build_screen(ctx: &Ctx, screen: Screen) -> (Vec<Line<'static>>, String) {
    match screen {
        Screen::Dashboard => (dashboard::draw(ctx), "[a]rm [d]isarm [T]akeoff [L]and [R]TL [h]old [m]ode · TAB panels · q quit".into()),
        Screen::ModeSelect => (lists::modes(ctx), "↑↓/jk select · ENTER set mode · [r] re-request · / search · [m] back".into()),
        Screen::Parameters => (lists::parameters(ctx), "ENTER edit · [v] ALL/CHANGED · [r] refresh · [b] reboot · / search · [p] back".into()),
        Screen::Log => (lists::event_log(ctx), "↑↓ scroll · PGUP/PGDN page · [c] clear · / search · [g] back".into()),
        Screen::FlightLogs => (
            lists::flight_logs(ctx),
            "ENTER download · [u] upload last to web · [a] EKF check · [r] refresh · ESC/x cancel · [l] back".into(),
        ),
        Screen::Shell => (lists::shell(ctx), "ENTER run · ↑↓ history · ←→ edit · PGUP/PGDN scroll · CTRL-C interrupt · CTRL-L clear · ESC panels".into()),
        Screen::Control => (other::control(ctx), "[r] re-request streams · [c] back · ESC panels".into()),
        Screen::Calibration => (other::calibration(ctx), "[g]yro [a]ccel [l]evel [c]ompass [b]aro · [r] re-request · [s] back".into()),
        Screen::About => (other::about(), "[?] back · ESC panels".into()),
        Screen::Map => (map::draw(ctx), "j/k scroll · keys listed below the map · [n] back · ESC panels".into()),
        Screen::PointCloud => (
            sensors::pointcloud(ctx),
            if ctx.app.session.cloud_view == sensors::CloudView::Free {
                "[+]/[-] zoom · [0] reset · [hjkl] rotate · [c] fixed views · [t] topic · [v] back".into()
            } else {
                "[+]/[-] zoom · [0] reset · [1] top [2] front [3] 45° · [c] free camera · [t] topic · [v] back".into()
            },
        ),
        Screen::Camera => (sensors::camera(ctx), "[1] topic 1 · [2] topic 2 · [b] low-bandwidth · [w] back · ESC panels".into()),
        Screen::Host => (hostui::host(ctx), "[i] speed test · [u] back · ESC panels".into()),
        Screen::Firmware => (
            hostui::firmware(ctx),
            "↑↓ firmware · ←→ port · ENTER flash · [r] refresh · [f] back · ESC panels (cancels a flash)".into(),
        ),
    }
}

fn draw_sidebar(frame: &mut Frame, app: &App, st: &State, area: Rect) {
    let [status_area, nav_area] = Layout::vertical([Constraint::Length(7), Constraint::Min(3)]).areas(area);

    // At-a-glance vehicle status, visible from every screen.
    let link = if st.connected {
        Span::styled("CONNECTED", Style::new().fg(GREEN))
    } else if st.vehicle_locked {
        Span::styled("LOST", Style::new().fg(RED).add_modifier(Modifier::BOLD))
    } else {
        Span::styled("WAITING", Style::new().fg(YELLOW))
    };
    let battery = if st.battery < 0.0 {
        Span::styled("--", Style::new().add_modifier(Modifier::DIM))
    } else {
        let color = if st.battery <= BATTERY_CRITICAL {
            RED
        } else if st.battery <= BATTERY_LOW {
            YELLOW
        } else {
            GREEN
        };
        Span::styled(format!("{:.0}%", st.battery), Style::new().fg(color))
    };
    let (fix_color, fix_name) = dashboard::gps_fix_display(st.gps_fix);
    let status = vec![
        Ln::new().raw(" ").spans(vec![link]).raw(format!(" sys {}", st.target_system)).line(),
        Ln::new().raw(" ").spans(vec![armed_span(st.armed)]).line(),
        Ln::new().raw(" ").bold(st.mode.clone()).line(),
        Ln::new().raw(" BAT ").spans(vec![battery]).line(),
        Ln::new().raw(" GPS ").fg(fix_name, fix_color).raw(format!(" {}", st.gps_sats)).line(),
    ];
    frame.render_widget(
        Paragraph::new(status).block(
            Block::bordered()
                .border_type(BorderType::Rounded)
                .border_style(Style::new().add_modifier(Modifier::DIM))
                .title(" STATUS "),
        ),
        status_area,
    );

    let focused = app.session.sidebar_focused;
    let rows: Vec<Line> = NAV_ITEMS
        .iter()
        .enumerate()
        .map(|(i, (screen, label, hotkey))| {
            let key = hotkey.map(|c| format!("[{c}]")).unwrap_or_else(|| "   ".into());
            let text = format!(" {key} {label:<16}");
            let current = *screen == app.session.screen;
            let selected = focused && i == app.session.nav_index;
            let style = if selected {
                Style::new().bg(GREEN).fg(Color::Black).add_modifier(Modifier::BOLD)
            } else if current {
                Style::new().fg(GREEN).add_modifier(Modifier::BOLD)
            } else {
                Style::new()
            };
            Line::styled(text, style)
        })
        .collect();
    let border = if focused { Style::new().fg(GREEN) } else { Style::new().add_modifier(Modifier::DIM) };
    frame.render_widget(
        Paragraph::new(rows).block(
            Block::bordered().border_type(BorderType::Rounded).border_style(border).title(" PANELS "),
        ),
        nav_area,
    );
}

fn draw_keybar(frame: &mut Frame, area: Rect, footer: &str, scroll_note: &str) {
    frame.render_widget(
        Paragraph::new(Ln::new().raw(" ").dim(format!("{footer}{scroll_note}")).line()),
        area,
    );
}

fn draw_prompt(frame: &mut Frame, app: &App, area: Rect) {
    let bar = Style::new().bg(Color::Blue).fg(Color::White).add_modifier(Modifier::BOLD);
    let lines = if let Some(c) = &app.session.confirm {
        vec![
            Ln::new().st(" CONFIRM ", bar).raw("  ").bold(c.text.clone()).line(),
            Ln::new()
                .st(" YES ?   ", bar)
                .raw("  ")
                .bold(format!("{}█", c.buffer))
                .raw("   ")
                .dim("ENTER confirm · ESC cancel")
                .line(),
        ]
    } else if let Some(i) = &app.session.input {
        vec![
            Ln::new().st(" INPUT   ", bar).raw("  ").raw(i.prompt.clone()).line(),
            Ln::new()
                .st("   >     ", bar)
                .raw(format!("  {}█   ", i.buffer))
                .dim("ENTER submit · ESC cancel")
                .line(),
        ]
    } else if app.session.search_active || !app.session.search_query.is_empty() {
        let cursor = if app.session.search_active { "█" } else { "" };
        vec![
            Ln::new()
                .st(" SEARCH  ", bar)
                .raw(format!("  /{}{cursor}   ", app.session.search_query))
                .dim(if app.session.search_active {
                    "ENTER done · ESC clear"
                } else {
                    "n / N next / previous match · / edit"
                })
                .line(),
        ]
    } else {
        return;
    };
    frame.render_widget(Paragraph::new(lines), area);
}
