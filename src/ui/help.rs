//! The `?` popup: the keys of the screen underneath, then the global ones.

use ratatui::Frame;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Clear, Paragraph};

use super::GREEN;
use crate::app::Screen;

type Keys = &'static [(&'static str, &'static str)];

const FLIGHT: Keys = &[
    ("a / d", "arm / disarm"),
    ("T / L / R / h", "takeoff / land / RTL / hold"),
    ("F", "offboard on / off"),
    ("K", "kill (flight termination)"),
    ("H / G / E", "set home / geofence action / EKF reset"),
];

const GLOBAL: Keys = &[
    (":", "command line (:help lists the commands)"),
    ("?", "this help"),
    ("A", "acknowledge the alert banner"),
    ("TAB / ESC", "focus the panel list"),
    ("j k / Ctrl-D Ctrl-U", "scroll / page"),
    ("q / Ctrl-C", "quit"),
];

fn screen_keys(screen: Screen) -> (Keys, bool) {
    // (keys, whether the flight commands work here too)
    let keys: Keys = match screen {
        Screen::Dashboard => &[
            ("m", "flight modes"),
            ("n", "mission map"),
            ("p", "parameters"),
            ("c / r", "control setpoints / RC, wind, outputs"),
            ("s", "sensor calibration"),
            ("l", "flight logs"),
            ("g", "event log"),
            ("t", "NSH shell"),
            ("u", "USB / network"),
            ("f", "flash firmware"),
            ("y", "preflight go / no-go"),
        ],
        Screen::Map => &[
            ("g", "goto: rel  10 0 -2 · local  l 5 -3 -10 · global  g lat lon alt"),
            ("x", "arm keyboard jog"),
            ("  k j / a d", "jog: forward, back / left, right"),
            ("  w s / h l", "jog: up, down / yaw"),
            ("  [ / ]", "jog: halve / double the step"),
            ("o / O", "load .kml or QGC .plan / upload its fence"),
            ("M", "upload the .plan mission to PX4"),
            ("W / C", "fly KML waypoints / lawnmower coverage"),
            ("V / v / B", "LiDAR overlay / topic / robot vs world frame"),
            ("N", "ROS planned-path overlay"),
            ("P", "publish position as NavSatFix"),
            ("i", "save a satellite snapshot"),
            ("arrows", "pan"),
            ("+ / - / 0", "zoom in / out / reset"),
            ("u / f", "follow vehicle / fit to KML"),
            ("t / c", "trail on/off / clear trail"),
            ("r", "re-request streams"),
            ("n", "back"),
        ],
        Screen::ModeSelect => &[("↑↓ / j k", "select"), ("ENTER", "set mode"), ("r", "re-request modes"), ("/ n N", "search"), ("m", "back")],
        Screen::Parameters => &[
            ("ENTER", "edit the selected parameter"),
            ("v", "all / changed only"),
            ("r", "reload from PX4"),
            ("s / l", "save to / load from a file"),
            ("D", "diff a saved file against the vehicle"),
            ("b", "reboot PX4"),
            ("/ n N", "search"),
            ("p", "back"),
        ],
        Screen::Log => &[("↑↓ PGUP PGDN", "scroll"), ("c", "clear"), ("/ n N", "search"), ("g", "back")],
        Screen::FlightLogs => &[
            ("ENTER", "download the selected log"),
            ("u", "upload the last download to flight review"),
            ("a", "ecl_ekf check of the last download"),
            ("r", "refresh the list"),
            ("x / ESC", "cancel a transfer"),
            ("/ n N", "search"),
            ("l", "back"),
        ],
        Screen::Shell => &[
            ("ENTER", "run"),
            ("↑↓", "history"),
            ("PGUP / PGDN", "scroll"),
            ("Ctrl-C", "interrupt"),
            ("Ctrl-L", "clear"),
            ("ESC", "back to the panel list (shell stays open)"),
        ],
        Screen::Host => &[("i", "speed test"), ("u", "back")],
        Screen::Control => &[("r", "re-request streams"), ("c", "back")],
        Screen::Calibration => &[
            ("g / a / l", "gyro / accel / level horizon"),
            ("c / b", "compass / baro"),
            ("x / ESC", "cancel a running calibration"),
            ("r", "re-request streams"),
            ("s", "back"),
        ],
        Screen::Firmware => &[("↑↓", "firmware file"), ("←→", "serial port"), ("ENTER", "flash"), ("r", "refresh"), ("f", "back")],
        Screen::Preflight => &[("y", "back")],
        Screen::About => &[],
    };
    (keys, matches!(screen, Screen::Dashboard | Screen::Map))
}

fn section(lines: &mut Vec<Line<'static>>, title: &str, keys: Keys) {
    if keys.is_empty() {
        return;
    }
    lines.push(Line::styled(format!(" {title}"), Style::new().fg(GREEN).add_modifier(Modifier::BOLD)));
    for (k, what) in keys {
        lines.push(Line::from(vec![
            Span::styled(format!("   {k:<22}"), Style::new().add_modifier(Modifier::BOLD)),
            Span::raw(what.to_string()),
        ]));
    }
    lines.push(Line::raw(""));
}

pub fn lines(screen: Screen) -> Vec<Line<'static>> {
    let (keys, flight) = screen_keys(screen);
    let mut lines = Vec::new();
    section(&mut lines, screen.title(), keys);
    if flight {
        section(&mut lines, "FLIGHT (type YES to confirm)", FLIGHT);
    }
    section(&mut lines, "EVERYWHERE", GLOBAL);
    lines
}

/// Centered over `area`; `scroll` is clamped here.
pub fn draw(frame: &mut Frame, area: Rect, screen: Screen, scroll: &mut usize) {
    let lines = lines(screen);
    let width = 84.min(area.width.saturating_sub(4));
    let height = (lines.len() as u16 + 2).min(area.height.saturating_sub(2));
    let [_, mid, _] = Layout::horizontal([Constraint::Fill(1), Constraint::Length(width), Constraint::Fill(1)]).areas(area);
    let [_, popup, _] = Layout::vertical([Constraint::Fill(1), Constraint::Length(height), Constraint::Fill(1)]).areas(mid);
    let visible = height.saturating_sub(2) as usize;
    *scroll = (*scroll).min(lines.len().saturating_sub(visible));
    let more = if lines.len() > visible { " j/k scroll ·" } else { "" };
    frame.render_widget(Clear, popup);
    frame.render_widget(
        Paragraph::new(lines).scroll((*scroll as u16, 0)).block(
            Block::bordered()
                .border_type(BorderType::Rounded)
                .border_style(Style::new().fg(GREEN))
                .title(" KEYS ")
                .title_bottom(Line::from(format!("{more} any key closes ")).right_aligned()),
        ),
        popup,
    );
}
