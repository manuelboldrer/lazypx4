//! MAVLink shell (NSH console) over SERIAL_CONTROL - QGC's "MAVLink
//! Console". Every keystroke goes to PX4 immediately and NSH echoes it back,
//! so we never echo locally; we just render what arrives.

use mavlink::dialects::development::{self as mav, MavMessage, SerialControlDev, SerialControlFlag};

use crate::config::{SHELL_MAX_LINES, UORB_RATE_PROBE_WINDOW};
use crate::link::Link;
use crate::mav::handlers::push_bounded;
use crate::state::{State, UorbRate, now};

fn serial_control(link: &Link, flags: SerialControlFlag, chunk: &[u8]) -> bool {
    let mut data = [0u8; 70];
    data[..chunk.len()].copy_from_slice(chunk);
    link.send(&MavMessage::SERIAL_CONTROL(mav::SERIAL_CONTROL_DATA {
        baudrate: 0,
        timeout: 0,
        device: SerialControlDev::SERIAL_CONTROL_DEV_SHELL,
        flags,
        count: chunk.len() as u8,
        data,
        // 0 = any, exactly what pymavlink sends for these MAVLink2
        // extension fields.
        target_system: 0,
        target_component: 0,
    }))
}

fn exclusive() -> SerialControlFlag {
    SerialControlFlag::SERIAL_CONTROL_FLAG_RESPOND | SerialControlFlag::SERIAL_CONTROL_FLAG_EXCLUSIVE
}

pub fn claim_shell(link: &Link) -> bool {
    serial_control(link, exclusive(), &[])
}

pub fn release_shell(link: &Link) -> bool {
    serial_control(link, SerialControlFlag::empty(), &[])
}

/// Send one edited command line and arm the echo filter for it.
pub fn send_command(link: &Link, st: &mut State, line: &str) -> bool {
    st.shell_echo = (!line.is_empty()).then(|| EchoFilter { command: line.to_string(), seen: false });
    let mut bytes = line.as_bytes().to_vec();
    bytes.push(b'\n');
    send_shell_raw(link, &bytes)
}

/// Drops the *duplicate* echo of a command we sent. Depending on the PX4
/// build the shell echoes a line once (NuttX NSH), or twice (POSIX pxh on
/// top of MavlinkShell's own echo): either as a second bare line
/// ("pxh> ver all" then "ver all") or glued together ("pxh> ver allver all").
/// Exactly one echo - the one on the prompt line - is kept.
#[derive(Debug)]
pub struct EchoFilter {
    command: String,
    seen: bool,
}

impl EchoFilter {
    /// While the duplicate echo is still arriving it is the unfinished
    /// line; hide it there too so it never flashes up.
    pub fn hides_partial(&self, partial: &str) -> bool {
        self.seen && !partial.is_empty() && self.command.starts_with(partial)
    }

    /// Filter one completed output line: `None` = drop it. Returns whether
    /// the filter is still waiting for more.
    fn apply(&mut self, mut line: String) -> (Option<String>, bool) {
        let cmd = &self.command;
        if !self.seen {
            let doubled = format!("{cmd}{cmd}");
            if line.ends_with(&doubled) {
                line.truncate(line.len() - cmd.len());
                return (Some(line), false);
            }
            if line.ends_with(cmd.as_str()) {
                self.seen = true;
                return (Some(line), true);
            }
            // Not an echo at all (e.g. output raced ahead): stop filtering.
            return (Some(line), false);
        }
        if line == *cmd {
            return (None, false);
        }
        (Some(line), false)
    }
}

/// Ask for the publish rate of `topics` (substring filters, as `uorb top`
/// takes them). The reply is parsed by [`parse_uorb_top_row`] and kept off
/// the shell screen.
pub fn probe_uorb_rates(link: &Link, st: &mut State, topics: &[String]) -> bool {
    st.uorb_probe_sent_at = now();
    // `-1` plus more than one filter also lists matching topics at 0 Hz.
    let mut cmds = format!("uorb top -1 {}\n", topics.join(" "));
    for topic in topics {
        cmds += &format!("listener {topic}\n");
    }
    send_shell_raw(link, cmds.as_bytes())
}

/// `listener` output: "TOPIC: name", then "timestamp: N (A seconds ago)"
/// and "position: [x, y, z]" for odometry topics.
fn parse_listener_line(st: &mut State, line: &str) {
    let line = line.trim();
    if let Some(rest) = line.strip_prefix("TOPIC: ") {
        st.uorb_listen_topic = rest.split_whitespace().next().unwrap_or_default().to_string();
        st.uorb_listen_age = None;
        return;
    }
    if st.uorb_listen_topic.is_empty() {
        return;
    }
    if let Some(rest) = line.strip_prefix("position: [") {
        let v: Vec<f64> = rest.trim_end_matches(']').split(',').filter_map(|n| n.trim().parse().ok()).collect();
        if let [x, y, z] = v[..] {
            let topic = std::mem::take(&mut st.uorb_listen_topic);
            let updated = now() - st.uorb_listen_age.take().unwrap_or(0.0);
            st.vision_pos.insert(topic, crate::state::VisionPos { x, y, z, updated });
        }
    } else if let Some(age) = line
        .strip_prefix("timestamp: ")
        .and_then(|r| r.split_once('('))
        .and_then(|(_, r)| r.strip_suffix(" seconds ago)"))
        .and_then(|a| a.trim().parse::<f64>().ok())
    {
        st.uorb_listen_age = Some(age);
    }
}

/// One `uorb top` table row: "NAME INST #SUB RATE #Q SIZE".
fn parse_uorb_top_row(line: &str) -> Option<(String, u32)> {
    let cols: Vec<&str> = line.split_whitespace().collect();
    let [name, nums @ ..] = cols.as_slice() else { return None };
    if nums.len() != 5 || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return None;
    }
    let nums: Vec<u32> = nums.iter().map(|n| n.parse().ok()).collect::<Option<_>>()?;
    let key = if nums[0] == 0 { name.to_string() } else { format!("{name}/{}", nums[0]) };
    Some((key, nums[2]))
}

pub fn send_shell_raw(link: &Link, bytes: &[u8]) -> bool {
    bytes.chunks(70).all(|chunk| serial_control(link, exclusive(), chunk))
}

/// The line currently being written, with just enough of a terminal model
/// for NSH / pxh line editing: CR, backspace, cursor left/right
/// (`ESC[nD` / `ESC[nC`) and erase-in-line (`ESC[K`). Tab completion and
/// history recall redraw the line with these, so appending blindly garbles
/// it.
#[derive(Debug, Default)]
pub struct ShellLine {
    pub chars: Vec<char>,
    pub cursor: usize,
    /// An escape sequence split across SERIAL_CONTROL packets.
    escape: Option<String>,
}

impl ShellLine {
    fn put(&mut self, c: char) {
        if self.cursor < self.chars.len() {
            self.chars[self.cursor] = c;
        } else {
            self.chars.resize(self.cursor, ' ');
            self.chars.push(c);
        }
        self.cursor += 1;
    }

    fn take(&mut self) -> String {
        self.cursor = 0;
        std::mem::take(&mut self.chars).into_iter().collect::<String>().trim_end().to_string()
    }

    /// Apply a complete CSI sequence (without the leading ESC `[`).
    fn csi(&mut self, seq: &str) {
        let Some(last) = seq.chars().last() else { return };
        let arg = &seq[..seq.len() - last.len_utf8()];
        let n: usize = arg.parse().unwrap_or(1).max(1);
        match last {
            'D' => self.cursor = self.cursor.saturating_sub(n),
            'C' => self.cursor += n,
            'G' => self.cursor = n - 1,
            'K' => match arg {
                "" | "0" => self.chars.truncate(self.cursor),
                "1" => self.chars.iter_mut().take(self.cursor).for_each(|c| *c = ' '),
                _ => self.chars.clear(),
            },
            _ => {} // colours etc.
        }
    }

    /// Feed one character; returns a completed line on "\n".
    fn feed(&mut self, ch: char) -> Option<String> {
        if let Some(esc) = self.escape.as_mut() {
            esc.push(ch);
            // ESC [ params... final byte (@..~); any other ESC x is 2 chars.
            let done = !esc.starts_with('[') || (esc.len() > 1 && ('@'..='~').contains(&ch));
            if done {
                let seq = self.escape.take().unwrap();
                if let Some(csi) = seq.strip_prefix('[') {
                    self.csi(csi);
                }
            }
            return None;
        }
        match ch {
            '\n' => return Some(self.take()),
            '\r' => self.cursor = 0,
            '\x1b' => self.escape = Some(String::new()),
            '\x08' => self.cursor = self.cursor.saturating_sub(1),
            '\x7f' => {
                if self.cursor > 0 {
                    self.cursor -= 1;
                    if self.cursor < self.chars.len() {
                        self.chars.remove(self.cursor);
                    }
                }
            }
            '\t' => {
                for _ in 0..4 {
                    self.put(' ');
                }
            }
            c if c.is_control() => {}
            c => self.put(c),
        }
        None
    }
}

pub fn handle_serial_control(st: &mut State, d: &mav::SERIAL_CONTROL_DATA) {
    if d.device != SerialControlDev::SERIAL_CONTROL_DEV_SHELL || d.count == 0 {
        return;
    }
    let count = (d.count as usize).min(70);
    let text = String::from_utf8_lossy(&d.data[..count]).into_owned();
    st.shell_active = true;

    let probing = now() - st.uorb_probe_sent_at < UORB_RATE_PROBE_WINDOW;
    for ch in text.chars() {
        let Some(line) = st.shell_line.feed(ch) else { continue };
        // Rows from any `uorb top`, the dashboard's poll or a typed one.
        if let Some((topic, hz)) = parse_uorb_top_row(&line) {
            st.uorb_rates.insert(topic, UorbRate { hz, updated: now() });
        }
        parse_listener_line(st, &line);
        if probing {
            continue;
        }
        let line = match st.shell_echo.take() {
            None => Some(line),
            Some(mut filter) => {
                let (kept, waiting) = filter.apply(line);
                if waiting {
                    st.shell_echo = Some(filter);
                }
                kept
            }
        };
        if let Some(line) = line {
            push_bounded(&mut st.shell_lines, line, SHELL_MAX_LINES);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(input: &str) -> (Vec<String>, String) {
        let mut line = ShellLine::default();
        let mut done = Vec::new();
        for ch in input.chars() {
            done.extend(line.feed(ch));
        }
        (done, line.chars.iter().collect())
    }

    #[test]
    fn crlf_and_redraw() {
        let (done, cur) = run("pxh> ver\r\nok\r\npxh> ab\rpxh> abc");
        assert_eq!(done, vec!["pxh> ver", "ok"]);
        assert_eq!(cur, "pxh> abc");
    }

    #[test]
    fn cursor_left_and_erase() {
        // Tab-completion style: move back 3, erase, write the completion.
        let (_, cur) = run("pxh> lis\x1b[3D\x1b[Klistener ");
        assert_eq!(cur, "pxh> listener ");
    }

    fn filtered(cmd: &str, lines: &[&str]) -> Vec<String> {
        let mut filter = Some(EchoFilter { command: cmd.into(), seen: false });
        let mut out = Vec::new();
        for l in lines {
            let mut kept = Some(l.to_string());
            if let Some(mut f) = filter.take() {
                let (k, waiting) = f.apply(kept.take().unwrap());
                kept = k;
                if waiting {
                    filter = Some(f);
                }
            }
            out.extend(kept);
        }
        out
    }

    #[test]
    fn echo_filter_keeps_exactly_one_echo() {
        // NuttX: single echo.
        assert_eq!(filtered("ver", &["nsh> ver", "HW arch"]), vec!["nsh> ver", "HW arch"]);
        // SITL: a second bare echo line.
        assert_eq!(filtered("ver", &["pxh> ver", "ver", "HW arch"]), vec!["pxh> ver", "HW arch"]);
        // SITL: both echoes on one line.
        assert_eq!(filtered("ver", &["pxh> verver", "HW arch"]), vec!["pxh> ver", "HW arch"]);
    }

    #[test]
    fn uorb_top_rows() {
        let row = |l| parse_uorb_top_row(l);
        assert_eq!(row("vehicle_visual_odometry      0    3  100  1  112 "), Some(("vehicle_visual_odometry".into(), 100)));
        assert_eq!(row("sensor_accel                 1    2  400  8   48"), Some(("sensor_accel/1".into(), 400)));
        assert_eq!(row("TOPIC NAME                 INST #SUB RATE #Q SIZE"), None);
        assert_eq!(row("update: 1s, topics: 2, total publications: 100, 11.2 kB/s"), None);
        assert_eq!(row("nsh> uorb top -1 vehicle_visual_odometry vehicle_mocap_odometry"), None);
    }

    #[test]
    fn listener_position() {
        let mut st = State::default();
        for l in [
            "TOPIC: vehicle_visual_odometry",
            " vehicle_odometry",
            "    timestamp: 290918036 (2.500000 seconds ago)",
            "    timestamp_sample: 290917926 (110 us before timestamp)",
            "    position: [1.50000, -2.00000, -0.75000]",
            "    q: [1.00000, 0.00000, 0.00000, 0.00000]",
        ] {
            parse_listener_line(&mut st, l);
        }
        let p = st.vision_pos["vehicle_visual_odometry"];
        assert_eq!((p.x, p.y, p.z), (1.5, -2.0, -0.75));
        assert!((now() - p.updated - 2.5).abs() < 0.5);
    }

    #[test]
    fn backspace_echo() {
        let (_, cur) = run("pxh> lx\x08 \x08s");
        assert_eq!(cur, "pxh> ls");
    }
}
