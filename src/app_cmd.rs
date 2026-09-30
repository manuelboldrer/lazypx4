//! The `:` command line: key handling, completion, running a command, and
//! leaving the TUI for `:!cmd` / `:sh`.

use ratatui::DefaultTerminal;

use crate::app::{Action, App, Screen, is_text};
use crate::cmdline::{self, COMMANDS};
use crate::mav::{guided, modes, params, shell};
use crate::state;

/// What `:!` / `:sh` runs once the main loop has the terminal: a command
/// line, or `None` for an interactive $SHELL.
#[derive(Debug, Clone)]
pub struct HostRun(pub Option<String>);

const PARAM_SUBCOMMANDS: &[&str] = &["get", "set", "save", "load", "refresh"];
const FENCE_WORDS: &[&str] = &["n", "w", "h", "r", "t", "upload"];

impl App {
    pub(crate) fn open_cmdline(&mut self) {
        self.session.cmd_active = true;
        self.session.cmd_candidates.clear();
        self.session.cmd_edit.clear();
    }

    pub(crate) fn handle_cmdline_key(&mut self, key: &str) {
        let edit = &mut self.session.cmd_edit;
        if key != "TAB" {
            self.session.cmd_candidates.clear();
        }
        match key {
            "ESC" => self.session.cmd_active = false,
            // Backspace on an empty line leaves, like vim.
            "BACKSPACE" if edit.text.is_empty() => self.session.cmd_active = false,
            "BACKSPACE" => edit.backspace(),
            "DELETE" => edit.delete(),
            "LEFT" => edit.left(),
            "RIGHT" => edit.right(),
            "HOME" | "CTRL_A" => edit.home(),
            "END" | "CTRL_E" => edit.end(),
            "UP" => edit.history_prev(),
            "DOWN" => edit.history_next(),
            "CTRL_U" => edit.clear(),
            "TAB" => self.complete_cmdline(),
            // Ctrl-J too: typeahead from the moment a `:sh` hands the
            // terminal back arrives as '\n', which raw mode reads as Ctrl-J.
            "ENTER" | "CTRL_J" => {
                let line = edit.submit();
                self.session.cmd_active = false;
                self.run_command_line(&line);
            }
            k if is_text(k) => edit.insert(k),
            _ => {}
        }
    }

    // -----------------------------------------------------------------
    // Completion
    // -----------------------------------------------------------------

    /// Candidates for the word under the cursor, and where that word starts.
    fn completion_candidates(&self, before: &str) -> (usize, Vec<String>) {
        let word_start = before.rfind(char::is_whitespace).map(|i| i + 1).unwrap_or(0);
        let word = &before[word_start..];
        let words: Vec<&str> = before[..word_start].split_whitespace().collect();

        if words.is_empty() {
            if word.starts_with('!') {
                return (word_start, Vec::new());
            }
            let names = COMMANDS
                .iter()
                .filter(|c| c.name != "!")
                .map(|c| c.name)
                .chain(self.settings.aliases.keys().map(String::as_str));
            return (word_start, cmdline::prefix_matches(word, names));
        }

        let command = cmdline::lookup(words[0]).map(|c| c.name).unwrap_or("");
        let st = state::lock(&self.shared);
        let param_names = || cmdline::prefix_matches(word, st.parameters.keys().map(String::as_str));
        let candidates = match (command, words.len()) {
            ("mode", 1) => {
                let labels: Vec<String> = modes::mode_options(&st).iter().map(|o| cmdline::mode_word(&o.label)).collect();
                cmdline::prefix_matches(word, labels.iter().map(String::as_str))
            }
            ("param", 1) => cmdline::prefix_matches(word, PARAM_SUBCOMMANDS.iter().copied()),
            ("param", 2) if matches!(words[1], "get" | "set") => param_names(),
            ("param", 2) if matches!(words[1], "save" | "load") => cmdline::complete_path(word, &self.settings.param_dir),
            ("set" | "get", 1) => param_names(),
            ("fence", 1) => cmdline::prefix_matches(word, FENCE_WORDS.iter().copied()),
            ("offboard", 1) => cmdline::prefix_matches(word, ["on", "off"]),
            ("mission", 1) => cmdline::prefix_matches(word, ["upload"]),
            ("wp", 1) => cmdline::prefix_matches(word, ["seq", "rand", "stop"]),
            ("kml", 1) => cmdline::complete_path(word, "."),
            ("diff", 1 | 2) => cmdline::complete_path(word, &self.settings.param_dir),
            ("replay", 1) => cmdline::prefix_matches(word, ["pause", "resume", "speed"]),
            ("help", 1) => cmdline::prefix_matches(word, COMMANDS.iter().map(|c| c.name)),
            _ => Vec::new(),
        };
        (word_start, candidates)
    }

    /// TAB: a single match is filled in; several extend to their common
    /// prefix and are listed on the prompt's hint row.
    fn complete_cmdline(&mut self) {
        let edit = &self.session.cmd_edit;
        let before: String = edit.text[..edit.cursor].iter().collect();
        let (start, candidates) = self.completion_candidates(&before);
        let start_chars = before[..start].chars().count();
        let word_chars = edit.cursor - start_chars;
        let replacement = match candidates.len() {
            0 => return,
            1 => {
                let c = &candidates[0];
                if c.ends_with('/') { c.clone() } else { format!("{c} ") }
            }
            _ => cmdline::common_prefix(&candidates),
        };
        if candidates.len() > 1 {
            self.session.cmd_candidates = candidates;
        }
        if replacement.chars().count() < word_chars {
            return;
        }
        let edit = &mut self.session.cmd_edit;
        for _ in 0..word_chars {
            edit.backspace();
        }
        edit.insert(&replacement);
    }

    /// The prompt's first row: the TAB candidates, the usage of the command
    /// being typed, or the command list.
    pub fn cmdline_hint(&self) -> String {
        if !self.session.cmd_candidates.is_empty() {
            let n = self.session.cmd_candidates.len();
            let shown: Vec<&str> = self.session.cmd_candidates.iter().take(40).map(String::as_str).collect();
            let more = if n > shown.len() { format!("  (+{} more)", n - shown.len()) } else { String::new() };
            return format!("{}{more}", shown.join("  "));
        }
        let text: String = self.session.cmd_edit.text.iter().collect();
        let (word, _) = cmdline::split(&text);
        if let Some(alias) = self.settings.aliases.get(&word) {
            return format!("alias: {word} = {alias}");
        }
        match cmdline::lookup(&word) {
            Some(c) => format!("{} - {}", c.usage, c.help),
            None => "TAB complete · ↑↓ history · ENTER run · ESC cancel · help lists the commands".into(),
        }
    }

    // -----------------------------------------------------------------
    // Running a command
    // -----------------------------------------------------------------

    pub(crate) fn run_command_line(&mut self, line: &str) {
        if line.trim().is_empty() {
            return;
        }
        let line = cmdline::expand_alias(line, &self.settings.aliases);
        let (word, args) = cmdline::split(&line);
        let Some(spec) = cmdline::lookup(&word) else {
            return state::lock(&self.shared).error(format!(":{word}: unknown command (:help lists them)"));
        };
        let err = |app: &Self, m: String| state::lock(&app.shared).error(m);
        let usage = |app: &Self| err(app, format!("usage: :{}", spec.usage));
        let args_v: Vec<&str> = args.split_whitespace().collect();

        match spec.name {
            "arm" => {
                self.handle_flight_command_key("a");
            }
            "disarm" => {
                self.handle_flight_command_key("d");
            }
            "land" => {
                self.handle_flight_command_key("L");
            }
            "rtl" => {
                self.handle_flight_command_key("R");
            }
            "hold" => {
                self.handle_flight_command_key("h");
            }
            "kill" => {
                self.handle_flight_command_key("K");
            }
            "sethome" => {
                self.handle_flight_command_key("H");
            }
            "ekfreset" => {
                self.handle_flight_command_key("E");
            }
            "reboot" => self.request_confirmation("REBOOT PX4? Vehicle will restart. Type YES", Action::Reboot),
            "takeoff" => {
                if self.takeoff_ready() {
                    self.takeoff_submit(&args);
                }
            }
            "offboard" => {
                let in_offboard = state::lock(&self.shared).mode == "OFFBOARD";
                match args_v.first().copied() {
                    None => {
                        self.handle_flight_command_key("F");
                    }
                    Some("on") if in_offboard => state::lock(&self.shared).warn("Already in OFFBOARD"),
                    Some("off") if !in_offboard => state::lock(&self.shared).warn("Not in OFFBOARD"),
                    Some("on" | "off") => {
                        self.handle_flight_command_key("F");
                    }
                    Some(_) => usage(self),
                }
            }
            "mode" => {
                let options = modes::mode_options(&state::lock(&self.shared));
                let labels: Vec<String> = options.iter().map(|o| o.label.clone()).collect();
                match cmdline::find_mode(&labels, &args) {
                    Ok(i) => {
                        let option = options[i].clone();
                        self.request_confirmation(format!("Set mode to {}? Type YES", option.label), Action::SetMode(option));
                    }
                    Err(e) => err(self, e),
                }
            }
            "goto" => self.goto_submit(&args),
            "param" => match args_v.first().copied() {
                Some("get") if args_v.len() == 2 => self.cmd_param_get(args_v[1]),
                Some("set") if args_v.len() == 3 => self.cmd_param_set(args_v[1], args_v[2]),
                Some("save") => {
                    if self.parameters_ready("Save") {
                        self.param_save_submit(args_v.get(1).copied().unwrap_or(""));
                    }
                }
                Some("load") if args_v.len() == 2 => {
                    if self.parameters_ready("Load") {
                        self.param_load_submit(args_v[1]);
                    }
                }
                Some("refresh") => {
                    params::request_parameters(&self.link, &mut state::lock(&self.shared));
                }
                _ => usage(self),
            },
            "set" if args_v.len() == 2 => self.cmd_param_set(args_v[0], args_v[1]),
            "get" if args_v.len() == 1 => self.cmd_param_get(args_v[0]),
            "fence" => match args_v.as_slice() {
                ["upload"] => self.open_fence_upload_confirm(),
                [letter] => self.fence_submit(letter),
                _ => usage(self),
            },
            "kml" if !args.is_empty() => self.load_kml(&args),
            "mission" if args_v.as_slice() == ["upload"] => self.open_mission_upload_confirm(),
            "wp" if args_v.first() == Some(&"stop") => {
                let mut st = state::lock(&self.shared);
                if st.wp_queue_active {
                    guided::cancel_wp_queue(&mut st, "cancelled by user");
                } else {
                    st.warn("No waypoint queue is running");
                }
            }
            "wp" if !args.is_empty() => {
                if self.queue_ready(true) {
                    self.wp_queue_submit(&args);
                }
            }
            "cover" if !args.is_empty() => {
                if self.queue_ready(false) {
                    self.coverage_submit(&args);
                }
            }
            "nsh" if !args.is_empty() => {
                if !self.ensure_shell() {
                    return err(self, "Could not open the MAVLink shell".into());
                }
                let mut st = state::lock(&self.shared);
                shell::send_command(&self.link, &mut st, &args);
                st.command(format!("nsh> {args}   (output on [t])"));
            }
            "!" if !args.is_empty() => self.request_host_run(Some(args)),
            "sh" => self.request_host_run(None),
            "help" => self.cmd_help(args_v.first().copied()),
            "ack" => {
                if !self.acknowledge_alerts() {
                    state::lock(&self.shared).info("No alerts to acknowledge");
                }
            }
            "replay" => {
                let result = self.link.with_player(|p| match args_v.as_slice() {
                    ["pause"] => {
                        p.paused = true;
                        Ok("Replay paused".to_string())
                    }
                    ["resume"] | ["play"] => {
                        p.paused = false;
                        p.rebase();
                        Ok("Replay resumed".into())
                    }
                    ["speed", x] => match x.parse::<f64>() {
                        Ok(v) if v > 0.0 && v.is_finite() => {
                            p.speed = v;
                            p.rebase();
                            Ok(format!("Replay speed {v}x"))
                        }
                        _ => Err("replay: speed must be a positive number".to_string()),
                    },
                    _ => Err(format!("usage: :{}", spec.usage)),
                });
                match result {
                    Some(Ok(m)) => state::lock(&self.shared).info(m),
                    Some(Err(m)) => err(self, m),
                    None => err(self, "replay: not replaying (start with --replay FILE)".into()),
                }
            }
            "tlog" => {
                let text = match (self.link.tlog_path(), &self.settings.tlog_dir) {
                    (Some(p), _) => format!("Recording telemetry to {}", p.display()),
                    (None, Some(dir)) => format!("Telemetry will be recorded to {dir}/ from the first packet"),
                    (None, None) if self.link.is_replay() => "Replaying - not recording".into(),
                    (None, None) => "Telemetry recording is off ([link] record_tlog / --no-tlog)".into(),
                };
                state::lock(&self.shared).info(text);
            }
            "diff" if !args_v.is_empty() && args_v.len() <= 2 => self.cmd_param_diff(args_v[0], args_v.get(1).copied()),
            "quit" => {
                // Same guard as [q]: never quit out from under a transfer
                // or an armed jog.
                let busy = {
                    let st = state::lock(&self.shared);
                    st.dl_active || st.cal_active || self.session.jog_armed
                };
                if busy {
                    err(self, "quit: a download / calibration / jog is active".into());
                } else {
                    self.shutdown.store(true, std::sync::atomic::Ordering::Relaxed);
                }
            }
            _ => usage(self),
        }
    }

    fn parameters_ready(&self, what: &str) -> bool {
        let mut st = state::lock(&self.shared);
        if !st.parameters_complete {
            st.warn(format!("{what}: wait for the full parameter list ([p] then [r])"));
            return false;
        }
        if what == "Load" && params::param_load_active(&st) {
            st.warn("A parameter load is already running");
            return false;
        }
        true
    }

    fn cmd_param_get(&mut self, name: &str) {
        let name = name.to_uppercase();
        let mut st = state::lock(&self.shared);
        match st.parameters.get(&name) {
            Some(p) => {
                let text = format!("{name} = {}", params::format_value(p));
                st.info(text);
            }
            None => {
                crate::mav::commands::request_param(&self.link, &name);
                st.info(format!("{name}: not loaded - requested it from PX4, try again in a moment"));
            }
        }
    }

    fn cmd_param_set(&mut self, name: &str, value: &str) {
        let name = name.to_uppercase();
        let known = state::lock(&self.shared).parameters.contains_key(&name);
        if !known {
            crate::mav::commands::request_param(&self.link, &name);
            state::lock(&self.shared)
                .warn(format!("{name}: not loaded yet - requested it from PX4, run the command again in a moment"));
            return;
        }
        self.submit_parameter_edit(&name, value);
    }

    /// `:diff FILE` (vs the vehicle) or `:diff A B`, listed in the event log.
    pub(crate) fn cmd_param_diff(&mut self, left: &str, right: Option<&str>) {
        let read = |app: &Self, name: &str| -> Result<(String, Vec<(String, f64)>), String> {
            let path = app.resolve_param_path(name);
            let text = std::fs::read_to_string(&path).map_err(|e| format!("diff: {}: {e}", path.display()))?;
            let label = path.file_name().unwrap_or_default().to_string_lossy().to_string();
            Ok((label, params::file_values(&params::parse_param_file(&text))))
        };
        let (left_label, left_values) = match read(self, left) {
            Ok(v) => v,
            Err(e) => return state::lock(&self.shared).error(e),
        };
        let (right_label, right_values) = match right {
            Some(r) => match read(self, r) {
                Ok(v) => v,
                Err(e) => return state::lock(&self.shared).error(e),
            },
            None => {
                let st = state::lock(&self.shared);
                if !st.parameters_complete {
                    drop(st);
                    return state::lock(&self.shared).warn("diff: wait for the full parameter list ([p] then [r]), or give two files");
                }
                let values = st.parameters.values().map(|p| (p.name.clone(), p.value)).collect();
                ("vehicle".to_string(), values)
            }
        };
        let d = params::diff_parameters(&left_values, &right_values);
        let mut st = state::lock(&self.shared);
        st.info(format!(
            "DIFF {left_label} vs {right_label}: {} differ, {} same, {} only in {left_label}, {} only in {right_label}",
            d.changed.len(),
            d.same,
            d.only_left.len(),
            d.only_right.len()
        ));
        for (name, a, b) in &d.changed {
            st.info(format!("  {name:<16} {:>14}  ->  {}", params::fmt_g(*a, 9), params::fmt_g(*b, 9)));
        }
        if !d.only_left.is_empty() {
            st.info(format!("  only in {left_label}: {}", d.only_left.join(" ")));
        }
        // Against the vehicle, "only on the vehicle" is nearly every
        // parameter a partial file leaves out - just the count.
        if !d.only_right.is_empty() && right.is_some() {
            st.info(format!("  only in {right_label}: {}", d.only_right.join(" ")));
        }
        drop(st);
        self.open_screen(Screen::Log);
    }

    fn cmd_help(&mut self, topic: Option<&str>) {
        let mut st = state::lock(&self.shared);
        if let Some(topic) = topic {
            match cmdline::lookup(topic) {
                Some(c) => st.info(format!(":{} - {}", c.usage, c.help)),
                None => st.error(format!("help: unknown command '{topic}'")),
            }
            return;
        }
        st.info("Commands (type : then the command; TAB completes, ↑↓ history):");
        for c in COMMANDS {
            let aliases = if c.aliases.is_empty() { String::new() } else { format!("  (also :{})", c.aliases.join(" :")) };
            st.info(format!("  :{:<34} {}{aliases}", c.usage, c.help));
        }
        if !self.settings.aliases.is_empty() {
            let from = self.settings.config_path.as_deref().unwrap_or("lazypx4.toml");
            st.info(format!("Aliases from {from}:"));
            for (name, expansion) in &self.settings.aliases {
                st.info(format!("  :{name:<34} = {expansion}"));
            }
        }
        drop(st);
        self.open_screen(Screen::Log);
    }

    // -----------------------------------------------------------------
    // :! / :sh - leave the TUI for a host command
    // -----------------------------------------------------------------

    /// Armed, the operator is about to lose sight of the vehicle: ask first.
    fn request_host_run(&mut self, command: Option<String>) {
        if state::lock(&self.shared).armed {
            let what = command.as_deref().map(|c| format!("run `{c}`")).unwrap_or_else(|| "open a shell".into());
            self.request_confirmation(
                format!("Vehicle is ARMED - leave the TUI to {what}? Heartbeat and telemetry keep running. Type YES"),
                Action::HostShell(HostRun(command)),
            );
        } else {
            self.session.host_run = Some(HostRun(command));
        }
    }

    /// Hand the terminal to the command, ticking (GCS heartbeat, timeouts)
    /// while it runs.
    pub(crate) fn run_host(&mut self, terminal: &mut DefaultTerminal, run: HostRun) -> std::io::Result<()> {
        let shell = std::env::var("SHELL").ok().filter(|s| !s.is_empty()).unwrap_or_else(|| "/bin/sh".into());
        let mut command = std::process::Command::new(&shell);
        let banner = match &run.0 {
            Some(line) => {
                command.arg("-c").arg(line);
                format!("lazypx4 $ {line}")
            }
            None => "lazypx4 is still running (heartbeat, telemetry, logs). `exit` returns to it.".into(),
        };
        let status = crate::suspend::run_foreground(terminal, command, &banner, run.0.is_some(), || self.tick())?;

        let label = run.0.as_deref().unwrap_or(&shell).to_string();
        let mut st = state::lock(&self.shared);
        match status {
            Ok(s) if s.success() => st.info(format!("host: `{label}` finished")),
            Ok(s) => st.warn(format!("host: `{label}` exited with {s}")),
            Err(e) => st.error(format!("host: could not run `{label}`: {e}")),
        }
        Ok(())
    }
}
