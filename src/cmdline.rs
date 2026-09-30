//! The `:` command line: the command table and the pure helpers (tokenising,
//! alias expansion, completion). What each command does lives in
//! `app_cmd.rs`.

pub struct CommandSpec {
    pub name: &'static str,
    pub aliases: &'static [&'static str],
    pub usage: &'static str,
    pub help: &'static str,
}

const fn cmd(
    name: &'static str,
    aliases: &'static [&'static str],
    usage: &'static str,
    help: &'static str,
) -> CommandSpec {
    CommandSpec { name, aliases, usage, help }
}

/// Every command still goes through the same "type YES" confirmation as
/// its hotkey.
pub const COMMANDS: &[CommandSpec] = &[
    cmd("arm", &[], "arm", "arm the vehicle"),
    cmd("disarm", &[], "disarm", "disarm the vehicle"),
    cmd("takeoff", &["to"], "takeoff [alt m]", "take off and climb (default 2.5 m)"),
    cmd("land", &[], "land", "land here"),
    cmd("rtl", &[], "rtl", "return to launch"),
    cmd("hold", &["loiter"], "hold", "hold position"),
    cmd("offboard", &["ob"], "offboard [on|off]", "enter / leave OFFBOARD (no argument toggles)"),
    cmd("kill", &[], "kill", "flight termination: stop the motors NOW"),
    cmd("mode", &[], "mode <name>", "set a flight mode (TAB completes the names)"),
    cmd("goto", &["go"], "goto [r|l|g|f] a b c [yaw]", "same syntax as [g] on the map"),
    cmd("sethome", &[], "sethome", "set home to the current position"),
    cmd("ekfreset", &[], "ekfreset", "ekf stop / ekf start through the NSH shell"),
    cmd("reboot", &[], "reboot", "reboot PX4"),
    cmd("param", &["p"], "param get|set|save|load|refresh ...", "param get NAME · set NAME VALUE · save [FILE] · load FILE"),
    cmd("set", &[], "set NAME VALUE", "short for param set"),
    cmd("get", &[], "get NAME", "short for param get"),
    cmd("fence", &[], "fence n|w|h|r|t | upload", "geofence action (GF_ACTION), or upload the KML fence"),
    cmd("kml", &["plan"], "kml FILE", "load a .kml or QGC .plan (waypoints + fence overlay)"),
    cmd("mission", &[], "mission upload", "upload the loaded .plan's mission to PX4"),
    cmd("wp", &[], "wp <n>|seq|rand N [heading] | stop", "fly KML waypoints, or stop the running queue"),
    cmd("cover", &[], "cover SPACING [ANGLE] [heading]", "lawnmower coverage of the KML polygon"),
    cmd("nsh", &[], "nsh LINE", "run LINE in the PX4 NSH shell (output on [t])"),
    cmd("!", &[], "!COMMAND", "run COMMAND on this computer (leaves the TUI; telemetry keeps running)"),
    cmd("sh", &["shell"], "sh", "open $SHELL on this computer; exit to come back"),
    cmd("ack", &[], "ack", "acknowledge the alert banner (same as [A])"),
    cmd("replay", &[], "replay pause|resume|speed X", "control a --replay"),
    cmd("tlog", &[], "tlog", "show where this session's .tlog is being written"),
    cmd("diff", &[], "diff FILE [FILE2]", "parameter differences: FILE vs the vehicle, or FILE vs FILE2"),
    cmd("help", &["h"], "help [command]", "list the commands in the event log"),
    cmd("quit", &["q"], "quit", "quit lazypx4"),
];

pub fn lookup(name: &str) -> Option<&'static CommandSpec> {
    let name = name.to_lowercase();
    COMMANDS.iter().find(|c| c.name == name || c.aliases.contains(&name.as_str()))
}

/// `!cmd` is its own command with the rest of the line (verbatim) as the
/// argument; everything else splits into a command word and its arguments.
pub fn split(line: &str) -> (String, String) {
    let line = line.trim();
    if let Some(rest) = line.strip_prefix('!') {
        return ("!".into(), rest.trim().into());
    }
    match line.split_once(char::is_whitespace) {
        Some((word, rest)) => (word.to_lowercase(), rest.trim().into()),
        None => (line.to_lowercase(), String::new()),
    }
}

/// One level of alias expansion: the first word is replaced, the rest of
/// the line is appended (so `:up 5` with `up = "takeoff"` is `takeoff 5`).
pub fn expand_alias(line: &str, aliases: &std::collections::BTreeMap<String, String>) -> String {
    let (word, rest) = split(line);
    match aliases.get(&word) {
        Some(expansion) if rest.is_empty() => expansion.clone(),
        Some(expansion) => format!("{expansion} {rest}"),
        None => line.trim().to_string(),
    }
}

/// Longest prefix shared by all candidates (case-insensitive, taken from
/// the first one).
pub fn common_prefix(candidates: &[String]) -> String {
    let Some(first) = candidates.first() else { return String::new() };
    let mut end = first.len();
    for c in &candidates[1..] {
        end = end.min(
            first
                .char_indices()
                .zip(c.chars())
                .find(|((_, a), b)| !a.eq_ignore_ascii_case(b))
                .map(|((i, _), _)| i)
                .unwrap_or(first.len().min(c.len())),
        );
    }
    first[..end].to_string()
}

pub fn prefix_matches<'a>(prefix: &str, words: impl IntoIterator<Item = &'a str>) -> Vec<String> {
    let p = prefix.to_lowercase();
    let mut out: Vec<String> = words.into_iter().filter(|w| w.to_lowercase().starts_with(&p)).map(str::to_string).collect();
    out.sort();
    out.dedup();
    out
}

/// File-name completion for `word`: `~/` expanded for the lookup but kept
/// in the result; directories get a trailing `/`. `base` is where a
/// relative path is looked up.
pub fn complete_path(word: &str, base: &str) -> Vec<String> {
    let (dir_part, file_part) = match word.rfind('/') {
        Some(i) => (&word[..=i], &word[i + 1..]),
        None => ("", word),
    };
    let lookup = if let Some(rest) = dir_part.strip_prefix("~/") {
        format!("{}/{rest}", std::env::var("HOME").unwrap_or_default())
    } else if dir_part.starts_with('/') {
        dir_part.to_string()
    } else {
        format!("{}/{dir_part}", base.trim_end_matches('/'))
    };
    let Ok(entries) = std::fs::read_dir(&lookup) else { return Vec::new() };
    let mut out: Vec<String> = entries
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            if !name.starts_with(file_part) || (name.starts_with('.') && !file_part.starts_with('.')) {
                return None;
            }
            let dir = e.file_type().map(|t| t.is_dir()).unwrap_or(false);
            Some(format!("{dir_part}{name}{}", if dir { "/" } else { "" }))
        })
        .collect();
    out.sort();
    out
}

/// A flight-mode label ("Position [POSITION_HOLD]", "OFFBOARD") as one
/// typeable word: the part before any " [", spaces -> '_'.
pub fn mode_word(label: &str) -> String {
    label.split(" [").next().unwrap_or(label).trim().replace(' ', "_")
}

fn squash(s: &str) -> String {
    s.chars().filter(|c| c.is_alphanumeric()).collect::<String>().to_lowercase()
}

/// Index of the mode `query` names: exact word, else a unique prefix, else
/// a unique substring (ignoring case and punctuation).
pub fn find_mode(labels: &[String], query: &str) -> Result<usize, String> {
    let q = squash(query);
    if q.is_empty() {
        return Err("mode: which one? (TAB lists them)".into());
    }
    let words: Vec<String> = labels.iter().map(|l| squash(&mode_word(l))).collect();
    if let Some(i) = words.iter().position(|w| *w == q) {
        return Ok(i);
    }
    for test in [|w: &String, q: &str| w.starts_with(q), |w: &String, q: &str| w.contains(q)] {
        let hits: Vec<usize> = (0..words.len()).filter(|&i| test(&words[i], &q)).collect();
        match hits.len() {
            1 => return Ok(hits[0]),
            0 => {}
            _ => {
                let names: Vec<String> = hits.iter().map(|&i| mode_word(&labels[i])).collect();
                return Err(format!("mode: '{query}' is ambiguous: {}", names.join(" ")));
            }
        }
    }
    Err(format!("mode: no mode matches '{query}'"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splits_bang_and_words() {
        assert_eq!(split("  !ls -la "), ("!".into(), "ls -la".into()));
        assert_eq!(split("Param set MPC_XY 3"), ("param".into(), "set MPC_XY 3".into()));
        assert_eq!(split("land"), ("land".into(), String::new()));
    }

    #[test]
    fn alias_expansion_appends_arguments() {
        let mut a = std::collections::BTreeMap::new();
        a.insert("up".to_string(), "takeoff".to_string());
        a.insert("agent".to_string(), "!systemctl restart agent".to_string());
        assert_eq!(expand_alias("up 5", &a), "takeoff 5");
        assert_eq!(expand_alias("agent", &a), "!systemctl restart agent");
        assert_eq!(expand_alias(" land ", &a), "land");
    }

    #[test]
    fn lookup_finds_aliases() {
        assert_eq!(lookup("q").unwrap().name, "quit");
        assert_eq!(lookup("TAKEOFF").unwrap().name, "takeoff");
        assert!(lookup("nope").is_none());
    }

    #[test]
    fn common_prefix_of_candidates() {
        let c = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(common_prefix(&c(&["MPC_XY_VEL_MAX", "MPC_XY_CRUISE"])), "MPC_XY_");
        assert_eq!(common_prefix(&c(&["land"])), "land");
        assert_eq!(common_prefix(&[]), "");
    }

    #[test]
    fn finds_modes_by_word_prefix_or_substring() {
        let labels: Vec<String> =
            ["Position [POSITION_HOLD]", "Hold [STANDARD]", "Offboard [CUSTOM]", "Altitude [ALTITUDE_HOLD]"]
                .iter()
                .map(|s| s.to_string())
                .collect();
        assert_eq!(find_mode(&labels, "hold"), Ok(1));
        assert_eq!(find_mode(&labels, "off"), Ok(2));
        assert_eq!(find_mode(&labels, "ALT"), Ok(3));
        assert!(find_mode(&labels, "t").is_err()); // posiTion, alTitude
        assert!(find_mode(&labels, "mission").is_err());
    }
}
