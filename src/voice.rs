//! Spoken alerts: a short sentence per alert in the language picked on the
//! About screen, through a Piper neural voice, or speech-dispatcher
//! (`spd-say`) when Piper or its voice is missing.

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Mutex;
use std::time::Duration;

/// One alert as (language, sentence).
pub type Message = (String, String);

/// What an alert sounds like, picked on the About screen.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AlertAudio {
    Mute,
    Ring,
    Italian,
    British,
    American,
    Dutch,
}

impl AlertAudio {
    pub const ALL: [AlertAudio; 6] =
        [AlertAudio::Mute, AlertAudio::Ring, AlertAudio::Italian, AlertAudio::British, AlertAudio::American, AlertAudio::Dutch];

    pub fn label(self) -> &'static str {
        match self {
            AlertAudio::Mute => "Mute",
            AlertAudio::Ring => "Ring",
            AlertAudio::Italian => "Italian",
            AlertAudio::British => "British",
            AlertAudio::American => "American",
            AlertAudio::Dutch => "Dutch",
        }
    }

    pub fn language(self) -> Option<&'static str> {
        match self {
            AlertAudio::Mute | AlertAudio::Ring => None,
            AlertAudio::Italian => Some("it"),
            AlertAudio::British => Some("en-GB"),
            AlertAudio::American => Some("en-US"),
            AlertAudio::Dutch => Some("nl"),
        }
    }

    /// The next option `step` places along, wrapping around.
    pub fn cycled(self, step: isize) -> AlertAudio {
        let n = Self::ALL.len() as isize;
        let i = Self::ALL.iter().position(|a| *a == self).unwrap_or(0) as isize;
        Self::ALL[(i + step).rem_euclid(n) as usize]
    }
}

/// A burst of alerts keeps only the newest few, so speech can't lag far
/// behind what's happening.
const MAX_QUEUED: usize = 3;
/// Below 1 speaks faster than the voice's natural pace.
const PIPER_LENGTH_SCALE: &str = "0.7";
/// spd-say's rate, from -100 to 100.
const SPD_RATE: &str = "50";
/// Long enough for Piper to load its voice on a busy machine.
const PIPER_TIMEOUT: Duration = Duration::from_secs(8);

struct Queue {
    pending: VecDeque<Message>,
    speaking: bool,
}

static QUEUE: Mutex<Queue> = Mutex::new(Queue { pending: VecDeque::new(), speaking: false });

/// `alert` as a sentence in `lang`.
pub fn message(alert: &str, lang: &str) -> Message {
    (lang.to_string(), phrase(alert, lang))
}

/// Queue `messages` and, unless another thread is already speaking, speak
/// until the queue is empty. Blocks while speaking.
pub fn say(messages: Vec<Message>) {
    {
        let mut queue = crate::host::lock(&QUEUE);
        for m in messages {
            if queue.pending.contains(&m) {
                continue;
            }
            if queue.pending.len() >= MAX_QUEUED {
                queue.pending.pop_front();
            }
            queue.pending.push_back(m);
        }
        if queue.speaking {
            return;
        }
        queue.speaking = true;
    }
    loop {
        let next = {
            let mut queue = crate::host::lock(&QUEUE);
            let next = queue.pending.pop_front();
            queue.speaking = next.is_some();
            next
        };
        let Some((lang, text)) = next else { return };
        // Neither engine works: stay silent like a missing player.
        let _ = speak_piper(&lang, &text) || speak_spd(&lang, &text);
    }
}

/// The Piper voice for `lang`, in ~/.local/share/lazypx4/voices.
fn piper_voice(lang: &str) -> Option<PathBuf> {
    let name = match lang {
        "it" => "it_IT-paola-medium",
        "en-GB" => "en_GB-alba-medium",
        "en-US" => "en_US-lessac-medium",
        "nl" => "nl_NL-ronnie-medium",
        _ => return None,
    };
    let path = PathBuf::from(std::env::var_os("HOME")?).join(".local/share/lazypx4/voices").join(format!("{name}.onnx"));
    path.is_file().then_some(path)
}

/// Piper kept running with its voice loaded: starting it takes seconds,
/// a sentence then takes well under one.
struct Piper {
    lang: String,
    child: Child,
    stdin: ChildStdin,
    wavs: Receiver<PathBuf>,
}

impl Drop for Piper {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

static PIPER: Mutex<Option<Piper>> = Mutex::new(None);

fn start_piper(lang: &str) -> Option<Piper> {
    let voice = piper_voice(lang)?;
    let dir = std::env::var_os("XDG_RUNTIME_DIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let mut child = Command::new("piper")
        .arg("--model")
        .arg(&voice)
        .arg("--output-dir")
        .arg(dir.join("lazypx4-voice"))
        .arg("--length-scale")
        .arg(PIPER_LENGTH_SCALE)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .ok()?;
    let stdin = child.stdin.take()?;
    let stderr = child.stderr.take()?;
    let (tx, wavs) = channel();
    // Its log is the only sign that a sentence is ready.
    let _ = std::thread::Builder::new().name("Piper".into()).spawn(move || {
        for line in BufReader::new(stderr).lines().map_while(Result::ok) {
            let Some((_, path)) = line.split_once("Wrote ") else { continue };
            if tx.send(PathBuf::from(path)).is_err() {
                return;
            }
        }
    });
    Some(Piper { lang: lang.to_string(), child, stdin, wavs })
}

/// Load `lang`'s voice ahead of the first alert, and drop any other voice.
pub fn warm(lang: Option<&str>) {
    let lang = lang.map(str::to_string);
    let _ = std::thread::Builder::new().name("PiperWarm".into()).spawn(move || {
        let mut piper = crate::host::lock(&PIPER);
        if piper.as_ref().map(|p| p.lang.as_str()) != lang.as_deref() {
            *piper = lang.as_deref().and_then(start_piper);
        }
    });
}

/// `text` as a WAV from the running Piper, starting it if needed.
fn synthesize(lang: &str, text: &str) -> Option<PathBuf> {
    let mut slot = crate::host::lock(&PIPER);
    if slot.as_ref().is_none_or(|p| p.lang != lang) {
        *slot = start_piper(lang);
    }
    let piper = slot.as_mut()?;
    let line = format!("{}\n", spoken_text(text));
    let wav = piper.stdin.write_all(line.as_bytes()).ok().and_then(|_| piper.stdin.flush().ok());
    let wav = wav.and_then(|_| piper.wavs.recv_timeout(PIPER_TIMEOUT).ok());
    if wav.is_none() {
        *slot = None;
    }
    wav
}

/// One line for Piper, without the pause a comma adds.
fn spoken_text(text: &str) -> String {
    text.replace(',', "").replace('\n', " ")
}

fn speak_piper(lang: &str, text: &str) -> bool {
    let Some(wav) = synthesize(lang, text) else { return false };
    let spoken = crate::host::play(&wav);
    let _ = std::fs::remove_file(&wav);
    spoken
}

fn speak_spd(lang: &str, text: &str) -> bool {
    Command::new("spd-say")
        .args(["--wait", "--priority", "important", "--rate", SPD_RATE, "--language", lang, "--", text])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|s| s.success())
}

/// `texts` in the order it, en, nl.
fn pick<'a>(lang: &str, texts: [&'a str; 3]) -> &'a str {
    match lang {
        "it" => texts[0],
        "nl" => texts[2],
        _ => texts[1],
    }
}

/// Every alert text comes from `State::alert` / `State::failsafe` callers,
/// or is a PX4 failsafe STATUSTEXT (see `classify_statustext`).
fn phrase(alert: &str, lang: &str) -> String {
    let upper = alert.to_uppercase();
    let has = |word: &str| upper.contains(word);
    if upper.starts_with("REC:") {
        return pick(lang, ["Problema di registrazione.", "Recording problem.", "Probleem met de opname."]).into();
    }
    if has("HEARTBEAT TIMEOUT") {
        return pick(lang, ["Collegamento perso.", "Link lost.", "Verbinding verloren."]).into();
    }
    if has("CRITICAL BATTERY") {
        let percent = alert.rsplit(':').next().unwrap_or("").trim().trim_end_matches('%');
        return pick(lang, ["Batteria critica, {} percento.", "Battery critical, {} percent.", "Batterij kritiek, {} procent."])
            .replace("{}", percent);
    }
    if upper.starts_with("EKF:") {
        return pick(lang, ["Errore nella stima di posizione.", "Position estimate fault.", "Fout in de positieschatting."])
            .into();
    }
    if upper.starts_with("MODE CHANGED") {
        let mode = alert.split_once("-> ").map_or("", |(_, rest)| rest);
        let mode = mode.split(" (").next().unwrap_or(mode);
        return pick(
            lang,
            [
                "Modalità cambiata in {}, senza comando.",
                "Mode changed to {}, not commanded.",
                "Modus gewijzigd naar {}, zonder commando.",
            ],
        )
        .replace("{}", mode);
    }
    if upper.starts_with("ACTUATOR") {
        let number = alert.split_whitespace().nth(1).unwrap_or("");
        return pick(
            lang,
            [
                "Attuatore {}, ritorno non confermato.",
                "Actuator {}, return not confirmed.",
                "Actuator {}, terugkeer niet bevestigd.",
            ],
        )
        .replace("{}", number);
    }
    if has("RC LOSS") || has("RC_LOSS") {
        return pick(lang, ["Segnale radiocomando perso.", "RC signal lost.", "RC-signaal verloren."]).into();
    }
    if has("OFFBOARD") {
        return pick(lang, ["Controllo offboard perso.", "Offboard control lost.", "Offboard-besturing verloren."]).into();
    }
    if has("DATA LINK") || has("DATALINK") {
        return pick(lang, ["Collegamento dati perso.", "Data link lost.", "Dataverbinding verloren."]).into();
    }
    if has("GPS") {
        return pick(lang, ["Guasto GPS.", "GPS failure.", "GPS-storing."]).into();
    }
    if has("BATTERY") {
        return pick(lang, ["Failsafe batteria.", "Battery failsafe.", "Batterij-failsafe."]).into();
    }
    pick(lang, ["Failsafe attivato.", "Failsafe activated.", "Failsafe geactiveerd."]).into()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all(alert: &str) -> Vec<String> {
        ["it", "en", "nl"].iter().map(|lang| phrase(alert, lang)).collect()
    }

    #[test]
    fn every_alert_reads_as_a_short_sentence() {
        assert_eq!(all("PX4 HEARTBEAT TIMEOUT - link lost"), ["Collegamento perso.", "Link lost.", "Verbinding verloren."]);
        assert_eq!(
            all("CRITICAL BATTERY: 14%"),
            ["Batteria critica, 14 percento.", "Battery critical, 14 percent.", "Batterij kritiek, 14 procent."]
        );
        assert_eq!(phrase("EKF: LOST", "en"), "Position estimate fault.");
        assert_eq!(
            phrase("MODE CHANGED Position -> Return (not commanded from lazypx4)", "nl"),
            "Modus gewijzigd naar Return, zonder commando."
        );
        assert_eq!(
            phrase("ACTUATOR 2 timer: return to +0.50 NOT confirmed by PX4 - check the output", "it"),
            "Attuatore 2, ritorno non confermato."
        );
    }

    #[test]
    fn px4_failsafes_are_named_or_generic() {
        assert_eq!(phrase("Failsafe enabled: RC loss", "en"), "RC signal lost.");
        assert_eq!(phrase("Failsafe: offboard lost", "it"), "Controllo offboard perso.");
        assert_eq!(phrase("Data link lost", "nl"), "Dataverbinding verloren.");
        assert_eq!(phrase("GPS failure", "en"), "GPS failure.");
        assert_eq!(phrase("Battery failsafe", "en"), "Battery failsafe.");
        assert_eq!(phrase("Failsafe activated", "nl"), "Failsafe geactiveerd.");
    }

    #[test]
    fn spoken_text_has_no_comma_pause() {
        assert_eq!(spoken_text("Battery critical, 14 percent."), "Battery critical 14 percent.");
        assert_eq!(spoken_text("Link\nlost."), "Link lost.");
    }

    #[test]
    fn message_carries_its_language() {
        let m = message("PX4 HEARTBEAT TIMEOUT - link lost", "it");
        assert_eq!(m, ("it".to_string(), "Collegamento perso.".to_string()));
    }

    #[test]
    fn alert_audio_cycles_and_only_languages_speak() {
        assert_eq!(AlertAudio::Mute.cycled(-1), AlertAudio::Dutch);
        assert_eq!(AlertAudio::Dutch.cycled(1), AlertAudio::Mute);
        assert_eq!(AlertAudio::Ring.language(), None);
        assert_eq!(AlertAudio::Dutch.language(), Some("nl"));
        assert_eq!(AlertAudio::American.language(), Some("en-US"));
    }
}
