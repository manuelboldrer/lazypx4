//! A local single-line editor with history - the NSH console's input line.
//!
//! Like PX4's own `Tools/mavlink_shell.py` and QGC's MAVLink console, the
//! command is edited locally and only sent on ENTER. Forwarding every
//! keystroke instead makes each character appear twice on PX4 builds whose
//! MAVLink shell echoes input itself on top of the shell's own echo (PX4
//! commit 66f197b6ea "mavlink_shell: no echo of commands", on POSIX/SITL).

#[derive(Debug, Default)]
pub struct LineEdit {
    pub text: Vec<char>,
    pub cursor: usize,
    history: Vec<String>,
    /// Position while browsing history; `None` = editing a fresh line.
    browsing: Option<usize>,
    /// The fresh line that was being typed before browsing started.
    stash: Vec<char>,
}

const HISTORY_MAX: usize = 200;

impl LineEdit {
    pub fn insert(&mut self, s: &str) {
        for c in s.chars() {
            self.text.insert(self.cursor, c);
            self.cursor += 1;
        }
    }

    pub fn backspace(&mut self) {
        if self.cursor > 0 {
            self.cursor -= 1;
            self.text.remove(self.cursor);
        }
    }

    pub fn delete(&mut self) {
        if self.cursor < self.text.len() {
            self.text.remove(self.cursor);
        }
    }

    pub fn left(&mut self) {
        self.cursor = self.cursor.saturating_sub(1);
    }

    pub fn right(&mut self) {
        self.cursor = (self.cursor + 1).min(self.text.len());
    }

    pub fn home(&mut self) {
        self.cursor = 0;
    }

    pub fn end(&mut self) {
        self.cursor = self.text.len();
    }

    pub fn clear(&mut self) {
        self.text.clear();
        self.cursor = 0;
        self.browsing = None;
    }

    /// Take the line for sending and remember it in the history.
    pub fn submit(&mut self) -> String {
        let line: String = self.text.iter().collect();
        if !line.trim().is_empty() && self.history.last() != Some(&line) {
            self.history.push(line.clone());
            if self.history.len() > HISTORY_MAX {
                self.history.remove(0);
            }
        }
        self.clear();
        line
    }

    pub fn history_prev(&mut self) {
        if self.history.is_empty() {
            return;
        }
        let index = match self.browsing {
            None => {
                self.stash = self.text.clone();
                self.history.len() - 1
            }
            Some(i) => i.saturating_sub(1),
        };
        self.browsing = Some(index);
        self.load(self.history[index].chars().collect());
    }

    pub fn history_next(&mut self) {
        let Some(i) = self.browsing else { return };
        if i + 1 < self.history.len() {
            self.browsing = Some(i + 1);
            self.load(self.history[i + 1].chars().collect());
        } else {
            self.browsing = None;
            let stash = std::mem::take(&mut self.stash);
            self.load(stash);
        }
    }

    fn load(&mut self, text: Vec<char>) {
        self.cursor = text.len();
        self.text = text;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edit_and_history() {
        let mut e = LineEdit::default();
        e.insert("ver al");
        e.insert("l");
        assert_eq!(e.submit(), "ver all");
        e.insert("top");
        e.history_prev();
        assert_eq!(e.text.iter().collect::<String>(), "ver all");
        e.history_next();
        assert_eq!(e.text.iter().collect::<String>(), "top");
        e.home();
        e.insert("x");
        e.end();
        e.backspace();
        assert_eq!(e.text.iter().collect::<String>(), "xto");
    }
}
