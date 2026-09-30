//! Hand the terminal to a child process (`:!`, `:sh`, the fleet view's
//! full-screen lazypx4) and take it back afterwards, calling `tick` the
//! whole time so heartbeats and telemetry never stop.

use std::io::Write;
use std::os::unix::process::CommandExt;
use std::process::{Command, ExitStatus};
use std::time::Duration;

use ratatui::DefaultTerminal;

/// Runs `command` in the foreground. `pause` shows the exit status and
/// waits for ENTER before redrawing (for commands that print and exit).
/// The outer result is the terminal switch, the inner one the child.
pub fn run_foreground(
    terminal: &mut DefaultTerminal,
    mut command: Command,
    banner: &str,
    pause: bool,
    mut tick: impl FnMut(),
) -> std::io::Result<std::io::Result<ExitStatus>> {
    use crossterm::{cursor, execute, terminal as term};

    let mut out = std::io::stdout();
    term::disable_raw_mode()?;
    execute!(out, term::LeaveAlternateScreen, cursor::Show)?;
    if !banner.is_empty() {
        println!("\x1b[2m{banner}\x1b[0m");
    }

    // Ctrl-C / Ctrl-\ belong to the child: we share its process group, so
    // ignore them here and restore the defaults in the child.
    let signals = [libc::SIGINT, libc::SIGQUIT];
    let previous: Vec<libc::sighandler_t> = signals.iter().map(|&s| unsafe { libc::signal(s, libc::SIG_IGN) }).collect();
    unsafe {
        command.pre_exec(move || {
            for s in signals {
                libc::signal(s, libc::SIG_DFL);
            }
            Ok(())
        });
    }

    let status = match command.spawn() {
        Ok(mut child) => loop {
            match child.try_wait() {
                Ok(Some(status)) => break Ok(status),
                Ok(None) => {
                    tick();
                    std::thread::sleep(Duration::from_millis(50));
                }
                Err(e) => break Err(e),
            }
        },
        Err(e) => Err(e),
    };

    if pause {
        let code = match &status {
            Ok(s) => s.code().map(|c| c.to_string()).unwrap_or_else(|| "signal".into()),
            Err(e) => format!("failed: {e}"),
        };
        print!("\n\x1b[2m[exit {code}] press ENTER to return to lazypx4\x1b[0m");
        let _ = out.flush();
        wait_for_enter(&mut tick);
    }

    for (s, h) in signals.iter().zip(previous) {
        unsafe { libc::signal(*s, h) };
    }
    term::enable_raw_mode()?;
    execute!(out, term::EnterAlternateScreen, cursor::Hide)?;
    terminal.clear()?;
    Ok(status)
}

/// Block on stdin (cooked mode) for a line while still ticking.
fn wait_for_enter(tick: &mut impl FnMut()) {
    loop {
        let mut fd = libc::pollfd { fd: libc::STDIN_FILENO, events: libc::POLLIN, revents: 0 };
        let ready = unsafe { libc::poll(&mut fd, 1, 100) };
        if ready > 0 {
            let _ = std::io::stdin().read_line(&mut String::new());
            return;
        }
        if ready < 0 && std::io::Error::last_os_error().kind() != std::io::ErrorKind::Interrupted {
            return;
        }
        tick();
    }
}
