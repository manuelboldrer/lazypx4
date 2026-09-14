# Contributing

Thanks for looking at `lazypx4`.

## Development setup

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev,map]"
pytest
python -m pyflakes lazypx4 tests
```

## Layout

See the "Layout" section in the [README](README.md). In short:

- `lazypx4/mavlink/` only reads and writes `lazypx4.state.state` (guarded by
  `state.lock`). It never renders.
- `lazypx4/render/` only reads `state` and prints. It never sends MAVLink.
- `lazypx4/navigation.py` is the one place that turns key presses into
  MAVLink commands and screen switches.
- `lazypx4/app.py` owns the threads and the main loop.

## Adding a screen

1. Add `render/<name>.py` with a `draw_<name>()` that builds a list of
   strings and calls `chrome.draw_lines(...)`.
2. Register it in `render/__init__.py`'s `DRAW_FUNCTIONS`.
3. Add `open_<name>_screen()` / `handle_<name>_key()` in `navigation.py` and
   wire the hotkey into `_SCREEN_OPENERS` and `process_key`.

## Guidelines

- Keep every state-changing action behind the `type YES` confirmation.
- Never trust a COMMAND_ACK for arm state - only a HEARTBEAT is authoritative.
- New dependencies need a good reason; `pymavlink` is the only hard one.
- Run `pyflakes` and `pytest` before opening a PR.
