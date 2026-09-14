# lazypx4

A terminal UI for a PX4 vehicle over MAVLink, in the spirit of
[lazygit](https://github.com/jesseduffield/lazygit) and
[lazydocker](https://github.com/jesseduffield/lazydocker): one screen, a
single keypress per view, sensible defaults, nothing to configure to get
started.

![lazypx4 demo](docs/demo.gif)

```
╭─ PX4 UAV · COMMAND / TELEMETRY
╰──────────────────────────────────────────────────────────────────────────────
 LINK: CONNECTED    SYS: 1    COMP: 1    LOCKED: YES
 MODE: HOLD
 STATE:  DISARMED
 HOST: CPU  34%   RAM  51% 2.1/4.0G   DISK 62% 14G free   load 0.82
 SVC:  rosbag ● REC    zenoh ● up
 FLIGHT: [a]arm [d]disarm [T]takeoff [L]land [R]RTL [h]hold [m]mode  (goto/jog on [n] map)

 POSITION / VELOCITY ─────────────────────────────────────────────────────────
   X:     12.400   Y:     -3.100   Z:      4.200 m
   VX:     0.010   VY:      0.000   VZ:      0.000 m/s
 ...
 SCREENS ────────────────────────────────────────────────────────────────────
   [m] MODE   [s] CALIBRATE   [n] MAP (goto + jog)   [e] ESTIMATION   [c] CONTROL
   [p] PARAMETERS   [g] EVENT LOG   [l] FLIGHT LOGS   [t] NSH   [q]/[ESC] EXIT
```

## What it does

`lazypx4` connects to a PX4 autopilot (SITL or a real vehicle) over a MAVLink
UDP link and gives you, from one keyboard-driven screen:

| Key | Screen | |
|-----|--------|--|
| `a` / `d` | Arm / disarm | HEARTBEAT-confirmed, always behind a `type YES` prompt |
| `T` / `L` / `R` | Takeoff / land / RTL | `NAV_TAKEOFF` (prompts for altitude) / `NAV_LAND` / `RETURN_TO_LAUNCH` |
| `m` | Flight modes | legacy modes **and** PX4 v1.15+ Standard Modes (custom / PX4-ROS2 external) |
| `h` | Hold | HOLD-family mode, or `DO_PAUSE_CONTINUE` |
| `s` | Sensor calibration | gyro / accel / level / compass / baro, following PX4's `[cal]` prompts |
| `p` | Parameters | browse, filter (`/`), edit, `ALL` / `CHANGED` view, reboot |
| `e` | Estimation | EKF health, innovation test ratios, GPS, rangefinder, barometer, height reference |
| `c` | Control | attitude / rate / position / velocity setpoints, guidance, RC sticks |
| `n` | Position map | ASCII plan view with trail; EKF local arrow + GNSS `⊕` overlaid with their offset, GNSS/RTK read-out (fix, EPH/EPV, correction rate/age, base-station distance); `g` = goto, `x` = keyboard jog; `i` saves a satellite snapshot |
| `l` | Flight logs | list and download `.ulg` logs (fast, queue-based downloader) |
| `g` | Event log | scrolling INFO / WARN / ERROR / FAILSAFE / COMMAND feed |
| `t` | MAVLink shell | PX4 NuttShell over `SERIAL_CONTROL`, like QGC's MAVLink Console |
| `?` | About | logo, author/contact, sponsor link, version |

Every state-changing action (arm, disarm, takeoff, land, RTL, hold, mode
change, parameter set, reboot, calibration, goto, arming jog) goes through a
`type YES` confirmation shown as a bar across the bottom of the screen.
`lazypx4` never force-arms - takeoff, goto and jog all require you to have
armed with `a` first, plus a GPS/global position.

Vim navigation (`j`/`k`, `Ctrl-D`/`Ctrl-U`, ...) works on every browse screen.

**Search** — on the Modes, Event log, Flight logs and Parameters screens,
`/` opens an incremental search: matches are highlighted, the cursor jumps to
the first one, `n` / `N` cycle forward / backward, `Esc` clears. The list is
never narrowed.

On the **Map screen** (`n`):

**Goto** — `g` sends the vehicle to a body-frame relative point: type
`forward right down [yaw]` in metres/degrees (e.g. `10 0 0` = 10 m straight
ahead, same height; `10 0 -2` = 10 m ahead and 2 m up). Sent as
`MAV_CMD_DO_REPOSITION` from the current position with the target altitude as
AMSL, so `down 0` never changes height. Needs a GPS fix and an armed vehicle;
shows the target before the `type YES`.

**Keyboard jog** — `x` arms jog (one `type YES`). While armed, each key sends
one `DO_REPOSITION` step from the current position, immediately:

| Key | | Key | |
|-----|--|-----|--|
| `k` / `j` | forward / backward | `w` / `s` | up / down |
| `a` / `d` | strafe left / right | `h` / `l` | yaw left / right |
| `[` / `]` | smaller / bigger step | `x` | disarm jog |

Default step is 1 m and 15°. Needs an armed vehicle with a GPS fix; `q` will
not quit while jog is armed. A red `JOG ARMED` banner and the last command
ACK are shown at the top of the map.

## Install

```bash
pip install .

# optional: annotate the satellite snapshot with pins + a scale bar
pip install ".[map]"
```

Requires Python 3.9+ and [`pymavlink`](https://pypi.org/project/pymavlink/).

## Run

```bash
lazypx4                        # listen on udpin:0.0.0.0:14560
lazypx4 --port 14550           # a different MAVLink UDP port
lazypx4 --log-dir ~/px4_logs   # where downloaded .ulg logs go
lazypx4 --disk-path /data      # which filesystem the HOST block reports
lazypx4 --help
```

The dashboard's **HOST** line shows the companion computer's CPU / RAM / disk
load (Linux, read from `/proc`), and **SVC** shows whether a rosbag recording
(`ros2 bag record` / `rosbag2`) and Zenoh (`zenohd` / `zenoh-bridge-*`) are
running - detected by scanning process command lines, so no ROS/Zenoh
dependency is added.

`lazypx4` needs an interactive terminal at least 60x16.

With PX4 SITL, point one of its MAVLink instances at the port and start:

```bash
# in pxh> ,  or via the SITL startup script
mavlink start -u 14560 -o 14560 -m normal -r 4000000
```

> **Calibration / event text:** PX4's `onboard` / `minimal` MAVLink modes do
> not stream `STATUSTEXT`, so calibration prompts won't appear. Use a
> `normal`-mode instance (`lazypx4` also re-requests `STATUSTEXT` / `EVENT`
> on start and on `[r]`).

## Layout

```
lazypx4/
├── config.py          constants, lookup tables, runtime Settings
├── util.py            safe_int / safe_float / clamp / finite
├── ansi.py            escape codes, terminal cursor, screen chrome
├── models.py          LogEvent, PendingArm, CustomMode, FlightLogEntry, Parameter
├── state.py           State (vehicle) + Session (UI) singletons, queues
├── eventlog.py        the in-memory event log + STATUSTEXT classification
├── search.py          the "/" incremental search shared by the list screens
├── sysmon.py          host CPU / RAM / disk + rosbag / zenoh checks (dashboard HOST block)
├── terminal.py        raw-mode setup + the keyboard reader thread
├── navigation.py      key -> action controller, screen switching, confirmations
├── satellite.py       satellite-image snapshot (background thread)
├── app.py             connect, start threads, run the render/poll loop
├── mavlink/
│   ├── connection.py    connect, GCS heartbeat, stream setup, vehicle_ready
│   ├── receiver.py      the background MAVLink receiver + dispatch table
│   ├── handlers.py      per-message telemetry handlers
│   ├── commands.py      arm/disarm, set_mode, hold, reboot
│   ├── guided.py        takeoff / land / RTL / "goto" / jog (MAV_CMD_DO_REPOSITION)
│   ├── modes.py         Standard Modes Protocol (AVAILABLE_MODES / CURRENT_MODE)
│   ├── parameters.py    PX4 parameter protocol
│   ├── calibration.py   MAV_CMD_PREFLIGHT_CALIBRATION
│   ├── flightlog.py     ULog listing + fast queue-based downloader
│   └── shell.py         NSH console over SERIAL_CONTROL
└── render/
    ├── chrome.py        frame painting + colour/label helpers
    └── *.py             one module per screen
```

### Threads

* **main** - render loop (10 Hz), key dispatch, periodic `check_*` housekeeping.
* **MAVLinkThread** - `recv_match` loop; folds messages into `state` under `state.lock`.
* **KeyboardThread** - decodes stdin bytes into key names onto a queue.
* **SysMonThread** - samples host CPU / RAM / disk and the rosbag / zenoh checks every 2 s.
* **FlightLog-N** / **SatelliteMap** - short-lived download workers.

`state.lock` (an `RLock`) guards all vehicle state. `LOG_DATA` packets bypass
it entirely and go straight to the download worker over a `queue.Queue`.

## Author

Manuel Boldrer - manuel.boldrer@gmail.com

Sponsor: https://github.com/sponsors/manuelboldrer

(Also shown in-app on the `?` About screen, along with the running version.)

## License

MIT - see [LICENSE](LICENSE).
