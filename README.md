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
 SVC:  rosbag ● REC    zenoh ● up    xrce-agent ● up
 FLIGHT: [a]arm [d]disarm [T]takeoff [L]land [R]RTL [h]hold [m]mode  (goto/jog on [n] map)

 POSITION / VELOCITY ─────────────────────────────────────────────────────────
   X:     12.400   Y:     -3.100   Z:      4.200 m
   VX:     0.010   VY:      0.000   VZ:      0.000 m/s
 ...
 SCREENS ────────────────────────────────────────────────────────────────────
   [m] MODE   [s] CALIBRATE   [n] MAP (goto + jog)   [e] ESTIMATION   [c] CONTROL
   [p] PARAMETERS   [g] EVENT LOG   [l] FLIGHT LOGS   [t] NSH   [f] FIRMWARE
   [u] USB/NET   [v] LIDAR   [?] ABOUT   [q]/[ESC] EXIT
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
| `r` | RC input | stick-position visualisation for CH1-4 (roll/pitch/throttle/yaw) plus a bar graph for every raw `RC_CHANNELS` value, RSSI/LQ/failsafe |
| `w` | Camera preview | up to two ROS 2 `sensor_msgs/Image`/`CompressedImage` topics shown at once - `1`/`2` set each slot's topic (blank clears it), `b` toggles a low-bandwidth ASCII-only render for slow links |
| `n` | Position map | ASCII plan view with trail; EKF local arrow + GNSS `⊕` overlaid with their offset, GNSS/RTK read-out (fix, EPH/EPV, correction rate/age, base-station distance); `g` = goto, `x` = keyboard jog; `i` saves a satellite snapshot |
| `l` | Flight logs | list and download `.ulg` logs (fast, queue-based downloader); `u` uploads the selected log to the PX4 flight-review web server and copies the plot URL, `a` runs the `ecl_ekf` health check on it |
| `g` | Event log | scrolling INFO / WARN / ERROR / FAILSAFE / COMMAND feed |
| `t` | MAVLink shell | PX4 NuttShell over `SERIAL_CONTROL`, like QGC's MAVLink Console |
| `f` | Flash firmware | pick a `.px4` file and a serial port, flash via `px_uploader.py` |
| `u` | USB / network | companion-computer sanity check: USB device enumeration, Wi-Fi/Ethernet link and IP - independent of the MAVLink link |
| `v` | LiDAR point cloud | summary + scatter view of a ROS 2 `sensor_msgs/PointCloud2` topic (e.g. Livox `/livox/points`), in the sensor's own frame; `1`/`2`/`3` switch top-down / front / oblique projection, `c` toggles a freely-rotatable camera panned/tilted with `hjkl`, `t` changes the subscribed topic |
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

# optional: the dashboard's ROS clock, the [v] LiDAR point-cloud screen and
# the [w] camera screen (needs a sourced ROS 2 install on PYTHONPATH too -
# see pyproject.toml)
pip install ".[ros]"

# optional: runtime deps of the standalone PX4 scripts used by [f] flash
# firmware and the flight-logs screen's upload / EKF-health-check actions
pip install ".[tools]"

# optional: the [u] host screen's on-demand internet speed test ([i])
pip install ".[net]"
```

Requires Python 3.9+ and [`pymavlink`](https://pypi.org/project/pymavlink/). Every
extra above is optional - without it, the corresponding screen/action just
reports "not found" instead of failing to start.

The `[w]` camera screen's topics can publish `sensor_msgs/CompressedImage`
instead of raw `Image`; decoding those additionally needs Pillow (the
`[map]` extra above).

## Run

```bash
lazypx4                        # listen on udpin:0.0.0.0:14560
lazypx4 --port 14550           # a different MAVLink UDP port
lazypx4 --log-dir ~/px4_logs   # where downloaded .ulg logs go
lazypx4 --disk-path /data      # which filesystem the HOST block reports
lazypx4 --firmware-dir ~/px4/build  # where [f] looks for .px4 files
lazypx4 --lidar-topic /livox/points # ROS 2 topic for the [v] screen
lazypx4 --camera-topic /camera/image_raw --camera-topic-2 /camera2/image_raw # [w] screen topics
lazypx4 --help
```

The dashboard's **HOST** line shows the companion computer's CPU / RAM / disk
load (Linux, read from `/proc`), and **SVC** shows whether a rosbag recording
(`ros2 bag record` / `rosbag2`), Zenoh (`zenohd` / `zenoh-bridge-*`) and the
uXRCE-DDS agent (`MicroXRCEAgent`, needed by PX4 v1.14+ to publish/subscribe
uORB topics as ROS 2 topics) are running - all three detected by scanning
process command lines, so no ROS/Zenoh/XRCE dependency is added just to show
their status. The separate `u` **USB / network** screen does the same kind of
host-only check for USB device enumeration and Wi-Fi/Ethernet link state.

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
├── config.py       constants, lookup tables, runtime Settings
├── util.py         safe_int / safe_float / clamp / finite
├── ansi.py         escape codes, terminal cursor, screen chrome
├── models.py       LogEvent, PendingArm, CustomMode, FlightLogEntry, Parameter
├── state.py        State (vehicle) + Session (UI) singletons, queues
├── eventlog.py     the in-memory event log + STATUSTEXT classification
├── search.py       the "/" incremental search shared by the list screens
├── sysmon.py       host CPU / RAM / disk + rosbag / zenoh / xrce-agent checks
├── netmon.py       host USB devices + Wi-Fi/Ethernet/IP checks (the [u] screen)
├── rosclock.py     optional rclpy node mirroring ROS 2 "now" for the dashboard
├── lidar.py        optional rclpy node summarizing a PointCloud2 topic ([v] screen)
├── jobs.py         generic background-subprocess runner (flash / upload / EKF check)
├── pxtools.py      wraps the standalone PX4 scripts under Tools/ as jobs
├── terminal.py     raw-mode setup + the keyboard reader thread
├── navigation.py   key -> action controller, screen switching, confirmations
├── satellite.py    satellite-image snapshot (background thread)
├── app.py          connect, start threads, run the render/poll loop
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
    ├── chrome.py     frame painting + colour/label helpers
    ├── jobpanel.py   shared "background job" status block (flash/upload/EKF)
    └── *.py          one module per screen (dashboard, about, host, firmware,
                       pointcloud, mode_select, control, estimation, calibration,
                       eventlog_screen, flightlog, parameters, shell, ...)
```

## Author

Manuel Boldrer - manuel.boldrer@gmail.com
Saxion University of Applied Sciences, Smart Mechatronics and Robotics Group.

(Also shown in-app on the `?` About screen, along with the running version.)


## License

MIT - see [LICENSE](LICENSE).

