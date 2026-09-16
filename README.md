# lazypx4

A terminal UI for a PX4 vehicle over MAVLink, in the spirit of
[lazygit](https://github.com/jesseduffield/lazygit) and
[lazydocker](https://github.com/jesseduffield/lazydocker), inspired by [mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status.git).

QGroundControl is built for one operator, one vehicle, a mouse and a lot of
screen space. `lazypx4` is for the other case: driving PX4 over SSH, from a
companion computer, or across a fleet of vehicles at once, where spinning up
a full GCS per UAV doesn't scale and every click and window switch costs
time. There is nothing to click - every screen and every action is a single
keypress, navigated vim-style (`j`/`k`, `Ctrl-D`/`Ctrl-U`, `/` search, ...),
with sensible defaults and nothing to configure to get started. One screen,
one keypress per view, a much faster and cleaner workflow than a mouse-driven
GCS - and one you can run N of, side by side in a terminal multiplexer, for a
multi-UAV setup.

It goes beyond MAVLink telemetry, too: the `[v]` LiDAR and `[w]` camera
screens embed live ROS 2 visualization (`PointCloud2`, `Image`/
`CompressedImage`) right next to the flight state, and the dashboard's
**HOST**/**SVC** lines and the `[u]` screen monitor the companion computer
itself - CPU/RAM/disk load, whether rosbag/Zenoh/the uXRCE-DDS agent are up,
USB device enumeration, Wi-Fi/Ethernet link. Vehicle state, sensor feeds and
companion-computer health all on one screen make it a genuinely useful
**preflight check**: arming/GPS/EKF status, sensor calibration, camera/LiDAR
feeds and companion-computer health, all glanceable before you ever take off.

![lazypx4 demo](docs/demo.gif)

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

### `Tools/`

`[f]` flash firmware, the flight-logs screen's `[u]` web upload and `[a]`
EKF health-check all shell out to real, standalone PX4 scripts rather than
reimplementing them - `px_uploader.py`, `upload_log.py` and
`ecl_ekf/process_logdata_ekf.py`. Rather than vendoring copies of them,
`lazypx4` looks for them under a `Tools/` directory (`--tools-dir`,
default `./Tools`), and this repo checks in `Tools` as a symlink to
`../PX4-Autopilot/Tools` - i.e. it expects a
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) checkout as a sibling
of this one:

```bash
cd ..
git clone https://github.com/PX4/PX4-Autopilot.git   # or your own checkout
cd lazypx4
```

If your PX4-Autopilot checkout lives somewhere else, either repoint the
symlink (`ln -sfn /path/to/PX4-Autopilot/Tools Tools`) or pass
`--tools-dir /path/to/PX4-Autopilot/Tools` instead. Without a valid
`Tools/`, every other screen and action still works - `[f]`/`[u]`/`[a]`
just report the script as not found. Installing the `tools` extra above
gets you these scripts' own runtime dependencies (pyserial, requests,
pyulog, ...); it does not fetch the scripts themselves.

### Standalone binary

`./build_binary.sh` builds a standalone, single-file `lazypx4` executable
with PyInstaller - it bundles Python and every pip dependency, so it runs
without activating `.venv` or having Python installed system-wide - and
symlinks it into `~/.local/bin/lazypx4` so it's on `PATH`.

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

## Status

`lazypx4` is under active testing - if you hit a bug or have an idea for an
improvement, please [open an issue](https://github.com/manuelboldrer/lazypx4/issues)
or send a PR. Thanks!

## Author

Manuel Boldrer - manuel.boldrer@gmail.com
Saxion University of Applied Sciences, Smart Mechatronics and Robotics Group.

(Also shown in-app on the `?` About screen, along with the running version.)


## License

MIT - see [LICENSE](LICENSE).

