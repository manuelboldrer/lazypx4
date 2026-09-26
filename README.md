# lazypx4
A vim-style, keyboard-only TUI for PX4 — built for headless, multi-vehicle ops.

![lazypx4 demo](docs/demo.gif)

A terminal UI for a PX4 vehicle over MAVLink, in the spirit of
[lazygit](https://github.com/jesseduffield/lazygit) and
[lazydocker](https://github.com/jesseduffield/lazydocker), inspired by [mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status.git).


QGroundControl is built for one operator, one vehicle, a mouse, and a lot of screen space. lazypx4 is for the other case: driving PX4 over SSH, from a companion computer, or across a fleet of UAVs at once — where spinning up a full GCS per vehicle doesn't scale and every click and window switch costs time.

No mouse, no menus, nothing to configure. One screen, one keypress per view, navigated vim-style (j/k, Ctrl-D/Ctrl-U, / to search). Pair it with tmux and you get a fast, tiled multi-UAV command center from a single terminal.

Beyond MAVLink

lazypx4 doesn't stop at telemetry:

[n] mission map — live ROS 2 LiDAR (PointCloud2) overlay placed at the vehicle, alongside flight state
[u] companion computer — CPU/RAM/disk load, USB enumeration, Wi-Fi/Ethernet link
HOST/SVC dashboard lines — is rosbag running? Is Zenoh up? Is the uXRCE-DDS agent alive?

Vehicle state, sensor feeds, and companion-computer health on one screen make it a genuinely useful preflight check — arming, GPS, EKF status, LiDAR feed, and system health, all glanceable before takeoff.

Why
Zero setup — sensible defaults, works out of the box
Zero mouse — every screen is one keypress away
Built for fleets — one lightweight process per vehicle, tile them in tmux
Full stack view — MAVLink + ROS 2 + companion-computer health in one place

One screen. One keypress. No clicking.


## What it does

`lazypx4` connects to a PX4 autopilot (SITL or a real vehicle) over a MAVLink
UDP link and gives you, from one keyboard-driven screen:

| Key | Screen | |
|-----|--------|--|
| `a` / `d` | Arm / disarm | HEARTBEAT-confirmed, always behind a `type YES` prompt |
| `T` / `L` / `R` | Takeoff / land / RTL | `NAV_TAKEOFF` (prompts for altitude) / `NAV_LAND` / `RETURN_TO_LAUNCH` |
| `m` | Flight modes | legacy modes **and** PX4 v1.15+ Standard Modes (custom / PX4-ROS2 external) |
| `h` | Hold | HOLD-family mode, or `DO_PAUSE_CONTINUE` |
| `F` | Offboard on / off | toggle: switches to `OFFBOARD`, or - when already in it - leaves it for HOLD; PX4 rejects the switch unless offboard setpoints are already streaming (e.g. from a ROS 2 node) |
| `K` | Kill | `MAV_CMD_DO_FLIGHTTERMINATION` - force-stops motors immediately, even in flight; not the same as disarm |
| `H` | Set home | `MAV_CMD_DO_SET_HOME` - marks the current position as home |
| `G` | Geofence action | Sets the `GF_ACTION` parameter (`n`one/`w`arning/`h`old/`r`eturn/`t`erminate) - PX4 has no runtime enable/disable command (it answers `MAV_CMD_DO_FENCE_ENABLE` `UNSUPPORTED`), so this is PX4's real mechanism; breach shown on the dashboard from `FENCE_STATUS` |
| `s` | Sensor calibration | gyro / accel / level / compass / baro, following PX4's `[cal]` prompts |
| `p` | Parameters | browse, filter (`/`), edit, `ALL` / `CHANGED` view, reboot; like QGC, each value shows what it means (enum label, set bitmask bits, unit) plus a description / allowed values / range block for the selected one - e.g. `EKF2_HGT_REF 1` -> `GPS`, with `0 = Barometric pressure  1 = GPS  2 = Range sensor  3 = Vision` |
| (home) | Dashboard | the default screen; its **NAVIGATION / ESTIMATION** section shows GPS fix, sats, HDOP/VDOP, EPH/EPV (horizontal/vertical accuracy, metres), RTK state, the global position as latitude / longitude (degrees) / altitude (AMSL, metres), EKF health verdict and estimator position accuracy, home and fence status, RC link |
| `c` / `r` | Control / RC | attitude / rate / position / velocity setpoints, guidance; RC connection, RSSI/LQ/failsafe, stick-position visualisation for CH1-4 (roll/pitch/throttle/yaw) plus a bar graph for every raw `RC_CHANNELS` value; wind estimate (`WIND_COV`); raw actuator outputs (`SERVO_OUTPUT_RAW`) |
| `n` | Mission | (sidebar entry right below Dashboard) ASCII plan view (drawn to scale, up = North) with trail, current flight mode and armed state at the top; EKF local arrow + GNSS `⊕` overlaid with their offset, home marker, GNSS/RTK read-out (fix, EPH/EPV, correction rate/age, base-station distance); an **ALTITUDE** check that compares the EKF's AMSL altitude with the KML's site altitude (mean of the non-zero altitudes in the file), the EKF's local `-D` against its global altitude above the local origin, and the downward rangefinder - each with a green/yellow/red offset; `g` = goto (`f` / `fire` flies over the last `/fire_gps_loc` fix at the current altitude, `f <alt MSL>` to set one), `x` = keyboard jog, `o` = load a `.kml` file's fence polygon + waypoints as a visualization overlay (view-only, never uploaded on its own) - each waypoint's local N/E and global lat/lon/alt are listed below the grid, `O` = upload the loaded fence polygon to the vehicle for real (MAVLink mission protocol, `mission_type=FENCE`) so PX4 actually enforces it, `f` = fit the map range to the loaded KML overlay, arrow keys = pan the view, `u` = follow the vehicle (on by default; off = centred on the local origin), `0` = reset zoom and pan, `V` = overlay the latest LiDAR scan (the `--lidar-topic` cloud, `v` changes the topic) on the plan view, `B` = the cloud's frame: robot frame (default - x forward / y left, rotated by the vehicle's yaw and placed at its position; assumes a centred, level mount) or an already-registered ENU world frame (`map`/`odom`) drawn as-is, `N` = overlay the ROS planned path (`--navpath-topic`, `nav_msgs/Path` in the ENU map frame, `/navsat_utm_path` by default) as blue dots with `s`/`e` at its ends - the path is published once per plan, so lazypx4 must already be running to receive it; a fix published on `/fire_gps_loc` (e.g. the UAV's own detection, or `P`) is always drawn as a red `F` with its local N/E and the fire's position relative to the vehicle in NED (N/E/D, D positive = fire below the UAV) plus 3-D distance listed below the grid (the topic is subscribed both reliable and best-effort, so a detector using sensor-data QoS is received too; with no fix yet the line shows whether any publisher is seen), `W` = fly the KML waypoints (`3` / `seq` / `rand 8`, optional `heading`), `C` = lawnmower coverage of its polygon (`5`, `5 90`), `P` = one-shot publish the vehicle's current position as a ROS 2 `sensor_msgs/NavSatFix`, `i` saves a satellite snapshot (the loaded `.kml` overlay is drawn onto it too, if one is loaded) |
| `l` | Flight logs | list and download `.ulg` logs (fast, queue-based downloader); `u` uploads the selected log to the PX4 flight-review web server and copies the plot URL, `a` runs the `ecl_ekf` health check on it |
| `g` | Event log | scrolling INFO / WARN / ERROR / FAILSAFE / COMMAND feed |
| `t` | MAVLink shell | PX4 NuttShell over `SERIAL_CONTROL`, like QGC's MAVLink Console |
| `f` | Flash firmware | pick a `.px4` file and a serial port, flash via `px_uploader.py` |
| `u` | USB / network | companion-computer sanity check: USB device enumeration, Wi-Fi/Ethernet link and IP - independent of the MAVLink link |
| `E` | EKF reset | re-initialises the estimator, behind `type YES` |
| `?` | About | logo, author/contact, version |

Every state-changing action (arm, disarm, takeoff, land, RTL, hold, mode
change, parameter set, reboot, calibration, goto, arming jog, kill, set home,
geofence action, geofence upload, offboard on/off) goes through a `type YES` confirmation shown as a
bar across the bottom of the screen. `lazypx4` never force-arms - takeoff,
goto and jog all require you to have armed with `a` first, plus a
GPS/global position.

Vim navigation (`j`/`k`, `Ctrl-D`/`Ctrl-U`, ...) works on every browse screen.

**Key parameters** — a KEY PARAMETERS panel at the top of the `p` screen shows
the ones you check most, live and decoded (`EKF2_HGT_REF  GPS (1)`,
`MIS_TAKEOFF_ALT  2.5 m`): flight (`MIS_TAKEOFF_ALT`, `RTL_RETURN_ALT`,
`MPC_XY_CRUISE`, `MPC_XY_VEL_MAX`, `MPC_LAND_SPEED`), failsafe (`GF_ACTION`,
`NAV_DLL_ACT`, `NAV_RCL_ACT`, `COM_LOW_BAT_ACT`), estimator (`EKF2_HGT_REF`,
`EKF2_RNG_CTRL`, `EKF2_OF_CTRL`, `EKF2_RNG_NOISE`, `EKF2_GPS_CTRL`,
`EKF2_GPS_CHECK`, `EKF2_BARO_CTRL`, `EKF2_MAG_TYPE`) and ROS 2 (`UXRCE_DDS_DOM_ID`,
`UXRCE_DDS_KEY`, `UXRCE_DDS_NS_IDX`). Enum parameters list every option with
the one in force highlighted (`0 Baro  [1 GPS]  2 Range  3 Vision`) and bitmasks
list every bit with the set ones highlighted. Edit the list in `KEY_PARAMETER_COLUMNS`
(`src/config.rs`). On a short terminal the parameter list shrinks to make
room, and the panel hides itself if that would leave fewer than 6 rows.

**Parameter descriptions** — the MAVLink parameter protocol only carries a
name and a number, so the meanings come from PX4's `parameters.json`, the same
metadata QGC uses. A copy from PX4 v1.17 is compiled into the binary (`data/param_meta.json.xz`); to match
the exact firmware on your vehicle, point `--param-defaults` at the
`parameters.json` from its build (e.g.
`build/px4_fmu-v6x_default/parameters.json`, plain or `.xz`), which also
enables the `CHANGED` view against the real defaults. Regenerate the bundled
copy with `python/scripts/make_param_meta.py`.

**Search** — on the Modes, Event log, Flight logs and Parameters screens,
`/` opens an incremental search: matches are highlighted, the cursor jumps to
the first one, `n` / `N` cycle forward / backward, `Esc` clears. The list is
never narrowed.

On the **Map screen** (`n`):

**Flight commands** — every dashboard flight command also works here (while
jog isn't armed): `a`/`d` arm/disarm, `T`/`L`/`R` takeoff/land/RTL, `h` hold,
`F` offboard on/off, `K` kill, `H` set home, `G` geofence action - same keys, same `type YES`
confirmations. Jog owns `a`/`d`/`h`/etc. instead whenever it's armed (`x`).

**View** — the grid is drawn to scale (a terminal row counts as two columns
tall, so the header shows separate E / N half-ranges). It follows the vehicle
by default (`u` toggles back to the old origin-centred view that grows to keep
the vehicle, setpoint and KML in sight); the arrow keys pan by a quarter of the
range, `+`/`-` zoom, `0` resets both. `j`/`k` and `Ctrl-D`/`Ctrl-U` still
scroll the page.

**LiDAR overlay** — `V` draws the latest `--lidar-topic` scan, coloured by
height. Points are taken in the robot frame (x forward / y left / z up),
rotated by the vehicle's yaw and placed at its position, so obstacles stay put
on the map while the vehicle moves through them. `B` switches to a world
(ENU `map`/`odom`) cloud that is drawn as-is; `v` changes the topic. The line
under the map shows the cloud's `frame_id` and the mode in use.

**Goto** — `g` sends the vehicle to a target in one of three frames, chosen
by an optional leading letter (default is relative, unchanged from before):

| Frame | Syntax | Example | Meaning |
|---|---|---|---|
| relative *(default)* | `[r] forward right down [yaw]` | `10 0 -2` | body-frame offset from the current position/heading, metres/degrees |
| local NED | `l N E D [yaw]` | `l 5 -3 -10` | absolute point in the local NED frame (same frame as this screen's grid/trail), origin-relative - needs a known local origin |
| global | `g lat lon alt [yaw]` | `g 52.218650 6.886870 40` | a literal lat/lon and AMSL altitude |

All three send `MAV_CMD_DO_REPOSITION` to an absolute lat/lon/AMSL target
(altitude is always AMSL, so relative `down 0` never changes height). Needs
an armed vehicle; shows the resolved target before the `type YES`.

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

**Waypoint queue** — `W` flies the KML loaded with `o`: one waypoint (`3`),
all in order (`seq`) or a random sequence (`rand 8`); add `heading` to rotate
onto each target before flying to it. Legs are `MAV_CMD_DO_REPOSITION` gotos
that auto-advance on arrival (`WP_QUEUE_ARRIVAL_M`) or time out
(`WP_QUEUE_LEG_TIMEOUT_S`). Needs an armed vehicle; `W` again cancels.

**Area coverage** — `C` sweeps the KML's (first) polygon with a lawnmower
pattern. Enter `<spacing m> [angle]`: parallel lines `spacing` metres apart,
alternating direction, ends pulled 1 m in from the boundary. `angle` is the
compass heading of the lines (`0` = north-south, `90` = east-west); omit it to
align them with the polygon's longest edge. Add `heading` (e.g. `5 90
heading`) to face each leg. The planned path is drawn on the map (cyan, `S`
start / `E` end, waypoint count and length) while the `type YES` prompt is up
- ESC discards it - and stays drawn while it flies; `W` queues get the same
preview. It runs through the same queue as `W` (current altitude, up to 500
waypoints), and `C` or `W` again cancels. On a concave polygon the vehicle may
cut across a notch between two segments, so keep the area convex for now.

**GPS publish** — `P` publishes the vehicle's current global position as a
one-shot ROS 2 `sensor_msgs/NavSatFix` on `/fire_gps_loc`, for external
tooling that wants the live fix on a topic (e.g. a "mark this spot"
workflow) rather than read by hand off this screen. Needs the ROS 2 build
(`--features ros`, below); a banner confirms success/failure.

## Install

lazypx4 is written in Rust ([ratatui](https://ratatui.rs) +
[rust-mavlink](https://github.com/mavlink/rust-mavlink)). You need a Rust
toolchain ([rustup.rs](https://rustup.rs)); then:

```bash
git clone https://github.com/manuelboldrer/lazypx4.git
cd lazypx4
install/install.sh          # plain build
# or, with the ROS 2 features (see below):
source /opt/ros/jazzy/setup.bash && install/install.sh --ros
```

`install/install.sh` builds `target/release/lazypx4`, symlinks it to
`~/.local/bin/lazypx4`, and creates a `.venv` with the Python dependencies
of the vendored PX4 scripts in `Tools/` (see below) plus `speedtest-cli`
for the `[u]` screen's speed test.

### Manual build

```bash
cargo build --release
./target/release/lazypx4
# or install to ~/.cargo/bin:
cargo install --path .
```

The default build is a single ~5 MB binary that needs only glibc - no
Python, no ROS - so you can copy it straight onto a companion computer. The
ROS 2 features (map LiDAR / NAVPATH / FIRE overlays, `[P]`) then say they're unavailable.

### With ROS 2

The map's LiDAR / NAVPATH / FIRE overlays, `[P]` GPS publish and the dashboard's ROS clock need the `ros`
feature:

```bash
source /opt/ros/jazzy/setup.bash
cargo build --release --features ros
```

`.cargo/config.toml` pins `IDL_PACKAGE_FILTER` to the message packages
lazypx4 uses, so r2r only generates those (much faster) and the build doesn't
depend on what your shell exports.

This uses [r2r](https://github.com/sequenceplanner/r2r), which goes through
`rcl`, so it talks over whatever RMW your sourced ROS install uses (Fast
DDS, Cyclone, Zenoh...). The ROS build links against the ROS libraries, so
**source ROS before running it** too. `--use-sim-time` makes the ROS clock
follow `/clock`.

### `Tools/`

`[f]` flash firmware, the flight-logs screen's `[u]` web upload and `[a]`
EKF health-check shell out to real, standalone PX4 scripts rather than
reimplementing them - `px_uploader.py`, `upload_log.py` and
`ecl_ekf/process_logdata_ekf.py`. This repo checks in `Tools/` as a vendored
copy of those scripts straight from
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (BSD-3-Clause, license
kept at `Tools/LICENSE`).

lazypx4 looks for them in `--tools-dir`; the default is `./Tools` if it
exists, otherwise the `Tools/` of the checkout the binary was built from, so
an installed binary works from any directory. The Python interpreter is
chosen in this order: `$VIRTUAL_ENV`, then the `.venv` next to the tools
directory, then `python3`. Their dependencies are listed in
`Tools/requirements.txt` (`install/install.sh` installs them).

To refresh them from a newer PX4-Autopilot release, copy the same files
back in from a checkout of it, or point `--tools-dir` at a PX4-Autopilot
checkout's `Tools/`. Without a valid `Tools/`, every other screen still
works - `[f]`/`[u]`/`[a]` just report the script as not found.

## Run

```bash
lazypx4                        # listen on udpin:0.0.0.0:14560
lazypx4 --port 14550           # a different MAVLink UDP port
lazypx4 --log-dir ~/px4_logs   # where downloaded .ulg logs go
lazypx4 --disk-path /data      # which filesystem the HOST block reports
lazypx4 --firmware-dir ~/px4/build  # where [f] looks for .px4 files
lazypx4 --lidar-topic /livox/points # ROS 2 PointCloud2 for the [n] screen's [V] overlay ([v] there changes it)
lazypx4 --navpath-topic /navsat_utm_path # ROS 2 nav_msgs/Path for the [n] screen's [N] overlay
lazypx4 --kml mission.kml       # preload the [n] mission screen's fence/waypoint overlay
lazypx4 --help
```

The UI comes up immediately and shows `WAITING FOR HEARTBEAT` until a
vehicle appears.

The dashboard's **HOST** line shows the companion computer's CPU / RAM / disk
load (Linux, read from `/proc`), and **SVC** shows whether a rosbag recording
(`ros2 bag record` / `rosbag2`), Zenoh (`zenohd` / `zenoh-bridge-*`) and the
uXRCE-DDS agent (`MicroXRCEAgent`, needed by PX4 v1.14+ to publish/subscribe
uORB topics as ROS 2 topics) are running - all three detected by scanning
process command lines, so no ROS/Zenoh/XRCE dependency is added just to show
their status. The separate `u` **USB / network** screen does the same kind of
host-only check for USB device enumeration and Wi-Fi/Ethernet link state.

`lazypx4` needs an interactive terminal at least 60x16.

### Getting a MAVLink port to connect to

`lazypx4` is a UDP *listener* (`udpin`): it does not go looking for the
vehicle, PX4 has to send MAVLink to it. So a MAVLink instance on the
autopilot must target the machine running `lazypx4` on the port you pass
with `--port` (default `14560`).

**PX4 SITL** - SITL already has an onboard/offboard instance sending to
`14540`, so the simplest option is `lazypx4 --port 14540`. To use the
default port instead, add an instance from `pxh>` or the SITL startup
script:

```bash
mavlink start -u 14560 -o 14560 -m onboard -r 4000000
```

**Real vehicle over Ethernet/Wi-Fi** (e.g. Pixhawk 6X/6C-ETH, or a flight
controller reached through a companion's network) - make the instance start
on every boot by putting the command in `etc/extras.txt` on the flight
controller's SD card (PX4 runs `/fs/microsd/etc/extras.txt` at the end of
startup, after the built-in configuration):

```bash
# on the SD card mounted on your computer (or via the MAVLink shell `[t]`)
mkdir -p etc
cat >> etc/extras.txt <<'EOF'
mavlink start -u 14560 -o 14560 -t 192.168.1.xx -m onboard -r 4000000
EOF
```

- `-t <ip>` is the **companion computer running `lazypx4`** (not the
  flight controller), `-o` is the port it listens on (`--port`), and `-u`
  is any free local port on the flight controller (pick one no other
  instance uses; `14580` is taken by SITL's own offboard instance).
- `-r 4000000` (bytes/s) is fine on a network link; lower it for a radio.
- `-m onboard` gives the companion-computer message set (`normal` also works).
- Reboot the flight controller and check with `mavlink status` in the
  shell that the new instance is streaming.

`extras.txt` is only read when it exists, so creating it is safe; edits take
effect after a reboot. If the flight controller is on a **serial/USB port**
instead, no UDP port exists yet - bridge it with `mavlink-router` or
`mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14560` and run
`lazypx4` on that address/port.

Alternatively, instead of `extras.txt`, you can configure a free MAVLink
instance through its parameters (`MAV_2_CONFIG`, `MAV_2_MODE`,
`MAV_2_RATE`, ...) and Ethernet settings (`MAV_2_UDP_PRT`); which
instance/params are available depends on the board and PX4 version.

## Source layout

```
src/
  main.rs          CLI
  app.rs           session, main loop, key dispatch, prompts, actions
  app_map.rs       keys for the map, sensor, host, firmware and log-tool screens
  link.rs          udpin socket + rust-mavlink codec (GCS identity 255/190)
  state.rs         shared vehicle state + event log
  mav/             receiver + handlers, commands, params, modes, shell, calibration,
                   flight logs, guided (goto / jog / waypoint queue / fence upload)
  geo.rs           projections, KML parser, lawnmower coverage
  satellite.rs     Esri snapshot + annotation + JSON sidecar
  host.rs          CPU / RAM / disk / services, network / USB / Tailscale / speed test
  jobs.rs          background px_uploader / upload_log / ecl_ekf jobs
  ros.rs           r2r node (feature `ros`) or a stub
  lineedit.rs      NSH input line with history
  ui/              frame + one module per screen group
```

The parameter metadata is embedded from `data/param_meta.json.xz` with
`include_bytes!`.

## Implementation notes

- **NSH input is edited locally and sent on ENTER** (↑/↓ history, ←/→ editing),
  like PX4's `Tools/mavlink_shell.py` and QGC's console. Recent PX4
  (`66f197b6ea`, "mavlink_shell: no echo of commands") makes the MAVLink shell
  echo input itself. On SITL, `pxh` echoes it too, so forwarding every keystroke
  showed each character twice. Exactly one echo of a sent command is kept.
- rust-mavlink needs the `mav2-message-extensions` feature, or it silently drops
  MAVLink 2 extension fields (GPS accuracy, dual-antenna yaw, extra battery cells).

## Python version

lazypx4 started as a Python application; that implementation is kept in
[`python/`](python/) (see [`python/README.md`](python/README.md) for its
install and usage). It is feature-equivalent but no longer the main
version.

## Status

`lazypx4` is under active testing - if you hit a bug or have an idea for an
improvement, please [open an issue](https://github.com/manuelboldrer/lazypx4/issues)
or send a PR. Thanks!

## Author

[Manuel Boldrer](https://manuelboldrer.github.io/) - manuel.boldrer@gmail.com
Saxion University of Applied Sciences, [Smart Mechatronics and Robotics Group](https://www.saxion.edu/research/research-groups/smart-mechatronics-and-robotics).

Developed with assistance from Anthropic's Claude AI.

(Also shown in-app on the `?` About screen, along with the running version.)


## License

MIT - see [LICENSE](LICENSE).

