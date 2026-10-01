# lazypx4 reference

Full list of keys, commands and options. Press `?` inside lazypx4 to see the keys for the current screen.

## Keys (single vehicle)

| Key | Screen / action |
|-----|-----------------|
| (home) | Dashboard — GPS/RTK, EKF health, position, home/fence, RC, host & services |
| `a` / `d` | Arm / disarm |
| `T` / `L` / `R` / `h` | Takeoff / land / RTL / hold |
| `F` / `K` | Offboard on/off / kill (flight termination) |
| `H` / `G` / `E` | Set home / geofence action (`GF_ACTION`) / EKF reset |
| `y` | Preflight go / no-go — see below |
| `m` | Flight modes (legacy + PX4 v1.15+ Standard Modes) |
| `n` | Mission map — see below |
| `c` / `r` | Control setpoints / RC channels, wind, actuator outputs |
| `p` | Parameters — browse, search, edit, decoded values, key-parameter panel, `s` save / `l` load / `D` diff a param file |
| `s` | Sensor calibration |
| `l` | Flight logs — download `.ulg`, upload to flight review, `ecl_ekf` check |
| `g` | Event log |
| `t` | MAVLink shell (NuttShell) |
| `f` | Flash firmware (`px_uploader.py`) |
| `u` | USB / network check |
| `:` | Command line — see below |
| `?` | Keys for the current screen |
| `A` | Acknowledge the alert banner |

Every state-changing action asks for a `type YES` confirmation. lazypx4 never
force-arms.

### Alerts

Link loss, critical battery, an EKF fault, any PX4 failsafe and an in-air
mode change that lazypx4 didn't command raise a flashing banner and ring
the terminal bell, so tmux flags the window even when you're looking at
another pane. `A` acknowledges.

### Preflight (`y`)

One screen, one verdict (GO / GO with warnings / NO-GO): link, PX4's own
arming check, sensor health, calibration, GPS / RTK, EKF, home, battery,
vibration, RC, geofence, disk, CPU, DDS agent / Zenoh and rosbag. The `a`
confirmation shows the same verdict. The dashboard also has sparklines
of battery, altitude, vibration and link rate.

### Command line (`:`)

Vim-style: `TAB` completes commands, mode names, parameter names and file
paths; `↑`/`↓` walk the history; `:help` lists everything in the event log.
Flight commands still ask for `type YES`.

| Command | |
|---------|--|
| `:arm` `:disarm` `:land` `:rtl` `:hold` `:kill` | Same as the hotkeys |
| `:takeoff 5` · `:mode posctl` · `:offboard on` | Flight commands with arguments |
| `:goto l 5 -3 -10` · `:wp seq` · `:cover 5 90` · `:fence upload` | Map commands without opening the map |
| `:set MPC_XY_VEL_MAX 3` · `:get GF_ACTION` · `:param save` / `load FILE` | Parameters |
| `:nsh top once` | Run a line in the PX4 NSH shell |
| `:diff saved.params` · `:diff a.params b.params` | Parameter differences, vs the vehicle or between files |
| `:mission upload` · `:ack` · `:tlog` · `:replay speed 4` | Mission, alerts, recording, replay |
| `:!cmd` · `:sh` | Run a command / open `$SHELL` on this computer |

`:!` and `:sh` leave the TUI while the command runs. lazypx4 keeps
sending its heartbeat and reading telemetry in the meantime, and asks for
`type YES` first if the vehicle is armed.

### Mission map (`n`)

Scale plan view with trail, home, EKF vs GNSS position and an altitude
cross-check. Flight keys work here too.

| Key | |
|-----|--|
| `g` | Goto — relative `10 0 -2`, local NED `l 5 -3 -10`, or global `g lat lon alt` |
| `x` | Keyboard jog (`k`/`j`/`a`/`d` move, `w`/`s` up/down, `h`/`l` yaw, `[`/`]` step) |
| `o` / `O` | Load a `.kml` or QGC `.plan` (fence + waypoints) / upload the fence to PX4 |
| `M` | Upload the loaded `.plan` mission to PX4 (`:mode mission` flies it) |
| `W` / `C` | Fly KML waypoints (`3`, `seq`, `rand 8`) / lawnmower coverage (`5 90`) |
| `V` / `v` / `B` | LiDAR overlay / change topic / robot vs world frame |
| `N` | ROS planned-path overlay |
| `P` | Publish current position as `sensor_msgs/NavSatFix` on `/fire_gps_loc` |
| `i` | Save a satellite snapshot |
| arrows, `+`/`-`, `0`, `u`, `f` | Pan, zoom, reset, follow vehicle, fit to KML |

## Recording, replay and fleets

Every session is recorded to `./px4_tlogs/*.tlog` (both directions, the
standard format MAVExplorer / pymavlink / QGC read); `--no-tlog` turns it
off. Replay one without a vehicle — nothing is sent:

```bash
lazypx4 --replay px4_tlogs/lazypx4_20260930_163553_p14560.tlog --replay-speed 4
```

`--fleet 14560-14563` shows one row per vehicle port (link, arm, mode,
battery, GPS, altitude, last reply / warning; alerts flash the row), with
the selected vehicle's event log underneath. It keeps sending GCS
heartbeats to every vehicle the whole time.

| Key | |
|-----|--|
| `j` / `k`, `SPACE` / `ESC` | Select / mark, unmark — commands go to the marked vehicles, else the selected one |
| `a` `d` `T` `L` `R` `h` `K` | Arm (shows each vehicle's preflight verdict), disarm, takeoff, land, RTL, hold, kill |
| `y` | Selected vehicle's preflight checklist (shown beside its events on wide terminals) |
| `m` / `F` / `H` / `E` / `b` | Mode (`posctl`, `hold`, `mission`, ...) / offboard toggle / set home / EKF reset / reboot |
| `g` / `x` | Goto (same syntax as the map) / keyboard jog — one vehicle at a time |
| `ENTER` | Full lazypx4 on that vehicle; `q` there returns to the fleet |
| `A` / `q` | Acknowledge alerts / quit |

## Parameters

On the parameter screen `s` saves every parameter in QGroundControl's
`.params` format (to `--param-dir`, default `./px4_params`) and `l` loads one
back. `l` also accepts MAVProxy-style `NAME VALUE`, NSH `param set NAME VALUE`
and YAML `NAME: VALUE` files. It shows only the values that differ and, after
`type YES`, sets them one at a time, checking each value PX4 echoes back.

Parameter descriptions come from a bundled PX4 v1.17 `parameters.json`; pass
`--param-defaults <build>/parameters.json` to match your firmware and enable
the `CHANGED` view.

## Settings file

Defaults can live in [`lazypx4.toml`](../lazypx4.toml) (`--config FILE`, else
`./lazypx4.toml`, else the one in the repo lazypx4 was built from):
port, directories, ROS topics, battery thresholds, alert bell, fleet ports,
the key-parameter panel, key remaps (`"K" = ""` disables the kill key) and
`:` aliases such as `agent = "!sudo systemctl restart micro-xrce-dds-agent"`.
Command-line flags override the file.

## Vendored tools

`Tools/` is a vendored copy of PX4's `px_uploader.py`, `upload_log.py` and
`ecl_ekf` (BSD-3-Clause); only `f` and the log upload/check need it.
