# lazypx4
A vim-style, keyboard-only TUI for PX4 — built for headless, multi-vehicle ops.

![lazypx4 demo](docs/demo.gif)

![lazypx4 rbl demo](docs/demo_rbl.gif)

In the spirit of [lazygit](https://github.com/jesseduffield/lazygit) and
[lazydocker](https://github.com/jesseduffield/lazydocker), inspired by
[mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status.git).

QGroundControl is built for one operator, one vehicle and a mouse. lazypx4 is
for the other case: PX4 over SSH, from a companion computer, or across a fleet
— one lightweight process per vehicle, tiled in tmux.

- **Zero setup** — sensible defaults, works out of the box
- **Zero mouse** — every screen is one keypress away, vim navigation (`j`/`k`, `Ctrl-D`/`Ctrl-U`, `/` search)
- **Full stack** — MAVLink + ROS 2 (LiDAR, planned path) + companion-computer health (CPU/RAM/disk, USB, network, rosbag / Zenoh / uXRCE-DDS agent) on one screen

## Keys

| Key | Screen / action |
|-----|-----------------|
| (home) | Dashboard — GPS/RTK, EKF health, position, home/fence, RC, host & services |
| `a` / `d` | Arm / disarm |
| `T` / `L` / `R` / `h` | Takeoff / land / RTL / hold |
| `F` / `K` | Offboard on/off / kill (flight termination) |
| `H` / `G` / `E` | Set home / geofence action (`GF_ACTION`) / EKF reset |
| `m` | Flight modes (legacy + PX4 v1.15+ Standard Modes) |
| `n` | Mission map — see below |
| `c` / `r` | Control setpoints / RC channels, wind, actuator outputs |
| `p` | Parameters — browse, search, edit, decoded values, key-parameter panel, `s` save / `l` load a param file |
| `s` | Sensor calibration |
| `l` | Flight logs — download `.ulg`, upload to flight review, `ecl_ekf` check |
| `g` | Event log |
| `t` | MAVLink shell (NuttShell) |
| `f` | Flash firmware (`px_uploader.py`) |
| `u` | USB / network check |
| `?` | About |

Every state-changing action asks for a `type YES` confirmation. lazypx4 never
force-arms.

### Mission map (`n`)

Scale plan view with trail, home, EKF vs GNSS position and an altitude
cross-check. Flight keys work here too.

| Key | |
|-----|--|
| `g` | Goto — relative `10 0 -2`, local NED `l 5 -3 -10`, or global `g lat lon alt` |
| `x` | Keyboard jog (`k`/`j`/`a`/`d` move, `w`/`s` up/down, `h`/`l` yaw, `[`/`]` step) |
| `o` / `O` | Load a `.kml` fence + waypoints / upload the fence to PX4 |
| `W` / `C` | Fly KML waypoints (`3`, `seq`, `rand 8`) / lawnmower coverage (`5 90`) |
| `V` / `v` / `B` | LiDAR overlay / change topic / robot vs world frame |
| `N` | ROS planned-path overlay |
| `P` | Publish current position as `sensor_msgs/NavSatFix` on `/fire_gps_loc` |
| `i` | Save a satellite snapshot |
| arrows, `+`/`-`, `0`, `u`, `f` | Pan, zoom, reset, follow vehicle, fit to KML |

## Install

Rust ([ratatui](https://ratatui.rs) + [rust-mavlink](https://github.com/mavlink/rust-mavlink)); needs a [Rust toolchain](https://rustup.rs).

```bash
git clone https://github.com/manuelboldrer/lazypx4.git
cd lazypx4
install/install.sh                                         # plain build
source /opt/ros/jazzy/setup.bash && install/install.sh --ros  # with ROS 2
```

This builds `target/release/lazypx4`, links it into `~/.local/bin`, and sets up
a `.venv` for the vendored PX4 scripts in `Tools/`. Manual build:
`cargo build --release [--features ros]`.

The plain build is a single ~5 MB binary (glibc only) — copy it onto any
companion computer. The ROS 2 build (via [r2r](https://github.com/sequenceplanner/r2r),
any RMW) enables the LiDAR / path / fire overlays and `P`; source ROS before
running it.

`Tools/` is a vendored copy of PX4's `px_uploader.py`, `upload_log.py` and
`ecl_ekf` (BSD-3-Clause); only `f` and the log upload/check need it.

## Run

```bash
lazypx4                          # listen on udpin:0.0.0.0:14560
lazypx4 --port 14540             # PX4 SITL's default offboard port
lazypx4 --kml mission.kml        # preload a map overlay
lazypx4 --help                   # log/firmware/tools dirs, ROS topics, --param-defaults, ...
```

lazypx4 *listens* — PX4 must send MAVLink to it. On a real vehicle, add to
`etc/extras.txt` on the SD card:

```bash
mavlink start -u 14560 -o 14560 -t <lazypx4-host-ip> -m onboard -r 4000000
```

For serial/USB, bridge with `mavlink-router` or
`mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14560`.

On the parameter screen `s` saves every parameter in QGroundControl's
`.params` format (to `--param-dir`, default `./px4_params`) and `l` loads one
back. `l` also accepts MAVProxy-style `NAME VALUE`, NSH `param set NAME VALUE`
and YAML `NAME: VALUE` files. It shows only the values that differ and, after
`type YES`, sets them one at a time, checking each value PX4 echoes back.

Parameter descriptions come from a bundled PX4 v1.17 `parameters.json`; pass
`--param-defaults <build>/parameters.json` to match your firmware and enable
the `CHANGED` view.

## Status

Under active testing — please [open an issue](https://github.com/manuelboldrer/lazypx4/issues) or send a PR.

## Author

[Manuel Boldrer](https://manuelboldrer.github.io/) - manuel.boldrer@gmail.com
Saxion University of Applied Sciences, [Smart Mechatronics and Robotics Group](https://www.saxion.edu/research/research-groups/smart-mechatronics-and-robotics).
Developed with assistance from Anthropic's Claude AI.

## License

MIT - see [LICENSE](LICENSE).
