# lazypx4-rs

A Rust / [ratatui](https://ratatui.rs) port of [lazypx4](../README.md): the vim-style,
keyboard-only TUI for PX4 over MAVLink. Every screen of the Python version is ported.

## Build and run

```sh
cargo build --release
./target/release/lazypx4-rs            # listens on udpin:0.0.0.0:14560, like lazypx4
./target/release/lazypx4-rs -p 14550   # e.g. PX4 SITL's GCS link
```

The default build is a single ~5 MB binary that needs only glibc. There's no
Python and no ROS, so you can copy it straight onto a companion computer. The
ROS 2 screens then say they're unavailable.

### With ROS 2 (`[v]` LiDAR, `[w]` camera, map NAVPATH / FIRE overlays, `[P]` publish, ROS clock)

```sh
source /opt/ros/jazzy/setup.bash
# optional: only generate the message types lazypx4 uses (much faster build)
export IDL_PACKAGE_FILTER="std_msgs;sensor_msgs;nav_msgs;geometry_msgs;builtin_interfaces;rosgraph_msgs;rcl_interfaces"
cargo build --release --features ros
```

This uses [r2r](https://github.com/sequenceplanner/r2r), which goes through
`rcl`, so it talks over whatever RMW your sourced ROS install uses (Fast DDS,
Cyclone, Zenoh...), just like rclpy. The ROS build links against ROS
libraries, so **source ROS before running it**. `--use-sim-time` makes the ROS
clock follow `/clock`.

Unlike the Python version, the UI comes up immediately and shows
`WAITING FOR HEARTBEAT` until a vehicle appears.

## Screens

| Key | Screen |
|-----|--------|
| (home) | Dashboard: link, FW, ROS / FCU clocks, mode, arm, PREARM / timeout / fence alerts, HOST / SVC lines, position, attitude, altitude, rangefinder, battery, GPS / EKF / RC, sensors, vibration |
| `a` `d` `T` `L` `R` `h` `K` `H` `G` `E` | arm / disarm / takeoff / land / RTL / hold / kill / set home / geofence action / EKF reset, each behind `type YES` |
| `n` | Mission map: plan view with trail, EKF vs GNSS, KML overlay (`o`), goto (`g`: relative / local / global / fire), keyboard jog (`x`), waypoint queue (`W`), lawnmower coverage (`C`), fence upload (`O`), satellite snapshot (`i`), LiDAR (`V`) / navpath (`N`) overlays, `[P]` GPS publish |
| `m` | Flight modes: legacy PX4 modes and the Standard Modes Protocol |
| `p` | Parameters: browse, `/` search, edit, ALL / CHANGED, reboot, key-parameter panel, meanings and descriptions |
| `g` | Event log |
| `l` | Flight logs: list, download, `u` upload to logs.px4.io, `a` ecl_ekf health check |
| `t` | NSH shell |
| `v` / `w` | ROS 2 LiDAR point cloud (top / front / 45° / free camera) and camera preview (truecolor or low-bandwidth ASCII) |
| `u` | USB / network: interfaces, Wi-Fi, ARP neighbours, Tailscale, USB devices, kernel USB log, on-demand speed test |
| `c` / `r` | Control / setpoints, RC sticks and channels, wind, actuator outputs |
| `s` | Sensor calibration |
| `f` | Flash firmware via `px_uploader.py` |
| `?` | About |

Navigation is the same as the Python version: TAB toggles the sidebar, `j`/`k`
and Ctrl-D/U/F/B move, `/` searches with `n`/`N` for next and previous match,
ESC focuses the panels, and `q` quits. All command-line flags match `lazypx4`
(`--kml`, `--lidar-topic`, `--camera-topic`, `--tools-dir`, ...; see `--help`).

## Tool scripts

Firmware flashing, the log upload and the EKF check still run the real PX4
scripts in `../Tools`, same as the Python version. The interpreter is chosen
in this order: `$VIRTUAL_ENV`, then the repo's `.venv` next to the tools
directory, then `python3`. Use `--tools-dir` when you don't start the binary
from the repo root.

## Layout

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

The parameter metadata comes in through `include_bytes!` from the Python package's
`lazypx4/data/param_meta.json.xz`, so the crate has to stay inside this repo, or
you copy that file next to it.

## Notes

- **NSH input is edited locally and sent on ENTER** (↑/↓ history, ←/→ editing),
  like PX4's `Tools/mavlink_shell.py` and QGC's console. Recent PX4
  (`66f197b6ea`, "mavlink_shell: no echo of commands") makes the MAVLink shell
  echo input itself. On SITL, `pxh` echoes it too, so forwarding every keystroke
  showed each character twice. Exactly one echo of a sent command is kept.
- rust-mavlink needs the `mav2-message-extensions` feature, or it silently drops
  MAVLink 2 extension fields (GPS accuracy, dual-antenna yaw, extra battery cells).
