# lazypx4

Fly and monitor PX4 drones from a terminal — no mouse, no GUI, works over SSH.

![lazypx4 demo](docs/demo.gif)

![lazypx4 rbl demo](docs/demo_rbl.gif)

## Why

QGroundControl is great when you have one drone, a screen and a mouse.
lazypx4 is for everything else: logged into a companion computer over SSH,
watching several drones at once, or working on a robot with no display.

It's a small program in the spirit of [lazygit](https://github.com/jesseduffield/lazygit)
and [lazydocker](https://github.com/jesseduffield/lazydocker), inspired by
[mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status.git).

## What it does

- **Dashboard** — GPS, battery, position, flight mode, and the health of the onboard computer, all on one screen
- **Flight control** — arm, take off, land, return home, change modes, fly to a point
- **Preflight check** — a single GO / NO-GO screen before you fly
- **Alerts** — flashing warning and a beep on link loss, low battery or failsafes
- **Mission map** — see where the drone is, load waypoints and geofences
- **Parameters, logs, calibration, firmware** — the usual setup tasks, without leaving the terminal
- **Fleet view** — watch and command several drones side by side
- **Recording & replay** — every session is saved and can be played back later
- **ROS 2 (optional)** — shows LiDAR scans and planned paths on the map

Everything is a single keypress away, with vim-style navigation. Anything that
changes the drone's state asks you to type `YES` first.

## Install

You need [Rust](https://rustup.rs).

```bash
git clone https://github.com/manuelboldrer/lazypx4.git
cd lazypx4
install/install.sh
```

For ROS 2 support, run `source /opt/ros/jazzy/setup.bash` and then
`install/install.sh --ros`.

The result is a single small program you can copy to any companion computer.

## Run

```bash
lazypx4                  # connect to a drone sending MAVLink on port 14560
lazypx4 --port 14540     # PX4 simulator (SITL)
lazypx4 --fleet 14560-14563   # several drones at once
lazypx4 --help           # all options
```

On a real drone, tell PX4 where to send data by adding this line to
`etc/extras.txt` on its SD card:

```bash
mavlink start -u 14560 -o 14560 -t <your-computer-ip> -m onboard -r 4000000
```

## Most-used keys

| Key | Action |
|-----|--------|
| `a` / `d` | Arm / disarm |
| `T` / `L` / `R` / `h` | Take off / land / return home / hold |
| `y` | Preflight check |
| `n` | Map |
| `p` | Parameters |
| `l` | Flight logs |
| `:` | Type a command (e.g. `:takeoff 5`) |
| `?` | Help for the current screen |
| `q` | Quit / go back |

The full list of keys, commands and settings is in
[docs/reference.md](docs/reference.md). Defaults can be changed in
[`lazypx4.toml`](lazypx4.toml).

## Status

Under active testing — feedback is welcome, please
[open an issue](https://github.com/manuelboldrer/lazypx4/issues) or send a PR.

## Author

[Manuel Boldrer](https://manuelboldrer.github.io/) - manuel.boldrer@gmail.com
Saxion University of Applied Sciences, [Smart Mechatronics and Robotics Group](https://www.saxion.edu/research/research-groups/smart-mechatronics-and-robotics).
Developed with assistance from Anthropic's Claude AI.

## License

MIT - see [LICENSE](LICENSE).
