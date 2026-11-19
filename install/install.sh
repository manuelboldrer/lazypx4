#!/usr/bin/env bash
# Build and install lazypx4 (Rust) into ~/.local/bin, plus a .venv for the
# vendored PX4 scripts under Tools/ ([f] flash, log upload, EKF check).
#
#   install/install.sh            # plain build, no ROS dependency
#   install/install.sh --ros      # with the ROS 2 screens (source ROS 2 first)
#   install/install.sh --no-deps  # skip the system packages (apt) step
#   install/install.sh --yes      # don't ask before installing (for scripts)
#
# Flags can be combined. On Debian/Ubuntu the missing system packages lazypx4
# builds with and shells out to are installed with apt (needs sudo), and Rust
# with rustup if cargo is missing - each after asking first.
set -euo pipefail
cd "$(dirname "$0")/.."

ros=0
deps=1
yes=0
for arg in "$@"; do
    case "$arg" in
        --ros) ros=1 ;;
        --no-deps) deps=0 ;;
        -y|--yes) yes=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "error: unknown option $arg (try --help)" >&2; exit 1 ;;
    esac
done

if [ "$ros" = 1 ] && [ -z "${ROS_DISTRO:-}" ]; then
    echo "error: --ros needs a sourced ROS 2 install (e.g. source /opt/ros/jazzy/setup.bash)" >&2
    exit 1
fi

# ask "question" -> 0 for yes. Default yes; with no terminal to ask on (and no
# --yes) the answer is no, so a piped / CI run never installs behind your back.
ask() {
    [ "$yes" = 1 ] && return 0
    if [ ! -t 0 ]; then
        echo "$1 -> skipped (no terminal to ask; rerun with --yes)" >&2
        return 1
    fi
    local reply
    read -r -p "$1 [Y/n] " reply
    case "$reply" in ""|[Yy]*) return 0 ;; *) return 1 ;; esac
}

# --- system dependencies -----------------------------------------------------
# build:    C toolchain + pkg-config (xz2 / image), python3-venv for Tools/
# --ros:    clang / libclang (r2r generates its bindings with bindgen)
# runtime:  all optional - lazypx4 greys out what is missing
#   tmux            pane recording, fleet tmux layout
#   iputils-ping    link latency pings ([u] host screen, recording)
#   iproute2, iw    host screen: interfaces, neighbours, wifi
#   lm-sensors      CPU temperatures (`sensors`)
#   ffmpeg          screen recording on X11
#   wf-recorder     screen recording on wlroots Wayland
#   speech-dispatcher / pulseaudio-utils / alsa-utils   spoken alerts + sounds
#                   (Piper voices are used instead when installed)
#   wl-clipboard / xclip                                copy to clipboard
#   curl            flight log upload, rustup
build_pkgs=(build-essential pkg-config python3 python3-venv python3-pip curl git)
ros_pkgs=(clang libclang-dev)
run_pkgs=(tmux iputils-ping iproute2 iw lm-sensors ffmpeg wf-recorder
          speech-dispatcher pulseaudio-utils alsa-utils wl-clipboard xclip)

if [ "$deps" = 1 ]; then
    pkgs=("${build_pkgs[@]}" "${run_pkgs[@]}")
    [ "$ros" = 1 ] && pkgs+=("${ros_pkgs[@]}")
    if command -v apt-get >/dev/null; then
        # Only what is missing; skip packages this release doesn't ship (e.g.
        # wf-recorder on old Ubuntu) instead of failing the whole install.
        missing=()
        for p in "${pkgs[@]}"; do
            dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "ok installed" && continue
            if apt-cache show "$p" >/dev/null 2>&1; then missing+=("$p"); else echo "  (skipping $p: not available)"; fi
        done
        if [ "${#missing[@]}" -eq 0 ]; then
            echo "System packages: all installed."
        else
            echo "Missing system packages:"
            echo "  ${missing[*]}"
            if ask "Install them now with sudo apt-get?"; then
                sudo=""
                [ "$(id -u)" -ne 0 ] && sudo="sudo"
                $sudo apt-get update -qq
                $sudo apt-get install -y -qq "${missing[@]}"
            else
                echo "Not installing them; the build may fail and some features will be greyed out."
            fi
        fi
    else
        echo "warning: no apt-get - install these (or your distro's equivalents) yourself:" >&2
        echo "  ${pkgs[*]}" >&2
    fi
fi

features=()
if [ "$ros" = 1 ]; then
    # Only generate the message types lazypx4 uses (much faster build).
    export IDL_PACKAGE_FILTER="std_msgs;std_srvs;geometry_msgs;builtin_interfaces;rosgraph_msgs;rcl_interfaces"
    features=(--features ros)
fi

# Rust: rustup's cargo may be installed but not on PATH yet in this shell.
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
if ! command -v cargo >/dev/null; then
    if [ "$deps" = 1 ] && command -v curl >/dev/null \
        && ask "cargo not found. Install Rust with rustup (into ~/.cargo, no sudo)?"; then
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
        . "$HOME/.cargo/env"
    else
        echo "error: cargo not found - install Rust from https://rustup.rs first" >&2
        exit 1
    fi
fi

cargo build --release "${features[@]}"
mkdir -p ~/.local/bin
ln -sf "$(pwd)/target/release/lazypx4" ~/.local/bin/lazypx4

# Python venv for the Tools/ scripts; lazypx4 picks up ./.venv next to Tools/
# on its own, no need to activate it.
python3 -m venv .venv
.venv/bin/pip install -q -r Tools/requirements.txt
# optional: the [u] host screen's internet speed test ([i]) runs `speedtest`
.venv/bin/pip install -q speedtest-cli
command -v speedtest >/dev/null || ln -sf "$(pwd)/.venv/bin/speedtest" ~/.local/bin/speedtest

echo
echo "Built:     $(pwd)/target/release/lazypx4"
echo "Symlinked: ~/.local/bin/lazypx4 -> target/release/lazypx4"
