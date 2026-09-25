#!/usr/bin/env bash
# Build and install lazypx4 (Rust) into ~/.local/bin, plus a .venv for the
# vendored PX4 scripts under Tools/ ([f] flash, log upload, EKF check).
#
#   install/install.sh          # plain build, no ROS dependency
#   install/install.sh --ros    # with the ROS 2 screens (source ROS 2 first)
set -euo pipefail
cd "$(dirname "$0")/.."

features=()
if [ "${1:-}" = "--ros" ]; then
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo "error: --ros needs a sourced ROS 2 install (e.g. source /opt/ros/jazzy/setup.bash)" >&2
        exit 1
    fi
    # Only generate the message types lazypx4 uses (much faster build).
    export IDL_PACKAGE_FILTER="std_msgs;sensor_msgs;nav_msgs;geometry_msgs;builtin_interfaces;rosgraph_msgs;rcl_interfaces"
    features=(--features ros)
fi

if ! command -v cargo >/dev/null; then
    echo "error: cargo not found - install Rust from https://rustup.rs first" >&2
    exit 1
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
