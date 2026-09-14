#!/usr/bin/env bash
# Build a standalone, single-file lazypx4 executable with PyInstaller.
#
# The result (dist/lazypx4) bundles Python and all pip dependencies, so it
# runs on this machine without activating .venv or having Python installed
# system-wide. Optional features that come from outside pip - the ROS 2
# clock (rclpy, sourced via PYTHONPATH from a ROS install) and satellite-map
# pin annotation (Pillow, only if the venv has it) - degrade gracefully if
# unavailable, matching normal (non-frozen) behavior.
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
    echo "error: .venv not found - create it and install lazypx4's dependencies first" >&2
    exit 1
fi

source .venv/bin/activate
python -m pip show pyinstaller >/dev/null 2>&1 || pip install pyinstaller -q

pyinstaller --onefile -y -n lazypx4 \
    --collect-submodules pymavlink \
    --paths . \
    pyinstaller_entry.py

echo
echo "Built: $(pwd)/dist/lazypx4"
echo "Copy it anywhere on your PATH, e.g.: cp dist/lazypx4 ~/.local/bin/"
