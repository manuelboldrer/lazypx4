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

# Install every optional-feature extra so the frozen binary has them all: `ros` (PyYAML + numpy,
# which rclpy itself transitively imports -- see the big comment below), `map` (Pillow, for
# satellite-snapshot pin annotation) and `tools` (deps of the Tools/ scripts the [f]/[u]/[a]
# screens shell out to). Safe to (re-)run every build even if some are already installed.
pip install '.[ros,map,tools]' -q

# Deliberately build with PYTHONPATH unset, even if this shell has ROS 2 sourced (e.g. from
# ~/.bashrc). rclpy itself is designed to come from *outside* pip -- lazypx4/__init__.py appends
# $PYTHONPATH to sys.path at runtime so the frozen binary can find a real, sourced ROS 2 install's
# rclpy dynamically, complete with its compiled extension (_rclpy_pybind11), its RMW implementation
# .so, and the ament index that resolves it all -- none of which PyInstaller's static bundling can
# reproduce. If PyInstaller's own analysis phase can *see* rclpy on PYTHONPATH at build time
# instead, it tries to bundle it anyway: it finds and partially packs the pure-Python pieces but
# has no hook for the compiled pieces, so the frozen binary fails with a *different*, more
# confusing error ("No module named 'rclpy._rclpy_pybind11'") instead of cleanly falling into
# lazypx4's own "ROS 2 not available" handling. --exclude-module is a second line of defense in
# case rclpy or its C extension ever ends up genuinely pip-installed into .venv (it never should
# be -- see pyproject.toml's `ros` extra comment).
env -u PYTHONPATH pyinstaller --onefile -y -n lazypx4 \
    --collect-submodules pymavlink \
    --exclude-module rclpy \
    --exclude-module rclpy._rclpy_pybind11 \
    --paths . \
    pyinstaller_entry.py

mkdir -p ~/.local/bin
ln -sf "$(pwd)/dist/lazypx4" ~/.local/bin/lazypx4

echo
echo "Built: $(pwd)/dist/lazypx4"
echo "Symlinked: ~/.local/bin/lazypx4 -> dist/lazypx4"
