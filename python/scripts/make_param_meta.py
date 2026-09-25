#!/usr/bin/env python3
"""Regenerate the bundled param_meta.json.xz from a PX4 build's parameters.json.

Writes both copies: the Python package's lazypx4/data/ and the Rust crate's
../data/ (embedded into the binary at build time). Run from python/:

    python scripts/make_param_meta.py PX4-Autopilot/build/px4_fmu-v6x_default/parameters.json v1.17.0
"""
import json
import lzma
import sys

from lazypx4.parammeta import compact_from_px4

src, version = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "")
with open(src, encoding="utf-8") as file:
    params = compact_from_px4(json.load(file))

for out in ("lazypx4/data/param_meta.json.xz", "../data/param_meta.json.xz"):
    with lzma.open(out, "wt", encoding="utf-8", preset=9) as file:
        json.dump({"px4": version, "params": params}, file, separators=(",", ":"))
    print(f"{len(params)} parameters -> {out}")
