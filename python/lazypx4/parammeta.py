"""PX4 parameter metadata (descriptions, enum / bitmask meanings, units).

The MAVLink parameter protocol only carries a name and a number, so
``EKF2_HGT_REF = 1`` says nothing about *what* 1 means. QGroundControl gets
that from PX4's ``parameters.json``; this module reads the same file. A
compact copy generated from a PX4 build ships in ``lazypx4/data`` (see
``scripts/make_param_meta.py``), and ``--param-defaults FILE`` can point at the
``parameters.json`` of the exact firmware in use instead, which wins.

Compact format: ``{"px4": "v1.17.0", "params": {NAME: entry}}`` where an entry
has any of ``s`` short description, ``l`` long description, ``u`` units,
``min`` / ``max`` / ``d`` (default), ``e`` enum ``[[value, text], ...]``,
``b`` bitmask ``[[bit, text], ...]`` and ``r`` (reboot required).
"""

from __future__ import annotations

import json
import lzma
import os

_BUNDLED = os.path.join(os.path.dirname(__file__), "data", "param_meta.json.xz")

_meta = None  # {name: entry}, loaded on first use
_source = ""


def compact_from_px4(data):
    """PX4 ``parameters.json`` (already parsed) -> the compact entry dict."""
    params = {}
    for p in data.get("parameters", []):
        name = p.get("name")
        if not name:
            continue
        entry = {}
        for src, dst in (
            ("shortDesc", "s"), ("longDesc", "l"), ("units", "u"),
            ("min", "min"), ("max", "max"), ("default", "d"),
        ):
            if p.get(src) not in (None, ""):
                entry[dst] = p[src]
        if p.get("values"):
            entry["e"] = [[v["value"], v.get("description", "")] for v in p["values"]]
        if p.get("bitmask"):
            entry["b"] = [[b["index"], b.get("description", "")] for b in p["bitmask"]]
        if p.get("rebootRequired"):
            entry["r"] = True
        params[name] = entry
    return params


def _read(path):
    """Read a PX4 ``parameters.json`` (plain or ``.xz``) or our compact file."""
    opener = lzma.open if path.endswith(".xz") else open
    with opener(path, "rt", encoding="utf-8") as file:
        data = json.load(file)
    if isinstance(data.get("parameters"), list):
        return compact_from_px4(data)
    return data.get("params", {})


def load(path=None):
    """(Re)load the metadata; ``path`` overrides the bundled file. Never
    raises - a missing or bad file just leaves descriptions off."""
    global _meta, _source
    for candidate in (path, _BUNDLED):
        if not candidate or not os.path.isfile(candidate):
            continue
        try:
            _meta = _read(candidate)
            _source = candidate
            return
        except Exception:
            continue
    _meta = {}
    _source = ""


def get(name):
    """The metadata entry for a parameter, or ``{}``."""
    if _meta is None:
        load()
    return _meta.get(name, {})


def source():
    return _source


def enum_label(entry, value):
    """Text for ``value`` in an enum parameter, or None (not an enum / no match)."""
    for enum_value, text in entry.get("e", ()):
        if abs(float(enum_value) - value) < 1e-6:
            return text
    return None


def bitmask_labels(entry, value):
    """Texts of the set bits of a bitmask parameter, or None (not a bitmask)."""
    bits = entry.get("b")
    if not bits:
        return None
    try:
        raw = int(round(value))
    except (ValueError, OverflowError):
        return None
    return [text for bit, text in bits if raw >> bit & 1]


def meaning(entry, value):
    """Short read-out for the value column: the enum label, the set bitmask
    bits, or the unit. Empty string if there is nothing to say."""
    if "e" in entry:
        label = enum_label(entry, value)
        return label if label is not None else "?"
    labels = bitmask_labels(entry, value)
    if labels is not None:
        return ", ".join(labels) if labels else "none"
    return entry.get("u", "")
