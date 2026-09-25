"""Small, dependency-free numeric helpers used across the package."""

from __future__ import annotations

import math


def safe_int(value, default=0):
    """int(value), or ``default`` if the conversion raises."""
    try:
        return int(value)
    except Exception:
        return default


def safe_float(value, default=0.0):
    """float(value), or ``default`` if the conversion raises."""
    try:
        return float(value)
    except Exception:
        return default


def clamp(value, minimum, maximum):
    """Constrain ``value`` to the inclusive range [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def finite(value, default=0.0):
    """Return ``value`` as a finite float, falling back to ``default``.

    PX4 sends NaN/Inf in message fields it is not populating (unused
    POSITION_TARGET axes, a diverged estimate, ...). Anything that reaches
    ``int(round(...))`` or a fixed-width format must be finite first.
    """
    value = safe_float(value, default)
    return value if math.isfinite(value) else default
