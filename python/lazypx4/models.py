"""Plain data records shared between the MAVLink layer and the UI."""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

from .config import ARM_CONFIRM_TIMEOUT, PARAM_VALUE_EPSILON


@dataclass
class LogEvent:
    timestamp: float
    level: str
    message: str

    @property
    def time_string(self):
        return time.strftime("%H:%M:%S", time.localtime(self.timestamp))


@dataclass
class PendingArm:
    desired_armed: bool
    sent_at: float
    timeout: float = ARM_CONFIRM_TIMEOUT

    ack_received: bool = False
    ack_result: Optional[int] = None


@dataclass
class CustomMode:
    mode_index: int
    custom_mode: int
    standard_mode: int
    properties: int
    name: str


@dataclass
class FlightLogEntry:
    id: int
    num_logs: int = 0
    last_log_num: int = 0
    time_utc: int = 0
    size: int = 0

    @property
    def time_string(self):
        if not self.time_utc:
            return "--"
        try:
            return datetime.fromtimestamp(
                self.time_utc, tz=timezone.utc
            ).strftime("%Y-%m-%d %H:%M:%S UTC")
        except (OverflowError, OSError, ValueError):
            return "--"

    @property
    def size_string(self):
        value = max(0, int(self.size))
        if value >= 1024 * 1024:
            return f"{value / (1024 * 1024):.1f} MiB"
        if value >= 1024:
            return f"{value / 1024:.1f} KiB"
        return f"{value} B"


@dataclass
class Parameter:
    name: str
    value: float = 0.0
    param_type: int = 0
    index: int = -1
    count: int = 0

    startup_value: Optional[float] = None
    default_value: Optional[float] = None

    last_update: float = 0.0

    pending: bool = False
    pending_value: Optional[float] = None
    pending_sent_at: float = 0.0

    @property
    def changed_from_startup(self):
        if self.startup_value is None:
            return False
        return abs(self.value - self.startup_value) > PARAM_VALUE_EPSILON

    @property
    def changed_from_default(self):
        if self.default_value is None:
            return None
        return abs(self.value - self.default_value) > PARAM_VALUE_EPSILON
