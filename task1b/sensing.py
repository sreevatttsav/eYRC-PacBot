"""Sensing layer for Task 1B (Phase 0/1).

Tof: per-sensor validity filter. Rejects invalid samples (None, NaN,
<= 0, or above MAX_RANGE -- the sim's huge invalid returns land here),
holds the last good value for hold_s, then reports "unknown" (None).

Status strings: "valid" (fresh sample this tick), "held" (within hold
window), "none" (unknown). Consumers must handle None -- see the
validity rules in task_1b_boilerplate.update().

StuckDetector: compares signed means of commanded vs measured yaw over
a 1 s window. Flags stuck when |commanded| > cmd_min and |measured| <
ratio * |commanded|. Mean (not RMS) is used deliberately: wheel jitter
has zero mean, so RMS-based tests fire on noise alone.

Imported by task_1b_boilerplate.py and replay.py (single source of
truth for both).
"""
import math
from collections import deque


class Tof:
    def __init__(self, max_range=2.0, hold_s=0.1, tau=0.05):
        self.max_range = max_range
        self.hold_s = hold_s
        self.tau = tau
        self.val = None
        self.age = 0.0
        self.status = "none"

    def update(self, raw, dt):
        ok = (raw is not None and raw == raw
              and 0.0 < raw <= self.max_range)
        if ok:
            a = dt / (self.tau + dt) if dt and dt > 0 else 1.0
            self.val = raw if self.val is None else self.val + a * (raw - self.val)
            self.age = 0.0
            self.status = "valid"
        else:
            self.age += dt if dt and dt > 0 else 0.0
            self.status = "held" if (self.val is not None
                                     and self.age <= self.hold_s) else "none"
        if self.status == "none":
            return None, "none"
        return self.val, self.status


class StuckDetector:
    """Windowed commanded-vs-measured yaw comparison (fix 2).

    update(cmd_yaw, meas_yaw, dt) -> bool (latched stuck flag).
    Flag goes high only on a completed window meeting the criterion,
    and clears on the first completed window that doesn't.
    """

    def __init__(self, window_s=1.0, cmd_min=0.5, ratio=0.25):
        self.window_s = window_s
        self.cmd_min = cmd_min
        self.ratio = ratio
        self.acc = 0.0
        self.sum_cmd = 0.0
        self.sum_meas = 0.0
        self.stuck = False
        self.cmd_mean = 0.0
        self.meas_mean = 0.0

    def update(self, cmd_yaw, meas_yaw, dt):
        dt = dt if dt and dt > 0 else 0.0
        self.acc += dt
        self.sum_cmd += cmd_yaw * dt
        self.sum_meas += meas_yaw * dt
        if self.acc >= self.window_s:
            self.cmd_mean = self.sum_cmd / self.acc
            self.meas_mean = self.sum_meas / self.acc
            self.stuck = (abs(self.cmd_mean) > self.cmd_min
                          and abs(self.meas_mean)
                          < self.ratio * abs(self.cmd_mean))
            self.acc = 0.0
            self.sum_cmd = 0.0
            self.sum_meas = 0.0
        return self.stuck
