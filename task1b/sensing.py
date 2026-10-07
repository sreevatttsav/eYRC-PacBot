"""Sensing layer for Task 1B (Phase 0/1).

Tof: per-sensor validity filter. Rejects invalid samples (None, NaN,
<= 0, or above MAX_RANGE -- the sim's huge invalid returns land here),
holds the last good value for hold_s, then reports "unknown" (None).

Status strings: "valid" (fresh sample this tick), "sat" (fresh sample
at the saturation cap -- a LOWER BOUND, not a distance), "held"
(coasting on the last good value inside the hold window), "none"
(unknown). Consumers must handle None AND sat -- see the validity
rules in task_1b_boilerplate.update().

sat_cap=None disables saturation handling (SAT_MODE=distance). With a
cap set, a raw reading within sat_eps of the cap reports (cap, "sat")
and latches: a hold following a sat still reports "sat", never
"held", so downstream never differences a bound.

StuckDetector: compares signed means of commanded vs measured yaw over
a 1 s window. Flags stuck when |commanded| > cmd_min and |measured| <
ratio * |commanded|. Mean (not RMS) is used deliberately: wheel jitter
has zero mean, so RMS-based tests fire on noise alone.

Imported by task_1b_boilerplate.py and replay.py (single source of
truth for both).
"""
import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class WallObservation:
    left: str
    right: str
    front: str
    hazard: str
    entrance: bool

    def wall(self, side):
        return self.left == "wall" if side > 0 else self.right == "wall"

    def opening(self, side):
        return self.left == "open" if side > 0 else self.right == "open"


class WallPerception:
    """Debounced wall and front-path classifications from fresh measurements."""

    def __init__(self, wall_max=0.14, open_min=0.20, dwell=0.12):
        self.wall_max, self.open_min, self.dwell = wall_max, open_min, dwell
        self.side = ["uncertain", "uncertain"]
        self.candidate = [None, None]
        self.elapsed = [0.0, 0.0]
        self.blocked_t = 0.0

    def update(self, ranges, statuses, dt, front_cos=1.0):
        fl, fr, sl, sr = ranges
        fs_l, fs_r, ss_l, ss_r = statuses
        for i, (v, status) in enumerate(((sl, ss_l), (sr, ss_r))):
            proposal = ("wall" if v <= self.wall_max else
                        "open" if v >= self.open_min else None) if (
                            v is not None and status == "valid") else None
            if proposal is None:
                self.candidate[i], self.elapsed[i] = None, 0.0
            elif proposal == self.side[i]:
                self.candidate[i], self.elapsed[i] = None, 0.0
            else:
                self.elapsed[i] = self.elapsed[i] + dt if self.candidate[i] == proposal else dt
                self.candidate[i] = proposal
                if self.elapsed[i] >= self.dwell:
                    self.side[i] = proposal
                    self.candidate[i], self.elapsed[i] = None, 0.0
        l = fl * front_cos if fs_l == "valid" and fl is not None else None
        r = fr * front_cos if fs_r == "valid" and fr is not None else None
        emergency = any(v is not None and v <= 0.08 for v in (l, r))
        both_close = l is not None and r is not None and l < 0.15 and r < 0.15
        self.blocked_t = self.blocked_t + dt if both_close else 0.0
        front_center = ((l + r) * 0.5
                        if l is not None and r is not None else None)
        # The two rays are splayed. The nearer ray may still see the wall
        # being rotated away from while the robot's centreline is open. Use a
        # centreline estimate, but retain a near-contact guard so one extreme
        # ray cannot falsely declare a path clear.
        front_clear = (
            front_center is not None
            and front_center >= self.open_min
            and min(l, r) >= 0.10
        )
        front = "blocked" if self.blocked_t >= self.dwell else (
            "clear" if front_clear else "uncertain")
        hazard = "emergency" if emergency else (
            "blocked_pair" if front == "blocked" else
            "single_ray" if (l is not None and l < 0.15) or
            (r is not None and r < 0.15) else "none")
        observed = [self.side[i] if status == "valid" else "uncertain"
                    for i, status in enumerate((ss_l, ss_r))]
        entrance = (l is not None and r is not None and l < 0.15 and r < 0.15
                    and abs(l-r) < 0.03 and observed == ["open", "open"])
        return WallObservation(*observed, front, hazard, entrance)


class Tof:
    def __init__(self, max_range=2.0, hold_s=0.1, tau=0.05,
                 sat_cap=None, sat_eps=0.002):
        self.max_range = max_range
        self.hold_s = hold_s
        self.tau = tau
        self.sat_cap = sat_cap
        self.sat_eps = sat_eps
        self.val = None
        self.age = 0.0
        self.status = "none"
        self.sat_latched = False

    def update(self, raw, dt):
        dt = dt if dt and dt > 0 else 0.0
        if (self.sat_cap is not None and raw is not None
                and raw == raw and abs(raw - self.sat_cap) <= self.sat_eps):
            self.val = self.sat_cap
            self.age = 0.0
            self.sat_latched = True
            self.status = "sat"
            return self.val, "sat"
        ok = (raw is not None and raw == raw
              and 0.0 < raw <= self.max_range)
        if ok:
            a = dt / (self.tau + dt) if dt > 0 else 1.0
            self.val = raw if self.val is None else self.val + a * (raw - self.val)
            self.age = 0.0
            self.sat_latched = False
            self.status = "valid"
        else:
            self.age += dt
            if self.val is not None and self.age <= self.hold_s:
                self.status = "sat" if self.sat_latched else "held"
            else:
                self.status = "none"
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

    def reset(self):
        """Discard a partial window when the controller changes modes."""
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
