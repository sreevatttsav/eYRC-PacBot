"""Small, explicitly uncertain lattice estimate for the PacBot maze.

Coordinates are relative to the first detected side-wall edge.  They are
bookkeeping, not a claim that wheel commands provide accurate global pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


PITCH = 0.22
SIDE_AHEAD_OF_AXLE = 0.028
FRONT_AXLE_OFFSET = 0.069
FRONT_COS = math.cos(math.radians(20.0))
K_LINEAR = 0.017
WALL_TARGET = 0.051


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Range:
    value: float | None
    kind: str  # distance, bound (0.300 saturation), or unknown

    @classmethod
    def read(cls, raw: object) -> "Range":
        try:
            value = float(raw)
        except (ValueError, TypeError):
            return cls(None, "unknown")
        if not math.isfinite(value) or value <= 0.0 or value > 2.0:
            return cls(None, "unknown")
        if abs(value - 0.300) <= 0.001:
            return cls(0.300, "bound")
        return cls(value, "distance")

    @property
    def distance(self) -> float | None:
        return self.value if self.kind == "distance" else None

    @property
    def open(self) -> bool:
        return self.kind == "bound" or (
            self.distance is not None and self.distance >= 0.20
        )

    @property
    def wall(self) -> bool:
        return self.distance is not None and self.distance <= 0.16


@dataclass(frozen=True)
class Frame:
    fl: Range
    fr: Range
    sl: Range
    sr: Range
    yaw: float | None
    dt: float

    @classmethod
    def from_payload(cls, data: dict) -> "Frame":
        gyro = data.get("gyro")
        try:
            yaw = float(gyro[2])
            if not math.isfinite(yaw):
                yaw = None
        except (TypeError, IndexError, ValueError):
            yaw = None
        try:
            dt = float(data.get("dt", 0.0))
        except (TypeError, ValueError):
            dt = 0.0
        if not math.isfinite(dt) or dt <= 0.0 or dt > 0.2:
            dt = 0.0
        return cls(*(Range.read(data.get(k)) for k in ("fl", "fr", "sl", "sr")),
                   yaw, dt)

    @property
    def front_wall_range(self) -> float | None:
        """Axial projection, only when two rays plausibly see one plane."""
        left, right = self.fl.distance, self.fr.distance
        if left is None or right is None or abs(left - right) > 0.035:
            return None
        return 0.5 * (left + right) * FRONT_COS

    @property
    def front_hazard(self) -> bool:
        return any(v is not None and v * FRONT_COS < 0.055
                   for v in (self.fl.distance, self.fr.distance))

    @property
    def front_open(self) -> bool:
        return all(r.open or (r.distance is not None and
                   r.distance * FRONT_COS >= 0.13) for r in (self.fl, self.fr))

    @property
    def doorway_signature(self) -> bool:
        r = self.front_wall_range
        return (r is not None and 0.06 <= r < 0.15 and
                self.sl.open and self.sr.open)

    @property
    def entrance_probe(self) -> bool:
        """One splayed ray can remain on a gate post after the other clears."""
        sides = (self.sl, self.sr)
        side_room = all(r.kind == "bound" or
                        (r.distance is not None and r.distance >= 0.14)
                        for r in sides)
        fronts_seen = all(r.kind != "unknown" for r in (self.fl, self.fr))
        return side_room and fronts_seen and not self.front_hazard


class LatticeEstimator:
    """Gyro heading plus short commanded-distance segments and wall anchors."""

    def __init__(self) -> None:
        self.psi = 0.0
        self.psi_valid = False
        self.y: float | None = None
        self.both_walls = False
        self.s: float | None = None
        self.cell = (0, 0)
        self.heading = 0  # east, north, west, south
        self.crossings = 0
        self.edge_count = 0
        self.last_edge_travel: float | None = None
        self.edge_spacings: list[float] = []
        self.duplicate_edges = 0
        self.front_count = 0
        self.rejected_anchors = 0
        self.travel = 0.0
        self.side_state: list[str | None] = [None, None]
        self.side_candidate: list[str | None] = [None, None]
        self.side_dwell = [0.0, 0.0]
        self.slope_sample: tuple[float, float | None, float | None] | None = None

    @staticmethod
    def _side_class(r: Range) -> str | None:
        if r.wall:
            return "wall"
        if r.open:
            return "open"
        return None

    def _edge(self, frame: Frame, ds: float) -> bool:
        crossed = False
        for index, r in enumerate((frame.sl, frame.sr)):
            observed = self._side_class(r)
            if observed is None:
                self.side_candidate[index] = None
                self.side_dwell[index] = 0.0
                continue
            if observed == self.side_state[index]:
                self.side_candidate[index] = None
                self.side_dwell[index] = 0.0
                continue
            if observed != self.side_candidate[index]:
                self.side_candidate[index] = observed
                self.side_dwell[index] = 0.0
            self.side_dwell[index] += frame.dt
            if self.side_dwell[index] < 0.04:
                continue
            old = self.side_state[index]
            self.side_state[index] = observed
            self.side_candidate[index] = None
            self.side_dwell[index] = 0.0
            if old is not None and old != observed:
                crossed = True
        if crossed and ds > 0.0:
            if (self.last_edge_travel is not None and
                    self.travel - self.last_edge_travel < 0.10):
                # The left and right rays often cross the same wall end at
                # slightly different times. They are one lattice landmark.
                self.duplicate_edges += 1
                return False
            expected = PITCH - SIDE_AHEAD_OF_AXLE
            if self.s is None or abs(self.s - expected) <= 0.075:
                self.s = expected
                self.edge_count += 1
                if self.last_edge_travel is not None:
                    self.edge_spacings.append(self.travel - self.last_edge_travel)
                self.last_edge_travel = self.travel
                return True
            self.rejected_anchors += 1
        return False

    def _anchor_front(self, frame: Frame) -> None:
        r = frame.front_wall_range
        if r is None or r >= 0.15 or not (frame.sl.wall or frame.sr.wall):
            return
        estimate = PITCH - (r + FRONT_AXLE_OFFSET)
        if not -0.02 <= estimate <= PITCH:
            return
        if self.s is None or abs(self.s - estimate) <= 0.075:
            self.s = estimate
            self.front_count += 1
            # Equal wall rays are an absolute corridor-heading landmark.
            if self.psi_valid and self.fl_fr_heading(frame) is not None:
                observed = self.fl_fr_heading(frame)
                self.psi += 0.04 * (observed - self.psi)
        else:
            self.rejected_anchors += 1

    @staticmethod
    def fl_fr_heading(frame: Frame) -> float | None:
        left, right = frame.fl.distance, frame.fr.distance
        if left is None or right is None or abs(left - right) > 0.035:
            return None
        mean = 0.5 * (left + right)
        if mean <= 0.045 or mean > 0.16:
            return None
        angle = math.radians(20.0)
        return clamp((left - right) * math.cos(angle) /
                     (2.0 * mean * math.sin(angle)), -0.25, 0.25)

    def _anchor_slope(self, frame: Frame, ds: float) -> None:
        if ds <= 0.0 or self.s is None or not self.psi_valid:
            self.slope_sample = None
            return
        l = frame.sl.distance if frame.sl.wall else None
        r = frame.sr.distance if frame.sr.wall else None
        if self.slope_sample is None:
            self.slope_sample = (self.travel, l, r)
            return
        start, old_l, old_r = self.slope_sample
        distance = self.travel - start
        if distance < 0.04:
            return
        estimates = []
        if l is not None and old_l is not None:
            estimates.append(-math.atan2(l - old_l, distance))
        if r is not None and old_r is not None:
            estimates.append(math.atan2(r - old_r, distance))
        if estimates and all(abs(v) <= 0.35 for v in estimates):
            implied = sum(estimates) / len(estimates)
            if abs(implied - self.psi) <= 0.30:
                self.psi += 0.08 * (implied - self.psi)
        self.slope_sample = (self.travel, l, r)

    def observe(self, frame: Frame, previous_wheels: tuple[float, float],
                anchor: bool = True) -> bool:
        """Return True when a new side edge re-anchors the cell position."""
        if frame.dt <= 0.0:
            return False
        if frame.yaw is not None:
            self.psi += frame.yaw * frame.dt
            self.psi_valid = True
        v = K_LINEAR * sum(previous_wheels) / 2.0
        ds = v * frame.dt * (math.cos(self.psi) if self.psi_valid else 1.0)
        self.travel += ds
        if self.s is not None:
            self.s += ds
            while self.s >= PITCH:
                self.s -= PITCH
                self.crossings += 1
                dx, dy = ((1, 0), (0, 1), (-1, 0), (0, -1))[self.heading]
                self.cell = (self.cell[0] + dx, self.cell[1] + dy)
            while self.s < 0.0:
                self.s += PITCH
                self.crossings -= 1
                dx, dy = ((1, 0), (0, 1), (-1, 0), (0, -1))[self.heading]
                self.cell = (self.cell[0] - dx, self.cell[1] - dy)
        edge = self._edge(frame, ds) if anchor and abs(self.psi) < 0.30 else False
        if anchor:
            self._anchor_front(frame)
        self.both_walls = frame.sl.wall and frame.sr.wall
        if self.psi_valid:
            correction = math.cos(self.psi)
            if self.both_walls:
                self.y = 0.5 * (frame.sl.distance - frame.sr.distance) * correction
            elif frame.sl.distance is not None and frame.sl.distance <= 0.12:
                self.y = (frame.sl.distance - WALL_TARGET) * correction
            elif frame.sr.distance is not None and frame.sr.distance <= 0.12:
                self.y = (WALL_TARGET - frame.sr.distance) * correction
            else:
                self.y = None
        else:
            self.y = None
        if anchor and abs(v) >= 0.03:
            self._anchor_slope(frame, ds)
        else:
            self.slope_sample = None
        return edge

    def finish_turn(self, quarter_turns: int, centered: bool) -> None:
        self.psi -= quarter_turns * math.pi / 2.0
        self.heading = (self.heading + quarter_turns) % 4
        self.s = PITCH / 2.0 if centered else None
        self.y = None
        self.side_state = [None, None]
        self.side_candidate = [None, None]
        self.side_dwell = [0.0, 0.0]
        self.slope_sample = None
