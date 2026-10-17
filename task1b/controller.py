"""Cell-based Task 1B controller.  No global continuous pose is assumed."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math

from lattice import Frame, LatticeEstimator, PITCH, K_LINEAR, RangeHold, clamp


@dataclass(frozen=True)
class Settings:
    cruise_mps: float = 0.04
    crawl_mps: float = 0.015
    dock_range: float = 0.06  # start conservatively; validate before 0.045
    max_wheel: float = 10.0
    yaw_gain: float = 0.0914
    decision_dwell: float = 0.15
    decision_max_dwell: float = 1.0
    center_tolerance: float = 0.018
    pivot_center_tolerance: float = 0.010
    pivot_y_limit: float = 0.008
    pivot_180_y_limit: float = 0.005
    gate_max_travel: float = 0.45
    reverse_distance: float = 0.08


class LatticeController:
    STATES = ("GATE", "CRUISE", "DECIDE", "DOCK", "PIVOT", "RECOVER")

    def __init__(self, settings: Settings | None = None) -> None:
        self.cfg = settings or Settings()
        self.est = LatticeEstimator()
        self.state = "GATE"
        self.last_wheels = (0.0, 0.0)
        self.tick = 0
        self.sim_time = 0.0
        self.events: list[dict] = []
        self.target_crossing = 0
        self.gate_start_travel = 0.0
        self.decision_time = 0.0
        self.votes = Counter()
        self.pending_turn = 0
        self.turn_start_psi = 0.0
        self.turn_cmd_window = 0.0
        self.turn_measured_window = 0.0
        self.recovery_phase = "BACK"
        self.recovery_cycles = 0
        self.recovery_start_travel = 0.0
        self.recovery_cmd_distance = 0.0
        self.recovery_front_start: float | None = None
        self.recovery_direction = 1
        self.last_recovery_direction: int | None = None
        self.last_turn_direction = -1
        self.visits = Counter()
        self.dock_progress: tuple[float, float] | None = None
        self.range_hold = RangeHold(0.12)
        self.decision_probe_distance = 0.0
        self.hazard_dwell = 0.0
        self.recovery_probe_distance = 0.0
        self.recovery_probe_used = False
        self.pivot_centered = False
        self.unverified_gate_reported = False

    def _set_state(self, state: str, reason: str) -> None:
        assert state in self.STATES
        if state == self.state:
            return
        self.events.append({"t": round(self.sim_time, 4), "from": self.state,
                            "to": state, "reason": reason,
                            "cell": self.est.cell, "heading": self.est.heading,
                            "s": self.est.s})
        self.state = state
        if state == "GATE":
            self.unverified_gate_reported = False

    def _wheels(self, left: float, right: float) -> tuple[float, float]:
        limit = self.cfg.max_wheel
        self.last_wheels = (clamp(left, -limit, limit), clamp(right, -limit, limit))
        return self.last_wheels

    def _track(self, frame: Frame, speed: float) -> tuple[float, float]:
        """Lateral error -> desired heading -> yaw-rate servo."""
        y = self.est.y
        if speed < 0.0 or y is None or not self.est.psi_valid:
            reference = 0.0
        else:
            reference = clamp(5.0 * y, -math.radians(12), math.radians(12))
        yaw = frame.yaw if frame.yaw is not None else 0.0
        yaw_request = clamp(1.5 * (reference - self.est.psi) - 0.40 * yaw,
                            -0.30, 0.30)
        base = speed / K_LINEAR
        steer = clamp(yaw_request / (2.0 * self.cfg.yaw_gain),
                      -0.70 * abs(base), 0.70 * abs(base))
        # For reverse the lateral feedback direction is inverted.
        return self._wheels(base - steer, base + steer)

    def _begin_cruise(self, reason: str) -> None:
        self.target_crossing = self.est.crossings + 1
        self._set_state("CRUISE", reason)

    def _begin_decision(self, reason: str) -> None:
        self.decision_time = 0.0
        self.votes.clear()
        self.pending_turn = 0
        self.decision_probe_distance = 0.0
        self.recovery_cycles = 0
        self.last_recovery_direction = None
        self.recovery_probe_used = False
        self._set_state("DECIDE", reason)

    def _recover(self, frame: Frame, reason: str) -> None:
        if self.recovery_cycles >= 3:
            self.recovery_phase = "HOLD"
            self._set_state("RECOVER", "recovery_limit_hold")
            return
        self.recovery_cycles += 1
        self.recovery_phase = "BACK"
        self.recovery_start_travel = self.est.travel
        self.recovery_cmd_distance = 0.0
        self.recovery_front_start = frame.front_wall_range
        self.recovery_direction = (-self.last_recovery_direction
                                   if self.last_recovery_direction is not None
                                   else -self.last_turn_direction)
        self._set_state("RECOVER", reason)

    def _can_pivot(self, frame: Frame, quarters: int) -> bool:
        limit = (self.cfg.pivot_180_y_limit if abs(quarters) == 2
                 else self.cfg.pivot_y_limit)
        return (frame.sl.kind != "unknown" and frame.sr.kind != "unknown" and
                self.est.y is not None and abs(self.est.y) <= limit and
                self.est.psi_valid and abs(self.est.psi) <= math.radians(15) and
                self.est.s is not None and
                abs(self.est.s - PITCH / 2) <= self.cfg.pivot_center_tolerance and
                self.est.last_position_anchor_travel is not None and
                abs(self.est.travel - self.est.last_position_anchor_travel) <= PITCH and
                self.est.position_uncertainty_m is not None and
                self.est.position_uncertainty_m <= 0.035)

    def _start_pivot(self, quarters: int, reason: str) -> None:
        self.pending_turn = quarters
        self.pivot_centered = reason == "cell_center_choice"
        self.turn_start_psi = self.est.psi
        self.turn_cmd_window = 0.0
        self.turn_measured_window = 0.0
        self._set_state("PIVOT", reason)

    @staticmethod
    def _side_vote(r) -> str:
        if r.wall:
            return "wall"
        if r.open:
            return "open"
        return "unknown"

    def _vote(self, frame: Frame) -> None:
        self.votes[("left", self._side_vote(frame.sl))] += 1
        self.votes[("right", self._side_vote(frame.sr))] += 1
        self.votes[("front", frame.front_label)] += 1

    def _majority(self, direction: str) -> str:
        labels = ("wall", "open", "unknown")
        return max(labels, key=lambda label: self.votes[(direction, label)])

    def _decision(self, frame: Frame) -> tuple[float, float]:
        if self.pending_turn == 0:
            self._vote(frame)
            self.decision_time += frame.dt
            if self.decision_time < self.cfg.decision_dwell:
                return self._wheels(0.0, 0.0)
            left = self._majority("left")
            front = self._majority("front")
            right = self._majority("right")
            if left == "open":
                self.pending_turn = 1
            elif front == "open":
                self._begin_cruise("front_open")
                return self._track(frame, self.cfg.cruise_mps)
            elif right == "open":
                self.pending_turn = -1
            elif front == "wall" and left == "wall" and right == "wall":
                self.pending_turn = 2
            else:
                # Give transient ambiguity a bounded extra sample window and
                # at most 2 cm of safe forward creep. Unknown never routes.
                if self.decision_time < self.cfg.decision_max_dwell:
                    if (self.decision_probe_distance < 0.02 and
                            not frame.front_hazard):
                        distance = min(self.cfg.crawl_mps * frame.dt,
                                       0.02 - self.decision_probe_distance)
                        self.decision_probe_distance += distance
                        return self._track(frame, self.cfg.crawl_mps)
                    return self._wheels(0.0, 0.0)
                self.votes.clear()
                self.decision_time = 0.0
                return self._wheels(0.0, 0.0)
            key = (*self.est.cell, self.est.heading)
            self.visits[key] += 1
            if self.visits[key] >= 3:
                self.events.append({"t": round(self.sim_time, 4),
                                    "event": "repeated_cell_heading", "key": key,
                                    "visits": self.visits[key]})
        choice = self.pending_turn
        if self._can_pivot(frame, choice):
            self._start_pivot(choice, "cell_center_choice")
            return self._wheels(0.0, 0.0)
        # A short measured advance can finish centering before the turn.
        if (self.est.s is not None and
                self.est.s < PITCH / 2 - self.cfg.pivot_center_tolerance
                and not frame.front_hazard and
                (frame.front_wall_range is None or
                 frame.front_wall_range > self.cfg.dock_range + 0.012)):
            return self._track(frame, self.cfg.crawl_mps)
        self._recover(frame, "pivot_entry_not_centered")
        return self._wheels(0.0, 0.0)

    def _pivot(self, frame: Frame) -> tuple[float, float]:
        target = self.pending_turn * math.pi / 2.0
        achieved = self.est.psi - self.turn_start_psi
        error = target - achieved
        yaw = frame.yaw or 0.0
        if abs(error) <= math.radians(2.0) and abs(yaw) <= 0.12:
            centered = self.pivot_centered
            self.events.append({"t": round(self.sim_time, 4),
                                "event": "turn_complete",
                                "target_deg": math.degrees(target),
                                "actual_deg": math.degrees(achieved),
                                "error_deg": math.degrees(error),
                                "center_error_m": (abs(self.est.s - PITCH / 2)
                                                   if self.est.s is not None else None),
                                "lateral_m": self.est.y,
                                "recovery": not centered})
            self.est.finish_turn(self.pending_turn, centered=centered)
            self.last_turn_direction = 1 if self.pending_turn > 0 else -1
            if centered:
                self._begin_cruise("pivot_complete")
            else:
                self.gate_start_travel = self.est.travel
                self._set_state("GATE", "recovery_pivot_complete")
            self.recovery_phase = "BACK"
            return self._wheels(0.0, 0.0)
        request = clamp(1.8 * error - 0.50 * yaw, -0.55, 0.55)
        steer = clamp(request / (2.0 * self.cfg.yaw_gain), -3.0, 3.0)
        if abs(error) > math.radians(4.0) and abs(steer) < 0.55:
            steer = math.copysign(0.55, error)
        command = self._wheels(-steer, steer)
        self.turn_cmd_window += abs(2.0 * steer * self.cfg.yaw_gain) * frame.dt
        self.turn_measured_window += abs(yaw) * frame.dt
        if self.turn_cmd_window >= 0.55:
            if self.turn_measured_window < math.radians(5.0):
                self._recover(frame, "pivot_no_gyro_progress")
                return self._wheels(0.0, 0.0)
            self.turn_cmd_window = 0.0
            self.turn_measured_window = 0.0
        return command

    def _recovery(self, frame: Frame) -> tuple[float, float]:
        if self.recovery_phase == "HOLD":
            return self._wheels(0.0, 0.0)
        if self.recovery_phase == "PROBE":
            if frame.front_label != "open" or frame.front_hazard:
                self.recovery_phase = "HOLD"
                return self._wheels(0.0, 0.0)
            self.recovery_probe_distance += self.cfg.crawl_mps * frame.dt
            if self.recovery_probe_distance >= 0.02:
                self.recovery_phase = "BACK"
                self.gate_start_travel = self.est.travel
                self._set_state("GATE", "recovery_probe_complete")
                return self._wheels(0.0, 0.0)
            return self._track(frame, self.cfg.crawl_mps)
        if self.recovery_phase == "BACK":
            progressed = self.recovery_cmd_distance
            front = frame.front_wall_range
            range_progress = (front is not None and
                              self.recovery_front_start is not None and
                              front - self.recovery_front_start >=
                              self.cfg.reverse_distance)
            if not range_progress and progressed < self.cfg.reverse_distance:
                return self._track(frame, -0.025)
            self.recovery_phase = "TURN"
            direction = self.recovery_direction
            if self.last_recovery_direction is None:
                if frame.sl.open and not frame.sr.open:
                    direction = 1
                elif frame.sr.open and not frame.sl.open:
                    direction = -1
            self.last_recovery_direction = direction
            self._start_pivot(direction, "recovery_backed_by_range_or_distance")
            return self._wheels(0.0, 0.0)
        raise AssertionError("unreachable recovery phase")

    def step(self, frame: Frame) -> tuple[float, float]:
        self.tick += 1
        self.sim_time += frame.dt
        if frame.dt <= 0.0 or frame.yaw is None:
            return self._wheels(0.0, 0.0)
        if self.state == "RECOVER" and self.recovery_phase == "BACK":
            previous_average = sum(self.last_wheels) / 2.0
            if previous_average < 0.0:
                self.recovery_cmd_distance += -K_LINEAR * previous_average * frame.dt
        self.est.observe(frame, self.last_wheels,
                         anchor=self.state in ("GATE", "CRUISE", "DECIDE", "DOCK"))
        if frame.front_hazard:
            self.hazard_dwell += frame.dt
        else:
            self.hazard_dwell = 0.0
        if (self.state not in ("RECOVER", "PIVOT", "GATE") and
                self.sim_time >= 0.30 and self.hazard_dwell >= 0.10):
            self._recover(frame, "front_contact_guard")
            return self._wheels(0.0, 0.0)
        if self.state == "GATE":
            if (not self.unverified_gate_reported and
                    self.est.s is None and
                    self.est.travel - self.gate_start_travel >= 0.08):
                self.events.append({"t": round(self.sim_time, 4),
                                    "event": "translation_unverified",
                                    "state": "GATE",
                                    "commanded_m": self.est.travel - self.gate_start_travel})
                self.unverified_gate_reported = True
            if self.est.s is not None:
                self._begin_cruise("wall_anchor_found")
                return self._track(frame, self.cfg.cruise_mps)
            if self.est.travel - self.gate_start_travel > self.cfg.gate_max_travel:
                self._recover(frame, "no_gate_landmark")
                return self._wheels(0.0, 0.0)
            if self.sim_time < 0.30:
                if frame.entrance_probe or frame.front_label == "open":
                    return self._track(frame, self.cfg.crawl_mps)
                # A symmetric close pair can be the two entrance posts. Keep
                # probing while both sides show room; the persistent physical
                # contact guard below still bounds this path.
                side_room = self.est._side_class(frame.sl) == "open" and self.est._side_class(frame.sr) == "open"
                if side_room and not frame.front_hazard:
                    return self._track(frame, self.cfg.crawl_mps)
                return self._wheels(0.0, 0.0)
            if frame.front_hazard and not frame.entrance_probe and self.hazard_dwell >= 0.10:
                self._recover(frame, "front_contact_guard")
                return self._wheels(0.0, 0.0)
            if self.recovery_cycles >= 2 and not self.recovery_probe_used:
                self.recovery_phase = "PROBE"
                self.recovery_probe_distance = 0.0
                self.recovery_probe_used = True
                self._set_state("RECOVER", "recovery_forward_probe")
                return self._wheels(0.0, 0.0)
            if self.recovery_probe_used:
                self.recovery_phase = "HOLD"
                self._set_state("RECOVER", "recovery_exhausted_hold")
                return self._wheels(0.0, 0.0)
            if frame.front_label == "open" or frame.entrance_probe:
                return self._track(frame, self.cfg.crawl_mps)
            side_room = self.est._side_class(frame.sl) == "open" and self.est._side_class(frame.sr) == "open"
            if side_room and not frame.front_hazard:
                return self._track(frame, self.cfg.crawl_mps)
            if self.recovery_cycles:
                self._recover(frame, "gate_front_uncertain")
            else:
                return self._wheels(0.0, 0.0)
            return self._wheels(0.0, 0.0)
        if self.state == "CRUISE":
            if self.est.s is None:
                self.gate_start_travel = self.est.travel
                self._set_state("GATE", "lost_cell_anchor")
                return self._wheels(0.0, 0.0)
            front = frame.front_wall_range
            if front is not None and front < 0.105:
                self.dock_progress = (self.est.travel, front)
                self._set_state("DOCK", "front_wall_near_center")
                return self._wheels(0.0, 0.0)
            if (self.est.crossings >= self.target_crossing and
                    self.est.s >= PITCH / 2 - self.cfg.center_tolerance):
                self._begin_decision("cell_center_reached")
                return self._wheels(0.0, 0.0)
            remaining = ((self.target_crossing - self.est.crossings) * PITCH
                         + PITCH / 2 - self.est.s)
            speed = clamp(0.5 * remaining, self.cfg.crawl_mps,
                          self.cfg.cruise_mps)
            if front is not None and front < 0.16:
                speed = min(speed, 0.020)
            return self._track(frame, speed)
        if self.state == "DOCK":
            front = frame.front_wall_range
            if front is None:
                self._recover(frame, "dock_lost_front_wall")
                return self._wheels(0.0, 0.0)
            if front < self.cfg.dock_range - 0.010:
                self._recover(frame, "dock_too_close")
                return self._wheels(0.0, 0.0)
            if front <= self.cfg.dock_range:
                self._begin_decision("dock_range_reached")
                return self._wheels(0.0, 0.0)
            if self.dock_progress is None:
                self.dock_progress = (self.est.travel, front)
            start_travel, start_range = self.dock_progress
            if start_range - front >= 0.008:
                self.dock_progress = (self.est.travel, front)
            elif self.est.travel - start_travel >= 0.04:
                self._recover(frame, "dock_no_range_progress")
                return self._wheels(0.0, 0.0)
            return self._track(frame, clamp(0.4 * (front - self.cfg.dock_range),
                                            self.cfg.crawl_mps, 0.025))
        if self.state == "DECIDE":
            return self._decision(frame)
        if self.state == "PIVOT":
            return self._pivot(frame)
        return self._recovery(frame)

    def snapshot(self) -> dict:
        return {"t": round(self.sim_time, 4), "state": self.state,
                "cell": self.est.cell, "heading": self.est.heading,
                "s": self.est.s, "s_valid": self.est.s is not None,
                "s_uncertainty_m": self.est.position_uncertainty_m,
                "last_position_anchor_travel_m": self.est.last_position_anchor_travel,
                "front_motion_evidence_m": self.est.front_motion_evidence_m,
                "psi": self.est.psi, "psi_valid": self.est.psi_valid,
                "y": self.est.y, "y_valid": self.est.y is not None,
                "wheels": self.last_wheels, "edges": self.est.edge_count,
                "last_edge_spacing": (self.est.edge_spacings[-1]
                                      if self.est.edge_spacings else None),
                "duplicate_edges": self.est.duplicate_edges,
                "front_anchors": self.est.front_count,
                "rejected_anchors": self.est.rejected_anchors}
