import math
import json
from pathlib import Path
import tempfile
import unittest

from controller import LatticeController
from lattice import Frame, K_LINEAR, LatticeEstimator, PITCH, RangeHold
from task_1b_boilerplate import RunLog
from controller import Settings


def frame(fl=0.300, fr=0.300, sl=0.051, sr=0.051,
          yaw=0.0, dt=0.02):
    return Frame.from_payload({"fl": fl, "fr": fr, "sl": sl, "sr": sr,
                               "gyro": [0, 0, yaw], "dt": dt})


class GeometryTests(unittest.TestCase):
    def test_splayed_projection_and_saturation(self):
        measured = frame(fl=0.10, fr=0.10)
        self.assertAlmostEqual(measured.front_wall_range, 0.09397, places=4)
        self.assertIsNone(frame().front_wall_range)
        self.assertTrue(frame().front_open)
        self.assertTrue(frame(fl=0.04).front_hazard)

    def test_side_edge_reanchors_and_spacing_matches_pitch(self):
        est = LatticeEstimator()
        speed = 2.0
        dt = 0.01
        for _ in range(5):
            est.observe(frame(sl=0.300, sr=0.051, dt=dt), (speed, speed))
        for _ in range(5):
            est.observe(frame(sl=0.051, sr=0.051, dt=dt), (speed, speed))
        self.assertEqual(est.edge_count, 1)
        self.assertAlmostEqual(est.s, PITCH - 0.028, delta=0.003)
        for _ in range(round(PITCH / (K_LINEAR * speed * dt)) - 3):
            est.observe(frame(sl=0.051, sr=0.051, dt=dt), (speed, speed))
        for _ in range(5):
            est.observe(frame(sl=0.300, sr=0.051, dt=dt), (speed, speed))
        self.assertEqual(est.edge_count, 2)
        self.assertAlmostEqual(est.edge_spacings[0], PITCH, delta=0.006)
        self.assertEqual(est.cell, (1, 0))

    def test_front_anchor_needs_side_wall_and_pair(self):
        est = LatticeEstimator()
        est.observe(frame(fl=0.10, fr=0.10, sl=0.300, sr=0.300), (0, 0))
        self.assertIsNone(est.s)  # entrance posts are not a front wall anchor
        est.observe(frame(fl=0.15, fr=0.15), (0, 0))
        self.assertIsNone(est.s)  # symmetric rays may both hit corridor walls
        est.observe(frame(fl=0.10, fr=0.10, sl=0.051, sr=0.051), (0, 0))
        self.assertAlmostEqual(est.s, PITCH - (0.10 * math.cos(math.radians(20)) + 0.069))
        self.assertIsNone(frame(fl=0.10, fr=0.18).front_wall_range)

    def test_stationary_front_range_is_not_repeated_position_evidence(self):
        est = LatticeEstimator()
        observed = frame(fl=0.10, fr=0.10)
        est.observe(observed, (0, 0))
        for _ in range(100):
            est.observe(observed, (0.88, 0.88))
        self.assertEqual(est.front_count, 1)
        self.assertEqual(est.front_motion_evidence_m, 0.0)
        self.assertGreater(est.position_uncertainty_m, 0.01)
        self.assertGreater(est.travel - est.last_position_anchor_travel, 0.02)

    def test_approaching_front_wall_is_observed_progress(self):
        est = LatticeEstimator()
        for distance in (0.11, 0.106, 0.102, 0.098):
            est.observe(frame(fl=distance, fr=distance), (1.0, 1.0))
        self.assertGreater(est.front_motion_evidence_m, 0.008)

    def test_two_side_sensors_on_one_edge_count_once(self):
        est = LatticeEstimator()
        for _ in range(5):
            est.observe(frame(sl=0.300, sr=0.300), (2.0, 2.0))
        for _ in range(5):
            est.observe(frame(sl=0.051, sr=0.300), (2.0, 2.0))
        for _ in range(5):
            est.observe(frame(sl=0.051, sr=0.051), (2.0, 2.0))
        self.assertEqual(est.edge_count, 1)
        self.assertEqual(est.duplicate_edges, 1)

    def test_front_classification_requires_a_consistent_wall_pair(self):
        self.assertEqual(frame(fl=0.09, fr=0.30).front_label, "unknown")
        self.assertEqual(frame(fl=0.10, fr=0.10).front_label, "wall")
        # Recorded at the first wrong U-turn in the sustained validation run.
        self.assertEqual(frame(fl=0.106305, fr=0.754407,
                               sl=0.081682, sr=0.084432).front_label, "open")
        self.assertEqual(frame(fl=0.30, fr=0.25).front_label, "open")
        self.assertEqual(frame(fl=0.13, fr=0.13).front_label, "open")
        self.assertEqual(frame(fl=None, fr=None).front_label, "unknown")

    def test_range_hold_is_per_ray_and_expires(self):
        hold = RangeHold(0.12)
        first = Frame.from_payload({"fl": 0.08, "fr": 0.30, "sl": 0.05,
                                    "sr": 0.05, "gyro": [0, 0, 0], "dt": 0.02}, hold)
        dropout = Frame.from_payload({"fl": None, "fr": 0.30, "sl": None,
                                      "sr": 0.05, "gyro": [0, 0, 0], "dt": 0.04}, hold)
        self.assertEqual(dropout.fl.value, first.fl.value)
        self.assertEqual(dropout.sl.value, first.sl.value)
        expired = None
        for _ in range(3):
            expired = Frame.from_payload({"fr": 0.30, "sr": 0.05,
                                          "gyro": [0, 0, 0], "dt": 0.05}, hold)
        self.assertEqual(expired.fl.kind, "unknown")


class ControllerTests(unittest.TestCase):
    def controller_at_center(self):
        ctl = LatticeController()
        ctl.est.s = PITCH / 2
        ctl.est.last_position_anchor_travel = ctl.est.travel
        ctl.est.position_uncertainty_m = 0.01
        ctl.est.psi_valid = True
        ctl.state = "DECIDE"
        return ctl

    def test_straight_corridor_goes_forward(self):
        ctl = self.controller_at_center()
        for _ in range(9):
            ctl.step(frame())
        self.assertEqual(ctl.state, "CRUISE")
        self.assertGreater(ctl.last_wheels[0], 0)
        self.assertGreater(ctl.last_wheels[1], 0)
        self.assertFalse(any(e.get("to") == "PIVOT" for e in ctl.events))

    def test_left_opening_pivots_on_gyro_and_updates_heading(self):
        ctl = self.controller_at_center()
        for _ in range(9):
            ctl.step(frame(sl=0.300, sr=0.051))
        self.assertEqual(ctl.state, "PIVOT")
        for _ in range(800):
            yaw = ctl.cfg.yaw_gain * (ctl.last_wheels[1] - ctl.last_wheels[0])
            ctl.step(frame(sl=0.300, sr=0.051, yaw=yaw))
            if ctl.state == "CRUISE":
                break
        self.assertEqual(ctl.state, "CRUISE")
        self.assertEqual(ctl.est.heading, 1)
        self.assertLess(abs(ctl.est.psi), math.radians(5))

    def test_asymmetric_recorded_front_does_not_choose_uturn(self):
        ctl = self.controller_at_center()
        for _ in range(9):
            ctl.step(frame(fl=0.106305, fr=0.754407,
                           sl=0.081682, sr=0.084432))
        self.assertEqual(ctl.state, "CRUISE")
        self.assertFalse(any(e.get("to") == "PIVOT" for e in ctl.events))

    def test_pivot_waits_for_actual_cell_center_and_anchor(self):
        ctl = self.controller_at_center()
        ctl.est.s = PITCH / 2 - 0.018
        for _ in range(9):
            ctl.step(frame(sl=0.300, sr=0.051))
        self.assertEqual(ctl.state, "DECIDE")
        self.assertGreater(ctl.last_wheels[0], 0)
        self.assertGreater(ctl.last_wheels[1], 0)
        for _ in range(50):
            ctl.step(frame(sl=0.300, sr=0.051))
            if ctl.state == "PIVOT":
                break
        self.assertEqual(ctl.state, "PIVOT")
        self.assertLessEqual(abs(ctl.est.s - PITCH / 2),
                             ctl.cfg.pivot_center_tolerance)

    def test_gate_probe_and_unknown_input_stop(self):
        ctl = LatticeController()
        left, right = 0.0, 0.0
        for _ in range(100):
            left, right = ctl.step(frame(fl=0.10, fr=0.10,
                                         sl=0.300, sr=0.300))
        self.assertEqual(ctl.state, "GATE")
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)
        self.assertGreater(ctl.est.travel, 0.02)
        self.assertEqual(ctl.step(Frame.from_payload({"dt": 0.02})), (0.0, 0.0))

    def test_gate_survives_asymmetric_post_ray(self):
        ctl = LatticeController()
        for _ in range(10):
            left, right = ctl.step(frame(fl=0.093, fr=1.20,
                                         sl=0.22, sr=0.22))
        self.assertEqual(ctl.state, "GATE")
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)

    def test_gate_reports_unverified_commanded_travel(self):
        ctl = LatticeController()
        for _ in range(300):
            ctl.step(frame(fl=0.300, fr=0.1887, sl=0.051, sr=0.051))
            if any(e.get("event") == "translation_unverified" for e in ctl.events):
                break
        self.assertTrue(any(e.get("event") == "translation_unverified"
                            for e in ctl.events))

    def test_distant_single_side_is_not_a_centering_reference(self):
        ctl = LatticeController()
        ctl.est.observe(frame(sl=0.16, sr=0.300), (0, 0))
        self.assertIsNone(ctl.est.y)

    def test_recovery_reverses_by_measured_distance(self):
        ctl = LatticeController()
        ctl._recover(frame(fl=0.10, fr=0.10), "test")
        left, right = ctl.step(frame(fl=0.10, fr=0.10))
        self.assertLess(left, 0)
        self.assertLess(right, 0)
        ctl.step(frame(fl=0.19, fr=0.19))
        self.assertEqual(ctl.state, "PIVOT")
        first_direction = ctl.last_recovery_direction
        ctl._recover(frame(fl=0.10, fr=0.10), "first_attempt_failed")
        self.assertEqual(ctl.recovery_direction, -first_direction)

    def test_reverse_heading_servo_is_stable_and_converges(self):
        for initial in (0.0, math.radians(20)):
            ctl = LatticeController()
            ctl.est.psi = initial
            ctl.est.psi_valid = True
            distance = 0.0
            dt = 0.01
            for _ in range(1000):
                yaw = ctl.cfg.yaw_gain * (ctl.last_wheels[1] - ctl.last_wheels[0])
                ctl._track(frame(yaw=yaw, dt=dt), -0.025)
                yaw = ctl.cfg.yaw_gain * (ctl.last_wheels[1] - ctl.last_wheels[0])
                ctl.est.psi += yaw * dt
                distance += K_LINEAR * abs(sum(ctl.last_wheels) / 2) * dt
                if distance >= 0.08:
                    break
            self.assertLess(abs(ctl.est.psi), math.radians(5))
        self.assertLess(abs(ctl.est.psi), math.radians(5))

    def test_decide_unknown_waits_without_recovery(self):
        ctl = self.controller_at_center()
        for _ in range(50):
            ctl.step(frame(fl=None, fr=None, sl=None, sr=None))
        self.assertEqual(ctl.state, "DECIDE")
        self.assertNotIn("decision_unknown", [e.get("reason") for e in ctl.events])
        self.assertLessEqual(ctl.decision_probe_distance, 0.02)

    def test_recovery_cycles_enter_hold_at_limit(self):
        ctl = LatticeController()
        for _ in range(4):
            ctl._recover(frame(), "test")
        self.assertEqual(ctl.recovery_phase, "HOLD")
        self.assertEqual(ctl.step(frame()), (0.0, 0.0))

    def test_recovery_gate_uses_side_edge_to_reanchor(self):
        ctl = LatticeController()
        ctl.state = "GATE"
        ctl.recovery_reacquire = True
        ctl.recovery_cycles = 1
        ctl.est.psi_valid = True
        ctl.gate_start_travel = ctl.est.travel
        for _ in range(30):
            ctl.step(frame(fl=0.300, fr=0.300, sl=0.051, sr=0.051))
        self.assertEqual(ctl.state, "GATE")
        for _ in range(30):
            ctl.step(frame(fl=0.300, fr=0.300, sl=0.300, sr=0.051))
            if ctl.state == "CRUISE":
                break
        self.assertEqual(ctl.state, "CRUISE")
        self.assertFalse(ctl.recovery_reacquire)
        self.assertEqual(ctl.recovery_cycles, 0)
        self.assertEqual(ctl.events[-1]["reason"], "recovery_landmark_found")

    def test_recovery_gate_holds_at_one_pitch_without_landmark(self):
        ctl = LatticeController()
        ctl.state = "GATE"
        ctl.recovery_reacquire = True
        ctl.recovery_cycles = 1
        ctl.est.psi_valid = True
        ctl.gate_start_travel = ctl.est.travel
        for _ in range(1000):
            ctl.step(frame(fl=0.300, fr=0.300, sl=0.051, sr=0.051))
            if ctl.state == "RECOVER":
                break
        self.assertEqual(ctl.state, "RECOVER")
        self.assertEqual(ctl.recovery_phase, "HOLD")
        self.assertLessEqual(ctl.est.travel, 0.221)
        self.assertEqual(ctl.events[-1]["reason"],
                         "recovery_reanchor_distance_exhausted")

    def test_recovery_gate_holds_on_unknown_front(self):
        ctl = LatticeController()
        ctl.state = "GATE"
        ctl.recovery_reacquire = True
        ctl.est.psi_valid = True
        ctl.step(frame(fl=None, fr=None, sl=0.051, sr=0.051))
        self.assertEqual(ctl.state, "RECOVER")
        self.assertEqual(ctl.recovery_phase, "HOLD")
        self.assertEqual(ctl.events[-1]["reason"],
                         "recovery_reanchor_front_uncertain")

    def test_gate_startup_grace_avoids_immediate_recovery(self):
        ctl = LatticeController()
        for _ in range(10):
            ctl.step(frame(fl=0.04, fr=0.30, sl=0.22, sr=0.22))
        self.assertEqual(ctl.state, "GATE")
        self.assertFalse(any(e.get("reason") == "front_contact_guard"
                             for e in ctl.events))

    def test_dock_stall_escalates_by_commanded_distance(self):
        ctl = LatticeController()
        ctl.state = "DOCK"
        ctl.est.s = 0.07
        ctl.est.psi_valid = True
        ctl.dock_progress = (0.0, 0.09)
        for _ in range(250):
            ctl.step(frame(fl=0.09 / math.cos(math.radians(20)),
                           fr=0.09 / math.cos(math.radians(20))))
            if ctl.state == "RECOVER":
                break
        self.assertEqual(ctl.state, "RECOVER")
        self.assertEqual(ctl.events[-1]["reason"], "dock_no_range_progress")

    def test_result_is_written_to_run_folder(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RunLog(Path(directory), "case", Settings())
            ctl = LatticeController()
            ctl.step(frame())
            raw = {"fl": 0.3, "fr": 0.3, "sl": 0.05, "sr": 0.05,
                   "gyro": [0, 0, 0], "dt": 0.02}
            log.record(ctl, raw)
            log.ticks.flush()
            tick = json.loads((log.directory / "ticks.jsonl").read_text())
            self.assertEqual(tick["raw"], raw)
            payload = {"solved": False, "collisions": 2, "score": 0.0}
            log.result(payload)
            log.close()
            self.assertEqual(json.loads((log.directory / "result.json").read_text()),
                             payload)

    def test_missing_simulator_result_is_marked_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RunLog(Path(directory), "no_verdict", Settings())
            log.close()
            result = json.loads((log.directory / "result.json").read_text())
            self.assertEqual(result["status"], "not_received")


if __name__ == "__main__":
    unittest.main()
