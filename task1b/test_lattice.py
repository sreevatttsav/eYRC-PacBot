import math
import json
from pathlib import Path
import tempfile
import unittest

from controller import LatticeController
from lattice import Frame, K_LINEAR, LatticeEstimator, PITCH
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
        est.observe(frame(fl=0.10, fr=0.10, sl=0.051, sr=0.051), (0, 0))
        self.assertAlmostEqual(est.s, PITCH - (0.10 * math.cos(math.radians(20)) + 0.069))
        self.assertIsNone(frame(fl=0.10, fr=0.18).front_wall_range)

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


class ControllerTests(unittest.TestCase):
    def controller_at_center(self):
        ctl = LatticeController()
        ctl.est.s = PITCH / 2
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

    def test_gate_probe_and_unknown_input_stop(self):
        ctl = LatticeController()
        left, right = ctl.step(frame(fl=0.10, fr=0.10, sl=0.300, sr=0.300))
        self.assertEqual(ctl.state, "GATE")
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)
        self.assertEqual(ctl.step(Frame.from_payload({"dt": 0.02})), (0.0, 0.0))

    def test_gate_survives_asymmetric_post_ray(self):
        ctl = LatticeController()
        for _ in range(10):
            left, right = ctl.step(frame(fl=0.093, fr=1.20,
                                         sl=0.22, sr=0.22))
        self.assertEqual(ctl.state, "GATE")
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)

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
            payload = {"solved": False, "collisions": 2, "score": 0.0}
            log.result(payload)
            log.close()
            self.assertEqual(json.loads((log.directory / "result.json").read_text()),
                             payload)


if __name__ == "__main__":
    unittest.main()
