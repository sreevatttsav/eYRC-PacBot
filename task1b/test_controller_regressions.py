"""Focused regressions for maze navigation recovery behavior."""
import unittest
from collections import deque
import sys
import types

# The controller's pure-Python state logic does not need an MQTT client.
try:
    import paho.mqtt.client  # noqa: F401
except ImportError:
    paho = types.ModuleType("paho")
    mqtt = types.ModuleType("paho.mqtt")
    client = types.ModuleType("paho.mqtt.client")
    paho.mqtt = mqtt
    mqtt.client = client
    sys.modules.update({"paho": paho, "paho.mqtt": mqtt,
                        "paho.mqtt.client": client})

from task_1b_boilerplate import CenteringController, YAW_GAIN_K


class ControllerRegressionTests(unittest.TestCase):
    def test_stuck_detector_is_reset_during_turn(self):
        ctl = CenteringController()
        ctl.state = "TURN"
        ctl._stuck_prev = True
        ctl.stuck.stuck = True
        ctl.stuck.acc = 0.7
        ctl.stuck.sum_cmd = 1.0
        ctl.stuck.sum_meas = 0.0

        ctl._finalize(-3.0, 3.0, 0.0, None, 0.0, 0.0, 0.02)

        self.assertEqual(ctl.state, "TURN")
        self.assertFalse(ctl.stuck_flag)
        self.assertFalse(ctl._stuck_prev)
        self.assertEqual(ctl.stuck.acc, 0.0)

    def test_wall_loss_turns_into_branch_even_with_opposite_wall(self):
        ctl = CenteringController()

        # Establish a left-hand wall-follow reference.
        for _ in range(70):
            ctl.update(0.8, 0.8, 0.20, 0.28, 0.0, 0.02)
        self.assertEqual(ctl.follow_side, 1.0)

        # Left opens into a branch while the right wall remains present.
        for _ in range(80):
            ctl.update(0.8, 0.8, 0.30, 0.20, 0.0, 0.02)
            if ctl.state == "TURN":
                break

        self.assertEqual(ctl.state, "TURN")
        self.assertEqual(ctl.turn_cause, "gap")
        self.assertGreater(ctl.turn_target, 0.0)

    def test_second_wedge_cycle_commits_to_open_side(self):
        ctl = CenteringController()
        ctl.wedge_active = True
        ctl.wedge_t = 0.5
        ctl.wedge_cycles = deque([0.1, 0.8])

        ctl.update(0.8, 0.8, 0.20, 0.25, 0.0, 0.02)

        self.assertEqual(ctl.state, "TURN")
        self.assertEqual(ctl.turn_cause, "wedge")
        self.assertLess(ctl.turn_target, 0.0)  # more clearance on the right

    def test_close_side_walls_with_clear_front_are_not_a_wedge(self):
        ctl = CenteringController()
        for _ in range(30):
            ctl.update(0.8, 0.8, 0.05, 0.06, 0.0, 0.02)
        self.assertFalse(ctl.wedge_active)
        self.assertNotEqual(ctl.state, "WEDGE")

        # With a blocked front, sustained close side walls are a real pinch.
        for _ in range(20):
            ctl.update(0.10, 0.10, 0.05, 0.06, 0.0, 0.02)
            if ctl.wedge_active:
                break
        self.assertTrue(ctl.wedge_active)

    def test_blocked_front_after_completed_turn_does_not_repeat_turn(self):
        ctl = CenteringController()
        dt = 0.02
        omega = 0.0
        transitions = []
        last_state = ctl.state
        completed_blocked_turn = False

        for _ in range(600):
            left, right, *_ = ctl.update(0.10, 0.10, 0.18, 0.12,
                                         omega, dt)
            cmd_omega = 1.25 * YAW_GAIN_K * (right - left)
            omega += (cmd_omega - omega) * dt / (0.12 + dt)
            if ctl.state != last_state:
                transitions.append((last_state, ctl.state))
                last_state = ctl.state
            if transitions[-1:] == [("BRAKE", "FOLLOW")]:
                completed_blocked_turn = True
                break

        self.assertTrue(completed_blocked_turn)
        self.assertIn(("FOLLOW", "REVERSE"), transitions)
        self.assertNotEqual(transitions[-1], ("BRAKE", "TURN"))


if __name__ == "__main__":
    unittest.main()
