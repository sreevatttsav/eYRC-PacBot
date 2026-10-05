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

from task_1b_boilerplate import CenteringController


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


if __name__ == "__main__":
    unittest.main()
