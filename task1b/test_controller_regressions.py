"""Focused regressions for maze navigation recovery behavior."""
import unittest
from collections import deque
import math
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

from task_1b_boilerplate import (
    CenteringController, FOLLOW_STEER_RATIO, FRONT_BACKOUT_SPEED,
    FRONT_EMERGENCY_DIST, FRONT_MIN_SPEED_MPS, FRONT_RAY_COS,
    FRONT_SPEED_TAPER_END, HEADING_PIVOT_THRESHOLD, K_LIN,
    YAW_GAIN_K, FOLLOW_ESTABLISH_S,
)
from task_1b_boilerplate import GATEWAY_APPROACH_M, GATEWAY_CORRIDOR_S


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
            ctl.update(0.8, 0.8, 0.051, 0.30, 0.0, 0.02)
        self.assertEqual(ctl.follow_side, 1.0)

        # Left opens into a branch while the right wall remains present.
        for _ in range(160):
            ctl.update(0.8, 0.8, 0.30, 0.051, 0.0, 0.02)
            if ctl.state == "TURN":
                break

        self.assertEqual(ctl.state, "TURN")
        self.assertEqual(ctl.turn_cause, "junction")
        self.assertGreater(ctl.turn_target, 0.0)

    def test_two_wall_corridor_arms_junction_detector(self):
        ctl = CenteringController()

        # A normal narrow corridor has both side ranges below FOLLOW_WALL_MAX.
        # It must still latch a wall so that its later loss can mean a branch.
        for _ in range(70):
            ctl.update(0.8, 0.8, 0.051, 0.051, 0.0, 0.02)
        self.assertEqual(ctl.follow_side, 1.0)

        for _ in range(160):
            ctl.update(0.8, 0.8, 0.30, 0.051, 0.0, 0.02)
            if ctl.state == "TURN":
                break

        self.assertEqual(ctl.state, "TURN")
        self.assertEqual(ctl.turn_cause, "junction")
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

    def test_single_close_front_ray_does_not_trigger_escape(self):
        ctl = CenteringController()
        for _ in range(20):
            left, right, *_ = ctl.update(0.10, 0.60, 0.18, 0.18,
                                         0.0, 0.02)
            self.assertEqual(ctl.state, "FOLLOW")
            self.assertEqual(ctl.spin_dir, 0.0)
            self.assertEqual(ctl.reverse_ticks, 0)
            self.assertGreater(left + right, 0.0)

    def test_single_ray_emergency_does_not_interrupt_committed_turn(self):
        ctl = CenteringController()
        ctl._start_turn(1.0, math.pi / 2.0, "progress_route")
        ctl.state = "TURN"

        left, right, *_ = ctl.update(0.08 / FRONT_RAY_COS, 0.30,
                                     0.18, 0.18, 0.0, 0.02)

        self.assertEqual(ctl.state, "TURN")
        self.assertFalse(ctl.front_backout_active)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)

    def test_gateway_wall_follow_ignores_entrance_jamb_single_ray(self):
        ctl = CenteringController()
        ctl.state = "FOLLOW"
        ctl.gateway_wall_follow_active = True
        ctl.follow_t = FOLLOW_ESTABLISH_S

        left, right, *_ = ctl.update(0.08 / FRONT_RAY_COS, 0.80,
                                     0.21, 0.19, 0.0, 0.02)

        self.assertEqual(ctl.state, "FOLLOW")
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertFalse(ctl.front_backout_active)

    def test_single_ray_in_corridor_does_not_hold_minimum_speed(self):
        ctl = CenteringController()
        ctl.gateway_wall_follow_active = True
        for _ in range(20):
            left, right, *_ = ctl.update(0.11 / FRONT_RAY_COS, 0.32,
                                         0.051, 0.051, 0.0, 0.02)

        self.assertEqual(ctl.state, "FOLLOW")
        self.assertEqual(ctl.observation.hazard, "single_ray")
        self.assertGreater((left + right) * 0.5, 1.5)

    def test_stale_opposite_front_ray_routes_around_close_wall(self):
        ctl = CenteringController()
        for _ in range(20):
            ctl.update(0.8, 0.8, 0.30, 0.30, 0.0, 0.02)
        for _ in range(30):
            left, right, *_ = ctl.update(None, 0.047, 0.30, 0.30,
                                         0.0, 0.02)
            if ctl.state == "REVERSE":
                break

        self.assertEqual(ctl.state, "REVERSE")
        self.assertLess(left, 0.0)
        self.assertLess(right, 0.0)
        self.assertEqual(ctl.turn_cause, "junction")

        for _ in range(40):
            ctl.update(None, 0.20, 0.30, 0.30, 0.0, 0.02)
            if ctl.state == "TURN":
                break
        self.assertEqual(ctl.state, "TURN")

    def test_lone_close_splayed_ray_does_not_steer(self):
        ctl = CenteringController()
        left, right, *_ = ctl.update(None, 0.10, 0.18, 0.18, 0.0, 0.02)

        self.assertEqual(ctl.state, "FOLLOW")
        self.assertEqual(ctl.steer_front, 0.0)
        self.assertAlmostEqual(left, right)
        self.assertGreater(left + right, 0.0)

    def test_front_distance_uses_forward_projection_and_pair_gate(self):
        ctl = CenteringController()
        _, _, _, e_front, _ = ctl.update(
            0.20, 0.29, 0.18, 0.18, 0.0, 0.02)

        self.assertAlmostEqual(e_front, (0.20 - 0.29) * FRONT_RAY_COS)
        self.assertEqual(ctl.steer_front, 0.0)  # asymmetric pair is not a wall

        ctl = CenteringController()
        ctl.update(0.20, 0.23, 0.18, 0.18, 0.0, 0.02)
        self.assertNotEqual(ctl.steer_front, 0.0)  # close, similar wall pair

    def test_near_front_taper_keeps_minimum_forward_speed(self):
        ctl = CenteringController()
        ray_distance = 0.09 / FRONT_RAY_COS
        left, right, *_ = ctl.update(ray_distance, 0.60, 0.18, 0.18,
                                     0.0, 0.02)

        self.assertEqual(ctl.state, "FOLLOW")
        self.assertGreaterEqual((left + right) * 0.5 * K_LIN,
                                FRONT_MIN_SPEED_MPS - 1e-6)

    def test_large_heading_error_does_not_pivot_in_corridor(self):
        ctl = CenteringController()
        ctl.gyro_th = -math.radians(25)
        left, right, *_ = ctl.update(0.8, 0.8, 0.12, 0.22, 0.0, 0.02)

        self.assertEqual(ctl.state, "FOLLOW")
        self.assertGreater(left + right, 0.0)

    def test_progress_watchdog_flags_three_matching_backouts(self):
        ctl = CenteringController()
        signature = (0.07, 0.20, 0.10, 0.12)
        for _ in range(3):
            ctl._record_backout_signature(signature)
        self.assertTrue(ctl.backout_loop_latched)

        # A repeated signature alone is not evidence of a safe route turn.
        left, right, *_ = ctl.update(0.8, 0.8, 0.18, 0.18, 0.0, 0.02)
        self.assertEqual(ctl.state, "BACKOUT_HOLD")
        self.assertEqual((left, right), (0.0, 0.0))

    def test_lone_emergency_ray_backs_until_clear(self):
        ctl = CenteringController()
        left, right, *_ = ctl.update(None, 0.0436, 0.18, 0.18, 0.0, 0.02)

        self.assertEqual(ctl.state, "FRONT_BACKOUT")
        self.assertAlmostEqual(left, -FRONT_BACKOUT_SPEED)
        self.assertAlmostEqual(right, -FRONT_BACKOUT_SPEED)

        for _ in range(20):
            left, right, *_ = ctl.update(None, 0.20, 0.18, 0.18, 0.0, 0.02)
            if ctl.state != "FRONT_BACKOUT":
                break
        self.assertNotEqual(ctl.state, "FRONT_BACKOUT")
        self.assertGreater(left + right, 0.0)

    def test_front_backout_starts_before_taper_can_stall(self):
        self.assertGreater(FRONT_EMERGENCY_DIST, FRONT_SPEED_TAPER_END)
        ctl = CenteringController()
        left, right, *_ = ctl.update(0.30, 0.07, 0.18, 0.18, 0.0, 0.02)

        self.assertEqual(ctl.state, "FRONT_BACKOUT")
        self.assertAlmostEqual(left, -FRONT_BACKOUT_SPEED)
        self.assertAlmostEqual(right, -FRONT_BACKOUT_SPEED)

    def test_slow_follow_steering_never_reverses_a_wheel(self):
        ctl = CenteringController()
        for _ in range(20):
            left, right, *_ = ctl.update(0.30, 0.08, 0.12, 0.07,
                                         0.0, 0.02)
            if ctl.state == "FOLLOW":
                self.assertGreaterEqual(min(left, right), -1e-9)
                self.assertAlmostEqual(
                    abs(right - left) / (left + right),
                    FOLLOW_STEER_RATIO, delta=0.251)

    def test_both_close_front_rays_still_trigger_escape(self):
        ctl = CenteringController()
        for _ in range(15):
            ctl.update(0.10, 0.10, 0.18, 0.18, 0.0, 0.02)
        self.assertEqual(ctl.state, "REVERSE")
        self.assertNotEqual(ctl.spin_dir, 0.0)
        self.assertAlmostEqual(abs(ctl.turn_target), 3.141592653589793)

    def test_blocked_front_turns_toward_classified_open_side(self):
        ctl = CenteringController()
        for _ in range(15):
            ctl.update(0.10, 0.10, 0.10, 0.30, 0.0, 0.02)

        self.assertEqual(ctl.state, "REVERSE")
        self.assertLess(ctl.spin_dir, 0.0)
        self.assertEqual(ctl.turn_cause, "junction")

    def test_gateway_handoff_returns_to_nearest_front_speed_control(self):
        ctl = CenteringController()
        # Reproduce the maze12 signature at the end of the entrance probe:
        # one close front ray, one open ray, and symmetric close side walls.
        ctl.gateway_active = True
        ctl.gateway_distance = GATEWAY_APPROACH_M
        ctl.gateway_corridor_t = GATEWAY_CORRIDOR_S

        left, right, *_ = ctl.update(0.10, 0.60, 0.10, 0.10, 0.0, 0.02)

        self.assertFalse(ctl.gateway_active)
        self.assertEqual(ctl.state, "FOLLOW")
        self.assertLessEqual((left + right) / 2.0,
                             0.02 / K_LIN + 1e-6)

    def test_gateway_extends_straight_approach_then_holds_if_still_blocked(self):
        ctl = CenteringController()
        ctl.gateway_active = True
        ctl.gateway_distance = GATEWAY_APPROACH_M * 0.8

        left, right, *_ = ctl.update(0.10, 0.10, 1.14, 1.14, 0.0, 0.02)
        self.assertEqual(ctl.state, "GATEWAY")
        self.assertFalse(ctl.gateway_failed)
        self.assertGreater(left + right, 0.0)

        # Reaching the longer range with both front rays still blocked is
        # not permission to fall through into the default-left escape turn.
        ctl.gateway_distance = GATEWAY_APPROACH_M
        left, right, *_ = ctl.update(0.10, 0.10, 1.14, 1.14, 0.0, 0.02)
        self.assertEqual(ctl.state, "GATEWAY_HOLD")
        self.assertTrue(ctl.gateway_failed)
        self.assertEqual((left, right), (0.0, 0.0))
        self.assertEqual(ctl.spin_dir, 0.0)

    def test_gateway_hands_off_on_acquired_side_wall_not_elapsed_time(self):
        ctl = CenteringController()
        ctl.update(0.10, 0.10, 1.14, 1.14, 0.0, 0.02)
        self.assertTrue(ctl.gateway_active)

        # Maze16 signature: front rays remain near the posts, but both side
        # ranges move inward enough to prove the corridor walls are arriving.
        for _ in range(40):
            left, right, *_ = ctl.update(0.10, 0.12, 0.50, 0.85,
                                         0.0, 0.02)
            if ctl.state == "FOLLOW":
                break

        self.assertEqual(ctl.state, "FOLLOW")
        self.assertFalse(ctl.gateway_active)
        self.assertTrue(ctl.gateway_wall_follow_active)
        self.assertEqual(ctl.follow_side, 1.0)
        self.assertGreaterEqual(min(left, right), -1e-9)
        self.assertEqual(ctl.spin_dir, 0.0)

        # The passage override is sensor-released once a front ray clears.
        for _ in range(30):
            ctl.update(0.30, 0.30, 0.50, 0.85, 0.0, 0.02)
            if not ctl.gateway_wall_follow_active:
                break
        self.assertFalse(ctl.gateway_wall_follow_active)

    def test_gateway_does_not_rearm_until_front_signature_clears(self):
        ctl = CenteringController()
        ctl.update(0.10, 0.10, 1.14, 1.14, 0.0, 0.02)
        for _ in range(40):
            ctl.update(0.10, 0.12, 0.50, 0.85, 0.0, 0.02)
            if ctl.state == "FOLLOW":
                break
        self.assertEqual(ctl.state, "FOLLOW")
        self.assertTrue(ctl.gateway_rearm_latched)

        # Persistent near/symmetric fronts and open sides are the same
        # gateway signature, not evidence of a new gateway.
        for _ in range(20):
            ctl.update(0.10, 0.12, 0.50, 0.85, 0.0, 0.02)
            self.assertEqual(ctl.state, "FOLLOW")

        # The gateway is a one-time entrance passage.
        for _ in range(30):
            ctl.update(0.30, 0.30, 0.50, 0.85, 0.0, 0.02)
        self.assertTrue(ctl.gateway_rearm_latched)
        for _ in range(40):
            ctl.update(0.10, 0.10, 0.50, 0.85, 0.0, 0.02)
            if ctl.state == "GATEWAY":
                break
        self.assertNotEqual(ctl.state, "GATEWAY")


if __name__ == "__main__":
    unittest.main()
