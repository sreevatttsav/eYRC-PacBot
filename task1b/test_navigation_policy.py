"""Deterministic sensing and small geometric closed-loop checks."""
import math
import sys
import types
import unittest

try:
    import paho.mqtt.client
except ImportError:
    paho = types.ModuleType('paho')
    mqtt = types.ModuleType('paho.mqtt')
    client = types.ModuleType('paho.mqtt.client')
    paho.mqtt = mqtt
    mqtt.client = client
    sys.modules.update({'paho': paho, 'paho.mqtt': mqtt,
                        'paho.mqtt.client': client})

from sensing import Tof, WallPerception
from task_1b_boilerplate import CenteringController, K_LIN, YAW_GAIN_K


class NavigationPolicyTests(unittest.TestCase):
    def test_measured_cap_and_longer_ranges(self):
        tof = Tof(sat_cap=None)
        self.assertEqual(tof.update(0.300, 0.02)[1], 'valid')
        self.assertEqual(tof.update(0.8, 0.02)[1], 'valid')
        self.assertEqual(tof.update(2.0, 0.02)[1], 'valid')
        self.assertEqual(tof.update(3.0, 0.02)[1], 'held')
        self.assertEqual(tof.update(None, 0.11)[1], 'none')

    def test_opening_requires_fresh_persistent_samples(self):
        p = WallPerception(dwell=0.1)
        for _ in range(5):
            o = p.update((1, 1, .05, .05), ('valid',)*4, .02)
        self.assertEqual(o.left, 'wall')
        o = p.update((1, 1, .30, .05), ('valid',)*4, .02)
        self.assertEqual(o.left, 'wall')
        for _ in range(10):
            o = p.update((1, 1, None, .05), ('valid', 'valid', 'held', 'valid'), .02)
        self.assertEqual(o.left, 'uncertain')
        for _ in range(5):
            o = p.update((1, 1, .30, .05), ('valid',)*4, .02)
        self.assertEqual(o.left, 'open')

    def test_uniform_corridor_never_routes_on_unchanged_ranges(self):
        ctl = CenteringController()
        for _ in range(900):
            l, r, *_ = ctl.update(.8, .8, .051, .051, 0, .02)
            self.assertEqual(ctl.state, 'FOLLOW')
            self.assertGreater(min(l, r), 0)
        self.assertAlmostEqual(ctl.wall_target, .051, delta=.002)

    def test_single_wall_steers_toward_learned_clearance(self):
        ctl = CenteringController()
        for _ in range(20):
            l, r, *_ = ctl.update(.8, .8, .08, .8, 0, .02)
        self.assertEqual(ctl.observation.left, 'wall')
        self.assertEqual(ctl.lat_mode, 'left_only')
        self.assertGreater(r, l)  # too far from left wall: steer left

    def test_single_ray_in_tight_corridor_routes_instead_of_driving_on(self):
        ctl = CenteringController()
        for _ in range(20):
            ctl.update(.11, .8, .051, .051, 0, .02)
        self.assertIn(ctl.state, ('REVERSE', 'TURN'))
        self.assertEqual(ctl.observation.hazard, 'single_ray')

    def test_persistent_near_single_ray_commits_a_turn(self):
        ctl = CenteringController()
        for _ in range(160):
            ctl.update(.115, .8, .051, .051, 0, .02)
            if ctl.state == 'TURN':
                break
        self.assertEqual(ctl.state, 'TURN')

    def test_emergency_does_not_interrupt_committed_turn(self):
        ctl = CenteringController()
        ctl._start_turn(1, math.pi/2, 'junction')
        ctl.turn_active = True
        ctl.state = 'TURN'
        ctl.update(.07, .8, .051, .051, 0, .02)
        self.assertEqual(ctl.state, 'TURN')
        self.assertEqual(ctl.turn_outcome, 'active')
        self.assertTrue(ctl.turn_active)
        self.assertNotEqual(ctl.spin_dir, 0)

    def test_closed_loop_corridor_and_left_branch(self):
        # Rays originate at the documented sensor locations. The corridor
        # walls are y=+/-0.085; a left opening starts at x=0.28.
        ctl = CenteringController()
        x, y, th, omega = 0.0, 0.0, 0.0, 0.0
        dt = .02
        def ray(ox, oy, angle):
            px = x + ox*math.cos(th) - oy*math.sin(th)
            py = y + ox*math.sin(th) + oy*math.cos(th)
            a = th + angle
            hits = []
            for wy, opening in ((-.085, False), (.085, True)):
                if abs(math.sin(a)) < 1e-6:
                    continue
                d = (wy-py)/math.sin(a)
                hx = px+d*math.cos(a)
                if d > 0 and not (opening and .28 <= hx <= .50):
                    hits.append(d)
            return min(hits, default=2.0)
        saw_advance = saw_turn = False
        for _ in range(900):
            fl = ray(.036, .005, math.radians(20))
            fr = ray(.036, -.005, -math.radians(20))
            sl = ray(-.005, .034, math.pi/2)
            sr = ray(-.005, -.034, -math.pi/2)
            l, r, *_ = ctl.update(fl, fr, sl, sr, omega, dt)
            saw_advance |= ctl.junction_stage == 'advance'
            if ctl.state == 'TURN':
                saw_turn = True
                break
            target_omega = YAW_GAIN_K*(r-l)
            omega += .2*(target_omega-omega)
            th += omega*dt
            x += K_LIN*(l+r)/2*math.cos(th)*dt
            y += K_LIN*(l+r)/2*math.sin(th)*dt
        self.assertTrue(saw_advance)
        self.assertTrue(saw_turn)
        self.assertGreater(x, .22)
        aborted_at = None
        for _ in range(400):
            fl = ray(.036, .005, math.radians(20))
            fr = ray(.036, -.005, -math.radians(20))
            sl = ray(-.005, .034, math.pi/2)
            sr = ray(-.005, -.034, -math.pi/2)
            l, r, *_ = ctl.update(fl, fr, sl, sr, omega, dt)
            target_omega = YAW_GAIN_K*(r-l)
            omega += .2*(target_omega-omega)
            th += omega*dt
            x += K_LIN*(l+r)/2*math.cos(th)*dt
            y += K_LIN*(l+r)/2*math.sin(th)*dt
            if ctl.turn_outcome == 'aborted' and aborted_at is None:
                aborted_at = (ctl.state, ctl.state_reason, x, y, th,
                              fl, fr, sl, sr, ctl.observation)
            if ctl.turn_outcome == 'complete':
                break
        self.assertEqual(ctl.turn_outcome, 'complete',
                         (aborted_at, ctl.state, ctl.state_reason, x, y, th))
        self.assertAlmostEqual(th, math.pi/2, delta=math.radians(12))


if __name__ == '__main__':
    unittest.main()
