"""Turn speed and shortest-arc regressions through real ARM wheel commands.

Synthetic headings verify controller decisions, not physical yaw accuracy.
"""
import unittest

from tests import test_turn as turn_harness
from tests import test_navigation as nav_harness
from host.simulator import SimulatedBoard
from host.turn_sim import wrap


class TurnFloorArmTests(unittest.TestCase):
    setUp = turn_harness.ArmTurnTests.setUp
    request = turn_harness.ArmTurnTests.request
    feed = turn_harness.ArmTurnTests.feed
    group_edge = turn_harness.ArmTurnTests.group_edge

    def test_small_errors_keep_turn_speed_floor_but_respect_requested_cap(self):
        for error, limit, expected in ((21, 500, 200), (50, 500, 200),
                                       (100, 500, 200), (200, 500, 200),
                                       (21, 12, 12), (21, 0, 0)):
            with self.subTest(error=error, limit=limit):
                self.assertEqual(self.arm.call('turn_control_speed', error, limit), expected)
        # A turn floor must not make straight-line yaw correction more abrupt.
        self.assertEqual(self.arm.call('turn_heading_correction', 21, 500), 4)
        self.assertEqual(self.arm.call('turn_heading_correction', 20, 500), 0)
        self.request('TURN CONFIG 0 20')
        self.assertEqual(self.arm.call('turn_control_speed', 21, 500), 0)

    def test_floor_reaches_wheels_and_stops_inside_tolerance(self):
        for target in (-21, 21):
            with self.subTest(target=target):
                self.setUp()
                self.feed(0, 0)
                self.assertTrue(self.request(f'TURN MOVE REL {target} 500 2000').ok)
                self.feed(1, 0)
                for _ in range(100):
                    self.arm.call('motion_tick')
                self.assertEqual([self.arm.status(i)[1] for i in range(4)],
                                 [200 if target < 0 else -200]*4)
                self.feed(40, target)
                self.assertEqual(self.arm.call('motion_chassis_active'), 0)

    def test_absolute_shortest_arc_at_every_feedback_sample(self):
        cases = ((0, 9000, (0, 3000, 6000, 8900)),
                 (0, -9000, (0, -3000, -6000, -8900)),
                 (17000, -17000, (17000, 17900, -17900, -17100)),
                 (-17000, 17000, (-17000, -17900, 17900, 17100)),
                 (0, -18000, (0, 9000, 17900)))
        for zero, target, samples in cases:
            with self.subTest(zero=zero, target=target):
                self.setUp()
                self.feed(0, zero)
                self.assertTrue(self.request('TURN ZERO').ok)
                for index, current in enumerate(samples, 1):
                    # Zero reference may itself straddle the IMU +/-180 boundary.
                    self.feed(index*20, round(wrap((zero+current)/100)*100))
                    if index == 1:
                        self.assertTrue(self.request(f'TURN MOVE ABS {target} 500 2000').ok)
                    status = self.feed(index*20+1, round(wrap((zero+current)/100)*100))
                    error = round(wrap((target-current)/100)*100)
                    self.assertEqual(int(status['error_cd']), error)
                    self.assertLessEqual(abs(error), 18000)
                    self.assertEqual([1 if self.arm.status(i)[1] > 0 else -1 for i in range(4)],
                                     [-1 if error > 0 else 1]*4)


class CruiseFloorArmTests(unittest.TestCase):
    setUp = nav_harness.ArmNavigationTests.setUp
    request = nav_harness.ArmNavigationTests.request
    configure = nav_harness.ArmNavigationTests.configure
    upload = nav_harness.ArmNavigationTests.upload
    start = nav_harness.ArmNavigationTests.start
    nav = nav_harness.ArmNavigationTests.nav
    tick = nav_harness.ArmNavigationTests.tick

    def test_arrival_orientation_uses_floor_in_both_directions(self):
        for target in (-21, 21):
            with self.subTest(target=target):
                self.setUp()
                self.configure([(0, 0)]*5, [0, target, 0, 0, 0], initial=0)
                self.assertTrue(self.start().ok)
                self.tick(1)
                for _ in range(100):
                    self.arm.call('motion_tick')
                self.assertEqual([self.arm.status(i)[1] for i in range(4)],
                                 [200 if target < 0 else -200]*4)


class TurnFloorDemoTests(unittest.TestCase):
    def test_standalone_and_cruise_share_floor_and_speed_cap(self):
        for limit in (12, 500):
            for owner in ('turn', 'navigation'):
                with self.subTest(limit=limit, owner=owner):
                    board = SimulatedBoard(clock=lambda: 1.)
                    board.armed = True
                    board.navigation.auto_imu = False
                    board.turn.feedback(0., True, 1, 1.)
                    if owner == 'turn':
                        board.turn.dispatch(1, ['MOVE', 'REL', '21', str(limit), '2000'])
                        board.turn.update(1., .01)
                    else:
                        nav = board.navigation
                        nav.initial_heading = nav.absolute_heading = 0.
                        nav.target_heading, nav.sps = .21, limit
                        nav.state, nav.turn_started = 'orienting', 1.
                        nav.turn(1., .01)
                    self.assertEqual([abs(a['target']) for a in board.axes[:4]], [min(limit, 200)]*4)


class AdjustableFloorTests(unittest.TestCase):
    setUp = TurnFloorArmTests.setUp
    request = TurnFloorArmTests.request
    feed = TurnFloorArmTests.feed

    def test_runtime_floor_is_atomic_preserved_by_legacy_config_and_rotation_only(self):
        """Exercise the wire parser and shared control, without physical motors."""
        result = self.request('TURN CONFIG 2000 20 350')
        self.assertTrue(result.ok, result.body)
        self.assertEqual(result.fields['min_sps'], '350')
        self.assertEqual(self.arm.call('turn_control_speed', 21, 500), 350)
        self.assertEqual(self.arm.call('turn_control_speed', 21, 100), 100)
        self.assertEqual(self.arm.call('turn_heading_correction', 21, 500), 4)
        self.assertTrue(self.request('TURN CONFIG 3000 30').ok)
        self.assertEqual(self.request('TURN STAT').fields['min_sps'], '350')
        for value in (-1, 0, 5001):
            self.assertFalse(self.request('TURN CONFIG 8000 50 {}'.format(value)).ok)
        fields = self.request('TURN STAT').fields
        self.assertEqual((fields['kp_centi'], fields['tolerance_cd'], fields['min_sps']), ('3000','30','350'))
        self.feed(0,0)
        self.assertTrue(self.request('TURN MOVE REL 9000 500 2000').ok)
        self.assertFalse(self.request('TURN CONFIG 2000 20 200').ok)


if __name__ == '__main__':
    unittest.main()
