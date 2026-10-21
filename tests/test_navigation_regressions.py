"""Cruise restart, bounded vector return and station ownership regressions.

Execute the compiled ARM image and queued host protocol without real hardware.
"""
import copy
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from host.navigation import DEFAULT
from host.navigation_runner import NavigationRunner
from tests import test_turn_inset as executor
from tests import test_ring_task as ring


class CruiseArmRegressions(unittest.TestCase):
    setUp = executor.GenericArmTests.setUp
    request = executor.GenericArmTests.request
    configure = executor.GenericArmTests.configure
    upload = executor.GenericArmTests.upload
    nav = executor.GenericArmTests.nav
    tick = executor.GenericArmTests.tick
    group_edge = executor.GenericArmTests.group_edge
    prepare = executor.GenericArmTests.prepare
    execute = executor.GenericArmTests.execute
    settle = executor.GenericArmTests.settle
    edges = executor.GenericArmTests.edges

    def test_repeated_drift_keeps_operation_and_destination(self):
        self.prepare(); self.execute(1, 0, 1000)
        for angle in (600, -1200, 17900):
            self.tick(self.now+1, angle)
            self.assertEqual(self.nav()['fault'], 'none')
            self.assertEqual(self.nav()['state'], 'turning')
            self.group_edge()
            self.settle(0)
            self.assertEqual(self.nav()['state'], 'moving')
            self.assertEqual(self.request('NAV EXEC').fields['op_id'], '1')
        self.edges(0)
        self.assertEqual((self.nav()['x'], self.nav()['y']), ('0', '1000'))
        self.assertEqual(self.nav()['state'], 'checkpoint')

    def test_stop_during_recovery_cannot_resume_on_new_heading(self):
        self.prepare(); self.execute(1, 0, 1000)
        self.tick(self.now+1, 1200); self.request('STOP'); self.group_edge()
        before = [self.arm.status(i)[0] for i in range(4)]
        self.settle(0)
        self.assertEqual(self.nav()['state'], 'stopped')
        self.assertFalse(self.arm.call('motion_chassis_active'))
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], before)

    def test_deviation_before_translation_is_corrected_before_departure(self):
        self.prepare(); self.tick(self.now+1, -1200)
        self.execute(1, 0, 100)
        self.assertEqual(self.nav()['fault'], 'none')
        self.settle(0); self.edges(0)
        self.assertEqual((self.nav()['state'], self.nav()['y']), ('checkpoint', '100'))

    def test_restart_mid_turn_uses_measured_heading_for_inward_move(self):
        """Feed actual START feedback through the host planner back into ARM."""
        self.prepare()
        self.execute(0, heading=9000)
        self.tick(self.now + 1, 4500)
        self.request('NAV STOP')
        self.group_edge()
        self.tick(self.now + 1, 4500)
        self.assertTrue(self.upload().ok)
        self.assertTrue(self.request('NAV START 42 HOST').ok)
        initial = self.request('NAV EXEC')
        self.assertEqual(initial.fields['heading_cd'], '4500')
        self.assertEqual(initial.fields['target_cd'], '9000')
        config = copy.deepcopy(DEFAULT)
        config['stations'] = [dict(name=name, x=p[0], y=p[1], heading=0)
                              for name, p in zip('OABCD', self.points)]
        config.update(grid=self.grid, route=[dict(station=1, hold=0)],
                      turn_inset_enabled=True, turn_inset_steps=4)
        sent = []
        owner = SimpleNamespace(connection=SimpleNamespace(
            submit=lambda body, tag=None: sent.append((body, tag)) or True),
            note=lambda *args: None)
        runner = NavigationRunner(owner)
        runner.start(config, 42)
        runner.reply(sent[-1][0], initial, sent[-1][1])
        runner.tick()
        command = sent[-1][0]
        self.assertEqual(command.split()[7], '4500')
        result = self.request(command)
        self.assertTrue(result.ok, result.body)
        self.assertEqual(result.fields['fault'], 'none')
        self.edges(4500)
        self.assertEqual(self.nav()['state'], 'checkpoint')

    def test_arbitrary_heading_returns_converge_to_original_destination(self):
        """Short and long returns must finish within half a pulse of the target."""
        for heading, inside in ((6000, (1000, 0)), (6000, (20, 3)),
                                (4500, (500, 500)), (12345, (7, 2))):
            with self.subTest(heading=heading, inside=inside):
                self.setUp()
                self.prepare()
                self.execute(1, *inside)
                self.edges()
                self.assertEqual(self.nav()['state'], 'checkpoint')
                self.execute(0, heading=heading)
                self.settle(heading)
                self.execute(1, 0, 0, heading)
                self.edges(heading)
                result = self.request('NAV EXEC').fields
                self.assertEqual(result['fault'], 'none', result)
                self.assertEqual((result['state'], result['x'], result['y']),
                                 ('checkpoint', '0', '0'))


class ArrivalHandoffRegressions(unittest.TestCase):
    setUp = ring.RingTaskTests.setUp
    configure = ring.RingTaskTests.configure
    send = ring.RingTaskTests.send
    flush = ring.RingTaskTests.flush
    step = ring.RingTaskTests.step
    until = ring.RingTaskTests.until
    frame = ring.RingTaskTests.frame
    finish = ring.RingTaskTests.finish
    start = ring.RingTaskTests.start

    def test_final_visual_station_releases_navigation_owner(self):
        """A final external task ends the runner after verified RESTORE feedback."""
        self.configure()
        config = self.console.mission.snapshot()
        config['route'] = config['route'][:1]
        self.console.save_navigation(config)
        self.start()
        self.finish()
        self.assertEqual(self.console.navigator.state, 'done')
        self.assertFalse(self.console.navigator.active)
        self.assertTrue(self.console.submit('SERVO 1 90')[0])
        events = self.console._reply('NAV STAT', self.send('NAV STAT'))
        snapshots = [e['fields'] for e in events if e['type'] == 'navigation_runner']
        self.assertTrue(snapshots)
        self.assertFalse(snapshots[-1]['active'])

    def test_regular_status_before_arrival_completion_cannot_start_handoff(self):
        """A legal periodic NAV STAT must not overtake the executor barrier."""
        self.configure()
        self.assertTrue(self.console.navigation_action('start')[0])
        self.flush()
        # Hold the actual completion response in flight: a fast simulator may
        # deliver ACK and query in one pump iteration now.
        delayed = []
        original = self.console._reply
        def intercept(command, reply, tag=None):
            if tag and tag[0] == 'navrunner' and tag[3] == 'complete' and self.console.navigator.kind == 2:
                delayed.append((command, reply, tag)); return []
            return original(command, reply, tag)
        with patch.object(self.console, '_reply', side_effect=intercept):
            self.until(lambda: bool(delayed))
        self.console._reply('NAV STAT', self.send('NAV STAT'))
        self.assertEqual(self.console.task.state, 'travel')
        self.console._reply(*delayed.pop())
        self.until(lambda: self.console.task.state == 'ring_vision')
        self.assertEqual(self.console.navigator.state, 'external')
        self.assertTrue(self.board.armed)
        self.finish()
        self.assertEqual(self.console.task.state, 'done')
        self.assertTrue(self.board.armed)


if __name__ == '__main__':
    unittest.main()
