"""Persist intermediate coordinate edits without relaxing motion checks."""
import copy
import tempfile
import unittest
from pathlib import Path

from host.navigation import DEFAULT, MissionConfig, validated
from tests import test_mission as fixture


class EditingPersistenceTests(unittest.TestCase):
    def test_off_lane_coordinates_survive_reload_and_can_be_corrected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'navigation.json'
            mission = MissionConfig(path)
            config = copy.deepcopy(DEFAULT)
            config['stations'][1]['x'] = 123
            config['stations'][1]['y'] = 456
            mission.save(config)
            restored = MissionConfig(path)
            self.assertEqual(restored.snapshot()['stations'][1]['x'], 123)
            with self.assertRaises(ValueError):
                restored.commands()
            config['grid']['x'][0] = 123
            config['stations'][0]['x'] = 123
            restored.save(config)
            self.assertTrue(restored.commands())

    def test_storage_still_rejects_invalid_numbers_without_overwriting(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = MissionConfig(Path(directory) / 'navigation.json')
            original = mission.save(copy.deepcopy(DEFAULT))
            before = mission.path.read_bytes()
            for value in (None, 1.5, 1000001, float('nan')):
                config = copy.deepcopy(original)
                config['stations'][1]['x'] = value
                with self.subTest(value=value), self.assertRaises(ValueError):
                    mission.save(config)
                self.assertEqual(mission.path.read_bytes(), before)

    def test_x_departure_constraints_apply_at_execution(self):
        config = copy.deepcopy(DEFAULT)
        config['stations'].append(dict(name='X', x=15, y=28801, heading=-180))
        config['start_station'] = 5
        self.assertEqual(validated(config, executable=False)['stations'][5]['x'], 15)
        with self.assertRaises(ValueError):
            validated(config)


class EditingExecutionTests(unittest.TestCase):
    setUp = fixture.MissionTests.setUp
    send = fixture.MissionTests.send
    flush = fixture.MissionTests.flush

    def test_debug_and_competition_reject_incomplete_coordinates_before_upload(self):
        config = self.console.mission.snapshot()
        config['stations'][1].update(x=1, y=1)
        self.console.save_navigation(config)
        sent = list(self.link.sent)
        for action in ('origin', 'start'):
            ok, reason = self.console.navigation_action(action)
            self.assertFalse(ok)
            self.assertIn('A', reason)
        ok, reason = self.console.competition.start()
        self.assertFalse(ok)
        self.assertIn('A', reason)
        self.assertEqual(self.link.sent, sent)
        self.assertFalse(self.console.nav_pending)


if __name__ == '__main__':
    unittest.main()
