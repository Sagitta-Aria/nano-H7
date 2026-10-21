"""Saved auxiliary limits survive reconnection through real protocol replies."""
from pathlib import Path
import unittest

from host.webapp import Console
from tests import test_mission as fixture


class AuxiliaryRestoreTests(unittest.TestCase):
    setUp = fixture.MissionTests.setUp
    send = fixture.MissionTests.send
    flush = fixture.MissionTests.flush

    def prepare(self):
        """Save nondefault limits and presets, then reset only board limits."""
        self.console.submit('ARM 0'); self.flush()
        self.console.submit('LIFT INIT'); self.flush()
        self.console.submit('AUX ZERO 6'); self.flush()
        path = Path(self.temp.name) / 'auxiliary.json'
        config = self.console.aux_settings.snapshot()
        config['5'].update(max=9500, sps=765, presets=[0, 9200])
        config['6'].update(min=-2000, max=100, sps=456, presets=[0, -1750])
        from host.auxiliary import AuxiliaryConfig
        AuxiliaryConfig(path).save(config)
        self.console = Console(navigation_path=Path(self.temp.name)/'navigation.json', auxiliary_path=path)
        self.console.connection = self.link
        self.console.device['_port'] = 'SIM'
        self.before = path.read_bytes()
        self.link.sent.clear()

    def reconnect(self):
        """Read the board handshake and status as a real connection does."""
        self.console._reply('HELLO', self.send('HELLO')); self.flush()
        for axis in (5, 6):
            self.console._reply('AUX STAT {}'.format(axis), self.send('AUX STAT {}'.format(axis)))
        self.flush()

    def test_saved_ranges_restore_without_moving_or_rezeroing(self):
        self.prepare(); self.reconnect()
        self.assertEqual(self.send('AUX STAT 5').fields['max'], '9500')
        self.assertEqual(self.send('AUX STAT 6').fields['min'], '-2000')
        self.assertEqual(self.send('AUX STAT 6').fields['max'], '100')
        self.assertEqual(self.console.aux_settings.path.read_bytes(), self.before)
        self.assertFalse(self.console.settings_pending)
        self.assertFalse(any(c.startswith(('ARM ', 'AUX ZERO ', 'AUX GOTO ', 'LIFT INIT', 'AUX REFERENCE ')) for c in self.link.sent))
        count = len(self.link.sent)
        self.reconnect()
        self.assertFalse(any(c.startswith('AUX CONFIG ') for c in self.link.sent[count:]))

    def test_missing_rotation_reference_waits_for_operator_then_restores(self):
        self.prepare()
        self.board.rotation_referenced = False
        self.reconnect()
        self.assertNotIn('AUX CONFIG 6 -2000 100', self.link.sent)
        self.assertNotIn('AUX ZERO 6', self.link.sent)
        self.console.submit('AUX ZERO 6'); self.flush()
        self.console._reply('AUX STAT 6', self.send('AUX STAT 6')); self.flush()
        self.assertEqual(self.send('AUX STAT 6').fields['min'], '-2000')

    def test_busy_axis_waits_and_saved_file_is_never_rewritten(self):
        self.prepare()
        self.board.axes[5].update(active=1, target=100, remaining=1000, finite=True)
        self.reconnect()
        self.assertNotIn('AUX CONFIG 6 -2000 100', self.link.sent)
        self.board.axes[5]['active'] = 0
        self.console._reply('AUX STAT 6', self.send('AUX STAT 6')); self.flush()
        self.assertEqual(self.send('AUX STAT 6').fields['min'], '-2000')
        self.assertEqual(self.console.aux_settings.path.read_bytes(), self.before)

    def test_position_outside_saved_range_reports_once_without_rezeroing(self):
        self.prepare()
        self.board.axes[5]['pos'] = 200
        self.reconnect()
        self.assertIn('不包含当前位置', self.console.settings_error)
        self.assertNotIn('AUX CONFIG 6 -2000 100', self.link.sent)
        self.assertEqual(self.send('AUX STAT 6').fields['pos'], '200')
        self.assertEqual(self.console.aux_settings.path.read_bytes(), self.before)


if __name__ == '__main__':
    unittest.main()
