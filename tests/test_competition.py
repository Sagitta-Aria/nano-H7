"""Competition recovery and restart tests through the queued protocol."""
import unittest
from pathlib import Path
from host.webapp import Console
from tests import test_mission as fixture


class CompetitionTests(unittest.TestCase):
    setUp = fixture.MissionTests.setUp
    send = fixture.MissionTests.send
    flush = fixture.MissionTests.flush
    step = fixture.MissionTests.step
    until = fixture.MissionTests.until

    def test_confirmed_tuning_survives_host_restart_and_board_reconnect(self):
        self.console.submit('ARM 0'); self.flush()
        self.console.submit('TURN CONFIG 3456 35 321'); self.flush()
        self.console.submit('CHASSIS CONFIG 9'); self.flush()
        restarted = Console(navigation_path=Path(self.temp.name)/'navigation.json')
        self.assertIn('TURN CONFIG 3456 35 321', restarted.control_settings.commands())
        self.assertIn('CHASSIS CONFIG 9', restarted.control_settings.commands())
        self.board.turn.kp = 20
        restarted.connection = self.link
        restarted.device['_port'] = 'SIM'
        self.console = restarted
        self.console._reply('HELLO', self.send('HELLO')); self.flush()
        self.assertEqual(self.board.turn.kp, 34.56)

    def test_one_start_establishes_origin_arms_and_runs_saved_route(self):
        self.console.submit('STOP'); self.flush()
        config = self.console.mission.snapshot()
        config['route'] = [dict(station=1, hold=0)]
        self.console.save_navigation(config)
        ok, message = self.console.competition.start()
        self.assertTrue(ok, message)
        self.until(lambda: self.console.navigator.state == 'done')
        self.assertIn('NAV RESET 0 0', self.link.sent)
        self.assertEqual(self.console.navigation.get('fault'), 'none')
        self.assertNotIn('AUX ZERO 6', self.link.sent)
        self.assertNotIn('LIFT DIR -1', self.link.sent)

    def test_stop_cancels_pending_start(self):
        self.console.submit('STOP'); self.flush()
        self.assertTrue(self.console.competition.start()[0])
        self.console.submit('STOP'); self.flush()
        for _ in range(10): self.step()
        self.assertFalse(self.console.competition.active)
        self.assertFalse(self.board.armed)

    def test_race_passes_debug_holds_without_changing_saved_route(self):
        self.console.submit('STOP'); self.flush()
        saved = self.console.mission.snapshot()
        self.assertTrue(all(leg['hold'] for leg in saved['route']))
        self.assertTrue(self.console.competition.start()[0])
        self.assertFalse(self.console.competition.start()[0])
        self.until(lambda: self.console.competition.state == 'done')
        self.assertEqual(self.console.mission.snapshot(), saved)

    def test_repeat_hello_does_not_reapply_settings_during_cruise(self):
        self.console.submit('TURN CONFIG 3200 20 250'); self.flush()
        before = len(self.link.sent)
        self.console._reply('HELLO', self.send('HELLO')); self.flush()
        self.assertNotIn('TURN CONFIG 3200 20 250', self.link.sent[before:])

    def test_rejected_tuning_does_not_replace_last_saved_value(self):
        self.console.submit('TURN CONFIG 3200 20 250'); self.flush()
        self.console.submit('TURN CONFIG 3200 20 0'); self.flush()
        self.assertEqual(self.console.control_settings.snapshot()['turn'], [3200, 20, 250])

    def test_lift_direction_restores_after_reboot_without_referencing_again(self):
        self.console.submit('ARM 0'); self.flush()
        self.console.submit('LIFT INIT'); self.flush()
        self.console.submit('LIFT DIR -1'); self.flush()
        self.assertEqual(self.console.control_settings.snapshot()['lift'], [-1])
        self.board.lift_down_sign = 0
        restarted = Console(navigation_path=Path(self.temp.name)/'navigation.json')
        restarted.connection = self.link; restarted.device['_port'] = 'SIM'
        self.console = restarted
        self.console._reply('HELLO', self.send('HELLO')); self.flush()
        self.assertEqual(self.send('LIFT STAT').fields['down_sign'], '-1')
        self.assertFalse(self.console.settings_pending)

    def test_live_route_recovers_yaw_and_finishes_without_second_start(self):
        self.console.submit('STOP'); self.flush()
        config = self.console.mission.snapshot()
        config['route'] = [dict(station=1, hold=0)]
        config['sps'] = 50
        self.console.save_navigation(config)
        self.assertTrue(self.console.competition.start()[0])
        self.until(lambda: self.board.navigation.state == 'moving')
        path = self.board.navigation.path_id
        self.board.navigation.feedback(self.board.navigation.absolute_heading+12, True, 1, self.clock[0])
        self.step()
        self.assertEqual(self.board.navigation.state, 'turning')
        self.until(lambda: self.console.competition.state == 'done')
        self.assertEqual(self.board.navigation.path_id, path)
        self.assertEqual(self.board.navigation.fault, 'none')
        self.assertEqual(sum(body.startswith('NAV START ') for body in self.link.sent), 1)

    def test_restart_waits_for_parameter_ack_and_atomic_save(self):
        from host.webserver import Session
        self.console.submit('ARM 0'); self.flush()
        session = Session(self.console)
        session.shutdown_server = lambda: None
        self.console.submit('TURN CONFIG 4200 30 250')
        self.assertTrue(self.console.settings_pending)
        self.assertFalse(session.request_restart()[0])
        self.flush()
        self.assertFalse(self.console.settings_pending)
        self.assertTrue(session.request_restart()[0])
        self.assertEqual(Console(navigation_path=Path(self.temp.name)/'navigation.json').control_settings.snapshot()['turn'], [4200, 30, 250])


if __name__ == '__main__':
    unittest.main()
