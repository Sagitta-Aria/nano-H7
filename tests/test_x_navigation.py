"""X departure through queued host protocol and the compiled ARM image.

Synthetic yaw and STEP registers verify sequencing/coordinates without moving
hardware; they cannot establish actual turn clearance or wheel slip.
"""
import copy
import unittest
from host.navigation import DEFAULT, validated, reference_command
from host.material_flow import build_plan
from tests import test_mission as host_fixture
from tests import test_turn_inset as arm_fixture
from tests import test_navigation as nav_fixture


def x_config():
    """Use field-size X and existing A/B/C/D, preserving the saved route shape."""
    config = copy.deepcopy(DEFAULT)
    config['stations'].append(dict(name='X', x=0, y=28800, heading=-180, arrival_enabled=True))
    config.update(start_station=5, turn_inset_steps=10, sps=5000, accel=20000, dwell_ms=0)
    config['grid']['y'][2] = 28800
    config['route'] = [dict(station=i, hold=0) for i in (1, 2)]
    return validated(config)


class XConfigTests(unittest.TestCase):
    def test_legacy_defaults_keep_origin_and_route(self):
        old = copy.deepcopy(DEFAULT)
        old.pop('start_station')
        result = validated(old)
        self.assertEqual(result['start_station'], 0)
        self.assertEqual(result['stations'], old['stations'])
        self.assertEqual(result['route'], old['route'])
        self.assertEqual(reference_command(result), 'NAV RESET 0 0')
        self.assertEqual(reference_command(x_config()), 'NAV RESET 0 0 0 28800 -18000')

    def test_invalid_start_pose_or_first_leg_fails_before_upload(self):
        for change in ('index', 'pose', 'heading', 'first'):
            config = x_config()
            if change == 'index': config['start_station'] = 1
            if change == 'pose': config['stations'][5]['y'] = 28799
            if change == 'heading': config['stations'][5]['heading'] = 0
            if change == 'first': config['route'][0]['station'] = 2
            with self.subTest(change=change), self.assertRaises(ValueError):
                validated(config)

    def test_material_route_keeps_x_start_and_returns_to_o(self):
        from host.mission import DEFAULT as TASK_DEFAULT
        route, _ = build_plan(x_config(), copy.deepcopy(TASK_DEFAULT))
        self.assertEqual(route['start_station'], 5)
        self.assertEqual([route['stations'][leg['station']]['name'] for leg in route['route']],
                         list('ABCDBCDO'))


class XHostTests(unittest.TestCase):
    setUp = host_fixture.MissionTests.setUp
    send = host_fixture.MissionTests.send
    flush = host_fixture.MissionTests.flush
    step = host_fixture.MissionTests.step

    def prepare(self):
        """Reference the disarmed X pose through the same action as Web/Tk."""
        self.console.submit('ARM 0'); self.flush()
        self.console._reply('HELLO', self.send('HELLO'))
        self.console._reply('INFO', self.send('INFO'))
        self.config = x_config()
        self.console.save_navigation(self.config)
        self.assertTrue(self.console.navigation_action('origin')[0]); self.flush()
        self.assertEqual(self.board.navigation.points[0], (0, 0))
        self.assertEqual((self.board.navigation.x4, self.board.navigation.y4), (0, 28800*4))
        self.console.submit('ARM 1'); self.flush()
        ok, message = self.console.navigation_action('start')
        self.assertTrue(ok, message); self.flush()

    def operations(self):
        """Extract finite vectors/turns from the real queued wire commands."""
        return [body.split() for body in self.link.sent
                if body.startswith('NAV EXEC ') and len(body.split()) == 10]

    def test_inward_turn_direct_to_a_then_original_route(self):
        self.prepare()
        self.assertEqual(self.send('NAV EXEC').fields['heading_cd'], '-18000')
        for _ in range(5000):
            self.step()
            if self.console.navigator.state == 'fault': self.fail(self.console.navigator.error)
            if self.console.navigator.state == 'done': break
        else: self.fail('X route failed to finish')
        ops = self.operations()
        first = next(i for i, op in enumerate(ops) if op[4] == '2')
        self.assertEqual([op[4] for op in ops[:first]], ['1', '0', '1'])
        self.assertEqual(ops[0][7], '-18000')
        self.assertNotEqual(ops[0][5:7], ['0', '28800'])
        self.assertEqual(ops[1][5:8], ['0', '0', '0'])
        self.assertEqual(ops[2][5:8], ['0', '14000', '0'])
        self.assertFalse(any(op[4:7] == ['1', '0', '28800'] for op in ops))
        self.assertEqual((self.console.navigation['x'], self.console.navigation['y']), ('14000', '0'))
        # The following leg uses the established center/lane planner again.
        following = ops[first+1:]
        self.assertTrue(any(op[4:8] == ['1', '14000', '14000', '9000'] for op in following))

    def test_stop_during_inward_or_turn_cannot_dispatch_a(self):
        for phase in ('inward', 'turn'):
            with self.subTest(phase=phase):
                if phase == 'turn': self.setUp()
                self.prepare()
                for _ in range(1000):
                    if self.console.navigator.phase == phase: break
                    self.step()
                else: self.fail('phase not reached: ' + phase)
                self.console.submit('STOP'); self.flush()
                count = len(self.operations())
                for _ in range(30): self.step()
                self.assertEqual(len(self.operations()), count)
                self.assertFalse(self.board.armed)

    def test_old_firmware_gate_never_resets_or_starts(self):
        self.console.submit('ARM 0'); self.flush()
        self.console.save_navigation(x_config())
        self.console.device.pop('nav_reference', None)
        count = len(self.link.sent)
        for action in ('origin', 'start'):
            ok, message = self.console.navigation_action(action)
            self.assertFalse(ok)
            self.assertIn('nav_reference', message)
        self.assertEqual(len(self.link.sent), count)

    def test_one_button_preparation_references_x(self):
        self.console.submit('STOP'); self.flush()
        self.console._reply('HELLO', self.send('HELLO'))
        self.console._reply('INFO', self.send('INFO'))
        self.console.save_navigation(x_config())
        self.assertTrue(self.console.competition.start()[0])
        for _ in range(1000):
            self.step()
            if self.console.navigator.active: break
        else: self.fail(self.console.competition.snapshot())
        self.assertIn('NAV RESET 0 0 0 28800 -18000', self.link.sent)
        self.assertEqual(self.board.navigation.points[0], (0, 0))


class XArmTests(unittest.TestCase):
    setUp = arm_fixture.GenericArmTests.setUp
    request = arm_fixture.GenericArmTests.request
    send = nav_fixture.ArmNavigationTests.request
    configure = arm_fixture.GenericArmTests.configure
    upload = arm_fixture.GenericArmTests.upload
    nav = arm_fixture.GenericArmTests.nav
    tick = arm_fixture.GenericArmTests.tick
    group_edge = arm_fixture.GenericArmTests.group_edge
    execute = arm_fixture.GenericArmTests.execute
    edges = arm_fixture.GenericArmTests.edges
    settle = arm_fixture.GenericArmTests.settle

    def prepare(self):
        """Short A near full-size X limits emulated STEP work; yaw stays real."""
        self.route = [1]
        self.configure(points=[(0,0),(0,28790),(14,0),(14,28800),(28,28790)], initial=3500,
                       grid=dict(x=[0,14,28], y=[0,28790,28800]))
        self.request('ARM 0')
        self.assertTrue(self.request('NAV RESET 0 0 0 28800 -18000').ok)
        self.request('ARM 1')
        self.assertTrue(self.upload().ok)
        self.assertTrue(self.request('NAV START 42 HOST').ok)
        self.op = 0

    def test_x_map_yaw_inward_and_direct_diagonal_at_zero(self):
        self.prepare()
        self.assertEqual(self.send('NAV EXEC').fields['heading_cd'], '18000')
        self.execute(1, 4, 28796, -18000); self.edges(0)
        self.assertEqual((self.nav()['x'], self.nav()['y']), ('4','28796'))
        self.execute(0, heading=0); self.settle(18000)
        self.execute(1, 0, 28790, 0); self.edges(18000)
        self.assertEqual((self.nav()['x'],self.nav()['y'],self.nav()['heading_cd']), ('0','28790','0'))
        # Re-uploading O's unchanged station row must not invalidate X's frame.
        self.request('NAV STOP')
        self.assertTrue(self.request('NAV POINT 0 0 0').ok)
        self.assertEqual(self.nav()['ready'], '1')
        self.assertTrue(self.upload().ok)
        self.assertTrue(self.request('NAV START 42 HOST').ok)
        self.assertEqual(self.nav()['heading_cd'], '0')

    def test_invalid_reference_is_atomic_and_legacy_reset_restores_zero(self):
        self.prepare()
        before = self.nav()
        self.assertFalse(self.request('NAV RESET 0 0 0 28800 -18000').ok)
        self.request('ARM 0')
        for body in ('NAV RESET 0 0 0 28800 -18001', 'NAV RESET 1000001 0 0 28800 -18000',
                     'NAV RESET 0 0 0 1000001 -18000'):
            self.assertFalse(self.request(body).ok)
            after = self.nav()
            self.assertEqual((after['x'],after['y']), (before['x'],before['y']))
        self.assertTrue(self.request('NAV RESET 0 0').ok)
        self.request('ARM 1'); self.assertTrue(self.upload().ok)
        self.assertTrue(self.request('NAV START 42 HOST').ok)
        self.assertEqual(self.nav()['heading_cd'], '0')

    def test_reference_capability_and_long_sequence_fit_protocol(self):
        # INFO reads the STM32 factory UID/flash-size page, not peripherals.
        self.arm.uc.mem_map(0x1FF1E000, 0x1000)
        reply = self.request('INFO', sequence=4294967295)
        self.assertEqual(reply.fields['nav_reference'], '1')
        self.assertLessEqual(len(self.arm.transmitted), 384)


if __name__ == '__main__': unittest.main()
