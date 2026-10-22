"""Touch launcher helpers and the O/X start-station override it relies on."""

import copy
import unittest
from types import SimpleNamespace

from host.competition import Competition
from host import launcher
from host.navigation import DEFAULT as NAV_DEFAULT


def state(**overrides):
    base = {
        'link': 'connected',
        'armed': False,
        'device': {'fw': '0.2.29', 'nav_reference': '1'},
        'competition': {'state': 'idle', 'error': ''},
        'task': {'state': 'idle', 'error': ''},
        'mission': {
            'start_station': 0,
            'stations': [{'name': 'O', 'x': 0, 'y': 0}, {'name': 'X', 'x': 0, 'y': 28800}],
        },
    }
    base.update(overrides)
    return base


class LauncherStateTests(unittest.TestCase):
    def test_start_payload_only_accepts_the_two_stations(self):
        self.assertEqual(launcher.start_payload('O'), {'full_flow': True, 'start_station': 'O'})
        self.assertEqual(launcher.start_payload('x')['start_station'], 'X')
        self.assertEqual(launcher.start_payload(None)['start_station'], 'O')
        with self.assertRaises(ValueError):
            launcher.start_payload('C')

    def test_x_needs_firmware_capability_and_a_configured_station(self):
        self.assertTrue(launcher.x_ready(state())[0])
        no_capability = state(device={'fw': '0.2.28'})
        ready, why = launcher.x_ready(no_capability)
        self.assertFalse(ready)
        self.assertIn('0.2.29', why)
        no_station = state(mission={'start_station': 0,
                                    'stations': [{'name': 'O', 'x': 0, 'y': 0}]})
        ready, why = launcher.x_ready(no_station)
        self.assertFalse(ready)
        self.assertIn('X', why)

    def test_vehicle_and_phase_lines(self):
        self.assertIn('未连接', launcher.vehicle_line(state(link='offline')))
        self.assertIn('未使能', launcher.vehicle_line(state()))
        self.assertIn('已使能', launcher.vehicle_line(state(armed=True)))
        self.assertIn('上位机未响应', launcher.vehicle_line({'offline': True}))
        self.assertEqual(launcher.phase_line(state()), launcher.PHASES['idle'])
        running = state(competition={'state': 'running'},
                        task={'state': 'material_cycle'})
        self.assertIn('抓取与放置', launcher.phase_line(running))
        self.assertTrue(launcher.running(running))
        self.assertFalse(launcher.running(state()))
        self.assertEqual(launcher.error_line(state(competition={'state': 'fault', 'error': '启动失败'})),
                         '启动失败')
        self.assertEqual(launcher.phase_line({'offline': True}), '等待上位机…')

    def test_station_name_follows_the_saved_choice(self):
        self.assertEqual(launcher.station_name(state()), 'O')
        self.assertEqual(launcher.station_name(state(
            mission={'start_station': 1, 'stations': [{'name': 'O'}, {'name': 'X'}]})), 'X')


class StartStationOverrideTests(unittest.TestCase):
    def route(self):
        return copy.deepcopy(NAV_DEFAULT)

    def test_o_and_x_resolve_by_name_without_touching_the_saved_route(self):
        route = self.route()
        route['stations'] = [dict(name='O', x=0, y=0, heading=0),
                             dict(name='X', x=0, y=28800, heading=-180)]
        chosen = Competition.with_start_station(route, 'X')
        self.assertEqual(chosen['start_station'], 1)
        self.assertEqual(route['start_station'], 0, '原始配置不能被修改')
        self.assertEqual(Competition.with_start_station(route, 'o')['start_station'], 0)
        self.assertEqual(Competition.with_start_station(route, 1)['start_station'], 1)

    def test_missing_or_unlaunchable_stations_are_rejected(self):
        route = self.route()
        route['stations'] = [dict(name='O', x=0, y=0, heading=0),
                             dict(name='A', x=0, y=14000, heading=0)]
        with self.assertRaises(ValueError):
            Competition.with_start_station(route, 'X')
        with self.assertRaises(ValueError):
            Competition.with_start_station(route, 'A')
        with self.assertRaises(ValueError):
            Competition.with_start_station(route, 5)
        with self.assertRaises(ValueError):
            Competition.with_start_station(route, None)

    def test_start_uses_the_override_for_this_run_only(self):
        console = SimpleNamespace(
            lock=None, nav_pending=False, navigator=SimpleNamespace(active=False),
            task=SimpleNamespace(active=False), pick_place=SimpleNamespace(active=False),
            camera_alignment=SimpleNamespace(active=False), axes={},
            mission=SimpleNamespace(snapshot=lambda: route()), task_settings=SimpleNamespace(
                snapshot=lambda: {'legs': {}}), pick_place_settings=None,
            autolink=None, task_cameras=None, device={'nav_reference': '1'})
        import threading
        console.lock = threading.RLock()
        import host.competition as competition_module
        original = competition_module.validated
        competition_module.validated = lambda config: config
        try:
            race = Competition(console)
            route = {
                'stations': [dict(name='O', x=0, y=0, heading=0),
                             dict(name='X', x=0, y=28800, heading=-180)],
                'route': [dict(station=1, hold=0)], 'grid': {'x': [0, 14000, 28000],
                                                             'y': [0, 14000, 28800]},
                'sps': 500, 'accel': 2000, 'dwell_ms': 0,
                'turn_inset_enabled': False, 'turn_inset_steps': 1000, 'start_station': 0,
            }
            console.mission = SimpleNamespace(snapshot=lambda: copy.deepcopy(route))
            console.task_settings = SimpleNamespace(snapshot=lambda: {'legs': {}})
            ok, message = race.start(full_flow=False, start_station='X')
            self.assertTrue(ok, message)
            self.assertEqual(race.route['start_station'], 1)
            # 同名非法选择要在一个空闲实例上验证，否则先被“任务正在执行”挡住。
            idle = Competition(console)
            bad, why = idle.start(full_flow=False, start_station='A')
            self.assertFalse(bad)
            self.assertIn('没有 A 点', why)
        finally:
            competition_module.validated = original


if __name__ == '__main__':
    unittest.main()
