"""Center route preferences at the planner and NX action scheduling boundary."""
import copy
import json
from pathlib import Path
import shutil
import subprocess
import unittest
from types import SimpleNamespace

from host.navigation import DEFAULT, mission_waypoints
from host.navigation_grid import plan_route
from host.navigation_runner import NavigationRunner


class CenterNavigationTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node.js is required for preview parity')
    def test_browser_preview_matches_nx_on_shifted_and_unequal_grids(self):
        """Exercise the actual JS preview, including starts between crossings."""
        cases = []
        for grid in [dict(x=[0, 14, 28], y=[0, 14, 28]),
                     dict(x=[-30, -10, 50], y=[-40, 20, 60]),
                     dict(x=[0, 1, 80], y=[-100, -1, 0])]:
            points = [(x, y) for x in grid['x'] for y in grid['y']]
            points += [(grid['x'][0], (grid['y'][0]+grid['y'][1])//2),
                       ((grid['x'][1]+grid['x'][2])//2, grid['y'][2])]
            cases.extend(dict(start=start, goal=goal, grid=grid) for start in points for goal in points)
        script = """
const fs = require('node:fs'), vm = require('node:vm');
const source = fs.readFileSync('host/static/app.js', 'utf8');
const context = vm.createContext({crypto: require('node:crypto').webcrypto, structuredClone});
vm.runInContext(source.slice(0, source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\\r?$/gm, ''), context);
context.cases = JSON.parse(fs.readFileSync(0, 'utf8'));
process.stdout.write(JSON.stringify(vm.runInContext('cases.map(c => planGridRoute(c.start,c.goal,c.grid))', context)));
"""
        result = subprocess.run(['node', '-e', script], input=json.dumps(cases),
                                text=True, capture_output=True, check=True,
                                cwd=Path(__file__).resolve().parents[1])
        expected = [[list(p) for p in plan_route(**case)] for case in cases]
        self.assertEqual(json.loads(result.stdout), expected)

    def test_equal_distance_routes_turn_at_center_in_both_directions(self):
        grid = dict(x=[0, 14, 28], y=[0, 14, 28])
        stations = [(0, 14), (14, 0), (28, 14), (14, 28)]
        for i, start in enumerate(stations):
            for goal in (stations[(i+1) % 4], stations[(i-1) % 4]):
                with self.subTest(start=start, goal=goal):
                    self.assertEqual(plan_route(start, goal, grid), [(14, 14), goal])

    def test_nonuniform_negative_grid_uses_middle_intersection(self):
        grid = dict(x=[-30, -10, 50], y=[-40, 20, 60])
        self.assertEqual(plan_route((-30, 20), (-10, -40), grid),
                         [(-10, 20), (-10, -40)])

    def test_straight_routes_and_actual_origin_destination_do_not_detour(self):
        grid = dict(x=[0, 14, 28], y=[0, 14, 28])
        for start, goal in [((0, 14), (28, 14)), ((14, 0), (14, 28)),
                            ((0, 14), (0, 0)), ((0, 7), (0, 21))]:
            with self.subTest(start=start, goal=goal):
                self.assertEqual(plan_route(start, goal, grid), [goal])
        self.assertEqual(plan_route((14, 14), (14, 14), grid), [])

    def test_nx_transit_center_turn_is_direct_but_arrival_keeps_excursion(self):
        """An offset middle lane is still the transit turn site, not the midpoint."""
        config = copy.deepcopy(DEFAULT)
        config.update(grid=dict(x=[0, 100, 300], y=[0, 100, 300]),
                      turn_inset_enabled=True, turn_inset_steps=20)
        config['stations'] = [dict(name=name, x=x, y=y, heading=0, arrival_enabled=False)
                              for name, x, y in [('O', 0, 0), ('A', 0, 100), ('B', 100, 0)]]
        config['route'] = [dict(station=2, hold=1, arrival_enabled=True, heading=9000/100)]
        runner = NavigationRunner(SimpleNamespace())
        runner.config = config
        runner.position = (0, 100)
        runner.planned_heading = 9000
        runner.tolerance = 20
        runner.leg = 0
        runner.entries = mission_waypoints(config, runner.position)
        runner.queue = []
        runner.plan_leg()
        self.assertEqual(runner.queue[:4], [
            (0, 0, 0, 9000, 'correct'), (1, 100, 100, 9000, 'travel'),
            (0, 0, 0, 18000, 'turn'), (1, 100, 0, 18000, 'travel')])
        self.assertEqual([a[-1] for a in runner.queue[4:]],
                         ['inward', 'turn', 'return', 'settle', 'arrival'])
        self.assertEqual(runner.queue[-3][1:3], (100, 0))


if __name__ == '__main__':
    unittest.main()
