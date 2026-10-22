"""Center route preferences at the planner and NX action scheduling boundary."""
import copy
import itertools
import unittest
from types import SimpleNamespace

from host.navigation import DEFAULT, mission_waypoints
from host.navigation_grid import plan_route
from host.navigation_runner import NavigationRunner


class CenterNavigationTests(unittest.TestCase):
    def test_shifted_and_unequal_grids_keep_lane_invariants(self):
        """Every planned route stays on lanes, moves on one axis, ends at the goal.

        The browser-side preview planner (`planGridRoute` in app.js) was removed
        together with the draft map, so the旧的 JS/Python 逐例对照 no longer has a
        counterpart. The Python planner is still the only one that produces the
        uploaded route, so its lane invariants are asserted directly over the
        same shifted, unequal and negative grids, including starts between
        crossings.
        """
        for grid in [dict(x=[0, 14, 28], y=[0, 14, 28]),
                     dict(x=[-30, -10, 50], y=[-40, 20, 60]),
                     dict(x=[0, 1, 80], y=[-100, -1, 0])]:
            points = [(x, y) for x in grid['x'] for y in grid['y']]
            points += [(grid['x'][0], (grid['y'][0]+grid['y'][1])//2),
                       ((grid['x'][1]+grid['x'][2])//2, grid['y'][2])]
            for start, goal in itertools.product(points, points):
                with self.subTest(grid=grid, start=start, goal=goal):
                    route = plan_route(start, goal, grid)
                    if start == goal:
                        self.assertEqual(route, [])
                        continue
                    self.assertEqual(tuple(route[-1]), goal)
                    previous = tuple(start)
                    for step in route:
                        self.assertNotEqual(tuple(step), previous)
                        changed = sum(1 for a, b in zip(previous, step) if a != b)
                        self.assertEqual(changed, 1, '通道段只能是横或竖')
                        if step[0] != previous[0]:
                            # 横移段贴着一条横通道，纵坐标保持不变
                            self.assertEqual(step[1], previous[1])
                            self.assertIn(previous[1], grid['y'])
                        else:
                            # 纵移段贴着一条竖通道，横坐标保持不变
                            self.assertEqual(step[0], previous[0])
                            self.assertIn(previous[0], grid['x'])
                        self.assertTrue(grid['x'][0] <= step[0] <= grid['x'][2])
                        self.assertTrue(grid['y'][0] <= step[1] <= grid['y'][2])
                        previous = tuple(step)

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
