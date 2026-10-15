"""Nano/PC lane-network planner; real-time execution belongs to H743.

Grid values are pulse coordinates, not obstacle footprints or localization.
The nine intersections use a fixed order to resolve equal-cost ties.
"""
import math


def round_pulse(value):
    """Round like firmware roundf, including negative half-pulse positions."""
    return math.floor(value + .5) if value >= 0 else math.ceil(value - .5)


def on_grid(point, grid):
    """Accept bounded lane centres, including positions between intersections."""
    x, y = point
    return (grid['x'][0] <= x <= grid['x'][2] and grid['y'][0] <= y <= grid['y'][2]
            and (x in grid['x'] or y in grid['y']))


def plan_route(start, goal, grid):
    """Return start-exclusive orthogonal waypoints; reject off-network points.

    Uses a small Dijkstra graph, with no diagonal/L-route fallback.
    Callers must validate the three strictly increasing coordinates per axis.
    """
    start, goal = tuple(start), tuple(goal)
    if not on_grid(start, grid) or not on_grid(goal, grid):
        raise ValueError('起点或目标不在田字格通道线上。')
    nodes = [(x, y) for x in grid['x'] for y in grid['y']] + [start, goal]
    distances, previous, used = [float('inf')] * 11, [-1] * 11, set()
    distances[9] = 0
    for _ in range(11):
        current = min((i for i in range(11) if i not in used), key=lambda i: distances[i])
        if current == 10:
            break
        used.add(current)
        a = nodes[current]
        for j, b in enumerate(nodes):
            if j in used or not ((a[0] == b[0] and a[0] in grid['x']) or
                                  (a[1] == b[1] and a[1] in grid['y'])):
                continue
            candidate = distances[current] + abs(a[0] - b[0]) + abs(a[1] - b[1])
            if candidate < distances[j]:
                distances[j], previous[j] = candidate, current
    reverse, current = [], 10
    while current != 9:
        if previous[current] < 0:
            raise ValueError('田字格通道没有可用路线。')
        reverse.append(nodes[current])
        current = previous[current]
    result, last = [], start
    for point in reversed(reverse):
        if point != last:
            result.append(point)
            last = point
    return result
