"""Nano/PC lane-network planner; real-time execution belongs to H743.

Grid values are pulse coordinates, not obstacle footprints or localization.
Equal-length routes prefer turns at the middle-lane intersection.
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

    Dijkstra keeps the incoming axis so equal distances can prefer fewer
    noncentral turns, then fewer total turns and segments. Distance remains
    primary: direct lanes never detour just to visit the center. This is NX
    policy, not a vehicle-footprint check. Callers validate increasing axes.
    """
    start, goal = tuple(start), tuple(goal)
    if not on_grid(start, grid) or not on_grid(goal, grid):
        raise ValueError('起点或目标不在田字格通道线上。')
    if start == goal:
        return []
    nodes = [(x, y) for x in grid['x'] for y in grid['y']] + [start, goal]
    center = (grid['x'][1], grid['y'][1])
    # Three states per node: no incoming edge, horizontal, vertical. Zero-length
    # edges are skipped so duplicate start/goal nodes cannot erase turn costs.
    costs, previous, used = [None] * 33, [-1] * 33, set()
    initial = 9 * 3
    costs[initial] = (0, 0, 0, 0)
    for _ in range(33):
        available = [i for i in range(33) if i not in used and costs[i] is not None]
        if not available:
            raise ValueError('田字格通道没有可用路线。')
        current = min(available, key=lambda i: costs[i])
        node, incoming = divmod(current, 3)
        if node == 10:
            break
        used.add(current)
        a = nodes[node]
        for j, b in enumerate(nodes):
            if a == b or not ((a[0] == b[0] and a[0] in grid['x']) or
                             (a[1] == b[1] and a[1] in grid['y'])):
                continue
            axis = 1 if a[0] != b[0] else 2
            destination = j * 3 + axis
            if destination in used:
                continue
            turning = bool(incoming and incoming != axis)
            distance, outer, turns, segments = costs[current]
            candidate = (distance + abs(a[0]-b[0]) + abs(a[1]-b[1]),
                         outer + int(turning and a != center),
                         turns + int(turning), segments + 1)
            if costs[destination] is None or candidate < costs[destination]:
                costs[destination], previous[destination] = candidate, current
    reverse = []
    while current != initial:
        reverse.append(nodes[current // 3])
        current = previous[current]
    return list(reversed(reverse))
