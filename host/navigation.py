"""Nano owns mission parameters and planning; H743 owns route execution.

Coordinates count wheel pulses in the departure frame. Saving
changes the next mission only; it never moves, resets or re-arms the board.
"""
import copy
import binascii
import json
import math
import os
import tempfile
from pathlib import Path
from .navigation_grid import on_grid, plan_route

CONFIG_PATH = Path(__file__).with_name("navigation.json")
LABELS = ("O", "A", "B", "C", "D")
DEFAULT = {"points": dict(zip(LABELS, ([0, 0], [0, 14000], [14000, 0],
                                      [14000, 28000], [28000, 14000]))),
           "headings": dict.fromkeys(LABELS, 0),
           "grid": {"x": [0, 14000, 28000], "y": [0, 14000, 28000]},
           "sps": 500, "accel": 2000, "dwell_ms": 3000}
TASK_ROUTE = (1, 2, 3, 4, 1)
PATH_MAX = 64


def mission_waypoints(config, start):
    """Plan all five task legs, retaining an entry for coincident arrivals."""
    config = validated(config)
    last, entries = tuple(start), []
    for leg, target in enumerate(TASK_ROUTE):
        goal = tuple(config["points"][LABELS[target]])
        for x, y in plan_route(last, goal, config["grid"]) or [goal]:
            entries.append((x, y, leg))
        last = goal
    if len(entries) > PATH_MAX:
        raise ValueError("路径超过 H743 的 64 个端点容量。")
    return entries


def path_checksum(entries):
    """CRC over canonical indexed ASCII rows; independent of wire sequence IDs."""
    rows = "".join("{} {} {} {}\n".format(i, *entry) for i, entry in enumerate(entries))
    return binascii.crc_hqx(rows.encode("ascii"), 0xffff)


def validate_path(entries, start, points, grid):
    """Demo's whole-route validator: legal lanes and exactly five task arrivals."""
    if not 5 <= len(entries) <= PATH_MAX or not on_grid(start, grid):
        return False
    last, expected = tuple(start), 0
    if len(points) != 5 or any(not on_grid(p, grid) for p in points.values()):
        return False
    for i, (x, y, leg) in enumerate(entries):
        final = i + 1 == len(entries) or entries[i + 1][2] != expected
        connected = ((last[0] == x and x in grid['x']) or
                     (last[1] == y and y in grid['y']))
        # Coincident task arrivals can lie between intersections on any lane.
        connected = connected or (last == (x, y) and on_grid(last, grid))
        if expected > 4 or leg != expected or not on_grid((x, y), grid) or not connected or \
                (not final and last == (x, y)) or \
                (final and (x, y) != tuple(points[TASK_ROUTE[expected]])):
            return False
        last = (x, y)
        if final:
            expected += 1
    return expected == 5


def validated(config):
    """Reject incomplete/noninteger/out-of-range parameters before uploading."""
    if isinstance(config, dict):
        config = copy.deepcopy(config)
        config.setdefault("headings", dict.fromkeys(LABELS, 0))
        config.setdefault("grid", copy.deepcopy(DEFAULT["grid"]))
    if not isinstance(config, dict) or set(config) != set(DEFAULT):
        raise ValueError("参数必须包含 points、headings、grid、sps、accel、dwell_ms。")
    points = config["points"]
    if not isinstance(points, dict) or set(points) != set(LABELS):
        raise ValueError("必须提供 O、A、B、C、D 五个坐标。")
    for name in LABELS:
        point = points[name]
        if not isinstance(point, (list, tuple)) or len(point) != 2 or any(
                type(value) is not int or abs(value) > 1000000 for value in point):
            raise ValueError("坐标必须是 -1000000～1000000 的整数脉冲。")
    grid = config["grid"]
    if not isinstance(grid, dict) or set(grid) != {"x", "y"}:
        raise ValueError("田字格必须提供三条竖线 x 和三条横线 y。")
    for values in grid.values():
        if not isinstance(values, (list, tuple)) or len(values) != 3 or any(
                type(v) is not int or abs(v) > 1000000 for v in values) or not values[0] < values[1] < values[2]:
            raise ValueError("每组通道坐标必须是范围内的三个递增整数脉冲。")
    for name, point in points.items():
        if not on_grid(point, grid):
            raise ValueError("{} 点不在田字格通道线上或超出通道范围。".format(name))
    headings = config["headings"]
    if not isinstance(headings, dict) or set(headings) != set(LABELS):
        raise ValueError("必须提供五个点的到达姿态。")
    for name, angle in headings.items():
        if type(angle) not in (int, float) or not math.isfinite(angle) or abs(angle) > 180 or \
                abs(angle * 100 - round(angle * 100)) > 1e-7:
            raise ValueError("到达姿态范围 -180～180°，最多两位小数。")
    if headings["O"] != 0:
        raise ValueError("出发姿态 O 固定为 0°。")
    for key, low, high in (("sps", 1, 5000), ("accel", 1, 20000), ("dwell_ms", 0, 60000)):
        if type(config[key]) is not int or not low <= config[key] <= high:
            raise ValueError("{} 范围 {}～{}。".format(key, low, high))
    return copy.deepcopy(config)


class MissionConfig:
    """JSON persistence with atomic replacement; corrupt files fail visibly."""
    def __init__(self, path=CONFIG_PATH):
        self.path = Path(path)
        self.value = validated(json.loads(self.path.read_text(encoding="utf-8"))) \
            if self.path.exists() else copy.deepcopy(DEFAULT)

    def snapshot(self):
        return copy.deepcopy(self.value)

    def save(self, config):
        value = validated(config)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                             suffix=".tmp", delete=False) as output:
                temporary = output.name
                json.dump(value, output, ensure_ascii=False, indent=2)
                output.write("\n")
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        self.value = value
        return self.snapshot()

    def commands(self, start=None, path_id=1):
        """One full planned mission; callers read fresh NAV STAT before upload.

        Default origin is for offline previews/tests. Live callers must provide
        the board's current rounded XY. Each command waits for its own ACK.
        """
        config = self.snapshot()
        start = tuple(config["points"]["O"] if start is None else start)
        if not 1 <= path_id <= 2147483647:
            raise ValueError("路径编号超出范围。")
        entries = mission_waypoints(config, start)
        return ["NAV POINT {} {} {}".format(index, *config["points"][name])
                for index, name in enumerate(LABELS)] + [
                    "NAV GRID {} {} {} {}".format(axis.upper(), *config["grid"][axis])
                    for axis in ("x", "y")] + [
                    "NAV ATTITUDE {} {}".format(index, round(config["headings"][name] * 100))
                    for index, name in enumerate(LABELS)] + [
                    "NAV CONFIG {sps} {accel} {dwell_ms}".format(**config),
                    "NAV BEGIN {} {} {} {}".format(path_id, len(entries), *start)] + [
                    "NAV WP {} {} {} {} {}".format(path_id, i, *entry)
                    for i, entry in enumerate(entries)] + [
                    "NAV COMMIT {} {}".format(path_id, path_checksum(entries)),
                    "NAV START {}".format(path_id)]
