"""NX owns parameters, route planning and action scheduling; H743 executes.

Coordinates count wheel pulses in the departure frame. Saving
changes the next mission only; it never moves, resets or re-arms the board.

Stations and route are separate: a station carries its coordinate and arrival
attitude and is edited once, while the route is any sequence of station
indices. One station may appear in the route repeatedly, each occurrence with
its own hold flag, because what the car does on the first visit to a point is
not what it does on the second.
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
# Station 0 remains the map origin, including when RESET establishes X's pose.
ORIGIN = 0
STATION_MAX = 32
LEG_MAX = 32
PATH_MAX = 64
NAME_MAX = 8
DEFAULT_NAMES = ("O", "A", "B", "C", "D")
DEFAULT_COORDS = ([0, 0], [0, 14000], [14000, 0], [14000, 28000], [28000, 14000])
# Preserves the historical O->A->B->C->D->A mission for configs predating
# the station/route split, so an existing field setup behaves unchanged.
DEFAULT_ROUTE = (1, 2, 3, 4, 1)
DEFAULT = {
    "stations": [{"name": name, "x": coord[0], "y": coord[1], "heading": 0, "arrival_enabled": False}
                 for name, coord in zip(DEFAULT_NAMES, DEFAULT_COORDS)],
    "route": [{"station": index, "hold": 0} for index in DEFAULT_ROUTE],
    "grid": {"x": [0, 14000, 28000], "y": [0, 14000, 28000]},
    "sps": 500, "accel": 2000, "dwell_ms": 3000,
    "turn_inset_enabled": False, "turn_inset_steps": 1000,
    "start_station": 0,
}


def migrate(config):
    """Accept the pre-station schema so a deployed navigation.json still loads.

    The old file had points/headings keyed by O..D plus a fixed five-leg route.
    Loading converts it into stations+route without changing coordinates or
    saved angles. Arrival orientation stays disabled until explicitly enabled.
    """
    if not isinstance(config, dict) or "points" not in config:
        return config
    points, headings = config.get("points"), config.get("headings")
    if not isinstance(points, dict):
        return config
    if not isinstance(headings, dict):
        headings = {}
    stations = []
    for name in DEFAULT_NAMES:
        if name not in points:
            return config
        coord = points[name]
        if not isinstance(coord, (list, tuple)) or len(coord) != 2:
            return config
        stations.append({"name": name, "x": coord[0], "y": coord[1],
                         "heading": headings.get(name, 0)})
    migrated = {key: config[key] for key in ("grid", "sps", "accel", "dwell_ms", "turn_inset_enabled", "turn_inset_steps", "start_station")
                if key in config}
    migrated["stations"] = stations
    migrated["route"] = [{"station": index, "hold": 0} for index in DEFAULT_ROUTE]
    migrated.setdefault("grid", copy.deepcopy(DEFAULT["grid"]))
    return migrated


def _validated_stations(stations, grid, executable):
    """Check station data; require lane membership only for execution."""
    if not isinstance(stations, list) or not 2 <= len(stations) <= STATION_MAX:
        raise ValueError("站点数量必须是 2～{} 个。".format(STATION_MAX))
    names = set()
    result = []
    for position, station in enumerate(stations):
        if not isinstance(station, dict) or not {"name", "x", "y", "heading"} <= set(station) or set(station) - {"name", "x", "y", "heading", "arrival_enabled"}:
            raise ValueError("每个站点必须包含 name、x、y、heading。")
        enabled = station.get('arrival_enabled', False)
        if type(enabled) is not bool:
            raise ValueError('到点调姿开关必须为布尔值。')
        name = station["name"]
        if not isinstance(name, str) or not name.strip() or len(name) > NAME_MAX or \
                not name.isascii() or not name.replace("_", "").isalnum():
            raise ValueError("站点名必须是 1～{} 个 ASCII 字母数字或下划线。".format(NAME_MAX))
        if name in names:
            raise ValueError("站点名 {} 重复，无法在状态回报里区分。".format(name))
        names.add(name)
        for axis in ("x", "y"):
            if type(station[axis]) is not int or abs(station[axis]) > 1000000:
                raise ValueError("坐标必须是 -1000000～1000000 的整数脉冲。")
        heading = station["heading"]
        if type(heading) not in (int, float) or not math.isfinite(heading) or \
                abs(heading) > 180 or abs(heading * 100 - round(heading * 100)) > 1e-7:
            raise ValueError("到达姿态范围 -180～180°，最多两位小数。")
        if position == ORIGIN and heading != 0:
            raise ValueError("出发站点的姿态固定为 0°。")
        if executable and not on_grid((station["x"], station["y"]), grid):
            raise ValueError("站点 {} 不在田字格通道线上或超出通道范围。".format(name))
        result.append({"name": name, "x": station["x"], "y": station["y"],
                       "heading": heading, "arrival_enabled": enabled})
    return result


def _validated_route(route, station_count):
    """A leg names an existing station and says whether to hold on arrival."""
    if not isinstance(route, list) or not 1 <= len(route) <= LEG_MAX:
        raise ValueError("路线必须是 1～{} 段。".format(LEG_MAX))
    result = []
    for leg in route:
        if not isinstance(leg, dict) or not {"station", "hold"} <= set(leg) or set(leg) - {"station", "hold", "task", "outward_steps", "right_steps", "arrival_enabled", "heading"}:
            raise ValueError("每段路线必须包含 station 和 hold。")
        index = leg["station"]
        if type(index) is not int or not 0 <= index < station_count:
            raise ValueError("路线引用了不存在的站点下标。")
        if leg["hold"] not in (0, 1, True, False):
            raise ValueError("hold 必须是 0 或 1。")
        item = {"station": index, "hold": int(leg["hold"])}
        if 'arrival_enabled' in leg:
            if type(leg['arrival_enabled']) is not bool:
                raise ValueError('路线段到点调姿开关必须为布尔值。')
            item['arrival_enabled'] = leg['arrival_enabled']
        if 'heading' in leg:
            angle = leg['heading']
            if type(angle) not in (int,float) or not math.isfinite(angle) or abs(angle)>180 or abs(angle*100-round(angle*100))>1e-7:
                raise ValueError('路线段到达角度范围 -180～180°，最多两位小数。')
            item['heading'] = angle
        if 'task' in leg:
            if type(leg['task']) is not int or not 0 <= leg['task'] <= 4:
                raise ValueError('视觉子任务编号必须为 0～4。')
            if leg['task'] and not item['hold']:
                raise ValueError('视觉子任务需要勾选到点挂起。')
            item['task'] = leg['task']
        for key, default in (('outward_steps', 400), ('right_steps', 800)):
            if key in leg or item.get('task') == 1:
                value = leg.get(key, default)
                if type(value) is not int or not 0 <= value <= 100000:
                    raise ValueError('任务位移必须为 0～100000 的整数脉冲。')
                item[key] = value
        result.append(item)
    return result


def validated(config, executable=True):
    """Validate data for storage, plus route constraints before execution.

    Persistence accepts intermediate station coordinates so field and station
    edits can be saved independently. Upload callers retain strict checks.
    """
    config = migrate(config)
    if isinstance(config, dict):
        config = copy.deepcopy(config)
        config.setdefault("grid", copy.deepcopy(DEFAULT["grid"]))
        config.setdefault("turn_inset_enabled", False)
        config.setdefault("turn_inset_steps", 1000)
        config.setdefault("start_station", ORIGIN)
    if not isinstance(config, dict) or set(config) != set(DEFAULT):
        raise ValueError("巡航参数字段缺失或包含未知字段。")
    grid = config["grid"]
    if not isinstance(grid, dict) or set(grid) != {"x", "y"}:
        raise ValueError("田字格必须提供三条竖线 x 和三条横线 y。")
    for values in grid.values():
        if not isinstance(values, (list, tuple)) or len(values) != 3 or any(
                type(v) is not int or abs(v) > 1000000 for v in values) or \
                not values[0] < values[1] < values[2]:
            raise ValueError("每组通道坐标必须是范围内的三个递增整数脉冲。")
    stations = _validated_stations(config["stations"], grid, executable)
    route = _validated_route(config["route"], len(stations))
    start = config['start_station']
    if type(start) is not int or not 0 <= start < len(stations) or (start != ORIGIN and stations[start]['name'] != 'X'):
        raise ValueError('巡航起点请选择 O 或 X。')
    if executable and start != ORIGIN and (stations[start]['x'], stations[start]['y'], stations[start]['heading']) != (0, 28800, -180):
        raise ValueError('X 起点必须为 (0, 28800)，朝向 -180°。')
    if executable and start != ORIGIN and stations[route[0]['station']]['name'] != 'A':
        raise ValueError('从 X 出发的第一段必须前往 A 点。')
    if type(config['turn_inset_enabled']) is not bool:
        raise ValueError('转前内移开关必须为布尔值。')
    if type(config['turn_inset_steps']) is not int or not 1 <= config['turn_inset_steps'] <= 100000:
        raise ValueError('转前内移范围 1～100000 整数脉冲。')
    for key, low, high in (("sps", 1, 5000), ("accel", 1, 20000), ("dwell_ms", 0, 60000)):
        if type(config[key]) is not int or not low <= config[key] <= high:
            raise ValueError("{} 范围 {}～{}。".format(key, low, high))
    return {"stations": stations, "route": route,
            "grid": {axis: list(grid[axis]) for axis in ("x", "y")},
            "sps": config["sps"], "accel": config["accel"],
            "dwell_ms": config["dwell_ms"],
            "turn_inset_enabled": config['turn_inset_enabled'],
            "turn_inset_steps": config['turn_inset_steps'], "start_station": start}


def reference_command(config):
    """Assert a physically placed O/X pose while disarmed; never move the car.

    X needs the extended RESET so station zero remains O and the first fresh
    START yaw represents -180 degrees in the existing field coordinate system.
    """
    config = validated(config)
    origin = station_point(config, ORIGIN)
    index = config.get('start_station', ORIGIN)
    if index == ORIGIN:
        return 'NAV RESET {} {}'.format(*origin)
    station = config['stations'][index]
    return 'NAV RESET {} {} {} {} {}'.format(*origin, station['x'], station['y'], round(station['heading']*100))


def arrival(config, leg):
    """Resolve this visit's explicit override; zero degrees never means disabled."""
    step = config['route'][leg]
    station = config['stations'][step['station']]
    return (step.get('arrival_enabled', station.get('arrival_enabled', False)),
            step.get('heading', station['heading']))


def station_point(config, index):
    station = config["stations"][index]
    return (station["x"], station["y"])


def mission_waypoints(config, start):
    """Plan every leg of the configured route, in order.

    A coincident arrival keeps one entry so the board still orients and dwells
    at a station it is already standing on.
    """
    config = validated(config)
    last, entries = tuple(start), []
    for leg, step in enumerate(config["route"]):
        goal = station_point(config, step["station"])
        for x, y in plan_route(last, goal, config["grid"]) or [goal]:
            entries.append((x, y, leg))
        last = goal
    if len(entries) > PATH_MAX:
        raise ValueError("路径超过 H743 的 {} 个端点容量。".format(PATH_MAX))
    return entries


def path_checksum(entries):
    """CRC over canonical indexed ASCII rows; independent of wire sequence IDs."""
    rows = "".join("{} {} {} {}\n".format(i, *entry) for i, entry in enumerate(entries))
    return binascii.crc_hqx(rows.encode("ascii"), 0xffff)


def validate_path(entries, start, stations, route, grid):
    """Demo's whole-route validator: legal lanes and one arrival per leg."""
    legs = len(route)
    if not legs or len(entries) < legs or len(entries) > PATH_MAX or not on_grid(start, grid):
        return False
    if not 2 <= len(stations) <= STATION_MAX or \
            any(not on_grid((s["x"], s["y"]), grid) for s in stations):
        return False
    last, expected = tuple(start), 0
    for i, (x, y, leg) in enumerate(entries):
        final = i + 1 == len(entries) or entries[i + 1][2] != expected
        connected = ((last[0] == x and x in grid['x']) or
                     (last[1] == y and y in grid['y']))
        # Coincident task arrivals can lie between intersections on any lane.
        connected = connected or (last == (x, y) and on_grid(last, grid))
        if expected >= legs or leg != expected or not on_grid((x, y), grid) or \
                not connected or (not final and last == (x, y)):
            return False
        if final:
            goal = stations[route[expected]["station"]]
            if (x, y) != (goal["x"], goal["y"]):
                return False
            expected += 1
        last = (x, y)
    return expected == legs


class MissionConfig:
    """JSON persistence with atomic replacement; corrupt files fail visibly."""
    def __init__(self, path=CONFIG_PATH):
        self.path = Path(path)
        self.value = validated(json.loads(self.path.read_text(encoding="utf-8")), executable=False) \
            if self.path.exists() else copy.deepcopy(DEFAULT)

    def snapshot(self):
        return copy.deepcopy(self.value)

    def save(self, config):
        value = validated(config, executable=False)
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

    def station_name(self, index):
        """Map a board-reported station index back to its configured name."""
        stations = self.value["stations"]
        return stations[index]["name"] if 0 <= index < len(stations) else "?"

    def commands(self, start=None, path_id=1, config=None, host_control=True):
        """One full planned mission; callers read fresh NAV STAT before upload.

        Default origin is for offline previews/tests. Live callers must provide
        the board's current rounded XY. Each command waits for its own ACK.
        Counts are declared before their rows so the board can size and verify
        both tables without assuming any fixed mission shape.
        """
        config = self.snapshot() if config is None else validated(config)
        start = tuple(station_point(config, config.get('start_station', ORIGIN)) if start is None else start)
        if not 1 <= path_id <= 2147483647:
            raise ValueError("路径编号超出范围。")
        entries = mission_waypoints(config, start)
        stations, route = config["stations"], config["route"]
        commands = ["NAV STATIONS {}".format(len(stations))] + [
            "NAV POINT {} {} {}".format(index, station["x"], station["y"])
            for index, station in enumerate(stations)] + [
            "NAV ATTITUDE {} {}".format(index, round(station["heading"] * 100))
            for index, station in enumerate(stations)] + [
            "NAV GRID {} {} {} {}".format(axis.upper(), *config["grid"][axis])
            for axis in ("x", "y")] + [
            "NAV LEGS {}".format(len(route))] + [
            "NAV LEG {} {} {}".format(index, step["station"], step["hold"])
            for index, step in enumerate(route)] + [
            "NAV CONFIG {sps} {accel} {dwell_ms}".format(**config)] + (
            ["NAV INSET {}".format(config['turn_inset_steps'])] if config['turn_inset_enabled'] and not host_control else []) + [
            "NAV BEGIN {} {} {} {}".format(path_id, len(entries), *start)] + [
            "NAV WP {} {} {} {} {}".format(path_id, i, *entry)
            for i, entry in enumerate(entries)] + [
            "NAV COMMIT {} {}".format(path_id, path_checksum(entries)),
            "NAV START {}{}".format(path_id, ' HOST' if host_control else '')]
        return ['NAV EXEC'] + commands if host_control else commands
