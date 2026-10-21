"""Station/route split: arbitrary leg counts, repeated stations, holds.

Runs against the demo board only; no physical motor or IMU access. The route
0ABCDBCDAO is the field mission that motivated the split, so it is the case
exercised end to end here.
"""
import copy
import tempfile
import unittest
from pathlib import Path

from host.navigation import (MissionConfig, DEFAULT, ORIGIN, validated,
                             mission_waypoints, path_checksum, validate_path,
                             station_point)
from host.protocol import encode, decode
from host.simulator import SimulatedBoard

FIELD_ORDER = ("A", "B", "C", "D", "B", "C", "D", "A", "O")


def field_config(holds=()):
    """The nine-leg field route over the five default stations."""
    config = copy.deepcopy(DEFAULT)
    names = [station["name"] for station in config["stations"]]
    config["route"] = [{"station": names.index(name),
                        "hold": 1 if index in holds else 0}
                       for index, name in enumerate(FIELD_ORDER)]
    return config


class RouteSchemaTests(unittest.TestCase):
    def test_nine_leg_route_with_repeated_stations_plans_and_validates(self):
        config = validated(field_config())
        self.assertEqual(len(config["stations"]), 5)
        self.assertEqual(len(config["route"]), 9)
        start = station_point(config, ORIGIN)
        entries = mission_waypoints(config, start)
        self.assertTrue(validate_path(entries, start, config["stations"],
                                      config["route"], config["grid"]))
        # One arrival per leg, in order, and the last leg returns to O.
        arrivals = [entry for i, entry in enumerate(entries)
                    if i + 1 == len(entries) or entries[i + 1][2] != entry[2]]
        self.assertEqual(len(arrivals), 9)
        self.assertEqual([a[2] for a in arrivals], list(range(9)))
        self.assertEqual(arrivals[-1][:2], tuple(start))
        # B, C and D are each visited twice, at the same coordinate both times.
        for name in ("B", "C", "D"):
            index = [s["name"] for s in config["stations"]].index(name)
            visits = [a[:2] for a, leg in zip(arrivals, config["route"])
                      if leg["station"] == index]
            self.assertEqual(len(visits), 2)
            self.assertEqual(visits[0], visits[1])

    def test_route_may_be_one_leg_and_station_count_is_independent(self):
        config = copy.deepcopy(DEFAULT)
        config["route"] = [{"station": 1, "hold": 0}]
        validated(config)
        # A station the route never visits is still legal: the table is config,
        # not an itinerary.
        config["route"] = [{"station": 2, "hold": 1}]
        self.assertEqual(len(validated(config)["route"]), 1)

    def test_schema_rejects_bad_routes_and_duplicate_names(self):
        for route in ([], [{"station": 9, "hold": 0}], [{"station": 1}],
                      [{"station": 1, "hold": 2}], [{"station": -1, "hold": 0}],
                      [{"station": 1, "hold": 0}] * 33):
            config = copy.deepcopy(DEFAULT)
            config["route"] = route
            with self.assertRaises(ValueError):
                validated(config)
        config = copy.deepcopy(DEFAULT)
        config["stations"][2]["name"] = "A"
        with self.assertRaises(ValueError):
            validated(config)
        for name in ("", "x" * 9, "a b", "站"):
            config = copy.deepcopy(DEFAULT)
            config["stations"][2]["name"] = name
            with self.assertRaises(ValueError):
                validated(config)

    def test_commands_declare_counts_before_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = MissionConfig(Path(directory) / "nav.json")
            mission.save(field_config(holds=(1, 2)))
            bodies = mission.commands()
        self.assertEqual(bodies[0], "NAV EXEC")
        self.assertEqual(bodies[1], "NAV STATIONS 5")
        self.assertIn("NAV LEGS 9", bodies)
        # Counts must precede the rows they size, or the board cannot verify.
        self.assertLess(bodies.index("NAV STATIONS 5"), bodies.index("NAV POINT 0 0 0"))
        self.assertLess(bodies.index("NAV LEGS 9"), bodies.index("NAV LEG 0 1 0"))
        self.assertIn("NAV LEG 1 2 1", bodies)
        self.assertIn("NAV LEG 2 3 1", bodies)
        self.assertEqual(bodies[-1], "NAV START 1 HOST")


class DemoMissionTests(unittest.TestCase):
    def setUp(self):
        self.now = [0.]
        self.board = SimulatedBoard(clock=lambda: self.now[0])
        self.sequence = 0

    def send(self, body):
        self.sequence += 1
        return decode(self.board.receive(encode(self.sequence, body)))

    def upload(self, config):
        with tempfile.TemporaryDirectory() as directory:
            mission = MissionConfig(Path(directory) / "nav.json")
            mission.save(config)
            self.send("NAV RESET 0 0")
            self.send("PING")
            self.send("ARM 1")
            # Board-only compatibility test; current NX scheduling is covered
            # by the Console mission/turn-inset tests and real Web SIM flow.
            for body in mission.commands(host_control=False):
                self.assertTrue(self.send(body).ok, body)
            return mission

    def run_until(self, predicate, limit=40000):
        for _ in range(limit):
            self.now[0] += .02
            self.send("PING")
            fields = self.send("NAV STAT").fields
            if predicate(fields):
                return fields
        self.fail("demo never reached the expected state")

    def test_nine_leg_mission_reaches_every_station_in_order(self):
        mission = self.upload(field_config())
        reached, seen = [], None
        for _ in range(40000):
            self.now[0] += .02
            self.send("PING")
            nav = self.send("NAV STAT").fields
            if nav["state"] == "waiting" and seen != nav["leg"]:
                seen = nav["leg"]
                reached.append(mission.station_name(int(nav["station"])))
            if nav["state"] == "done":
                break
        else:
            self.fail("demo never finished the nine-leg mission")
        self.assertEqual(reached, list(FIELD_ORDER))
        self.assertEqual(self.send("NAV STAT").fields["legs"], "9")

    def test_held_leg_waits_for_go_and_rejects_the_wrong_leg(self):
        # Hold on leg index 1, the first arrival at B.
        self.upload(field_config(holds=(1,)))
        fields = self.run_until(lambda f: f["held"] == "1")
        self.assertEqual(fields["leg"], "2")
        held_at = (fields["x"], fields["y"])
        # The dwell timer must not release a held leg on its own.
        for _ in range(500):
            self.now[0] += .02
            self.send("PING")
            self.assertEqual(self.send("NAV STAT").fields["held"], "1")
        path_id = self.send("NAV STAT").fields["path_id"]
        # A GO naming another leg cannot start this one.
        self.assertFalse(self.send("NAV GO {} 5".format(path_id)).ok)
        self.assertEqual(self.send("NAV STAT").fields["held"], "1")
        self.assertEqual((self.send("NAV STAT").fields["x"],
                          self.send("NAV STAT").fields["y"]), held_at)
        self.assertTrue(self.send("NAV GO {} 1".format(path_id)).ok)
        fields = self.run_until(lambda f: f["leg"] != "2" or f["state"] == "done")
        self.assertEqual(self.send("NAV STAT").fields["held"], "0")
