"""Run the actual ARM image and demo; no physical motor/IMU access."""
import copy
import queue
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from tests import test_chassis as chassis_harness
from host.navigation import MissionConfig, DEFAULT, validated, mission_waypoints, path_checksum
from host.navigation_grid import plan_route, on_grid, round_pulse
from host.protocol import encode, decode
from host.simulator import SimulatedBoard
from host.webapp import Console
from host.imu import ImuHub
from host.transport import Connection


class ArmNavigationTests(unittest.TestCase):
    setUp = chassis_harness.ArmChassisTests.setUp
    request = chassis_harness.ArmChassisTests.request
    group_edge = chassis_harness.ArmChassisTests.group_edge

    def configure(self, points=None, attitudes=None, initial=17000, grid=None):
        points = points or [(0,0), (0,14), (14,0), (14,28), (28,14)]
        self.points, self.attitudes = points, attitudes or [0]*5
        self.last_start_sequence = None
        self.grid = {}
        for axis, name in enumerate(('x', 'y')):
            values = sorted({p[axis] for p in points})
            if len(values) == 1: values = [values[0], values[0]+14, values[0]+28]
            if len(values) == 2: values.append(values[-1]+values[-1]-values[0])
            self.grid[name] = values
        if grid is not None: self.grid = grid
        self.now, self.initial = 0, initial
        self.request("ARM 0")
        self.assertTrue(self.request('NAV RESET {} {}'.format(*points[0])).ok)
        for name, values in self.grid.items():
            self.assertTrue(self.request('NAV GRID {} {} {} {}'.format(name.upper(), *values)).ok)
        # Counts precede rows; both tables must be complete before BEGIN.
        self.assertTrue(self.request(f"NAV STATIONS {len(points)}").ok)
        for i, (x, y) in enumerate(points):
            self.assertTrue(self.request(f"NAV POINT {i} {x} {y}").ok)
            self.assertTrue(self.request(f"NAV ATTITUDE {i} {(attitudes or [0]*5)[i]}").ok)
        self.route = getattr(self, "route", None) or [1, 2, 3, 4, 1]
        self.assertTrue(self.request(f"NAV LEGS {len(self.route)}").ok)
        for i, station in enumerate(self.route):
            self.assertTrue(self.request(f"NAV LEG {i} {station} 0").ok)
        self.request("NAV CONFIG 500 2000 3000")
        self.request("ARM 1")
        self.assertTrue(self.request(f"NAV IMU {initial} 1 1").ok)

    def upload(self, entries=None, start=None, ident=42):
        start = start or (int(self.nav()['x']), int(self.nav()['y']))
        config = copy.deepcopy(DEFAULT)
        config.update(
            stations=[{"name": name, "x": p[0], "y": p[1], "heading": 0}
                      for name, p in zip('OABCD', self.points)],
            route=[{"station": s, "hold": 0} for s in getattr(self, "route", None) or [1,2,3,4,1]],
            grid=self.grid)
        entries = entries if entries is not None else mission_waypoints(config, start)
        self.assertTrue(self.request('NAV BEGIN {} {} {} {}'.format(ident,len(entries),*start)).ok)
        for i, entry in enumerate(entries):
            self.assertTrue(self.request('NAV WP {} {} {} {} {}'.format(ident,i,*entry)).ok)
        return self.request('NAV COMMIT {} {}'.format(ident,path_checksum(entries)))

    def start(self, sequence=None):
        if sequence is None or sequence != self.last_start_sequence:
            status = self.nav()
            if status['ready'] == '1' and status['state'] not in ('turning','moving','orienting','waiting','blocked'):
                self.assertTrue(self.upload().ok)
        result = self.request('NAV START', **({'sequence': sequence} if sequence else {}))
        if result.ok: self.last_start_sequence = sequence
        return result

    def nav(self):
        return self.request("NAV STAT").fields

    def tick(self, now, relative=0, feedback=True, epoch=1):
        self.now = now
        self.arm.write32(self.arm.symbols["xTickCount"], now)
        if feedback:
            heading = (self.initial + relative + 18000) % 36000 - 18000
            self.arm.call("navigation_imu", heading, 1, epoch, now)
        self.arm.call("navigation_tick", now)

    def align(self, relative):
        # Every call feeds distinct samples, completing any high pulse on STOP.
        for _ in range(5):
            self.tick(self.now+100, relative)
            if self.arm.call("motion_chassis_active") and self.nav()["state"] in ("turning", "orienting"):
                self.group_edge()
        return self.nav()

    def finish_forward(self, relative):
        for _ in range(300):
            if not self.arm.call("motion_chassis_active"):
                break
            self.group_edge()
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.tick(self.now+1, relative)

    def test_five_legs_forward_and_arrival_attitude_before_dwell(self):
        attitudes = [0,9000,-9000,18000,4500]
        self.configure(attitudes=attitudes)
        self.assertTrue(self.start().ok)
        expected = [(0,14), (14,0), (14,28), (28,14), (0,14)]
        for index, point in enumerate(expected):
            while self.nav()['state'] == 'turning':
                status = self.nav()
                bearing = int(status["target_cd"])
                self.assertIn(bearing, (0, 9000, -9000, 18000))
                start = (int(status['x']), int(status['y']))
                waypoint = (int(status['wx']), int(status['wy']))
                self.assertNotEqual(start[0] == waypoint[0], start[1] == waypoint[1])
                self.assertTrue(on_grid(waypoint, self.grid))
                self.assertEqual(self.align(bearing)["state"], "moving")
                before = [self.arm.status(i)[0] for i in range(4)]
                self.finish_forward(bearing)
                after = [self.arm.status(i)[0] for i in range(4)]
                delta = [a-b for a,b in zip(after,before)]
                self.assertEqual(delta, [-abs(delta[0]), abs(delta[0]), -abs(delta[0]), abs(delta[0])])
                self.assertGreater(delta[1], 0)
            status = self.nav()
            self.assertEqual(status["state"], "orienting")
            self.assertEqual((int(status["x"]),int(status["y"])), point)
            arrival = int(status["arrival_cd"])
            self.tick(self.now+1, bearing)
            # Exercise real rotation pulses: their sum must not move XY.
            if self.arm.call("motion_chassis_active"):
                self.group_edge(); self.group_edge()
            status = self.align(arrival)
            self.assertEqual(status["state"], "waiting")
            self.assertEqual((int(status["x"]),int(status["y"])), point)
            for command in ("CHASSIS MOVE FWD 1 100 2000", "MOVE 1 1 100 2000", "WHEELS 0 0 0 0 2000", "NAV POINT 1 2 3", "NAV ATTITUDE 1 0", "NAV GRID X 0 14 28"):
                self.assertFalse(self.request(command).ok, command)
            # Keep fresh feedback throughout the full three-second dwell.
            dwell_end = self.now + int(status["wait_ms"])
            while self.now + 100 < dwell_end:
                self.tick(self.now+100, arrival)
                self.assertEqual(self.nav()["state"], "waiting")
            self.tick(dwell_end, arrival)
            self.assertEqual(self.nav()["state"], "done" if index==4 else "turning")

    def test_only_cardinal_bearings_even_for_diagonal_goals(self):
        for point, angle in [((0,10),0),((10,0),9000),((0,-10),18000),((-10,0),-9000),((10,10),0),((-10,-10),18000)]:
            self.setUp()
            self.configure([(0,0),point,(0,0),(0,0),(0,0)])
            self.start()
            self.assertEqual(int(self.nav()["target_cd"]), angle)
            self.assertEqual(self.nav()["heading_cd"], "0")

    def test_requires_new_samples_and_stable_angle_before_forward(self):
        self.configure()
        self.start()
        self.tick(10)
        for now in (50,100,200,250):
            self.tick(now, feedback=False)
            self.assertEqual(self.nav()["state"], "turning")
            self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        # A sample outside tolerance resets the consecutive-sample confirmation.
        self.tick(260, 0)
        self.tick(360, 300)
        self.assertEqual(self.nav()["state"], "turning")
        self.assertEqual(self.align(0)["state"], "moving")

    def test_same_source_loss_pauses_all_phases_changed_source_still_faults(self):
        for phase in ("turning", "moving", "orienting", "waiting"):
            self.setUp(); self.configure(); self.start()
            if phase != "turning":
                self.align(0)
            if phase in ("orienting", "waiting"):
                self.finish_forward(0)
            if phase == "waiting":
                self.align(0)
            self.assertEqual(self.nav()["state"], phase)
            self.tick(self.now+300, feedback=False)
            before = self.nav()
            self.assertEqual((before['fault'], before['paused']), ('none', '1'))
            self.group_edge()
            self.tick(self.now+1)
            self.assertEqual(self.nav()['paused'], '1')
            self.tick(self.now+60); self.tick(self.now+60)
            self.assertEqual(self.nav()['paused'], '0')
            self.assertEqual(self.nav()['move_id'], before['move_id'])
            self.assertEqual(self.nav()['fault'], 'none')
        self.setUp(); self.configure(); self.start()
        self.assertTrue(self.request("NAV IMU 17000 0 1").ok)
        self.assertEqual((self.nav()['fault'], self.nav()['paused']), ('none', '1'))
        self.setUp(); self.configure(); self.start()
        self.tick(1, epoch=2)
        self.assertEqual(self.nav()["fault"], "imu_changed")

    def test_start_requires_fresh_imu_and_turn_timeout(self):
        self.configure([(0,0),(14,0),(0,0),(0,0),(0,0)])
        self.tick(300, feedback=False)
        self.assertFalse(self.start().ok)
        self.tick(301)
        self.start()
        for now in range(401, 60302, 100):
            self.tick(now)
        self.assertEqual(self.nav()["fault"], "turn_timeout")

    def test_rotation_shortest_wrap_and_xy_unchanged(self):
        self.configure([(0,0),(0,0),(0,0),(0,0),(0,0)], [0,-17900,0,0,0], initial=0)
        self.start()
        self.tick(1,-17920)
        self.group_edge(); self.group_edge()
        # +0.2 degrees is within the completion band; no pulses needed.
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [0]*4)
        self.tick(2,17800)  # error +3 degrees: CW, not a -357 degree turn.
        self.group_edge(); self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [-1,-1,-1,-1])
        self.assertEqual((self.nav()["x"],self.nav()["y"]), ("0","0"))

    def test_diagonal_goal_uses_two_stopped_cardinal_segments(self):
        self.configure([(0,0),(14,14),(0,0),(0,0),(0,0)])
        self.start()
        self.align(0)
        self.finish_forward(0)
        self.assertEqual((self.nav()['x'], self.nav()['y']), ('0', '14'))
        self.assertEqual(self.nav()['state'], 'turning')
        self.assertEqual(self.nav()['target_cd'], '9000')
        self.assertEqual(self.nav()['wait_ms'], '0')
        self.assertFalse(self.arm.call('motion_chassis_active'))
        self.align(9000)
        before = [self.arm.status(i)[0] for i in range(4)]
        self.finish_forward(9000)
        self.assertEqual([self.arm.status(i)[0]-before[i] for i in range(4)], [-14,14,-14,14])
        self.assertEqual((self.nav()["x"],self.nav()["y"]), ("14","14"))

    def test_deviation_recovers_translation_and_auxiliary_remains_active(self):
        self.configure(); self.start(); self.align(0)
        self.request("MOVE 5 5 100 2000")
        self.group_edge()
        self.tick(self.now+1, 600)
        self.assertEqual(self.nav()["fault"], "none")
        self.assertEqual(self.nav()['state'], 'turning')
        self.group_edge()
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.assertTrue(self.arm.status(4)[4])
        self.align(0)
        self.assertEqual(self.nav()['state'], 'moving')

    def test_obstacle_and_explicit_stop_never_resume(self):
        self.configure(); self.start()
        self.request("NAV OBSTACLE 1")
        self.assertEqual(self.nav()["state"], "blocked")
        self.request("NAV OBSTACLE 0"); self.tick(100)
        self.assertEqual(self.nav()["state"], "blocked")
        self.assertFalse(self.request("NAV REPLAN").ok)
        self.request("NAV STOP"); self.tick(200)
        self.assertEqual(self.nav()["state"], "stopped")
        self.assertTrue(self.request("CHASSIS MOVE FWD 2 100 2000").ok)
        self.assertEqual(self.nav()["ready"], "0")

    def test_start_replay_does_not_recapture_heading_or_restart(self):
        self.configure(); self.start(sequence=1000)
        self.tick(100,900)
        self.assertTrue(self.start(sequence=1000).ok)
        self.assertEqual(self.nav()["heading_cd"], "900")

    def test_partial_pose_projects_cardinal_forward_pulses(self):
        for point, angle in [((14,0),9000), ((-14,0),-9000),
                             ((0,14),0), ((0,-14),18000)]:
            self.setUp(); self.configure([(0,0),point,(0,0),(0,0),(0,0)])
            self.start(); self.align(angle)
            for _ in range(20): self.group_edge()
            self.tick(self.now+1, angle)
            status = self.nav()
            self.assertEqual(status["state"], "moving")
            self.assertEqual((int(status["x"]),int(status["y"])),
                             (10 if point[0]>0 else -10 if point[0]<0 else 0,
                              10 if point[1]>0 else -10 if point[1]<0 else 0))

    def test_restart_preserves_field_heading_and_retained_coordinates(self):
        self.configure(); self.start(); self.align(0)
        for _ in range(8): self.group_edge()
        self.request("NAV STOP")
        self.assertEqual((self.nav()["x"],self.nav()["y"]),("0","4"))
        self.tick(self.now+1,9000)
        self.assertTrue(self.start().ok)
        status=self.nav()
        self.assertEqual(status["heading_cd"],"9000")
        self.assertEqual((status["x"],status["y"]),("0","4"))
        self.assertEqual(status['target_cd'], '0')

    def test_late_feedback_cannot_hide_a_gap_or_reconnect_origin(self):
        self.configure(); self.start()
        self.tick(300)  # A new sample arrives before the next navigation tick.
        self.assertEqual((self.nav()['fault'], self.nav()['paused']), ('none', '1'))
        self.request("NAV STOP")
        self.tick(301, epoch=2)
        self.assertEqual(self.nav()["ready"],"0")
        self.assertFalse(self.start().ok)

    def test_between_intersection_endpoints_follow_lanes_and_match_demo(self):
        cases = [((0,7), (28,21)), ((7,0), (21,28)), ((14,7), (7,14)), ((28,7), (0,21))]
        for start, goal in cases:
            self.setUp()
            self.configure([start, goal, (0,0), (14,14), (28,28)],
                           grid={'x': [0,14,28], 'y': [0,14,28]})
            expected = plan_route(start, goal, self.grid)
            self.assertTrue(self.start().ok)
            reached = []
            while self.nav()['state'] == 'turning':
                status = self.nav()
                reached.append((int(status['wx']), int(status['wy'])))
                bearing = int(status['target_cd'])
                self.align(bearing)
                self.finish_forward(bearing)
            self.assertEqual(reached, expected)
            self.assertEqual(self.nav()['state'], 'orienting')
            self.assertEqual((int(self.nav()['x']), int(self.nav()['y'])), goal)

    def test_off_grid_future_goal_rejects_before_any_wheel_output(self):
        self.configure()
        self.request('NAV POINT 4 7 7')
        self.assertFalse(self.request("NAV START").ok)
        self.assertEqual(self.nav()['route_ready'], '0')
        self.assertFalse(self.arm.call('motion_chassis_active'))
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [0]*4)

    def test_grid_validation_and_maximum_report_remain_complete(self):
        self.configure()
        for command in ('NAV GRID X 0 0 28', 'NAV GRID X 28 14 0', 'NAV GRID Z 0 14 28',
                        'NAV GRID Y -1000001 0 1000000'):
            self.assertFalse(self.request(command).ok)
        self.request('ARM 0'); self.request('NAV RESET -1000000 -1000000')
        for axis in ('X', 'Y'): self.request(f'NAV GRID {axis} -1000000 0 1000000')
        for i in range(5):
            self.request(f'NAV POINT {i} {-1000000 if i == 0 else 1000000} -1000000')
        self.grid = {'x': [-1000000,0,1000000], 'y': [-1000000,0,1000000]}
        self.points = [(-1000000,-1000000)] + [(1000000,-1000000)]*4
        self.request('ARM 1')
        self.assertTrue(self.start(sequence=2147483647).ok)
        status = self.nav()
        self.assertIn('segments', status)
        self.assertEqual(status['fault'], 'none')


class GridPlannerTests(unittest.TestCase):
    def test_all_lane_points_only_use_legal_edges_and_shortest_distance(self):
        # Independent one-pulse BFS oracle on a small unequal-spacing network.
        from collections import deque
        grid = {'x': [-4,0,7], 'y': [-3,1,5]}
        legal = {(x,y) for x in range(-4,8) for y in range(-3,6) if on_grid((x,y),grid)}
        for start in legal:
            distance, todo = {start: 0}, deque([start])
            while todo:
                x,y = todo.popleft()
                for point in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                    if point in legal and point not in distance:
                        distance[point] = distance[(x,y)] + 1
                        todo.append(point)
            for goal in legal:
                last, length = start, 0
                for point in plan_route(start, goal, grid):
                    self.assertTrue((point[0] == last[0] and point[0] in grid['x']) or
                                    (point[1] == last[1] and point[1] in grid['y']))
                    length += abs(point[0]-last[0])+abs(point[1]-last[1])
                    last = point
                self.assertEqual(last, goal)
                self.assertEqual(length, distance[goal])

    def test_off_lane_outside_and_legacy_parameters(self):
        self.assertEqual([round_pulse(v) for v in (-1.5,-.5,.5,1.5)], [-2,-1,1,2])
        grid = DEFAULT['grid']
        for point in ((1,1), (-1,0), (0,28001)):
            with self.assertRaises(ValueError): plan_route(point,(0,0),grid)
        legacy = copy.deepcopy(DEFAULT)
        del legacy['grid']
        self.assertEqual(validated(legacy),DEFAULT)
        for bad in ([0,0,28000],[28000,14000,0],[0,14000],[False,14000,28000]):
            value = copy.deepcopy(DEFAULT); value['grid']['x'] = bad
            with self.assertRaises(ValueError): validated(value)
        value = copy.deepcopy(DEFAULT); value['stations'][2]['x'] = 1; value['stations'][2]['y'] = 1
        with self.assertRaises(ValueError): validated(value)


class HostNavigationTests(unittest.TestCase):
    def test_saved_parameters_roundtrip_and_invalid_write_is_atomic(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "navigation.json"
            config = MissionConfig(path)
            self.assertEqual(config.snapshot(), DEFAULT)
            value = config.snapshot()
            value["grid"] = {'x': [-15, 14000, 32000], 'y': [0, 14000, 28000]}
            value["stations"][0].update(x=-15, y=20)
            value["stations"][1].update(x=-15, y=14000)
            value["stations"][4].update(x=32000, y=17000)
            config.save(value)
            self.assertEqual(MissionConfig(path).snapshot(), value)
            original = path.read_bytes()
            value["stations"][1]["x"] = 1.5
            with self.assertRaises(ValueError):
                config.save(value)
            self.assertEqual(path.read_bytes(), original)

    def test_default_route_demo_stops_three_seconds_at_every_target(self):
        now = [0.]
        board = SimulatedBoard(clock=lambda: now[0])
        sequence = 0
        def send(body):
            nonlocal sequence
            sequence += 1
            return decode(board.receive(encode(sequence, body)))
        send("NAV RESET 0 0"); send("PING"); send("ARM 1")
        for body in MissionConfig().commands(host_control=False):
            self.assertTrue(send(body).ok, body)
        reached = []
        wait_start = None
        for _ in range(20000):
            now[0] += .02
            send("PING")
            nav = send("NAV STAT").fields
            if nav["state"] == "waiting" and wait_start is None:
                wait_start = now[0]
                # The board reports a station index; names stay in host config.
                reached.append((MissionConfig().station_name(int(nav["station"])),
                                int(nav["x"]), int(nav["y"])))
                self.assertFalse(any(axis["active"] for axis in board.axes[:4]))
            if nav["state"] != "waiting" and wait_start is not None:
                self.assertGreaterEqual(now[0] - wait_start, 3.)
                wait_start = None
            if nav["state"] == "done":
                break
        else:
            self.fail("demo never finished the default mission")
        self.assertEqual(reached, [("A", 0, 14000), ("B", 14000, 0), ("C", 14000, 28000),
                                   ("D", 28000, 14000), ("A", 0, 14000)])

    def test_upload_requires_each_ack_and_stop_cancels_remaining_commands(self):
        class Worker:
            def __init__(self): self.sent = []
            def submit(self, body, tag=None): self.sent.append((body, tag)); return True
        with tempfile.TemporaryDirectory() as directory:
            console = Console(Path(directory) / "nav.json")
            console.connection = Worker()
            console.device = {"nav": "1", "nav_heading": "1", "nav_grid": "1", "nav_path": "1", "_port": "SIM"}
            console.armed = True
            console.navigation = {"ready": "1", "state": "idle"}
            self.assertTrue(console.navigation_action("start")[0])
            worker = console.connection
            self.assertEqual(len(worker.sent), 1)
            command, tag = worker.sent[-1]
            console._reply(command, decode(encode(1, "OK cmd=NAV_STAT ready=1 state=idle x=0 y=0 stations=5 held=0")), tag)
            self.assertEqual(len(worker.sent), 2)
            command, tag = worker.sent[-1]
            console.submit("STOP")
            console._reply(command, decode(encode(2, "OK cmd=NAV_POINT")), tag)
            self.assertEqual(worker.sent[-1][0], "STOP")
            self.assertFalse(console.nav_upload)

    def test_demo_link_expiry_stops_mission_during_wait(self):
        now = [0.]
        board = SimulatedBoard(clock=lambda: now[0])
        def send(sequence, body): return decode(board.receive(encode(sequence, body)))
        send(1, "NAV RESET 0 0")
        send(2, "NAV STATIONS 5")
        for index in range(5): send(index + 3, f"NAV POINT {index} 0 0")
        for index in range(5): send(index + 8, f"NAV ATTITUDE {index} 0")
        send(13, "NAV LEGS 5")
        for index, station in enumerate((1, 2, 3, 4, 1)):
            send(14 + index, f"NAV LEG {index} {station} 0")
        send(10, "PING"); send(11, "ARM 1")
        config = copy.deepcopy(DEFAULT)
        config['stations'] = [{"name": name, "x": 0, "y": 0, "heading": 0}
                              for name in "OABCD"]
        entries = mission_waypoints(config, (0,0))
        send(100, "NAV BEGIN 1 5 0 0")
        for i, entry in enumerate(entries): send(101+i, "NAV WP 1 {} {} {} {}".format(i,*entry))
        send(106, "NAV COMMIT 1 {}".format(path_checksum(entries)))
        send(12, "NAV START")
        self.assertEqual(board.navigation.state, "orienting")
        for i in range(1, 6):
            now[0] = i * .1
            send(20+i, "PING")
        self.assertEqual(board.navigation.state, "waiting")
        now[0] = 1.6
        send(13, "NAV STAT")
        self.assertEqual(board.navigation.state, "stopped")
        self.assertFalse(board.armed)


class HeadingBridgeTests(unittest.TestCase):
    def test_verified_yaw_sign_age_and_invalid_sensor(self):
        hub = ImuHub()
        hub.state, hub.identity, hub.generation = "streaming", {"uid":"fixture"}, 4
        hub.latest = {"valid":True,"yaw_deg":-90.,"timestamp_ms":12,"sample_count":34}
        with patch("host.imu.time.monotonic", return_value=10.):
            hub.last_new_sample = 9.9
            self.assertEqual(hub.navigation_feedback()["heading_cd"],9000)
            self.assertTrue(hub.navigation_feedback()["valid"])
            hub.last_new_sample = 9.8
            self.assertFalse(hub.navigation_feedback()["valid"])
            hub.last_new_sample = 9.9
            hub.latest["valid"] = False
            self.assertFalse(hub.navigation_feedback()["valid"])
            hub.latest["valid"] = True
            hub.identity = {}
            self.assertFalse(hub.navigation_feedback()["valid"])

    def test_worker_newest_only_duplicate_expiry_and_reconnect(self):
        sample = {"key":(1,100,10),"heading_cd":9000,"valid":True,"epoch":1}
        worker = Connection("FAKE_NO_OPEN", queue.Queue(), imu_provider=lambda:dict(sample))
        worker.nav_heading = True
        worker.bridge_epoch = 1
        sent = []
        worker.exchange = lambda body: sent.append(body)
        with patch("host.transport.time.monotonic", return_value=1.):
            worker.forward_imu()
        self.assertEqual(sent[-1],"NAV IMU 9000 1 2")
        with patch("host.transport.time.monotonic", return_value=2.):
            worker.forward_imu()
        self.assertEqual(len(sent),1, "duplicate samples must not refresh MCU lease")
        sample.update(valid=False)
        with patch("host.transport.time.monotonic", return_value=3.):
            worker.forward_imu()
        self.assertEqual(sent[-1],"NAV IMU 9000 0 2")
        sample.update(valid=True,key=(2,1,1),epoch=2,heading_cd=-9000)
        with patch("host.transport.time.monotonic", return_value=4.):
            worker.forward_imu()
        self.assertEqual(sent[-1],"NAV IMU -9000 1 3")
        sample.update(key=(2,0,0))
        with patch("host.transport.time.monotonic", return_value=5.):
            worker.forward_imu()
        self.assertEqual(sent[-1],"NAV IMU -9000 1 4")

    def test_configuration_legacy_migration_decimal_angles_and_rejections(self):
        # The deployed pre-station file must still load and mean the same route.
        legacy = {"points": {"O":[0,0],"A":[0,14000],"B":[14000,0],
                             "C":[14000,28000],"D":[28000,14000]},
                  "headings": {"O":0,"A":0,"B":0,"C":0,"D":0},
                  "grid": {"x":[0,14000,28000],"y":[0,14000,28000]},
                  "sps":500,"accel":2000,"dwell_ms":3000}
        expected=copy.deepcopy(DEFAULT)
        for station in expected['stations']:station['arrival_enabled']=False
        self.assertEqual(validated(legacy),expected)
        del legacy["headings"]
        self.assertEqual([s["heading"] for s in validated(legacy)["stations"]], [0]*5)
        value=copy.deepcopy(DEFAULT); value["stations"][2]["heading"]=-90.25
        with tempfile.TemporaryDirectory() as directory:
            mission=MissionConfig(Path(directory)/"nav.json"); mission.save(value)
            self.assertIn("NAV ATTITUDE 2 -9025",mission.commands())
        for bad in (True,181,-181,float("nan"),90.123):
            value["stations"][2]["heading"]=bad
            with self.assertRaises(ValueError): validated(value)
        value=copy.deepcopy(DEFAULT); value["stations"][0]["heading"]=1
        with self.assertRaises(ValueError): validated(value)

    def test_real_start_rejects_missing_imu_and_legacy_firmware(self):
        with tempfile.TemporaryDirectory() as directory:
            console=Console(Path(directory)/"nav.json")
            console.connection=object()
            console.device={"nav":"1","nav_heading":"1","nav_grid":"1","nav_path":"1","_port":"FAKE_NO_OPEN"}
            console.armed=True; console.navigation={"ready":"1","state":"idle"}
            ok,message=console.navigation_action("start")
            self.assertFalse(ok); self.assertIn("H723",message)
            del console.device['nav_grid']
            ok,message=console.navigation_action('start')
            self.assertFalse(ok); self.assertIn('0.2.14',message)
            del console.device["nav_heading"]
            ok,message=console.navigation_action("start")
            self.assertFalse(ok); self.assertIn("0.2.9",message)


if __name__ == "__main__":
    unittest.main()
