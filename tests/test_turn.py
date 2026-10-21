"""Standalone yaw task tests against the actual ARM image and host transports.
No physical gyro, wheel, pulse timing or angular accuracy is simulated as proof.
"""
import queue
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import tkinter as tk
from tests import test_chassis as chassis_harness
from host.protocol import encode, decode, Reply
from host.simulator import SimulatedBoard
from host.transport import Connection
from host.turn import turn_command
from host.turn_panel import TurnPanel
from host.webapp import Console


class ArmTurnTests(unittest.TestCase):
    setUp = chassis_harness.ArmChassisTests.setUp
    request = chassis_harness.ArmChassisTests.request
    group_edge = chassis_harness.ArmChassisTests.group_edge

    def feed(self, now, heading, valid=1, epoch=1):
        self.arm.write32(self.arm.symbols["xTickCount"], now)
        self.arm.call("navigation_imu", heading, valid, epoch, now)
        self.arm.call("turn_tick", now)
        return self.request("TURN STAT").fields

    def settle(self, heading, start=100):
        result = None
        for now in (start, start+100, start+200, start+300):
            result = self.feed(now, heading)
        return result

    def test_relative_crosses_wrap_and_completes_without_route(self):
        self.feed(0, 17000)
        self.assertTrue(self.request("TURN MOVE REL 9000 500 2000").ok)
        self.feed(1, 17000)
        self.assertTrue(self.arm.call("turn_busy"))
        self.assertEqual(self.request("NAV STAT").fields["ready"], "0")
        self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [-1]*4)
        self.group_edge()
        self.assertEqual(self.feed(100, -10000)["state"], "turning")
        self.assertEqual(self.settle(-10000, 200)["state"], "done")
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.assertEqual(self.arm.call("motion_armed"), 1)
        self.assertEqual([self.arm.status(i)[0] for i in range(4, 7)], [0]*3)

    def test_absolute_requires_zero_and_uses_shortest_path(self):
        self.feed(0, 17000)
        self.assertFalse(self.request("TURN MOVE ABS 9000 500 2000").ok)
        self.assertTrue(self.request("TURN ZERO").ok)
        self.feed(1, -2000)  # +170 degrees from zero
        self.assertTrue(self.request("TURN MOVE ABS -17000 500 2000").ok)
        status = self.feed(2, -2000)
        self.assertEqual(status["error_cd"], "2000")
        self.group_edge(); self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [-1]*4)
        status = self.settle(0)
        self.assertEqual(status["state"], "done")
        self.assertEqual(status["zero"], "1")

    def test_zero_angle_requires_distinct_stable_samples(self):
        self.feed(0, 0)
        self.assertTrue(self.request("TURN MOVE REL 0 500 2000").ok)
        for now in range(0, 250, 10):
            self.arm.call("turn_tick", now)
        self.assertEqual(self.request("TURN STAT").fields["state"], "turning")
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.assertEqual(self.settle(0)["state"], "done")

    def test_overshoot_finishes_high_before_reversing(self):
        self.feed(0, 0)
        self.request("TURN MOVE REL 1000 500 2000")
        self.feed(1, 0); self.group_edge()
        positions = [self.arm.status(i)[0] for i in range(4)]
        self.feed(2, 2000)
        self.assertEqual(self.arm.call("motion_chassis_active"), 15)
        self.feed(3, 2000)
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], positions)
        self.group_edge()
        self.feed(4, 2000)
        self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [0]*4)

    def test_same_epoch_loss_pauses_but_changed_epoch_never_resumes(self):
        for mode, reason in (("stale", "imu_stale"), ("invalid", "imu_invalid"), ("epoch", "imu_changed")):
            self.setUp(); self.feed(0, 0)
            self.request("TURN ZERO")
            self.request("TURN MOVE REL 9000 500 2000")
            self.feed(1, 0)
            if mode == "stale":
                self.arm.call("turn_tick", 301)
            else:
                self.feed(2, 0, valid=0 if mode == "invalid" else 1, epoch=2 if mode == "epoch" else 1)
            status = self.request("TURN STAT").fields
            if mode != 'epoch':
                self.assertEqual((status['state'], status['fault'], status['paused']), ('turning', 'none', '1'))
                self.feed(302, 0)
                self.assertEqual(self.request('TURN STAT').fields['paused'], '1')
                self.feed(362, 0); self.feed(422, 0)
                self.assertEqual(self.request('TURN STAT').fields['paused'], '0')
                self.assertTrue(self.arm.call('turn_busy'))
                continue
            self.assertEqual((status["state"], status["fault"]), ("fault", reason))
            self.feed(302, 0, epoch=2 if mode == "epoch" else 1)
            self.assertEqual(self.arm.call("turn_busy"), 0)
            self.assertEqual(self.arm.call("motion_chassis_active"), 0)
            if mode == "epoch":
                self.assertEqual(self.request("TURN STAT").fields["zero"], "0")
                self.assertFalse(self.request("TURN MOVE ABS 0 500 2000").ok)

    def test_active_task_blocks_wheels_and_stop_preserves_auxiliary(self):
        self.feed(0, 0)
        self.request("TURN MOVE REL 9000 500 2000")
        for command in ("CHASSIS MOVE FWD 1 100 2000", "CHASSIS JOG FWD 100 2000",
                        "MOVE 1 1 100 2000", "WHEELS 0 0 0 0 2000", "TURN ZERO", "TURN MOVE REL 0 500 2000"):
            self.assertFalse(self.request(command).ok, command)
        self.request("MOVE 5 100 100 2000")
        self.feed(1, 0); self.group_edge()
        self.assertTrue(self.request("TURN STOP").ok)
        self.assertEqual(self.request("TURN STAT").fields["state"], "stopped")
        self.group_edge()
        self.feed(2, 0)
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.assertEqual(self.arm.status(4)[4], 1)
        self.assertEqual(self.arm.call("motion_armed"), 1)

    def test_global_halt_disarm_and_nav_stop_cancel_turn(self):
        for command in ("CHASSIS HALT", "STOP", "ARM 0", "NAV STOP"):
            self.setUp(); self.feed(0, 0)
            self.request("TURN MOVE REL 9000 500 2000")
            self.feed(1, 0)
            self.request(command); self.feed(2, 0)
            self.assertEqual(self.arm.call("turn_busy"), 0, command)
            self.assertEqual(self.arm.call("motion_chassis_active"), 0, command)

    def test_replay_does_not_recapture_relative_origin(self):
        self.feed(0, 0)
        command = "TURN MOVE REL 9000 500 2000"
        self.request(command, sequence=1000)
        self.feed(1, 1000)
        self.assertTrue(self.request(command, sequence=1000).ok)
        self.assertEqual(self.request("TURN STAT").fields["error_cd"], "8000")

    def test_invalid_arguments_disarmed_and_stale_zero_rejected(self):
        self.feed(0, 0)
        for body in ("TURN MOVE REL 18001 500 2000", "TURN MOVE BAD 0 500 2000", "TURN MOVE REL 0 0 2000", "TURN MOVE REL 0 500 0"):
            self.assertFalse(self.request(body).ok)
        self.request("ARM 0")
        self.assertFalse(self.request("TURN MOVE REL 0 500 2000").ok)
        self.arm.write32(self.arm.symbols["xTickCount"], 300)
        self.assertFalse(self.request("TURN ZERO").ok)

    def test_zero_preserves_navigation_coordinates_and_turn_cannot_overlap_route(self):
        self.request("ARM 0")
        self.request("NAV RESET 123 456")
        self.request("NAV GRID X 123 133 143")
        self.request("NAV GRID Y 456 466 476")
        for index, point in enumerate(((123,456),(123,466),(133,456),(133,476),(143,466))):
            self.request(f"NAV POINT {index} {point[0]} {point[1]}")
        self.feed(0, 0)
        self.assertTrue(self.request("TURN ZERO").ok)
        status = self.request("NAV STAT").fields
        self.assertEqual((status["ready"], status["x"], status["y"]), ("1","123","456"))
        self.request("ARM 1")
        self.request("TURN MOVE REL 9000 500 2000")
        self.assertFalse(self.request("NAV START").ok)
        self.feed(1, 0)
        self.assertEqual(self.settle(9000)["state"], "done")
        status = self.request("NAV STAT").fields
        self.assertEqual((status["ready"], status["x"], status["y"]), ("1","123","456"))
        from host.navigation import DEFAULT, mission_waypoints, path_checksum
        import copy
        config = copy.deepcopy(DEFAULT)
        coords = ((123,456),(123,466),(133,456),(133,476),(143,466))
        config.update(stations=[{"name": n, "x": c[0], "y": c[1], "heading": 0}
                                for n, c in zip('OABCD', coords)],
                      grid={'x':[123,133,143], 'y':[456,466,476]})
        entries = mission_waypoints(config, (123,456))
        self.assertTrue(self.request('NAV STATIONS 5').ok)
        for i, (x, y) in enumerate(coords):
            self.assertTrue(self.request('NAV POINT {} {} {}'.format(i, x, y)).ok)
            self.assertTrue(self.request('NAV ATTITUDE {} 0'.format(i)).ok)
        self.assertTrue(self.request('NAV LEGS 5').ok)
        for i, station in enumerate((1,2,3,4,1)):
            self.assertTrue(self.request('NAV LEG {} {} 0'.format(i, station)).ok)
        self.request('NAV BEGIN 1 {} 123 456'.format(len(entries)))
        for i, entry in enumerate(entries): self.request('NAV WP 1 {} {} {} {}'.format(i,*entry))
        self.assertTrue(self.request('NAV COMMIT 1 {}'.format(path_checksum(entries))).ok)
        self.assertTrue(self.request("NAV START").ok)
        self.assertFalse(self.request("TURN MOVE REL 9000 500 2000").ok)

    def test_half_turn_tie_and_stuck_turn_timeout(self):
        for angle in (-18000, 18000):
            self.setUp(); self.feed(0, 0)
            self.request(f"TURN MOVE REL {angle} 500 2000")
            self.feed(1, 0); self.group_edge(); self.group_edge()
            self.assertEqual([self.arm.status(i)[0] for i in range(4)], [-1]*4)
            for now in range(100, 60001, 100):
                status = self.feed(now, 0)
            self.assertEqual((status["state"],status["fault"]), ("fault","turn_timeout"))


class TurnHostTests(unittest.TestCase):
    def test_command_validation_avoids_angle_rounding(self):
        self.assertEqual(turn_command("REL", "-90.25", 500, 2000), "TURN MOVE REL -9025 500 2000")
        for angle in ("", "NaN", "Infinity", "90.001", "181", True, None):
            with self.assertRaises(ValueError): turn_command("REL", angle, 500, 2000)
        for speed in (0, 5001, 2.5, True):
            with self.assertRaises(ValueError): turn_command("ABS", 90, speed, 2000)

    def test_demo_relative_and_absolute_complete_without_route(self):
        now = [10.]
        board = SimulatedBoard(clock=lambda: now[0])
        sequence = [0]
        def send(body):
            sequence[0] += 1
            return decode(board.receive(encode(sequence[0], body)))
        now[0] += .01; send("PING"); send("ARM 1")
        self.assertTrue(send("TURN ZERO").ok)
        for body in ("TURN MOVE REL 9000 500 2000", "TURN MOVE ABS -9000 500 2000"):
            self.assertTrue(send(body).ok)
            for _ in range(1000):
                now[0] += .02; send("PING")
                if send("TURN STAT").fields["state"] == "done": break
            self.assertEqual(send("TURN STAT").fields["state"], "done")
            self.assertFalse(board.navigation.ready)
        self.assertLessEqual(abs((board.navigation.absolute_heading-37+90+180)%360-180), .2)

    def test_console_guards_and_reports_status(self):
        with tempfile.TemporaryDirectory() as directory:
            console = Console(navigation_path=Path(directory)/"nav.json")
            console.connection = Mock(); console.connection.submit.return_value = True
            console.device = {"turn":"1", "_port":"SIM"}
            self.assertFalse(console.turn_action("start")[0])
            console.armed = True
            self.assertTrue(console.turn_action("start", angle="-90.25")[0])
            console.connection.submit.assert_called_with("TURN MOVE REL -9025 500 2000")
            self.assertFalse(console.turn_action("start", mode="ABS", angle=90)[0])
            console._reply("TURN ZERO", Reply(1,"OK cmd=TURN_STAT state=idle zero=1 imu=1"))
            self.assertEqual(console.hello()["turn"]["zero"], "1")
            self.assertTrue(console.turn_action("start", mode="ABS", angle=90)[0])
            console.turn["state"] = "turning"
            self.assertFalse(console.turn_action("zero")[0])
            self.assertTrue(console.turn_action("stop")[0])
            console.nav_pending = True
            self.assertTrue(console.turn_action("stop")[0])
            console.nav_pending = False
            console.turn["state"] = "done"
            console.device["_port"] = "FAKE_NO_OPEN"
            console.imu.navigation_feedback = Mock(return_value={"valid":False})
            self.assertFalse(console.turn_action("start")[0])

    def test_stop_discards_pending_turn_preserves_auxiliary(self):
        worker = Connection("SIM", queue.Queue())
        worker.submit("TURN MOVE REL 9000 500 2000")
        worker.submit("TURN ZERO")
        worker.submit("MOVE 5 100 100 2000")
        worker.submit("TURN STOP")
        self.assertEqual(worker.commands.get_nowait()[2], "TURN STOP")
        self.assertEqual(worker.commands.get_nowait()[2], "MOVE 5 100 100 2000")
        self.assertTrue(worker.commands.empty())


class TurnTkTests(unittest.TestCase):
    def test_modes_validation_and_telemetry_use_shared_commands(self):
        root = tk.Tk(); root.withdraw()
        try:
            app = SimpleNamespace(connected=True, send=Mock(return_value=True),
                connection=SimpleNamespace(port="SIM"), values={"armed":tk.StringVar(value="已使能")},
                navigation=SimpleNamespace(uploading=False, telemetry={}), chassis=SimpleNamespace(intent=None))
            panel = TurnPanel(app, root)
            panel.reply("HELLO", Reply(1,"OK cmd=HELLO turn=1"))
            panel.angle.set("-90.25"); panel.action("start")
            app.send.assert_called_with("TURN MOVE REL -9025 500 2000")
            panel.mode.set("转到零点角度")
            app.send.reset_mock(); panel.action("start"); app.send.assert_not_called()
            panel.reply("TURN ZERO", Reply(2,"OK cmd=TURN_STAT state=idle zero=1 imu=1"))
            panel.action("start"); app.send.assert_called_with("TURN MOVE ABS -9025 500 2000")
            panel.angle.set("90.001"); app.send.reset_mock(); panel.action("start"); app.send.assert_not_called()
            panel.action("stop"); app.send.assert_called_with("TURN STOP")
            panel.reply('HELLO', Reply(3,'OK cmd=HELLO turn=1 turn_tuning=1 turn_precision=1'))
            panel.reply('TURN STAT', Reply(4,'OK cmd=TURN_STAT state=idle min_sps=200'))
            panel.kp.set('80'); panel.tolerance.set('0.50'); panel.action('configure')
            app.send.assert_called_with('TURN CONFIG 8000 50 200')
            panel.reply('TURN CONFIG 8000 50 200', Reply(5,'OK cmd=TURN_STAT state=idle kp_centi=8000 tolerance_cd=50 min_sps=200'))
            self.assertIn('±0.50°',panel.status.get())
        finally:
            root.destroy()


if __name__ == "__main__": unittest.main()
