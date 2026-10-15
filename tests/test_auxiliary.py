"""Exercise actual AXF limits/targets and host persistence/stop queue behavior."""
import queue
import tempfile
import unittest
from pathlib import Path

from tests import test_chassis as chassis_harness
from host.auxiliary import AuxiliaryConfig, validated
from host.transport import Connection
from host.simulator import SimulatedBoard
from host.protocol import encode, decode
from host.webapp import Console


class ArmAuxiliaryTests(unittest.TestCase):
    setUp = chassis_harness.ArmChassisTests.setUp
    request = chassis_harness.ArmChassisTests.request

    def configure(self, low=-30, high=40):
        self.request("ARM 0")
        self.assertTrue(self.request("AUX ZERO 6").ok)
        self.assertTrue(self.request("AUX CONFIG 6 {} {}".format(low, high)).ok)
        self.request("ARM 1")

    def finish(self, axis):
        for _ in range(30000):
            if not self.arm.status(axis-1)[4]:
                return
            self.arm.edge(axis-1)
        self.fail("axis failed to complete")

    def test_rotation_needs_explicit_reference_and_bounds(self):
        self.assertEqual(self.request("HELLO").fields["aux_control"], "1")
        self.assertFalse(self.request("AUX STEP 6 1 300 1000").ok)
        self.assertFalse(self.request("AUX ZERO 6").ok)
        self.request("ARM 0")
        self.assertFalse(self.request("AUX CONFIG 6 -10 10").ok)
        self.assertTrue(self.request("AUX ZERO 6").ok)
        self.request("ARM 1")
        self.assertFalse(self.request("AUX GOTO 6 0 300 1000").ok)
        self.request("ARM 0")
        self.assertTrue(self.request("AUX CONFIG 6 0 0").ok)
        self.request("ARM 1")
        self.assertTrue(self.request("AUX GOTO 6 0 300 1000").ok)
        self.assertFalse(self.request("AUX STEP 6 1 300 1000").ok)

    def test_rotation_absolute_presets_raw_bypass_and_replay(self):
        self.configure()
        for target in (40, -30, 10, 0):
            self.assertTrue(self.request("AUX GOTO 6 {} 300 1000".format(target)).ok)
            self.finish(6)
            self.assertEqual(self.request("AUX STAT 6").fields["pos"], str(target))
        self.assertTrue(self.request("AUX STEP 6 40 300 1000", sequence=1000).ok)
        self.arm.edge(5); self.arm.edge(5)
        self.assertTrue(self.request("AUX STEP 6 40 300 1000", sequence=1000).ok)
        self.assertEqual(self.arm.status(5)[2], 39)
        self.assertFalse(self.request("AUX GOTO 6 0 300 1000").ok)
        self.finish(6)
        self.assertFalse(self.request("AUX STEP 6 1 300 1000").ok)
        self.assertFalse(self.request("MOVE 6 1 300 1000").ok)
        self.assertEqual(self.arm.call("motion_move", 5, 1, 300, 1000, 1), 0)
        self.assertEqual(self.arm.status(5)[0], 40)

    def test_lift_range_can_extend_and_all_old_paths_obey_it(self):
        for sign in (1, -1):
            self.setUp(); self.request("ARM 0")
            self.request("LIFT DIR {}".format(sign))
            self.assertTrue(self.request("AUX CONFIG 5 0 9000").ok)
            self.request("ARM 1")
            self.assertTrue(self.request("AUX GOTO 5 9000 3000 2000").ok)
            self.finish(5)
            self.assertEqual(self.arm.status(4)[0], 9000*sign)
            f = self.request("LIFT STAT").fields
            self.assertEqual((f["pos"], f["max"], f["down_room"]), ("9000", "9000", "0"))
            self.assertFalse(self.request("LIFT MOVE DOWN 1 100 1000").ok)
            self.assertFalse(self.request("MOVE 5 {} 100 1000".format(sign)).ok)
            self.assertTrue(self.request("AUX GOTO 5 0 3000 2000").ok)
            self.finish(5)
            self.assertEqual(self.request("LIFT STAT").fields["pos"], "0")

    def test_config_allows_armed_idle_refuses_busy_and_excluded_current_position(self):
        self.configure()
        self.assertTrue(self.request("AUX CONFIG 6 -20 20").ok)
        self.request("AUX GOTO 6 10 300 1000")
        self.assertFalse(self.request("AUX CONFIG 6 -30 40").ok)
        self.finish(6)
        self.assertTrue(self.request("AUX CONFIG 6 -30 40").ok)
        self.request("AUX GOTO 6 30 300 1000"); self.finish(6)
        self.request("ARM 0")
        self.assertFalse(self.request("AUX CONFIG 6 -20 20").ok)
        f = self.request("AUX STAT 6").fields
        self.assertEqual((f["min"], f["max"], f["pos"]), ("-30", "40", "30"))
        self.assertFalse(self.request("AUX CONFIG 5 -1 8000").ok)
        self.assertFalse(self.request("AUX CONFIG 5 10 10").ok)
        self.assertFalse(self.request("AUX ZERO 5").ok)

    def test_nonzero_lift_minimum_applies_to_legacy_up_moves(self):
        self.request("AUX GOTO 5 20 300 1000"); self.finish(5)
        self.request("ARM 0")
        self.assertTrue(self.request("AUX CONFIG 5 10 30").ok)
        self.request("ARM 1")
        self.assertFalse(self.request("LIFT MOVE UP 11 100 1000").ok)
        self.assertFalse(self.request("AUX GOTO 5 0 300 1000").ok)
        self.assertTrue(self.request("LIFT MOVE UP 10 100 1000").ok)
        self.finish(5)
        f = self.request("LIFT STAT").fields
        self.assertEqual((f["pos"], f["up_room"], f["down_room"], f["min"]), ("10", "0", "20", "10"))

    def test_halt_finishes_high_pulse_and_preserves_other_axes_and_reference(self):
        self.configure()
        self.request("AUX STEP 5 20 300 1000")
        self.request("AUX STEP 6 20 300 1000")
        self.arm.edge(5)
        self.assertEqual(self.arm.status(5)[0], 1)
        self.request("AUX HALT 6")
        self.assertEqual(self.arm.status(5)[4], 1)
        self.arm.edge(5)
        self.assertEqual(self.arm.status(5)[4], 0)
        self.assertEqual(self.arm.status(5)[0], 1)
        self.assertEqual(self.arm.status(4)[4], 1)
        self.assertEqual(self.arm.call("motion_armed"), 1)
        self.request("STOP"); self.finish(5)
        for verb in ("HELLO", "LIFT INIT", "AUX STAT 6"):
            self.assertTrue(self.request(verb).ok)
        f = self.request("AUX STAT 6").fields
        self.assertEqual((f["pos"], f["min"], f["max"], f["referenced"], f["configured"]), ("1", "-30", "40", "1", "1"))


class AuxiliaryHostTests(unittest.TestCase):
    def test_save_roundtrip_and_validation_never_sends_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"aux.json"
            console = Console(auxiliary_path=path)
            p = console.hello()["aux_settings"]
            p["6"].update(min=-100, max=200, sps=500, step=20, presets=[-50, 150])
            console.save_auxiliary(p)
            self.assertIsNone(console.connection)
            self.assertEqual(AuxiliaryConfig(path).snapshot(), p)
            p["6"]["presets"][0] = -101
            with self.assertRaises(ValueError): console.save_auxiliary(p)
            self.assertEqual(AuxiliaryConfig(path).snapshot()["6"]["presets"], [-50, 150])
            for key, value in (("sps", True), ("step", 0), ("min", 0.5)):
                p = AuxiliaryConfig(path).snapshot(); p["6"][key] = value
                with self.assertRaises(ValueError): validated(p)

    def test_aux_halt_removes_only_target_axis_pending_moves(self):
        worker = Connection("SIM", queue.Queue())
        worker.set_jog("CHASSIS JOG FWD 500 2000")
        for body in ("MOVE 6 10 300 1000", "AUX GOTO 6 20 300 1000", "AUX STEP 5 10 300 1000", "MOVE 7 5 300 1000"):
            worker.submit(body)
        worker.submit("AUX HALT 6")
        bodies = [worker.commands.get_nowait()[2] for _ in range(worker.commands.qsize())]
        self.assertEqual(bodies, ["AUX HALT 6", "AUX STEP 5 10 300 1000", "MOVE 7 5 300 1000"])
        self.assertIsNotNone(worker.current_jog())

    def test_simulator_absolute_moves_and_dynamic_legacy_lift_bounds(self):
        now = [0.]
        board = SimulatedBoard(clock=lambda: now[0])
        sequence = 0
        def request(body):
            nonlocal sequence
            sequence += 1
            return decode(board.receive(encode(sequence, body)))
        for body in ("LIFT INIT", "LIFT DIR -1", "AUX CONFIG 5 0 20", "AUX ZERO 6", "AUX CONFIG 6 -20 20", "PING", "ARM 1"):
            self.assertTrue(request(body).ok, body)
        self.assertTrue(request("AUX GOTO 5 20 300 1000").ok)
        self.assertTrue(request("AUX GOTO 6 -20 300 1000").ok)
        for _ in range(100): now[0] += .01; request("PING")
        self.assertEqual(request("AUX STAT 5").fields["pos"], "20")
        self.assertEqual(request("AUX STAT 6").fields["pos"], "-20")
        self.assertFalse(request("LIFT MOVE DOWN 1 300 1000").ok)
        self.assertFalse(request("MOVE 6 -1 300 1000").ok)
        request("STOP"); request("LIFT INIT")
        self.assertEqual(request("AUX STAT 6").fields["pos"], "-20")


if __name__ == "__main__": unittest.main()
