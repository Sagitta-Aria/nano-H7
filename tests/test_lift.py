"""M5 soft-limit regression on compiled ARM code and the demo protocol.

No hardware is moved. ARM tests count scheduled STEP edges; they do not prove
physical direction, homing accuracy or absence of missed motor steps.
"""
import unittest
from host.protocol import encode, decode
from host.simulator import SimulatedBoard
from tests.test_arm import ArmImage, SCRATCH, TIM5


class ArmLiftTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmImage()
        self.arm.call("motion_init")
        self.sequence = 100

    def request(self, body, sequence=None):
        """Exercise framing and the actual compiled command dispatch."""
        self.sequence += 1
        self.arm.transmitted.clear()
        self.arm.uc.mem_write(SCRATCH + 4096, encode(sequence or self.sequence, body).rstrip(b"\n") + b"\0")
        self.arm.call("process_line", SCRATCH + 4096)
        return decode(bytes(self.arm.transmitted))

    def setup_direction(self, sign=1):
        self.assertTrue(self.request("LIFT INIT").ok)
        self.assertTrue(self.request(f"LIFT DIR {sign}").ok)
        self.assertTrue(self.request("PING").ok)
        self.assertTrue(self.request("ARM 1").ok)

    def finish(self, steps):
        """Advance every high and low compare event, then require exact stopping."""
        for _ in range(2 * steps):
            self.arm.edge(4)
        self.assertEqual(self.arm.status(4)[4], 0)
        self.assertEqual(self.arm.read32(TIM5 + 0x0C) & 2, 0)

    def test_unreferenced_and_unknown_direction_cannot_move(self):
        self.request("PING"); self.request("ARM 1")
        self.assertEqual(self.request("MOVE 5 1 100 1000").fields["code"], "M5_NOT_REFERENCED")
        self.assertEqual(self.arm.call("motion_move", 4, 1, 100, 1000, 1), 0)
        self.assertTrue(self.request("LIFT INIT").ok)
        self.assertEqual(self.request("LIFT MOVE DOWN 1 100 1000").fields["code"], "M5_DIRECTION_UNKNOWN")
        self.assertEqual(self.arm.status(4)[4], 0)

    def test_cumulative_limit_and_top_for_both_raw_direction_signs(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                self.setUp(); self.setup_direction(sign)
                self.assertEqual(self.request("LIFT MOVE UP 1 100 1000").fields["code"], "M5_SOFT_LIMIT")
                self.assertTrue(self.request("LIFT MOVE DOWN 5000 3000 2000").ok)
                self.finish(5000)
                self.assertEqual(self.request("LIFT MOVE DOWN 3001 3000 2000").fields["code"], "M5_SOFT_LIMIT")
                self.assertTrue(self.request("LIFT MOVE DOWN 3000 3000 2000").ok)
                self.finish(3000)
                status = self.request("LIFT STAT").fields
                self.assertEqual((status["pos"], status["up_room"], status["down_room"]), ("8000", "8000", "0"))
                self.assertEqual(self.arm.status(4)[0], sign * 8000)
                self.assertEqual(self.arm.call("motion_move", 4, sign, 100, 1000, 99), 0)
                self.assertEqual(self.request(f"MOVE 5 {sign} 100 1000").fields["code"], "M5_SOFT_LIMIT")
                self.assertTrue(self.request("LIFT MOVE UP 8000 3000 2000").ok)
                self.finish(8000)
                self.assertEqual(self.request("LIFT STAT").fields["pos"], "0")
                self.assertEqual(self.request("LIFT MOVE UP 1 100 1000").fields["code"], "M5_SOFT_LIMIT")

    def test_partial_stop_reconnect_and_hello_keep_original_reference(self):
        self.setup_direction()
        self.assertTrue(self.request("LIFT MOVE DOWN 500 1000 1000").ok)
        for _ in range(21):
            self.arm.edge(4)
        self.request("STOP")
        self.arm.edge(4)  # final low edge, no extra rising pulse
        self.assertEqual(self.arm.status(4)[0], 11)
        for body in ("HELLO", "LIFT INIT", "LIFT INIT", "LIFT STAT"):
            self.assertTrue(self.request(body).ok)
        status = self.request("LIFT STAT").fields
        self.assertEqual((status["pos"], status["down_room"]), ("11", "7989"))
        self.assertEqual(self.request("LIFT DIR -1").fields["code"], "M5_DIR_REQUIRES_TOP_DISARMED")
        self.request("PING"); self.request("ARM 1")
        self.assertEqual(self.request("LIFT MOVE UP 12 100 1000").fields["code"], "M5_SOFT_LIMIT")
        self.assertTrue(self.request("LIFT MOVE UP 11 100 1000").ok)
        self.finish(11)

    def test_replayed_move_does_not_restart_or_extend_limit(self):
        self.setup_direction()
        cmd = "LIFT MOVE DOWN 20 100 1000"
        self.assertTrue(self.request(cmd, 1000).ok)
        self.arm.edge(4); self.arm.edge(4)
        self.assertTrue(self.request(cmd, 1000).ok)
        self.assertEqual(self.arm.status(4)[2], 19)
        self.finish(19)
        self.assertEqual(self.arm.status(4)[0], 20)

    def test_direction_change_is_disarmed_top_only(self):
        self.setup_direction()
        self.assertFalse(self.request("LIFT DIR -1").ok)
        self.request("STOP")
        self.assertTrue(self.request("LIFT DIR -1").ok)
        self.assertEqual(self.request("LIFT STAT").fields["down_sign"], "-1")
        self.assertFalse(self.request("LIFT DIR 0").ok)

    def test_m5_reverses_pin_for_down_and_up_without_reversing_depth(self):
        """Inspect actual compiled DIR writes and STEP counts for both mappings."""
        gpio_c_bsrr = 0x58020818
        for sign in (1, -1):
            with self.subTest(sign=sign):
                self.setUp(); self.setup_direction(sign)
                self.assertTrue(self.request("LIFT MOVE DOWN 2 100 1000").ok)
                self.assertEqual(self.arm.read32(gpio_c_bsrr), 1 << (18 if sign > 0 else 2))
                self.finish(2)
                self.assertEqual(self.request("LIFT STAT").fields["pos"], "2")
                self.assertTrue(self.request("LIFT MOVE UP 2 100 1000").ok)
                self.assertEqual(self.arm.read32(gpio_c_bsrr), 1 << (2 if sign > 0 else 18))
                self.finish(2)
                self.assertEqual(self.request("LIFT STAT").fields["pos"], "0")

    def test_other_axis_dir_polarities_are_unchanged(self):
        """M5 inversion must not change the raw DIR mapping of the six other axes."""
        pins = ((0, 0x58022018, 8), (1, 0x58022018, 3),
                (2, 0x58022018, 4), (3, 0x58020818, 13),
                (5, 0x58021818, 11), (6, 0x58020418, 9))
        self.assertEqual(self.arm.call("motion_arm", 1), 1)
        for axis, register, pin in pins:
            for sign in (1, -1):
                with self.subTest(axis=axis, sign=sign):
                    self.assertEqual(self.arm.call("motion_move", axis, sign, 100, 1000, 1), 1)
                    self.assertEqual(self.arm.read32(register), 1 << (pin if sign > 0 else pin + 16))
                    self.arm.edge(axis); self.arm.edge(axis)

    def test_handshake_advertises_8000_limit(self):
        self.assertEqual(self.request("HELLO").fields["lift_max"], "8000")


class DemoLiftTests(unittest.TestCase):
    def test_demo_matches_limits_and_reconnect_semantics(self):
        now = [100.]
        board = SimulatedBoard(clock=lambda: now[0])
        seq = 0
        def request(body):
            nonlocal seq
            seq += 1
            return decode(board.receive(encode(seq, body)))
        self.assertEqual(request("MOVE 5 1 100 1000").fields["code"], "M5_NOT_REFERENCED")
        request("LIFT INIT")
        self.assertEqual(request("MOVE 5 1 100 1000").fields["code"], "M5_DIRECTION_UNKNOWN")
        request("LIFT DIR -1"); request("PING"); request("ARM 1")
        self.assertFalse(request("LIFT MOVE UP 1 100 1000").ok)
        self.assertTrue(request("LIFT MOVE DOWN 20 3000 2000").ok)
        for _ in range(100):
            now[0] += .01; request("PING")
        request("STOP"); request("LIFT INIT")
        self.assertEqual(request("LIFT STAT").fields["pos"], "20")
        request("PING"); request("ARM 1")
        self.assertEqual(request("LIFT MOVE DOWN 7981 100 1000").fields["code"], "M5_SOFT_LIMIT")
        self.assertEqual(request("MOVE 5 21 100 1000").fields["code"], "M5_SOFT_LIMIT")


if __name__ == "__main__":
    unittest.main()
