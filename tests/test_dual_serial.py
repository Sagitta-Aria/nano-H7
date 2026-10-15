"""Actual ARM UART IRQs, line assembly, replay and mirrored responses.

MMIO readiness is supplied by the harness; electrical and radio timing still
require the real board. No physical serial port is opened by these tests.
"""
import unittest
from host.protocol import encode, decode
from tests.test_arm import ArmImage, SCRATCH

UARTS = (0x40011000, 0x40007800)
IRQS = ("USART1_IRQHandler", "UART7_IRQHandler")


class DualSerialTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmImage()
        self.arm.call("serial_init")

    def feed(self, port, data):
        """Inject received bytes through the real ISR, not directly into a ring."""
        for byte in data:
            self.arm.write32(UARTS[port] + 0x24, byte)
            self.arm.write32(UARTS[port] + 0x1C, (1 << 7) | (1 << 5))
            self.arm.call(IRQS[port])
        self.arm.write32(UARTS[port] + 0x1C, 1 << 7)

    def replies(self):
        self.assertEqual(self.arm.transmitted, self.arm.wireless_transmitted)
        return [decode(line) for line in bytes(self.arm.transmitted).splitlines(keepends=True)]

    def request(self, port, sequence, body):
        self.arm.transmitted.clear()
        self.arm.wireless_transmitted.clear()
        self.feed(port, encode(sequence, body))
        self.arm.call("app_comm_poll")
        return self.replies()[0]

    def test_pin_mux_baud_and_both_receivers_enabled(self):
        gpioa = 0x58020000
        for pin, af in [(8, 11), (15, 11), (9, 7), (10, 7)]:
            self.assertEqual((self.arm.read32(gpioa) >> (pin * 2)) & 3, 2)
            self.assertEqual((self.arm.read32(gpioa + 0x24) >> ((pin - 8) * 4)) & 15, af)
        for pin in (8, 10):
            self.assertEqual((self.arm.read32(gpioa + 0x0C) >> (pin * 2)) & 3, 1)
        for base in UARTS:
            self.assertEqual(self.arm.read32(base + 0x0C), 868)
            self.assertEqual(self.arm.read32(base) & 0x2D, 0x2D)
            self.assertEqual(self.arm.read32(base + 4), 0)  # 1 stop bit
            self.assertEqual(self.arm.read32(base + 8), 0)  # No hardware flow control

    def test_interleaved_partial_frames_remain_separate_and_broadcast(self):
        a, b = encode(10, "LED 1 0"), encode(20, "HELLO")
        self.feed(0, a[:8])
        self.feed(1, b[:6])
        self.arm.call("app_comm_poll")
        self.assertEqual(self.replies(), [])
        self.feed(1, b[6:])
        self.arm.call("app_comm_poll")
        self.feed(0, a[8:])
        self.arm.call("app_comm_poll")
        replies = self.replies()
        self.assertEqual([r.sequence for r in replies], [20, 10])
        self.assertEqual(replies[0].fields["serial"], "dual")
        self.assertEqual(self.arm.call("board_led_state"), 1)

    def test_bad_or_oversized_frame_does_not_damage_other_link(self):
        self.feed(0, b"x" * 400 + b"\n")
        self.feed(1, encode(1, "LED 0 1"))
        self.arm.call("app_comm_poll")
        self.assertEqual(self.replies()[0].fields["led"], "2")
        self.arm.call("app_comm_poll")
        self.assertTrue(self.request(0, 2, "HELLO").ok)
        self.feed(0, b"@3 HELLO*0000\n")
        self.assertTrue(self.request(1, 4, "PING").ok)
        self.assertEqual(self.arm.read32(self.arm.symbols["bad_frames"]), 2)

    def test_line_timeout_is_per_link(self):
        self.feed(0, encode(1, "HELLO")[:5])
        self.arm.call("app_comm_poll")
        self.arm.write32(self.arm.symbols["xTickCount"], 150)
        frame = encode(2, "HELLO")
        self.feed(1, frame[:5])
        self.arm.call("app_comm_poll")
        self.arm.write32(self.arm.symbols["xTickCount"], 201)
        self.feed(1, frame[5:])
        self.arm.call("app_comm_poll")
        self.assertEqual([r.sequence for r in self.replies()], [2])
        self.assertEqual(self.arm.read32(self.arm.symbols["bad_frames"]), 1)
        self.assertTrue(self.request(0, 3, "HELLO").ok)

    def test_same_motion_frame_on_second_link_does_not_restart(self):
        self.arm.call("motion_init")
        self.arm.call("motion_lift_init")
        self.arm.call("motion_lift_direction", 1)
        self.request(0, 1, "PING")
        self.request(0, 2, "ARM 1")
        self.assertTrue(self.request(1, 3, "MOVE 5 10 100 2000").ok)
        self.arm.edge(4)
        self.arm.edge(4)
        self.assertTrue(self.request(0, 3, "MOVE 5 10 100 2000").ok)
        self.assertEqual(self.arm.status(4)[2], 9)
        self.assertEqual(self.request(0, 3, "MOVE 5 20 100 2000").fields["code"], "SEQUENCE_REUSE")
        self.assertTrue(self.request(1, 4, "STOP").ok)
        self.assertEqual(self.arm.call("motion_armed"), 0)

    def test_stalled_transmitter_does_not_block_other_copy(self):
        payload = b"dual-response-test\n"
        for failed in (0, 1):
            with self.subTest(failed=failed):
                self.arm = ArmImage()
                self.arm.cycle_step = 400000
                self.arm.write32(UARTS[failed] + 0x1C, 0)
                self.arm.uc.mem_write(SCRATCH, payload + b"\0")
                self.assertEqual(self.arm.call("serial_write", SCRATCH), 0)
                outputs = [bytes(self.arm.transmitted), bytes(self.arm.wireless_transmitted)]
                self.assertEqual(outputs[failed], b"")
                self.assertEqual(outputs[1 - failed], payload)
                self.assertEqual(self.arm.call("serial_tx_errors"), 1)

    def test_rx_ring_overflow_and_wrap_are_isolated(self):
        self.feed(0, b"a" * 1024)
        self.feed(1, b"B")
        self.assertEqual(self.arm.call("serial_rx_errors"), 1)
        self.assertEqual(self.arm.call("serial_read", 1), ord("B"))
        for _ in range(1023):
            self.assertEqual(self.arm.call("serial_read", 0), ord("a"))
        self.assertEqual(self.arm.call("serial_read", 0), 0xFFFFFFFF)
        self.feed(0, b"C")
        self.assertEqual(self.arm.call("serial_read", 0), ord("C"))
        self.assertEqual(self.arm.call("serial_rx_bytes"), 1026)


if __name__ == "__main__":
    unittest.main()
