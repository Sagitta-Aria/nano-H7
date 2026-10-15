import queue
import time
import unittest
from unittest.mock import patch
from host.protocol import encode, decode, Framer
from host.simulator import SimulatedBoard, SimulatedSerial
from host.transport import Connection


class ProtocolTests(unittest.TestCase):
    def test_crc_and_fragmentation(self):
        frame = encode(123, "LED 1 0")
        stream = Framer()
        packets = []
        for byte in frame + encode(124, "STAT"):
            packets += stream.feed(bytes([byte]))
        self.assertEqual([decode(p).body for p in packets], ["LED 1 0", "STAT"])
        damaged = frame.replace(b"LED 1", b"LED 0")
        with self.assertRaises(ValueError):
            decode(damaged)

    def test_resync_after_oversize(self):
        stream = Framer()
        self.assertEqual(stream.feed(b"x" * 500 + b"\n" + encode(2, "PING")), [encode(2, "PING")])

    def test_reject_framing_and_sequence(self):
        for seq in (0, -1, 2**32):
            with self.assertRaises(ValueError):
                encode(seq, "PING")
        for body in ("", "LED\n1", "PING*1234", "中文"):
            with self.assertRaises(ValueError):
                encode(1, body)


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.board = SimulatedBoard(clock=lambda: self.now)
        self.sequence = 0
        self.request("LIFT INIT")
        self.request("LIFT DIR 1")

    def request(self, body):
        self.sequence += 1
        return decode(self.board.receive(encode(self.sequence, body)))

    def test_led_and_system(self):
        self.assertEqual(self.request("LED 1 1").fields["led"], "3")
        self.assertEqual(self.request("STAT").fields["led"], "3")
        self.assertEqual(self.request("HELLO").fields["red"], "fixed")
        self.assertIn("uid", self.request("INFO").fields)
        self.assertFalse(self.request("LED 2 0").ok)

    def test_arm_limits_and_timeout(self):
        self.assertFalse(self.request("ARM 1").ok)
        self.request("PING")
        self.assertTrue(self.request("ARM 1").ok)
        self.assertFalse(self.request("MOVE 5 10 3001 1000").ok)
        self.assertFalse(self.request("WHEELS 5001 0 0 0 2000").ok)
        self.assertTrue(self.request("WHEELS 5000 -4000 3000 -2000 2000").ok)
        self.now += 1.1
        self.assertEqual(self.request("STAT").fields["armed"], "0")
        self.assertEqual(self.board.timeouts, 1)

    def test_duplicate_move_and_exact_steps(self):
        self.request("LIFT DIR -1")
        self.request("PING")
        self.request("ARM 1")
        frame = encode(99, "MOVE 5 -50 100 2000")
        first = self.board.receive(frame)
        self.assertEqual(self.board.receive(frame), first)
        self.assertEqual(self.board.replay_hits, 1)
        self.assertFalse(decode(self.board.receive(encode(99, "MOVE 5 500 100 2000"))).ok)
        for _ in range(100):
            self.now += .01
            self.board.update()
            self.request("PING")
        self.assertEqual(self.board.axes[4]["pos"], -50)
        self.assertFalse(self.board.axes[4]["active"])


class WorkerTests(unittest.TestCase):
    def test_worker_handshake_led_and_close(self):
        events = queue.Queue()
        worker = Connection("SIM", events)
        worker.start()
        deadline = time.monotonic() + 3
        connected = False
        while time.monotonic() < deadline:
            kind, value = events.get(timeout=1)
            if kind == "connected":
                connected = True
                break
            self.assertNotEqual(kind, "error", value)
        self.assertTrue(connected)
        worker.submit("LED 0 1")
        while True:
            kind, value = events.get(timeout=1)
            if kind == "reply" and value[0] == "LED 0 1":
                self.assertEqual(value[1].fields["led"], "2")
                break
        worker.close()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(worker.serial.board.armed)

    def test_retry_keeps_same_sequence(self):
        class DropFirstReply(SimulatedSerial):
            def __init__(self):
                super().__init__()
                self.sent = []

            def write(self, data):
                self.sent.append(data)
                count = super().write(data)
                if len(self.sent) == 1:
                    self.buffer.clear()
                return count

        worker = Connection("SIM", queue.Queue())
        worker.serial = DropFirstReply()
        reply = worker.exchange("LED 1 0")
        self.assertTrue(reply.ok)
        self.assertEqual(worker.serial.sent[0], worker.serial.sent[1])
        self.assertEqual(worker.serial.board.replay_hits, 1)


if __name__ == "__main__":
    unittest.main()
