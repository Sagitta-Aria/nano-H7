"""H723 identity, port exclusion and telemetry failure regression checks.

Synthetic USB transports verify the actual worker/handshake without opening
the user's COM ports. Browser plotting is exercised by imu_browser.cjs.
"""
import collections
import queue
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from host.imu import ImuHub, LineStream, candidate_ports, parse_identity
from host.transport import Connection, list_ports
from host.webapp import Console

CANDIDATE = {"id": "COM_TEST", "description": "ST USB CDC", "serial_number": "ABC123", "verified": False}
UID = "1234567890ABCDEF12345678"


def frame(count=1, calibrated=True):
    """Independent wire fixture whose axes are easy to distinguish in plots."""
    return ("IMU1,%d,%d,1,%d,1,0,2000,10,20,30,1,2,3,4,5,6,7,8,9,25,1,0,0,0\r\n" %
            (count * 10, count, calibrated)).encode()


def identity(nonce, mcu="STM32H723VGT6"):
    """Build an identity fixture, independent of firmware formatting helpers."""
    return (f"IMU! {nonce} board=DM-MC02 mcu={mcu} sensor=BMI088 protocol=IMU1 fw=1.1.0 uid={UID}\r\n").encode()


class UsbFixture:
    """Model an IMU, unrelated CDC device, old firmware, or a disconnected stream."""
    def __init__(self, mcu="STM32H723VGT6", imu=True, answers=True, valid=True):
        """The injected behavior controls only a synthetic test device."""
        self.mcu, self.imu, self.answers, self.valid = mcu, imu, answers, valid
        self.writes = []
        self.closed = False
        self.replies = collections.deque()
        self.count = 0
        self.silent = False

    def reset_input_buffer(self):
        """Fixture starts empty; retain no bytes from a preceding connection."""
        pass

    def read_until(self, delimiter, size):
        """Generate line cadence and allow finite cancellation on silent devices."""
        time.sleep(.005)
        if self.replies:
            return self.replies.popleft()
        if self.silent:
            return b""
        self.count += 1
        return frame(self.count, self.valid) if self.imu else b"unrelated debug log\n"

    def write(self, data):
        """Only a challenge can produce an identity; unsolicited guesses fail."""
        self.writes.append(data)
        if self.answers:
            self.replies.append(identity(data.decode().strip().split()[1], self.mcu))

    def close(self):
        """Record the owner's cleanup so refusal cannot leave the port open."""
        self.closed = True


def wait(predicate, seconds=1):
    """Bound asynchronous tests and surface the hub's final state on failure."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return predicate()


class ImuTests(unittest.TestCase):
    """Recognition requires live USB metadata and a fresh protocol response."""
    def hub(self, fixture, ports=None):
        """Register cleanup for a worker fixture, including failed assertions."""
        hub = ImuHub(port_provider=ports or (lambda: [dict(CANDIDATE)]), serial_factory=lambda p: fixture)
        self.addCleanup(hub.close)
        return hub

    def test_ch340_and_jlink_are_not_candidates_and_st_is_not_h743(self):
        """An OS 'USB serial' description must never select an unrelated device."""
        ports = [SimpleNamespace(device="COM30", description="USB Serial", vid=0x1366, pid=0x0105, serial_number="JLINK"),
                 SimpleNamespace(device="COM38", description="CH340", vid=0x1A86, pid=0x7523, serial_number=None),
                 SimpleNamespace(device="COM42", description="USB Serial", vid=0x0483, pid=0x5740, serial_number="ST_UID")]
        with patch("host.imu.usb_ports", return_value=ports):
            self.assertEqual([p["id"] for p in candidate_ports()], ["COM42"])
        with patch("serial.tools.list_ports.comports", return_value=ports):
            self.assertEqual([p for p, d in list_ports()], ["COM30", "COM38"])
        with patch("host.imu.usb_ports", return_value=ports), patch("serial.Serial") as opened:
            events = queue.Queue()
            worker = Connection("COM42", events)
            worker.run()
            opened.assert_not_called()
            self.assertIn("未发送 H743", events.get()[1])

    def test_unlisted_or_no_serial_number_never_opens(self):
        """Reject arbitrary COM strings and ambiguous USB devices before I/O."""
        factory = unittest.mock.Mock()
        hub = ImuHub(serial_factory=factory, port_provider=lambda: [dict(CANDIDATE)])
        self.assertFalse(hub.connect("COM38")[0])
        hub.port_provider = lambda: [{**CANDIDATE, "serial_number": ""}]
        self.assertFalse(hub.connect(CANDIDATE["id"])[0])
        factory.assert_not_called()

    def test_exact_identity_nonce_and_uid(self):
        """Reject wrong MCU, cached nonce, empty UID, or unversioned telemetry."""
        nonce = "0123456789abcdef"
        self.assertEqual(parse_identity(identity(nonce), nonce)["uid"], UID)
        self.assertIsNone(parse_identity(identity(nonce, "STM32H743IIT6"), nonce))
        self.assertIsNone(parse_identity(identity("fedcba9876543210"), nonce))
        self.assertIsNone(parse_identity(identity(nonce).replace(UID.encode(), b"0"*24), nonce))
        self.assertIsNone(parse_identity(frame(), nonce))

    def test_verified_data_only_and_no_motor_commands(self):
        """Publish samples only after exact identity, without any H743 writes."""
        device = UsbFixture()
        hub = self.hub(device)
        self.assertTrue(hub.connect(CANDIDATE["id"])[0])
        self.assertTrue(wait(lambda: hub.state == "streaming"), hub.snapshot())
        snap = hub.snapshot(history=True)
        self.assertEqual(snap["identity"]["mcu"], "STM32H723VGT6")
        self.assertEqual(snap["latest"]["linear_accel_ms2"], (7., 8., 9.))
        self.assertTrue(snap["latest"]["valid"])
        self.assertEqual(len(device.writes), 1)
        self.assertTrue(device.writes[0].startswith(b"IMU? "))
        hub.close()
        self.assertFalse(hub.snapshot()["latest"]["valid"])
        self.assertTrue(device.closed)
        self.assertEqual(len(device.writes), 1, "disconnect must not send H743 STOP")

    def test_unrelated_device_receives_no_challenge(self):
        """Matching VID/PID is insufficient; passive data gate precedes every write."""
        device = UsbFixture(imu=False)
        hub = self.hub(device)
        # Cancel before timeout so the same no-write property is checked quickly.
        hub.connect(CANDIDATE["id"])
        time.sleep(.07)
        hub.close()
        self.assertFalse(hub.history)
        self.assertFalse(hub.identity)
        self.assertEqual(device.writes, [])

    def test_wrong_board_and_old_firmware_never_publish(self):
        """Telemetry-looking input is not accepted before a challenge reply."""
        for device in (UsbFixture(mcu="STM32H743IIT6"), UsbFixture(answers=False)):
            with self.subTest(device=device):
                hub = self.hub(device)
                hub.connect(CANDIDATE["id"])
                self.assertTrue(wait(lambda: bool(device.writes)))
                time.sleep(.06)
                self.assertFalse(hub.history)
                self.assertFalse(hub.identity)
                hub.close()

    def test_calibration_and_stale_show_invalid_without_zero_values(self):
        """A stopped or calibrating sensor must not appear as live valid data."""
        device = UsbFixture(valid=False)
        hub = self.hub(device)
        hub.connect(CANDIDATE["id"])
        self.assertTrue(wait(lambda: hub.latest is not None))
        self.assertFalse(hub.snapshot()["latest"]["valid"])
        device.valid = True
        self.assertTrue(wait(lambda: hub.state == "streaming"))
        device.silent = True
        self.assertTrue(wait(lambda: hub.state == "stale"), hub.snapshot())
        self.assertFalse(hub.snapshot()["latest"]["valid"])
        self.assertEqual(hub.snapshot()["latest"]["pitch_deg"], 20.)

    def test_port_reassignment_is_rejected_before_open(self):
        """Reusing a COM name for a different USB UID cannot reuse a selection."""
        calls = 0

        def ports():
            """Return changed metadata on the worker's second enumeration."""
            nonlocal calls
            calls += 1
            return [{**CANDIDATE, "serial_number": "ABC123" if calls == 1 else "OTHER"}]

        factory = unittest.mock.Mock()
        hub = ImuHub(port_provider=ports, serial_factory=factory)
        self.addCleanup(hub.close)
        hub.connect(CANDIDATE["id"])
        self.assertTrue(wait(lambda: hub.state == "error"))
        factory.assert_not_called()

    def test_history_bounded_and_web_keeps_h743_ownership(self):
        """UI buffers are bounded and two connection paths cannot own one port."""
        hub = self.hub(UsbFixture())
        for i in range(2000):
            hub.history.append({"sample_count": i})
            hub.pending.append({"sample_count": i})
        self.assertEqual(len(hub.history), 1500)
        self.assertEqual(len(hub.drain()[0]["points"]), 200)
        console = Console()
        console.device["_port"] = "COM_TEST"
        console.connection = SimpleNamespace(port="COM_TEST")
        self.assertFalse(console.connect_imu("COM_TEST")[0])
        self.assertIn("imu", console.hello())

    def test_fragmented_lines_and_overflow_resynchronization(self):
        """USB timeout/packet boundaries must not create fabricated samples."""
        chunks = iter([frame()[:20], b"", frame()[20:], b"x"*769, b"tail\n", frame(2)])
        stream = LineStream(SimpleNamespace(read_until=lambda *a, **k: next(chunks)))
        self.assertIsNone(stream.read()); self.assertIsNone(stream.read())
        self.assertEqual(stream.read(), frame())
        self.assertIsNone(stream.read()); self.assertIsNone(stream.read())
        self.assertEqual(stream.read(), frame(2))


if __name__ == "__main__":
    unittest.main()
