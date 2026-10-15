"""Replay real ARM parser/integration and host recording/configuration paths.

No flashing, physical sensor accuracy or actual UART timing is claimed here.
"""
import json
import struct
import unittest
from pathlib import Path
from tests import test_chassis as chassis_tests


def packet(x=0, y=0, dt=5065, quality=200):
    """Build a complete Upix fixture including ignored/reserved bytes."""
    data = struct.pack("<hhHHBB", x, y, dt, 0x1234, quality, 1)
    checksum = 0
    for b in data:
        checksum ^= b
    return b"\xfe\x0a" + data + bytes((checksum, 0x55))


class ArmFlowTests(unittest.TestCase):
    setUp = chassis_tests.ArmChassisTests.setUp
    request = chassis_tests.ArmChassisTests.request

    def feed(self, data, now=10):
        """Run the compiled byte parser, preserving arbitrary packet boundaries."""
        self.arm.write32(self.arm.symbols["xTickCount"], now)
        for b in data:
            self.arm.call("optical_flow_byte", b, now)

    def stat(self):
        return self.request("FLOW STAT").fields

    def test_signed_vector_sum_and_fragment_resynchronisation(self):
        self.feed(b"noise\xfe\x11" + packet(100, -50)[:7])
        self.assertEqual(self.stat()["frames"], "0")
        self.feed(packet(100, -50)[7:])
        self.assertEqual((self.stat()["x_um"], self.stat()["y_um"]), ("300", "-150"))
        damaged = bytearray(packet(500, 500)); damaged[12] ^= 1
        self.feed(bytes(damaged)+packet(-100, 50))
        s = self.stat()
        self.assertEqual((s["x_um"], s["y_um"], s["accepted"]), ("0", "0", "2"))
        self.assertGreater(int(s["bad"]), 0)

    def test_fractional_increment_precision_and_future_only_calibration(self):
        self.assertTrue(self.request("FLOW CONFIG 10000 500000 500000 0 0 1").ok)
        for _ in range(10):
            self.feed(packet(1, -1))
        self.assertEqual((self.stat()["x_um"], self.stat()["y_um"]), ("5", "-5"))
        self.request("FLOW CONFIG 30000 1000000 1000000 0 0 1")
        self.assertEqual(self.stat()["x_um"], "5")
        self.feed(packet(100, 0))
        self.assertEqual(self.stat()["x_um"], "305")
        before = self.request("FLOW CONFIG").fields
        self.assertFalse(self.request("FLOW CONFIG 30000 0 1000000 0 0 1").ok)
        self.assertEqual(self.request("FLOW CONFIG").fields, before)

    def test_recording_time_freeze_restart_and_replay(self):
        self.feed(packet(100, 0), 10)
        self.request("FLOW START", sequence=1000)
        self.feed(packet(100, 50), 20)
        self.request("FLOW START", sequence=1000)
        self.assertEqual(self.stat()["rx_um"], "300")
        stopped = self.request("FLOW STOP").fields
        self.assertEqual((stopped["start_ms"], stopped["end_ms"]), ("10", "20"))
        self.feed(packet(100, 50), 30)
        self.assertEqual((self.stat()["x_um"], self.stat()["rx_um"], self.stat()["ry_um"]), ("900", "300", "150"))
        self.request("FLOW CONFIG 40000 1000000 1000000 0 0 1")
        self.assertEqual(self.stat()["rx_um"], "300")
        restarted = self.request("FLOW START").fields
        self.assertEqual((restarted["rx_um"], restarted["start_ms"], restarted["x_um"]), ("0", "30", "900"))

    def test_swap_sign_mounting_and_heading_rotate_each_increment(self):
        self.request("NAV IMU 0 1 1")
        self.feed(packet(0, 100), 10)
        self.request("NAV IMU 9000 1 1")
        self.feed(packet(0, 100), 20)
        s = self.request("FLOW WORLD").fields
        self.assertEqual((s["wx_um"], s["wy_um"], self.stat()["y_um"]), ("300", "300", "600"))
        self.request("FLOW CONFIG 30000 2000000 1000000 5 9000 1")
        self.feed(packet(100, 200), 30)
        self.assertEqual((self.stat()["x_um"], self.stat()["y_um"]), ("300", "1800"))

    def test_heading_gap_epoch_lock_and_quality_rejection(self):
        self.request("FLOW CONFIG 30000 1000000 1000000 0 0 100")
        self.feed(packet(100, 0, quality=0)+packet(100, 0, quality=99)+packet(100, 0, dt=0))
        self.assertEqual(self.stat()["accepted"], "0")
        self.feed(packet(100, 0), 20)
        self.assertEqual(self.request("FLOW WORLD").fields["gaps"], "1")
        self.request("NAV IMU 0 1 1")
        self.feed(packet(100, 0), 30)
        self.feed(packet(100, 0), 400)
        s = self.request("FLOW WORLD").fields
        self.assertEqual((s["wx_um"], s["gaps"], s["heading"]), ("300", "2", "0"))
        self.request("NAV IMU 0 1 2")
        self.feed(packet(100, 0), 410)
        s = self.request("FLOW WORLD").fields
        self.assertEqual((s["wx_um"], s["locked"], s["gaps"]), ("300", "1", "3"))
        self.assertEqual(self.stat()["x_um"], "1200")

    def test_uart_pin_setup_bounded_ring_and_age_wrap(self):
        self.arm.call("serial_init")
        self.arm.call("optical_flow_init")
        gpio_b = 0x58020400
        self.assertEqual((self.arm.read32(gpio_b+0x24) >> 12) & 15, 7)
        self.assertEqual((self.arm.read32(gpio_b+0x24) >> 24) & 15, 7)
        self.assertEqual(self.arm.read32(0x4000480C), 868)
        self.arm.write32(0x4000481C, 1 << 5)
        for b in packet(100, -100):
            self.arm.write32(0x40004824, b)
            self.arm.call("USART3_IRQHandler")
        self.arm.write32(self.arm.symbols["xTickCount"], 10)
        self.request("FLOW START")  # command boundary must first drain queued bytes
        self.assertEqual(self.stat()["x_um"], "300")
        self.assertEqual(self.stat()["rx_um"], "0")
        self.feed(packet(), 0xFFFFFFFA)
        self.arm.write32(self.arm.symbols["xTickCount"], 3)
        self.assertEqual(self.stat()["online"], "1")

    def test_captured_sensor_data_matches_replay_sum(self):
        capture = Path(__file__).resolve().parents[1]/"docs/optical_flow_com38_received_20261013.json"
        if not capture.exists():
            self.skipTest("local sensor capture is absent")
        r = json.loads(capture.read_text(encoding="utf-8"))
        data = b"".join(bytes.fromhex(s["hex"]) for s in r["samples"])
        self.feed(data)
        self.assertEqual(self.stat()["frames"], "920")
        self.assertEqual(self.stat()["accepted"], "920")
        valid = []
        for i in range(len(data)-13):
            f = data[i:i+14]
            xor = 0
            for b in f[2:12]: xor ^= b
            if f[:2] == b"\xfe\x0a" and f[13] == 0x55 and f[12] == xor:
                valid.append(struct.unpack("<hh", f[2:6]))
        self.assertEqual(int(self.stat()["x_um"]), sum(x for x, _ in valid)*3)
        self.assertEqual(int(self.stat()["y_um"]), sum(y for _, y in valid)*3)

    def test_uart_overflow_discards_fragment_then_recovers(self):
        self.arm.write32(0x4000481C, 1 << 5)
        for b in packet(100, 100)[:5] + bytes(1024):
            self.arm.write32(0x40004824, b)
            self.arm.call("USART3_IRQHandler")
        self.arm.call("optical_flow_poll", 10)
        self.assertEqual(self.stat()["accepted"], "0")
        self.assertGreater(int(self.stat()["uart"]), 0)
        for b in packet(-100, 100):
            self.arm.write32(0x40004824, b)
            self.arm.call("USART3_IRQHandler")
        self.arm.call("optical_flow_poll", 20)
        self.assertEqual((self.stat()["x_um"], self.stat()["y_um"]), ("-300", "300"))
        self.assertEqual(self.arm.read32(0x08000000+4*(16+39)), self.arm.symbols["USART3_IRQHandler"] | 1)

    def test_long_totals_preserve_tiny_increments_and_bounded_telemetry(self):
        self.request("FLOW CONFIG 1000 10000 10000 0 0 1")
        self.arm.uc.mem_write(self.arm.symbols["x_um"], struct.pack("<q", 1000000000000000))
        for _ in range(1000): self.feed(packet(1, 0))
        self.assertEqual(self.stat()["x_um"], "1000000000001")
        self.arm.uc.mem_write(self.arm.symbols["x_um"], struct.pack("<q", 4000000000000000001))
        self.feed(packet(100, 100))
        self.assertEqual(self.stat()["sat"], "1")
        for name in ("x_um", "y_um"):
            self.arm.uc.mem_write(self.arm.symbols[name], struct.pack("<q", 4000000000000000000))
        for name in ("base_x", "base_y"):
            self.arm.uc.mem_write(self.arm.symbols[name], struct.pack("<q", -4000000000000000000))
        for name in ("packets", "accepted", "bad", "low_quality", "rx_errors", "mark_ms", "end_ms"):
            self.arm.write32(self.arm.symbols[name], 0xffffffff)
        for name, value in (("marked",1),("recording",1),("config_revision",0xffffffff),
                            ("quality",255),("dt_us",65535),("last_x",-32768),("last_y",-32768)):
            self.arm.write32(self.arm.symbols[name], value)
        self.arm.write32(self.arm.symbols["xTickCount"], 0xffffffff)
        self.assertTrue(self.request("FLOW STAT").ok)
        self.assertLess(len(self.arm.transmitted), 384)




if __name__ == "__main__":
    unittest.main()
