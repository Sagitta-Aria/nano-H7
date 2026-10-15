"""Control API checked against the real wire protocol over the demo board.

The two properties worth testing here are the ones that would hurt a robot:
a motion command must never be re-sent behind the caller's back, and a failed
continuous-motion pass must leave the board stopped rather than running on the
last accepted target.
"""
import time
import unittest

from host.agent import Agent
from host.control import Control, ControlError


def wait_for_board(control, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        serial = getattr(control.agent.connection, "serial", None)
        if serial is not None and getattr(serial, "is_open", False):
            return serial.board
        time.sleep(.01)
    raise AssertionError("demo board was never opened")


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.control = Control("SIM", agent=Agent("SIM", connect_timeout=5.0))
        self.control.connect()
        self.control.agent.request("LIFT DIR 1", expect_ok=True)
        self.board = wait_for_board(self.control)
        self.addCleanup(self.control.close)

    # ---- typed access -------------------------------------------------

    def test_hello_reports_protocol_and_capabilities(self):
        fields = self.control.hello()
        self.assertEqual(fields["protocol"], "1")
        self.assertEqual(fields["serial"], "dual")
        self.assertEqual(self.control.require_protocol("1")["board"], "SIMULATOR")

    def test_require_protocol_rejects_a_mismatch(self):
        with self.assertRaises(ControlError):
            self.control.require_protocol("2")

    def test_status_fields_are_integers_not_strings(self):
        status = self.control.status()
        for key in ("uptime_ms", "heap_free", "armed", "timeout", "bad"):
            self.assertIsInstance(status[key], int, key)
        self.assertEqual(status["armed"], 0)

    def test_axis_status_is_typed(self):
        axis = self.control.axis(5)
        self.assertEqual(axis["axis"], 5)
        self.assertEqual(axis["pos"], 0)
        self.assertIsInstance(axis["remaining"], int)

    # ---- motion guards -------------------------------------------------

    def test_arm_needs_a_lease_and_then_reports_armed(self):
        self.control.arm()
        self.assertEqual(self.control.status()["armed"], 1)
        self.control.disarm()
        self.assertEqual(self.control.status()["armed"], 0)

    def test_wheels_requires_exactly_four_speeds(self):
        self.control.arm()
        with self.assertRaises(ControlError):
            self.control.wheels((100, 100, 100))
        self.control.wheels((100, 100, 100, 100))

    def test_wheels_translates_to_the_board_axes(self):
        self.control.arm()
        self.control.wheels((10, -10, 20, -20))
        self.assertEqual([int(a["target"]) for a in self.board.axes[:4]], [10, -10, 20, -20])

    def test_move_then_wait_uses_the_boards_own_completion(self):
        self.control.arm()
        accepted = self.control.move_axis(5, steps=50, sps=3000, accel=5000)
        self.assertEqual(accepted["state"], "accepted")
        final = self.control.wait_axis(5, move_id=accepted["move_id"], timeout=5.0)
        self.assertEqual(final["remaining"], 0)
        self.assertEqual(final["active"], 0)
        self.assertEqual(final["pos"], 50)

    def test_wait_axis_times_out_instead_of_returning_early(self):
        self.control.arm()
        # A move_id that never appears must not be accepted as "finished".
        self.control.move_axis(5, steps=20, sps=100, accel=100)
        with self.assertRaises(ControlError):
            self.control.wait_axis(5, move_id=999999, timeout=.5, interval=.05)

    def test_rejected_motion_raises_with_the_board_reason(self):
        # Not armed: the firmware refuses, and the caller must be told why.
        with self.assertRaises(ControlError) as caught:
            self.control.wheels((100, 100, 100, 100))
        self.assertIn("DISARMED", str(caught.exception))

    def test_motion_is_never_auto_retried_after_a_link_loss(self):
        self.control.arm()
        self.control.agent.connection.serial.close()
        deadline = time.monotonic() + 3.0
        while self.control.agent.is_alive and time.monotonic() < deadline:
            time.sleep(.02)
        with self.assertRaises(ControlError) as caught:
            self.control.wheels((100, 100, 100, 100))
        # The message must say it was not confirmed, not that it succeeded.
        self.assertIn("not confirmed", str(caught.exception))

    def test_queries_do_recover_across_a_link_loss(self):
        self.control.agent.connection.serial.close()
        deadline = time.monotonic() + 3.0
        while self.control.agent.is_alive and time.monotonic() < deadline:
            time.sleep(.02)
        status = self.control.status()          # idempotent: retried and recovered
        self.assertEqual(status["_ok"], "1")
        self.assertIsInstance(status["uptime_ms"], int)

    # ---- the safety-critical primitive ---------------------------------

    def test_hold_velocity_renews_the_target_and_stops_at_the_end(self):
        self.control.arm()
        report = self.control.hold_velocity((0, 0, 150, -150), seconds=1.0, hz=10.0)
        self.assertGreaterEqual(report["renewals"], 8)
        self.assertEqual(report["hz"], 10.0)
        # Stopped, not merely finished: every wheel target is zero.
        self.assertEqual([int(a["target"]) for a in self.board.axes[:4]], [0, 0, 0, 0])

    def test_hold_velocity_refuses_a_rate_below_the_lease_floor(self):
        with self.assertRaises(ControlError):
            self.control.hold_velocity((0, 0, 100, -100), seconds=1.0, hz=0.5)

    def test_a_failed_pass_stops_the_board_before_raising(self):
        self.control.arm()
        calls = []
        original = self.control.wheels

        def failing_on_second(speeds, accel=2000):
            calls.append(tuple(speeds))
            if len(calls) == 2:
                raise ControlError("synthetic mid-run failure")
            return original(speeds, accel=accel)

        self.control.wheels = failing_on_second
        with self.assertRaises(ControlError):
            self.control.hold_velocity((0, 0, 200, -200), seconds=2.0, hz=10.0)
        # The stop must be the last thing sent, even though the run aborted.
        self.assertEqual(calls[-1], (0, 0, 0, 0))
        self.assertEqual([int(a["target"]) for a in self.board.axes[:4]], [0, 0, 0, 0])

    def test_hold_velocity_stops_even_when_interrupted(self):
        self.control.arm()
        calls = []
        original = self.control.wheels

        def counting(speeds, accel=2000):
            calls.append(tuple(speeds))
            if len(calls) == 2:
                raise KeyboardInterrupt()
            return original(speeds, accel=accel)

        self.control.wheels = counting
        with self.assertRaises(KeyboardInterrupt):
            self.control.hold_velocity((0, 0, 200, -200), seconds=3.0, hz=10.0)
        self.assertEqual(calls[-1], (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
