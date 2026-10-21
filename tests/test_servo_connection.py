"""Connection initialization uses unchanged servo firmware via SIM transport."""
import queue
import time
import unittest
from unittest.mock import patch

from host.servo_config import initial_commands
from host.simulator import SimulatedSerial
from host.transport import Connection


class ServoConnectionTests(unittest.TestCase):
    def wait_event(self, events, predicate):
        """Wait for a serial worker event, with a short bounded test deadline."""
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            event = events.get(timeout=max(.01, deadline - time.monotonic()))
            if predicate(event):
                return event
        self.fail("serial worker did not deliver the expected event")

    def test_connection_initializes_and_hello_queries_do_not_repeat_motion(self):
        events = queue.Queue()
        connection = Connection("SIM", events)
        connection.start()
        try:
            self.wait_event(events, lambda event: event[0] == "connected")
            self.assertEqual(connection.serial.board.servo_steps, [0, 240])
            connection.submit("SERVO 2 90")
            self.wait_event(events, lambda event: event[0] == "reply" and event[1][0] == "SERVO 2 90")
            connection.submit("HELLO")
            self.wait_event(events, lambda event: event[0] == "reply" and event[1][0] == "HELLO")
            self.assertEqual(connection.serial.board.servo_steps, [0, 180])
        finally:
            connection.close()
            connection.join(2)

    def test_rejected_initial_target_never_reports_connected(self):
        stream = SimulatedSerial()
        dispatch = stream.board.dispatch
        def reject(sequence, body):
            return "ERR code=ANGLE_RANGE" if body == "SERVO 2 120" else dispatch(sequence, body)
        stream.board.dispatch = reject
        events = queue.Queue()
        with patch("host.transport.SimulatedSerial", return_value=stream):
            connection = Connection("SIM", events)
            connection.start()
            self.wait_event(events, lambda event: event[0] == "error")
            connection.join(2)
        self.assertEqual(stream.board.servo_steps, [180, 180])
        self.assertFalse(any(event[0] == "connected" for event in list(events.queue)))

    def test_scales_checked_and_commands_follow_physical_pins(self):
        with self.assertRaises(ValueError):
            initial_commands({"servo":"2", "servo2_travel":"0"})
        self.assertEqual(initial_commands({}), [])
        self.assertEqual(initial_commands({"servo":"2"}), ["SERVO 2 120", "SERVO 1 0"])
        self.assertEqual(initial_commands({"servo":"2", "servo1_pin":"PH9", "servo2_pin":"PH6"}), ["SERVO 2 120", "SERVO 1 0"])
        self.assertEqual(initial_commands({"servo":"2", "servo1_pin":"PH6", "servo2_pin":"PE4", "servo1_travel":"1800", "servo2_travel":"2700"}), ["SERVO 1 120", "SERVO 2 0"])


if __name__ == "__main__":
    unittest.main()
