"""Headless agent checked against the real wire protocol over the demo board.

These tests do not open a COM port and are not hardware proof. They do prove
that the agent shares transport.py's heartbeat and retry path, surfaces board
errors instead of hiding them, and that its CLI runs without a GUI.
"""
import io
import json
import os
import queue
import sys
import tempfile
import threading
import time
import types
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest.mock import patch

import host.agent as agent_module
from host.agent import Agent, AgentError, main
from host.protocol import Reply


def wait_for_board(agent, timeout=5.0):
    """Block until transport has opened the demo board's serial object."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        serial = getattr(agent.connection, "serial", None)
        if serial is not None and getattr(serial, "is_open", False):
            return serial.board
        time.sleep(.01)
    raise AssertionError("demo board was never opened")


class AgentTests(unittest.TestCase):
    def setUp(self):
        self.agent = Agent("SIM", connect_timeout=5.0)
        self.agent.connect()
        self.agent.request("LIFT DIR 1", expect_ok=True)
        self.board = wait_for_board(self.agent)
        self.addCleanup(self.agent.close)

    # ---- handshake ---------------------------------------------------

    def test_connect_completes_handshake_and_reports_capabilities(self):
        # connect() returning means HELLO, PING and INFO all succeeded, exactly
        # as transport.Connection requires for the GUI.
        hello = self.agent.hello
        self.assertIsNotNone(hello)
        self.assertTrue(hello.ok)
        fields = hello.fields
        self.assertEqual(fields["protocol"], "1")
        self.assertEqual(fields["serial"], "dual")
        self.assertEqual(fields["chassis"], "1")

    def test_request_returns_matching_reply_with_sequence(self):
        reply = self.agent.request("STAT")
        self.assertIsInstance(reply, Reply)
        self.assertTrue(reply.ok)
        self.assertGreaterEqual(reply.sequence, 1)
        self.assertEqual(reply.body.split()[1], "cmd=STAT")

    def test_every_request_gets_a_distinct_sequence(self):
        first = self.agent.request("PING")
        second = self.agent.request("PING")
        self.assertNotEqual(first.sequence, second.sequence)
        self.assertEqual(second.sequence, first.sequence % 0xFFFFFFFF + 1)

    # ---- typed helpers -----------------------------------------------

    def test_fields_parses_board_state_and_keeps_raw_body(self):
        stat = self.agent.fields("STAT")
        self.assertEqual(stat["_ok"], "1")
        self.assertIn("cmd=STAT", stat["_body"])
        for key in ("uptime_ms", "heap_free", "timeout", "armed", "bad"):
            self.assertIn(key, stat)
        self.assertEqual(stat["armed"], "0")

    def test_led_command_is_executed_verbatim(self):
        reply = self.agent.request("LED 1 0", expect_ok=True)
        self.assertEqual(reply.fields["led"], "1")
        self.assertEqual(self.board.led, 1)

    def test_expect_ok_raises_instead_of_returning_the_error_reply(self):
        with self.assertRaises(AgentError) as caught:
            self.agent.request("SERVO 1 270.5", expect_ok=True)
        self.assertIn("ANGLE_RANGE", str(caught.exception))

    # ---- heartbeat ----------------------------------------------------

    def test_heartbeat_stays_alive_over_multiple_lease_windows(self):
        # The firmware lease is 1000 ms and transport pings every 250 ms. Four
        # pings per lease window means the board must never record a timeout.
        time.sleep(1.2)
        stat = self.agent.fields("STAT")
        self.assertEqual(stat["timeout"], "0")

    def test_submit_does_not_block_and_reply_arrives_later(self):
        self.assertTrue(self.agent.submit("LED 0 1"))
        self.agent.drain_for(.5)
        self.assertEqual(self.board.led, 2)

    def test_submit_then_request_on_same_body_uses_the_fresh_reply(self):
        # submit() answers are untagged; request() must ignore them and wait for
        # its own tagged answer instead of failing or returning the stale one.
        self.assertTrue(self.agent.submit("LED 1 1"))
        self.agent.drain_for(.4)
        reply = self.agent.request("LED 1 1")
        self.assertIsNotNone(reply)
        self.assertTrue(reply.ok)

    def test_poll_until_returns_when_condition_matches(self):
        result = self.agent.poll_until("STAT", "armed", lambda value: value == 0,
                                       timeout=2.0, interval=.05)
        self.assertEqual(result["armed"], "0")

    # ---- failure paths ------------------------------------------------

    def test_unreachable_port_raises_instead_of_hanging(self):
        broken = Agent("COM199_DSH_TEST", connect_timeout=3.0)
        with self.assertRaises(AgentError):
            broken.connect()
        broken.close()

    def test_request_before_connect_is_refused(self):
        fresh = Agent("SIM")
        with self.assertRaises(AgentError):
            fresh.request("PING")

    def test_lost_link_surfaces_as_agent_error(self):
        self.agent.connection.serial.close()          # board vanishes mid-session
        with self.assertRaises(AgentError):
            for _ in range(20):
                self.agent.request("PING", timeout=.3)
                time.sleep(.05)

    def test_reconnect_restores_a_usable_session_after_link_loss(self):
        # A single dropped frame must not end the session for good: the old
        # design left connect() unable to run again ("already connected").
        self.agent.connection.serial.close()
        with self.assertRaises(AgentError):
            for _ in range(20):
                self.agent.request("PING", timeout=.3)
                time.sleep(.05)
        self.assertFalse(self.agent.is_alive)
        self.assertTrue(self.agent.link_error)

        self.assertTrue(self.agent.ensure_link(), "reconnect reported failure")
        self.assertTrue(self.agent.is_alive)
        self.assertIsNone(self.agent.link_error)
        hello = self.agent.hello
        self.assertIsNotNone(hello, "reconnect did not redo the handshake")
        self.assertEqual(hello.fields["protocol"], "1")
        self.assertTrue(self.agent.request("PING").ok)

    def test_non_idempotent_commands_are_never_auto_retried(self):
        # Replaying MOVE on a fresh link could duplicate a physical movement.
        for body in ("MOVE 5 10 100 2000", "WHEELS 100 100 100 100 2000",
                     "ARM 1", "STOP", "CHASSIS MOVE FWD 10 100 2000"):
            self.assertFalse(agent_module.is_idempotent(body), body)
        for body in ("PING", "STAT", "HELLO", "INFO", "AXIS 5", "LED 1 0",
                     "CHASSIS STAT", "SERVO 1 90.0"):
            self.assertTrue(agent_module.is_idempotent(body), body)
        with self.assertRaises(ValueError):
            self.agent.request_resilient("MOVE 5 10 100 2000", attempts=2)

    def test_resilient_request_recovers_a_lost_ping(self):
        self.agent.connection.serial.close()
        deadline = time.monotonic() + 3.0
        while self.agent.is_alive and time.monotonic() < deadline:
            time.sleep(.02)                       # worker notices the dead port
        self.assertFalse(self.agent.is_alive, "dead port never surfaced")
        reply = self.agent.request_resilient("STAT", attempts=2)
        self.assertTrue(reply.ok)
        self.assertIn("cmd=STAT", reply.body)

    def test_watchdog_reconnects_without_letting_the_board_lease_lapse(self):
        # The phase-1 acceptance run in miniature: kill the link mid-flight and
        # require the watchdog to restore it, with the board recording no lease
        # timeout once the dust settles.
        notices = []
        killer = threading.Timer(.5, self.agent.connection.serial.close)
        killer.start()
        self.addCleanup(killer.cancel)
        summary = self.agent.run_watchdog(3.0, interval=.1, report_every=1.0,
                                          log=notices.append)
        self.assertGreaterEqual(summary["reconnects"], 1,
                                "watchdog never noticed the dead link")
        self.assertTrue(summary["alive"], "watchdog did not restore the link")
        self.assertEqual(summary["board_timeouts"], "0")
        self.assertTrue(any("reconnected" in note for note in notices), notices)

    def test_untagged_reply_for_the_same_body_is_dropped_not_reused(self):
        # A reply left over from submit() must never answer a later request()
        # for the same text, otherwise motion feedback could be misread.
        body = "MOVE 5 10 100 2000"
        self.agent._pending[body] = [Reply(1, "OK cmd=MOVE state=accepted")]
        reply = self.agent.request(body)
        self.assertIsNotNone(reply)
        self.assertNotEqual(reply.sequence, 1)
        self.assertNotIn(body, self.agent._pending)

    def test_heartbeat_replies_never_satisfy_a_caller_request(self):
        # transport PINGs every 250 ms and its answer lands untagged under the
        # bare body "PING". Asking for PING 40 times must therefore still return
        # 40 distinct answers belonging to the caller, in increasing sequence
        # order. The old body-matching design returned a heartbeat reply about
        # 6% of the time, which showed up here as a sequence going backwards.
        sequences = []
        tags = []
        original_submit = self.agent.connection.submit

        def interleave_heartbeat(body, tag=None):
            """Force an untagged heartbeat to arrive before each caller reply."""
            tags.append(tag)
            self.agent.events.put(("reply", ("PING", Reply(1, "OK cmd=PING marker=heartbeat"))))
            return original_submit(body, tag=tag)

        with patch.object(self.agent.connection, "submit", side_effect=interleave_heartbeat):
            for _ in range(40):
                reply = self.agent.request("PING")
                self.assertIsNotNone(reply)
                self.assertTrue(reply.ok)
                self.assertNotIn("marker", reply.fields)
                sequences.append(reply.sequence)
                time.sleep(.02)
        self.assertEqual(len(set(sequences)), 40)
        self.assertEqual(sequences, sorted(sequences))
        # Bare PING is valid background traffic, even during the final request.
        # Only the unique caller tags must have been consumed completely.
        self.assertTrue(all(tag not in self.agent._pending for tag in tags),
                        "a caller PING answer was left unclaimed")

    def test_connect_fails_loudly_when_hello_reply_is_lost(self):
        # Reporting connected=True with no HELLO would hide a protocol mismatch.
        agent = Agent("SIM", connect_timeout=5.0)
        agent._store_reply = lambda payload: None       # swallow every reply
        with self.assertRaises(AgentError) as caught:
            agent.connect()
        self.assertIn("handshake reply for HELLO", str(caught.exception))
        agent.close()

    def test_warnings_are_recorded_not_raised(self):
        self.agent.connection.events.put(("warning", "synthetic warning"))
        self.agent.request("PING")
        self.assertIn("synthetic warning", self.agent.warnings)


class AgentCliTests(unittest.TestCase):
    """The CLI is what actually runs on the Jetson, so exercise it in-process."""

    def run_cli(self, argv):
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            status = main(argv)
        return status, buffer.getvalue()

    def test_check_against_demo_handshakes_without_a_port(self):
        status, output = self.run_cli(["--demo", "--check"])
        self.assertEqual(status, 0)
        self.assertIn("connected: SIM", output)
        self.assertIn("protocol=1", output)

    def test_json_output_is_parseable(self):
        status, output = self.run_cli(["--demo", "--check", "--json"])
        self.assertEqual(status, 0)
        payload = json.loads(output.strip().splitlines()[0])
        self.assertTrue(payload["connected"])
        self.assertEqual(payload["port"], "SIM")
        self.assertIn("protocol=1", payload["hello"])

    def test_heartbeat_report_shows_zero_failures(self):
        status, output = self.run_cli(["--demo", "--json", "--heartbeat-seconds", "1.2"])
        self.assertEqual(status, 0)
        report = json.loads(output.strip().splitlines()[-1])
        self.assertGreaterEqual(report["pings"], 4)
        self.assertEqual(report["failures"], 0)
        self.assertEqual(report["board_timeouts"], "0")

    def test_script_mode_runs_each_line_and_skips_comments(self):
        handle = tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8")
        handle.write("# comment line\n\nHELLO\nSTAT\n")
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        status, output = self.run_cli(["--demo", "--script", handle.name])
        self.assertEqual(status, 0)
        self.assertIn("cmd=HELLO", output)
        self.assertIn("cmd=STAT", output)
        self.assertNotIn("# comment line", output)

    def test_missing_port_argument_is_a_usage_error(self):
        with self.assertRaises(SystemExit) as caught:
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                main([])
        self.assertEqual(caught.exception.code, 2)

    def test_unreachable_port_exits_nonzero(self):
        status, _ = self.run_cli(["--port", "COM199_DSH_TEST"])
        self.assertEqual(status, 2)


class SerialIntegrationTests(unittest.TestCase):
    """The port-opening path with a real pyserial module doing the work.

    A fake `serial` is injected into sys.modules so the test runs on machines
    without pyserial, while transport.py itself is untouched: it still does
    `import serial` and `serial.Serial(...)`. This covers the code that runs in
    the seconds before the STM32 replies -- wrong device, no permission, a port
    that refuses writes -- which the simulator never reaches.
    """

    class _FakeSerial:
        """Records constructor arguments and honours open/close semantics."""
        instances = []

        def __init__(self, port, baudrate, timeout=None, write_timeout=None):
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self.write_timeout = write_timeout
            self.written = bytearray()
            self.is_open = True
            self.buffer = bytearray()
            SerialIntegrationTests._FakeSerial.instances.append(self)

        def write(self, data):
            if not self.is_open:
                raise OSError("port is closed")
            self.written.extend(data)
            return len(data)

        def read(self, size=256):
            if not self.is_open:
                raise OSError("port is closed")
            chunk = bytes(self.buffer[:size])
            del self.buffer[:size]
            if not chunk:
                time.sleep(.005)
            return chunk

        def close(self):
            self.is_open = False

    def setUp(self):
        self.instances = self._FakeSerial.instances = []
        self.original_serial = sys.modules.get('serial')
        self.addCleanup(self._restore_serial)
        self.fake = types.ModuleType("serial")
        self.fake.Serial = self._FakeSerial
        sys.modules["serial"] = self.fake

    def _restore_serial(self):
        if self.original_serial is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules['serial'] = self.original_serial

    def _serve_replies(self, connection, body_to_reply):
        """Answer the worker's requests on a background thread."""
        from host.protocol import decode, encode

        def loop():
            for _ in range(400):
                serial_obj = getattr(connection, "serial", None)
                if serial_obj is not None and serial_obj.written:
                    for frame in bytes(serial_obj.written).splitlines(keepends=True):
                        try:
                            request = decode(frame)
                        except ValueError:
                            continue
                    del serial_obj.written[:]
                    reply = body_to_reply.get(request.body)
                    if reply is not None:
                        serial_obj.buffer.extend(encode(request.sequence, reply))
                time.sleep(.005)

        thread = threading.Thread(target=loop, daemon=True)
        thread.start()
        return thread

    def test_real_serial_constructor_receives_the_documented_parameters(self):
        from host.transport import Connection

        events = queue.Queue()
        connection = Connection("/dev/ttyUSB0", events)
        self._serve_replies(connection, {
            "HELLO": "OK cmd=HELLO protocol=1 serial=dual chassis=1",
            "PING": "OK cmd=PING",
            "INFO": "OK cmd=INFO cpu_hz=400000000",
        })
        connection.start()
        deadline = time.monotonic() + 5.0
        kinds = []
        while time.monotonic() < deadline:
            try:
                kinds.append(events.get(timeout=.1)[0])
            except queue.Empty:
                continue
            if "connected" in kinds or "error" in kinds:
                break
        connection.close()
        connection.join(2.0)

        self.assertEqual(len(self.instances), 1, "serial.Serial was not used")
        opened = self.instances[0]
        self.assertEqual(opened.port, "/dev/ttyUSB0")
        self.assertEqual(opened.baudrate, 115200)
        self.assertIsNotNone(opened.timeout)
        self.assertIsNotNone(opened.write_timeout)
        self.assertIn("connected", kinds)
        self.assertFalse(opened.is_open, "the port was left open after close()")

    def test_permission_denied_is_reported_not_retried_forever(self):
        from host.transport import Connection

        def refuse(*_args, **_kwargs):
            raise PermissionError(13, "Permission denied")

        self.fake.Serial = refuse
        events = queue.Queue()
        connection = Connection("/dev/ttyUSB0", events)
        connection.start()
        try:
            kind, payload = events.get(timeout=5.0)
        except queue.Empty:
            self.fail("open failure produced no event")
        connection.close()
        connection.join(2.0)
        self.assertEqual(kind, "error")
        self.assertIn("Permission denied", str(payload))

    def test_missing_device_is_reported_with_the_device_name(self):
        from host.transport import Connection

        def missing(*_args, **_kwargs):
            raise FileNotFoundError(2, "No such file or directory")

        self.fake.Serial = missing
        events = queue.Queue()
        connection = Connection("/dev/ttyUSB0", events)
        connection.start()
        try:
            kind, payload = events.get(timeout=5.0)
        except queue.Empty:
            self.fail("missing device produced no event")
        connection.close()
        connection.join(2.0)
        self.assertEqual(kind, "error")
        # The message must name the device, otherwise "not found" and
        # "permission denied" are indistinguishable to whoever is wiring it up.
        self.assertIn("/dev/ttyUSB0", str(payload))

    def test_list_ports_survives_a_missing_pyserial(self):
        from host.transport import list_ports

        # pyserial may be absent (a bare board) or (as on this dev machine)
        # present; force the absent case so list_ports must return empty rather
        # than blow up. None makes "import serial.tools.list_ports" raise.
        sys.modules["serial"] = None
        self.assertEqual(list_ports(), [])


if __name__ == "__main__":
    unittest.main()
