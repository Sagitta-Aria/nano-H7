"""Exercise the web console against the demo transport and a real HTTP server.

This checks the parts a browser cannot: that the console funnels the simulator
through the same protocol encoder, that polling and role hand-off work, and that
the HTTP layer refuses a second page's commands. The page's own behaviour is
covered separately by tests/test_webpage.py, which validates the DOM contract.
"""
import json
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from host.webapp import Console
from host.webserver import Handler, Session, STATIC

ROOT = Path(__file__).resolve().parents[1]


def wait_for(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.02)
    return False


class ConsoleTests(unittest.TestCase):
    """No HTTP: the session object on its own."""

    def setUp(self):
        self.settings_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.settings_dir.cleanup)
        root = Path(self.settings_dir.name)
        self.console = Console(root/'navigation.json', root/'auxiliary.json', root/'optical_flow.json')
        self.seen = []
        self.ready = False

    def tearDown(self):
        if self.console.attached():
            self.console.disconnect()
            wait_for(lambda: not self.console.attached())
        self.console.drain()  # never leave an unread "error" event behind

    def pump(self, seconds=1.):
        """Drain like the server pump does, and remember what came out."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.seen.extend(self.console.drain())
            self.console.tick()
            time.sleep(.01)

    def pump_until(self, predicate, timeout=5.):
        """Pump while waiting: a reply only reaches the console when drained."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.seen.extend(self.console.drain())
            self.console.tick()
            if predicate():
                return True
            time.sleep(.01)
        return predicate()

    def connect_demo(self):
        """Connect the simulated board and wait for HELLO, draining as we go.

        Draining matters: the worker reports a handshake failure as an "error"
        event, and an undrained queue would turn that into a bare timeout.
        """
        self.console.last_client = time.monotonic()
        ok, why = self.console.connect("SIM")
        self.assertTrue(ok, why)
        if not self.pump_until(lambda: self.console.device.get("fw"), timeout=8.):
            self.fail(f"handshake did not complete; events: {self.seen[-6:]}")

    def test_hello_handshake_reaches_the_page(self):
        self.connect_demo()
        hello = self.console.hello()
        self.assertEqual(hello["link"], "connected")
        self.assertTrue(hello["demo"])
        self.assertEqual(hello["device"]["fw"], "0.2.29")
        self.assertEqual(hello["device"]["nav_grid"], "1")
        self.assertEqual(hello["device"]["servo1_travel"], "2700")
        self.assertEqual(hello["device"]["servo2_travel"], "1800")
        self.assertEqual(hello["device"]["servo"], "2")
        self.assertEqual(hello["servo_channels"], 2)
        self.assertEqual(hello["travel"], 270.0)

    def test_polling_fills_axes_servo_and_chassis(self):
        self.connect_demo()
        self.pump(2.)
        self.assertEqual(len(self.console.axes), 7)
        self.assertTrue(self.console.servo, "SERVO STAT should populate the servo snapshot")
        raw = self.console.servo.get("raw", "")
        # Both wired channels must appear in the one packed reply body.
        self.assertIn("ch=2 angle=120.0 us=1833", raw)
        self.assertIn("ch=1 angle=0.0 us=500", raw)
        self.assertEqual(self.console.servo_count(), 2)
        self.assertTrue(self.console.chassis, "CHASSIS STAT should populate the chassis snapshot")
        self.assertEqual(self.console.chassis["state"], "idle")

    def test_commands_use_the_real_protocol_and_update_state(self):
        self.connect_demo()
        self.console.submit("ARM 1")
        self.assertTrue(self.pump_until(lambda: self.console.armed), "ARM must take effect")
        self.console.submit("SERVO 1 135")
        self.console.submit("SERVO 2 45")
        self.assertTrue(self.pump_until(lambda: "ch=1 angle=135.0" in self.console.servo.get("raw", "")
                                        and "ch=2 angle=45.0" in self.console.servo.get("raw", "")),
                        f"servo={self.console.servo}")
        self.console.submit("SERVO 1 270.5")
        self.console.submit("SERVO 1 90x")
        self.console.submit("SERVO 3 90")
        self.pump(.6)
        raw = self.console.servo.get("raw", "")
        self.assertIn("ch=1 angle=135.0", raw, "refused angles must not move the servo")
        self.assertIn("ch=2 angle=45.0", raw)

    def test_arm_ack_and_following_snapshot_always_agree(self):
        """The SSE pump sends a snapshot immediately after each ARM reply.

        Exercise that boundary without hardware: stale cached STAT must not
        undo the accepted ARM/STOP state when the page adopts the snapshot.
        """
        from host.protocol import Reply
        for body, accepted in (("ARM 1", True), ("ARM 0", False),
                               ("ARM 1", True), ("STOP", False)):
            self.console._reply("STAT", Reply(1, "OK cmd=STAT armed={}".format(int(not accepted))))
            events = self.console._reply(body, Reply(2, "OK cmd={} armed={}".format(body.split()[0], int(accepted))))
            self.assertEqual(events[-1], {"type": "arm", "armed": accepted})
            self.assertEqual(self.console.hello()["armed"], accepted,
                             "snapshot overwrote the successful " + body + " reply")
        self.console._reply("ARM 1", Reply(3, "ERR code=HEARTBEAT_REQUIRED"))
        self.assertFalse(self.console.hello()["armed"], "rejected ARM must not claim enabled")

    def test_s2_half_turn_limit_and_midpoint_are_channel_specific(self):
        """Check GUI transport through the simulator, including rejected moves."""
        self.connect_demo()
        self.console.submit("SERVO 2 180")
        self.assertTrue(self.pump_until(lambda: "ch=2 angle=180.0 us=2500" in self.console.servo.get("raw", "")))
        self.console.submit("SERVO 2 180.5")
        self.assertTrue(self.pump_until(lambda: any("ANGLE_RANGE" in row["text"] for row in self.console.log)))
        self.assertIn("ch=2 angle=180.0 us=2500", self.console.servo["raw"])
        self.console.submit("SERVO 2 90")
        self.assertTrue(self.pump_until(lambda: "ch=2 angle=90.0 us=1500" in self.console.servo.get("raw", "")))

    def test_stop_is_submitted_and_disarms(self):
        self.connect_demo()
        self.console.submit("ARM 1")
        self.assertTrue(self.pump_until(lambda: self.console.armed))
        self.console.submit("STOP")
        self.assertTrue(self.pump_until(lambda: not self.console.armed))

    def test_lift_connect_reference_and_live_limits(self):
        """Exercise automatic INIT and semantic moves through the real worker."""
        self.connect_demo()
        self.assertTrue(self.pump_until(lambda: self.console.lift.get("initialized") == "1"))
        self.assertEqual(self.console.lift["down_sign"], "0")
        self.assertEqual(self.console.lift["pos"], "0")
        self.console.submit("LIFT DIR -1")
        self.assertTrue(self.pump_until(lambda: self.console.lift.get("down_sign") == "-1"))
        self.console.submit("ARM 1")
        self.assertTrue(self.pump_until(lambda: self.console.armed))
        self.console.submit("LIFT MOVE DOWN 25 1000 2000")
        self.assertTrue(self.pump_until(lambda: self.console.lift.get("pos") == "25"
                                        and self.console.lift.get("active") == "0"))
        self.assertEqual(self.console.lift["down_room"], "7975")
        self.assertTrue(any(e["type"] == "lift" and e["fields"].get("pos") == "25" for e in self.seen))
        self.console.submit("LIFT INIT")
        self.console.submit("LIFT MOVE UP 26 1000 2000")
        self.assertTrue(self.pump_until(lambda: any("M5_SOFT_LIMIT" in row["text"] for row in self.console.log)))
        self.assertEqual(self.console.lift["pos"], "25")
        self.console.submit("LIFT MOVE UP 25 1000 2000")
        self.assertTrue(self.pump_until(lambda: self.console.lift.get("pos") == "0"
                                        and self.console.lift.get("active") == "0"))
        self.assertEqual(self.console.hello()["lift"]["down_room"], "8000")

    def test_wiring_acknowledgement_gates_a_real_port_only(self):
        self.connect_demo()
        # The demo board never needs the acknowledgement, and none was given.
        self.assertFalse(self.console.device.get("_ack"))
        self.assertTrue(self.console.submit("ARM 1")[0])
        # A real port name must engage the guard until it is acknowledged.
        self.console.device["_port"] = "COM_NOT_REAL"
        ok, why = self.console.submit("ARM 1")
        self.assertFalse(ok)
        self.assertIn("接线", why)
        self.console.set_wiring_ack(True)
        self.assertTrue(self.console.submit("ARM 1")[0])

    def test_malformed_bodies_are_refused_before_the_queue(self):
        for body in ("", "   ", "*", "PING\nPING", "A" * 200, "PING\u4e2d"):
            ok, why = self.console.submit(body)
            self.assertFalse(ok, f"{body!r} should be refused")
            self.assertIsInstance(why, str)
        self.assertFalse(self.console.submit("PING")[0], "not connected yet")


    def test_control_hands_over_when_the_controller_goes_quiet(self):
        """A page that dies without closing must not lock every later page out."""
        session = Session(Console(), exit_when_idle=False, control_lease=.3)
        session.join("dead-page")
        self.assertTrue(session.is_controller("dead-page"))
        session.join("live-page")
        self.assertFalse(session.is_controller("live-page"), "the second page is read-only at first")
        # Silence the first page past its lease, then let the pump hand over.
        with session.lock:
            session.touched["dead-page"] = time.monotonic() - 5.
        self.assertTrue(session.release_stale_control())
        self.assertEqual(session.controller, "live-page")
        self.assertTrue(session.is_controller("live-page"))
        self.assertFalse(session.is_controller("dead-page"))

    def test_leaving_frees_the_role_for_the_next_page(self):
        session = Session(Console(), exit_when_idle=False)
        session.join("first")
        session.join("second")
        session.leave("first")
        self.assertEqual(session.controller, "second")
        session.leave("second")
        self.assertIsNone(session.controller)

    def test_stale_handover_notifies_each_page_of_its_own_role(self):
        """A live readonly browser must learn when a ghost controller expires."""
        session = Session(Console(), control_lease=.3)
        old, _ = session.join('old')
        live, _ = session.join('live')
        session.touched['old'] = time.monotonic() - 5
        session.touch('live')
        self.assertTrue(session.release_stale_control())
        def roles(inbox):
            events = []
            while not inbox.empty():
                events.append(inbox.get_nowait())
            return [e['role'] for e in events if e['type'] == 'role']
        self.assertEqual(roles(old), ['readonly'])
        self.assertEqual(roles(live), ['controller'])

    def test_leave_does_not_grant_all_remaining_pages_control(self):
        """Closing one page must promote only one of multiple remaining views."""
        session = Session(Console())
        session.join('first')
        second, _ = session.join('second')
        third, _ = session.join('third')
        session.leave('first')
        self.assertEqual(second.get_nowait()['role'], 'controller')
        self.assertEqual(third.get_nowait()['role'], 'readonly')

    def test_heartbeat_and_claim_keep_one_controller_and_stop_before_transfer(self):
        """Inbound browser leases prevent passive pages stealing a live controller."""
        from unittest.mock import patch
        session = Session(Console(), control_lease=.3)
        first, _ = session.join('first')
        second, _ = session.join('second')
        self.assertEqual(session.heartbeat('first'), 'controller')
        self.assertEqual(session.heartbeat('second'), 'readonly')
        self.assertFalse(session.release_stale_control())
        with patch.object(session.console, 'attached', return_value=True), patch.object(session.console, 'submit') as submit:
            submit.return_value = False, 'queue full'
            self.assertFalse(session.claim('second')[0])
            self.assertEqual(session.controller, 'first')
            submit.return_value = True, 'STOP'
            self.assertTrue(session.claim('second')[0])
            submit.assert_called_with('STOP')
            self.assertEqual(session.heartbeat('first'), 'readonly')
            self.assertEqual(session.heartbeat('second'), 'controller')
        self.assertFalse(session.claim('unregistered')[0])

    def test_stale_stream_close_cannot_remove_reconnected_page(self):
        session = Session(Console())
        old, _ = session.join('page')
        current, first = session.join('page')
        self.assertTrue(first)
        session.leave('page', old)
        self.assertIs(session.clients['page'], current)
        self.assertEqual(session.controller, 'page')

    def test_all_stale_pages_do_not_pass_control_back_and_forth(self):
        session = Session(Console(), control_lease=.3)
        session.join('first')
        session.join('second')
        for token in session.touched:
            session.touched[token] = time.monotonic() - 5
        self.assertTrue(session.release_stale_control())
        self.assertIsNone(session.controller)
        self.assertEqual(session.heartbeat('second'), 'controller')



class HttpTests(unittest.TestCase):
    """One real server on a loopback port, driven with urllib."""

    @classmethod
    def setUpClass(cls):
        from http.server import ThreadingHTTPServer
        import threading
        cls.console = Console()
        cls.session = Session(cls.console, exit_when_idle=False)
        handler = type("BoundHandler", (Handler,), {"session": cls.session})
        cls.httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        cls.httpd.daemon_threads = True
        cls.port = cls.httpd.server_address[1]
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, kwargs={"poll_interval": .1},
                                      daemon=True)
        cls.thread.start()
        cls.pump = threading.Thread(target=cls.session.run, daemon=True)
        cls.pump.start()

    @classmethod
    def tearDownClass(cls):
        cls.session.stopping.set()
        if cls.console.attached():
            cls.console.disconnect()
            time.sleep(.2)
        cls.httpd.shutdown()
        cls.httpd.server_close()

    def setUp(self):
        with self.session.lock:
            self.session.controller = None

    def call(self, path, payload=None, token="page-one", method=None):
        url = f"http://127.0.0.1:{self.port}{path}"
        data = None if payload is None else json.dumps(payload).encode()
        request = urllib.request.Request(url, data=data, method=method or ("POST" if data else "GET"))
        request.add_header("Content-Type", "application/json")
        if token:
            request.add_header("X-Session", token)
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                body = response.read().decode()
                return response.status, json.loads(body) if body.startswith(("{", "[")) else body
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read().decode())

    def test_static_page_and_assets_are_served(self):
        for path in ("/", "/index.html", "/styles.css", "/app.js"):
            status, body = self.call(path, token=None)
            self.assertEqual(status, 200, path)
            self.assertIn("H743", body if path != "/app.js" else body + "H743")

    def test_auxiliary_settings_persist_without_motion_and_respect_controller(self):
        import tempfile
        from host.auxiliary import AuxiliaryConfig
        original = self.console.aux_settings
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"aux.json"
                self.console.aux_settings = AuxiliaryConfig(path)
                config = self.console.aux_settings.snapshot()
                config["6"].update(min=-100, max=100, presets=[-20, 80])
                status, reply = self.call("/api/auxiliary/config", {"config": config})
                self.assertEqual((status, reply["ok"]), (200, True))
                self.assertEqual(AuxiliaryConfig(path).snapshot(), config)
                self.assertEqual(self.call("/api/state", token=None)[1]["aux_settings"], config)
                status, _ = self.call("/api/auxiliary/config", {"config": config}, token="page-two")
                self.assertEqual(status, 409)
                config["6"]["presets"][0] = -101
                status, reply = self.call("/api/auxiliary/config", {"config": config})
                self.assertEqual((status, reply["ok"]), (400, False))
                self.assertEqual(AuxiliaryConfig(path).snapshot()["6"]["presets"], [-20, 80])
        finally:
            self.console.aux_settings = original

    def test_automatic_policy_requires_controller_and_persists(self):
        """The real HTTP policy endpoint must preserve readonly-page permissions."""
        import tempfile
        from host.autolink import AutoDevices
        with tempfile.TemporaryDirectory() as directory:
            manager = AutoDevices(self.console, Path(directory) / 'links.json', port_provider=lambda: [])
            self.console.autolink = manager
            try:
                status, reply = self.call('/api/devices/automatic', {'enabled': True})
                self.assertEqual((status, reply['ok']), (200, True))
                status, reply = self.call('/api/devices/automatic', {'enabled': False}, token='page-two')
                self.assertEqual(status, 409)
                self.assertTrue(manager.config['enabled'])
                status, reply = self.call('/api/devices/automatic', {'enabled': 'false'})
                self.assertEqual(status, 400)
                status, reply = self.call('/api/devices/automatic', {'enabled': False})
                self.assertEqual((status, reply['ok']), (200, True))
                self.assertFalse(self.call('/api/state', token=None)[1]['auto_devices']['enabled'])
                self.assertFalse(AutoDevices(self.console, manager.path).config['enabled'])
            finally:
                self.console.autolink = None

    def test_state_then_connect_command_and_refused_overlong_body(self):
        status, state = self.call("/api/state", token=None)
        self.assertEqual(status, 200)
        self.assertIn("link", state)

        self.call("/api/connect", {"port": "SIM"})
        self.assertTrue(wait_for(lambda: self.console.device.get("fw")), "handshake")
        self.console.last_client = time.monotonic()

        status, reply = self.call("/api/command", {"body": "ARM 1"})
        self.assertEqual((status, reply["ok"]), (200, True))
        status, reply = self.call("/api/command", {"body": "SERVO 1 45"})
        self.assertEqual((status, reply["ok"]), (200, True))
        status, reply = self.call("/api/command", {"body": "SERVO 2 150"})
        self.assertEqual((status, reply["ok"]), (200, True))
        self.assertTrue(wait_for(lambda: "ch=1 angle=45.0" in self.console.servo.get("raw", "")
                                 and "ch=2 angle=150.0" in self.console.servo.get("raw", "")),
                        self.console.servo)

        status, reply = self.call("/api/command", {"body": "SERVO 1 999"})
        self.assertEqual(status, 200)
        self.assertTrue(reply["ok"], "the board answers ANGLE_RANGE; HTTP itself succeeded")
        self.assertTrue(wait_for(lambda: any("ANGLE_RANGE" in entry["text"] for entry in self.console.log)))

    def test_standalone_turn_http_modes_and_controller_guard(self):
        from unittest.mock import patch
        with self.session.lock:
            self.session.controller = "page-one"
        with patch.object(self.console, "turn_action", return_value=(True,"queued")) as action:
            status, reply = self.call("/api/turn/action", {"action":"start", "mode":"ABS", "angle":"-90.25", "sps":300, "accel":1000})
            self.assertEqual((status,reply["ok"]),(200,True))
            action.assert_called_once_with("start","ABS","-90.25",300,1000,20,None,None)
            action.reset_mock()
            status, reply = self.call('/api/turn/action', {'action':'configure','kp':'80','tolerance':'0.50','min_sps':350})
            self.assertEqual((status,reply['ok']),(200,True))
            action.assert_called_once_with('configure','REL',0,500,2000,'80','0.50',350)
            action.reset_mock()
            self.assertEqual(self.call("/api/turn/action", {"action":"stop"}, token="page-two")[0],409)
            self.assertEqual(self.call("/api/turn/action", {"action":"zero"}, token=None)[0],400)
            action.assert_not_called()

    def test_second_page_is_read_only_and_needs_a_token(self):
        with self.session.lock:
            self.session.controller = "page-one"
        try:
            status, reply = self.call("/api/command", {"body": "STOP"}, token="page-two")
            self.assertEqual(status, 409)
            self.assertIn("只读", reply["error"])
            status, reply = self.call("/api/command", {"body": "STOP"}, token=None)
            self.assertEqual(status, 400)
        finally:
            with self.session.lock:
                self.session.controller = None

    def test_session_http_heartbeat_claim_and_old_controller_refusal(self):
        """Use real HTTP permission endpoints with two registered browser streams."""
        self.session.join('page-one')
        self.session.join('page-two')
        try:
            self.assertEqual(self.call('/api/session/heartbeat', {})[1]['role'], 'controller')
            self.assertEqual(self.call('/api/session/heartbeat', {}, token='page-two')[1]['role'], 'readonly')
            self.assertEqual(self.call('/api/session/claim', {}, token=None)[0], 400)
            self.assertEqual(self.call('/api/session/claim', {}, token='unknown')[0], 409)
            status, reply = self.call('/api/session/claim', {}, token='page-two')
            self.assertEqual((status, reply['role']), (200, 'controller'))
            self.assertEqual(self.call('/api/command', {'body':'PING'})[0], 409)
            self.assertEqual(self.call('/api/session/heartbeat', {})[1]['role'], 'readonly')
        finally:
            self.session.leave('page-one')
            self.session.leave('page-two')

    def test_stream_delivers_hello_and_promotes_after_leave(self):
        import socket
        connection = socket.create_connection(("127.0.0.1", self.port), timeout=5)
        connection.sendall(b"GET /api/events?token=stream-test HTTP/1.1\r\n"
                           b"Host: 127.0.0.1\r\n\r\n")
        connection.settimeout(5)
        blob = b""
        deadline = time.monotonic() + 5
        while b'"type": "hello"' not in blob and time.monotonic() < deadline:
            blob += connection.recv(4096)
        self.assertIn(b"event-stream", blob)
        self.assertIn(b'"type": "hello"', blob)
        self.assertTrue(wait_for(lambda: self.session.clients), "the stream must register a client")
        connection.close()
        self.assertTrue(wait_for(lambda: not self.session.clients), "closing the page frees the slot")
        self.assertIsNone(self.session.controller)

    def test_live_page_revives_polling_after_console_has_been_idle(self):
        self.console.last_client = time.monotonic() - 100
        inbox, _ = self.session.join("late-page")
        try:
            self.assertTrue(wait_for(lambda: time.monotonic() - self.console.last_client < 1.))
        finally:
            self.session.leave("late-page")

    def test_stream_token_comes_from_the_query_and_owns_the_role(self):
        """EventSource cannot set headers; reading the query token is what lets a
        page both watch and command. Getting this wrong makes every page read-only.
        """
        import socket
        with self.session.lock:
            self.session.controller = None
        connection = socket.create_connection(("127.0.0.1", self.port), timeout=5)
        connection.sendall(b"GET /api/events?token=query-token HTTP/1.1\r\n"
                           b"Host: 127.0.0.1\r\n\r\n")
        connection.settimeout(5)
        blob = b""
        deadline = time.monotonic() + 5
        while b'"type": "hello"' not in blob and time.monotonic() < deadline:
            blob += connection.recv(4096)
        try:
            self.assertTrue(wait_for(lambda: "query-token" in self.session.clients),
                            f"stream token was not parsed: {list(self.session.clients)}")
            status, reply = self.call("/api/command", {"body": "PING"}, token="query-token")
            self.assertEqual((status, reply["ok"]), (200, True),
                             "the streaming page must be allowed to command")
        finally:
            connection.close()
        self.assertTrue(wait_for(lambda: not self.session.clients))

    def test_unknown_route_is_json_not_html(self):
        status, reply = self.call("/api/nope", {})
        self.assertEqual(status, 404)
        self.assertFalse(reply["ok"])


if __name__ == "__main__":
    unittest.main()
