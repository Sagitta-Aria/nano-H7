"""Exercise restart authorization, motion guards and the actual HTTP lifecycle."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from unittest.mock import Mock

from host.webserver import Handler, Session
from http.server import ThreadingHTTPServer


class IdleConsole:
    """Minimal telemetry source for HTTP tests; never opens a hardware port."""

    def __init__(self):
        self.lock = threading.RLock()
        self.nav_pending = False
        self.aux_pending = {}
        self.frame = {"armed": False, "turn": {}, "navigation": {}, "axes": {}}

    def hello(self):
        return self.frame


class RestartHTTP(unittest.TestCase):
    def setUp(self):
        self.console = IdleConsole()
        self.session = Session(self.console)
        self.session.shutdown_server = Mock()
        self.session.controller = "owner"
        handler = type("RestartHandler", (Handler,), {"session": self.session})
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:{}".format(self.server.server_port)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def post(self, path="/api/host/restart", token="owner"):
        request = urllib.request.Request(self.url + path,
            data=json.dumps({"token": token}).encode(),
            headers={"Content-Type": "application/json"})
        try:
            response = urllib.request.urlopen(request, timeout=3)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.status, json.load(response)

    def test_readonly_or_missing_token_cannot_restart(self):
        for token, expected in (("viewer", 409), ("", 400)):
            with self.subTest(token=token):
                code, result = self.post(token=token)
                self.assertEqual(code, expected)
                self.assertFalse(result["ok"])
        self.assertFalse(self.session.restart_requested.is_set())
        self.session.shutdown_server.assert_not_called()

    def test_armed_or_active_tasks_cannot_restart(self):
        cases = [
            {"armed": True},
            {"axes": {"5": {"active": "1", "sps": "0"}}},
            {"axes": {"1": {"active": "0", "sps": "500"}}},
            {"turn": {"active": "1"}},
            {"navigation": {"state": "waiting"}},
        ]
        idle = dict(self.console.frame)
        for fields in cases:
            with self.subTest(fields=fields):
                self.console.frame = dict(idle, **fields)
                self.assertEqual(self.post()[0], 409)
                self.assertFalse(self.session.restart_requested.is_set())
        self.console.frame = idle
        self.console.nav_pending = True
        self.assertEqual(self.post()[0], 409)
        self.console.nav_pending = False
        self.console.aux_pending = {"5": {"pending": True}}
        self.assertEqual(self.post()[0], 409)
        self.session.shutdown_server.assert_not_called()

    def test_unsupported_launch_is_reported(self):
        self.session.shutdown_server = None
        self.assertEqual(self.post()[0], 409)
        self.assertFalse(self.session.restart_requested.is_set())

    def test_accepted_restart_returns_identity_and_blocks_further_writes(self):
        code, result = self.post()
        self.assertEqual(code, 202)
        self.assertTrue(result["ok"])
        self.assertEqual(result["instance"], self.session.instance)
        for path in ("/api/host/restart", "/api/command", "/api/connect", "/api/navigation/config"):
            self.assertEqual(self.post(path)[0], 503)
        with urllib.request.urlopen(self.url + "/api/host/state") as response:
            self.assertTrue(json.load(response)["restarting"])
        deadline = time.monotonic() + 2
        while not self.session.stopping.is_set() and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertTrue(self.session.stopping.is_set())
        self.session.shutdown_server.assert_called_once()


@unittest.skipUnless(sys.platform.startswith("linux"), "Real process replacement is verified on Nano Linux")
class RestartProcess(unittest.TestCase):
    def test_demo_service_restarts_and_reconnects_without_arming(self):
        # The independent SIM process owns no real serial ports or saved settings.
        with socket.socket() as reserved:
            reserved.bind(("127.0.0.1", 0))
            port = reserved.getsockname()[1]
        process = subprocess.Popen([sys.executable, "-u", "-m", "host.webserver",
            "--port", str(port), "--no-browser", "--demo"],
            cwd=str(Path(__file__).resolve().parents[1]),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        url = "http://127.0.0.1:{}".format(port)

        def wait_json(path, predicate):
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                try:
                    with urllib.request.urlopen(url + path, timeout=1) as response:
                        result = json.load(response)
                    if predicate(result):
                        return result
                except (OSError, ValueError):
                    pass
                time.sleep(.1)
            self.fail("Service did not recover: " + path)

        try:
            old = wait_json("/api/host/state", lambda frame: bool(frame.get("instance")))
            wait_json("/api/state", lambda frame: bool(frame.get("device", {}).get("fw")))
            request = urllib.request.Request(url + "/api/host/restart",
                data=b'{"token":"sim-test"}', headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=3) as response:
                self.assertEqual(response.status, 202)
            new = wait_json("/api/host/state", lambda frame: frame.get("instance") != old["instance"])
            self.assertFalse(new["restarting"])
            frame = wait_json("/api/state", lambda frame: bool(frame.get("device", {}).get("fw")))
            self.assertEqual(frame["port"], "SIM")
            self.assertFalse(frame["armed"])
            self.assertIsNone(process.poll(), "exec must preserve the service process")
            os.kill(process.pid, 0)
        finally:
            process.terminate()
            process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
