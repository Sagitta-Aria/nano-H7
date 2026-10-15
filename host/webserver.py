"""HTTP + SSE front end for host.webapp.Console.

Deliberately dependency-free: one threading HTTP server, a static directory and
Server-Sent Events. No WebSocket library, no build step. The browser never talks
to the board directly — every byte still goes through the same serial worker,
protocol encoder and request replay cache the desktop console used.

Security boundary, stated plainly: the board's motion commands are on this HTTP
surface, so the server binds 127.0.0.1 by default. `--host 0.0.0.0` exposes motor
control to the whole network with no authentication; only do that on a link you
trust. Only one page may hold control at a time; a second one is read-only.
"""
import argparse
import json
import mimetypes
import os
import queue
import sys
import threading
import time
import urllib.parse
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .webapp import Console
from .camera import CameraSlots

STATIC = Path(__file__).resolve().parent / "static"
PAGE = STATIC / "index.html"
IDLE_EXIT_SECONDS = 20.   # all pages closed for this long -> release the board and exit
CONTROL_LEASE_SECONDS = 6.  # a silent controller hands the role to the next page
RESTART_EXIT_CODE = 75  # internal handoff from serve() to the CLI re-exec


class Session:
    """Fan-out hub: worker events in, one SSE stream per browser page out."""

    def __init__(self, console, exit_when_idle=False, control_lease=CONTROL_LEASE_SECONDS):
        self.console = console
        self.cameras = CameraSlots()
        self.camera = self.cameras.hubs["1"]  # legacy camera API is camera 1
        self.exit_when_idle = exit_when_idle
        self.control_lease = control_lease
        self.lock = threading.Lock()
        self.clients = {}      # token -> queue
        self.touched = {}      # token -> last time that page sent anything
        self.controller = None
        self.stopping = threading.Event()
        self.mutation_lock = threading.Lock()
        self.instance = uuid.uuid4().hex
        self.restart_requested = threading.Event()
        self.shutdown_server = None
        # Seeded from "now", never from zero: a monotonic clock starts at boot, so
        # zero would look like hours of idleness and stop the pump in its first
        # iteration, leaving every page connected but frozen.
        self.last_seen = time.monotonic()
        self.seen_page = False

    def join(self, token):
        inbox = queue.Queue(maxsize=512)
        with self.lock:
            self.clients[token] = inbox
            self.touched[token] = time.monotonic()
            if self.controller is None:
                self.controller = token
            first = self.controller == token
            self.last_seen = time.monotonic()
        return inbox, first

    def _roles_locked(self):
        """Send each registered page its own role; caller holds the session lock."""
        for token, inbox in self.clients.items():
            try:
                inbox.put_nowait({'type': 'role', 'role': 'controller' if token == self.controller else 'readonly'})
            except queue.Full:
                pass  # The pump removes blocked streams; heartbeat also returns the role.

    def leave(self, token, inbox=None):
        with self.lock:
            if inbox is not None and self.clients.get(token) is not inbox:
                return  # An old SSE closing cannot remove its replacement stream.
            self.clients.pop(token, None)
            self.touched.pop(token, None)
            if self.controller == token:
                # Promote the longest-waiting page so a reload keeps control.
                self.controller = next(iter(self.clients), None)
            self.last_seen = time.monotonic()
            self._roles_locked()

    def touch(self, token):
        with self.lock:
            if token in self.clients:
                self.touched[token] = time.monotonic()

    def release_stale_control(self, now=None):
        """Hand the role on if the controller has gone quiet.

        Without this, a page that vanishes without closing its socket (a killed
        browser, a suspended laptop) would keep the only write permission for the
        life of the server and every later page would be read-only.
        """
        now = now or time.monotonic()
        with self.lock:
            controller = self.controller
            idle = controller is not None and now - self.touched.get(controller, 0.) > self.control_lease
            if not idle or not self.clients:
                return False
            self.controller = next((token for token in self.clients if token != controller
                                    and now - self.touched.get(token, 0.) <= self.control_lease), None)
            promoted = self.controller
            self._roles_locked()
        if promoted != controller:
            self.broadcast({"type": "log", "entry": {
                "time": time.strftime("%H:%M:%S"), "kind": "warn",
                "text": "原控制页已停止响应，控制权已交给当前页面。"}})
        return True

    def heartbeat(self, token):
        """Renew a browser's lease using inbound traffic, never outbound telemetry."""
        self.touch(token)
        with self.lock:
            if self.controller is None and token in self.clients:
                self.controller = token
                self._roles_locked()
        self.release_stale_control()
        with self.lock:
            return 'controller' if self.controller == token else 'readonly'

    def claim(self, token):
        """Explicitly transfer control to a registered page, queuing STOP first."""
        with self.lock:
            if token not in self.clients:
                return False, '页面尚未连接，请刷新后重试。'
            if self.controller != token:
                if self.console.attached():
                    ok, message = self.console.submit('STOP')
                    if not ok:
                        return False, message
                self.controller = token
                self.touched[token] = time.monotonic()
                self._roles_locked()
        return True, 'controller'

    def is_controller(self, token):
        """True when this page may command the board.

        A page normally claims the role by opening the event stream. A command
        that arrives first (a reload can race its own stream) claims an empty
        role rather than being refused, so a single page is never locked out of
        its own board.
        """
        with self.lock:
            if self.controller is None:
                self.controller = token
                self.touched[token] = time.monotonic()
                return True
            if self.controller == token:
                self.touched[token] = time.monotonic()
                return True
            return False

    def broadcast(self, event):
        dead = []
        with self.lock:
            for token, inbox in self.clients.items():
                try:
                    inbox.put_nowait(event)
                except queue.Full:
                    dead.append(token)
        for token in dead:
            self.leave(token)

    def request_restart(self):
        """Accept a restart only while disarmed and idle; HTTP mutations are serialized."""
        if self.shutdown_server is None:
            return False, "当前启动方式不支持重启上位机。"
        with self.console.lock:
            frame = self.console.hello()
            if frame["armed"]:
                return False, "请先停止运动并取消电机使能，再重启上位机。"
            busy = (self.console.nav_pending or self.console.aux_pending or
                    frame["turn"].get("active") == "1" or
                    frame["navigation"].get("state") in
                    ("moving", "turning", "orienting", "waiting", "blocked") or
                    any(axis.get("active") == "1" or float(axis.get("sps", 0)) != 0
                        for axis in frame["axes"].values()))
            if busy:
                return False, "仍有运动或待执行任务，请先停止后再重启上位机。"
            self.restart_requested.set()
        return True, "正在重启上位机，页面将自动刷新。"

    def restart(self):
        """Stop the pump before graceful HTTP shutdown; serve() releases devices next."""
        self.stopping.set()
        self.shutdown_server()

    def run(self):
        """Drain board events, poll the board, and push to every page.

        A failure in here would silently freeze every page while the port stays
        open, so the loop reports it instead of dying quietly.
        """
        watchdog = time.monotonic()
        while not self.stopping.is_set():
            try:
                with self.lock:
                    watching = bool(self.clients)
                if watching:
                    self.console.last_client = time.monotonic()
                events = self.console.drain()
                if events:
                    for event in events:
                        self.broadcast(event)
                    self.broadcast({"type": "snapshot", "frames": self.console.hello()})
                self.console.tick()
                if self.console.autolink:
                    self.console.autolink.tick()
                self.release_stale_control()
                now = time.monotonic()
                # Liveness ping: proves the pipe is still writable so a vanished
                # page is dropped instead of quietly holding the control lock.
                if now - watchdog >= 1.:
                    watchdog = now
                    self.broadcast({"type": "ping", "t": int(now)})
                if self.exit_when_idle:
                    with self.lock:
                        watching = bool(self.clients)
                        if watching:
                            self.last_seen = now
                    # Only count time after the first page has been seen, so a
                    # server started alone does not exit before anyone opens it.
                    if not watching and self.console.device.get("_port"):
                        with self.lock:
                            idle = now - self.last_seen > IDLE_EXIT_SECONDS and self.seen_page
                    else:
                        idle = False
                        if watching:
                            self.seen_page = True
                    if idle:
                        self.stopping.set()
                        return
            except Exception as exc:  # noqa: BLE001 - the pump must never die silently
                self.console.note("error", f"推送循环异常（已继续）：{exc!r}")
                try:
                    self.broadcast({"type": "log", "entry": {
                        "time": time.strftime("%H:%M:%S"), "kind": "error",
                        "text": f"推送循环异常（已继续）：{exc!r}"}})
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(.2)
            time.sleep(.05)


class Handler(BaseHTTPRequestHandler):
    server_version = "h743-console"
    protocol_version = "HTTP/1.1"
    session = None

    def log_message(self, fmt, *args):
        pass  # The board log panel is the real log; stderr stays clean.

    # ---- helpers -----------------------------------------------------------

    def json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except (ValueError, UnicodeDecodeError):
            return {}

    def token(self, payload=None):
        """Resolve this page's session token.

        EventSource cannot set request headers, so the stream passes its token in
        the query string. Reading it here keeps the stream and the commands that
        follow it on the same identity — otherwise the stream would register an
        empty client and lock its own page out of control.
        """
        token = (self.headers.get("X-Session") or (payload or {}).get("token") or "").strip()
        if token:
            return token
        query = urllib.parse.urlparse(self.path).query
        return (urllib.parse.parse_qs(query).get("token") or [""])[0].strip()

    def guard(self, payload=None):
        """Mutations are refused for a second page or a missing session token."""
        token = self.token(payload)
        if not token:
            self.json({"ok": False, "error": "缺少会话令牌，请刷新页面。"}, 400)
            return None
        self.session.touch(token)
        if not self.session.is_controller(token):
            self.json({"ok": False,
                       "error": "另一个页面正在控制这块板；当前页面为只读。若那个页面已关闭，"
                                "几秒后会自动接管。"}, 409)
            return None
        return token

    # ---- routes ------------------------------------------------------------

    def camera_route(self, path):
        """Keep /api/camera for camera 1; /api/camera2 owns a separate hub."""
        for prefix, slot in (("/api/camera/", "1"), ("/api/camera2/", "2")):
            if path.startswith(prefix):
                return slot, path[len(prefix):]
        return None, None

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        slot, action = self.camera_route(path)
        if slot:
            hub = self.session.cameras.hubs[slot]
            if action == "devices":
                self.json(self.session.cameras.listing(slot))
            elif action == "state":
                self.json(hub.snapshot())
            elif action == "frame":
                data = hub.image()
                if not data:
                    self.json({"error": "摄像头尚未出图。"}, 503)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
            else:
                self.json({"error": "未知摄像头操作。"}, 404)
            return
        if path == "/api/state":
            self.json(self.session.console.hello())
        elif path == "/api/host/state":
            self.json({"instance": self.session.instance,
                       "restarting": self.session.restart_requested.is_set()})
        elif path == "/api/ports":
            self.json(self.session.console.ports())
        elif path == "/api/imu/ports":
            self.json(self.session.console.imu.ports())
        elif path == "/api/imu/state":
            self.json(self.session.console.imu.snapshot(history=True))
        elif path == "/api/events":
            self.stream()
        else:
            self.static(path)

    def do_POST(self):
        # No new command/config write may race an accepted service restart.
        with self.session.mutation_lock:
            if self.session.restart_requested.is_set():
                self.json({"ok": False, "error": "上位机正在重启，请稍候。"}, 503)
                return
            self.post_route()

    def post_route(self):
        """Dispatch writes under the session mutation lock, including restart."""
        path = self.path.split("?", 1)[0]
        payload = self.body()
        if path == "/api/host/restart":
            if self.guard(payload) is None:
                return
            ok, message = self.session.request_restart()
            if not ok:
                self.json({"ok": False, "error": message}, 409)
                return
            try:
                self.json({"ok": True, "detail": message, "instance": self.session.instance}, 202)
                self.wfile.flush()
            finally:
                # shutdown() must run outside the serve_forever thread, after
                # the response has been sent even if the requesting page closes.
                threading.Timer(.2, self.session.restart).start()
            return
        slot, action = self.camera_route(path)
        if slot:
            if self.guard(payload) is None:
                return
            try:
                hub = self.session.cameras.hubs[slot]
                if action == "start":
                    frame = self.session.cameras.start(slot, payload.get("device"), payload.get("resolution"))
                elif action == "snapshot":
                    frame = hub.capture()
                elif action == "stop":
                    frame = self.session.cameras.stop(slot)
                elif action == "bind":
                    binding = self.session.cameras.bind(slot, payload.get("device"))
                    self.json({"ok": True, "binding": binding})
                    return
                else:
                    self.json({"ok": False, "error": "未知摄像头操作。"}, 404)
                    return
            except (ValueError, OSError) as error:
                self.json({"ok": False, "error": str(error)}, 400)
                return
            self.json({"ok": True, "camera": frame})
            return
        if path in ('/api/session/heartbeat', '/api/session/claim'):
            token = self.token(payload)
            if not token:
                self.json({'ok': False, 'error': '缺少会话令牌，请刷新页面。'}, 400)
                return
            if path.endswith('/heartbeat'):
                self.json({'ok': True, 'role': self.session.heartbeat(token)})
            else:
                ok, message = self.session.claim(token)
                self.json({'ok': ok, 'role': 'controller' if ok else 'readonly',
                           'error': None if ok else message}, 200 if ok else 409)
        elif path == "/api/devices/automatic":
            if self.guard(payload) is None:
                return
            if not self.session.console.autolink or not isinstance(payload.get("enabled"), bool):
                self.json({"ok": False, "error": "自动连接未启用或参数无效。"}, 400)
                return
            self.session.console.autolink.set_enabled(payload['enabled'])
            self.json({"ok": True, "automatic": self.session.console.autolink.snapshot()})
        elif path in ("/api/imu/connect", "/api/imu/disconnect"):
            if self.guard(payload) is None:
                return
            if path.endswith("/connect"):
                ok, message = self.session.console.connect_imu(str(payload.get("port") or ""))
            else:
                self.session.console.imu.close()
                ok, message = True, "offline"
            self.json({"ok": ok, "state": message, "error": None if ok else message})
        elif path == "/api/connect":
            port = str(payload.get("port") or "SIM")
            if self.session.console.attached():
                ok, message = self.session.console.disconnect()
                self.json({"ok": ok, "state": message})
                return
            ok, message = self.session.console.connect(port, payload.get("ack"))
            self.json({"ok": ok, "state": message, "error": None if ok else message})
        elif path == "/api/disconnect":
            ok, message = self.session.console.disconnect()
            self.json({"ok": ok, "state": message})
        elif path == "/api/command":
            if self.guard(payload) is None:
                return
            ok, message = self.session.console.submit(payload.get("body", ""), payload.get("ack"))
            self.json({"ok": ok, "detail": message})
        elif path == "/api/jog":
            if self.guard(payload) is None:
                return
            ok, message = self.session.console.jog(payload.get("body"))
            self.json({"ok": ok, "detail": message})
        elif path == "/api/auxiliary/config":
            if self.guard(payload) is None:
                return
            try:
                config = self.session.console.save_auxiliary(payload.get("config"))
            except (ValueError, OSError) as error:
                self.json({"ok": False, "error": str(error)}, 400)
                return
            self.session.broadcast({"type": "aux_settings", "config": config})
            self.json({"ok": True, "config": config})
        elif path == "/api/auxiliary/apply":
            if self.guard(payload) is None:
                return
            ok, message = self.session.console.apply_auxiliary(payload.get('config'), payload.get('axis'))
            self.json({'ok':ok,'detail':message,'error':None if ok else message})
        elif path == "/api/navigation/config":
            if self.guard(payload) is None:
                return
            try:
                config = self.session.console.save_navigation(payload.get("config"))
            except (ValueError, OSError) as error:
                self.json({"ok": False, "error": str(error)}, 400)
                return
            self.session.broadcast({"type": "mission", "config": config})
            self.json({"ok": True, "config": config})
        elif path == "/api/flow/config":
            if self.guard(payload) is None:
                return
            try:
                config = self.session.console.save_flow(payload.get("config"))
            except (ValueError, OSError) as error:
                self.json({"ok": False, "error": str(error)}, 400)
                return
            self.session.broadcast({"type": "flow_settings", "config": config})
            self.json({"ok": True, "config": config})
        elif path == "/api/flow/action":
            if self.guard(payload) is None:
                return
            ok, message = self.session.console.flow_action(payload.get("action"))
            self.json({"ok": ok, "detail": message, "error": None if ok else message})
        elif path == "/api/turn/action":
            if self.guard(payload) is None:
                return
            ok, message = self.session.console.turn_action(payload.get("action"), payload.get("mode", "REL"),
                payload.get("angle", 0), payload.get("sps", 500), payload.get("accel", 2000), payload.get("kp", 20), payload.get("tolerance"))
            self.json({"ok": ok, "detail": message, "error": None if ok else message})
        elif path == "/api/navigation/action":
            if self.guard(payload) is None:
                return
            ok, message = self.session.console.navigation_action(payload.get("action"))
            self.json({"ok": ok, "detail": message, "error": None if ok else message})
        else:
            self.json({"ok": False, "error": "未知接口"}, 404)

    def static(self, path):
        relative = "index.html" if path in ("/", "") else path.lstrip("/")
        target = (STATIC / relative).resolve()
        if STATIC not in target.parents and target != STATIC:
            self.json({"ok": False, "error": "越界路径"}, 403)
            return
        if not target.is_file():
            target = PAGE
        data = target.read_bytes()
        kind = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        if kind.startswith("text/") or kind in ("application/javascript", "image/svg+xml"):
            kind += "; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    # ---- Server-Sent Events ------------------------------------------------

    def stream(self):
        token = self.token()
        inbox, first = self.session.join(token)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            self.emit({"type": "role", "role": "controller" if first else "readonly"})
            self.emit(self.session.console.hello())
            while not self.session.stopping.is_set():
                try:
                    event = inbox.get(timeout=1.)
                except queue.Empty:
                    event = {"type": "ping", "t": int(time.time())}
                self.emit(event)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.session.leave(token, inbox)
            self.close_connection = True

    def emit(self, event):
        blob = json.dumps(event, ensure_ascii=False)
        self.wfile.write(f"data: {blob}\n\n".encode())
        self.wfile.flush()

    def handle_one_request(self):
        """Detect a closed page during a quiet SSE stream instead of hanging."""
        try:
            BaseHTTPRequestHandler.handle_one_request(self)
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.close_connection = True


def say(text):
    """Print without assuming the console code page can encode ° or µ."""
    encoding = getattr(sys.stdout, "encoding", None) or "utf-8"
    sys.stdout.write(text.encode(encoding, "replace").decode(encoding) + "\n")
    sys.stdout.flush()


def serve(host="127.0.0.1", port=8770, open_browser=True, exit_when_idle=False,
          demo=False, connect=None, auto_devices=False):
    if auto_devices and (demo or connect):
        raise ValueError("--auto-devices cannot be combined with --demo or --connect")
    console = Console()
    if auto_devices:
        from .autolink import AutoDevices
        console.autolink = AutoDevices(console)
    session = Session(console, exit_when_idle=exit_when_idle)
    handler = type("BoundHandler", (Handler,), {"session": session})
    httpd = ThreadingHTTPServer((host, port), handler)
    httpd.daemon_threads = True
    session.shutdown_server = httpd.shutdown
    url = f"http://{'127.0.0.1' if host in ('0.0.0.0', '::') else host}:{httpd.server_address[1]}/"
    console.note("info", f"Web 控制台已启动：{url}")
    if demo:
        console.connect("SIM")
    elif connect:
        console.connect(connect)
    pump = threading.Thread(target=session.run, name="h743-web-pump", daemon=True)
    pump.start()
    if open_browser:
        threading.Thread(target=lambda: (time.sleep(.4), webbrowser.open(url)), daemon=True).start()
    say(f"H743 Web 控制台: {url}")
    say(f"绑定 {host}；Ctrl+C 退出。板卡连接独立于页面，关闭页面只是停止轮询。")
    try:
        httpd.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        print("\n收到中断，正在断开板卡…")
    finally:
        session.stopping.set()
        pump.join(timeout=2.)
        session.cameras.close()
        console.imu.close()
        if console.attached():
            console.disconnect()
            deadline = time.monotonic() + 2.
            while console.attached() and time.monotonic() < deadline:
                console.drain()
                time.sleep(.05)
        httpd.server_close()
    return RESTART_EXIT_CODE if session.restart_requested.is_set() else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1",
                        help="listen address; 0.0.0.0 exposes motor control to the network (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8770, help="listen port (default: 8770)")
    parser.add_argument("--demo", action="store_true", help="connect the simulated board immediately")
    parser.add_argument("--connect", metavar="PORT", help="open this serial port immediately")
    parser.add_argument("--auto-devices", action="store_true",
                        help="automatically identify and reconnect H743/H723 USB devices (Nano)")
    parser.add_argument("--no-browser", action="store_true", help="do not open a browser window")
    parser.add_argument("--exit-on-close", action="store_true",
                        help="exit once every page has been closed for a while")
    args = parser.parse_args(argv)
    if args.auto_devices and (args.demo or args.connect):
        parser.error("--auto-devices cannot be combined with --demo or --connect")
    result = serve(args.host, args.port, not args.no_browser, args.exit_on_close,
                   args.demo, args.connect, args.auto_devices)
    if result == RESTART_EXIT_CODE:
        # Preserve CLI options and the service PID. All serial/camera handles
        # are closed before replacing this process; saved JSON stays on disk.
        os.execv(sys.executable, [sys.executable, "-u", "-m", "host.webserver", *sys.argv[1:]])
    return result


if __name__ == "__main__":
    raise SystemExit(main())
