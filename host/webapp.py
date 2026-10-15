"""Board session for the web console: the same worker, protocol and safety
rules as the Tk console, with no Tk import.

One serial worker owns all I/O (host.transport.Connection). This class is the
only other thread that talks to it: it drains the worker's event queue, runs the
same 1 s STAT / 200 ms AXIS / SERVO STAT polling the desktop console ran, and
keeps the last good snapshot so a newly opened page sees state immediately.

Safety rules that must not drift from the firmware:
  * ARM is refused without a recent PING, and the worker sends PING every 250 ms
    while connected, so the board never relies on this class for its lease.
  * STOP clears pending requests and takes queue priority inside the worker.
  * Real serial ports need an explicit wiring acknowledgement before arming.
  * A rejected command is reported, never silently retried with new parameters.
"""
import queue
import threading
import time

from .transport import Connection, list_ports
from .imu import ImuHub
from .navigation import MissionConfig, CONFIG_PATH
from .turn import turn_command, gain_command, tuning_command
from .straight import straight_command
from .optical_flow import FlowSettings, CONFIG_PATH as FLOW_CONFIG_PATH, config_command
from .auxiliary import AuxiliaryConfig, validated as validated_auxiliary, CONFIG_PATH as AUX_CONFIG_PATH

STALE_AFTER = 30.0       # page heartbeat: no poll for this long means nobody is watching
POLL_AXES_PER_TICK = 2   # a full seven-axis sweep takes ~0.7 s, as in the Tk console
SERVO_POLL_PERIOD = .5
CHASSIS_POLL_PERIOD = .5


class Console:
    def __init__(self, navigation_path=CONFIG_PATH, auxiliary_path=AUX_CONFIG_PATH, optical_path=FLOW_CONFIG_PATH):
        self.lock = threading.RLock()
        self.events = queue.Queue()
        self.connection = None
        self.imu = ImuHub()
        self.autolink = None
        self.armed = False
        self.device = {}      # last reply fields by command, for late-joining pages
        self.axes = {}        # axis number -> fields
        self.servo = {}
        self.chassis = {}
        self.lift = {}
        self.auxiliary = {}
        self.aux_settings = AuxiliaryConfig(auxiliary_path)
        self.last_auxiliary = 0.
        self.log = []
        self.last_poll = 0.
        self.last_stat = 0.
        self.last_servo = 0.
        self.last_chassis = 0.
        self.last_lift = 0.
        self.axis_poll = 1
        self.last_client = 0.
        self._order = 0
        self.mission = MissionConfig(navigation_path)
        self.navigation = {}
        self.last_navigation = 0.
        self.turn = {}
        self.last_turn = 0.
        self.flow = {}
        self.flow_settings = FlowSettings(optical_path)
        self.last_flow = 0.
        self.nav_upload = []
        self.nav_generation = 0
        self.nav_pending = False
        self.aux_pending = {}
        self.aux_generation = 0

    # ---- serial connection -------------------------------------------------

    def ports(self):
        found = [{"id": "SIM", "label": "模拟板（无需硬件）", "description": "无硬件演示"}]
        for device, description in list_ports():
            found.append({"id": device, "label": device, "description": description})
        return {"ports": found, "current": self.device.get("_port")}

    def last_port(self):
        return self.device.get("_port")

    def connect(self, port, acknowledge=False, expected_board=None, expected_uid=None):
        """Start the worker. Waiting for HELLO happens in the worker itself."""
        with self.lock:
            if self.imu.thread and self.imu.thread.is_alive() and self.imu.port == port:
                return False, "该端口正在由 H723 陀螺仪占用。"
            if self.connection:
                return False, "已经连接；请先断开。"
            self.events = queue.Queue()
            self.device = {"_port": port, "_ack": bool(acknowledge)}
            self.axes, self.servo, self.armed = {}, {}, False
            self.lift = {}
            self.auxiliary = {}
            self.aux_pending.clear()
            self.navigation = {}
            self.turn = {}
            self.flow = {}
            self.nav_upload = []
            self.nav_pending = False
            self.nav_generation += 1
            self.connection = Connection(port, self.events, expected_board=expected_board,
                                         expected_uid=expected_uid, imu_provider=self.imu.navigation_feedback)
            self.connection.start()
            self.last_client = time.monotonic()
        self.note("connect", f"正在连接 {'模拟板' if port == 'SIM' else port}…")
        return True, "connecting"

    def disconnect(self):
        with self.lock:
            if not self.connection:
                return False, "尚未连接。"
            self.connection.close()
            self.aux_pending.clear()
            self.nav_upload = []
            self.nav_pending = False
            self.nav_generation += 1
        self.note("connect", "正在断开，发送 STOP…")
        return True, "closing"

    def attached(self):
        with self.lock:
            return self.connection is not None

    # ---- commands ----------------------------------------------------------

    def submit(self, body, acknowledge=False):
        """Queue one protocol body. STOP bypasses every guard except "connected"."""
        body = " ".join(str(body).split())
        if not body or len(body) >= 128 or "*" in body or not body.isascii():
            return False, "命令必须是不含 * 的 ASCII 正文，且短于 128 字节。"
        with self.lock:
            if body.startswith("CHASSIS STRAIGHT "):
                if self.device.get("straight") != "1":
                    return False, "陀螺仪辅助直线需要 H743 0.2.17 固件。"
                parts = body.split()
                try:
                    if len(parts) != 6:
                        raise ValueError("陀螺仪辅助直线参数数量错误。")
                    body = straight_command(parts[2], *(int(value) for value in parts[3:]))
                except ValueError as exc:
                    return False, str(exc)
            if body in ("STOP", "CHASSIS HALT", "NAV STOP", "ARM 0"):
                self.nav_upload = []
                self.nav_pending = False
                self.nav_generation += 1
                self.aux_pending.clear()
            elif self.nav_pending and (body.startswith(("MOVE ", "WHEELS ", "CHASSIS ", "NAV ", "TURN ")) and
                                       body not in ("NAV STAT", "CHASSIS STAT", "TURN STAT", "TURN STOP")):
                return False, "正在下发巡航，请等待完成或先停止。"
            connection = self.connection
            # Read the port from the console's own record, not from the worker:
            # a worker that has already fallen back to the simulator would
            # otherwise report "SIM" and silently skip this guard.
            real = connection is not None and self.device.get("_port") not in (None, "SIM")
            acknowledged = acknowledge or self.device.get("_ack")
        if connection is None:
            return False, "请先连接模拟板或真实串口。"
        if body == "ARM 1" and real and not acknowledged:
            return False, "真实板使能前请先勾选“已核对驱动器接线”。"
        if not connection.submit(body):
            return False, "发送队列已满，请稍后再试。"
        return True, body

    def set_wiring_ack(self, value):
        with self.lock:
            self.device["_ack"] = bool(value)
        return True

    def navigation_action(self, action):
        """Queue origin or an acknowledged MCU upload; never auto-arm/reset."""
        with self.lock:
            if not self.connection or self.device.get("nav_heading") != "1":
                return False, "请连接支持陀螺仪巡航的 0.2.9 固件。"
            if action == "origin":
                if self.armed or self.nav_pending:
                    return False, "建立 O 点前请解除使能并停止巡航。"
                x, y = self.mission.snapshot()["points"]["O"]
                return self.submit("NAV RESET {} {}".format(x, y))
            if action == "start":
                if self.device.get("nav_grid") != "1" or self.device.get("nav_path") != "1":
                    return False, "完整路径上传需要 0.2.14 固件，请先更新 H743。"
                if self.nav_pending or not self.armed or self.navigation.get("ready") != "1":
                    return False, "请先建立 O 点并使能，或等待当前下发结束。"
                if self.navigation.get("state") in ("turning", "moving", "orienting", "waiting", "blocked") or self.turn.get("state") == "turning":
                    return False, "请先停止当前巡航。"
                if self.device.get("_port") != "SIM" and not self.imu.navigation_feedback()["valid"]:
                    return False, "请先连接 H723 并等待校准完成及新航向数据。"
                self.nav_generation += 1
                self.nav_pending = True
                self.nav_upload = ["NAV STAT"]
                return self._navigation_next()
            if action == "stop":
                return self.submit("NAV STOP")
            return False, "未知巡航动作。"

    def _navigation_next(self):
        if not self.nav_upload:
            return True, "uploaded"
        body = self.nav_upload.pop(0)
        if not self.connection.submit(body, tag=("navigation", self.nav_generation)):
            self.nav_upload = []
            self.nav_pending = False
            return False, "发送队列已满，巡航未启动。"
        return True, body

    def turn_action(self, action, mode="REL", angle=0, sps=500, accel=2000, kp=20, tolerance=None):
        """Queue standalone yaw actions with capability/freshness checks.

        No route upload, XY reset or implicit arming. MCU rechecks ownership;
        stale page telemetry can never authorize overlapping wheel motion.
        """
        with self.lock:
            if not self.connection or self.device.get("turn") != "1":
                return False, "请连接支持闭环转角的 0.2.10 固件。"
            if action == "stop":
                return self.submit("TURN STOP")
            if action not in ("zero", "start", "configure"):
                return False, "未知转角动作。"
            if self.nav_pending or self.navigation.get("state") in ("turning", "moving", "orienting", "waiting", "blocked") or \
                    self.chassis.get("state") in ("moving", "jogging", "switching") or self.turn.get("state") == "turning":
                return False, "请先停止底盘运动或坐标巡航。"
            if action == "configure":
                if self.device.get("turn_tuning") != "1":
                    return False, "可调 P 需要 0.2.15 固件。"
                try:
                    if tolerance is not None:
                        if self.device.get('turn_precision') != '1':
                            return False, '可调角度容差需要 0.2.16 固件。'
                        return self.submit(tuning_command(kp, tolerance))
                    return self.submit(gain_command(kp))
                except ValueError as error:
                    return False, str(error)
            if self.device.get("_port") != "SIM" and not self.imu.navigation_feedback()["valid"]:
                return False, "请先连接 H723 并等待校准完成及新航向数据。"
            if action == "zero":
                return self.submit("TURN ZERO")
            if not self.armed:
                return False, "请先使能输出。"
            try:
                body = turn_command(mode, angle, sps, accel)
            except ValueError as error:
                return False, str(error)
            if mode == "ABS" and self.turn.get("zero") != "1":
                return False, "请先将当前方向设为转角零点。"
            return self.submit(body)

    def save_navigation(self, config):
        with self.lock:
            return self.mission.save(config)

    def save_flow(self, config):
        """Persist only; the explicit apply action changes future board increments."""
        with self.lock:
            return self.flow_settings.save(config)

    def flow_action(self, action):
        """Queue MCU-owned recording or saved calibration without resetting totals."""
        with self.lock:
            if not self.connection or self.device.get("flow") != "1":
                return False, "请连接支持光流的 0.2.11 固件。"
            if action == "apply":
                body = config_command(self.flow_settings.snapshot())
            elif action in ("start", "stop"):
                body = "FLOW " + action.upper()
            else:
                return False, "未知光流动作。"
            return self.submit(body)

    def save_auxiliary(self, config):
        """Persist settings only; explicit CONFIG commands apply board limits."""
        with self.lock:
            return self.aux_settings.save(config)

    def apply_auxiliary(self, config, axis):
        """Apply one axis; persist only after matching MCU limits are confirmed."""
        axis = str(axis)
        with self.lock:
            if axis not in ('5', '6'):
                return False, "请选择 M5 或 M6。"
            try:
                value = validated_auxiliary(config)[axis]
            except ValueError as error:
                return False, str(error)
            f = self.auxiliary.get(axis, {})
            if not self.connection or self.device.get('aux_control') != '1':
                return False, "请先连接辅助轴控制板。"
            if self.armed and self.device.get('aux_live') != '1':
                return False, "使能状态下修改范围需要 0.2.15 固件；旧固件须先解除使能。"
            if self.nav_pending or axis in self.aux_pending or f.get('active') != '0' or f.get('referenced') != '1':
                return False, "请先建立零位并停止该轴，或等待上次应用完成。"
            if not value['min'] <= int(f.get('pos', 0)) <= value['max']:
                return False, "新范围必须包含板端当前位置。"
            self.aux_generation += 1
            tag = ('auxiliary_apply', axis, self.aux_generation)
            body = 'AUX CONFIG {} {} {}'.format(axis, value['min'], value['max'])
            if not self.connection.submit(body, tag=tag):
                return False, "发送队列已满，范围未应用。"
            self.aux_pending[axis] = (tag, value)
            return True, "等待板端确认 M{} 范围。".format(axis)

    def jog(self, body):
        """Latest-only intent for held keys/buttons; the worker expires it."""
        with self.lock:
            if body and self.nav_pending:
                return False, "正在下发巡航，请等待完成或先停止。"
            connection = self.connection
        if connection is None:
            return False, "请先连接。"
        if body is None:
            connection.set_jog(None)
        else:
            connection.set_jog(" ".join(str(body).split()))
        return True, body

    # ---- polling loop ------------------------------------------------------

    def tick(self):
        """Free function for the caller's loop; kept separate for testability."""
        with self.lock:
            watcher = time.monotonic() - self.last_client < STALE_AFTER
            connection = self.connection
        if connection is None or not watcher or connection.commands.qsize() >= 3:
            return
        now = time.monotonic()
        if now - self.last_stat >= 1.:
            self._send("STAT")
            self.last_stat = now
        elif now - self.last_poll >= .2:
            for _ in range(POLL_AXES_PER_TICK):
                self._send(f"AXIS {self.axis_poll}")
                self.axis_poll = self.axis_poll % 7 + 1
            self.last_poll = now
        if now - self.last_servo >= SERVO_POLL_PERIOD:
            self._send("SERVO STAT")
            self.last_servo = now
        if now - self.last_chassis >= CHASSIS_POLL_PERIOD:
            self._send("CHASSIS STAT")
            self.last_chassis = now
        if self.device.get("lift") == "1" and now - self.last_lift >= .25:
            self._send("LIFT STAT")
            self.last_lift = now
        if self.device.get("aux_control") == "1" and now - self.last_auxiliary >= .25:
            self._send("AUX STAT 5")
            self._send("AUX STAT 6")
            self.last_auxiliary = now
        if self.device.get("nav") == "1" and now - self.last_navigation >= .25:
            self._send("NAV STAT")
            self.last_navigation = now
        if self.device.get("turn") == "1" and now - self.last_turn >= .25:
            self._send("TURN STAT")
            self.last_turn = now
        if self.device.get("flow") == "1" and now - self.last_flow >= .3:
            for body in ("FLOW STAT", "FLOW WORLD", "FLOW CONFIG"):
                self._send(body)
            self.last_flow = now

    def _send(self, body):
        with self.lock:
            connection = self.connection
        if connection is not None:
            connection.submit(body)

    def note(self, kind, text):
        """Server-originated line for the log panel."""
        with self.lock:
            self._order += 1
            entry = {"seq": self._order, "time": time.strftime("%H:%M:%S"),
                     "kind": kind, "text": text}
            self.log.append(entry)
            del self.log[:-400]
            self.events.put(("log", entry))

    # ---- event drain -------------------------------------------------------

    def drain(self, limit=64):
        """Pop worker events, mirror state, and return page events."""
        out = []
        while len(out) < limit:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            out.extend(self._absorb(kind, payload))
        out.extend(self.imu.drain())
        return out

    def _absorb(self, kind, payload):
        if kind == "reply":
            return self._reply(*payload)
        if kind == "connected":
            with self.lock:
                self.device["_serial"] = payload
            return [{"type": "link", "state": "connected", "port": payload},
                    {"type": "device", "fields": {k: v for k, v in self.device.items()
                                                  if not k.startswith("_")}},
                    {"type": "log", "entry": {"time": time.strftime("%H:%M:%S"), "kind": "ok",
                                              "text": f"连接成功：{payload}"}}]
        if kind == "disconnected":
            with self.lock:
                self.connection = None
                self.armed = False
                self.nav_upload = []
                self.nav_pending = False
                self.nav_generation += 1
                self.aux_pending.clear()
            return [{"type": "link", "state": "offline", "port": payload},
                    {"type": "log", "entry": {"time": time.strftime("%H:%M:%S"), "kind": "warn",
                                              "text": "连接已关闭；真实板心跳租约将超时失效。"}}]
        if kind in ("error", "warning"):
            return [{"type": "log", "entry": {"time": time.strftime("%H:%M:%S"),
                                              "kind": "error" if kind == "error" else "warn",
                                              "text": str(payload)}}]
        return []

    def _reply(self, command, reply, tag=None):
        extra = []
        with self.lock:
            if tag and tag[0] == 'auxiliary_apply' and self.aux_pending.get(tag[1], (None,))[0] == tag:
                _, value = self.aux_pending.pop(tag[1])
                fields = reply.fields
                if reply.ok and fields.get('axis') == tag[1] and fields.get('min') == str(value['min']) and fields.get('max') == str(value['max']):
                    config = self.aux_settings.snapshot()
                    config[tag[1]] = value
                    try:
                        saved = self.aux_settings.save(config)
                        extra.append({'type':'aux_settings','config':saved})
                        extra.append({'type':'status','text':'M{} 范围已由板端确认并保存。'.format(tag[1])})
                    except (OSError, ValueError) as error:
                        self.note('error', '板端范围已生效，但保存失败：' + str(error))
                else:
                    extra.append({'type':'status','text':'M{} 范围未应用：{}'.format(tag[1],reply.body)})
            if tag == ("navigation", self.nav_generation):
                if reply.ok:
                    if command == "NAV STAT":
                        try:
                            fields = reply.fields
                            if fields.get("ready") != "1" or fields.get("state") in ("turning", "moving", "orienting", "waiting", "blocked"):
                                raise ValueError("坐标未建立或车辆仍在运动。")
                            self.nav_upload = self.mission.commands(
                                (int(fields["x"]), int(fields["y"])),
                                self.nav_generation % 2147483647 + 1)
                        except (ValueError, KeyError) as error:
                            self.nav_upload = []
                            self.nav_pending = False
                            self.note("error", "路径规划失败：" + str(error))
                    if command.startswith("NAV START"):
                        self.nav_pending = False
                    ok, message = self._navigation_next()
                    if not ok:
                        self.note("error", message)
                else:
                    self.nav_upload = []
                    self.nav_pending = False
                    self.note("error", "巡航下发失败，未启动：" + reply.body)
        fields = reply.fields
        name = fields.get("cmd", "")
        with self.lock:
            self._order += 1
            quiet = (name in ("PING", "STAT", "AXIS", "SERVO_STAT", "CHASSIS_STAT", "CHASSIS_JOG", "NAV_STAT", "NAV_IMU") or command in ("CHASSIS STAT", "LIFT STAT", "AUX STAT 5", "AUX STAT 6", "TURN STAT", "FLOW STAT", "FLOW WORLD", "FLOW CONFIG")) \
                and reply.ok
            entry = {"seq": self._order, "time": time.strftime("%H:%M:%S"),
                     "kind": "ok" if reply.ok else "error",
                     "text": f"{command} → {reply.body}"}
            if not quiet:
                self.log.append(entry)
                del self.log[:-400]
        out = extra
        if not reply.ok:
            out.append({"type": "log", "entry": entry})
            out.append({"type": "status", "text": f"命令被拒绝：{fields.get('code', reply.body)}"})
            return out
        if not quiet:
            out.append({"type": "log", "entry": entry})
        if name == "HELLO":
            with self.lock:
                self.device.update({k: v for k, v in fields.items() if k != "cmd"})
            out.append({"type": "device", "fields": {k: v for k, v in self.device.items()
                                                     if not k.startswith("_")}})
        elif name == "INFO":
            with self.lock:
                self.device.update({k: v for k, v in fields.items() if k != "cmd"})
            out.append({"type": "device", "fields": {k: v for k, v in self.device.items()
                                                     if not k.startswith("_")}})
        elif name == "STAT":
            with self.lock:
                self.armed = fields.get("armed") == "1"
                self.device["stat"] = fields
            out.append({"type": "stat", "fields": fields})
        elif name == "AXIS":
            with self.lock:
                self.axes[fields.get("axis")] = fields
            out.append({"type": "axis", "fields": fields})
        elif name == "SERVO_STAT":
            with self.lock:
                self.servo = {"cmd": "SERVO_STAT", "raw": reply.body}
            out.append({"type": "servo", "raw": reply.body, "channels": self.servo_count(),
                        "travel": self._travel()})
        elif name == "SERVO":
            with self.lock:
                # Keep the per-channel view from the last full status and patch in
                # the channel that just moved, so a page that joins between two
                # polls still shows both horns.
                frames = dict(self.servo.get("frames", {}))
                frames[str(fields.get("ch"))] = {"angle": fields.get("angle"),
                                                 "us": fields.get("us"), "on": "1"}
                self.servo = {"cmd": "SERVO_STAT",
                              "raw": " ".join(
                                  f"OK cmd=SERVO_STAT ch={ch} angle={frame['angle']} "
                                  f"us={frame['us']} on={frame['on']}"
                                  for ch, frame in sorted(frames.items())),
                              "frames": frames}
            out.append({"type": "servo", "raw": self.servo["raw"], "channels": self.servo_count(),
                        "travel": self._travel()})
        elif name == "LIFT_STAT":
            with self.lock:
                self.lift = fields
            out.append({"type": "lift", "fields": fields})
        elif name == "AUX_STAT":
            with self.lock:
                self.auxiliary[fields["axis"]] = fields
            out.append({"type": "auxiliary", "fields": fields})
        elif name in ("CHASSIS", "CHASSIS_STAT"):
            with self.lock:
                self.chassis = fields
            out.append({"type": "chassis", "fields": fields})
        elif name == "NAV_STAT":
            with self.lock:
                self.navigation = fields
            out.append({"type": "navigation", "fields": fields})
        elif name.startswith("FLOW_"):
            with self.lock:
                self.flow.update(fields)
            out.append({"type": "flow", "fields": fields})
        elif name == "TURN_STAT":
            with self.lock:
                self.turn = fields
            out.append({"type": "turn", "fields": fields})
        elif name == "LED" or "led" in fields:
            out.append({"type": "led", "mask": int(fields.get("led", 0))})
        if "armed" in fields:
            with self.lock:
                self.armed = fields["armed"] == "1"
            out.append({"type": "arm", "armed": fields["armed"] == "1"})
        return out

    def servo_count(self):
        try:
            return max(1, int(self.device.get("servo", 1) or 1))
        except (TypeError, ValueError):
            return 1

    def _travel(self):
        try:
            return int(self.device.get("servo_travel", 2700)) / 10.
        except (TypeError, ValueError):
            return 270.0

    # ---- snapshots ---------------------------------------------------------

    def hello(self):
        """Everything a newly opened page needs to render immediately."""
        automatic = self.autolink.snapshot() if self.autolink else None
        with self.lock:
            return {
                "type": "hello",
                "link": "connected" if self.connection else "offline",
                "port": self.device.get("_port"),
                "armed": self.armed,
                "wiring_ack": bool(self.device.get("_ack")),
                "device": {k: v for k, v in self.device.items() if not k.startswith("_")},
                "axes": self.axes,
                "servo": self.servo,
                "servo_channels": self.servo_count(),
                "chassis": self.chassis,
                "lift": self.lift,
                "auxiliary": self.auxiliary,
                "aux_settings": self.aux_settings.snapshot(),
                "navigation": self.navigation,
                "turn": self.turn,
                "flow": self.flow,
                "flow_settings": self.flow_settings.snapshot(),
                "mission": self.mission.snapshot(),
                "travel": self._travel(),
                "log": self.log[-120:],
                "demo": self.device.get("_port") == "SIM",
                "imu": self.imu.snapshot(),
                "auto_devices": automatic,
            }

    def connect_imu(self, port):
        """Keep H743/IMU serial ownership exclusive even during handshakes."""
        with self.lock:
            if self.connection and self.device.get("_port") == port:
                return False, "该端口已由 H743 连接占用，请先断开。"
            return self.imu.connect(port)
