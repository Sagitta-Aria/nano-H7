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

from .transport import Connection, EventQueue, list_ports
from .imu import ImuHub
from . import navigation
from .mission import Mission, MissionSettings
from .navigation import MissionConfig, CONFIG_PATH
from .navigation_runner import NavigationRunner
from .turn import turn_command, gain_command, tuning_command
from .straight import straight_command
from .optical_flow import FlowSettings, CONFIG_PATH as FLOW_CONFIG_PATH, config_command
from .auxiliary import AuxiliaryConfig, validated as validated_auxiliary, CONFIG_PATH as AUX_CONFIG_PATH
from .pick_place import PickPlace
from .b_pickup import BPickupSettings
from .material_flow import MaterialFlowSettings
from .control_settings import ControlSettings
from .competition import Competition

STALE_AFTER = 30.0       # page heartbeat: no poll for this long means nobody is watching
POLL_AXES_PER_TICK = 2   # a full seven-axis sweep takes ~0.7 s, as in the Tk console
SERVO_POLL_PERIOD = .5
CHASSIS_POLL_PERIOD = .5


class Console:
    def __init__(self, navigation_path=CONFIG_PATH, auxiliary_path=AUX_CONFIG_PATH, optical_path=FLOW_CONFIG_PATH, mission_path=None):
        self.lock = threading.RLock()
        self.wakeup = threading.Event()
        self.events = EventQueue(self.wakeup)
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
        # A custom navigation file also isolates task settings in tests/demo QA.
        self.task_settings = MissionSettings(mission_path or self.mission.path.with_name('mission.json'))
        self.task = Mission(self)
        self.task_cameras = None  # Web Session attaches the NX camera resource owner.
        self.navigator = NavigationRunner(self)
        from .camera_alignment import CameraAlignment
        self.camera_alignment = CameraAlignment(self, self.mission.path.with_name('camera_alignment.json'))
        self.nav_align = {}
        self.motion = {}
        self.motion_fault_keys = {}
        self.pick_place = PickPlace(self, self.mission.path.with_name('pick_place.json'))
        self.b_pickup = BPickupSettings(self, self.mission.path.with_name('b_pickup.json'))
        self.material_settings = MaterialFlowSettings(self, self.mission.path.with_name('material_flow.json'))
        self.control_settings = ControlSettings(self.mission.path.with_name('control_settings.json'))
        self.settings_pending = set()
        self.settings_generation = 0
        self.settings_error = ''
        self.settings_restored = False
        self.aux_restore_waiting = set()
        self.competition = Competition(self)
        from .imu_pause import ImuPause
        self.imu_pause = ImuPause(self)

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
            self.events = EventQueue(self.wakeup)
            self.device = {"_port": port, "_ack": bool(acknowledge)}
            self.settings_restored = False
            self.aux_restore_waiting.clear()
            self.settings_pending.clear(); self.settings_error = ''
            self.axes, self.servo, self.armed = {}, {}, False
            self.lift = {}
            self.auxiliary = {}
            self.aux_pending.clear()
            self.navigation = {}
            self.nav_align = {}
            self.motion = {}
            self.motion_fault_keys = {}
            self.turn = {}
            self.flow = {}
            self.nav_upload = []
            self.nav_pending = False
            self.nav_generation += 1
            self.navigator.stop()
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
            self.navigator.stop('连接断开。')
            self.competition.stop()
            if self.pick_place.active:self.pick_place.stop('连接断开。')
            if self.task.active:self.task.stop('连接断开。')
            if self.camera_alignment.active:self.camera_alignment.stop('连接断开。')
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
            if self.competition.state in ('connecting', 'preparing') and body not in ('STOP', 'ARM 0', 'NAV STOP', 'CHASSIS HALT', 'TURN STOP', 'PING', 'HELLO', 'INFO', 'STAT', 'MOTION STAT', 'SERVO STAT', 'CHASSIS STAT', 'NAV STAT', 'NAV EXEC', 'NAV ALIGN', 'TURN STAT', 'LIFT STAT', 'FLOW STAT', 'FLOW WORLD', 'FLOW CONFIG') and not body.startswith(('AXIS ', 'AUX STAT ')):
                return False, '比赛启动准备中。'
            if self.pick_place.active:
                if body in ('STOP', 'CHASSIS HALT', 'NAV STOP', 'ARM 0', 'TURN STOP', 'AUX HALT 5', 'AUX HALT 6'):
                    self.pick_place.stop('操作员停止。', send=False)
                    body = 'STOP'
                elif body not in ('PING','HELLO','INFO','STAT','MOTION STAT','SERVO STAT','CHASSIS STAT','LIFT STAT','AUX STAT 5','AUX STAT 6','NAV STAT','NAV EXEC','NAV ALIGN','TURN STAT','FLOW STAT','FLOW WORLD','FLOW CONFIG') and not body.startswith('AXIS '):
                    return False, '抓取与放置运行中，请先停止再手动操作。'
            if self.navigator.active and body not in ('STOP','NAV STOP','CHASSIS HALT','ARM 0','TURN STOP','MOTION CLEAR','NAV STAT','NAV EXEC','NAV ALIGN','TURN STAT','CHASSIS STAT','PING','HELLO','INFO','STAT','MOTION STAT','SERVO STAT','LIFT STAT','FLOW STAT','FLOW WORLD','FLOW CONFIG') and not body.startswith(('NAV GO ','AXIS ','AUX STAT ')):
                return False, 'NX 正在执行巡航，请先停止再手动操作。'
            if self.camera_alignment.active:
                if body in ('STOP','CHASSIS HALT','NAV STOP','ARM 0','TURN STOP'):
                    self.camera_alignment.stop('操作员停止。', send=False)
                    body = 'STOP'
                elif body not in ('PING','HELLO','INFO','STAT','MOTION STAT','SERVO STAT','CHASSIS STAT','LIFT STAT','AUX STAT 5','AUX STAT 6','NAV STAT','NAV ALIGN','TURN STAT','FLOW STAT','FLOW WORLD','FLOW CONFIG') and not body.startswith('AXIS '):
                    return False, '摄像头对准运行中，请先停止对准再手动操作。'
            if body == 'MOTION CLEAR':
                if self.motion.get('v') != '1':return False,'故障复位需要 H743 0.2.22 或更新固件。'
                if self.armed or self.task.active or self.navigator.active:return False,'请先停止全部并解除使能，再复位。'
                self.nav_upload=[];self.nav_pending=False;self.nav_generation+=1;self.aux_pending.clear()
            if body == 'ARM 1' and self.motion.get('latched') == '1':
                return False,'运动故障仍锁存：'+self.motion.get('reason','unknown')+'；请排除原因后复位。'
            if self.task.active:
                if body in ('STOP','CHASSIS HALT','NAV STOP','ARM 0','TURN STOP'):
                    self.task.stop('操作员停止。',send=False)
                    body='STOP'
                elif body not in ('PING','HELLO','INFO','STAT','MOTION STAT','SERVO STAT','CHASSIS STAT','LIFT STAT','AUX STAT 5','AUX STAT 6','NAV STAT','NAV ALIGN','TURN STAT','FLOW STAT','FLOW WORLD','FLOW CONFIG') and not body.startswith('AXIS '):
                    return False,'自动任务运行中，请先停止任务再手动操作。'
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
            if body in ("STOP", "CHASSIS HALT", "NAV STOP", "ARM 0", "TURN STOP"):
                self.competition.stop()
                self.settings_pending.clear()
                self.navigator.stop("操作员停止。")
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
        with self.lock:
            if body.startswith(('TURN CONFIG ', 'CHASSIS CONFIG ', 'LIFT DIR ')):
                self.settings_generation += 1
                tag = ('settings_apply', self.settings_generation)
                self.settings_pending.add(tag)
                accepted = connection.submit(body, tag=tag)
                if not accepted: self.settings_pending.discard(tag)
            else:
                accepted = connection.submit(body)
            if not accepted:
                return False, "发送队列已满，请稍后再试。"
        return True, body

    def set_wiring_ack(self, value):
        with self.lock:
            self.device["_ack"] = bool(value)
        return True

    def navigation_action(self, action, task_owned=False):
        """Queue origin or an acknowledged MCU upload; never auto-arm/reset."""
        with self.lock:
            if self.pick_place.active and action != 'stop':return False,'请先停止抓取与放置。'
            if self.camera_alignment.active and action != 'stop':return False,'请先停止摄像头对准。'
            if self.task.active and not task_owned and action!='stop':
                return False,'自动任务拥有巡航控制权。'
            if not self.connection or self.device.get("nav_heading") != "1":
                return False, "请连接支持陀螺仪巡航的 0.2.9 固件。"
            if action in ('origin', 'start'):
                try:
                    navigation.validated(self.task.route if task_owned else self.mission.snapshot())
                except ValueError as error:
                    return False, '巡航参数尚不可执行：' + str(error)
            if action == "origin":
                if self.armed or self.nav_pending:
                    return False, "建立起点前请解除使能并停止巡航。"
                config = self.mission.snapshot()
                if config.get('start_station', 0) and self.device.get('nav_reference') != '1':
                    return False, 'X 点建系需要支持 nav_reference=1 的 H743 固件。'
                return self.submit(navigation.reference_command(config))
            if action == "start":
                if self.mission.snapshot().get('start_station', 0) and self.device.get('nav_reference') != '1':
                    return False, 'X 点建系需要支持 nav_reference=1 的 H743 固件。'
                if not task_owned and any(leg.get('task') for leg in self.mission.snapshot()['route']):
                    return self.task_action('start')
                if self.device.get("nav_grid") != "1" or self.device.get("nav_path") != "1":
                    return False, "完整路径上传需要 0.2.14 固件，请先更新 H743。"
                if self.navigator.active or self.nav_pending or not self.armed or self.navigation.get("ready") != "1":
                    return False, "请先建立所选起点并使能，或等待当前下发结束。"
                if self.navigation.get("state") in ("turning", "moving", "orienting", "waiting", "nudging", "blocked", "suspended") or self.turn.get("state") == "turning":
                    return False, "请先停止当前巡航。"
                if self.device.get("_port") != "SIM" and not self.imu.navigation_feedback()["valid"]:
                    return False, "请先连接 H723 并等待校准完成及新航向数据。"
                self.nav_generation += 1
                self.nav_pending = True
                self.nav_config = self.task.route if task_owned else self.mission.snapshot()
                self.nav_upload = ["NAV STAT"]
                return self._navigation_next()
            if action == "stop":
                return self.submit("NAV STOP")
            if action == 'go':
                if self.navigation.get('held')!='1':return False,'当前没有挂起的站点。'
                return self.submit('NAV GO {} {}'.format(self.navigation['path_id'],int(self.navigation['leg'])-1))
            return False, "未知巡航动作。"

    def save_task(self, config):
        """Save only while idle; running plans retain a fixed configuration."""
        with self.lock:
            if self.task.active:raise ValueError('请先停止自动任务。')
            return self.task_settings.save(config)

    def task_action(self, action):
        """Start without implicit arming; capability checks precede upload."""
        with self.lock:
            if action=='stop':
                if self.competition.active:
                    return self.submit('STOP')
                self.task.stop();return True,'stopped'
            if self.pick_place.active:return False,'请先停止抓取与放置。'
            if self.camera_alignment.active:return False,'请先停止摄像头对准。'
            if action!='start':return False,'未知任务动作。'
            if self.motion.get('latched')=='1':return False,'请先排除运动故障并复位。'
            if any(f.get('state')=='fault' for f in (self.chassis,self.turn,self.navigation)):
                return False,'底盘、转向或巡航仍有故障，请先排除原因并复位。'
            if self.nav_align.get('v')!='2':return False,'自动任务需要支持 NAV ALIGN v=2 的新固件。'
            if self.nav_pending or self.task.active:return False,'当前正在下发或执行任务。'
            if any(a.get('active')=='1' for a in self.axes.values()) or self.aux_pending:
                return False,'请先停止所有轴并等待参数应用完成。'
            try:
                if self.competition.state == 'running':
                    self.task.start(self.competition.task_config, self.competition.route)
                else:self.task.start(self.task_settings.snapshot(),self.mission.snapshot())
            except ValueError as error:return False,str(error)
            ok,message=self.navigation_action('start',task_owned=True)
            if not ok:self.task.stop(message,send=False)
            return ok,message

    def task_vision(self, sample):
        """Vision is an input mailbox; only tick() can issue a motor command."""
        with self.lock:self.task.vision(sample)

    def pick_place_action(self, action, **options):
        """Shared manual/future vision entry point; each start handles one item."""
        with self.lock:
            if action == 'stop':
                self.pick_place.stop()
                return True, '已停止抓取与放置。'
            if action not in ('pick', 'place'):
                return False, '未知抓取与放置动作。'
            try:
                self.pick_place.start(action, **options)
            except (ValueError, TypeError) as error:
                return False, str(error)
            return True, '已启动一次{}。'.format('抓取' if action == 'pick' else '放置')

    def pick_place_vision(self, sample):
        """A vision result only fills the current job's mailbox; tick drives I/O."""
        with self.lock:
            self.pick_place.vision(sample)

    def _navigation_next(self):
        if not self.nav_upload:
            return True, "uploaded"
        body = self.nav_upload.pop(0)
        if not self.connection.submit(body, tag=("navigation", self.nav_generation)):
            self.nav_upload = []
            self.nav_pending = False
            return False, "发送队列已满，巡航未启动。"
        return True, body

    def turn_action(self, action, mode="REL", angle=0, sps=500, accel=2000, kp=20, tolerance=None, min_sps=None):
        """Queue standalone yaw actions with capability/freshness checks.

        No route upload, XY reset or implicit arming. MCU rechecks ownership;
        stale page telemetry can never authorize overlapping wheel motion.
        """
        with self.lock:
            if not self.connection or self.device.get("turn") != "1":
                return False, "请连接支持闭环转角的 0.2.10 固件。"
            if self.task.active and action!='stop':return False,'自动任务运行中。'
            if action == "stop":
                return self.submit("TURN STOP")
            if action not in ("zero", "start", "configure"):
                return False, "未知转角动作。"
            if self.nav_pending or self.navigation.get("state") in ("turning", "moving", "orienting", "waiting", "nudging", "blocked", "suspended") or \
                    self.chassis.get("state") in ("moving", "jogging", "switching") or self.turn.get("state") == "turning":
                return False, "请先停止底盘运动或坐标巡航。"
            if action == "configure":
                if self.device.get("turn_tuning") != "1":
                    return False, "可调 P 需要 0.2.15 固件。"
                try:
                    if min_sps is not None and (tolerance is None or 'min_sps' not in self.turn):
                        return False, '最低转向轮速需要新版固件并收到 TURN STAT 参数回读。'
                    if tolerance is not None:
                        if self.device.get('turn_precision') != '1':
                            return False, '可调角度容差需要 0.2.16 固件。'
                        return self.submit(tuning_command(kp, tolerance, min_sps))
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
            if self.competition.active:raise ValueError('比赛正在准备或执行，参数将在结束后修改。')
            if self.camera_alignment.active:raise ValueError('请先停止摄像头对准。')
            if self.navigator.active or self.task.active or self.nav_pending or self.navigation.get('state') in ('turning','moving','orienting','waiting','nudging','blocked','suspended'):
                raise ValueError('请先停止巡航再保存路线。')
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
            if self.pick_place.active:return False,'请先停止抓取与放置。'
            if self.camera_alignment.active:return False,'请先停止摄像头对准。'
            if self.task.active:return False,'自动任务运行中。'
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
            self.aux_restore_waiting.discard(axis)
            return True, "等待板端确认 M{} 范围。".format(axis)

    def _restore_auxiliary(self, fields):
        """Restore saved limits once an idle axis has a real position reference.

        Called with board status under the session lock. Never infer a zero,
        move an axis, or rewrite settings; failed restores require explicit retry.
        """
        axis = fields.get('axis')
        if axis not in self.aux_restore_waiting or axis in self.aux_pending:
            return
        if (fields.get('referenced') != '1' or fields.get('active') != '0' or
                self.task.active or self.pick_place.active or self.camera_alignment.active or
                self.competition.active or self.navigator.active or self.nav_pending or
                (self.armed and self.device.get('aux_live') != '1')):
            return
        value = self.aux_settings.snapshot()[axis]
        self.aux_restore_waiting.discard(axis)
        if (fields.get('configured') == '1' and fields.get('min') == str(value['min']) and
                fields.get('max') == str(value['max'])):
            return
        if not value['min'] <= int(fields['pos']) <= value['max']:
            self.settings_error = 'M{} 已保存范围不包含当前位置，请检查后手动应用。'.format(axis)
            self.note('error', self.settings_error)
            return
        self.settings_generation += 1
        tag = ('settings', self.settings_generation, 'aux' + axis)
        self.settings_pending.add(tag)
        if not self.connection.submit('AUX CONFIG {} {} {}'.format(axis, value['min'], value['max']), tag=tag):
            self.settings_pending.discard(tag)
            self.settings_error = 'M{} 已保存范围恢复发送失败。'.format(axis)

    def jog(self, body):
        """Latest-only intent for held keys/buttons; the worker expires it."""
        with self.lock:
            if body and self.competition.active:return False,'比赛拥有运动控制权。'
            if body and self.pick_place.active:return False,'抓取与放置拥有运动控制权。'
            if body and self.camera_alignment.active:return False,'摄像头对准拥有运动控制权。'
            if body and self.task.active:return False,'自动任务拥有运动控制权。'
            if body and (not self.armed or self.motion.get('latched')=='1'):
                return False,'请先排除故障并使能。'
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
            paused = self.imu_pause.sync()
            if not paused and self.imu_pause.replies:
                saved, self.imu_pause.replies = self.imu_pause.replies, []
                for command, reply, tag in saved: self._reply(command, reply, tag)
            if paused and self.armed and self.motion.get('latched') != '1':
                return
            executors = (self.competition, self.pick_place, self.camera_alignment, self.task, self.navigator)
            running = any(executor.active for executor in executors)
            self.competition.tick()
            self.pick_place.tick()
            self.camera_alignment.tick()
            self.task.tick()
            self.navigator.tick()
            if running and not any(executor.active for executor in executors):
                return  # Do not append a debug poll burst to a terminal STOP.
            watcher = self.nav_pending or self.competition.active or self.pick_place.active or self.navigator.active or self.task.active or self.camera_alignment.active or time.monotonic() - self.last_client < STALE_AFTER
            connection = self.connection
        if connection is None or not watcher or connection.commands.qsize() >= 3:
            return
        now = time.monotonic()
        if self.camera_alignment.active:
            # NX alignment owns tagged status queries. Keep essential fault and
            # disarm feedback, while avoiding a backlog of idle UI telemetry
            # delaying the heading check after every local detection.
            if now-self.last_stat >= .5:
                self._send('STAT'); self.last_stat = now
            if self.motion.get('v')=='1' and now-self.last_chassis >= .2:
                self._send('MOTION STAT'); self.last_chassis = now
            return
        if self.nav_pending or self.competition.active or self.navigator.active or self.task.active or self.pick_place.active:
            # Executors own completion queries. Debug panels must not enqueue a
            # dozen unrelated reads ahead of the next movement on a 115200 link.
            if now-self.last_stat >= .5:
                self._send('STAT'); self.last_stat = now
            if self.motion.get('v') == '1' and now-self.last_chassis >= .5:
                self._send('MOTION STAT'); self.last_chassis = now
            # Station/vision tasks still supervise navigation while it is held.
            if self.task.active and not self.nav_pending and now-self.last_navigation >= .5:
                self._send('NAV STAT'); self.last_navigation = now
            return
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
            if self.motion.get('v')=='1':self._send('MOTION STAT')
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
            version=self.device.get('fw','').split('.')
            if (not self.nav_pending and not self.nav_align and len(version)==3 and
                    all(part.isdigit() for part in version) and tuple(map(int,version))>=(0,2,20)):
                self._send('NAV ALIGN')
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
            connection.submit(body, tag=('telemetry',))

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
                if self.competition.active:self.competition.stop('控制板连接断开。')
                self.settings_pending.clear(); self.settings_restored = False
                if self.pick_place.active:self.pick_place.stop('串口断开，动作不会自动恢复。',fault=True,send=False)
                if self.camera_alignment.active:self.camera_alignment.stop('串口断开，对准不会自动恢复。',fault=True,send=False)
                if self.task.active:self.task.stop('串口断开，任务不会自动恢复。',fault=True,send=False)
                self.navigator.stop('串口断开。',fault=True)
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
            if kind=='error':
                with self.lock:
                    if self.pick_place.active:self.pick_place.fail(str(payload))
                    if self.task.active:self.task.fail(str(payload))
            return [{"type": "log", "entry": {"time": time.strftime("%H:%M:%S"),
                                              "kind": "error" if kind == "error" else "warn",
                                              "text": str(payload)}}]
        return []

    def _reply(self, command, reply, tag=None):
        with self.lock:
            if (command in ('STOP', 'ARM 0') or reply.fields.get('armed') == '0'
                    or reply.fields.get('latched') == '1'
                    or reply.fields.get('fault', 'none') != 'none'):
                self.imu_pause.replies.clear()
            if self.imu_pause.sync() and not self.imu_pause.terminal(command, reply):
                # Successful action ACKs are retained, never reissued. STAT and
                # heartbeat telemetry can still update safety/status caches.
                if tag and tag[0] not in ('telemetry',):
                    self.imu_pause.replies.append((command, reply, tag))
                    return []
            self.competition.reply(command, reply, tag)
            if tag in self.settings_pending:
                self.settings_pending.remove(tag)
                if not reply.ok and tag[0] == 'settings': self.settings_error = '已保存参数恢复失败：' + reply.body
                if reply.ok and command == 'LIFT STAT' and 'lift' in self.control_settings.value:
                    wanted = self.control_settings.value['lift'][0]
                    if int(reply.fields.get('down_sign', 0)) != wanted:
                        self.settings_generation += 1
                        restore_tag = ('settings', self.settings_generation, 'lift')
                        self.settings_pending.add(restore_tag)
                        if not self.connection.submit('LIFT DIR {}'.format(wanted), tag=restore_tag):
                            self.settings_pending.discard(restore_tag)
                            self.settings_error = '升降方向恢复发送失败。'
            if reply.ok and command.startswith(('TURN CONFIG ', 'CHASSIS CONFIG ', 'LIFT DIR ')):
                try:
                    self.control_settings.accept(command, reply.fields)
                    if self.settings_error.startswith('参数已生效，但保存失败：'): self.settings_error = ''
                except (OSError, ValueError, KeyError) as error:
                    self.settings_error = '参数已生效，但保存失败：' + str(error)
                    self.note('error', self.settings_error)
        if tag and tag[0] in ('camera_align','camera_velocity'):
            with self.lock:self.camera_alignment.reply(command,reply,tag)
        extra = []
        with self.lock:
            self.pick_place.reply(command,reply,tag)
            self.task.reply(command,reply,tag)
            self.navigator.reply(command,reply,tag)
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
                            if 'stations' not in fields or 'held' not in fields:
                                raise ValueError('站点路线需要 H743 0.2.19 或更新固件。')
                            if fields.get("ready") != "1" or fields.get("state") in ("turning", "moving", "orienting", "waiting", "nudging", "blocked", "suspended"):
                                raise ValueError("坐标未建立或车辆仍在运动。")
                            self.nav_upload = self.mission.commands(
                                (int(fields["x"]), int(fields["y"])),
                                self.nav_generation % 2147483647 + 1, config=self.nav_config, host_control=True)
                        except (ValueError, KeyError) as error:
                            self.nav_upload = []
                            self.nav_pending = False
                            if self.task.active:self.task.fail('路径规划失败：'+str(error))
                            self.note("error", "路径规划失败：" + str(error))
                    if command == 'NAV EXEC' and (reply.fields.get('cmd') != 'NAV_EXEC' or reply.fields.get('v') != '1'):
                        self.nav_upload=[]; self.nav_pending=False
                        if self.task.active: self.task.fail('需要 H743 通用逐步执行接口。')
                        self.note('error','需要 H743 通用逐步执行接口，未启动运动。')
                        return []
                    if command.startswith("NAV START"):
                        self.navigator.start(self.nav_config,command.split()[2])
                        self.nav_pending = False
                        if self.task.active:
                            self.task.path_id=command.split()[2]
                            self.task.state='travel';self.task.last_nav=time.monotonic()
                    ok, message = self._navigation_next()
                    if not ok:
                        if self.task.active:self.task.fail(message)
                        self.note("error", message)
                else:
                    self.nav_upload = []
                    self.nav_pending = False
                    if self.task.active:self.task.fail('路径上传失败：'+reply.body)
                    self.note("error", "巡航下发失败，未启动：" + reply.body)
        fields = reply.fields
        name = fields.get("cmd", "")
        with self.lock:
            self._order += 1
            quiet = ((tag and tag[0] == 'camera_velocity') or name in ("PING", "STAT", "AXIS", "SERVO_STAT", "CHASSIS_STAT", "CHASSIS_JOG", "NAV_STAT", "NAV_IMU") or command in ("NAV EXEC", "CHASSIS STAT", "LIFT STAT", "AUX STAT 5", "AUX STAT 6", "TURN STAT", "FLOW STAT", "FLOW WORLD", "FLOW CONFIG")) \
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
                if not self.settings_restored:
                    self.settings_restored = True
                    self.settings_generation += 1
                    self.settings_pending.clear(); self.settings_error = ''
                    restore_commands = self.control_settings.commands(include_lift=False)
                    if 'lift' in self.control_settings.value: restore_commands.append('LIFT STAT')
                    if fields.get('aux_control') == '1' and self.aux_settings.path.exists():
                        self.aux_restore_waiting = {'5', '6'}
                        restore_commands += ['AUX STAT 5', 'AUX STAT 6']
                    for index, body in enumerate(restore_commands):
                        restore_tag = ('settings', self.settings_generation, index)
                        self.settings_pending.add(restore_tag)
                        if not self.connection or not self.connection.submit(body, tag=restore_tag):
                            self.settings_pending.discard(restore_tag)
                            self.settings_error = '已保存参数恢复发送失败。'
                self.device.update({k: v for k, v in fields.items() if k != "cmd"})
                try:
                    if tuple(int(v) for v in fields.get('fw','').split('.')) >= (0,2,21):
                        self._send('MOTION STAT')
                except ValueError:pass
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
                if fields.get('motion_fault','0')!='0':
                    self._fault_stop('motor','运动故障：步进输出已停止；请查询具体原因后复位。')
            out.append({"type": "stat", "fields": fields})
        elif name == 'MOTION_STAT':
            with self.lock:
                self.motion=fields
                if fields.get('latched')=='1':
                    message='运动故障：'+fields.get('reason','unknown')+'；旧任务已取消，请排除原因后复位。'
                    self._fault_stop('motor',message)
                    if self.task.state=='fault' and self.task.error.startswith('运动故障'):
                        self.task.error=message
                else:self.motion_fault_keys.pop('motor',None)
                if command=='MOTION CLEAR':
                    self.navigation={**self.navigation,'ready':'0','state':'stopped','held':'0'}
                    self._send('NAV STAT');self._send('CHASSIS STAT');self._send('TURN STAT')
                    out.append({'type':'status','text':'复位成功，保持停止。请确认辅助轴零点并重新建立 O 点，再使能、启动。'})
            out.append({'type':'motion','fields':fields})
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
                self._restore_auxiliary(fields)
            out.append({"type": "auxiliary", "fields": fields})
        elif name in ("CHASSIS", "CHASSIS_STAT"):
            with self.lock:
                self.chassis = fields
                if fields.get('state')=='fault':
                    self._fault_stop('chassis','底盘故障：'+fields.get('fault','unknown'))
                else:self.motion_fault_keys.pop('chassis',None)
            out.append({"type": "chassis", "fields": fields})
        elif name == 'NAV_EXEC':
            out.append({'type':'navigation_runner','fields':self.navigator.snapshot()})
        elif name == "NAV_STAT":
            with self.lock:
                self.navigation = fields
                if not self.imu_pause.paused: self.task.observe(fields)
                if fields.get('state')=='fault':
                    self._fault_stop('navigation','巡航故障：'+fields.get('fault','unknown'))
                else:self.motion_fault_keys.pop('navigation',None)
            out.append({"type": "navigation", "fields": fields})
            # WAITING/external/final-RESTORE transitions use NAV STAT rather
            # than NAV EXEC; publish their ownership changes to the UI too.
            out.append({'type':'navigation_runner','fields':self.navigator.snapshot()})
        elif name == 'NAV_ALIGN':
            with self.lock:self.nav_align=fields
        elif name.startswith("FLOW_"):
            with self.lock:
                self.flow.update(fields)
            out.append({"type": "flow", "fields": fields})
        elif name == "TURN_STAT":
            with self.lock:
                self.turn = fields
                if fields.get('state')=='fault':
                    self._fault_stop('turn','转向故障：'+fields.get('fault','unknown'))
                else:self.motion_fault_keys.pop('turn',None)
            out.append({"type": "turn", "fields": fields})
        elif name == "LED" or "led" in fields:
            out.append({"type": "led", "mask": int(fields.get("led", 0))})
        if command in ('STOP','ARM 0','CHASSIS HALT','NAV STOP'):
            # Includes a worker-expired jog lease: the browser may have missed
            # keyup, and must not retain an already-canceled local busy flag.
            out.append({'type':'jog_cancel'})
        if "armed" in fields:
            with self.lock:
                self.armed = fields["armed"] == "1"
                if not self.armed and self.pick_place.state == 'running':self.pick_place.fail('电机使能已取消。')
                if not self.armed and self.navigator.active: self.navigator.stop("电机使能已取消。",fault=True)
                if not self.armed and self.task.active:self.task.fail('电机使能已取消。')
            out.append({"type": "arm", "armed": fields["armed"] == "1"})
        if tag and tag[0] == 'competition':
            # Advance only after this reply has updated ARM/navigation caches.
            # No extra pump interval between startup commands.
            with self.lock: self.competition.tick()
        return out

    def _fault_stop(self, source, message):
        """Cancel queued work and jog leases once per observed fault, under lock.

        A fault is terminal for the current task. It never retries motion or arms.
        Fault telemetry remains visible until the operator addresses its cause.
        """
        if source in self.motion_fault_keys and not self.task.active and not self.pick_place.active:return
        self.motion_fault_keys[source]=message
        if self.pick_place.active:self.pick_place.fail(message)
        elif self.task.active:self.task.fail(message)
        elif self.connection:self.submit('STOP')
        self.note('error',message)

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
                "pick_place": self.pick_place.snapshot(),
                "pick_place_config": self.pick_place.settings.snapshot(),
                "b_pickup_config": self.b_pickup.snapshot(),
                "material_flow_config": self.material_settings.snapshot(),
                "control_settings": self.control_settings.snapshot(),
                "settings_error": self.settings_error,
                "competition": self.competition.snapshot(),
                "navigation": self.navigation,
                "navigation_runner": self.navigator.snapshot(),
                "turn": self.turn,
                "motion": self.motion,
                "flow": self.flow,
                "flow_settings": self.flow_settings.snapshot(),
                "mission": self.mission.snapshot(),
                "task": self.task.snapshot(),
                "camera_alignment": self.camera_alignment.snapshot(),
                "camera_alignment_config": self.camera_alignment.settings.snapshot(),
                "task_config": self.task_settings.snapshot(),
                "travel": self._travel(),
                "log": self.log[-120:],
                "demo": self.device.get("_port") == "SIM",
                "imu": self.imu.snapshot(),
                "imu_pause": self.imu_pause.snapshot(),
                "auto_devices": automatic,
            }

    def connect_imu(self, port):
        """Keep H743/IMU serial ownership exclusive even during handshakes."""
        with self.lock:
            if self.connection and self.device.get("_port") == port:
                return False, "该端口已由 H743 连接占用，请先断开。"
            return self.imu.connect(port)
