"""One operator start owns preparation through the saved route's completion."""
import time
from .navigation import reference_command, validated


class Competition:
    """Nonblocking startup; matching ACKs advance steps, STOP retires callbacks."""
    def __init__(self, owner):
        self.owner = owner
        self.generation = 0
        self.state = 'idle'
        self.error = ''
        self.pending = None
        self.queue = []
        self.full_flow = False

    @property
    def active(self):
        return self.state in ('connecting', 'preparing', 'running')

    def snapshot(self):
        """Status for the competition screen; no browser owns the sequence."""
        return dict(state=self.state, error=self.error, active=self.active,
                    full_flow=self.full_flow)

    def stop(self, error=''):
        """Cancel queued startup before an explicit stop or connection loss."""
        self.generation += 1
        self.pending = None; self.queue = []
        self.state = 'fault' if error else 'stopped'
        self.error = error

    def fail(self, error):
        """Report a failed handshake without ever replaying a movement command."""
        self.stop(error)
        if self.owner.connection: self.owner.connection.submit('STOP')
        self.owner.note('error', error)

    def start(self, full_flow=False):
        """The button authorizes origin and ARM; no wiring checkbox is required."""
        c = self.owner
        with c.lock:
            if type(full_flow) is not bool:
                return False, 'full_flow 必须为布尔值。'
            if self.active or c.nav_pending or c.navigator.active or c.task.active or c.pick_place.active or c.camera_alignment.active:
                return False, '当前任务正在执行。'
            if any(a.get('active') == '1' for a in c.axes.values()):
                return False, '车辆仍在执行动作。'
            route, task_config = c.mission.snapshot(), c.task_settings.snapshot()
            if full_flow:
                from .material_flow import build_plan
                from .pick_place import stacked_config
                try:
                    route, task_config = build_plan(route, task_config)
                    stacked_config(c.pick_place.settings.snapshot())
                except ValueError as error:return False, str(error)
                if not c.task_cameras:
                    return False, '全流程需要NX双摄像头服务。'
            try:
                route = validated(route)
            except ValueError as error:
                return False, '巡航参数尚不可执行：' + str(error)
            self.generation += 1; self.state = 'connecting'; self.error = ''
            self.full_flow = full_flow
            self.pending = None; self.queue = []; self.started = time.monotonic()
            self.route = route
            # Race stations advance on task completion, without the debug dwell.
            self.route['dwell_ms'] = 0
            self.task_config = task_config
            self.camera_token = None
            # A debug-only manual hold has no operator release in a race.
            # Keep every configured station action, but let empty holds pass.
            for index, leg in enumerate(self.route['route'], 1):
                if not leg.get('task') and not self.task_config['legs'].get(str(index)):
                    leg['hold'] = 0
        # AutoDevices.tick takes its own lock before Console.lock.
        if c.autolink: c.autolink.set_enabled(True)
        return True, '正在连接设备、恢复参数并启动已保存路线。'

    def reply(self, command, reply, tag):
        """Stale or unrelated replies cannot arm or advance a new run."""
        if not self.pending or tag != self.pending[0]: return
        self.pending = None
        if not reply.ok:
            self.fail('启动准备失败：' + reply.body)
            return
        if self.full_flow:
            try:self.check_mechanism(command, reply.fields)
            except (KeyError, ValueError, TypeError) as error:self.fail('启动准备失败：'+str(error))

    def check_mechanism(self, command, fields):
        """Verify the start-established rotation zero and negative lift direction."""
        if command in ('LIFT DIR -1', 'LIFT STAT') and fields.get('down_sign') != '-1':
            raise ValueError('M5负方向下降设置未生效。')
        if command == 'AUX ZERO 6' and (fields.get('axis') != '6' or
                fields.get('referenced') != '1' or fields.get('pos') != '0'):
            raise ValueError('M6启动位置零点设置未生效。')
        if not command.startswith('AUX STAT '):return
        from .pick_place import cycle_steps, stacked_config
        from .servo_config import wire_angle
        axis = int(command.split()[-1])
        if (fields.get('axis') != str(axis) or fields.get('active') != '0' or
                fields.get('referenced') != '1' or int(fields['pos']) != 0 or fields.get('configured') != '1'):
            raise ValueError('M{}必须已建立零点、配置范围且停在0。'.format(axis))
        low, high = int(fields['min']), int(fields['max'])
        config = self.owner.pick_place.settings.snapshot()
        ground = dict(config, pick_down=config['place_drop_down'])
        stacked = stacked_config(config)
        for mode, config in (('pick', self.owner.b_pickup.cycle_config(config)),
                             ('pick', ground), ('place', config), ('place', stacked)):
            for item in (1,2,3):
                for step in cycle_steps(config, mode, item):
                    if step.get('axis') == axis and not low <= step['target'] <= high:
                        raise ValueError('M{}抓放目标{}超出已保存范围{}～{}。'.format(axis,step['target'],low,high))
                    if 'servo' in step:
                        wire_angle(self.owner.device,step['servo'],step['angle'])

    def tick(self):
        """Prepare once and let the existing route/task executor own travel."""
        if not self.active: return
        c = self.owner
        now = time.monotonic()
        if self.state == 'running':
            if c.navigator.state == 'fault' or c.task.state == 'fault':
                self.stop(c.navigator.error or c.task.error)
            elif not c.nav_pending and not c.navigator.active and not c.task.active:
                self.state = 'done'
            return
        if now - self.started > 30:
            self.fail('启动准备超时：' + ('等待设备或有效航向。' if self.state == 'connecting' else '等待板端应答。'))
            return
        if self.state == 'connecting':
            if not c.connection or c.device.get('nav_path') != '1': return
            if c.device.get('_port') != 'SIM' and not c.imu.navigation_feedback()['valid']: return
            if c.settings_pending: return
            if c.settings_error:
                self.fail(c.settings_error); return
            first_vision = next((leg.get('task') for leg in self.route['route'] if leg.get('task') in (1, 4)), None)
            if first_vision == 1 and c.task_cameras:
                if self.camera_token is None:
                    self.camera_token = c.task_cameras.request('1')
                camera = c.task_cameras.snapshot()
                if camera.get('state') in ('fault', 'error'):
                    self.fail(camera.get('error') or '摄像头启动失败。'); return
                if camera.get('token') != self.camera_token or camera.get('state') != 'ready': return
            if self.route.get('start_station', 0) and c.device.get('nav_reference') != '1':
                self.fail('X 点建系需要支持 nav_reference=1 的 H743 固件。'); return
            # HELLO already restored confirmed settings; do not replay them on
            # every start after waiting for settings_pending above.
            self.queue = ['ARM 0']
            if self.full_flow:
                # Each operator start references the stationary M6 pose once.
                # Zero before CONFIG: the board requires a reference to set limits.
                # M5 keeps its top reference and positive logical depth values.
                self.queue += ['LIFT DIR -1', 'AUX ZERO 6']
                if c.aux_settings.path.exists():
                    ranges = c.aux_settings.snapshot()
                    self.queue += ['AUX CONFIG {} {} {}'.format(axis, ranges[str(axis)]['min'], ranges[str(axis)]['max']) for axis in (5,6)]
                self.queue += ['LIFT STAT', 'AUX STAT 5', 'AUX STAT 6']
            self.queue += [reference_command(self.route),
                          'ARM 1', 'STAT', 'NAV STAT', 'TURN STAT', 'NAV ALIGN']
            self.state = 'preparing'
        if self.pending:
            if now - self.pending[1] > 3: self.fail('启动准备应答超时。')
            return
        if self.queue:
            command = self.queue.pop(0)
            tag = ('competition', self.generation, len(self.queue))
            self.pending = (tag, now)
            if not c.connection.submit(command, tag=tag): self.fail('启动准备发送失败。')
            return
        self.state = 'running'
        ok, reason = c.task_action('start')
        if not ok: self.fail(reason)
