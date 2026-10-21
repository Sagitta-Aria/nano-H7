"""One-item pick/place cycles for the web console and future vision callers.

All public executor methods run under Console.lock. Motion stays in the existing
serial owner; neither importing this module nor saving parameters moves hardware.
Positions are logical absolute pulses from the established M5/M6 zero points.
"""
import copy
import json
import os
import tempfile
import time
import uuid
from pathlib import Path

from .servo_config import wire_angle, wire_channel
from .task_code import COLORS, parse_task_code


DEFAULT = {
    'pick_down': 9200, 'pick_rotation': -1750,
    'place_take_down': 9200, 'place_rotation': -1750, 'place_drop_down': 9200,
    'd_second_drop_down': -1,
    'lift_sps': 300, 'lift_accel': 1000,
    'rotation_sps': 300, 'rotation_accel': 1000,
    's1_open': 120, 's1_closed': 75,
    's2_slot_1': 270, 's2_slot_2': 135, 's2_slot_3': 0,
    's1_settle_ms': 600, 's2_settle_ms': 1000,
    'step_timeout_ms': 60000, 'ack_timeout_ms': 3000, 'vision_timeout_ms': 30000,
}


def validated(config):
    """Validate a complete editable snapshot; never change board travel limits."""
    if isinstance(config, dict) and 'd_second_drop_down' not in config:
        # Upgrade old files in memory; never invent a calibrated stacking depth.
        config = dict(config, d_second_drop_down=-1)
    if not isinstance(config, dict) or set(config) != set(DEFAULT):
        raise ValueError('抓取与放置参数不完整，或含有未知参数。')
    for key, value in config.items():
        low, high = 0, 1000000
        if key == 'd_second_drop_down':
            low = -1
        elif key in ('pick_rotation', 'place_rotation'):
            low = -1000000
        elif key.endswith('_sps'):
            low, high = 1, 3000
        elif key.endswith('_accel'):
            low, high = 1, 20000
        elif key in ('s1_open', 's1_closed'):
            high = 180
        elif key.startswith('s2_slot_'):
            high = 270
        elif key.endswith('_settle_ms'):
            low, high = 100, 30000
        elif key.endswith('_timeout_ms'):
            low, high = 1000, 600000
        if type(value) is not int or not low <= value <= high:
            raise ValueError('{} 必须是 {}～{} 范围内的整数。'.format(key, low, high))
    if config['s1_open'] == config['s1_closed']:
        raise ValueError('S1 张开与闭合角度不能相同。')
    return copy.deepcopy(config)


def stacked_config(config):
    """Use only for second-D placement; require a separately calibrated depth."""
    config = validated(config)
    depth = config['d_second_drop_down']
    if not 0 <= depth < config['place_drop_down']:
        raise ValueError('请标定第二次D点放料下降：必须≥0且小于地面放料下降，-1表示未标定。')
    return dict(config, place_drop_down=depth)


class PickPlaceSettings:
    """Persist only this module's parameters atomically, without serial I/O."""
    validate = staticmethod(validated)

    def __init__(self, path, initial=None):
        self.path = Path(path)
        self.value = self.validate(json.loads(self.path.read_text(encoding='utf-8'))
                      if self.path.exists() else (DEFAULT if initial is None else initial))

    def snapshot(self):
        return copy.deepcopy(self.value)

    def save(self, config):
        value = self.validate(config)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                    dir=self.path.parent, suffix='.tmp', delete=False) as output:
                temporary = output.name
                json.dump(value, output, ensure_ascii=False, indent=2)
                output.write('\n')
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        self.value = value
        return self.snapshot()


def cycle_steps(config, mode, item):
    """Build one cycle; item is a one-based batch ordinal, never a color ID.

    Pick fills slots 1/2/3 (270/135/0); place retrieves slots 3/2/1 (0/135/270).
    A stage containing both an axis and a servo dispatches them together and
    waits for both. Servo completion means an acknowledged command plus dwell,
    since the current PWM servos provide no measured-position feedback.
    """
    c = validated(config)
    if mode not in ('pick', 'place') or type(item) is not int or not 1 <= item <= 3:
        raise ValueError('动作必须为 pick/place，每批序号必须为 1～3。')

    def axis(label, number, target):
        return dict(label=label, axis=number, target=target)

    def servo(label, channel, angle):
        return dict(label=label, servo=channel, angle=angle)

    opened = servo('S1 张开', 1, c['s1_open'])
    closed = servo('S1 闭合', 1, c['s1_closed'])
    top = axis('升降回最高点 0', 5, 0)
    home = axis('旋转回原点 0', 6, 0)
    if mode == 'pick':
        transfer = axis('旋转到转移位置，同时定位 S2', 6, c['pick_rotation'])
        transfer.update(servo=2, angle=c['s2_slot_{}'.format(item)])
        return [opened, axis('下降抓取物料', 5, c['pick_down']), closed, top,
                transfer, opened, home]
    slot = 4 - item
    return [opened, axis('旋转到车上取料位置', 6, c['place_rotation']),
            servo('S2 定位车上取料槽', 2, c['s2_slot_{}'.format(slot)]),
            axis('下降到车上取料位置', 5, c['place_take_down']), closed, top,
            dict(label='最高点稳定等待 0.5 秒', wait_ms=500), home,
            axis('下降到放料位置', 5, c['place_drop_down']), opened, top]


class PickPlace:
    """Nonblocking executor; tick/reply own transitions, vision only sets a mailbox.

    Each start executes exactly one item, with fresh board preflight and a unique
    run ID. STOP/disconnect/fault invalidates all pending replies; no auto-resume.
    Future vision orchestration should start a gated cycle, read request, confirm
    it through vision(), and await state=done before moving the chassis again.
    """
    def __init__(self, console, path):
        self.console = console
        self.settings = PickPlaceSettings(path)
        self.state = 'idle'
        self.error = ''
        self.job_id = None
        self.context = {}
        self.steps = []
        self.index = -1
        self.pending = {}
        self.sequence = 0
        self.request = None
        self.vision_ready = False
        self.positions = {}
        self.axis_move = None
        self.servo_until = 0
        self.last_query = 0

    @property
    def active(self):
        return self.state in ('camera', 'vision', 'pickup_wait', 'pickup_delay', 'preflight', 'arming', 'running')

    def snapshot(self):
        """Return immutable status for UI/vision; polling never advances motion."""
        return dict(state=self.state, active=self.active, error=self.error,
                    job_id=self.job_id, context=copy.deepcopy(self.context),
                    step=self.index + 1, total=len(self.steps),
                    label=(self.steps[self.index]['label'] if 0 <= self.index < len(self.steps)
                           else getattr(self, 'setup_label', '')),
                    request=copy.deepcopy(self.request))

    def save(self, config):
        if self.active or self.console.task.active or self.console.competition.active:
            raise ValueError('请先停止抓取与放置，再保存参数。')
        return self.settings.save(config)

    def start(self, mode, item=1, batch=1, task_code=None, require_vision=False, *,
              owner=None, ground=False, profile='default'):
        """Start one cycle, applying saved travel ranges and arming as needed.

        Existing references and direction are retained; setup never re-zeros
        an axis. Standalone vision gates precede setup. The B station instead
        inserts its color gate after the verified descent and before closing.
        A standalone B test opens camera one and uses this same lowered gate.
        """
        c = self.console
        owned = (owner is c.task and c.task.active and
                 c.task.state == 'material_cycle' and
                 c.navigation.get('state') == 'suspended' and
                 c.navigation.get('path_id') == c.task.path_id)
        if (owner is not None or ground) and not owned:
            raise ValueError('抓放流程尚未取得当前挂起站点控制权。')
        if self.active:
            raise ValueError('抓取与放置正在运行。')
        if type(batch) is not int or batch not in (1, 2) or type(require_vision) is not bool:
            raise ValueError('批次必须为 1 或 2，视觉等待必须为布尔值。')
        if profile not in ('default', 'b_pickup'):
            raise ValueError('抓取参数方案必须为 default 或 b_pickup。')
        if profile == 'b_pickup' and (mode != 'pick' or require_vision or owner is not None or ground):
            raise ValueError('B点完整抓取仅支持独立的相机抓取测试。')
        if profile == 'b_pickup' and c.task_cameras is None:
            raise ValueError('B点完整抓取需要摄像头1任务控制。')
        if task_code is not None and not isinstance(task_code, str):
            raise ValueError('二维码任务码必须是字符串。')
        config = self.settings.snapshot()
        b_station_pick = owned and c.task.material.station == 'B' and mode == 'pick'
        if profile == 'b_pickup' or b_station_pick:
            config = c.b_pickup.cycle_config(config)
        if ground:
            config['pick_down'] = config['place_drop_down']
        stacked = (owned and c.task.material.station == 'D' and batch == 2 and mode == 'place')
        if stacked:
            config = stacked_config(config)
        steps = cycle_steps(config, mode, item)
        if b_station_pick or profile == 'b_pickup':
            # Confirm the lowered endpoint before asking for color. This gate
            # belongs to both B entry points; generic/C/D cycles stay separate.
            steps.insert(2, dict(label='B点框内等待目标颜色', pickup_gate=True))
            delay = c.b_pickup.snapshot()['recognition_delay_ms']
            steps.insert(3, dict(label='B点目标色已确认，延迟 {} ms 后抓取'.format(delay),
                                 pickup_delay_ms=delay))
        if not c.connection or c.device.get('aux_control') != '1':
            raise ValueError('控制板未连接，或不支持升降与旋转。')
        if c.motion.get('latched') == '1':
            raise ValueError('请先排除运动故障并复位。')
        # At a confirmed station handoff the UI AXIS cache can still describe
        # travel: mission polling omits those debug queries. The owned cycle's
        # preflight reads all seven live axes before any mechanism command.
        if ((c.task.active and not owned) or (c.navigator.active and not owned) or c.camera_alignment.active or
                c.nav_pending or c.aux_pending or
                (not owned and any(a.get('active') == '1' for a in c.axes.values())) or
                c.navigation.get('state') in (('moving', 'turning', 'orienting', 'waiting', 'nudging', 'blocked', 'checkpoint') if owned else ('moving', 'turning', 'orienting', 'waiting', 'nudging', 'suspended', 'blocked', 'checkpoint')) or
                c.chassis.get('state') in ('moving', 'jogging', 'switching', 'fault') or
                c.turn.get('state') in ('turning', 'fault')):
            raise ValueError('请先停止底盘、视觉对准、巡航和其他运动。')
        if c.servo_count() < 2:
            raise ValueError('抓取与放置需要 S1、S2 两路舵机。')
        # Validate both physical servo routes before any command is queued.
        for step in steps:
            if 'servo' in step:
                wire_angle(c.device, step['servo'], step['angle'])
        context = dict(mode=mode, item=item, batch=batch,
                       slot=4-item if mode == 'pick' else item,
                       pickup_index=item if mode == 'pick' else 4-item)
        if stacked:
            context['profile'] = 'd_second'
        if profile == 'b_pickup' or b_station_pick:
            context['profile'] = 'b_pickup'
        code = task_code if task_code is not None else (c.task.task_plan or {}).get('code')
        if code:
            plan = parse_task_code(code)
            # Parser items are in fixed retrieval order; retain color/marker
            # identity while the executor uses separately calibrated S2 angles.
            selected = plan['batches'][batch-1]['items'][context['slot']-1]
            context.update(task_code=plan['code'], color_id=selected['color_id'],
                           color_name=selected['color_name'], target=selected['target'])
        if profile == 'b_pickup' or b_station_pick:
            color_id = context.get('color_id')
            if color_id is None:
                target = c.camera_alignment.settings.snapshot()['target']
                color_id = next(int(key) for key, value in COLORS.items() if value[0] == target)
            color, color_name = COLORS[str(color_id)]
            context.update(color_id=color_id, color=color, color_name=color_name)
        camera_token = c.task_cameras.request('1', timeout=None) if profile == 'b_pickup' else None
        self.job_id = uuid.uuid4().hex
        self.owner = owner
        self.context, self.config, self.steps = context, config, steps
        self.index, self.error, self.sequence = -1, '', 0
        self.pending = {}
        self.positions = {}
        self.axis_move = None
        self.vision_ready = False
        self.camera_token = camera_token
        self.pickup_camera_started = False
        self.started = time.monotonic()
        self.state = 'camera' if profile == 'b_pickup' else 'vision' if require_vision else 'preflight'
        self.request = (dict(job_id=self.job_id, issued_at=time.time(), **context)
                        if require_vision else None)
        self.preflight = []
        self.preflight_started = False
        self.setup_label = '等待摄像头1启动' if camera_token is not None else '读取机构状态'
        self.applied_ranges = set()
        self.saved_ranges = c.aux_settings.snapshot() if c.aux_settings.path.exists() else {}

    def vision(self, sample, *, owner=None):
        """Accept only this waiting job's fresh matching result, without I/O.

        A producer sends job_id/mode/batch/item, ready=true, captured_at (Unix
        seconds), and the requested color_id for pick or target for place.
        Stale, repeated, wrong-target and previous-run samples are rejected.
        """
        if self.state not in ('vision', 'pickup_wait') or self.vision_ready or not isinstance(sample, dict):
            raise ValueError('当前没有待确认的抓取与放置视觉请求。')
        if self.state == 'pickup_wait':
            station_owned = owner is self.owner and owner is self.console.task
            standalone_owned = (self.owner is None and owner is self and
                                self.context.get('profile') == 'b_pickup')
            if not (station_owned or standalone_owned):
                raise ValueError('B点颜色确认必须来自当前抓取流程。')
        if any(sample.get(key) != self.request[key] for key in ('job_id', 'mode', 'batch', 'item')):
            raise ValueError('视觉结果与当前动作、批次或序号不匹配。')
        key = 'color_id' if self.context['mode'] == 'pick' else 'target'
        if key in self.context and sample.get(key) != self.context[key]:
            raise ValueError('视觉目标与二维码任务不匹配。')
        captured = sample.get('captured_at')
        now = time.time()
        if (type(captured) not in (int, float) or
                not max(self.request['issued_at'], now-2) <= captured <= now+1):
            raise ValueError('视觉结果时间无效或已过期。')
        if sample.get('ready') is not True:
            raise ValueError('视觉尚未确认目标已对准。')
        self.vision_ready = True

    def stop(self, reason='已停止；请确认机构位置后再启动。', fault=False, send=True):
        """Cancel queued commands through global STOP; never open S1 on a fault."""
        was_active = self.active
        if was_active and self.context.get('profile') == 'b_pickup' and self.owner is None:
            alignment = self.console.camera_alignment
            if (alignment.task_options or {}).get('job_id') == self.job_id:
                alignment.stop(reason, fault=fault, send=False)
            cameras = self.console.task_cameras
            if cameras and cameras.snapshot().get('token') == self.camera_token:
                cameras.cancel()
        self.state = 'fault' if fault else 'stopped'
        self.error = reason
        self.pending.clear()
        self.request = None
        self.vision_ready = False
        if was_active and send and self.console.connection:
            self.console.connection.submit('STOP')

    def fail(self, reason):
        if getattr(self, 'owner', None) is self.console.task and self.console.task.active:
            self.stop(reason, fault=True, send=False)
            self.console.task.fail('抓放动作失败：'+reason)
            return
        self.stop(reason, fault=True)

    def send(self, body, purpose):
        """Track acknowledgements by job and sequence; never retry motion."""
        self.sequence += 1
        tag = ('pick_place', self.job_id, self.sequence)
        self.pending[tag] = (purpose, time.monotonic())
        if not self.console.connection or not self.console.connection.submit(body, tag=tag):
            self.fail('串口队列已满或连接断开。')

    def _preflight(self, purpose, fields):
        """Validate freshly queried zero/limits/idle status, not cached UI data."""
        if purpose == 'STAT':
            if fields.get('motion_fault', '0') != '0':
                raise ValueError('运动故障未清除。')
        elif purpose == 'MOTION STAT':
            if fields.get('latched') != '0':
                raise ValueError('运动故障未清除。')
        elif purpose == 'LIFT STAT':
            if fields.get('down_sign') not in ('1', '-1'):
                raise ValueError('请先确认 M5 向下方向。')
        elif purpose.startswith('AXIS '):
            if fields.get('axis') != purpose.split()[-1] or fields.get('active') != '0':
                raise ValueError('请先停止所有轴。')
        else:
            axis = int(purpose.split()[-1])
            if fields.get('axis') != str(axis):
                raise ValueError('M{} 状态应答不匹配。'.format(axis))
            if fields.get('active') != '0':
                raise ValueError('M{} 仍在运动。'.format(axis))
            if fields.get('referenced') != '1':
                raise ValueError('M{} 零位尚未建立，请在上方设置零位。'.format(axis))
            position, low, high = int(fields['pos']), int(fields['min']), int(fields['max'])
            if position != 0:
                raise ValueError('启动前 M{} 必须位于原点 0，当前为 {}。'.format(axis, position))
            saved = self.saved_ranges.get(str(axis))
            if (saved and axis not in self.applied_ranges and
                    (fields.get('configured') != '1' or (low, high) != (saved['min'], saved['max']))):
                self.applied_ranges.add(axis)
                self.setup_label = '应用 M{} 已保存范围 {}～{}'.format(axis, saved['min'], saved['max'])
                self.send('AUX CONFIG {} {} {}'.format(axis, saved['min'], saved['max']), 'setup')
                self.preflight.insert(0, 'AUX STAT {}'.format(axis))
                return
            if fields.get('configured') != '1':
                raise ValueError('M{} 没有可用的已保存范围，请在上方填写并保存范围。'.format(axis))
            for step in self.steps:
                if step.get('axis') == axis and not low <= step['target'] <= high:
                    raise ValueError('M{} 目标 {} 超出范围 {}～{}，请调整目标或上方范围。'.format(axis, step['target'], low, high))
            self.positions[axis] = position

    def reply(self, body, reply, tag):
        """Advance only matching tagged ACKs and verified finite-move completion."""
        if not self.active or tag not in self.pending:
            return
        purpose, _ = self.pending.pop(tag)
        if not reply.ok:
            self.fail('{}：{}'.format(body, reply.body))
            return
        f = reply.fields
        try:
            if purpose.startswith('preflight:'):
                self._preflight(purpose.split(':', 1)[1], f)
            elif purpose == 'arm':
                if f.get('armed') != '1':
                    raise ValueError('控制板未完成使能。')
            elif purpose == 'move':
                if f.get('axis') != str(self.axis_move['axis']) or not f.get('move_id'):
                    raise ValueError('辅助轴应答缺少匹配的运动编号。')
                self.axis_move['id'] = f['move_id']
            elif purpose == 'servo':
                channel = self.steps[self.index]['servo']
                self.servo_until = time.monotonic() + self.config['s{}_settle_ms'.format(channel)] / 1000
            elif purpose == 'axis':
                if f.get('axis') != str(self.axis_move['axis']) or f.get('move_id') != self.axis_move['id']:
                    raise ValueError('辅助轴运动编号已变化，本次动作取消。')
                if f.get('active') == '0':
                    if f.get('remaining') != '0':
                        raise ValueError('辅助轴提前停止，尚未完成目标脉冲。')
                    self.send('AUX STAT {}'.format(self.axis_move['axis']), 'position')
            elif purpose == 'position':
                if (f.get('axis') != str(self.axis_move['axis']) or
                        f.get('active') != '0' or f.get('referenced') != '1' or
                        int(f['pos']) != self.axis_move['target']):
                    raise ValueError('辅助轴位置未到达目标或零位已失效。')
                self.positions[self.axis_move['axis']] = self.axis_move['target']
                self.axis_move = None
        except (ValueError, KeyError, TypeError) as error:
            self.fail('抓取与放置检查失败：' + str(error))

    def _next(self, now):
        """Launch one stage; the sole parallel stage overlaps M6 and S2."""
        self.index += 1
        if self.index >= len(self.steps):
            self.state = 'done'
            self.index = len(self.steps)-1
            return
        self.stage_started = now
        self.servo_until = 0
        step = self.steps[self.index]
        if step.get('pickup_gate'):
            self.state = 'pickup_wait'
            self.vision_ready = False
            self.request = dict(job_id=self.job_id, issued_at=time.time(), **self.context)
            return
        if 'pickup_delay_ms' in step:
            # Only B inserts this stage. Keep STOP and connection guards live;
            # intentional dwell is independent of mechanism motion timeouts.
            self.state = 'pickup_delay'
            self.pickup_delay_until = now + step['pickup_delay_ms']/1000
            return
        axis = step.get('axis')
        if axis is not None and self.positions[axis] != step['target']:
            self.axis_move = dict(axis=axis, target=step['target'], id=None)
            prefix = 'lift' if axis == 5 else 'rotation'
            self.send('AUX GOTO {} {} {} {}'.format(axis, step['target'],
                      self.config[prefix+'_sps'], self.config[prefix+'_accel']), 'move')
        if 'servo' in step and self.active:
            channel = step['servo']
            self.send('SERVO {} {}'.format(wire_channel(self.console.device, channel),
                      wire_angle(self.console.device, channel, step['angle'])), 'servo')

    def _pickup_camera(self, now):
        """Supervise the standalone B color gate; only fresh NX evidence releases it.

        Called while lowered by this job. It never sends chassis commands or
        accepts external confirmation; stale completion starts recognition again.
        """
        alignment = self.console.camera_alignment
        if not self.pickup_camera_started or (alignment.state == 'done' and now-alignment.last_capture > .75):
            alignment.start(dict(kind='block', target=self.context['color'],
                wait_for_color=True, pickup_only=True, job_id=self.job_id,
                pickup_roi_pct=self.console.b_pickup.recognition_roi()), pickup_owner=self)
            self.pickup_camera_started = True
            return
        if (alignment.task_options or {}).get('job_id') != self.job_id:
            raise ValueError('B点相机抓取动作已变化。')
        if alignment.state in ('fault', 'stopped'):
            raise ValueError('B点识别失败：'+alignment.error)
        if alignment.state == 'done':
            self.vision(dict(self.request, ready=True,
                captured_at=alignment.vision['captured_at']), owner=self)

    def tick(self):
        """Run from Console.tick; never sleeps and never blocks camera processing."""
        if not self.active:
            return
        c, now = self.console, time.monotonic()
        if not c.connection or (self.state in ('running', 'pickup_wait', 'pickup_delay') and not c.armed) or c.motion.get('latched') == '1':
            self.fail('连接中断、使能取消或运动故障，本次动作已取消。')
            return
        if any(now-sent > self.config['ack_timeout_ms']/1000 for _, sent in self.pending.values()):
            self.fail('等待控制板应答超时。')
            return
        if self.state == 'camera':
            camera = c.task_cameras.snapshot()
            if (camera.get('token') != self.camera_token or
                    camera.get('state') in ('error', 'fault')):
                self.fail(camera.get('error') or '摄像头1切换已取消。')
            elif camera.get('state') == 'ready' and camera.get('slot') == '1':
                self.state = 'preflight'
                self.setup_label = '读取机构状态'
            return
        if self.state == 'pickup_wait':
            if self.owner is None:
                try:
                    self._pickup_camera(now)
                except (ValueError, KeyError, TypeError) as error:
                    self.fail(str(error))
                    return
            if self.vision_ready:
                self.state = 'running'
                self.request = None
                self._next(now)
            return
        if self.state == 'pickup_delay':
            if now >= self.pickup_delay_until:
                self.state = 'running'
                self._next(now)
            return
        if self.state == 'vision':
            if now-self.started > self.config['vision_timeout_ms']/1000:
                self.fail('等待视觉确认超时。')
            elif self.vision_ready:
                self.state = 'preflight'
                self.request = None
            return
        if self.state == 'preflight':
            if not self.preflight_started:
                self.preflight_started = True
                self.preflight = ['STAT'] + (['MOTION STAT'] if c.motion.get('v') == '1' else [])
                self.preflight += ['LIFT STAT', 'AUX STAT 5', 'AUX STAT 6']
                self.preflight += ['AXIS {}'.format(axis) for axis in range(1, 8)]
            if self.pending:
                return
            if self.preflight:
                body = self.preflight.pop(0)
                self.send(body, 'preflight:'+body)
            elif not c.armed:
                self.state = 'arming'
                self.setup_label = '自动使能电机'
                self.send('ARM 1', 'arm')
            else:
                self.state = 'running'
                self._next(now)
            return
        if self.state == 'arming':
            if not self.pending:
                if not c.armed:
                    self.fail('控制板未完成使能。')
                else:
                    self.state = 'running'
                    self._next(now)
            return
        if now-self.stage_started > self.config['step_timeout_ms']/1000:
            self.fail('步骤执行超时：'+self.steps[self.index]['label'])
            return
        if self.pending:
            return
        if self.axis_move:
            if now-self.last_query >= .1:
                self.last_query = now
                self.send('AXIS {}'.format(self.axis_move['axis']), 'axis')
        # A wait-only stage starts after the previous axis completion/position
        # checks, keeping STOP and fault handling live before rotating home.
        elif now >= max(self.servo_until,
                        self.stage_started + self.steps[self.index].get('wait_ms', 0)/1000):
            self._next(now)
