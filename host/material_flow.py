"""Two-batch material policy over the existing navigation and mechanism owners.

Importing/building a plan performs no I/O. Station execution requires an ACKed
NAV SUSPEND; every finite move and mechanism job has its own completion identity.
Point offsets are commanded pulses, not measured encoder positions.
"""
import copy
import math
import time
from .pick_place import PickPlaceSettings

OFFSETS = {1: 0, 2: 2150, 3: 4230}
DEFAULT = dict(point2_steps=OFFSETS[2], point3_steps=OFFSETS[3])


class MaterialFlowSettings(PickPlaceSettings):
    """Persist C/D absolute point offsets; saving never commands hardware."""

    @staticmethod
    def validate(config):
        """Require ordered integer offsets so every forward/backward delta is valid."""
        if not isinstance(config, dict) or set(config) != set(DEFAULT):
            raise ValueError('C/D点位距离参数不完整，或含有未知参数。')
        for key in DEFAULT:
            if type(config[key]) is not int or not 1 <= config[key] <= 100000:
                raise ValueError('点位距离必须是1～100000的整数脉冲。')
        if config['point3_steps'] <= config['point2_steps']:
            raise ValueError('点1→点3必须大于点1→点2的距离。')
        return dict(config)

    def __init__(self, owner, path):
        self.owner = owner
        super().__init__(path, initial=DEFAULT)

    def save(self, config):
        """Call under Console.lock; startup and active motion owners block edits."""
        c = self.owner
        if (c.task.active or c.competition.active or c.pick_place.active or
                c.navigator.active or c.camera_alignment.active or c.nav_pending):
            raise ValueError('请先停止自动任务、巡航和抓放动作，再保存C/D点位距离。')
        return super().save(config)

    def offsets(self):
        """Copy saved absolute offsets for an entire run, including both visits."""
        return {1: 0, 2: self.value['point2_steps'], 3: self.value['point3_steps']}


def build_plan(route, config):
    """Use saved geometry/attitudes, replacing only this run's route and actions.

    C/D retain the existing convention: the configured station is ring point 2.
    Keep saved headings/outward search; departure uses navigation turn inset.
    Legacy ring-task right exits do not belong to this material route.
    """
    from .navigation import validated
    route, config = copy.deepcopy(route), copy.deepcopy(config)
    names = {s['name'].upper(): i for i, s in enumerate(route['stations'])}
    if any(name not in names for name in 'OABCD') or names['O'] != 0:
        raise ValueError('全流程需要 O（原点）、A、B、C、D 五个站点。')
    templates, occurrences = {}, {}
    for leg in route['route']:
        templates.setdefault(leg['station'], []).append(leg)
    route['route'] = []
    config['legs'] = {}
    for index, name in enumerate(('A', 'B', 'C', 'D', 'B', 'C', 'D', 'O'), 1):
        visits = templates.get(names[name], [{}])
        occurrence = occurrences.get(name, 0)
        old = visits[min(occurrence, len(visits)-1)]
        occurrences[name] = occurrence+1
        leg = dict(station=names[name], hold=int(name != 'O'))
        for key in ('arrival_enabled', 'heading'):
            if key in old:
                leg[key] = old[key]
        if name in 'BCD':
            leg['arrival_enabled'] = True
        route['route'].append(leg)
        if name == 'A':
            config['legs'][str(index)] = [dict(op='task_code')]
        elif name != 'O':
            config['legs'][str(index)] = [dict(op='material', station=name,
                batch=1 if index < 5 else 2,
                outward_steps=old.get('outward_steps', 0),
                right_steps=0)]
    route['dwell_ms'] = 0
    config.update(timeout_s=7200, leg_timeout_s=600, action_timeout_s=120)
    return validated(route), config


class MaterialStation:
    """One held B/C/D visit; Mission owns cancellation and tagged serial replies.

    B tracks visual translations and physically returns to its departure XY.
    C/first-D anchor 1; second-D restores point 2 from remembered layer colors.
    Both visits use the same absolute offsets and finish at reference point 2.
    Tray contents are software bookkeeping, updated only after a matched job.
    """
    def __init__(self, owner, action):
        self.owner, self.c = owner, owner.console
        self.station, self.batch = action['station'], action['batch']
        if not owner.task_plan:
            raise ValueError('尚未锁定本轮二维码任务码。')
        self.items = copy.deepcopy(owner.task_plan['batches'][self.batch-1]['items'])
        self.offsets = dict(owner.material_offsets)
        self.jobs = []
        self.position = 0
        self.current = None
        self.job_id = None
        self.phase = 'suspend'
        self.started = time.monotonic()
        self.alignment_budget_at = None
        self.return_attempts = 0
        self.options = owner.ring_parameters(owner.leg, action)
        self.stacked = self.station == 'D' and self.batch == 2
        if self.stacked:
            if set(owner.d_layout) != {1, 2, 3} or owner.d_heading is None:
                raise ValueError('缺少第一批D点成功放置的完整颜色布局。')
            difference = (self.options['arrival_heading_deg']-owner.d_heading+180) % 360-180
            if abs(difference) > 10:
                raise ValueError('两次D点到站朝向须一致，避免点1/3方向颠倒。')
            self.options.update(stack_layout=copy.deepcopy(owner.d_layout),
                                material_offsets=dict(self.offsets))
        owner.ring_options = self.options
        if self.station == 'B':
            if owner.tray:
                raise ValueError('转盘仍有上一批物料，不能开始新一批抓取。')
            self.jobs = [('pick', item) for item in reversed(self.items)]
        else:
            if owner.tray != {item['slot']: item for item in self.items}:
                raise ValueError('转盘物料身份与本批任务不一致，停止自动放置。')
            self.jobs = [('place', item) for item in self.items]
            if self.station == 'C':
                self.jobs += [('pick', item) for item in sorted(self.items, key=lambda item: item['target'])]

    def snapshot(self):
        """Expose physical slot, destination and progress without advancing I/O."""
        return dict(station=self.station, batch=self.batch, phase=self.phase,
                    item=copy.deepcopy(self.current), remaining=len(self.jobs),
                    position_steps=self.position, offsets=dict(self.offsets))

    def send(self, body, purpose):
        """Route through the sole mission mailbox; no independent serial queue."""
        self.owner.send(body, 'material_'+purpose)

    def start(self):
        """Begin ownership transfer only at the runner's confirmed held arrival."""
        self.owner.state = 'material_suspend'
        self.send('NAV SUSPEND {} {}'.format(self.owner.path_id,
                  self.owner.leg-self.owner.leg_base-1), 'suspend')

    def wait_for_handoff(self):
        """Recheck a refused SUSPEND; an arrival may have resumed yaw correction.

        No motion is retried. Only this held path/leg, fresh IMU and a matched
        idle executor permit another ownership request; all other faults stop.
        """
        self.phase = 'handoff'
        self.owner.state = 'material_handoff'
        self.owner.qr_status = '{}点等待导航交接，正在核对到站和航向修正状态'.format(self.station)
        self.send('NAV STAT', 'handoff_status')

    def align(self, item=None):
        """Wait for B crop color while lowered, or calibrate C/D's first ring."""
        options = dict(self.options, anchor_only=True)
        if item is not None:
            options.update(kind='block', target=item['color'], wait_for_color=True,
                           pickup_only=True, pickup_roi_pct=self.c.b_pickup.recognition_roi())
        self.c.camera_alignment.start(task_options=options)
        self.phase = 'align'
        self.alignment_budget_at = time.monotonic() if self.station == 'B' else None
        self.owner.state = 'material_align'

    def preserve_alignment_budget(self, now):
        """Exclude B color waiting/alignment from station and route budgets.

        Mission calls this before its timeout checks, including camera startup.
        B waits for camera/color without a timer; reply/mechanism guards remain.
        """
        if self.station == 'B' and self.phase in ('camera', 'align') and self.alignment_budget_at is not None:
            elapsed = now-self.alignment_budget_at
            self.started += elapsed
            self.owner.started += elapsed
            self.owner.leg_started += elapsed
            self.alignment_budget_at = now

    def next_job(self):
        """Keep tray retrieval fixed; C recollection visits physical markers 1/2/3."""
        if not self.jobs:
            if self.station == 'B':
                self.phase = 'return'
                self.send('NAV STAT', 'return_pose')
            else:
                self.move_to(2, 'finish')
            return
        self.mode, self.current = self.jobs.pop(0)
        if self.station == 'B':
            self.cycle()
        else:
            self.move_to(self.current['target'], 'cycle')

    def move_to(self, target, after):
        """Compute each delta from this run's saved absolute point offsets."""
        delta = self.offsets[target]-self.position
        self.destination = self.offsets[target]
        if delta:
            self.move('FWD' if delta > 0 else 'BACK', abs(delta), after)
        else:
            self.after_move(after)

    def move(self, direction, steps, after):
        """Wait for exact move ID, zero remaining and idle before transferring control."""
        self.after, self.move_id = after, None
        self.phase = 'move'
        self.owner.state = 'material_move'
        cfg = self.c.camera_alignment.settings.snapshot()
        speed = cfg['ring_sequence']['sps']
        self.deadline = time.monotonic()+steps/speed+10
        self.send('CHASSIS STRAIGHT {} {} {} {}'.format(
            direction, steps, speed, cfg['tracking']['accel']), 'move_accept')

    def after_move(self, after):
        """Commit a physical endpoint only after its movement completion barrier."""
        if after == 'cycle':
            self.cycle()
        elif after == 'finish':
            self.owner.state = 'ring_fix'
            self.owner.stage_started = time.monotonic()
            self.owner.send(self.owner.ring_pose_command('POSE', self.options['reference']), 'ring_fix')
        else:
            self.send('NAV STAT', 'return_pose')

    def cycle(self):
        """Start one job in its original tray slot; C uses the ground drop height."""
        self.owner.state = 'material_cycle'
        self.phase = 'cycle'
        item = self.current['pickup_index'] if self.mode == 'pick' else self.current['slot']
        self.c.pick_place.start(self.mode, item=item, batch=self.batch,
            task_code=self.owner.task_plan['code'], owner=self.owner,
            ground=self.station == 'C' and self.mode == 'pick')
        self.job_id = self.c.pick_place.job_id

    def reply(self, purpose, fields):
        """Validate tagged navigation/finite movement evidence before advancing."""
        purpose = purpose[len('material_'):]
        o = self.owner
        if purpose == 'suspend':
            self.send('NAV STAT', 'suspended')
        elif purpose == 'handoff_status':
            if (fields.get('path_id') != o.path_id or
                    fields.get('leg') != str(o.leg-o.leg_base) or
                    fields.get('ready') != '1' or fields.get('imu') != '1' or
                    fields.get('fault', 'none') != 'none' or
                    fields.get('state') not in ('waiting', 'orienting') or
                    (fields.get('state') == 'waiting' and fields.get('held') != '1')):
                raise ValueError('导航交接条件失效：'+str(fields))
            if fields.get('state') == 'waiting':
                self.send('NAV EXEC', 'handoff_idle')
        elif purpose == 'handoff_idle':
            if (fields.get('path_id') != o.path_id or fields.get('host') != '1' or
                    fields.get('op_id') != str(self.c.navigator.op) or
                    fields.get('ready') != '1' or fields.get('imu') != '1' or
                    fields.get('fault', 'none') != 'none' or
                    fields.get('state') not in ('waiting', 'orienting')):
                raise ValueError('导航交接执行器状态失效：'+str(fields))
            if fields.get('state') == 'waiting' and fields.get('active') == '0':
                self.phase = 'handoff_ready'
        elif purpose == 'suspended':
            self.check_pose(fields)
            if self.station == 'B':
                self.send(o.ring_pose_command('POSE', self.options['reference']), 'tracking')
            else:
                self.align()
        elif purpose == 'tracking':
            camera = self.c.task_cameras.snapshot()
            if (camera.get('state') != 'ready' or camera.get('slot') != '1' or
                    self.c.task_cameras.slots.hubs['1'].snapshot().get('state') != 'ready'):
                raise ValueError('摄像头1未保持打开，请检查A点相机切换状态。')
            self.camera_token = camera['token']
            self.phase = 'camera'
            self.alignment_budget_at = time.monotonic()
            o.state = 'material_camera'
        elif purpose == 'move_accept':
            self.move_id = fields.get('move_id')
            if self.move_id is None:
                raise ValueError('点间移动缺少运动编号。')
        elif purpose == 'move_status':
            if fields.get('move_id') != self.move_id or fields.get('fault', 'none') != 'none':
                raise ValueError('点间移动编号变化或运动故障。')
            if fields.get('state') in ('stopped', 'fault'):
                raise ValueError('点间移动已被中断。')
            if fields.get('state') == 'done' and fields.get('active') == '0':
                if fields.get('remaining') != '0':
                    raise ValueError('点间移动尚未走完目标脉冲。')
                if self.station != 'B':
                    self.position = self.destination
                # Console absorbs this status after dispatching tagged replies.
                # Transfer on the next pump so cached idle agrees with the ACK.
                self.phase = 'arrived'
        elif purpose == 'return_pose':
            self.check_pose(fields)
            dx, dy = (v-int(fields[k]) for k, v in zip(('x', 'y'), self.options['reference']))
            if max(abs(dx), abs(dy)) <= 10:
                o.stage_started = time.monotonic()
                o.ring_restore()
                return
            self.return_attempts += 1
            if self.return_attempts > 8:
                raise ValueError('抓取后回到B点通道的脉冲积分未收敛。')
            angle = math.radians(int(fields['heading_cd'])/100)
            forward = round(dx*math.sin(angle)+dy*math.cos(angle))
            right = round(dx*math.cos(angle)-dy*math.sin(angle))
            if abs(forward) >= abs(right):
                direction, steps = ('FWD' if forward > 0 else 'BACK'), abs(forward)
            else:
                direction, steps = ('RIGHT' if right > 0 else 'LEFT'), abs(right)
            self.move(direction, steps, 'return')

    def check_pose(self, fields):
        """Reject a changed route or lost IMU during external ownership."""
        if (fields.get('state') != 'suspended' or fields.get('ready') != '1' or
                fields.get('imu') != '1' or fields.get('path_id') != self.owner.path_id):
            raise ValueError('物料任务期间导航控制权或航向已失效。')

    def tick(self, now):
        """Nonblocking station pump; camera and mechanism executors retain their guards."""
        if now-self.started > 1800:
            raise ValueError('物料站点任务超过30分钟。')
        if self.phase in ('handoff', 'handoff_ready'):
            if now-self.started > 60:
                raise ValueError('导航交接未就绪：60秒内未确认本段到站静止。')
            if not self.owner.pending and now-self.owner.last_query >= .1:
                self.owner.last_query = now
                if self.phase == 'handoff_ready':
                    self.phase = 'suspend'
                    self.start()
                else:
                    self.send('NAV STAT', 'handoff_status')
        elif self.phase == 'camera':
            camera = self.c.task_cameras.snapshot()
            if camera.get('token') != self.camera_token or camera.get('state') in ('error', 'fault'):
                raise ValueError(camera.get('error') or '摄像头1切换已取消。')
            if camera.get('state') == 'ready' and camera.get('slot') == '1':
                self.next_job()
        elif self.phase == 'align':
            alignment = self.c.camera_alignment
            if alignment.state in ('fault', 'stopped'):
                raise ValueError('物料视觉校准失败：'+alignment.error)
            if alignment.state == 'done':
                if self.station == 'B':
                    job = self.c.pick_place
                    if job.job_id != self.job_id or job.state != 'pickup_wait':
                        raise ValueError('B点待抓取动作已变化。')
                    if now-alignment.last_capture > .75:
                        self.owner.state = 'material_cycle'
                        self.align(self.current)
                        return
                    job.vision(dict(job.request, ready=True,
                        captured_at=alignment.vision['captured_at']), owner=self.owner)
                    self.phase = 'cycle'
                    self.owner.state = 'material_cycle'
                else:
                    seq = alignment.ring_sequence
                    if self.stacked:
                        accepted = seq.recognized[-1] if seq and seq.recognized else {}
                        if (accepted.get('point') != 2 or not accepted.get('aligned') or
                                accepted.get('identity_source') != 'remembered_color'):
                            raise ValueError('尚未完成第二次D点中间圆环校准。')
                        self.position = self.offsets[2]
                    else:
                        if not seq or [r['digit'] for r in seq.recognized] != [1] or not seq.recognized[0]['aligned']:
                            raise ValueError('尚未完成点1视觉校准。')
                        self.position = 0
                    self.next_job()
        elif self.phase == 'cycle':
            job = self.c.pick_place
            if job.job_id != self.job_id or job.state in ('fault', 'stopped'):
                raise ValueError('抓放动作失效：'+job.error)
            if job.state == 'pickup_wait':
                self.align(self.current)
            if job.state == 'done':
                if self.mode == 'pick':
                    self.owner.tray[self.current['slot']] = copy.deepcopy(self.current)
                else:
                    self.owner.tray.pop(self.current['slot'])
                    if self.station == 'D' and self.batch == 1:
                        self.owner.d_layout[self.current['target']] = copy.deepcopy(self.current)
                        self.owner.d_heading = self.options['arrival_heading_deg']
                self.next_job()
        elif self.phase == 'move':
            if now > self.deadline:
                raise ValueError('点间移动超时。')
            if self.move_id and now-self.owner.last_query >= .1:
                self.owner.last_query = now
                self.send('CHASSIS STAT', 'move_status')
        elif self.phase == 'arrived':
            self.after_move(self.after)
        if not self.owner.pending and now-self.owner.last_query >= .25:
            self.owner.last_query = now
            self.owner.send('NAV STAT', 'query')
