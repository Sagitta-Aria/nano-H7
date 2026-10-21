"""NX camera alignment, independent of photos and browser lifetime.

NX recognizes raw frames, computes two-axis speeds and mixes gyro correction
into four wheel targets. The existing serial owner sends expiring, latest-only
WHEELS commands for color targets. Ring mode uses bounded finite pulse search,
same-frame row alignment and ordered OCR through the existing board primitives.
"""
import copy
import json
import queue
import threading
import time
from collections import deque

from .camera_vision import VisionSettings
from .vision_alignment import VisualPID, camera_to_body, mix_wheels
from .ring_sequence import RingSequence
from .servo_config import wire_channel, wire_angle


class CameraAlignment:
    """Exclusive standalone visual servo. Public calls hold Console.lock."""
    RUNNING = ('zero', 'initializing', 'claw_opening', 'tracking', 'braking', 'settling', 'ring_sequence', 'pickup_wait')

    def __init__(self, console, path):
        self.console = console
        self.settings = VisionSettings(path)
        self.hub = None
        self.state = 'idle'
        self.generation = self.sequence = 0
        self.pending = None
        self.results = queue.Queue()
        self.capturing = False
        self.capture_token = 0
        self.resume_heading_required = False
        self.resume_old_heading_tag = None
        self.error = ''
        self.vision = self.preview_image = None
        self.stable = self.iterations = self.total = 0
        self.last_frame_id = 0
        self.frame_age_ms = None
        self.dropped_frames = 0
        self.stream_started = False
        self.vx = self.vy = self.yaw = 0
        self.wheels = [0]*4
        self.polarities = [0]*4
        self.ring_sequence = None
        self.velocity_revision = 0
        self.reset_velocity_feedback()
        self.velocity_trace = deque(maxlen=80)
        self.fault_context = None
        self.task_options = None

    def reset_velocity_feedback(self):
        """Invalidate stop evidence on a new stream or finite-to-servo handoff."""
        self.velocity_body = None
        self.zero_ack_revision = self.wheel_query_revision = None
        self.last_query_wheels = False

    def record_velocity_fault(self, body, reply):
        """Persist rejected command context before STOP erases motion intent.

        This is diagnostic evidence, not encoder feedback. Board snapshots may
        precede the rejection; the bounded trace shows host request/ACK order.
        """
        self.fault_context = dict(time=time.strftime('%Y-%m-%d %H:%M:%S'),
            command=body, reply=reply.body, state=self.state,
            sequence=self.ring_sequence.snapshot() if self.ring_sequence else None,
            vision=copy.deepcopy(self.vision), polarities=list(self.polarities),
            wheel_targets=list(self.wheels), velocity_revision=self.velocity_revision,
            zero_ack_revision=self.zero_ack_revision,
            armed=self.console.armed, chassis=copy.deepcopy(self.console.chassis),
            turn=copy.deepcopy(self.console.turn), motion=copy.deepcopy(self.console.motion),
            trace=list(self.velocity_trace))
        try:
            path = self.settings.path.parent/'logs'/'camera_alignment_faults.jsonl'
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('a', encoding='utf-8') as output:
                output.write(json.dumps(self.fault_context, ensure_ascii=False)+'\n')
        except OSError as error:
            self.fault_context['record_error'] = str(error)

    def record_row_event(self, event, **details):
        """Bounded stage evidence for image yaw -> board turn -> approach yaw.

        Record commanded-pulse and gyro estimates, not physical floor truth.
        Keep two files of at most roughly 1 MiB each; never record every frame.
        """
        c = self.console
        options = self.task_options or {}
        map_heading = c.navigation.get('heading_cd')
        arrival = options.get('arrival_heading_deg')
        map_error = ((arrival*100-int(map_heading)+18000) % 36000-18000
                     if arrival is not None and map_heading is not None else None)
        entry = dict(time=time.strftime('%Y-%m-%d %H:%M:%S'), event=event,
                     monotonic=time.monotonic(), generation=self.generation,
                     run_id=c.task.run_id, leg=c.task.leg,
                     frame_id=self.last_frame_id, capture_time=getattr(self, 'last_capture', None),
                     operation=self.ring_sequence.operation if self.ring_sequence else None,
                     operation_id=self.ring_sequence.operation_id if self.ring_sequence else None,
                     visual_tolerance_deg=self.config['ring_sequence']['angle_tolerance_deg'],
                     approach_tolerance_cd=getattr(self, 'heading_tolerance', None),
                     arrival_heading_deg=arrival, map_heading_error_cd=map_error,
                     phase=self.ring_sequence.phase if self.ring_sequence else None,
                     row=copy.deepcopy(self.ring_sequence.line) if self.ring_sequence else None,
                     imu=c.imu.navigation_feedback(), navigation=copy.deepcopy(c.navigation),
                     turn=copy.deepcopy(c.turn))
        entry.update(details)
        self.velocity_trace.append(entry)
        try:
            path = self.settings.path.parent/'logs'/'row_alignment.jsonl'
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists() and path.stat().st_size > 1024*1024:
                path.replace(path.with_suffix('.previous.jsonl'))
            with path.open('a', encoding='utf-8') as out:
                out.write(json.dumps(entry, ensure_ascii=False)+'\n')
        except OSError as error:
            c.note('warn', '圆列诊断记录失败：'+str(error))

    @property
    def active(self):
        return self.state in self.RUNNING

    def snapshot(self):
        return {'state': self.state, 'active': self.active, 'busy': self.active or self.capturing,
                'error': self.error, 'vision': copy.deepcopy(self.vision),
                'stable_frames': self.ring_sequence.stable if self.ring_sequence else self.stable,
                'iterations': self.iterations,
                'total_steps': self.ring_sequence.total if self.ring_sequence else 0,
                'tuning': getattr(self, 'tuning', False), 'has_image': False, 'image_revision': self.sequence, 'frame_age_ms': self.frame_age_ms,
                'dropped_frames': self.dropped_frames, 'streaming': self.stream_started,
                'velocity': {'right': self.vx, 'forward': self.vy, 'yaw': self.yaw},
                'wheel_targets': list(self.wheels),
                'fault_context': copy.deepcopy(self.fault_context),
                'ring_sequence': self.ring_sequence.snapshot() if self.ring_sequence else None,
                'control': ('pickup_color_wait' if self.task_options and self.task_options.get('pickup_only')
                            else 'ring_sequence' if self.ring_sequence else 'visual_pid_velocity')}

    def save(self, config):
        if self.console.task.active: raise ValueError('请先停止自动任务。')
        if self.active or self.capturing: raise ValueError('请先停止对准并等待识别完成。')
        return self.settings.save(config)

    def stop(self, message='已停止', fault=False, send=True):
        """Cancel the velocity lease before invalidating delayed frames/replies."""
        was_active = self.active
        if self.console.connection:
            self.console.connection.clear_camera_velocity()
        self.generation += 1
        self.pending = None
        self.capturing = False
        self.state = 'fault' if fault else 'stopped'
        self.error = message
        self.vx = self.vy = self.yaw = 0
        self.wheels = [0]*4
        self.end_stream()
        if was_active and send and self.console.connection:
            pickup = self.console.pick_place
            if (fault and pickup.active and pickup.owner is None and
                    pickup.context.get('profile') == 'b_pickup' and
                    (self.task_options or {}).get('job_id') == pickup.job_id):
                # Retain the camera fault in the owning test's status. Its
                # cancellation sends the same priority STOP to the board.
                pickup.fail('B点识别失败：'+message)
            else:
                self.console.submit('STOP')

    def end_stream(self):
        """Stop recognition, retaining the manual photo and local monitor."""
        if self.stream_started:
            self.stream_started = False
            self.hub.stop_tracking()

    def begin_stream(self, phase=None):
        """Start continuous NX raw perception with an independent generation."""
        self.generation += 1
        self.capturing = False
        self.reset_velocity_feedback()
        if phase is None:
            self.hub.start_tracking(self.config)
        else:
            self.hub.start_tracking(self.config, phase=phase)
        self.stream_started = True
        self.last_frame_id = self.dropped_frames = 0
        self.frame_age_ms = None
        self.capture_after = time.monotonic()
        self.vision_deadline = time.monotonic()+(15 if phase else 3)

    def send(self, body, purpose):
        """Serialize tagged setup/status requests; velocity uses its own lease."""
        self.sequence += 1
        tag = ('camera_align', self.generation, self.sequence)
        self.pending = (tag, purpose, time.monotonic())
        if purpose == 'wheel_status':
            self.wheel_query_revision = self.velocity_revision
        self.velocity_trace.append(dict(t=round(time.monotonic(),3), event='query',
                                        body=body, purpose=purpose, revision=self.velocity_revision))
        if not self.console.connection or not self.console.connection.submit(body, tag=tag):
            self.stop('串口队列已满或连接已断开。', True)

    def start(self, task_options=None, *, pickup_owner=None):
        """Start alignment, or B's lowered color wait under its mechanism owner.

        Alignment captures a stationary gyro zero. B recognition only consumes
        frames and never creates a wheel lease, zero command or PID motion.
        """
        c = self.console
        pickup_only = bool(task_options and task_options.get('pickup_only'))
        standalone_pickup = (pickup_only and pickup_owner is c.pick_place and
                             not c.task.active and c.pick_place.owner is None and
                             c.pick_place.context.get('profile') == 'b_pickup' and
                             c.pick_place.state == 'pickup_wait' and
                             task_options.get('job_id') == c.pick_place.job_id)
        pickup_owned = (pickup_only and c.task.active and c.task.material and
                        c.task.material.station == 'B' and c.task.state == 'material_cycle' and
                        c.pick_place.state == 'pickup_wait' and c.pick_place.owner is c.task and
                        c.pick_place.job_id == c.task.material.job_id and
                        c.navigation.get('path_id') == c.task.path_id) or standalone_pickup
        if pickup_only and not pickup_owned:
            raise ValueError('B点尚未下降到待抓取位置。')
        if getattr(c, 'pick_place', None) and c.pick_place.active and not pickup_owned:
            raise ValueError('请先停止抓取与放置。')
        if self.active or self.capturing: raise ValueError('正在识别或对准，请等待完成。')
        if not self.hub or self.hub.snapshot()['state'] != 'ready': raise ValueError('请先打开 NX 摄像头一。')
        if not c.connection or not c.armed: raise ValueError('请先连接控制板并使能车辆。')
        if c.device.get('turn') != '1': raise ValueError('固件需支持 H723 航向反馈。')
        owned = task_options is not None and c.task.active and c.navigation.get('state') == 'suspended'
        if task_options is not None and not (owned or standalone_pickup):
            raise ValueError('圆列子任务尚未取得挂起站点控制权。')
        if (c.task.active and not owned) or (getattr(c,'navigator',None) and c.navigator.active and not owned) or c.nav_pending or c.aux_pending or (not owned and any(a.get('active') == '1' for a in c.axes.values())):
            raise ValueError('请先停止其他任务和所有轴。')
        if (c.navigation.get('state') in ('turning','moving','orienting','waiting','nudging','blocked') or
                (c.navigation.get('state') == 'suspended' and not owned) or
                (not owned and (c.chassis.get('state') in ('moving','jogging','switching') or c.turn.get('state') == 'turning'))):
            raise ValueError('请先停止巡航和底盘任务；挂起站点由巡航适配器接入。')
        if c.motion.get('latched') == '1' or any(f.get('state') == 'fault' for f in (c.navigation,c.chassis,c.turn)):
            raise ValueError('请先排除并复位运动故障。')
        channel = wire_channel(c.device, 1)
        if channel > c.servo_count():
            raise ValueError('固件未提供 S1 舵机通道。')
        self.claw_command = 'SERVO {} {}'.format(channel, wire_angle(c.device, 1, 170))
        mechanism = c.pick_place.config if pickup_only else c.pick_place.settings.snapshot()
        self.claw_settle_s = mechanism['s1_settle_ms']/1000.
        self.claw_until = None
        self.generation += 1
        self.config = self.settings.snapshot()
        self.task_options = copy.deepcopy(task_options)
        if owned or standalone_pickup:
            self.config['kind'] = task_options.get('kind', 'ring')
            if self.config['kind'] == 'block':
                self.config['target'] = task_options['target']
        if pickup_only:
            self.config['pickup_roi_pct'] = list(task_options['pickup_roi_pct'])
        if self.config['kind'] == 'ring' and c.device.get('straight') != '1':
            raise ValueError('圆列搜索需要支持 CHASSIS STRAIGHT 的固件。')
        if task_options and task_options.get('stack_layout'):
            from .stack_ring import StackRingSequence
            self.config['stack_layout'] = copy.deepcopy(task_options['stack_layout'])
            self.ring_sequence = StackRingSequence(self)
        else:
            self.ring_sequence = RingSequence(self) if self.config['kind'] == 'ring' else None
        self.pid = VisualPID(self.config['tracking'])
        self.started = time.monotonic()
        self.resume_heading_required = False
        self.last_query = self.last_control = self.last_capture = self.last_result = self.heading_time = 0.
        self.error = ''; self.vision = None
        self.fault_context = None
        self.velocity_trace.clear()
        self.stable = self.iterations = self.total = 0
        self.vx = self.vy = self.yaw = 0
        self.wheels = self.polarities = [0]*4
        self.heading_error = 0
        self.heading_tolerance = 20
        self.heading_kp = 20.
        self.capturing = False
        if pickup_only:
            # The mechanism has verified its lowered endpoint and the station
            # owns a stopped chassis. Recognition never creates wheel commands.
            self.open_recognition_claw()
            return
        self.state = 'initializing'
        self.send('CHASSIS STAT', 'setup')

    def open_recognition_claw(self):
        """Open physical S1 to 170 before active recognition; never for previews.

        The caller owns a stopped chassis. The board ACK only accepts the PWM
        target, so tick also waits the mechanism's saved S1 settling interval.
        """
        self.state = 'claw_opening'
        self.send(self.claw_command, 'claw_open')

    def start_recognition(self):
        """Begin fresh perception only after S1's ACK and settling interval.

        B's lowered color gate acquires no chassis control. Active alignment
        records gyro zero after the claw has settled, before any wheel motion.
        """
        self.begin_stream()
        if self.task_options and self.task_options.get('pickup_only'):
            self.state = 'pickup_wait'
            self.capture()
        else:
            self.state = 'zero'
            self.send('TURN ZERO', 'zero')

    def recognize(self, image=None):
        """Preview one NX recognition result without acquiring motion control."""
        if self.active or self.capturing or self.console.task.active: raise ValueError('正在识别、对准或执行任务，请等待完成。')
        if image is not None: raise ValueError('识别由 NX 连续相机帧执行，网页照片仅供查看。')
        if not self.hub or self.hub.snapshot()['state'] != 'ready': raise ValueError('请先打开 NX 摄像头一。')
        self.generation += 1
        self.task_options = None
        self.config = self.settings.snapshot()
        self.ring_sequence = None
        self.error = ''; self.vision = None
        self.stable = self.iterations = self.total = 0
        self.state = 'previewing'
        self.begin_stream()
        self.capture()

    def capture(self, image=None):
        """Wait off-pump for the next unique local detection, without a shutter."""
        if self.capturing: return
        self.capturing = True
        generation, after, frame_id = self.generation, self.capture_after, self.last_frame_id
        token = self.capture_token
        first_digit_frame = self.ring_sequence and self.ring_sequence.phase == 'seek' and frame_id == 0
        pickup_wait = bool(self.task_options and self.task_options.get('pickup_only'))
        def work():
            try:
                # B cannot move the chassis: silence only pauses its open claw.
                # Hub cancellation/disconnect still wakes an unbounded waiter.
                report = (self.hub.tracking_sample(after, frame_id, timeout=None) if pickup_wait else
                          self.hub.tracking_sample(after, frame_id, timeout=15) if first_digit_frame
                          else self.hub.tracking_sample(after, frame_id))
                with self.console.lock:
                    if token == self.capture_token:
                        self.results.put((generation, report, None, None))
            except Exception as error:
                with self.console.lock:
                    if token == self.capture_token:
                        self.results.put((generation, None, None, str(error)))
        threading.Thread(target=work, name='camera-one-detection', daemon=True).start()

    def reply(self, body, reply, tag):
        """Validate velocity ACKs separately from tagged gyro/braking status."""
        if not self.active: return
        if tag and tag[:2] == ('camera_velocity', self.generation):
            self.velocity_trace.append(dict(t=round(time.monotonic(),3), event='velocity_ack',
                                            body=body, reply=reply.body, tag=list(tag)))
            if not reply.ok:
                self.record_velocity_fault(body, reply)
                self.stop('四轮速度被拒绝：'+reply.body, True)
            elif (tag == ('camera_velocity', self.generation, self.velocity_revision)
                  and body == self.velocity_body and not any(self.wheels)):
                self.zero_ack_revision = self.velocity_revision
            return
        if not self.pending or tag != self.pending[0]: return
        purpose = self.pending[1]; self.pending = None
        self.velocity_trace.append(dict(t=round(time.monotonic(),3), event='query_ack',
                                        body=body, purpose=purpose, reply=reply.body,
                                        revision=self.wheel_query_revision if purpose == 'wheel_status' else None))
        if not reply.ok: self.stop('对准命令被拒绝：'+reply.body, True); return
        f = reply.fields
        if purpose.startswith('ring_') and self.ring_sequence:
            self.ring_sequence.reply(purpose, f)
            return
        if purpose == 'setup':
            if int(f.get('active', '15')): self.stop('底盘仍在运动。', True); return
            self.invert = int(f['invert'])
            self.open_recognition_claw()
        elif purpose == 'claw_open':
            self.claw_until = time.monotonic()+self.claw_settle_s
        elif purpose == 'zero':
            self.send('TURN STAT', 'heading')
        elif purpose == 'heading':
            if self.resume_heading_required and tag == self.resume_old_heading_tag:
                self.resume_old_heading_tag = None
                self.send('TURN STAT', 'heading')
                return
            if f.get('imu') != '1' or f.get('zero') != '1' or f.get('active') != '0' or f.get('fault','none') != 'none':
                self.stop('陀螺仪反馈失效或航向控制被占用。', True); return
            self.heading_error = int(f['error_cd'])
            self.heading_tolerance = int(f.get('tolerance_cd','20'))
            if self.ring_sequence:
                self.heading_tolerance = min(self.heading_tolerance, self.ring_sequence.turn_band())
            self.heading_kp = int(f.get('kp_centi','2000'))/100.
            self.heading_time = time.monotonic()
            self.resume_heading_required = False
            if self.state == 'zero':
                if self.ring_sequence and self.ring_sequence.phase != 'approach':
                    self.ring_sequence.begin()
                else:
                    self.state = 'tracking'; self.capture()
        elif purpose == 'wheel_status' and int(f.get('active','15')) == 0:
            # An idle snapshot can arrive after a new frame resumed motion.
            # Release directions only for this exact acknowledged zero intent.
            if (self.state not in ('braking','settling') or any(self.wheels)
                    or self.zero_ack_revision != self.velocity_revision
                    or self.wheel_query_revision != self.velocity_revision):
                return
            self.polarities = [0]*4
            if (self.state == 'braking' and self.ring_sequence and
                    self.ring_sequence.phase == 'realign_brake'):
                self.ring_sequence.resume_row_alignment()
                return
            if self.state == 'braking': self.state = 'tracking'
            elif self.state == 'settling' and self.stable >= 3 and abs(self.heading_error) <= self.heading_tolerance:
                self.console.connection.clear_camera_velocity()
                if self.ring_sequence and self.ring_sequence.phase == 'approach':
                    if not self.last_result or time.monotonic()-self.last_result > .75:
                        self.stable = 0
                        return
                    self.ring_sequence.finish_approach()
                    return
                self.generation += 1  # The outstanding frame waiter is canceled below.
                self.capturing = False
                self.wheels = [0]*4
                self.state = 'done'; self.end_stream()

    def consume(self, report):
        """Update both PID axes from new raw frames, including while driving."""
        now = time.monotonic()
        captured, frame_id = report.get('captured_monotonic',0), report.get('frame_id',0)
        self.frame_age_ms = round((now-captured)*1000,1)
        if frame_id <= self.last_frame_id or captured < self.capture_after or not 0 <= now-captured <= .75:
            self.last_frame_id = max(self.last_frame_id,frame_id)
            self.dropped_frames += 1; self.stable = 0
            if self.ring_sequence:
                self.ring_sequence.stable = 0
                self.ring_sequence.lost_line = 0
                self.ring_sequence.previous_center = None
            return
        if self.ring_sequence:
            if report.get('kind') != 'ring' or report.get('target') != 'ring':
                self.stop('圆列识别结果类型不匹配。', True); return
            self.last_frame_id = frame_id
            self.last_capture = captured; self.last_result = now
            self.vision_deadline = now+3
            self.iterations += 1
            self.ring_sequence.consume(report)
            return
        expected = self.config['target'] if self.config['kind'] == 'block' else 'ring'
        if report.get('kind') != self.config['kind'] or report.get('target') != expected:
            self.stop('识别结果与当前目标不匹配。',True); return
        if self.state == 'pickup_wait':
            from .camera_vision import pickup_roi_bounds
            try:
                roi = pickup_roi_bounds(report['width'], report['height'], self.config['pickup_roi_pct'])
                center = report.get('center')
                inside = (report.get('found') and center is not None and
                          roi[0] <= center[0] < roi[0]+roi[2] and
                          roi[1] <= center[1] < roi[1]+roi[3] and list(report.get('roi', [])) == roi)
            except (KeyError, TypeError, ValueError, IndexError):
                self.stop('B点识别结果缺少有效的画面或识别框坐标。', True)
                return
            if captured <= self.last_capture or captured-self.last_capture > .75:
                self.stable = 0
            self.stable = self.stable+1 if inside else 0
            self.last_frame_id, self.last_capture = frame_id, captured
            self.vision_deadline = now+3
            self.iterations += 1
            if self.stable >= 3:
                self.generation += 1
                self.capturing = False
                self.state = 'done'
                self.end_stream()
            return
        if not report.get('found'):
            if self.task_options and self.config['kind'] == 'block':
                # A rotating supply may not yet show the requested color.
                # Fresh negative frames brake translation; they never authorize
                # a pick. B may wait indefinitely while fresh frames arrive.
                self.last_frame_id = frame_id
                self.last_capture = captured
                self.last_result = 0
                self.vision_deadline = now+3
                self.stable = 0
                self.vx = self.vy = 0
                self.pid = VisualPID(self.config['tracking'])
                if self.state != 'braking':self.state = 'tracking'
                return
            self.stop('未找到指定目标，车辆对准已停止。',True); return
        self.last_frame_id = frame_id
        self.last_capture = captured; self.last_result = now
        self.vision_deadline = now+3
        self.servo_sample(report)

    def servo_sample(self, report):
        """Update the shared velocity servo after the caller verifies target identity."""
        try:
            camera_right, camera_forward = self.pid.update(report['error_dx'],report['error_dy'],report['captured_monotonic'],
                latency=max(0., time.monotonic()-report['captured_monotonic']))
            self.vx,self.vy = camera_to_body(camera_right, camera_forward, self.config['tracking']['camera_yaw_deg'])
        except (ValueError,KeyError) as error: self.stop(str(error),True); return
        self.iterations += 1
        # A predictive brake may request zero before reaching the target. Only
        # measured pixel error, followed by the idle barrier, proves alignment.
        aligned = max(abs(report['error_dx']),abs(report['error_dy'])) <= self.config['tracking']['deadzone_px']
        self.stable = self.stable+1 if aligned else 0
        if self.state != 'braking': self.state = 'settling' if aligned else 'tracking'

    def control(self, now):
        """Run heading/mixing at 20 Hz and renew the serial motion lease.

        Reversal first ramps all wheels to zero and confirms actual idle status.
        Loss of new visual results brakes before the 3-second stream fault.
        """
        t = self.config['tracking']
        if self.heading_time and now-self.heading_time > .6:
            self.stop('陀螺仪航向查询超时。',True); return
        moving = (not self.resume_heading_required and self.last_result and
                  now-self.last_result <= .75 and now-self.last_capture <= 1.2)
        yaw = 0 if abs(self.heading_error) <= self.heading_tolerance else self.heading_error*self.heading_kp/100.
        self.yaw = round(max(-600,min(600,yaw))) if moving else 0
        wanted = mix_wheels(self.vx if moving else 0,self.vy if moving else 0,self.yaw,self.invert,t['sps'])
        restart_after_zero = (any(wanted) and any(self.polarities) and self.velocity_body
                              and self.velocity_body.startswith('WHEELS 0 0 0 0 '))
        # Once zero was requested, even same-direction motion must wait for
        # actual idle: firmware may still be finishing a high STEP pulse.
        if self.state != 'braking' and (restart_after_zero or any(old*new < 0 for old,new in zip(self.polarities,wanted))):
            self.state = 'braking'
        if self.state == 'braking': wanted = [0]*4
        for i,value in enumerate(wanted):
            if value: self.polarities[i] = 1 if value > 0 else -1
        self.wheels = wanted
        body = 'WHEELS {} {} {} {} {}'.format(*wanted,t['accel'])
        if body != self.velocity_body:
            self.velocity_revision += 1
            self.velocity_body = body
            self.zero_ack_revision = None
            self.velocity_trace.append(dict(t=round(now,3), event='velocity_intent',
                                            body=body, revision=self.velocity_revision,
                                            state=self.state, polarities=list(self.polarities)))
        self.console.connection.set_camera_velocity(body,('camera_velocity',self.generation,self.velocity_revision))

    def tick(self):
        """Nonblocking NX executor: receive vision, regulate, and poll feedback."""
        now = time.monotonic()
        if self.active and (not self.console.connection or not self.console.armed):
            self.stop('连接断开或车辆失去使能。',True)
        if self.active and self.console.motion.get('latched') == '1':
            self.stop('运动故障已锁存。',True)
        timeout = 300 if self.ring_sequence else 60
        waiting_for_color = (self.task_options and self.task_options.get('wait_for_color')
                             and self.config['kind'] == 'block')
        if self.active and not waiting_for_color and now-self.started > timeout:
            self.stop('视觉流程超过 {} 秒。'.format(timeout),True)
        if (self.active and self.ring_sequence and
                self.ring_sequence.phase in ('approach', 'realign_brake') and now > self.ring_sequence.deadline):
            self.stop('点位 {} 靠近校准超过 30 秒。'.format(self.ring_sequence.expected), True)
        if self.state in ('initializing', 'claw_opening'):
            if self.pending and now-self.pending[2] > 2:
                self.stop('对准串口应答超时。', True)
            elif self.state == 'claw_opening' and self.claw_until is not None and now >= self.claw_until:
                self.start_recognition()
            return
        try: generation,report,jpeg,error = self.results.get_nowait()
        except queue.Empty: pass
        else:
            if generation == self.generation:
                self.capturing = False
                if error and (self.active or self.state == 'previewing'): self.stop(error,True)
                elif error: pass
                else:
                    self.vision = report; self.sequence += 1
                    if self.state == 'previewing':
                        self.state = 'preview'; self.end_stream()
                    elif self.active: self.consume(report)
        if not self.active: return
        if self.state == 'ring_sequence':
            if self.pending and now-self.pending[2] > 2:
                self.stop('圆列串口应答超时。', True); return
            if (self.ring_sequence.phase not in ('dwell', 'spacing', 'complete') and
                    not self.ring_sequence.operation and now > self.vision_deadline):
                self.stop('NX 圆列识别超过 3 秒没有新鲜结果。', True); return
            self.ring_sequence.tick(now)
            return
        if self.state == 'pickup_wait':
            if now-self.last_capture > .75: self.stable = 0
            if not self.capturing: self.capture()
            return
        if now > self.vision_deadline:
            self.stop('NX 连续识别超过 3 秒没有新鲜结果。',True); return
        if self.state in ('tracking','braking','settling'):
            if not self.capturing: self.capture()
            if now-self.last_control >= .05:
                self.last_control = now; self.control(now)
                if not self.active: return
        if self.pending:
            if now-self.pending[2] > 2: self.stop('对准串口应答超时。',True)
            return
        if now-self.last_query >= .1 and self.state in ('tracking','braking','settling'):
            self.last_query = now
            # Alternate while braking/settling so gyro feedback remains fresh.
            # Alternate actual queries, not frame parity: two frames per poll
            # can otherwise keep every poll odd and starve idle confirmation.
            wheel_query = (self.state in ('braking','settling') and not self.last_query_wheels
                           and self.zero_ack_revision == self.velocity_revision
                           and not any(self.wheels))
            self.last_query_wheels = wheel_query
            self.send('CHASSIS STAT' if wheel_query else 'TURN STAT','wheel_status' if wheel_query else 'heading')
