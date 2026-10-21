"""Bounded circle-row search and ordered OCR using existing finite board moves.

Fresh frames can stop a search in progress; turns require stationary confirmation.
Two circles must coexist in one frame; different vehicle poses are never mixed.
Pulse positions are commanded travel estimates, never encoder measurements.
"""
import copy
import math
import time

from .vision_alignment import VisualPID


def row_line(centers, camera_yaw_deg):
    """Return the longest same-frame baseline and the nearest CW body yaw.

    Use for a camera view whose image axes map to the floor plane. Installation
    yaw alone does not calibrate perspective distortion or a tilted floor view.
    An unoriented line chooses at most 90 degrees, preserving the initial forward
    direction; the operator's constraint places digit 1 behind the vehicle.
    """
    if len(centers) < 2:
        return None
    pairs = [(a, b) for i, a in enumerate(centers) for b in centers[i+1:]]
    a, b = max(pairs, key=lambda pair: math.dist(*pair))
    dx, dy = b[0]-a[0], b[1]-a[1]
    if math.hypot(dx, dy) < 20:
        return None
    angle = math.radians(camera_yaw_deg)
    right = math.cos(angle)*dx+math.sin(angle)*dy
    forward = math.sin(angle)*dx-math.cos(angle)*dy
    error = (math.degrees(math.atan2(right, forward))+90) % 180-90
    return {'points': [list(a), list(b)], 'error_deg': error}


class RingSequence:
    """Own the ring branch of CameraAlignment, under the same Console lock.

    Only acknowledged finite CHASSIS STRAIGHT / TURN MOVE commands are issued.
    Each completion needs its operation id and idle status before observing again.
    Stop/disarm and serial deadlines remain owned by the parent; active moves
    use the board's gyro freshness guards.
    """
    def __init__(self, owner):
        self.owner = owner
        self.config = owner.config['ring_sequence']
        self.phase = 'search'
        self.position = self.total = self.leg = self.stable = 0
        self.expected = 1
        self.recognized = []
        self.line = None
        self.previous_angle = None
        self.operation = self.operation_id = None
        self.halt_requested = self.halt_sent = False
        self.lost_line = 0
        self.last_direction = 0
        self.advance_pending = False
        self.observations = 0
        self.previous_center = None
        self.pending_target = None
        self.dwell_until = 0.
        self.visible_digits = []
        self.target_last_seen = 0.
        self.after = time.monotonic()
        self.deadline = self.after+30
        self.task_options = getattr(owner, 'task_options', None)
        self.outward_used = self.returning = False

    def turn_band(self):
        """Use half the visual band, in centidegrees, for convergence margin."""
        return max(1, min(1000, math.floor(self.config['angle_tolerance_deg']*50)))

    def snapshot(self):
        """Expose phase, search limits, line and accepted order for the UI."""
        return dict(phase=self.phase, expected=self.expected, position_steps=self.position,
                    range_steps=self.config['range_steps'], total_steps=self.total,
                    recognized=copy.deepcopy(self.recognized), line=copy.deepcopy(self.line),
                    visible_digits=copy.deepcopy(self.visible_digits),
                    returning=self.returning, outward_used=self.outward_used,
                    spacing_steps=self.config['spacing_steps'],
                    wait_remaining_s=round(max(0., self.dwell_until-time.monotonic()), 1)
                    if self.phase == 'dwell' else 0.)

    def begin(self):
        """Enter geometry search after the parent has verified the stationary IMU."""
        self.owner.state = 'ring_sequence'
        self.owner.capture_after = self.after = time.monotonic()
        self.owner.capture()

    def move(self, amount):
        """Issue a finite pulse slice; never extrapolate travel from elapsed time."""
        self.operation = ('move', amount)
        self.last_direction = 1 if amount > 0 else -1
        self.operation_id = None
        self.halt_requested = self.halt_sent = False
        self.owner.capture_after = self.after = time.monotonic()
        self.deadline = time.monotonic()+abs(amount)/self.config['sps']+8
        self.owner.send('CHASSIS STRAIGHT {} {} {} {}'.format(
            'FWD' if amount > 0 else 'BACK', abs(amount), self.config['sps'],
            self.owner.config['tracking']['accel']), 'ring_accept')

    def turn(self, error):
        """Rotate a bounded increment using the existing H723 closed-loop turn."""
        limit = self.config['max_turn_deg']
        angle = round(max(-limit, min(limit, error))*100)
        self.operation = ('turn', angle)
        self.operation_id = None
        self.phase = 'align'
        self.deadline = time.monotonic()+30
        body = 'TURN MOVE REL {} {} {}'.format(
            angle, self.config['sps'], self.owner.config['tracking']['accel'])
        console = self.owner.console
        if console.device.get('turn_band') == '1':
            body += ' {}'.format(self.turn_band())
        elif int(console.turn.get('tolerance_cd', '1000')) >= self.config['angle_tolerance_deg']*100:
            self.owner.stop('底盘转向容差大于圆列要求，需更新支持独立转向容差的固件。', True)
            return
        self.owner.record_row_event('turn_request', requested_cd=angle, command=body)
        self.owner.send(body, 'ring_accept')

    def outward(self):
        """Move one configurable slice toward the nearest field boundary, then retry."""
        self.outward_used = True
        amount = self.task_options['outward_steps']
        if not amount:
            self.owner.stop('圆列搜索失败，外移脉冲配置为 0。', True)
            return
        self.phase = 'outward'
        self.operation = ('outward', amount)
        self.operation_id = None
        self.halt_requested = self.halt_sent = False
        self.deadline = time.monotonic()+amount/self.config['sps']+8
        self.owner.send('CHASSIS STRAIGHT {} {} {} {}'.format(
            self.task_options['outward_direction'], amount, self.config['sps'],
            self.owner.config['tracking']['accel']), 'ring_accept')

    def reply(self, purpose, fields):
        """Accept only this operation's completed, fault-free status snapshot."""
        if purpose == 'ring_accept':
            self.operation_id = fields.get('move_id')
            if self.operation_id is None:
                self.owner.stop('圆列运动缺少操作编号。', True)
            return
        if purpose == 'ring_halt':
            self.halt_sent = True
            return
        if purpose != 'ring_status' or not self.operation:
            return
        if fields.get('fault', 'none') != 'none':
            self.owner.stop('圆列运动故障：'+fields['fault'], True)
            return
        if self.operation[0] == 'turn' and (fields.get('imu') != '1' or fields.get('zero') != '1'):
            self.owner.stop('圆列转向的陀螺仪反馈失效。', True)
            return
        if fields.get('move_id') != self.operation_id:
            self.owner.stop('圆列运动编号发生变化。', True)
            return
        interrupted = self.halt_sent and fields.get('state') == 'stopped'
        if fields.get('state') == 'fault' or (fields.get('state') == 'stopped' and not interrupted):
            self.owner.stop('圆列运动被中断。', True)
            return
        if (fields.get('state') != 'done' and not interrupted) or int(fields.get('active', '15')):
            return
        kind, amount = self.operation
        if kind == 'turn':
            self.owner.record_row_event('turn_complete', requested_cd=amount, status=dict(fields))
        if kind == 'move':
            if self.phase == 'spacing' and (interrupted or fields.get('remaining') != '0'):
                self.owner.stop('点间定距移动尚未完成。', True); return
            remaining = int(fields.get('remaining', '-1')) if interrupted else 0
            if not 0 <= remaining <= abs(amount):
                self.owner.stop('中断搜索的剩余脉冲无效。', True); return
            traveled = (1 if amount > 0 else -1)*(abs(amount)-remaining)
            self.position += traveled
            self.total += abs(traveled)
        self.operation = self.operation_id = None
        self.halt_requested = self.halt_sent = False
        self.stable = 0
        self.lost_line = 0
        self.previous_angle = None
        self.observations = 0
        self.previous_center = None
        # Completion already confirms stationary wheels. Require a new frame,
        # without discarding another 100 ms of usable stationary observations.
        self.owner.capture_after = self.after = time.monotonic()
        self.owner.vision_deadline = self.after+3
        self.deadline = self.after+30
        if kind == 'move' and self.phase == 'spacing':
            # This is a pulse-positioned station, not a new OCR observation.
            self.recognized.append(dict(digit=self.expected, aligned=True,
                positioning='fixed_spacing', position_steps=self.position,
                center=None, frame_id=None))
            self.finish_position()
            return
        if kind == 'move' and self.phase == 'advance':
            self.phase = 'align'
            self.line = None
        if kind == 'outward':
            self.total += amount
            self.position = self.leg = 0
            self.phase = 'search'

    def recover_line(self):
        """Restart a local bounded sweep after three fresh missing-row frames.

        A running turn must first halt and report idle using its original id.
        Rebase pulse estimates because a changed heading invalidates the old
        search axis. Only a later forward sighting can release the 500-step move.
        """
        self.phase = 'recover'
        self.position = self.leg = self.stable = self.lost_line = 0
        self.last_direction = 0
        self.line = self.previous_angle = None
        self.advance_pending = False
        if self.operation:
            self.halt_requested = True
        else:
            self.next_search()

    def next_search(self):
        """Sweep 0 -> -X -> +X -> 0 once, then fail without further movement."""
        extent = self.config['range_steps']
        targets = (-extent, extent, 0)
        while self.leg < len(targets) and self.position == targets[self.leg]:
            self.leg += 1
        if self.leg == len(targets):
            if self.task_options and self.phase == 'search' and not self.outward_used:
                self.outward()
                return
            self.owner.stop('搜索范围内未同时找到两个圆，请调整 X 或摄像头视野。', True)
            return
        delta = targets[self.leg]-self.position
        self.move((1 if delta > 0 else -1)*min(abs(delta), self.config['scan_step']))

    def consume(self, report):
        """Stop on moving-frame evidence; make all turn/OCR decisions at rest."""
        if report['captured_monotonic'] < self.after:
            return
        self.visible_digits = copy.deepcopy(report.get('digits', []))
        # Keep the displayed line tied to the current frame, including OCR and
        # approach. A previously accepted heading is not permanent evidence.
        self.line = copy.deepcopy(report.get('row_line'))
        if self.line:
            error = self.line.get('error_deg')
            if not isinstance(error, (int, float)) or not math.isfinite(error) or abs(error) > 90:
                self.owner.stop('圆列方向无效。', True)
                return
        if self.phase == 'realign_brake':
            self.owner.last_result = 0.
            return
        if self.phase == 'approach':
            self.approach_sample(report)
            return
        if self.phase in ('dwell', 'spacing', 'complete'):
            return
        if self.operation:
            if self.halt_requested or self.phase in ('advance', 'outward'):
                return
            line = report.get('row_line')
            if self.operation[0] == 'turn':
                self.lost_line = 0 if line else self.lost_line+1
                if self.lost_line >= 3:
                    self.recover_line()
                return
            found = line is not None if self.phase == 'search' else (
                (self.phase == 'recover' and self.last_direction > 0 and line is not None) or
                (self.phase == 'seek' and report.get('found') is True and report.get('digit') == str(self.expected)))
            self.stable = self.stable+1 if found else 0
            if self.stable >= 3:
                self.halt_requested = True
                if self.phase == 'search':
                    self.phase = 'align'
                elif self.phase == 'recover':
                    self.advance_pending = True
            return
        if self.phase == 'recover':
            # Never accept a backward sighting. The forward sweep must collect
            # three new frames; its stopped pose then starts a full 500 pulses.
            if self.advance_pending:
                return
            line = report.get('row_line')
            if self.last_direction > 0 and self.leg == 1 and line:
                self.stable += 1
                if self.stable >= 3:
                    self.advance_pending = True
                return
            self.stable = 0
            self.next_search()
            return
        if self.phase == 'advance':
            return
        if self.phase in ('search', 'align'):
            line = report.get('row_line')
            self.line = copy.deepcopy(line)
            if not line:
                self.stable = 0
                self.previous_angle = None
                if self.phase == 'search':
                    self.next_search()
                else:
                    self.lost_line += 1
                    if self.lost_line >= 3:
                        self.recover_line()
                return
            self.lost_line = 0
            error = line['error_deg']
            if not math.isfinite(error) or abs(error) > 90:
                self.owner.stop('圆列方向无效。', True)
                return
            difference = abs((error-self.previous_angle+90) % 180-90) if self.previous_angle is not None else 180
            band = self.config['angle_tolerance_deg']
            # A stable outside-band frame cannot count toward the three
            # inside-band confirmations that authorize leaving alignment.
            same_band = (self.previous_angle is not None and
                         (abs(error) <= band) == (abs(self.previous_angle) <= band))
            self.stable = self.stable+1 if same_band and difference <= band else 1
            self.previous_angle = error
            if self.stable < 3:
                return
            self.stable = 0
            if abs(error) > self.config['angle_tolerance_deg']:
                self.turn(error)
                return
            self.phase = 'seek'
            self.owner.record_row_event('row_aligned', frame_id=report['frame_id'])
            self.position = 0
            self.switch_digit()
            return
        if self.line and abs(self.line['error_deg']) > self.config['angle_tolerance_deg']:
            # The stationary OCR pose may have drifted since initial geometry.
            # Require three new geometry frames before another finite turn.
            self.phase = 'align'
            self.stable = self.observations = 0
            self.previous_angle = self.previous_center = None
            return
        matched = report.get('digit') == str(self.expected) and report.get('found') is True
        self.observations += 1
        same_ring = (matched and self.previous_center is not None and
                     math.dist(report['center'], self.previous_center) <= 30)
        self.stable = self.stable+1 if same_ring else (1 if matched else 0)
        self.previous_center = list(report['center']) if matched else None
        if matched:
            # Three independent, stationary OCR frames must agree. Other digits
            # do not advance the order, even if more than one ring is visible.
            if self.stable < 3:
                return
            accepted = dict(digit=self.expected, center=list(report['center']),
                            frame_id=report['frame_id'], position_steps=self.position)
            self.start_approach(accepted)
            return
        self.stable = 0
        if self.observations < 5:
            return
        remaining = self.config['range_steps']-abs(self.position)
        if remaining <= 0:
            self.owner.stop('在 X 脉冲范围内未确认数字 {}。'.format(self.expected), True)
            return
        self.move((-1 if self.expected == 1 or self.returning else 1)*min(remaining, self.config['scan_step']))

    def start_approach(self, accepted):
        """Keep the expected digit selected and reuse the pixel/gyro servo.

        Called only after three stationary OCR confirmations. Zero the gyro
        while idle so approach holds the row heading established by TURN REL.
        A successful OCR alone never marks a station complete.
        """
        self.pending_target = accepted
        self.owner.record_row_event('approach_begin', target=copy.deepcopy(accepted))
        self.phase = 'approach'
        self.owner.pid = VisualPID(self.owner.config['tracking'])
        self.owner.reset_velocity_feedback()
        self.owner.stable = self.stable = 0
        self.owner.vx = self.owner.vy = self.owner.yaw = 0
        self.owner.state = 'zero'
        self.target_last_seen = time.monotonic()
        self.deadline = self.target_last_seen+30
        self.owner.capture_after = self.after = self.target_last_seen
        self.owner.send('TURN ZERO', 'zero')

    def approach_sample(self, report):
        """Approach only the expected digit; lost identity commands zero velocity."""
        if self.line and abs(self.line['error_deg']) > self.config['angle_tolerance_deg']:
            # Retire translation before transferring to finite TURN control.
            # The parent's acknowledged-zero/idle barrier invokes resume below.
            self.owner.record_row_event('realign_trigger', frame_id=report['frame_id'],
                                        heading_error_cd=self.owner.heading_error)
            self.phase = 'realign_brake'
            self.owner.vx = self.owner.vy = self.owner.yaw = 0
            self.owner.last_result = 0.
            self.owner.stable = self.stable = 0
            self.owner.state = 'braking'
            return
        if not self.matches(report):
            self.owner.vx = self.owner.vy = self.owner.yaw = 0
            self.owner.last_result = 0.
            self.owner.stable = self.stable = 0
            if self.owner.state != 'braking':
                self.owner.state = 'tracking'
            if time.monotonic()-self.target_last_seen > 3:
                self.owner.stop('靠近时丢失点位身份 {}，已停车。'.format(self.expected), True)
            return
        self.target_last_seen = time.monotonic()
        self.owner.servo_sample(report)
        self.stable = self.owner.stable

    def matches(self, report):
        """The normal ring sequence requires actual OCR of its expected digit."""
        return report.get('digit') == str(self.expected) and report.get('found') is True

    def resume_row_alignment(self):
        """Resume geometry only after this velocity revision is ACKed and idle.

        Called by CameraAlignment's braking barrier, never by a camera frame.
        New stationary frames decide the turn; the drift frame cannot start it.
        """
        self.owner.console.connection.clear_camera_velocity()
        self.owner.state = 'ring_sequence'
        self.phase = 'align'
        self.pending_target = None
        self.stable = self.lost_line = self.observations = 0
        self.previous_angle = self.previous_center = None
        self.owner.capture_after = self.after = time.monotonic()
        self.owner.vision_deadline = self.after+3
        self.deadline = self.after+30

    def finish_approach(self):
        """Anchor point 1 after visual alignment and idle, then retire perception."""
        self.owner.record_row_event('approach_complete', frame_id=self.owner.last_frame_id,
                                    heading_error_cd=self.owner.heading_error)
        self.pending_target.update(aligned=True, center=list(self.owner.vision['center']),
                                frame_id=self.owner.last_frame_id, positioning='vision')
        self.recognized.append(self.pending_target)
        self.pending_target = None
        self.position = 0  # Subsequent pulse offsets are relative to calibrated 1.
        self.owner.generation += 1
        self.owner.capturing = False
        self.owner.end_stream()
        self.finish_position()

    def finish_position(self):
        """Advance an accepted station; finish at 3, or at returned 2 in cruise.

        Call only after visual idle at 1 or full finite-motion completion at
        later points. Fixed-spacing points do not require OCR or visual PID.
        """
        self.stable = 0
        self.owner.state = 'ring_sequence'
        self.owner.vx = self.owner.vy = self.owner.yaw = 0
        self.owner.wheels = [0]*4
        if self.expected == 1 and self.task_options and self.task_options.get('anchor_only'):
            self.phase = 'complete'
            self.owner.state = 'done'
            return
        if self.expected == 3:
            if self.task_options:
                self.returning = True
                self.phase = 'dwell'
                self.dwell_until = time.monotonic()
                self.deadline = self.dwell_until+30
                return
        if self.expected == 3 or self.returning:
            self.phase = 'complete'
            self.owner.state = 'done'
            self.owner.generation += 1
            self.owner.capturing = False
            self.owner.end_stream()
            return
        self.phase = 'dwell'
        self.dwell_until = time.monotonic()
        self.deadline = self.dwell_until+30

    def switch_digit(self):
        """Start a fresh OCR stream for the expected digit, discarding old votes."""
        self.owner.end_stream()
        self.owner.begin_stream(str(self.expected))
        self.observations = 0
        self.previous_center = None
        self.after = self.owner.capture_after
        self.deadline = time.monotonic()+30

    def tick(self, now):
        """Poll finite motion, or obtain fresh stationary perception without blocking."""
        if now > self.deadline:
            self.owner.stop('圆列阶段超时：'+self.phase, True)
            return
        if self.phase == 'dwell' and now >= self.dwell_until:
            self.expected = 2 if self.returning else self.expected+1
            self.phase = 'spacing'
            self.stable = 0
            self.move((-1 if self.returning else 1)*self.config['spacing_steps'])
        if (self.advance_pending and not self.operation and not self.owner.pending
                and now >= self.after):
            self.advance_pending = False
            self.phase = 'advance'
            self.line = None
            self.move(500)
        if self.operation:
            if self.halt_requested and not self.halt_sent and not self.owner.pending:
                self.owner.send('CHASSIS HALT', 'ring_halt')
                return
            if not self.owner.pending and now-self.owner.last_query >= .02:
                self.owner.last_query = now
                self.owner.send('TURN STAT' if self.operation[0] == 'turn' else 'CHASSIS STAT', 'ring_status')
        if (self.phase not in ('dwell', 'spacing', 'complete') and
                now >= self.after and not self.owner.capturing):
            self.owner.capture()
