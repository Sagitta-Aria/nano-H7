"""Continuous A-point QR search within fixed bounds using finite chassis moves.

NX owns the sweep and return; existing MCU SUSPEND/POSE/RESTORE primitives
preserve navigation ownership. Positions are pulse estimates, not encoders.
"""
import time

SPEED = 2500
EXTENT = 3200
STEP = 800
DWELL = 1.5


class QRSearch:
    """Repeat 0 -> -3200 -> +3200 -> 0 until code lock or operator STOP."""
    def __init__(self, owner):
        self.owner = owner
        self.phase = 'watch'
        self.position = self.leg = 0
        self.suspended = self.returning = self.halt = False
        self.amount = 0
        self.move_id = None
        self.last_status = 0.
        self.started = time.monotonic()
        self.deadline = self.started + DWELL
        self.last_frame = None
        self.origin = [int(owner.console.navigation[k]) for k in ('x', 'y')]

    def snapshot(self):
        """Expose bounded commanded travel and phase without querying hardware."""
        return dict(phase=self.phase, position_steps=self.position, speed=SPEED,
                    range_steps=EXTENT, step=STEP, returning=self.returning)

    def sighting(self, valid, captured):
        """A moving-frame detection requests HALT; only post-stop frames vote."""
        self.last_frame = captured
        if self.phase == 'moving' and valid and not self.halt:
            self.halt = True
        if self.phase == 'watch' and valid:
            self.deadline = time.monotonic()+DWELL
        return self.phase == 'watch' and captured >= self.owner.qr_after

    def move(self, amount):
        """Send at most 800 pulses and retain its identity until actual idle."""
        if not amount or abs(amount) > STEP or abs(self.position+amount) > EXTENT:
            self.owner.fail('二维码搜索步长或范围无效。'); return
        self.amount = amount
        self.move_id = None
        self.halt = False
        self.phase = 'accept'
        self.deadline = time.monotonic()+8
        self.owner.qr_votes = 0
        self.owner.qr_status = '{}：{}脉冲 → {}脉冲，速度{}'.format(
            '返回A点' if self.returning else 'A点前后搜索', self.position, self.position+amount, SPEED)
        self.owner.send('CHASSIS STRAIGHT {} {} {} {}'.format(
            'FWD' if amount > 0 else 'BACK', abs(amount), SPEED, self.owner.route['accel']), 'qr_search_accept')

    def next_move(self):
        """Advance within the sweep, or unwind the exact accepted pulse offset."""
        if self.returning:
            if self.position == 0:
                self.phase = 'verify'
                self.owner.send('NAV STAT', 'qr_search_origin')
                return
            target = 0
        else:
            targets = (-EXTENT, EXTENT, 0)
            while self.leg < len(targets) and self.position == targets[self.leg]:
                self.leg += 1
            if self.leg == len(targets):
                self.leg = 0
            target = targets[self.leg]
        delta = target-self.position
        self.move((1 if delta > 0 else -1)*min(STEP, abs(delta)))

    def return_to_origin(self):
        """After camera handoff, return physically before restoring route coordinates."""
        self.returning = True
        self.owner.state = 'qr_return'
        self.next_move()

    def tick(self):
        """Poll accepted moves; no wall-time extrapolation can complete a step."""
        o = self.owner
        now = time.monotonic()
        if o.pending: return
        if self.phase == 'moving':
            if now > self.deadline:
                o.fail('二维码搜索移动超时。'); return
            if self.halt is True:
                self.halt = 'sent'
                o.send('CHASSIS HALT', 'qr_search_halt'); return
            if now-self.last_status >= .1:
                self.last_status = now
                o.send('CHASSIS STAT', 'qr_search_status')
        elif self.phase == 'watch' and now >= self.deadline:
            if self.last_frame is None and now-self.started < 3:
                return
            if self.last_frame is None or time.time()-self.last_frame > 3:
                o.fail('摄像头2连续未产生新帧，停止搜索。'); return
            if not self.suspended:
                self.phase = 'suspend'
                o.send('NAV SUSPEND {} {}'.format(o.path_id,o.leg-o.leg_base-1), 'qr_search_suspend')
            else:
                self.next_move()

    def reply(self, purpose, fields):
        """Only tagged operation status can finish a slice or resume navigation."""
        o = self.owner
        if purpose == 'qr_search_suspend':
            self.suspended = True
            self.phase = 'pose'
            o.send(o.ring_pose_command('POSE', self.origin), 'qr_search_pose')
        elif purpose == 'qr_search_pose':
            self.next_move()
        elif purpose == 'qr_search_accept':
            self.move_id = fields.get('move_id')
            if self.move_id is None: o.fail('二维码搜索缺少运动编号。'); return
            self.phase = 'moving'
        elif purpose == 'qr_search_status':
            if fields.get('move_id') != self.move_id or fields.get('fault', 'none') != 'none':
                o.fail('二维码搜索运动编号或反馈异常。'); return
            interrupted = self.halt == 'sent' and fields.get('state') == 'stopped'
            if fields.get('state') in ('fault','stopped') and not interrupted:
                o.fail('二维码搜索移动被中断。'); return
            if fields.get('active') != '0' or (fields.get('state') != 'done' and not interrupted): return
            remaining = int(fields.get('remaining', '-1'))
            if not 0 <= remaining <= abs(self.amount) or (not interrupted and remaining):
                o.fail('二维码搜索剩余脉冲无效。'); return
            self.position += (1 if self.amount > 0 else -1)*(abs(self.amount)-remaining)
            self.phase = 'watch'
            self.move_id = None
            self.halt = False
            o.qr_votes = 0
            o.qr_after = time.time()+.1
            self.deadline = time.monotonic()+DWELL
            if self.returning: self.next_move()
        elif purpose == 'qr_search_origin':
            if (fields.get('state') != 'suspended' or fields.get('ready') != '1' or
                    fields.get('imu') != '1' or fields.get('path_id') != o.path_id):
                o.fail('二维码搜索返回后的导航状态无效。'); return
            if max(abs(int(fields[k])-value) for k,value in zip(('x','y'), self.origin)) > 10:
                o.fail('二维码搜索回位积分与A点不一致，停止续航。'); return
            self.phase = 'done'
            o.qr_status = '任务码已锁定，相机已切换，已返回A点并恢复巡航'
            # Reuse the tested external-task reupload path and original leg IDs.
            o.state = 'ring_end'
            o.send(o.ring_pose_command('RESTORE', self.origin), 'ring_end')
