"""Freeze executor budgets during a board-confirmed IMU hold, without replay.

The serial worker owns the pause clock. Console owns executor state and ACKs.
Only explicit budget timestamps move; camera/gyro measurements keep real age.
"""
import time
import queue


class ImuPause:
    """Coordinate same-session waiting; STOP and terminal faults bypass it."""
    TIMERS = ('started', 'stage_started', 'leg_started', 'group_started',
              'wait_started', 'wait_until', 'servo_until', 'deadline',
              'dwell_until', 'claw_until', 'vision_deadline', 'last_nav',
              'alignment_budget_at', 'qr_budget_at', 'target_last_seen')

    def __init__(self, console):
        self.console = console
        self.connection = None
        self.paused = False
        self.seen = 0.
        self.replies = []

    def objects(self):
        """List the owned executors once, including their current children."""
        c = self.console
        return [obj for obj in (c.competition, c.pick_place, c.camera_alignment,
                c.task, c.navigator, c.task.material, c.task.qr_search,
                c.camera_alignment.ring_sequence) if obj is not None]

    def shift(self, elapsed):
        """Exclude paused wall time from budgets and outstanding ACK deadlines."""
        for obj in self.objects():
            for name in self.TIMERS:
                value = getattr(obj, name, None)
                if isinstance(value, (int, float)) and value:
                    setattr(obj, name, value+elapsed)
            pending = getattr(obj, 'pending', None)
            if isinstance(pending, tuple):
                obj.pending = (*pending[:-1], pending[-1]+elapsed)
            elif isinstance(pending, dict):
                obj.pending = {tag: (purpose, sent+elapsed)
                               for tag, (purpose, sent) in pending.items()}

    def sync(self):
        """Called under Console.lock before both reply dispatch and task ticks."""
        c, now = self.console, time.monotonic()
        link = c.connection
        if link is not self.connection:
            self.connection, self.seen, self.paused = link, 0., False
            self.replies.clear()
        clock = getattr(link, 'imu_pause_snapshot', None)
        snapshot = clock() if clock else None
        # Lightweight/offline links may not implement the worker clock.
        paused, total = snapshot if isinstance(snapshot, tuple) and len(snapshot) == 2 else (False, 0.)
        elapsed = max(0., total-self.seen)
        self.seen = total
        if elapsed: self.shift(elapsed)
        # The worker may observe an entire outage between Console ticks.
        # Its accumulated clock still requires the fresh-frame/heading barrier.
        if paused != self.paused or (elapsed > 0 and not paused):
            self.paused = paused
            c.note('warn' if paused else 'ok',
                   'IMU 信号超时：已停车并保留任务，等待原信号恢复。' if paused else
                   'IMU 连续有效样本已确认：继续原任务。')
            if link: link.clear_camera_velocity()
            a = c.camera_alignment
            if a.active:
                a.vx = a.vy = a.yaw = 0
                a.last_result = 0.
                a.stable = 0
                if not paused:
                    from .vision_alignment import VisualPID
                    a.pid = VisualPID(a.config['tracking'])
                    a.capture_after = now
                    a.heading_time = 0.
                    a.resume_heading_required = True
                    # A pre-pause heading ACK may still be buffered. Consume it
                    # only to release its tag, then query a genuinely new one.
                    a.resume_old_heading_tag = (a.pending[0] if a.pending
                                                and a.pending[1] == 'heading' else None)
                    a.vision_deadline = now+3
                    a.reset_velocity_feedback()
                    # Invalidate an outstanding pre-pause camera waiter without
                    # changing action tags or replaying any accepted command.
                    a.capture_token += 1
                    a.capturing = False
                    while not a.results.empty():
                        try: a.results.get_nowait()
                        except queue.Empty: break
                    if a.ring_sequence:
                        a.ring_sequence.after = now
                        a.ring_sequence.stable = 0
                        a.ring_sequence.previous_angle = None
        return paused

    @staticmethod
    def terminal(command, reply):
        """Never buffer STOP, disarm, serial rejection or a latched fault."""
        f = reply.fields
        return (not reply.ok or command in ('STOP', 'ARM 0', 'NAV STOP', 'CHASSIS HALT', 'TURN STOP')
                or f.get('armed') == '0' or f.get('latched') == '1'
                or f.get('fault', 'none') != 'none' or f.get('state') in ('fault', 'stopped'))

    def snapshot(self):
        return dict(paused=self.paused, reason='imu_timeout' if self.paused else '',
                    paused_s=round(self.seen, 3))
