"""Standalone turn demo with synthetic yaw. Mirrors protocol/ownership guards;
the demo geometry cannot establish physical rotation accuracy or wheel timing.
"""
from .navigation_sim import wrap as navigation_wrap
from .turn import TURN_MIN_SPS


def wrap(angle):
    """Match the MCU's deterministic clockwise tie for exactly 180 degrees."""
    result = navigation_wrap(angle)
    return 180. if result == -180. else result


class TurnSim:
    """Own only M1..M4 while turning, preserving the mission and auxiliary axes."""
    def __init__(self, board):
        self.board = board
        self.state, self.mode, self.fault = "idle", "REL", "none"
        self.zero = None
        self.origin = self.target = self.heading = 0.
        self.valid, self.epoch, self.revision = False, 0, 0
        self.sample_time = self.started = 0.
        self.sps, self.accel, self.mask, self.move_id = 500, 2000, 5, 0
        self.kp = 20.
        self.tolerance_cd = 20
        self.task_tolerance_cd = 20
        self.min_sps = TURN_MIN_SPS
        self.stable_revision = self.stable_count = 0
        self.stable_started = self.stable_last = self.fraction = 0.
        self.paused = False
        self.pause_started = self.pause_total = self.pause_seen = 0.
        self.resume_samples = 0
        self.resume_first = 0.

    def imu_required(self):
        """Task owners require yaw; ordinary manual raw wheel commands do not."""
        b = self.board
        return (self.busy() or b.straight.busy() or b.navigation.busy()
                or b.navigation.state == 'suspended')

    def hold(self, now):
        """Keep finite operation ownership and discard suspended visual velocity."""
        if not self.paused:
            self.paused, self.pause_started = True, now
            if self.board.navigation.state == 'suspended' and not self.board.straight.busy() and not self.busy():
                self.board.navigation.halt_wheels()
        self.resume_samples = 0
        for axis in self.board.axes[:4]: axis['velocity'] = 0.

    def waiting(self, now):
        """Mirror the MCU lease and freeze task budgets, never sample ages."""
        required = self.board.armed and self.imu_required()
        if self.paused and not required:
            self.pause_total += now-self.pause_started
            self.paused, self.resume_samples = False, 0
        if required and not self.fresh(now): self.hold(now)
        total = self.pause_total + (now-self.pause_started if self.paused else 0.)
        elapsed, self.pause_seen = max(0., total-self.pause_seen), total
        self.started += elapsed
        n = self.board.navigation
        for name in ('turn_started', 'wait_started', 'micro_started'):
            setattr(n, name, getattr(n, name)+elapsed)
        return self.paused

    def busy(self):
        return self.state == "turning"

    def fresh(self, now):
        return self.valid and now - self.sample_time < .3

    def feedback(self, heading, valid, epoch, now):
        """Same-session loss holds; changing the yaw reference remains terminal."""
        self.board.straight.feedback_guard(valid, epoch, self.fresh(now))
        if epoch != self.epoch:
            self.zero = None
            if self.busy():
                self.finish("fault", "imu_changed")
        if (not valid or not self.fresh(now)) and self.board.armed and self.imu_required():
            self.hold(now)
        self.heading, self.valid, self.epoch, self.sample_time = wrap(heading), valid, epoch, now
        self.revision += 1
        if self.paused and valid:
            if not self.resume_samples: self.resume_first = now
            self.resume_samples += 1
            if self.resume_samples >= 3 and now-self.resume_first >= .1-1e-9:
                self.pause_total += now-self.pause_started
                self.paused, self.resume_samples, self.stable_count = False, 0, 0

    def finish(self, state, fault="none"):
        """Release only turn-owned wheels; never disarm or stop auxiliary axes."""
        self.board.navigation.halt_wheels()
        self.state, self.fault = state, fault

    def stop(self):
        if self.busy():
            self.finish("stopped")

    def report(self):
        current = wrap(self.heading - self.origin)
        active = sum(1 << i for i, a in enumerate(self.board.axes[:4]) if a["active"]) if self.busy() else 0
        return (f"OK cmd=TURN_STAT state={self.state} mode={self.mode} zero={int(self.zero is not None)} "
                f"current_cd={round(current*100)} target_cd={round(self.target*100)} "
                f"error_cd={round(wrap(self.target-current)*100)} imu={int(self.fresh(self.board.clock()))} "
                f"move_id={self.move_id} active={active} fault={self.fault} kp_centi={round(self.kp*100)} tolerance_cd={self.tolerance_cd} min_sps={self.min_sps} paused={int(self.paused)} task_tolerance_cd={self.task_tolerance_cd}")

    def dispatch(self, sequence, args):
        """Validate the same finite angle and state boundaries as real firmware."""
        now, board = self.board.clock(), self.board
        if args == ["STAT"]:
            return self.report()
        if args == ["STOP"]:
            self.stop()
            return self.report()
        busy = self.busy() or board.navigation.busy() or board.chassis_state in ("moving", "jogging") or any(a["active"] for a in board.axes[:4])
        if len(args) in (2,3,4) and args[0] == "CONFIG":
            kp = board.number(args[1], 0, 2147483647)
            tolerance = board.number(args[2], 1, 1000) if len(args) >= 3 else self.tolerance_cd
            minimum = board.number(args[3], 1, 5000) if len(args) == 4 else self.min_sps
            if busy:
                return "ERR code=TURN_CONFIG_RANGE_OR_BUSY"
            self.kp = kp / 100.
            self.tolerance_cd = tolerance
            self.min_sps = minimum
            return self.report()
        if args == ["ZERO"]:
            if busy or self.paused or not self.fresh(now):
                return "ERR code=TURN_ZERO_REQUIRES_IDLE_IMU"
            self.zero = self.origin = self.heading
            self.target, self.state, self.fault = 0., "idle", "none"
            return self.report()
        if len(args) in (5, 6) and args[:1] == ["MOVE"] and args[1] in ("REL", "ABS"):
            angle = board.number(args[2], -18000, 18000) / 100
            sps, accel = board.number(args[3], 1, 5000), board.number(args[4], 1, 20000)
            band = board.number(args[5], 1, 1000) if len(args) == 6 else self.tolerance_cd
            if busy or self.paused or not board.armed or not self.fresh(now) or (args[1] == "ABS" and self.zero is None):
                return "ERR code=TURN_NOT_READY_BUSY_DISARMED_OR_IMU"
            self.mode, self.origin = args[1], self.zero if args[1] == "ABS" else self.heading
            self.target, self.sps, self.accel, self.mask = wrap(angle), sps, accel, board.invert
            self.started, self.move_id, self.state, self.fault = now, sequence, "turning", "none"
            self.task_tolerance_cd = band
            self.stable_count, self.stable_revision, self.fraction = 0, self.revision, 0.
            return f"OK cmd=TURN_MOVE state=accepted move_id={sequence}"
        return "ERR code=BAD_ARGUMENT"

    def update(self, now, dt):
        """Synthetic rotation follows fresh feedback; completion needs new samples."""
        if not self.busy():
            return
        if not self.board.armed:
            self.stop()
            return
        if self.paused: return
        if now - self.started >= 60:
            self.finish("fault", "turn_timeout")
            return
        error = wrap(self.target - wrap(self.heading - self.origin))
        if abs(error) <= self.task_tolerance_cd/100 + 1e-9:
            self.board.navigation.halt_wheels()
            if self.revision != self.stable_revision:
                self.stable_revision = self.revision
                if not self.stable_count or self.sample_time - self.stable_last > .2:
                    self.stable_started, self.stable_count = self.sample_time, 0
                self.stable_last = self.sample_time
                self.stable_count += 1
                if self.stable_count >= 3:
                    self.finish("done")
            return
        self.stable_count, self.stable_revision = 0, self.revision
        if self.kp == 0:
            self.board.navigation.halt_wheels()
            return
        sps, direction = min(self.sps, max(self.min_sps, int(abs(error)*self.kp + .5))), 1 if error > 0 else -1
        for i, axis in enumerate(self.board.axes[:4]):
            sign = direction * (-1 if i % 2 else 1) * (-1 if self.mask & (1 << i) else 1)
            axis.update(active=1, target=sign*sps, velocity=sign*sps, finite=False, move_id=self.move_id, accel=self.accel)
        if self.board.navigation.auto_imu and dt > 0:
            change = direction * min(abs(error), sps / 5 * dt)
            self.fraction += abs(change)*5
            steps = int(self.fraction)
            self.fraction -= steps
            for i, axis in enumerate(self.board.axes[:4]):
                axis["pos"] += steps * direction * (-1 if i % 2 else 1) * (-1 if self.mask & (1 << i) else 1)
            self.board.navigation.feedback(self.heading + change, True, self.epoch, now)
