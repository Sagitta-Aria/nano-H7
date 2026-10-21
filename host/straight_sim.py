"""Coarse heading-assisted translation demo, never proof of physical accuracy.

Mirrors distance accounting, speed mixing and feedback guards. Timer pulse
width/jitter are tested separately in the actual compiled ARM image.
"""
import math


def wrap(angle):
    """Shortest yaw difference, matching MCU clockwise tie at 180 degrees."""
    result = (angle+180) % 360-180
    return 180. if result == -180 else result


class StraightTrajectory:
    """Own a finite translation pulse budget, shared by manual moves and cruise."""
    def __init__(self, board, signs, yaw, steps, sps, accel, move_id):
        self.board, self.signs, self.yaw = board, signs, yaw
        self.remaining_q, self.sps, self.accel = steps * 4, sps, accel
        self.mix = [a*b for a, b in zip(signs, yaw)]
        for axis, sign in zip(board.axes[:4], signs):
            axis.update(active=1, finite=False, remaining=steps, move_id=move_id,
                        target=sign*sps, velocity=sign*min(10, sps), accel=accel, fraction=0.)

    def advance(self, dt, correction):
        """Advance independent wheel pulses; sum/4 terminates without undoing yaw."""
        rotation = 0.
        while dt > 1e-9 and self.remaining_q:
            part = min(dt, .01)
            dt -= part
            target = min(self.sps, max(1, int(math.sqrt(2*self.accel*math.ceil(self.remaining_q/4)))))
            limit = target // 3
            correction_now = max(-limit, min(limit, correction))
            base = target-abs(correction_now)
            for axis, sign, yaw, mix in zip(self.board.axes[:4], self.signs, self.yaw, self.mix):
                target_sps = base + mix*correction_now
                velocity = abs(axis['velocity'])
                velocity += max(-self.accel*part, min(self.accel*part, target_sps-velocity))
                axis.update(velocity=sign*velocity, target=sign*target_sps)
                axis['fraction'] += velocity*part
                count = min(self.remaining_q, int(axis['fraction']))
                axis['fraction'] -= count
                self.remaining_q -= count
                axis['pos'] += sign*count
                rotation += sign*yaw*count/4
            for axis in self.board.axes[:4]:
                axis['remaining'] = (self.remaining_q+3)//4
        if not self.remaining_q:
            for axis in self.board.axes[:4]:
                axis.update(active=0, velocity=0., target=0)
        return rotation


def correction(board, error):
    """Shared displayed yaw P and tolerance deadband, clockwise positive."""
    if board.turn.kp == 0 or abs(error) <= board.turn.tolerance_cd/100 + 1e-9:
        return 0
    value = max(1, int(abs(error)*board.turn.kp+.5))
    return value if error > 0 else -value


class StraightSim:
    """Preserve finite translation on same-session holds; epoch resets fault."""
    def __init__(self, board):
        self.board, self.line = board, None
        self.target = self.error = 0.
        self.epoch, self.assisted, self.fault = 0, False, 'none'

    def busy(self):
        return self.line is not None and self.board.chassis_state == 'moving'

    def stop(self, fault=None):
        if self.busy():
            self.line = None
            for axis in self.board.axes[:4]:
                axis.update(active=0, velocity=0., target=0)
            self.board.chassis_state = 'fault' if fault else 'stopped'
            self.fault = fault or 'none'

    def feedback_guard(self, valid, epoch, previously_fresh):
        if not self.busy():
            return
        if epoch != self.epoch:
            self.stop('imu_changed')

    def start(self, signs, yaw, steps, sps, accel, move_id):
        self.target, self.epoch = self.board.turn.heading, self.board.turn.epoch
        self.error, self.fault, self.assisted = 0., 'none', True
        self.line = StraightTrajectory(self.board, signs, yaw, steps, sps, accel, move_id)

    def update(self, now, dt):
        if not self.busy():
            return
        if not self.board.armed:
            self.stop()
            return
        if self.board.turn.paused: return
        self.error = wrap(self.target-self.board.turn.heading)
        rotation = self.line.advance(dt, correction(self.board, self.error))
        if self.board.navigation.auto_imu and rotation:
            self.board.navigation.feedback(self.board.turn.heading+rotation*.08, True, self.epoch, now)
        if not self.line.remaining_q:
            self.line = None
            self.board.chassis_state = 'done'
