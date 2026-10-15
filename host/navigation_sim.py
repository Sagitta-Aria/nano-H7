"""Demo forward-only mission with explicit synthetic yaw, never real IMU data.

Mirrors MCU phase/freshness/settle guards. It cannot prove wheel slip or timing.
"""
import math
from .navigation_grid import on_grid, round_pulse
from .navigation import path_checksum, validate_path
from .straight_sim import StraightTrajectory, correction


def wrap(angle):
    return (angle + 180) % 360 - 180


class NavigationSim:
    def __init__(self, board):
        self.board = board
        self.points, self.headings = {}, dict.fromkeys(range(5), 0.)
        self.ready, self.state = False, "idle"
        self.x4 = self.y4 = 0.
        self.previous = [0] * 4
        self.mask, self.leg, self.route = 5, 0, (1, 2, 3, 4, 1)
        self.sps, self.accel, self.dwell = 500, 2000, 3.
        self.wait_started = self.turn_started = 0.
        self.move_id, self.obstacle, self.line = 0, False, None
        self.absolute_heading, self.initial_heading, self.target_heading = 37., 37., 0.
        self.auto_imu, self.imu_valid = True, True
        self.imu_time, self.imu_epoch, self.imu_revision = board.clock(), 1, 0
        self.stable_revision, self.stable_count = 0, 0
        self.stable_started = self.stable_last = 0.
        self.fault = "none"
        self.turn_fraction = 0.
        self.frame_set = False
        self.grid = {"x": [0, 14000, 28000], "y": [0, 14000, 28000]}
        self.path, self.path_index = [], 0
        self.staged, self.uploaded = [], []
        self.uploading, self.committed, self.path_id = False, False, 0

    def busy(self):
        return self.state in ("turning", "moving", "orienting", "waiting", "blocked")

    def fresh(self, now):
        return self.imu_valid and now - self.imu_time < .3

    def pose(self):
        if not self.ready:
            return
        wheels = [(axis["pos"] - old) * (-1 if self.mask & (1 << i) else 1)
                  for i, (axis, old) in enumerate(zip(self.board.axes, self.previous))]
        self.previous = [axis["pos"] for axis in self.board.axes[:4]]
        if self.state == "moving":
            angle = math.radians(wrap(self.absolute_heading - self.initial_heading))
            self.x4 += sum(wheels) * math.sin(angle)
            self.y4 += sum(wheels) * math.cos(angle)

    def halt_wheels(self):
        self.line = None
        for axis in self.board.axes[:4]:
            axis.update(active=0, velocity=0., target=0)

    def clear_upload(self):
        """STOP/config/session changes invalidate staged and unconsumed routes."""
        self.uploading = self.committed = False

    def stop(self):
        self.board.turn.stop()
        self.clear_upload()
        self.pose()
        if self.busy():
            self.state = "stopped"
        self.halt_wheels()

    def fail(self, reason):
        self.pose()
        self.halt_wheels()
        self.state, self.fault = "fault", reason

    def feedback(self, angle, valid, epoch, now):
        self.board.turn.feedback(angle, valid, epoch, now)
        self.pose()
        if self.busy():
            if epoch != self.imu_epoch:
                self.fail("imu_changed")
            elif not self.fresh(now):
                self.fail("imu_stale")
        if self.frame_set and epoch != self.imu_epoch:
            self.ready, self.frame_set = False, False
        if epoch != self.imu_epoch:
            self.clear_upload()
        self.absolute_heading, self.imu_valid = wrap(angle), valid
        self.imu_epoch, self.imu_time = epoch, now
        self.imu_revision += 1
        if not valid and self.busy():
            self.fail("imu_invalid")

    def begin_turn(self, state, angle, now):
        self.state, self.target_heading, self.turn_started = state, wrap(angle), now
        self.stable_count, self.stable_revision = 0, self.imu_revision
        self.turn_fraction = 0.

    def launch(self, now):
        self.path, last = [], (round_pulse(self.x4/4), round_pulse(self.y4/4))
        for x, y, leg in self.uploaded:
            if leg == self.leg and (x, y) != last:
                self.path.append((x, y))
                last = (x, y)
        self.path_index = 0
        self.launch_segment(now)

    def launch_segment(self, now):
        """Intersections turn without mission dwell; only the final goal orients."""
        if self.path_index == len(self.path):
            self.begin_turn("orienting", self.headings[self.route[self.leg]], now)
        else:
            target = self.path[self.path_index]
            dx, dy = target[0] - round_pulse(self.x4/4), target[1] - round_pulse(self.y4/4)
            if bool(dx) == bool(dy):
                self.fail("grid_segment")
                return
            angle = (90 if dx > 0 else -90) if dx else (0 if dy > 0 else 180)
            self.begin_turn("turning", angle, now)

    def launch_forward(self):
        target = self.path[self.path_index]
        dx, dy = target[0] - round_pulse(self.x4/4), target[1] - round_pulse(self.y4/4)
        total = abs(dx) + abs(dy)
        if not total or (dx and dy):
            self.fail("move_start")
            return
        signs = [-1 if self.mask & (1 << i) else 1 for i in range(4)]
        yaw = [(-1 if i % 2 else 1)*sign for i, sign in enumerate(signs)]
        self.line = StraightTrajectory(self.board, signs, yaw, total, self.sps, self.accel, self.move_id)
        self.state = "moving"

    def turn(self, now, dt):
        error = wrap(self.target_heading - wrap(self.absolute_heading - self.initial_heading))
        if now - self.turn_started >= 60:
            self.fail("turn_timeout")
            return
        if abs(error) <= self.board.turn.tolerance_cd/100 + 1e-9:
            self.halt_wheels()
            if self.imu_revision != self.stable_revision:
                self.stable_revision = self.imu_revision
                if not self.stable_count or self.imu_time - self.stable_last > .2:
                    self.stable_count, self.stable_started = 0, self.imu_time
                self.stable_count += 1
                self.stable_last = self.imu_time
                if self.stable_count >= 3 and self.stable_last - self.stable_started >= .3 - 1e-9:
                    if self.state == "turning":
                        self.launch_forward()
                    else:
                        self.state, self.wait_started = "waiting", now
            return
        self.stable_count = 0
        self.stable_revision = self.imu_revision
        if self.board.turn.kp == 0:
            self.halt_wheels()
            return
        sps = min(self.sps, max(1, int(abs(error) * self.board.turn.kp + .5)))
        direction = 1 if error > 0 else -1
        for i, axis in enumerate(self.board.axes[:4]):
            sign = direction * (-1 if i % 2 else 1) * (-1 if self.mask & (1 << i) else 1)
            axis.update(active=1, target=sign*sps, velocity=sign*sps, finite=False, move_id=self.move_id)
        # Synthetic geometry: five wheel pulses per degree. External feedback
        # mode intentionally does not invent yaw from commands.
        if self.auto_imu and dt > 0:
            change = direction * min(abs(error), sps / 5 * dt)
            self.turn_fraction += abs(change) * 5
            steps = int(self.turn_fraction)
            self.turn_fraction -= steps
            for i, axis in enumerate(self.board.axes[:4]):
                axis["pos"] += steps * direction * (-1 if i % 2 else 1) * (-1 if self.mask & (1 << i) else 1)
            self.feedback(self.absolute_heading + change, True, 1, now)

    def update(self, now, dt):
        if self.uploading and now - self.upload_time >= 5:
            self.clear_upload()
        if self.auto_imu and dt > 0:
            self.feedback(self.absolute_heading, True, 1, now)
        if self.busy() and not self.board.armed:
            self.stop()
        if self.busy() and not self.fresh(now):
            self.fail("imu_stale")
        if self.state in ("turning", "orienting"):
            self.turn(now, dt)
        elif self.state == "moving" and self.line:
            error = wrap(self.target_heading - wrap(self.absolute_heading - self.initial_heading))
            if abs(error) > 5:
                self.fail("heading_deviation")
                return
            rotation = self.line.advance(dt, correction(self.board, error))
            if self.auto_imu and rotation:
                self.feedback(self.absolute_heading+rotation*.08, True, self.imu_epoch, now)
            self.pose()
            if not self.line.remaining_q:
                point = self.path[self.path_index]
                self.x4, self.y4 = point[0]*4, point[1]*4
                self.halt_wheels()
                self.path_index += 1
                self.launch_segment(now)
        elif self.state == "waiting":
            error = wrap(self.headings[self.route[self.leg]] - wrap(self.absolute_heading - self.initial_heading))
            if abs(error) > self.board.turn.tolerance_cd/100 + 1e-9:
                self.begin_turn("orienting", self.headings[self.route[self.leg]], now)
            elif now - self.wait_started >= self.dwell:
                self.leg += 1
                if self.leg == 5:
                    self.leg, self.state = 4, "done"
                else:
                    self.launch(now)
        self.pose()

    def report(self):
        point = self.points.get(self.route[self.leg], (0, 0))
        wait = max(0, int((self.dwell-(self.board.clock()-self.wait_started))*1000)) if self.state == "waiting" else 0
        return ("OK cmd=NAV_STAT state={} ready={} x={} y={} target={} tx={} ty={} leg={} "
                "wait_ms={} move_id={} obstacle={} planner=host_grid heading_cd={} target_cd={} arrival_cd={} imu={} fault={} "
                "wx={} wy={} segment={} segments={} path_id={} route_ready={}").format(
                    self.state, int(self.ready), round_pulse(self.x4/4), round_pulse(self.y4/4), "OABCD"[self.route[self.leg]],
                    *point, self.leg+1, wait, self.move_id, int(self.obstacle),
                    round(wrap(self.absolute_heading-self.initial_heading)*100), round(self.target_heading*100),
                    round(self.headings[self.route[self.leg]]*100), int(self.fresh(self.board.clock())), self.fault,
                    *(self.path[self.path_index] if self.path_index < len(self.path) else point),
                    min(self.path_index + 1, len(self.path)), len(self.path), self.path_id, int(self.committed))

    def dispatch(self, sequence, args):
        board, number = self.board, self.board.number
        if args == ["STAT"]:
            self.pose()
            return self.report()
        if args == ["STOP"]:
            self.stop()
            return "OK cmd=NAV_STOP"
        if len(args) == 4 and args[0] == "IMU":
            angle, valid, epoch = number(args[1], -18000, 18000), number(args[2], 0, 1), number(args[3], 1, 2147483647)
            self.auto_imu = False
            self.feedback(angle/100, bool(valid), epoch, board.clock())
            return "OK cmd=NAV_IMU"
        if len(args) == 3 and args[0] == "RESET":
            x, y = [number(v, -1000000, 1000000) for v in args[1:]]
            if board.armed or self.busy() or board.turn.busy() or any(a["active"] for a in board.axes[:4]):
                return "ERR code=NAV_RESET_REQUIRES_DISARMED_IDLE"
            self.x4, self.y4, self.points[0] = x*4, y*4, (x, y)
            self.previous, self.mask = [a["pos"] for a in board.axes[:4]], board.invert
            self.ready, self.state, self.leg, self.obstacle, self.fault = True, "idle", 0, False, "none"
            self.frame_set = False
            self.clear_upload()
            self.path, self.path_index = [], 0
            return self.report()
        if len(args) == 3 and args[0] == "ATTITUDE":
            index, angle = number(args[1], 0, 4), number(args[2], -18000, 18000)
            if self.busy():
                return "ERR code=NAV_BUSY"
            self.clear_upload()
            self.headings[index] = wrap(angle/100)
            return "OK cmd=NAV_ATTITUDE"
        if len(args) == 5 and args[0] == "GRID" and args[1] in ("X", "Y"):
            values = [number(v, -1000000, 1000000) for v in args[2:]]
            if self.busy() or not values[0] < values[1] < values[2]:
                return "ERR code=NAV_GRID_INVALID_OR_BUSY"
            self.clear_upload()
            self.grid[args[1].lower()] = values
            self.path, self.path_index = [], 0
            return "OK cmd=NAV_GRID"
        if len(args) == 4 and args[0] in ("POINT", "CONFIG"):
            if args[0] == "POINT":
                index = number(args[1], 0, 4)
                values = tuple(number(v, -1000000, 1000000) for v in args[2:])
            else:
                values = (number(args[1], 1, 5000), number(args[2], 1, 20000), number(args[3], 0, 60000))
            if self.busy():
                return "ERR code=NAV_BUSY"
            self.clear_upload()
            if args[0] == "POINT":
                if index == 0 and self.points.get(0) != values:
                    self.ready = False
                self.points[index] = values
            else:
                self.sps, self.accel, dwell = values
                self.dwell = dwell/1000.
            return "OK cmd=NAV_" + args[0]
        if len(args) == 5 and args[0] == "BEGIN":
            ident, count = number(args[1], 1, 2147483647), number(args[2], 5, 64)
            start = tuple(number(v, -1000000, 1000000) for v in args[3:])
            if not self.ready or len(self.points) != 5 or self.busy() or board.turn.busy() or any(a["active"] for a in board.axes[:4]):
                return "ERR code=NAV_UPLOAD_NOT_READY"
            self.pose()
            self.clear_upload()
            if start != (round_pulse(self.x4/4), round_pulse(self.y4/4)) or not on_grid(start, self.grid):
                return "ERR code=NAV_UPLOAD_NOT_READY"
            self.upload_id, self.upload_count, self.start_xy = ident, count, start
            self.upload_epoch, self.upload_time = self.imu_epoch, board.clock()
            self.staged, self.uploading = [], True
            return "OK cmd=NAV_BEGIN"
        if len(args) == 6 and args[0] == "WP":
            ident, index = number(args[1], 1, 2147483647), number(args[2], 0, 63)
            entry = tuple(number(v, -1000000, 1000000) for v in args[3:5]) + (number(args[5], 0, 4),)
            if not self.uploading or ident != self.upload_id or index != len(self.staged) or index >= self.upload_count or \
                    self.busy() or board.turn.busy() or any(a["active"] for a in board.axes[:4]) or board.clock() - self.upload_time >= 5:
                return "ERR code=NAV_UPLOAD_ORDER_OR_BUSY"
            self.staged.append(entry)
            self.upload_time = board.clock()
            return "OK cmd=NAV_WP"
        if len(args) == 3 and args[0] == "COMMIT":
            ident, crc = number(args[1], 1, 2147483647), number(args[2], 0, 65535)
            self.pose()
            if not self.uploading or ident != self.upload_id or len(self.staged) != self.upload_count or \
                    crc != path_checksum(self.staged) or not self.ready or self.upload_epoch != self.imu_epoch or \
                    self.busy() or board.turn.busy() or any(a["active"] for a in board.axes[:4]) or \
                    self.start_xy != (round_pulse(self.x4/4), round_pulse(self.y4/4)) or board.clock() - self.upload_time >= 5 or \
                    not validate_path(self.staged, self.start_xy, self.points, self.grid):
                return "ERR code=NAV_PATH_INVALID_OR_INCOMPLETE"
            self.uploaded, self.path_id = list(self.staged), ident
            self.committed, self.uploading = True, False
            return "OK cmd=NAV_COMMIT"
        if args == ["START"] or len(args) == 2 and args[0] == "START":
            requested = number(args[1], 1, 2147483647) if len(args) == 2 else 0
            if not self.committed or requested and requested != self.path_id or not self.ready or len(self.points) != 5 or self.obstacle or self.busy() or board.turn.busy() or not board.armed or \
                    any(a["active"] for a in board.axes[:4]) or not self.fresh(board.clock()):
                return "ERR code=NAV_NOT_READY_BUSY_DISARMED_OR_IMU"
            if not on_grid((round_pulse(self.x4/4), round_pulse(self.y4/4)), self.grid) or any(
                    not on_grid(p, self.grid) for p in self.points.values()):
                self.fail("off_grid")
                return "ERR code=NAV_NOT_READY_BUSY_DISARMED_OR_IMU"
            self.pose()
            if self.upload_epoch != self.imu_epoch or self.start_xy != (round_pulse(self.x4/4), round_pulse(self.y4/4)):
                self.clear_upload()
                return "ERR code=NAV_NOT_READY_BUSY_DISARMED_OR_IMU"
            self.committed = False
            self.mask, self.move_id, self.leg = board.invert, sequence, 0
            self.previous = [a["pos"] for a in board.axes[:4]]
            if not self.frame_set:
                self.initial_heading = self.absolute_heading
            self.fault = "none"
            self.frame_set = True
            self.launch(board.clock())
            return "OK cmd=NAV_START state=accepted move_id={}".format(sequence)
        if len(args) == 2 and args[0] == "OBSTACLE":
            self.obstacle = bool(number(args[1], 0, 1))
            if self.obstacle and self.busy():
                self.stop()
                self.state = "blocked"
            return self.report()
        if args == ["REPLAN"]:
            return "ERR code=NAV_PLANNER_NOT_IMPLEMENTED"
        return "ERR code=BAD_ARGUMENT"
