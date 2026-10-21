"""Demo forward-only mission with explicit synthetic yaw, never real IMU data.

Mirrors MCU phase/freshness/settle guards. It cannot prove wheel slip or timing.
"""
import math
from .navigation_grid import on_grid, round_pulse
from .navigation import path_checksum, validate_path
from .straight_sim import StraightTrajectory, correction


# Mecanum strafe loses travel per pulse to roller slip, so lateral distance is
# not equal to forward distance. 1.0 is the undistorted geometric value; the
# real figure must be measured on the field and is deliberately not guessed.
LATERAL_SCALE = 1.0


def wrap(angle):
    return (angle + 180) % 360 - 180


class NavigationSim:
    def __init__(self, board):
        self.board = board
        # Station and leg counts are declared by the host before their rows;
        # nothing here assumes a fixed mission shape.
        self.station_count, self.leg_count = 0, 0
        self.points, self.headings = {}, {}
        self.route, self.holds = [], []
        self.ready, self.state = False, "idle"
        self.x4 = self.y4 = 0.
        self.previous = [0] * 4
        self.mask, self.leg, self.released = 5, 0, False
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
        self.departure_heading = 0.
        self.grid = {"x": [0, 14000, 28000], "y": [0, 14000, 28000]}
        self.path, self.path_index = [], 0
        self.staged, self.uploaded = [], []
        self.uploading, self.committed, self.path_id = False, False, 0
        self.micro_count, self.micro_steps, self.micro_id = 0, 0, 0
        self.micro_started = 0.
        self.external_tracking = False
        self.host_mode=False; self.host_op=0; self.host_next_leg=0; self.host_hold_heading=0.
        self.vector=None
        self.recovering = False
        self.vector_travel = False
        self.inset_steps, self.pocket_phase, self.pocket_returns = 0, 0, 0
        self.pocket_steps, self.pocket_progress, self.pocket_total = [], 0., 0
        self.pocket_origin, self.pocket_heading = (0., 0.), 0.

    def invalidate(self):
        """External-controller motions preserve the field reference; manual moves revoke it."""
        if self.state != 'suspended':
            self.ready = False

    def busy(self):
        return self.state in ("turning", "moving", "orienting", "waiting", "blocked", "nudging", "checkpoint")

    def fresh(self, now):
        return self.imu_valid and now - self.imu_time < .3

    def validate_staged(self):
        """Adapt the board's tables to the shared whole-route validator."""
        if not self.configured():
            return False
        stations = [{"name": str(i), "x": self.points[i][0], "y": self.points[i][1],
                     "heading": self.headings[i]} for i in range(self.station_count)]
        route = [{"station": self.route[i], "hold": self.holds[i]}
                 for i in range(self.leg_count)]
        return validate_path(self.staged, self.start_xy, stations, route, self.grid)

    def configured(self):
        """Both tables fully populated; a partial upload can never execute."""
        return (self.station_count and self.leg_count and
                len(self.points) == self.station_count and
                len(self.headings) == self.station_count and
                all(station is not None for station in self.route))

    def pose(self):
        if not self.ready:
            return
        wheels = [(axis["pos"] - old) * (-1 if self.mask & (1 << i) else 1)
                  for i, (axis, old) in enumerate(zip(self.board.axes, self.previous))]
        self.previous = [axis["pos"] for axis in self.board.axes[:4]]
        if self.state in ("moving", "nudging") or self.pocket_phase in (1, 3) or (self.state == 'suspended' and self.external_tracking):
            # The signs[] rows for forward and strafe-left are orthogonal, so a
            # dot product with each recovers that component without inverting a
            # matrix. Wheel order is M1 rear-left, M2 rear-right, M3 front-left,
            # M4 front-right. Rotation needs wheelbase/track and is excluded:
            # turning in place never translates.
            angle = math.radians(wrap(self.absolute_heading - self.initial_heading))
            forward = wheels[0] + wheels[1] + wheels[2] + wheels[3]
            lateral = (wheels[0] - wheels[1] - wheels[2] + wheels[3]) * LATERAL_SCALE
            self.x4 += forward * math.sin(angle) - lateral * math.cos(angle)
            self.y4 += forward * math.cos(angle) + lateral * math.sin(angle)

    def halt_wheels(self):
        self.line = None
        self.vector = None
        for axis in self.board.axes[:4]:
            axis.update(active=0, velocity=0., target=0)

    def clear_upload(self):
        """STOP/config/session changes invalidate staged and unconsumed routes."""
        self.uploading = self.committed = False

    def stop(self):
        self.recovering = False
        self.board.turn.stop()
        self.clear_upload()
        self.pose()
        if self.state == 'suspended':self.ready=False
        if self.busy() or self.state == 'suspended':
            self.state = "stopped"
        self.external_tracking = False
        self.pocket_phase = 0
        self.halt_wheels()

    def fail(self, reason):
        self.recovering = False
        self.pose()
        if self.state=='suspended':self.ready=False
        self.halt_wheels()
        self.state, self.fault = "fault", reason
        self.pocket_phase = 0

    def feedback(self, angle, valid, epoch, now):
        self.board.turn.feedback(angle, valid, epoch, now)
        self.pose()
        if self.busy() or self.state == 'suspended':
            if epoch != self.imu_epoch:
                self.fail("imu_changed")
        if self.frame_set and epoch != self.imu_epoch:
            self.ready, self.frame_set = False, False
        if epoch != self.imu_epoch:
            self.clear_upload()
        self.absolute_heading, self.imu_valid = wrap(angle), valid
        self.imu_epoch, self.imu_time = epoch, now
        self.imu_revision += 1

    def begin_turn(self, state, angle, now, allow_inset=True):
        self.state, self.target_heading, self.turn_started = state, wrap(angle), now
        self.stable_count, self.stable_revision = 0, self.imu_revision
        self.turn_fraction = 0.
        self.pocket_phase, self.pocket_returns = 0, 0
    def pocket_move(self, dx, dy, cap=0):
        """Demo the firmware's map-to-wheel DDA excursion, never real motion."""
        angle = math.radians(wrap(self.absolute_heading-self.initial_heading))
        forward = dx*math.sin(angle)+dy*math.cos(angle)
        lateral = (-dx*math.cos(angle)+dy*math.sin(angle))/LATERAL_SCALE
        values = [forward+lateral, forward-lateral, forward-lateral, forward+lateral]
        peak = max(map(abs, values))
        scale = min(1., cap/peak) if cap and peak else 1.
        self.pocket_steps = [round_pulse(v*scale)*(-1 if self.mask & (1 << i) else 1) for i,v in enumerate(values)]
        self.pocket_total = max(map(abs, self.pocket_steps))
        if not self.pocket_total:
            return False
        self.pocket_progress = 0.
        self.pocket_heading = wrap(self.absolute_heading-self.initial_heading)
        for axis, steps in zip(self.board.axes[:4], self.pocket_steps):
            sign = 1 if steps >= 0 else -1
            axis.update(active=int(bool(steps)), finite=True, remaining=abs(steps),
                        target=sign*self.sps, velocity=sign*min(10,self.sps), move_id=self.move_id)
        return True

    def finish_turn(self, now):
        """Return to the saved lane/station before allowing travel or arrival."""
        if self.recovering:
            self.recovering = False; self.turn_started = now
            if max(abs(self.x4/4-self.host_goal[0]), abs(self.y4/4-self.host_goal[1])) <= .51:
                self.finish_vector(now)
            else: self.vector_start()
            return
        if self.host_mode and self.state == 'turning':
            self.state='checkpoint'; return
        if self.state == 'turning':
            self.launch_forward()
        else:
            self.state,self.wait_started = 'waiting',now

    def pocket_tick(self, now, dt):
        """Advance common demo DDA edges; no rotation or automatic yaw change."""
        if abs(wrap(self.pocket_heading-wrap(self.absolute_heading-self.initial_heading))) > 5:
            self.fail('heading_deviation')
            return
        previous = int(self.pocket_progress)
        self.pocket_progress = min(self.pocket_total,self.pocket_progress+self.sps*dt)
        current = int(self.pocket_progress)
        for axis, steps in zip(self.board.axes[:4],self.pocket_steps):
            amount = abs(steps)
            before = (previous*amount+self.pocket_total//2)//self.pocket_total
            after = (current*amount+self.pocket_total//2)//self.pocket_total
            axis['pos'] += (after-before)*(1 if steps >= 0 else -1)
            axis['remaining'] = amount-after
            if current == self.pocket_total:
                axis.update(active=0,velocity=0.,target=0)
        self.pose()
        if current == self.pocket_total:
            if self.pocket_phase == 1:
                self.pocket_phase = 2
                self.stable_count,self.stable_revision = 0,self.imu_revision
            else:
                self.finish_turn(now)

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
            self.begin_turn("orienting", (self.host_hold_heading if self.host_mode else self.headings.get(self.station(), 0.)), now)
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
        self.vector_travel = False

    def recover_heading(self, now):
        """Keep the goal and map frame while the yaw controller corrects drift."""
        if not self.host_mode and not self.vector_travel:
            self.host_goal = self.path[self.path_index]
        self.pose(); self.halt_wheels(); self.recovering = True
        self.begin_turn('turning', self.target_heading, now, allow_inset=False)

    def finish_vector(self, now):
        """Complete only the current segment, including a recovered legacy route."""
        self.x4, self.y4 = self.host_goal[0]*4, self.host_goal[1]*4
        if self.host_mode: self.state = 'checkpoint'
        else:
            self.path_index += 1
            self.launch_segment(now)

    def turn(self, now, dt):
        error = wrap(self.target_heading - wrap(self.absolute_heading - self.initial_heading))
        if now - self.turn_started >= 60:
            self.fail("turn_timeout")
            return
        if self.pocket_phase in (1, 3):
            self.pocket_tick(now,dt)
            return
        if abs(error) <= self.board.turn.tolerance_cd/100 + 1e-9:
            self.halt_wheels()
            if self.imu_revision != self.stable_revision:
                self.stable_revision = self.imu_revision
                if not self.stable_count or self.imu_time - self.stable_last > .2:
                    self.stable_count, self.stable_started = 0, self.imu_time
                self.stable_count += 1
                self.stable_last = self.imu_time
                if self.stable_count >= 3:
                    self.finish_turn(now)
            return
        self.stable_count = 0
        self.stable_revision = self.imu_revision
        if self.board.turn.kp == 0:
            self.halt_wheels()
            return
        sps = min(self.sps, max(self.board.turn.min_sps, int(abs(error) * self.board.turn.kp + .5)))
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
        if (self.busy() or self.state == 'suspended') and not self.board.armed:
            self.stop()
        if self.board.turn.paused: return
        if self.state == "nudging" and now-self.micro_started >= 60:
            self.fail("micro_timeout")
        if self.state in ("turning", "orienting"):
            self.turn(now, dt)
        elif self.state == 'moving' and (self.host_mode or self.vector_travel):
            self.vector_tick(now,dt)
        elif self.state in ("moving", "nudging") and self.line:
            error = wrap(self.target_heading - wrap(self.absolute_heading - self.initial_heading))
            if self.state == 'moving' and abs(error) > 5:
                self.recover_heading(now)
                return
            rotation = self.line.advance(dt, correction(self.board, error))
            if self.auto_imu and rotation:
                self.feedback(self.absolute_heading+rotation*.08, True, self.imu_epoch, now)
            self.pose()
            if not self.line.remaining_q and self.state == 'nudging':
                self.micro_count += 1
                self.halt_wheels()
                self.begin_turn('orienting', (self.host_hold_heading if self.host_mode else self.headings[self.station()]), now, allow_inset=False)
            elif not self.line.remaining_q:
                point = self.path[self.path_index]
                self.x4, self.y4 = point[0]*4, point[1]*4
                self.halt_wheels()
                self.path_index += 1
                self.launch_segment(now)
        elif self.state == "waiting":
            error = wrap((self.host_hold_heading if self.host_mode else self.headings.get(self.station(), 0.)) - wrap(self.absolute_heading - self.initial_heading))
            if abs(error) > self.board.turn.tolerance_cd/100 + 1e-9:
                self.begin_turn("orienting", (self.host_hold_heading if self.host_mode else self.headings.get(self.station(), 0.)), now, allow_inset=False)
            elif self.holds[self.leg] and not self.released:
                # Held legs wait for the host's NAV GO instead of a timer, so
                # vision can finish before the next leg starts.
                pass
            elif not self.host_mode and now - self.wait_started >= self.dwell:
                self.leg += 1
                self.released = False
                if self.leg == self.leg_count:
                    self.leg, self.state = self.leg_count - 1, "done"
                else:
                    self.launch(now)
        self.pose()

    def exec_report(self):
        """Dedicated compact status keeps the legacy NAV STAT frame bounded."""
        return ('OK cmd=NAV_EXEC v=1 host={} path_id={} op_id={} state={} x={} y={} '
                'heading_cd={} target_cd={} ready={} imu={} active={} tolerance_cd={} fault={} paused={}').format(
            int(self.host_mode),self.path_id,self.host_op,self.state,round_pulse(self.x4/4),round_pulse(self.y4/4),
            round(wrap(self.absolute_heading-self.initial_heading)*100),round(self.target_heading*100),
            int(self.ready),int(self.fresh(self.board.clock())),sum(1<<i for i,a in enumerate(self.board.axes[:4]) if a['active']),
            self.board.turn.tolerance_cd,self.fault,int(self.board.turn.paused))

    def exec_action(self,args):
        """Mirror one generic operation; never plan a station or centre move."""
        number=self.board.number
        path,op=number(args[1],1,2147483647),number(args[2],1,2147483647)
        kind=number(args[3],0,4); x,y=[number(v,-1000000,1000000) for v in args[4:6]]
        angle=number(args[6],-18000,18000)/100
        speed,accel=number(args[7],1,5000),number(args[8],1,20000)
        if (not self.host_mode or path!=self.path_id or op<=self.host_op or not self.ready or self.obstacle or
            not self.board.armed or not self.fresh(self.board.clock()) or any(a['active'] for a in self.board.axes[:4]) or
            (self.state!='checkpoint' and not (self.state=='waiting' and kind>=3))):
            return 'ERR code=NAV_EXEC_NOT_READY_OR_STALE_OPERATION'
        self.pose()
        if kind==2 and (x!=self.host_next_leg or not 0<=x<self.leg_count or
            max(abs(self.x4/4-self.points[self.route[x]][0]),abs(self.y4/4-self.points[self.route[x]][1]))>.51):
            return 'ERR code=NAV_EXEC_NOT_READY_OR_STALE_OPERATION'
        if kind in (3,4) and (self.state!='waiting' or self.holds[self.leg] and not self.released or kind==4 and self.leg+1!=self.leg_count):
            return 'ERR code=NAV_EXEC_NOT_READY_OR_STALE_OPERATION'
        self.host_op=op; self.move_id=op; self.sps=speed; self.accel=accel
        self.turn_started=self.board.clock(); self.target_heading=angle
        self.stable_count=0;self.stable_revision=self.imu_revision
        if kind==0: self.state='turning'
        elif kind==1:
            self.host_goal=(x,y);self.host_corrections=0
            if abs(wrap(angle-wrap(self.absolute_heading-self.initial_heading)))>5:self.recover_heading(self.board.clock())
            elif max(abs(x-self.x4/4),abs(y-self.y4/4))<=.51:
                self.x4,self.y4=x*4,y*4;self.state='checkpoint'
            else:self.vector_start()
        elif kind==2:
            self.leg=x;self.host_next_leg+=1;self.host_hold_heading=angle
            self.released=False;self.wait_started=self.board.clock();self.state='waiting'
            self.micro_count=self.micro_steps=0
        else:self.state='done' if kind==4 else 'checkpoint'
        return self.exec_report()

    def vector_start(self):
        """Signed wheel projection excludes yaw pulses from finite distance."""
        angle=math.radians(wrap(self.absolute_heading-self.initial_heading))
        dx,dy=self.host_goal[0]-self.x4/4,self.host_goal[1]-self.y4/4
        forward=dx*math.sin(angle)+dy*math.cos(angle)
        lateral=(-dx*math.cos(angle)+dy*math.sin(angle))/LATERAL_SCALE
        weights=[round_pulse(forward+(lateral if i in (0,3) else -lateral))*(-1 if self.mask&(1<<i) else 1) for i in range(4)]
        peak=max(map(abs,weights)); total=sum(v*v for v in weights)
        if not peak:self.fail('move_start');return
        self.vector=dict(weights=weights,peak=peak,remaining=total,total=total,fractions=[0.]*4)
        self.state='moving'
        self.vector_travel = True
        for a,w in zip(self.board.axes,weights):
            a.update(active=int(bool(w)),finite=False,remaining=peak,move_id=self.move_id,target=w*self.sps/peak,velocity=w*self.sps/peak)

    def vector_tick(self,now,dt):
        error=wrap(self.target_heading-wrap(self.absolute_heading-self.initial_heading))
        if now-self.turn_started>=60:self.fail('move_timeout');return
        if abs(error)>5:self.recover_heading(now);return
        v=self.vector
        if v is None:self.fail('move_interrupted');return
        remaining=max(0,v['remaining']*v['peak']/v['total'])
        speed=max(1,min(self.sps,math.sqrt(2*self.accel*remaining)))
        yaw_correction=max(-speed/3,min(speed/3,correction(self.board,error)))
        base=speed-abs(yaw_correction)
        events=[]
        for i,(a,w) in enumerate(zip(self.board.axes[:4],v['weights'])):
            yaw=(1 if i%2==0 else -1)*(-1 if self.mask&(1<<i) else 1)
            rate=base*w/v['peak']+yaw*yaw_correction
            fraction=v['fractions'][i]
            amount=abs(rate)*dt+fraction; count=int(amount);v['fractions'][i]=amount-count
            if rate:
                events.extend(((pulse+1-fraction)/abs(rate),i,1 if rate>0 else -1)
                              for pulse in range(count))
            a.update(active=int(bool(rate)),target=rate,velocity=rate)
        # A serial/browser tick may cover several pulses on every wheel. Apply
        # them in simulated time order, including all simultaneous edges before
        # stopping. Per-wheel batches used to consume the whole distance budget
        # before later wheels moved, inventing drift and failed short returns.
        events.sort()
        elapsed=dt
        for index,(when,i,delta) in enumerate(events):
            self.board.axes[i]['pos']+=delta
            v['remaining']-=delta*v['weights'][i]
            if v['remaining']<=0 and (index+1==len(events) or events[index+1][0]-when>1e-12):
                elapsed=when
                break
        self.pose()
        if self.auto_imu and yaw_correction:self.feedback(self.absolute_heading+yaw_correction*elapsed/5,True,1,now)
        if v['remaining']<=0:
            self.halt_wheels()
            for a in self.board.axes[:4]:a['remaining']=0
            if max(abs(self.x4/4-self.host_goal[0]),abs(self.y4/4-self.host_goal[1]))<=.51:
                self.finish_vector(now)
            else:
                self.vector_start()

    def micro_ready(self, ident, index):
        """Only the current armed, fresh, unreleased held leg owns corrections."""
        return (ident == self.path_id and index == self.leg and self.state == 'waiting' and
                self.leg_count and self.holds[self.leg] and not self.released and
                self.board.armed and self.fresh(self.board.clock()) and
                not any(a['active'] for a in self.board.axes[:4]))

    def station(self):
        """Current leg's station index, or 0 before a route is uploaded."""
        if self.leg_count and self.leg < len(self.route) and self.route[self.leg] is not None:
            return self.route[self.leg]
        return 0

    def report(self):
        index = self.station()
        point = self.points.get(index, (0, 0))
        held = int(bool(self.leg_count and self.state == "waiting" and
                        self.holds[self.leg] and not self.released))
        wait = max(0, int((self.dwell-(self.board.clock()-self.wait_started))*1000)) if self.state == "waiting" else 0
        return ("OK cmd=NAV_STAT state={} ready={} x={} y={} target={} tx={} ty={} leg={} "
                "wait_ms={} move_id={} obstacle={} planner=host_grid heading_cd={} target_cd={} arrival_cd={} imu={} fault={} "
                "wx={} wy={} segment={} segments={} path_id={} route_ready={} "
                "station={} legs={} stations={} held={} paused={}").format(
                    self.state, int(self.ready), round_pulse(self.x4/4), round_pulse(self.y4/4), index,
                    *point, self.leg+1, wait, self.move_id, int(self.obstacle),
                    round(wrap(self.absolute_heading-self.initial_heading)*100), round(self.target_heading*100),
                    round((self.host_hold_heading if self.host_mode else self.headings.get(index, 0.))*100), int(self.fresh(self.board.clock())), self.fault,
                    *(self.path[self.path_index] if self.path_index < len(self.path) else point),
                    min(self.path_index + 1, len(self.path)), len(self.path), self.path_id, int(self.committed),
                    index, self.leg_count, self.station_count, held, int(self.board.turn.paused))

    def dispatch(self, sequence, args):
        board, number = self.board, self.board.number
        if self.state == 'suspended' and args and args[0] not in ('STAT','ALIGN','STOP','IMU','SUSPEND','POSE','RESTORE','OBSTACLE'):
            return 'ERR code=NAV_BUSY'
        if args == ["STAT"]:
            self.pose()
            return self.report()
        if args == ['EXEC']: return self.exec_report()
        if len(args)==9 and args[0]=='EXEC': return self.exec_action(args)
        if args == ['INSET'] or len(args) == 2 and args[0] == 'INSET':
            if len(args) == 2:
                steps = number(args[1],0,100000)
                if steps:return 'ERR code=NAV_INSET_RANGE_OR_BUSY'
                self.clear_upload(); self.inset_steps = steps
            return f'OK cmd=NAV_INSET v=1 steps={self.inset_steps} phase={self.pocket_phase}'
        if args == ['ALIGN']:
            return ('OK cmd=NAV_ALIGN v=2 suspend=1 state={} path_id={} leg={} pending={} used={} max=1000 op_id={}').format(
                self.state, self.path_id, self.leg+1, self.micro_count, self.micro_steps, self.micro_id)
        if ((len(args)==3 and args[0]=='SUSPEND') or
                (len(args)==5 and args[0] in ('POSE','RESTORE'))):
            ident,index=number(args[1],1,2147483647),number(args[2],0,31)
            if (ident != self.path_id or index != self.leg or not self.ready or not self.frame_set or
                    not self.leg_count or not self.holds[self.leg] or self.released or not board.armed or
                    not self.fresh(board.clock()) or board.turn.busy() or any(a['active'] for a in board.axes[:4])):
                return 'ERR code=NAV_HANDOFF_NOT_READY'
            if args[0]=='SUSPEND':
                if self.state!='waiting':return 'ERR code=NAV_HANDOFF_NOT_READY'
                self.pose();self.state='suspended';self.external_tracking=False
            else:
                if self.state!='suspended':return 'ERR code=NAV_HANDOFF_NOT_READY'
                if board.chassis_state in ('moving','jogging','switching'):return 'ERR code=NAV_HANDOFF_NOT_READY'
                x,y=number(args[3],-1000000,1000000),number(args[4],-1000000,1000000)
                release=args[0]=='RESTORE'
                if release and not on_grid((x,y),self.grid):return 'ERR code=NAV_HANDOFF_NOT_READY'
                self.pose();self.x4,self.y4=x*4,y*4;self.external_tracking=not release
                if release:self.state='stopped';self.clear_upload()
            return 'OK cmd=NAV_'+args[0]
        if len(args) == 7 and args[0] == 'NUDGE':
            ident, index = number(args[1],1,2147483647), number(args[2],0,31)
            direction = args[3]
            steps, sps, accel = number(args[4],1,1000), number(args[5],1,5000), number(args[6],1,20000)
            if not self.micro_ready(ident,index) or direction not in ('FWD','BACK','LEFT','RIGHT') or \
                    abs(wrap((self.host_hold_heading if self.host_mode else self.headings[self.station()]) - wrap(self.absolute_heading-self.initial_heading))) > board.turn.tolerance_cd/100 + 1e-9 or \
                    self.micro_steps+steps > 1000 or self.micro_count >= 16:
                return 'ERR code=NAV_MICRO_NOT_READY_OR_LIMIT'
            rows={'FWD':(1,1,1,1),'BACK':(-1,-1,-1,-1),'LEFT':(1,-1,-1,1),'RIGHT':(-1,1,1,-1)}
            signs=[v*(-1 if self.mask & (1<<i) else 1) for i,v in enumerate(rows[direction])]
            yaw=[(-1 if i%2 else 1)*(-1 if self.mask & (1<<i) else 1) for i in range(4)]
            self.line=StraightTrajectory(board,signs,yaw,steps,sps,accel,sequence)
            self.micro_steps+=steps; self.micro_id=sequence
            self.micro_started=board.clock()
            self.target_heading=(self.host_hold_heading if self.host_mode else self.headings[self.station()]);self.state='nudging'
            return 'OK cmd=NAV_NUDGE op_id={}'.format(sequence)
        if len(args) == 3 and args[0] == 'FIX':
            ident,index=number(args[1],1,2147483647),number(args[2],0,31)
            if not self.micro_ready(ident,index) or abs(wrap((self.host_hold_heading if self.host_mode else self.headings[self.station()]) -
                    wrap(self.absolute_heading-self.initial_heading))) > board.turn.tolerance_cd/100 + 1e-9:
                return 'ERR code=NAV_MICRO_NOT_READY'
            self.pose()
            self.x4,self.y4=[v*4 for v in self.points[self.station()]]
            self.previous=[a['pos'] for a in board.axes[:4]]
            self.micro_count,self.micro_steps=0,0
            return 'OK cmd=NAV_FIX'
        if args == ["STOP"]:
            self.stop()
            return "OK cmd=NAV_STOP"
        if len(args) == 4 and args[0] == "IMU":
            angle, valid, epoch = number(args[1], -18000, 18000), number(args[2], 0, 1), number(args[3], 1, 2147483647)
            self.auto_imu = False
            self.feedback(angle/100, bool(valid), epoch, board.clock())
            return 'OK cmd=NAV_IMU paused={}'.format(int(board.turn.paused))
        if len(args) in (3, 6) and args[0] == "RESET":
            x, y = [number(v, -1000000, 1000000) for v in args[1:3]]
            px, py = (x, y) if len(args) == 3 else [number(v, -1000000, 1000000) for v in args[3:5]]
            angle = 0. if len(args) == 3 else number(args[5], -18000, 18000)/100
            if board.armed or self.busy() or board.turn.busy() or any(a["active"] for a in board.axes[:4]):
                return "ERR code=NAV_RESET_REQUIRES_DISARMED_IDLE"
            self.x4, self.y4, self.points[0] = px*4, py*4, (x, y)
            self.departure_heading = angle
            self.previous, self.mask = [a["pos"] for a in board.axes[:4]], board.invert
            self.ready, self.state, self.leg, self.obstacle, self.fault = True, "idle", 0, False, "none"
            self.frame_set = False
            self.micro_count,self.micro_steps=0,0
            self.clear_upload()
            self.path, self.path_index = [], 0
            return self.report()
        if len(args) == 3 and args[0] == "GO":
            ident, index = number(args[1], 1, 2147483647), number(args[2], 0, 31)
            # Releasing names the leg it releases: a late GO for the previous
            # station cannot start the next one.
            if ident != self.path_id or self.state != "waiting" or index != self.leg or \
                    not self.leg_count or not self.holds[self.leg] or self.released or self.micro_count or \
                    not board.armed or not self.fresh(board.clock()) or any(a['active'] for a in board.axes[:4]) or \
                    abs(self.x4/4-self.points[self.station()][0]) > .51 or abs(self.y4/4-self.points[self.station()][1]) > .51:
                return "ERR code=NAV_NOT_HELD_OR_WRONG_LEG"
            self.released = True
            self.wait_started = board.clock()
            return "OK cmd=NAV_GO leg={}".format(index)
        if len(args) == 2 and args[0] == "STATIONS":
            count = number(args[1], 2, 32)
            if self.busy():
                return "ERR code=NAV_BUSY"
            # Resizing discards stale rows from a differently shaped mission,
            # but keeps the origin: `ready` means pulse coordinates exist, and
            # only a physical RESET or a moved origin may clear that.
            self.clear_upload()
            origin = self.points.get(0)
            self.station_count = count
            self.points = {0: origin} if origin is not None else {}
            self.headings = {}
            self.route, self.holds, self.leg_count = [], [], 0
            return "OK cmd=NAV_STATIONS count={}".format(count)
        if len(args) == 2 and args[0] == "LEGS":
            count = number(args[1], 1, 32)
            if self.busy() or not self.station_count:
                return "ERR code=NAV_BUSY_OR_NO_STATIONS"
            self.clear_upload()
            self.leg_count = count
            self.route, self.holds = [None]*count, [0]*count
            return "OK cmd=NAV_LEGS count={}".format(count)
        if len(args) == 4 and args[0] == "LEG":
            if not self.leg_count:
                return "ERR code=NAV_NO_ROUTE"
            index = number(args[1], 0, self.leg_count - 1)
            station = number(args[2], 0, max(0, self.station_count - 1))
            hold = number(args[3], 0, 1)
            if self.busy():
                return "ERR code=NAV_BUSY"
            self.clear_upload()
            self.route[index], self.holds[index] = station, hold
            return "OK cmd=NAV_LEG"
        if len(args) == 3 and args[0] == "ATTITUDE":
            index = number(args[1], 0, max(0, self.station_count - 1))
            angle = number(args[2], -18000, 18000)
            if self.busy() or not self.station_count:
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
                index = number(args[1], 0, max(0, self.station_count - 1))
                values = tuple(number(v, -1000000, 1000000) for v in args[2:])
            else:
                values = (number(args[1], 1, 5000), number(args[2], 1, 20000), number(args[3], 0, 60000))
            if self.busy():
                return "ERR code=NAV_BUSY"
            self.clear_upload()
            if args[0] == "POINT":
                if not self.station_count:
                    return "ERR code=NAV_NO_STATIONS"
                if index == 0 and self.points.get(0) != values:
                    self.ready = False
                self.points[index] = values
            else:
                self.sps, self.accel, dwell = values
                self.dwell = dwell/1000.
                self.inset_steps = 0
            return "OK cmd=NAV_" + args[0]
        if len(args) == 5 and args[0] == "BEGIN":
            ident, count = number(args[1], 1, 2147483647), number(args[2], 1, 64)
            start = tuple(number(v, -1000000, 1000000) for v in args[3:])
            if not self.ready or not self.configured() or count < self.leg_count or \
                    self.busy() or board.turn.busy() or any(a["active"] for a in board.axes[:4]):
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
            entry = tuple(number(v, -1000000, 1000000) for v in args[3:5]) + \
                (number(args[5], 0, max(0, self.leg_count - 1)),)
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
                    not self.validate_staged():
                return "ERR code=NAV_PATH_INVALID_OR_INCOMPLETE"
            self.uploaded, self.path_id = list(self.staged), ident
            self.committed, self.uploading = True, False
            return "OK cmd=NAV_COMMIT"
        if args == ["START"] or len(args) in (2,3) and args[0] == "START":
            requested = number(args[1], 1, 2147483647) if len(args) >= 2 else 0
            if len(args)==3 and args[2]!='HOST':return 'ERR code=BAD_ARGUMENT'
            if not self.committed or requested and requested != self.path_id or not self.ready or not self.configured() or self.obstacle or self.busy() or board.turn.busy() or not board.armed or \
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
            self.released = False
            self.micro_count,self.micro_steps=0,0
            self.previous = [a["pos"] for a in board.axes[:4]]
            self.host_mode=len(args)==3
            self.host_op=0;self.host_next_leg=0
            if not self.frame_set:
                self.target_heading=self.departure_heading
                self.initial_heading = wrap(self.absolute_heading-self.departure_heading)
            self.fault = "none"
            self.frame_set = True
            if self.host_mode:self.state='checkpoint'
            else:self.launch(board.clock())
            return "OK cmd=NAV_START state=accepted host={} move_id={}".format(int(self.host_mode),sequence)
        if len(args) == 2 and args[0] == "OBSTACLE":
            self.obstacle = bool(number(args[1], 0, 1))
            if self.obstacle and self.busy():
                self.stop()
                self.state = "blocked"
            return self.report()
        if args == ["REPLAN"]:
            return "ERR code=NAV_PLANNER_NOT_IMPLEMENTED"
        return "ERR code=BAD_ARGUMENT"
