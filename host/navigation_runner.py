"""NX-owned route/arrival/turn-excursion scheduler.

Only bounded generic MCU actions move the car. ACKs never mean completion;
matching post-ACK queries do. Runs under Console.lock; no serial/thread owner.
"""
import math
import time
from .navigation import arrival, mission_waypoints, validated
from .navigation_grid import round_pulse


def wrap(angle):
    """Shortest signed map-angle difference in centidegrees."""
    return (angle + 18000) % 36000 - 18000


def turn_actions(config, origin, before, target, tolerance, transit=False):
    """Plan a deliberate heading change; measurement drift never adds an inset.

    The pulse cap is the busiest translation wheel, matching the former input.
    Return is explicit, followed by yaw settling without another excursion.
    Transit turns at the middle-lane crossing stay there even on asymmetric
    grids. Arrival orientation retains its configured inward/return sequence.
    """
    actions = []
    changed = abs(wrap(target-before)) > tolerance
    center_transit = transit and tuple(origin) == (config['grid']['x'][1], config['grid']['y'][1])
    if config['turn_inset_enabled'] and changed and not center_transit:
        dx = (config['grid']['x'][0]+config['grid']['x'][2])/2-origin[0]
        dy = (config['grid']['y'][0]+config['grid']['y'][2])/2-origin[1]
        angle = math.radians(before/100)
        forward = dx*math.sin(angle)+dy*math.cos(angle)
        lateral = -dx*math.cos(angle)+dy*math.sin(angle)
        peak = max(abs(forward+lateral),abs(forward-lateral))
        scale = min(1.,config['turn_inset_steps']/peak) if peak else 0.
        inside = (round_pulse(origin[0]+dx*scale),round_pulse(origin[1]+dy*scale))
        if inside != tuple(origin):
            actions.append((1,*inside,before,'inward'))
            actions.append((0,0,0,target,'turn'))
            actions.append((1,*origin,target,'return'))
            actions.append((0,0,0,target,'settle'))
            return actions
    return [(0,0,0,target,'turn' if changed else 'correct')]


class NavigationRunner:
    """Single in-flight operation; STOP/reconnect retire every late reply."""
    def __init__(self, owner):
        self.owner=owner; self.generation=0; self.state='idle'; self.pending=None
        self.error=''; self.phase=''; self.leg=0; self.op=0; self.path_id=None
        # Vision gives an absolute heading at C/D; the difference to the board's
        # own yaw bookkeeping becomes a bias applied to every commanded angle.
        # It is recomputed per vision calibration, never accumulated, and it
        # never changes the board's map frame.
        self.bias_cd=0

    @property
    def active(self):
        return self.state not in ('idle','done','stopped','fault')

    def set_heading_bias(self, bias_cd):
        """Adopt a vision-verified heading bias for all later turn targets."""
        self.bias_cd=int(bias_cd)

    def snapshot(self):
        return dict(state=self.state,phase=self.phase,leg=self.leg+1,
                    op_id=self.op,path_id=self.path_id,error=self.error,active=self.active,
                    bias_cd=self.bias_cd)

    def start(self, config, path_id):
        """Pin saved settings only after START HOST was acknowledged."""
        self.generation+=1; self.config=validated(config); self.path_id=str(path_id)
        self.state='initial'; self.pending=None; self.error=''; self.phase=''; self.leg=0; self.op=0
        self.queue=[]; self.last_poll=0.; self.group_started=time.monotonic()
        self.send('NAV EXEC','initial')

    def stop(self, reason='', fault=False, send=False):
        """Cancel callbacks before a priority stop; never retry motion or arm."""
        was=self.active
        self.generation+=1; self.pending=None; self.queue=[]
        self.state='fault' if fault else 'stopped'; self.error=reason
        if was and send and self.owner.connection:
            self.owner.connection.submit('STOP')

    def fail(self, reason):
        self.stop(reason,True,True)
        self.owner.note('error','NX 巡航停止：'+reason)

    def finish_external(self, path_id):
        """Release the final station owner after Mission verifies RESTORE.

        This never moves the car or accepts completion for another path. The
        caller must have queried the restored, stopped MCU before invoking it.
        """
        if self.state != 'external' or self.path_id != str(path_id):
            return False
        self.generation += 1
        self.pending = None
        self.queue = []
        self.state = self.phase = 'done'
        return True

    def send(self, body, purpose):
        tag=('navrunner',self.generation,self.op,purpose)
        self.pending=(tag,body,purpose,time.monotonic())
        if not self.owner.connection or not self.owner.connection.submit(body,tag=tag):
            self.fail('发送队列已满或串口已关闭。')

    def plan_leg(self):
        """NX chooses segment headings and optional arrival orientation per visit."""
        origin=self.position
        # X has a deliberate off-lane departure: inset, turn to map zero, then
        # translate directly to A without returning to X. Generic MCU vectors
        # retain heading and integrate XY; route upload still validates X->A.
        start = self.config['stations'][self.config.get('start_station', 0)]
        x_start = (self.leg == 0 and self.config.get('start_station', 0) != 0 and start['name'] == 'X' and
                   origin == (start['x'], start['y']))
        if x_start:
            inset_config = dict(self.config, turn_inset_enabled=True)
            actions = turn_actions(inset_config, origin, self.planned_heading, 0, self.tolerance)
            self.queue.extend(a for a in actions if a[-1] not in ('return', 'settle'))
            goal = self.config['stations'][self.config['route'][0]['station']]
            self.queue.append((1, goal['x'], goal['y'], 0, 'travel'))
            self.planned_heading = 0
            origin = (goal['x'], goal['y'])
        for x,y,index in self.entries:
            if index != self.leg: continue
            if x_start: continue
            dx,dy=x-origin[0],y-origin[1]
            if dx or dy:
                target=9000 if dx>0 else -9000 if dx<0 else 0 if dy>0 else 18000
                self.queue.extend(turn_actions(self.config,origin,self.planned_heading,target,self.tolerance,transit=True))
                self.queue.append((1,x,y,target,'travel'))
                self.planned_heading=target; origin=(x,y)
        enabled,angle=arrival(self.config,self.leg)
        if enabled:
            target=round(angle*100)
            self.queue.extend(turn_actions(self.config,origin,self.planned_heading,target,self.tolerance))
            self.planned_heading=target
        self.queue.append((2,self.leg,0,self.planned_heading,'arrival'))
        self.state='ready'; self.group_started=time.monotonic()

    def reply(self, body, reply, tag):
        """Only a tagged query following the matching ACK advances the plan."""
        if not self.active or not self.pending or tag!=self.pending[0]: return
        purpose=self.pending[2]; self.pending=None
        if not reply.ok:
            self.fail('命令被拒绝：'+reply.body); return
        f=reply.fields
        if f.get('path_id')!=self.path_id or f.get('ready')!='1' or f.get('imu')!='1' or f.get('fault','none')!='none':
            self.fail('路径、坐标或航向反馈失效。'); return
        if purpose=='initial':
            if f.get('cmd')!='NAV_EXEC' or f.get('v')!='1' or f.get('host')!='1' or f.get('state')!='checkpoint' or f.get('active')!='0':
                self.fail('固件未进入逐步执行模式。'); return
            # A stopped turn may retain an unreached target. Only the measured
            # map yaw describes the body frame for the first inward move.
            self.position=(int(f['x']),int(f['y'])); self.planned_heading=int(f['heading_cd'])
            self.tolerance=int(f['tolerance_cd']); self.entries=mission_waypoints(self.config,self.position)
            self.plan_leg(); self.tick(); return
        if purpose=='accept':
            if f.get('op_id')!=str(self.op): self.fail('动作编号未确认。'); return
            self.state='executing'; self.last_poll=time.monotonic()
            self.send('NAV EXEC','complete'); return
        if purpose=='complete':
            if f.get('op_id')!=str(self.op): self.fail('动作被替换或过期。'); return
            expected='waiting' if self.kind==2 else 'done' if self.kind==4 else 'checkpoint'
            if f.get('state') in ('stopped','fault','blocked','suspended'):
                self.fail('动作中断：'+f.get('state','unknown')); return
            if f.get('state')==expected and f.get('active')=='0':
                self.position=(int(f['x']),int(f['y']))
                if self.kind==2:
                    self.state='waiting'; self.wait_started=None; self.last_poll=0.
                elif self.kind==4: self.state='done'; self.phase='done'
                else: self.state='ready'
                self.tick()
            return
        if purpose=='wait':
            if f.get('state')=='suspended':
                self.state='external'; return
            if f.get('state') not in ('waiting','orienting','nudging'):
                # A station task may release ownership and replan a new path.
                self.fail('站点等待期间状态改变。'); return
            if f.get('held')=='1' or f.get('state')!='waiting':
                self.wait_started=None; return
            now=time.monotonic()
            if self.wait_started is None: self.wait_started=now
            if now-self.wait_started >= self.config['dwell_ms']/1000:
                last=self.leg+1==len(self.config['route'])
                self.queue=[(4 if last else 3,0,0,self.planned_heading,'done' if last else 'leave')]
                if not last: self.leg+=1
                # A held task or dwell is stationary waiting, outside the next
                # motion's deadline. The inset/turn/return group stays bounded.
                self.state='ready'; self.group_started=now
                self.tick()

    def tick(self):
        """Advance without sleeping; continues when browser tabs are closed."""
        if not self.active: return
        now=time.monotonic()
        if self.pending:
            if now-self.pending[3]>2: self.fail('应答超时，动作未重发。')
            return
        if self.state=='external': return
        if self.state=='waiting':
            if now-self.last_poll>=.02:
                self.last_poll=now; self.send('NAV STAT','wait')
            return
        if now-self.group_started>60:
            self.fail('动作流程超过 60 秒。'); return
        if self.state=='executing':
            if now-self.last_poll>=.02:
                self.last_poll=now; self.send('NAV EXEC','complete')
            return
        if self.state=='ready':
            if not self.queue:
                self.plan_leg()
            previous_phase=self.phase
            self.kind,x,y,heading,self.phase=self.queue.pop(0)
            # A turn excursion shares one deadline through its final settle.
            if self.phase in ('inward','travel','arrival','leave','done') or (self.phase in ('turn','correct') and previous_phase!='inward'):
                self.group_started=now
            self.op+=1
            # The queue stores configured map angles; only the wire value
            # carries the vision-derived bias, so telemetry stays comparable.
            # With no bias the configured angle is sent verbatim (180 stays
            # 180, not -180) so existing telemetry and tests are unchanged.
            angle = heading if not self.bias_cd else wrap(heading+self.bias_cd)
            self.send('NAV EXEC {} {} {} {} {} {} {} {}'.format(
                self.path_id,self.op,self.kind,x,y,angle,
                self.config['sps'],self.config['accel']),'accept')
