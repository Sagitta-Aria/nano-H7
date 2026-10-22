"""Nonblocking held-station task runner, owned and ticked by Console.

No camera, serial port or thread is opened here. Vision supplies timestamped
results for a specific run/request; commands use the existing serial owner.
After visual alignment FIX corrects the vehicle reference pose at the known
station, so the next leg departs from the calibrated location, never retraces.
"""
import copy
import json
import math
import os
import tempfile
import time
from pathlib import Path
from .vision_alignment import DEFAULT as CALIBRATION_DEFAULT, validated as validate_calibration, alignment_step
from .task_code import parse_task_code
from .qr_search import QRSearch
from .material_flow import MaterialStation

CONFIG_PATH = Path(__file__).with_name('mission.json')
ACTIVE_NAV = ('turning', 'moving', 'orienting', 'waiting', 'nudging', 'blocked', 'suspended')
DEFAULT = {'timeout_s': 480, 'leg_timeout_s': 120, 'action_timeout_s': 30,
           'vision_max_age_ms': 500, 'calibration': copy.deepcopy(CALIBRATION_DEFAULT), 'legs': {}}


def integer(value, low, high, label):
    """Reject bools/fractions before values can become physical commands."""
    if type(value) is not int or not low <= value <= high:
        raise ValueError('{} 必须是 {}～{} 的整数。'.format(label, low, high))
    return value


def validated(config):
    """Validate a complete plan without inventing positions or calibration."""
    if not isinstance(config, dict) or set(config) != set(DEFAULT):
        raise ValueError('任务配置字段不完整。')
    value = copy.deepcopy(config)
    for k, lo, hi in (('timeout_s',1,14400),('leg_timeout_s',1,600),
                      ('action_timeout_s',1,300),('vision_max_age_ms',50,1000)):
        integer(value[k],lo,hi,k)
    value['calibration'] = validate_calibration(value['calibration'])
    if not isinstance(value['legs'],dict):
        raise ValueError('legs 必须按从 1 开始的路线段号配置。')
    for leg,actions in value['legs'].items():
        if not isinstance(leg,str) or not leg.isdigit() or str(int(leg))!=leg or not 1<=int(leg)<=32:
            raise ValueError('任务段号范围 1～32。')
        if not isinstance(actions,list) or len(actions)>64:
            raise ValueError('每段最多 64 个动作。')
        for action in actions:
            if not isinstance(action,dict):
                raise ValueError('动作必须是对象。')
            op=action.get('op')
            required={'align':{'op','kind','target'}, 'scan':{'op','key'},
                      'ring_row':{'op','outward_steps','right_steps'}, 'task_code':{'op'},
                      'material':{'op','station','batch','outward_steps','right_steps'},
                      'servo':{'op','channel','angle','settle_ms'},
                      'aux':{'op','axis','position','sps','accel'}, 'wait':{'op','ms'}}
            if not isinstance(op,str) or op not in required or set(action)!=required[op]:
                raise ValueError('动作字段无效：{}'.format(op))
            if op == 'task_code':
                pass
            elif op in ('ring_row', 'material'):
                integer(action['outward_steps'],0,100000,'外移脉冲')
                integer(action['right_steps'],0,100000,'右移脉冲')
                if op == 'material':
                    integer(action['batch'],1,2,'物料批次')
                    if action['station'] not in ('B','C','D'):
                        raise ValueError('物料站点只能是 B/C/D。')
            elif op=='align':
                if action['kind'] not in ('block','ring'):
                    raise ValueError('视觉种类应为 block 或 ring。')
                target=action['target']
                if isinstance(target,dict):
                    if set(target)!={'scan','index','map'} or not isinstance(target['scan'],str) or not isinstance(target['map'],dict):
                        raise ValueError('动态目标需要 scan、index、map。')
                    integer(target['index'],0,127,'二维码字符下标')
                    if not target['map'] or any(not isinstance(k,str) or len(k)!=1 or not isinstance(v,str) or not v or len(v)>32 for k,v in target['map'].items()):
                        raise ValueError('二维码字符映射无效。')
                elif not isinstance(target,str) or not target or len(target)>32:
                    raise ValueError('请输入明确的视觉目标。')
            elif op=='scan':
                if not isinstance(action['key'],str) or not action['key'] or len(action['key'])>32:
                    raise ValueError('二维码结果名称无效。')
            elif op=='servo':
                integer(action['channel'],1,2,'舵机通道')
                from .servo_config import SERVO_LAYOUT
                a=action['angle']
                if type(a) not in (int,float) or not 0<=a<=SERVO_LAYOUT[action['channel']]['travel'] or not math.isfinite(a):
                    raise ValueError('舵机物理角度越界。')
                integer(action['settle_ms'],100,10000,'舵机等待时间')
            elif op=='aux':
                integer(action['axis'],5,6,'辅助轴');integer(action['position'],-1000000,1000000,'辅助轴位置')
                integer(action['sps'],1,3000,'辅助轴速度');integer(action['accel'],1,20000,'加速度')
            else:
                integer(action['ms'],0,60000,'等待时间')
    return value


class MissionSettings:
    """Persist only the plan; no implicit arming, origin reset or movement."""
    def __init__(self,path=CONFIG_PATH):
        self.path=Path(path)
        self.value=validated(json.loads(self.path.read_text(encoding='utf-8'))) if self.path.exists() else copy.deepcopy(DEFAULT)

    def snapshot(self):
        return copy.deepcopy(self.value)

    def save(self,value):
        value=validated(value);self.path.parent.mkdir(parents=True,exist_ok=True)
        name=None
        try:
            with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=self.path.parent,delete=False) as f:
                name=f.name;json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
            os.replace(name,self.path)
        finally:
            if name and os.path.exists(name):os.unlink(name)
        self.value=value
        return self.snapshot()


class Mission:
    """Single-owner state machine; all entry points run under Console.lock.

    Only tagged ACKs advance commands; completion requires a subsequent tagged
    status query. Generation changes invalidate replies after STOP/reconnect.
    """
    def __init__(self,console):
        self.console=console;self.generation=0;self.state='idle';self.error=''
        self.pending=None;self.request=None;self.scans={};self.leg=0;self.index=0
        self.last_nav=0.;self.sequence=0;self.sample=None;self.run_id=''
        self.leg_base = 0
        self.ring_options = None
        self.task_plan = None
        self.qr_status = ''
        self.camera_token = None
        self.uses_task_code = False
        self.qr_search = None
        self.qr_budget_at = None
        self.material = None
        self.tray = {}
        self.d_layout = {}
        self.d_heading = None
        self.material_mode = False

    @property
    def active(self):
        return self.state not in ('idle','done','stopped','fault')

    def snapshot(self):
        return {'state':self.state,'active':self.active,'error':self.error,'run_id':self.run_id,
                'leg':self.leg,'action':self.index+1,'request':copy.deepcopy(self.request),'scans':dict(self.scans),
                'ring_task':copy.deepcopy(self.ring_options), 'task_plan':copy.deepcopy(self.task_plan),
                'qr_status':self.qr_status, 'uses_task_code':self.uses_task_code,
                'qr_search':self.qr_search.snapshot() if self.qr_search else None,
                'material':self.material.snapshot() if self.material else None,
                'tray':copy.deepcopy(self.tray), 'd_layout':copy.deepcopy(self.d_layout)}

    def start(self,config,route):
        """Pin both configuration snapshots for this run, then upload via Console."""
        if self.active:raise ValueError('任务已经在运行。')
        self.config=validated(config);self.route=copy.deepcopy(route)
        self.material_offsets = self.console.material_settings.offsets()
        if any(a.get('op') == 'material' and a.get('station') == 'D' and a.get('batch') == 2
               for actions in self.config['legs'].values() for a in actions):
            from .pick_place import stacked_config
            stacked_config(self.console.pick_place.settings.snapshot())
        self.leg_base = 0
        self.ring_options = None
        for i, step in enumerate(route['route'], 1):
            slot = step.get('task', 0)
            if slot == 1:
                self.config['legs'][str(i)] = [dict(op='ring_row', outward_steps=step['outward_steps'], right_steps=step['right_steps'])]
            elif slot == 4:
                self.config['legs'][str(i)] = [dict(op='task_code')]
            elif slot in (2,3) and not self.config['legs'].get(str(i)):
                raise ValueError('第 {} 段的子任务 {} 尚未配置动作。'.format(i,slot))
        qr_legs = [int(key) for key, actions in self.config['legs'].items()
                   if any(a['op'] == 'task_code' for a in actions)]
        if len(qr_legs) > 1:
            raise ValueError('每次运行只能在一次到达A点时锁定任务码。')
        qr_leg = qr_legs[0] if qr_legs else None
        self.material_mode = any(a['op'] == 'material' for actions in self.config['legs'].values() for a in actions)
        if self.material_mode and qr_leg is None:
            raise ValueError('物料全流程必须先在A点锁定任务码。')
        if qr_leg is not None:
            if not 1 <= qr_leg <= len(route['route']):
                raise ValueError('任务码动作的路线段不存在。')
            station = route['stations'][route['route'][qr_leg-1]['station']]
            if station['name'].upper() != 'A' or len(self.config['legs'][str(qr_leg)]) != 1:
                raise ValueError('任务码切换必须独占A点的一次到站动作。')
            if not getattr(self.console, 'task_cameras', None):
                raise ValueError('任务码流程需要NX双摄像头服务。')
            if self.console.nav_align.get('suspend') != '1':
                raise ValueError('A点二维码搜索需要NAV SUSPEND / POSE / RESTORE支持。')
        for key, actions in self.config['legs'].items():
            for a in actions:
                if a['op'] in ('ring_row', 'material'):
                    if len(actions) != 1:
                        raise ValueError('圆列子任务必须独占该站点动作列表。')
                    if a['op'] == 'material':
                        if not 1 <= int(key) <= len(route['route']):
                            raise ValueError('物料动作引用了不存在的路线段。')
                        station = route['stations'][route['route'][int(key)-1]['station']]
                        if station['name'].upper() != a['station'] or int(key) <= qr_leg:
                            raise ValueError('物料动作必须位于扫码之后的对应站点。')
                    self.ring_parameters(int(key), a)
                    c = self.console
                    if c.nav_align.get('suspend') != '1':
                        raise ValueError('巡航与独立底盘控制交接需要 H743 通用 NAV SUSPEND / POSE / RESTORE 接口；当前固件尚不支持。')
                    deferred_camera = qr_leg is not None and qr_leg < int(key)
                    if not deferred_camera and (not c.camera_alignment.hub or c.camera_alignment.hub.snapshot()['state'] != 'ready'):
                        raise ValueError('圆列巡航子任务需要先打开 NX 摄像头一。')
        for key,actions in self.config['legs'].items():
            if int(key)>len(route['route']) or (actions and not route['route'][int(key)-1]['hold']):
                raise ValueError('有动作的路线段必须存在并勾选到点挂起。')
            if any(a['op']=='align' for a in actions) and not self.config['calibration']['verified']:
                raise ValueError('视觉对准需要先确认相机目标点与车辆站点坐标、像素到脉冲标定。')
        # Every held leg must be intentionally configured, even a pass-through [].
        for i,leg in enumerate(route['route'],1):
            if leg['hold'] and str(i) not in self.config['legs']:
                raise ValueError('挂起的第 {} 段尚未配置任务。'.format(i))
        import uuid
        self.generation+=1;self.run_id=uuid.uuid4().hex
        self.started=self.stage_started=self.leg_started=time.monotonic();self.last_nav=self.started
        self.last_query=0.
        self.state='uploading';self.error='';self.pending=None;self.request=None;self.scans={}
        self.leg=self.index=0;self.path_id=None;self.last_seen_frame={};self.last_capture={}
        self.vision_sessions={};self.retired_sessions=set()
        self.task_plan = None
        self.material = None
        self.tray = {}
        self.d_layout = {}
        self.d_heading = None
        self.qr_search = None
        self.qr_budget_at = None
        self.qr_status = '等待到达A点后读取新任务码' if qr_leg else ''
        self.uses_task_code = qr_leg is not None
        self.camera_token = self.console.task_cameras.request('2') if qr_leg else None

    def stop(self,reason='已停止',fault=False,send=True):
        """Invalidate all delayed work before issuing priority STOP."""
        was=self.active
        if self.uses_task_code and getattr(self.console, 'task_cameras', None):
            self.console.task_cameras.cancel()
        if self.state.startswith(('ring_', 'material_')) and self.console.camera_alignment.active:
            self.console.camera_alignment.stop(reason, fault=fault, send=False)
        if self.material and self.console.pick_place.active:
            self.console.pick_place.stop(reason, fault=fault, send=False)
        self.generation+=1;self.pending=None;self.request=None;self.sample=None
        self.state='fault' if fault else 'stopped';self.error=reason
        if was and send and self.console.connection:self.console.submit('STOP')

    def fail(self,message):
        self.stop(message,True)

    def send(self,body,purpose):
        self.sequence+=1;tag=('mission',self.generation,self.sequence)
        self.pending=(tag,body,purpose,time.monotonic())
        if not self.console.connection or not self.console.connection.submit(body,tag=tag):
            self.fail('任务发送队列已满或连接关闭。')

    def ring_parameters(self, leg, action):
        """Pin map-relative outward search and a legal exit before any motion.

        Digit 2 restores the configured station pose. Local search/grasp travel
        is intentionally uncounted; the final right translation is odometry.
        Station heading is a planning reference, never a new IMU yaw origin.
        Grid cruise requires the exit to remain on a lane.
        """
        from .navigation import arrival
        if not 1 <= leg <= len(self.route['route']):
            raise ValueError('圆列动作引用了不存在的路线段。')
        step = self.route['route'][leg-1]
        station = self.route['stations'][step['station']]
        grid = self.route['grid']
        enabled, angle_deg = arrival(self.route,leg-1)
        if not enabled: raise ValueError('圆列子任务需要明确启用该次到点调姿。')
        angle = math.radians(angle_deg)
        right = (math.cos(angle), -math.sin(angle))
        forward = (math.sin(angle), math.cos(angle))
        x,y = station['x'],station['y']
        boundaries = [(x-grid['x'][0],(-1,0)), (grid['x'][2]-x,(1,0)),
                      (y-grid['y'][0],(0,-1)), (grid['y'][2]-y,(0,1))]
        _, outward = min(boundaries, key=lambda b:b[0])
        vectors = {'RIGHT':right, 'LEFT':tuple(-v for v in right),
                   'FWD':forward, 'BACK':tuple(-v for v in forward)}
        direction = max(vectors, key=lambda k:sum(a*b for a,b in zip(vectors[k],outward)))
        # 退出点就是到站点本身：视觉阶段实际移动了多少由板端里程积分给出，
        # 上位机按实测差回位后再声明坐标，不再用固定右移脉冲去预测。
        exit_xy = [x, y]
        self.ring_return_attempts = 0
        return dict(outward_steps=action['outward_steps'],right_steps=action['right_steps'],
                    outward_direction=direction,reference=[x,y],exit=exit_xy,
                    arrival_heading_deg=angle_deg)

    def ring_tick(self, now):
        """Coordinate the real camera executor, measured return and remaining route."""
        c = self.console
        if now-self.stage_started > 360:
            self.fail('圆列子任务超时。'); return
        if self.state == 'ring_start':
            # Declare the arrival pose BEFORE any vision motion, so the board's
            # own odometry integrates the whole calibration phase and the return
            # below is measured rather than predicted.
            self.state='ring_pose'
            self.send(self.ring_pose_command('POSE',self.ring_options['reference']),'ring_pose')
            return
        if self.state == 'ring_pose':
            if now-self.last_query >= .25:
                self.last_query=now
                self.send('NAV STAT','ring_pose_check')
            return
        if self.state == 'ring_fix':
            if now-self.last_query >= .25:
                self.last_query=now
                self.send('NAV STAT','ring_return')
            return
        if self.state == 'ring_return_move':
            if now-self.last_query >= .1:
                self.last_query=now
                self.send('CHASSIS STAT','ring_return_status')
            return
        if self.state == 'ring_vision':
            a = c.camera_alignment
            if a.state in ('fault','stopped'):
                self.fail('圆列子任务停止：'+a.error); return
            if a.state == 'done':
                seq = a.ring_sequence
                if not seq or [r['digit'] for r in seq.recognized] != [1,2,3,2] or not all(r['aligned'] for r in seq.recognized):
                    self.fail('圆列子任务尚未完成 1→2→3→2 定位。'); return
                self.state='ring_fix'
                self.last_query=0.
                return
        elif self.state == 'ring_upload':
            if self.ring_upload:
                body=self.ring_upload.pop(0);self.send(body,'ring_upload')
            return
        if now-self.last_query >= .25:
            self.last_query=now;self.send('NAV STAT','query')

    def ring_pose_command(self, command, point):
        """Declare an NX-verified XY in the unchanged departure frame."""
        return 'NAV {} {} {} {} {}'.format(command,self.path_id,self.leg-self.leg_base-1,*point)

    def ring_restore(self):
        """Commit only the accepted finite exit endpoint, then allow replanning."""
        self.state='ring_end'
        self.send(self.ring_pose_command('RESTORE',self.ring_options['exit']),'ring_end')

    RING_RETURN_TOLERANCE_PULSES = 20
    RING_HEADING_BIAS_LIMIT_CD = 600

    def ring_return_pose(self, f):
        """Drive back to the arrival point from the board's own odometry.

        The vision phase moves the car, so the exit cannot be predicted from a
        fixed right-shift. Everything after the early POSE is integrated by the
        board; this loop removes the measured difference with body-frame axis
        moves, then declares the arrival XY.
        """
        if (f.get('state')!='suspended' or f.get('ready')!='1' or
                f.get('path_id')!=self.path_id or f.get('imu')!='1'):
            self.fail('视觉定位后的导航交接状态无效。');return
        x,y=int(f['x']),int(f['y'])
        dx,dy=self.ring_options['reference'][0]-x,self.ring_options['reference'][1]-y
        if max(abs(dx),abs(dy))<=self.RING_RETURN_TOLERANCE_PULSES:
            self.ring_heading_bias(f);return
        self.ring_return_attempts=getattr(self,'ring_return_attempts',0)+1
        if self.ring_return_attempts>8:
            self.fail('视觉任务后的回位脉冲积分未收敛。');return
        angle=math.radians(int(f['heading_cd'])/100)
        forward=round(dx*math.sin(angle)+dy*math.cos(angle))
        right=round(dx*math.cos(angle)-dy*math.sin(angle))
        if abs(forward)>=abs(right):
            direction,steps=('FWD' if forward>0 else 'BACK'),abs(forward)
        else:
            direction,steps=('RIGHT' if right>0 else 'LEFT'),abs(right)
        if not steps:
            self.ring_heading_bias(f);return
        cfg=self.console.camera_alignment.config
        self.state='ring_return_accept'
        self.last_query=0.
        self.send('CHASSIS STRAIGHT {} {} {} {}'.format(
            direction,steps,cfg['ring_sequence']['sps'],cfg['tracking']['accel']),'ring_return_accept')

    def ring_heading_bias(self, f):
        """Adopt the vision-verified heading for every later turn target.

        The map frame and the board's yaw origin are untouched; only the angle
        handed to each NAV EXEC carries the bias, so later legs stop drifting
        with the accumulated yaw error.
        """
        expected=round(self.ring_options['arrival_heading_deg']*100)
        delta=(int(f['heading_cd'])-expected+18000)%36000-18000
        if abs(delta)>self.RING_HEADING_BIAS_LIMIT_CD:
            self.fail('视觉平行后的航向偏差 {:.2f}° 超过 ±{:.0f}°，请检查圆列布局与到达姿态。'.format(
                delta/100,self.RING_HEADING_BIAS_LIMIT_CD/100));return
        self.console.navigator.set_heading_bias(delta)
        self.ring_restore()

    def target(self,action):
        """Resolve a configured QR character mapping, never infer color rules."""
        target=action['target']
        if isinstance(target,dict):
            text=self.scans.get(target['scan'],'');i=target['index']
            if i>=len(text) or text[i] not in target['map']:
                raise ValueError('二维码任务不匹配配置的目标映射。')
            return target['map'][text[i]]
        return target

    def vision(self,value):
        """Accept the requested target and a fresh, increasing source/session frame.

        Source uses Unix captured_at seconds (synchronised clocks), not time of
        HTTP delivery. request_id prevents an old station's result from acting;
        session_id permits producer restarts without accepting retired sessions.
        """
        r=self.request
        if not self.active or self.state!='vision' or not r or not isinstance(value,dict):
            raise ValueError('当前没有等待中的视觉请求。')
        for k in ('run_id','request_id','kind','target'):
            if value.get(k)!=r[k]:raise ValueError('视觉结果不属于当前请求。')
        captured=value.get('captured_at')
        if type(captured) not in (float,int) or not 0<captured<1e11:
            raise ValueError('视觉时间戳无效。')
        age=time.time()-captured
        source=value.get('source');frame=value.get('frame_id')
        if not isinstance(source,str) or not source or len(source)>64 or type(frame) is not int or frame<0:
            raise ValueError('视觉来源或帧号无效。')
        if age<-.1 or age>self.config['vision_max_age_ms']/1000 or not math.isfinite(age):
            raise ValueError('视觉帧已过期或时间戳无效。')
        if value['captured_at']<r['after']:raise ValueError('视觉帧早于当前请求。')
        # Legacy producers retain monotonic-frame semantics. Restartable producers
        # must identify each process lifetime; never infer a restart from rollback.
        session=value.get('session_id','legacy')
        if not isinstance(session,str) or not session or len(session)>64:
            raise ValueError('视觉会话无效。')
        identity=(source,session)
        if identity in self.retired_sessions:raise ValueError('视觉会话已结束。')
        if frame<=self.last_seen_frame.get(identity,-1):raise ValueError('重复或倒序视觉帧。')
        if captured<=self.last_capture.get(source,0):raise ValueError('重复或倒序采集时间。')
        if value.get('found') is True and r['kind']=='qr':
            text=value.get('text')
            if not isinstance(text,str) or not text or len(text)>128:raise ValueError('二维码内容无效。')
        elif value.get('found') is True:
            for k in ('error_dx','error_dy'):
                if type(value.get(k)) not in (int,float) or abs(value[k])>10000 or not math.isfinite(value[k]):
                    raise ValueError('视觉偏差无效。')
        if identity not in self.last_seen_frame and len(self.last_seen_frame)>=256:
            raise ValueError('视觉会话数量超限，请停止任务并检查生产者。')
        previous=self.vision_sessions.get(source)
        if previous is not None and previous!=session:
            self.retired_sessions.add((source,previous))
            self.stable=0;self.sample=None;self.stable_source=None
        self.vision_sessions[source]=session
        self.last_seen_frame[identity]=frame;self.last_capture[source]=captured
        if value.get('found') is not True:
            self.stable=0;self.sample=None;return
        self.sample=copy.deepcopy(value)

    def next_action(self):
        self.index+=1;self.state='actions';self.request=None;self.sample=None
        self.iterations=0
        self.stage_started=time.monotonic()

    def begin_vision(self,action):
        self.sequence+=1;self.stable=0;self.sample=None
        self.stable_source=None;self.stable_captured=0.
        self.request={'run_id':self.run_id,'request_id':str(self.sequence),
                      'kind':'qr' if action['op']=='scan' else action['kind'],
                      'target':action['key'] if action['op']=='scan' else self.target(action),
                      'after':time.time(),
                      'station':self.route['stations'][self.route['route'][self.leg-1]['station']]['name']}
        self.state='vision'

    def task_code_tick(self, now):
        """At confirmed A arrival, lock three fresh equal codes before camera handoff.

Only current camera-two frames are accepted, never its last-result display.
The plan survives closing the QR process and stays fixed until the next run.
"""
        cameras = self.console.task_cameras
        transition = cameras.snapshot()
        if transition.get('token') != self.camera_token:
            self.fail('任务相机切换已取消。'); return
        if transition['state'] == 'error':
            self.fail(transition['error']); return
        if self.state == 'qr_return':
            self.qr_search.tick()
            if not self.pending and now-self.last_query >= .25:
                self.last_query = now
                self.send('NAV STAT', 'query')
            return
        if self.state == 'qr_switch':
            if transition['state'] == 'ready' and transition['slot'] == '1':
                if cameras.slots.hubs['1'].snapshot()['state'] != 'ready':
                    self.fail('摄像头1未就绪，保持在A点。'); return
                self.qr_status = '任务码已锁定；摄像头2已关闭，摄像头1已打开'
                if self.qr_search and self.qr_search.suspended:
                    self.qr_search.return_to_origin()
                else:
                    self.next_action()
        elif transition['state'] == 'ready':
            frame = cameras.slots.hubs['2'].snapshot()
            if frame['state'] == 'error':
                self.fail('摄像头2已断开，未取得有效任务码。'); return
            qr = frame.get('qr') or {}
            frame_id = qr.get('frame_id', 0)
            captured = qr.get('captured_at', 0)
            fresh = (frame['state'] == 'ready' and captured >= self.qr_after and
                     0 <= time.time()-captured <= self.config['vision_max_age_ms']/1000)
            if frame_id > self.qr_frame:
                self.qr_frame = frame_id
                if fresh and self.qr_search:
                    self.qr_search.sighting(False, captured)
                try:
                    if not fresh or captured <= self.qr_last_capture or qr.get('status') != 'valid':
                        raise ValueError('等待摄像头2识别有效四组任务码')
                    plan = parse_task_code(qr.get('code'))
                except ValueError as error:
                    self.qr_votes = 0
                    self.qr_status = str(error)
                else:
                    if self.qr_search and not self.qr_search.sighting(True, captured):
                        self.qr_votes = 0
                        self.qr_search.tick()
                        return
                    consecutive = plan['code'] == self.qr_candidate and captured-self.qr_last_capture <= 1.5
                    self.qr_votes = self.qr_votes+1 if consecutive else 1
                    self.qr_candidate = plan['code']
                    self.qr_last_capture = captured
                    self.qr_status = '任务码连续确认 {}/3：{}'.format(self.qr_votes, plan['code'])
                    if self.qr_votes >= 3:
                        self.task_plan = plan
                        self.scans['task'] = plan['code']
                        self.state = 'qr_switch'
                        self.qr_budget_at = None
                        self.stage_started = now
                        self.qr_status = '任务码已锁定，正在关闭摄像头2并打开摄像头1'
                        self.camera_token = cameras.request('1', timeout=None)
                        self.qr_budget_at = now
            elif time.time()-self.qr_last_capture > 1.5:
                # Polling the same cached report while the next frame decodes
                # must not erase a vote. A real interruption does break the run.
                self.qr_votes = 0
        if self.state == 'qr_wait' and transition['state'] == 'ready' and self.qr_search:
            self.qr_search.tick()
        if self.active and not self.pending and self.state in ('qr_wait', 'qr_switch') and now-self.last_query >= .25:
            self.last_query = now
            self.send('NAV STAT', 'query')

    def reply(self,body,reply,tag):
        """Consume only the in-flight operation; stale telemetry never completes it."""
        if not self.active or not self.pending or tag!=self.pending[0]:return
        _,_,purpose,_=self.pending;self.pending=None
        if not reply.ok:
            if (purpose == 'material_suspend' and body.startswith('NAV SUSPEND ') and
                    reply.fields.get('code') == 'NAV_HANDOFF_NOT_READY'):
                self.material.wait_for_handoff()
                return
            self.fail('任务命令被拒绝（{}）：{}'.format(body, reply.body));return
        f=reply.fields
        if purpose.startswith('material_'):
            try:self.material.reply(purpose, f)
            except (ValueError, KeyError, TypeError) as error:self.fail(str(error))
            return
        if purpose.startswith('qr_search_'):
            self.qr_search.reply(purpose, f); return
        if purpose=='ring_begin':
            self.state='ring_start';self.send('NAV STAT','query');return
        if purpose=='ring_pose_check':
            if (f.get('state')!='suspended' or f.get('ready')!='1' or
                    f.get('path_id')!=self.path_id or f.get('imu')!='1'):
                self.fail('视觉定位前的导航交接状态无效。');return
            try:self.console.camera_alignment.start(task_options=self.ring_options)
            except ValueError as error:self.fail(str(error));return
            self.state='ring_vision';return
        if purpose=='ring_return':
            self.ring_return_pose(f);return
        if purpose=='ring_return_accept':
            self.ring_move_id=f.get('move_id')
            if self.ring_move_id is None:self.fail('回位移动缺少运动编号。');return
            self.state='ring_return_move';self.last_query=0.;return
        if purpose=='ring_return_status':
            if f.get('move_id') != self.ring_move_id or f.get('fault','none') != 'none' or f.get('state') in ('stopped','fault'):
                self.fail('回位移动被中断或发生故障。');return
            if f.get('state')=='done' and f.get('active')=='0':
                if f.get('remaining')!='0':self.fail('回位移动未走完全部脉冲。');return
                self.last_query=0.;self.send('NAV STAT','ring_return')
            return
        if purpose=='ring_end':
            self.state='ring_replan';self.send('NAV STAT','ring_replan');return
        if purpose=='ring_replan':
            if f.get('ready')!='1' or f.get('state')!='stopped' or f.get('path_id')!=self.path_id:
                self.fail('视觉任务结束后的导航坐标无效。');return
            if self.leg == len(self.route['route']):
                if not self.console.navigator.finish_external(self.path_id):
                    self.fail('最终视觉站点的巡航控制权不匹配。');return
                self.state='done';return
            cfg=copy.deepcopy(self.route);cfg['route']=cfg['route'][self.leg:]
            # The board restarts numbering at 1; user-facing legs keep their
            # original numbers, including tasks at repeated stations.
            self.leg_base=self.leg
            new_id=int(self.path_id)%2147483647+1
            try:
                self.ring_upload=self.console.mission.commands((int(f['x']),int(f['y'])),new_id,config=cfg,host_control=True)
                self.ring_config=cfg
            except ValueError as error:self.fail('剩余路线规划失败：'+str(error));return
            self.state='ring_upload';return
        if purpose=='ring_upload':
            if body.startswith('NAV START '):
                self.path_id=body.split()[2];self.state='travel'
                self.console.navigator.start(self.ring_config,self.path_id)
                self.last_nav=self.leg_started=time.monotonic()
            return
        if purpose=='query':
            self.observe(f);return
        if purpose=='start':self.state='travel';return
        if purpose=='go':
            self.departed_leg=self.leg;self.state='departing';return
        if purpose=='fix':self.next_action();return
        if purpose=='nudge':
            self.op_id=f.get('op_id');self.state='nudge_wait';return
        if purpose=='aux':
            self.op_id=f.get('move_id');self.state='aux_wait';return
        if purpose=='aux_query':
            action=self.actions[self.index]
            if f.get('move_id')==self.op_id and f.get('active')=='0':
                if f.get('remaining')!='0':self.fail('辅助轴未到达目标。')
                else:self.send('AUX STAT {}'.format(action['axis']),'aux_position')
            return
        if purpose=='aux_position':
            if int(f['pos'])!=self.actions[self.index]['position'] or f.get('active')!='0':self.fail('辅助轴未到达目标。')
            else:self.next_action()
            return
        if purpose=='nudge_query':
            if f.get('op_id')==self.op_id and f.get('state')=='waiting':
                self.state='vision';self.begin_vision(self.actions[self.index])
            elif f.get('state') in ('fault','stopped','blocked'):self.fail('对准微调失败。')
            return
        if purpose=='servo':self.state='servo_wait';self.wait_until=time.monotonic()+self.actions[self.index]['settle_ms']/1000

    def observe(self,f):
        if not self.active or f.get('cmd')!='NAV_STAT':return
        if self.state=='uploading':return
        now=time.monotonic();self.last_nav=now
        if self.state.startswith(('ring_', 'material_')):
            if f.get('ready') != '1' or f.get('state') in ('fault','blocked'):
                self.fail('圆列任务期间导航失效。');return
            if f.get('state')=='stopped' and self.state not in ('ring_end','ring_replan','ring_upload'):
                self.fail('圆列任务期间巡航被停止。');return
            if self.state != 'ring_upload' and self.path_id is not None and f.get('path_id')!=self.path_id:
                self.fail('圆列任务期间路径编号改变。')
            return
        if f.get('state') in ('fault','stopped','blocked') or f.get('ready')!='1':
            self.fail('巡航中断：'+f.get('fault',f.get('state','unknown')));return
        path=f.get('path_id')
        if self.path_id is not None and path!=self.path_id:
            self.fail('巡航路径编号改变。');return
        if self.state=='travel' and self.path_id is None:self.path_id=path
        if f.get('state')=='done':
            if self.state not in ('travel','departing'):self.fail('站点任务期间巡航意外结束。')
            else:
                if self.material_mode:
                    station = self.route['stations'][self.route['route'][-1]['station']]
                    if station['name'].upper() == 'O':
                        self.console.task_cameras.release('1')
                        self.qr_status = '已回到O点，关闭摄像头1'
                self.state='done';self.request=None
            return
        leg=int(f.get('leg',0))+self.leg_base
        if self.state in ('travel','departing'):
            # A fast next leg may already be held at the first observed poll.
            # Its new leg number proves the acknowledged GO has departed.
            if self.state=='departing' and (leg!=self.departed_leg or f.get('held')!='1'):
                self.state='travel'
            if leg!=self.leg:
                self.leg=leg;self.leg_started=now
            # Periodic telemetry can see WAITING before the runner's post-ACK
            # query. Station actions must not steal ownership until that query
            # has confirmed this exact path/leg; otherwise SUSPEND overtakes it.
            runner = self.console.navigator
            if (f.get('held')=='1' and self.state!='departing' and
                    runner.state=='waiting' and runner.path_id==path and
                    runner.leg + 1 == int(f.get('leg', 0))):
                self.actions=self.config['legs'].get(str(leg),[]);self.index=0
                self.iterations=0;self.stage_started=now;self.state='actions'

    def tick(self):
        """Bounded work per pump pass; waits never sleep or open hardware."""
        if not self.active:return
        now=time.monotonic()
        # Searching has no elapsed-time limit. Preserve the remaining route
        # budgets so a late code can still hand off cameras and resume travel.
        if self.state in ('qr_wait', 'qr_switch') and self.qr_budget_at is not None:
            elapsed = now-self.qr_budget_at
            self.started += elapsed
            self.leg_started += elapsed
            self.qr_budget_at = now
        if self.state.startswith('material_') and self.material:
            self.material.preserve_alignment_budget(now)
        if now-self.started>self.config['timeout_s']:self.fail('整场任务超时。');return
        if not self.state.startswith(('ring_', 'material_')) and now-self.leg_started>self.config['leg_timeout_s']:self.fail('当前路线段超时。');return
        if self.state not in ('uploading','ring_upload') and now-self.last_nav>3:self.fail('导航状态超过 3 秒未更新。');return
        if self.pending:
            if now-self.pending[3]>2:self.fail('任务应答超时。')
            return
        if self.state.startswith('ring_'):
            self.ring_tick(now);return
        if self.state.startswith('material_'):
            try:self.material.tick(now)
            except (ValueError, KeyError, TypeError) as error:self.fail(str(error))
            return
        if self.state=='uploading':
            if not self.console.nav_pending:self.fail('路径上传未完成。')
            return
        if self.state in ('travel','departing'):
            # The runner's matching completion query establishes ownership;
            # query held stations immediately, without a second 250 ms gate.
            interval = .02 if self.console.navigator.state == 'waiting' else .25
            if now-self.last_query>=interval:self.last_query=now;self.send('NAV STAT','query')
            return
        action_timeout = 90 if self.state == 'qr_return' else self.config['action_timeout_s']
        if self.state not in ('qr_wait', 'qr_switch') and now-self.stage_started>action_timeout:
            self.fail('站点动作超时。');return
        if self.state in ('qr_wait', 'qr_switch', 'qr_return'):
            self.task_code_tick(now); return
        if self.state=='actions':
            if self.index>=len(self.actions):self.send('NAV GO {} {}'.format(self.path_id,self.leg-self.leg_base-1),'go');return
            a=self.actions[self.index];op=a['op']
            if op == 'task_code':
                self.qr_after = time.time()
                self.qr_frame = self.qr_votes = 0
                self.qr_candidate = None
                self.qr_last_capture = 0.
                self.qr_status = '已到达A点，等待连续三帧有效任务码'
                self.stage_started = now
                self.state = 'qr_wait'
                self.qr_budget_at = now
                self.qr_search = QRSearch(self)
            elif op=='ring_row':
                self.ring_options=self.ring_parameters(self.leg,a)
                self.stage_started=now;self.state='ring_begin'
                self.send('NAV SUSPEND {} {}'.format(self.path_id,self.leg-self.leg_base-1),'ring_begin')
            elif op == 'material':
                try:
                    self.material = MaterialStation(self, a)
                    self.material.start()
                except (ValueError, KeyError, TypeError) as error:self.fail(str(error))
            elif op in ('scan','align'):
                try:self.begin_vision(a)
                except ValueError as error:self.fail(str(error))
            elif op=='servo':
                from .servo_config import wire_channel, wire_angle
                self.send('SERVO {} {}'.format(wire_channel(self.console.device,a['channel']),wire_angle(self.console.device,a['channel'],a['angle'])),'servo')
            elif op=='aux':
                self.send('AUX GOTO {} {} {} {}'.format(a['axis'],a['position'],a['sps'],a['accel']),'aux')
            else:self.wait_until=now+a['ms']/1000;self.state='wait'
        elif self.state in ('servo_wait','wait'):
            if now>=self.wait_until:self.next_action()
        elif self.state=='aux_wait':
            if now-self.last_query>=.02:
                self.last_query=now;self.send('AXIS {}'.format(self.actions[self.index]['axis']),'aux_query')
        elif self.state=='nudge_wait':
            if now-self.last_query>=.02:self.last_query=now;self.send('NAV ALIGN','nudge_query')
        elif self.state=='vision' and self.sample:
            sample,self.sample=self.sample,None
            if time.time()-sample['captured_at']>self.config['vision_max_age_ms']/1000:self.stable=0;return
            if self.request['kind']=='qr':
                self.scans[self.actions[self.index]['key']]=sample['text'];self.next_action();return
            c=self.config['calibration'];dx,dy=sample['error_dx'],sample['error_dy']
            correction=alignment_step(dx,dy,c)
            if (sample['source']!=self.stable_source or
                    sample['captured_at']-self.stable_captured>self.config['vision_max_age_ms']/1000):
                self.stable=0
            self.stable_source=sample['source'];self.stable_captured=sample['captured_at']
            if correction['aligned']:
                self.stable+=1
                if self.stable>=c['stable_frames']:
                    self.request=None;self.send('NAV FIX {} {}'.format(self.path_id,self.leg-self.leg_base-1),'fix')
            else:
                self.stable=0;self.iterations+=1
                if self.iterations>c['max_iterations']:self.fail('视觉对准超过迭代上限。');return
                self.request=None
                self.send('NAV NUDGE {} {} {} {} {} {}'.format(self.path_id,self.leg-self.leg_base-1,correction['direction'],correction['steps'],correction['sps'],correction['accel']),'nudge')
