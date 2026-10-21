"""Task orchestration through the real protocol, using a deterministic demo clock."""
import copy
import queue
import tempfile
import json
import threading
import time
import urllib.request
import urllib.error
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

from host.mission import DEFAULT, validated
from host.mission_vision import sample_for, VisionPublisher
from host.navigation import DEFAULT as NAV_DEFAULT
from host.protocol import encode, decode
from host.simulator import SimulatedBoard
from host.webapp import Console
from host.webserver import Handler, Session


class Link:
    """Queued protocol endpoint; replies are delivered only by the test pump."""
    def __init__(self):
        self.commands = queue.Queue()
        self.sent = []

    def submit(self, body, tag=None):
        self.sent.append(body)
        if body == 'STOP':
            while not self.commands.empty():self.commands.get_nowait()
        self.commands.put((body,tag))
        return True

    def set_jog(self, body):
        self.sent.append(body)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.clock = [10.]
        self.clockpatch = patch('time.monotonic', side_effect=lambda:self.clock[0])
        self.wallpatch = patch('time.time', side_effect=lambda:1700000000+self.clock[0])
        self.clockpatch.start();self.wallpatch.start()
        self.addCleanup(self.clockpatch.stop);self.addCleanup(self.wallpatch.stop)
        self.console = Console(navigation_path=Path(self.temp.name)/'navigation.json')
        self.console.connection = self.link = Link()
        self.board = SimulatedBoard(clock=lambda:self.clock[0])
        self.seq = 0
        self.console.device['_port']='SIM'
        self.console._reply('HELLO',self.send('HELLO'))
        route=copy.deepcopy(NAV_DEFAULT)
        for s in route['stations']:
            s['x']//=1000;s['y']//=1000;s['heading']=0
        route['grid']={k:[v//1000 for v in values] for k,values in route['grid'].items()}
        route['route']=[{'station':s,'hold':1} for s in [1,2,3,4,2,3,4,1,0]]
        route['dwell_ms']=0
        self.console.save_navigation(route)
        self.console.submit('NAV RESET 0 0');self.console.submit('ARM 1');self.flush()
        self.console.nav_align=self.send('NAV ALIGN').fields
        self.console.last_client=0 # No browser alive during the mission.

    def send(self,body):
        self.seq+=1
        if body!='PING':
            self.board.receive(encode(self.seq+1000000,'PING'))
        return decode(self.board.receive(encode(self.seq,body)))

    def flush(self):
        for _ in range(500):
            if self.link.commands.empty():return
            body,tag=self.link.commands.get_nowait()
            self.console._reply(body,self.send(body),tag)
        self.fail('unexpected unbounded command queue')

    def step(self,dt=.05):
        self.clock[0]+=dt;self.send('PING')
        self.console.tick();self.flush()

    def until(self,predicate,limit=6000):
        for _ in range(limit):
            if predicate():return
            if self.console.task.state=='fault':self.fail(self.console.task.error)
            self.step()
        self.fail('mission timed out: '+str(self.console.task.snapshot()))

    def plan(self,actions=None):
        config=copy.deepcopy(DEFAULT)
        config['legs']={str(i):[] for i in range(1,10)}
        if actions:config['legs'].update(actions)
        config['calibration'].update(verified=True,matrix=[[1,0],[0,1]])
        self.console.save_task(config)
        ok,why=self.console.task_action('start');self.assertTrue(ok,why)

    def sample(self,frame,dx=0,dy=0,**extra):
        value=dict(self.console.task.request)
        value.update(source='test-camera',frame_id=frame,captured_at=1700000000+self.clock[0],
                     found=True,error_dx=dx,error_dy=dy)
        value.update(extra)
        self.console.task_vision(value)
        return value

    def test_full_repeated_route_scan_align_nudge_fix_actions_and_go(self):
        self.plan({'1':[{'op':'scan','key':'task'},
                        {'op':'align','kind':'block','target':{'scan':'task','index':0,'map':{'1':'red'}}},
                        {'op':'servo','channel':1,'angle':120,'settle_ms':100},
                        {'op':'wait','ms':100}],
                   '2':[{'op':'align','kind':'ring','target':'3'}]})
        self.until(lambda:self.console.task.state=='vision')
        self.sample(1,text='123+321+456+654');self.step()
        self.until(lambda:self.console.task.request and self.console.task.request['kind']=='block')
        self.sample(2,dx=7);self.step()
        self.assertTrue(any(b.startswith('NAV NUDGE ') for b in self.link.sent))
        self.until(lambda:self.console.task.state=='vision')
        displaced=[a['pos'] for a in self.board.axes[:4]]
        for frame in range(3,6):self.sample(frame);self.step()
        self.assertTrue(any(b.startswith('NAV FIX ') for b in self.link.sent))
        self.assertEqual(displaced,[a['pos'] for a in self.board.axes[:4]],'FIX must never retrace wheels')
        self.until(lambda:self.console.task.request and self.console.task.request['kind']=='ring')
        self.assertEqual(self.console.task.leg,2)
        for frame in range(6,9):self.sample(frame);self.step()
        self.until(lambda:self.console.task.state=='done')
        self.assertEqual(self.console.task.scans['task'],'123+321+456+654')
        self.assertEqual(sum(b.startswith('NAV GO ') for b in self.link.sent),9)
        self.assertIn('SERVO 2 120',self.link.sent)
        self.assertEqual(self.console.navigation['station'],'0')

    def test_stale_wrong_duplicate_missing_and_changed_source_frames_cannot_fix(self):
        self.plan({'1':[{'op':'align','kind':'block','target':'red'}]})
        self.until(lambda:self.console.task.state=='vision')
        for fields in ({'target':'blue'}, {'captured_at':1}, {'captured_at':float('nan')},
                       {'error_dx':float('nan')}, {'request_id':'previous'}):
            with self.assertRaises(ValueError):self.sample(1,**fields)
        self.sample(1);self.step()
        with self.assertRaises(ValueError):self.sample(1)
        self.sample(2,found=False);self.step()
        self.sample(3);self.step()
        self.sample(4,source='another-camera');self.step()
        self.assertFalse(any(b.startswith('NAV FIX ') for b in self.link.sent))
        for _ in range(12):self.step()
        self.sample(5,source='another-camera');self.step()
        self.assertFalse(any(b.startswith('NAV FIX ') for b in self.link.sent))

    def test_stop_invalidates_delayed_commands_and_rejects_manual_control(self):
        self.plan({'1':[{'op':'scan','key':'task'}]})
        self.until(lambda:self.console.task.state=='vision')
        for b in ('CHASSIS MOVE FWD 10 100 1000','SERVO 1 0','NAV FIX 2 0','ARM 1'):
            self.assertFalse(self.console.submit(b)[0],b)
        self.assertFalse(self.console.jog('CHASSIS JOG FWD 100 1000')[0])
        self.assertFalse(self.console.apply_auxiliary({},5)[0])
        self.assertFalse(self.console.navigation_action('go')[0])
        self.assertFalse(self.console.turn_action('start')[0])
        with self.assertRaises(ValueError):self.console.save_navigation(self.console.mission.snapshot())
        old=dict(self.console.task.request)
        self.assertTrue(self.console.submit('NAV STOP')[0]);self.flush()
        self.assertEqual(self.console.task.state,'stopped')
        with self.assertRaises(ValueError):self.console.task_vision(old)
        count=len(self.link.sent)
        for _ in range(20):self.step()
        self.assertFalse(any(b.startswith(('NAV GO ','NAV NUDGE ')) or
                             (b.startswith('SERVO ') and b!='SERVO STAT') for b in self.link.sent[count:]))

    def test_camera_restart_accepts_zero_resets_stability_and_retires_old_session(self):
        self.plan({'1':[{'op':'align','kind':'block','target':'red'}]})
        self.until(lambda:self.console.task.state=='vision')
        self.sample(50,session_id='before');self.step()
        self.sample(51,session_id='before');self.step()
        self.sample(0,session_id='after');self.step()
        self.assertEqual(self.console.task.stable,1)
        with self.assertRaisesRegex(ValueError,'会话'):
            self.sample(52,session_id='before')
        with self.assertRaises(ValueError):self.sample(0,session_id='after')
        self.sample(1,session_id='after');self.step()
        self.assertFalse(any(b.startswith('NAV FIX ') for b in self.link.sent))
        self.sample(2,session_id='after');self.step()
        self.assertTrue(any(b.startswith('NAV FIX ') for b in self.link.sent))

    def test_motor_fault_cancels_pending_work_and_reports_reason(self):
        self.plan({'1':[{'op':'scan','key':'task'}]})
        self.until(lambda:self.console.task.state=='vision')
        self.board.inject_motion_fault('step_deadline')
        self.console._reply('MOTION STAT',self.send('MOTION STAT'))
        self.assertFalse(self.console.task.active)
        self.assertIn('step_deadline',self.console.task.error)
        self.assertIsNone(self.console.task.request)
        self.assertFalse(self.console.nav_pending)
        self.assertIn('STOP',self.link.sent)
        self.flush()
        self.assertTrue(self.console.submit('MOTION CLEAR')[0]);self.flush()
        self.assertEqual(self.console.motion['latched'],'0')
        self.assertFalse(self.board.armed)
        self.assertFalse(self.board.navigation.ready)
        self.assertFalse(self.console.task_action('start')[0])
        self.assertTrue(self.console.navigation_action('origin')[0]);self.flush()
        self.console.submit('ARM 1');self.flush()
        self.plan()
        self.until(lambda:self.console.task.state=='done')

    def test_action_timeout_and_stale_navigation_stop_globally(self):
        self.plan({'1':[{'op':'scan','key':'task'}]})
        self.until(lambda:self.console.task.state=='vision')
        self.console.task.stage_started-=31
        self.console.tick();self.flush()
        self.assertEqual(self.console.task.state,'fault')
        self.assertEqual(self.link.sent[-1],'STOP')
        self.assertFalse(self.board.armed)

    def test_old_firmware_and_unconfigured_held_legs_do_not_upload(self):
        self.console.nav_align={}
        self.assertFalse(self.console.task_action('start')[0])
        self.console.nav_align={'v':'2'}
        self.assertFalse(self.console.task_action('start')[0])
        self.assertFalse(self.console.nav_pending)

    def test_alignment_requires_calibrated_vehicle_reference(self):
        config=copy.deepcopy(DEFAULT)
        config['legs']={str(i):[] for i in range(1,10)}
        config['legs']['1']=[{'op':'align','kind':'block','target':'red'}]
        self.console.save_task(config)
        self.assertFalse(self.console.task_action('start')[0])

    def test_auxiliary_completion_uses_operation_id_and_logical_position(self):
        self.console.submit('STOP');self.flush()
        for command in ('LIFT INIT','LIFT DIR -1','AUX CONFIG 5 0 100',
                        'AUX ZERO 6','AUX CONFIG 6 -100 100','ARM 1'):
            self.assertTrue(self.send(command).ok,command)
        self.console._reply('STAT',self.send('STAT'))
        self.plan({'1':[{'op':'aux','axis':5,'position':20,'sps':100,'accel':1000},
                        {'op':'aux','axis':6,'position':-20,'sps':100,'accel':1000}]})
        self.until(lambda:self.console.task.state=='done')
        self.assertEqual(self.send('AUX STAT 5').fields['pos'],'20')
        self.assertEqual(self.send('AUX STAT 6').fields['pos'],'-20')

    def test_delayed_ack_is_ignored_after_stop(self):
        self.plan({'1':[{'op':'align','kind':'block','target':'red'}]})
        self.until(lambda:self.console.task.state=='vision')
        self.sample(1,dx=10)
        self.console.task.tick()
        old=self.console.task.pending
        self.assertTrue(old)
        self.console.submit('STOP');self.flush()
        self.console._reply(old[1],decode(encode(999,'OK cmd=NAV_NUDGE op_id=999')),old[0])
        self.assertEqual(self.console.task.state,'stopped')
        self.assertIsNone(self.console.task.pending)

    def test_missing_ack_and_missing_navigation_fault_without_resume(self):
        self.plan({'1':[{'op':'scan','key':'task'}]})
        self.until(lambda:self.console.task.state=='vision')
        self.console.task.last_nav-=4
        self.console.task.tick();self.flush()
        self.assertEqual(self.console.task.state,'fault')
        self.assertFalse(self.board.armed)


class VisionContractTests(unittest.TestCase):
    def test_adapter_does_not_relabel_wrong_color_digit_or_mode(self):
        r={'run_id':'run','request_id':'1','kind':'block','target':'red'}
        report={'mode':'color','color_block':{'color':'blue','error_dx':0,'error_dy':0}}
        self.assertFalse(sample_for(r,report,'camera',1,1)['found'])
        report['color_block']['color']='red'
        self.assertTrue(sample_for(r,report,'camera',1,1)['found'])
        r.update(kind='ring',target='3')
        for report in ({'mode':'digit','ring':True,'digit':'2'},
                       {'mode':'color','ring':True,'digit':'3'}):
            self.assertFalse(sample_for(r,report,'camera',1,1)['found'])

    def test_bad_config_types_fail_as_validation_errors(self):
        for action in ({'op':[]},{'op':None},{'op':'wait','ms':True}):
            c=copy.deepcopy(DEFAULT);c['legs']={'1':[action]}
            with self.assertRaises(ValueError):validated(c)
        c=copy.deepcopy(DEFAULT);c['calibration']['verified']=True
        with self.assertRaises(ValueError):validated(c)


class MissionHttpTests(unittest.TestCase):
    """Real loopback HTTP input without any serial/camera handles."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        root=Path(self.temp.name)
        self.console=Console(root/'nav.json',root/'aux.json',root/'flow.json')
        self.session=Session(self.console)
        self.session.join('owner');self.session.join('viewer')
        handler=type('TestHandler',(Handler,),{'session':self.session})
        self.http=ThreadingHTTPServer(('127.0.0.1',0),handler);self.http.daemon_threads=True
        self.url='http://127.0.0.1:{}/'.format(self.http.server_address[1])
        self.thread=threading.Thread(target=self.http.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.close)

    def close(self):
        self.http.shutdown();self.http.server_close();self.thread.join(timeout=1)

    def call(self,path,payload,token=None):
        request=urllib.request.Request(self.url+path,json.dumps(payload).encode(),{'Content-Type':'application/json'})
        if token:request.add_header('X-Session',token)
        try:
            with urllib.request.urlopen(request,timeout=2) as response:return response.status,json.load(response)
        except urllib.error.HTTPError as error:return error.code,json.load(error)

    def test_role_guards_and_local_vision_publisher(self):
        for path,data in (('api/mission/action',{'action':'start'}),
                          ('api/mission/config',{'config':DEFAULT})):
            status,_=self.call(path,data,'viewer');self.assertEqual(status,409)
        status,body=self.call('api/mission/vision',{'sample':{}},None)
        self.assertEqual(status,400);self.assertFalse(body['ok'])
        route=copy.deepcopy(NAV_DEFAULT);route['route'][0]['hold']=1
        config=copy.deepcopy(DEFAULT);config['legs']={'1':[{'op':'scan','key':'task'}]}
        with self.console.lock:
            self.console.task.start(config,route)
            self.console.task.leg=1
            self.console.task.begin_vision(config['legs']['1'][0])
        publisher=VisionPublisher(self.url);self.addCleanup(publisher.close)
        deadline=time.monotonic()+3
        while not publisher.request() and time.monotonic()<deadline:time.sleep(.02)
        request=publisher.request();self.assertTrue(request)
        sample=sample_for(request,{'qr_text':'123+321'},'network-test',50,time.time())
        publisher.publish(sample)
        while self.console.task.sample is None and time.monotonic()<deadline:time.sleep(.02)
        self.assertEqual(self.console.task.sample['text'],'123+321')
        before=self.console.task.sample['session_id']
        publisher.close()
        restarted=VisionPublisher(self.url);self.addCleanup(restarted.close)
        deadline=time.monotonic()+3
        while not restarted.request() and time.monotonic()<deadline:time.sleep(.02)
        restarted.publish(sample_for(restarted.request(),{'qr_text':'456+654'},'network-test',0,time.time()))
        while self.console.task.sample['session_id']==before and time.monotonic()<deadline:time.sleep(.02)
        self.assertNotEqual(self.console.task.sample['session_id'],before)
        self.assertEqual(self.console.task.sample['frame_id'],0)
        self.assertEqual(self.console.task.sample['text'],'456+654')
        self.assertIsNone(self.console.connection,'mailbox must not open a serial connection')


if __name__=='__main__':unittest.main()
