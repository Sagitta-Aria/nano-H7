"""NX decisions and compiled ARM generic execution, including real STEP edges.

Synthetic yaw/plant prove sequencing, not floor friction or physical clearance.
"""
import copy
import json
import random
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
from host.navigation import DEFAULT,MissionConfig,validated,arrival
from host.navigation_runner import turn_actions
from host.protocol import encode,decode
from tests import test_navigation as arm_fixture
from tests import test_mission as host_fixture
from tests.test_arm import TIM8


class InsetConfigTests(unittest.TestCase):
    def test_old_files_load_without_enabling_any_arrival_or_rewriting_parameters(self):
        current=copy.deepcopy(DEFAULT)
        for station in current['stations']:
            station.pop('arrival_enabled')
        current['stations'][2]['heading']=-90.25
        legacy=dict(points={s['name']:[s['x'],s['y']] for s in current['stations']},
                    headings={s['name']:s['heading'] for s in current['stations']},
                    sps=500,accel=2000,dwell_ms=3000,grid=current['grid'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'navigation.json'
            for original in (current,legacy):
                with self.subTest(schema='points' if 'points' in original else 'stations'):
                    path.write_text(json.dumps(original),encoding='utf-8')
                    before=path.read_bytes()
                    loaded=MissionConfig(path).snapshot()
                    self.assertTrue(all(not s['arrival_enabled'] for s in loaded['stations']))
                    self.assertEqual(loaded['stations'][2]['heading'],-90.25)
                    self.assertEqual(loaded['route'],current['route'])
                    self.assertEqual(path.read_bytes(),before)

    def test_explicit_zero_is_distinct_from_disabled_and_old_configs_preserve_angles(self):
        value=copy.deepcopy(DEFAULT)
        value['stations'][1].update(heading=0,arrival_enabled=False)
        value['route']=[dict(station=1,hold=0),dict(station=1,hold=0,arrival_enabled=True,heading=0)]
        self.assertEqual(arrival(validated(value),0),(False,0))
        self.assertEqual(arrival(validated(value),1),(True,0))
        for s in value['stations']:s.pop('arrival_enabled')
        self.assertFalse(validated(value)['stations'][1]['arrival_enabled'])
        self.assertEqual(validated(value)['stations'][1]['heading'],0)

    def test_invalid_switches_and_overrides_are_rejected(self):
        for key,value in [('arrival_enabled',1),('heading',True),('heading',180.001)]:
            config=copy.deepcopy(DEFAULT);config['route'][0][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):validated(config)
        config=copy.deepcopy(DEFAULT);config['stations'][1]['arrival_enabled']=1
        with self.assertRaises(ValueError):validated(config)

    def test_no_firmware_inset_or_autonomous_start_is_uploaded(self):
        with tempfile.TemporaryDirectory() as directory:
            config=MissionConfig(Path(directory)/'nav.json')
            value=config.snapshot();value['turn_inset_enabled']=True;config.save(value)
            commands=config.commands(path_id=19)
            self.assertEqual(commands[0],'NAV EXEC')
            self.assertEqual(commands[-1],'NAV START 19 HOST')
            self.assertFalse(any(c.startswith('NAV INSET') for c in commands))

    def test_planned_same_heading_is_only_correction_at_every_station(self):
        config=copy.deepcopy(DEFAULT);config['turn_inset_enabled']=True
        for name,point in [('A',(0,14000)),('B',(14000,0)),('custom',(28000,14000))]:
            with self.subTest(station=name):
                self.assertEqual(turn_actions(config,point,0,0,20),[(0,0,0,0,'correct')])
        self.assertEqual(turn_actions(config,(0,14000),18000,-18000,20)[0][-1],'correct')
        actions=turn_actions(config,(0,14000),0,9000,20)
        self.assertEqual([a[-1] for a in actions],['inward','turn','return','settle'])
        self.assertEqual(actions[0][1:3],(1000,14000))
        self.assertEqual(actions[2][1:3],(0,14000))
        self.assertEqual([a[-1] for a in turn_actions(config,(14000,14000),0,9000,20)],['turn'])


class GenericArmTests(unittest.TestCase):
    setUp=arm_fixture.ArmNavigationTests.setUp
    request=arm_fixture.ArmNavigationTests.request
    configure=arm_fixture.ArmNavigationTests.configure
    upload=arm_fixture.ArmNavigationTests.upload
    nav=arm_fixture.ArmNavigationTests.nav
    tick=arm_fixture.ArmNavigationTests.tick
    group_edge=arm_fixture.ArmNavigationTests.group_edge

    def prepare(self):
        self.route=[1];self.configure(initial=0);self.assertTrue(self.upload().ok)
        self.assertTrue(self.request('NAV START 42 HOST').ok)
        self.op=0

    def execute(self,kind,x=0,y=0,heading=0):
        self.op+=1
        result=self.request('NAV EXEC 42 {} {} {} {} {} 500 2000'.format(self.op,kind,x,y,heading))
        self.assertTrue(result.ok,result.body)
        return result

    def settle(self,heading=0):
        for _ in range(5):self.tick(self.now+100,heading)

    def edges(self,heading=0):
        """Schedule actual individual timer compares, respecting unequal rates."""
        for count in range(10000):
            if not self.arm.call('motion_chassis_active'):
                self.tick(self.now+1,heading)
                if not self.arm.call('motion_chassis_active'):return
            enabled=self.arm.read32(TIM8+0x0c)&0x1e
            counter=self.arm.read32(TIM8+0x24)
            offsets=[((self.arm.read32(TIM8+0x34+4*i)-counter)&65535,i) for i in range(4) if enabled&(2<<i)]
            if not offsets:self.fail('active motion without timer ownership')
            delta=min(v for v,i in offsets)
            flags=sum(2<<i for v,i in offsets if v==delta)
            self.arm.write32(TIM8+0x24,(counter+delta)&65535)
            self.arm.write32(TIM8+0x10,flags);self.arm.call('TIM8_CC_IRQHandler');self.arm.write32(TIM8+0x10,0)
            self.arm.call('motion_tick')
            if count%10==0:self.tick(self.now+1,heading)
        self.fail('finite vector never completed')

    def test_host_start_never_moves_and_no_automatic_next_action(self):
        self.prepare();self.settle()
        self.assertEqual(self.nav()['state'],'checkpoint')
        self.assertEqual(self.arm.call('motion_chassis_active'),0)
        self.assertFalse(self.request('NAV INSET 1000').ok)
        self.execute(1,0,14);self.edges()
        self.assertEqual((self.nav()['state'],self.nav()['x'],self.nav()['y']),('checkpoint','0','14'))
        self.settle();self.assertEqual(self.nav()['state'],'checkpoint')

    def test_shortest_wrap_and_return_drift_is_a_new_generic_correction(self):
        self.prepare();self.execute(0,heading=9000);self.tick(self.now+1,0)
        self.assertEqual([self.arm.status(i)[1]>0 for i in range(4)],[False]*4)
        self.arm.call('motion_chassis_stop');self.edges(9000);self.settle(9000)
        self.execute(1,4,0,9000);self.edges(9060)
        self.assertEqual(self.nav()['fault'],'none')
        self.execute(0,heading=9000);self.settle(9000)
        self.assertEqual(self.nav()['state'],'checkpoint')

    def test_replayed_operation_stop_and_stale_feedback_cannot_resume(self):
        self.prepare();self.execute(1,0,14)
        self.assertFalse(self.request('NAV EXEC 42 1 1 0 14 0 500 2000').ok)
        self.group_edge();before=[self.arm.status(i)[0] for i in range(4)]
        self.request('STOP');self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)],before)
        self.assertFalse(self.request('NAV EXEC 42 2 1 0 14 0 500 2000').ok)
        self.setUp();self.prepare();self.execute(1,0,14)
        self.tick(301,feedback=False)
        self.assertEqual(self.nav()['fault'],'imu_stale')
        self.tick(302);self.assertEqual(self.nav()['state'],'fault')

    def test_large_deviation_recovers_and_framing_survives_max_ids(self):
        self.prepare();self.execute(1,0,14);self.tick(1,600)
        self.assertEqual(self.nav()['fault'],'none')
        self.assertEqual(self.nav()['state'],'turning')
        self.group_edge(); self.settle(0); self.edges(0)
        self.assertEqual(self.nav()['state'],'checkpoint')
        self.assertEqual((self.nav()['x'],self.nav()['y']),('0','14'))
        self.request('NAV EXEC',sequence=4294967295)
        self.assertLessEqual(len(self.arm.transmitted),384)

    def test_diagonal_zero_wheels_receive_yaw_and_reverse_only_after_falling_edge(self):
        self.prepare();self.execute(1,1000,1000)
        self.tick(1,-100)
        for _ in range(100):self.arm.call('motion_tick')
        self.assertLess(self.arm.status(0)[1],0)
        self.arm.edge(0)
        before=self.arm.status(0)[0]
        self.tick(2,100);self.arm.call('motion_tick')
        self.assertEqual(self.arm.status(0)[0],before)
        self.assertLess(self.arm.status(0)[1],0)
        self.arm.edge(0)
        self.assertEqual(self.arm.status(0)[0],before)
        self.arm.call('motion_tick')
        self.assertGreater(self.arm.status(0)[1],0)

    def test_arbitrary_heading_vector_and_explicit_arrival(self):
        self.prepare();self.execute(0,heading=4500);self.settle(4500)
        self.execute(1,3,7,4500);self.edges(4500)
        self.assertEqual((self.nav()['state'],self.nav()['x'],self.nav()['y']),('checkpoint','3','7'))
        self.assertFalse(self.request('NAV EXEC 42 9 2 0 0 4500 500 2000').ok)


class HostRouteTests(unittest.TestCase):
    setUp=host_fixture.MissionTests.setUp
    send=host_fixture.MissionTests.send
    flush=host_fixture.MissionTests.flush
    step=host_fixture.MissionTests.step

    def start_route(self,config):
        self.console.save_navigation(config)
        ok,message=self.console.navigation_action('start');self.assertTrue(ok,message);self.flush()

    def finish_route(self,perturb=False,release=False):
        phases=[];changed=set()
        for _ in range(10000):
            runner=self.console.navigator
            phases.append((runner.leg,runner.phase))
            if perturb and runner.phase=='return' and runner.state=='executing' and runner.op not in changed:
                changed.add(runner.op);self.board.navigation.absolute_heading+=.6
            if release and self.console.navigation.get('held')=='1':
                self.console.navigation_action('go');self.flush()
            self.step()
            if runner.state=='fault':self.fail(runner.error)
            if runner.state=='done':return phases
        self.fail('NX route did not finish')

    def test_no_arrival_turn_or_inset_at_any_disabled_station(self):
        config=self.console.mission.snapshot()
        config['route']=[dict(station=i,hold=0) for i in (1,2,3,4,1,0)]
        config['turn_inset_enabled']=True;config['turn_inset_steps']=4
        self.start_route(config);self.finish_route(perturb=True)
        # Travel turns may inset; immediately before each ARRIVE, yaw is the
        # last incoming lane heading, never the station's unused zero angle.
        commands=[b.split() for b in self.link.sent if b.startswith('NAV EXEC ') and len(b.split())==10]
        arrivals=[c for c in commands if c[4]=='2']
        self.assertEqual([int(c[7]) for c in arrivals],[0,18000,0,9000,-9000,18000])
        first=next(i for i,c in enumerate(commands) if c[4]=='2')
        self.assertEqual([c[4] for c in commands[:first]],['0','1'])
        second=next(i for i,c in enumerate(commands[first+1:], first+1) if c[4]=='2')
        # A departure still returns from its inset. The next travel reaches the
        # center, turns there without another excursion, then travels to B.
        leg=commands[first+1:second]
        center=next(i for i,c in enumerate(leg) if c[4:7]==['1','14','14'])
        self.assertEqual([(c[4],c[5],c[6],c[7]) for c in leg[center:]],
                         [('1','14','14','9000'),('0','0','0','18000'),('1','14','0','18000')])

    def test_fractional_heading_return_with_jittered_simulation_clock(self):
        """Browser/serial ticks are uneven; wheel batches must not create drift."""
        config=self.console.mission.snapshot()
        config['stations'][2].update(heading=-90.25,arrival_enabled=True)
        config.update(route=[dict(station=i,hold=0) for i in (1,2,3,4)]+
                      [dict(station=1,hold=0,arrival_enabled=True,heading=0)],
                      dwell_ms=0,turn_inset_enabled=True,turn_inset_steps=4)
        self.start_route(config)
        schedule=random.Random(10)
        for _ in range(4000):
            self.step(schedule.uniform(.02,.08))
            runner=self.console.navigator
            self.assertNotIn(runner.state,('fault','stopped'),
                             (runner.snapshot(),self.board.navigation.fault))
            if runner.state=='done':break
        self.assertEqual(runner.state,'done')
        self.assertEqual((self.console.navigation['x'],self.console.navigation['y']),('0','14'))

    def test_repeated_station_modes_zero_target_and_return_drift(self):
        config=self.console.mission.snapshot();config['dwell_ms']=0
        config['turn_inset_enabled']=True;config['turn_inset_steps']=4
        config['stations'][1].update(heading=90,arrival_enabled=False)
        config['route']=[dict(station=1,hold=0),dict(station=1,hold=0,arrival_enabled=True,heading=0),
                         dict(station=1,hold=0,arrival_enabled=True,heading=90)]
        self.start_route(config);phases=self.finish_route(perturb=True)
        self.assertFalse(any(leg==0 and phase=='inward' for leg,phase in phases))
        self.assertFalse(any(leg==1 and phase=='inward' for leg,phase in phases))
        self.assertTrue(any(leg==2 and phase=='inward' for leg,phase in phases))
        self.assertEqual(self.console.navigation['fault'],'none')

    def test_disabled_arrival_reports_incoming_heading_instead_of_unused_station_angle(self):
        config=self.console.mission.snapshot()
        config['stations'][1].update(heading=90,arrival_enabled=False)
        config['route']=[dict(station=1,hold=1)]
        self.start_route(config)
        for _ in range(500):
            self.step()
            if self.console.navigator.state=='waiting':break
        else:self.fail('first arrival was not reached')
        fields=self.send('NAV STAT').fields
        self.assertLessEqual(abs(int(fields['heading_cd'])),20)
        self.assertEqual(fields['arrival_cd'],'0')

    def test_long_station_wait_does_not_consume_the_next_action_deadline(self):
        for held,dwell in ((1,0),(0,60000)):
            with self.subTest(held=held,dwell=dwell):
                self.setUp()
                config=self.console.mission.snapshot()
                config['route']=[dict(station=1,hold=held),dict(station=2,hold=0)]
                config['dwell_ms']=dwell
                self.start_route(config)
                for _ in range(500):
                    self.step()
                    if self.console.navigator.state=='waiting':break
                else:self.fail('station wait was not reached')
                if held:
                    for _ in range(1300):self.step()
                    self.assertTrue(self.console.navigation_action('go')[0]);self.flush()
                self.finish_route()

    def test_stop_invalidates_late_ack_and_rejects_manual_motion(self):
        config=self.console.mission.snapshot();config['route']=[dict(station=1,hold=0)]
        self.start_route(config);self.step()
        self.assertFalse(self.console.submit('WHEELS 100 100 100 100 1000')[0])
        pending=self.console.navigator.pending
        self.console.submit('STOP');self.flush()
        if pending:
            tag,body,_,_=pending;self.console._reply(body,self.send(body),tag)
        for _ in range(10):self.step()
        self.assertFalse(self.console.navigator.active)
        self.assertFalse(self.board.armed)

    def test_old_firmware_is_rejected_before_any_config_or_motion(self):
        config=self.console.mission.snapshot();config['route']=[dict(station=1,hold=0)]
        self.console.save_navigation(config)
        self.console.navigation_action('start')
        body,tag=self.link.commands.get_nowait();self.console._reply(body,self.send(body),tag)
        body,tag=self.link.commands.get_nowait();self.assertEqual(body,'NAV EXEC')
        self.console._reply(body,decode(encode(1,'ERR code=BAD_ARGUMENT')),tag)
        self.assertFalse(self.console.nav_pending)
        self.assertFalse(any(b.startswith('NAV START') for b in self.link.sent))


if __name__=='__main__':unittest.main()
