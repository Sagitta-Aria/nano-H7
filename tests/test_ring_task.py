"""Held navigation -> real camera state machine -> exit -> replanned cruise.

Only image capture and the wheel plant are synthetic; protocol ACKs, exclusive
ownership, digit ordering, idle barriers and route upload all use real code.
"""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from host.mission import DEFAULT
from host.navigation import DEFAULT as NAV_DEFAULT, validated
from tests import test_mission as mission_fixture
from tests.test_camera_servo import VelocityLink


class RingTaskTests(unittest.TestCase):
    setUp = mission_fixture.MissionTests.setUp
    send = mission_fixture.MissionTests.send
    flush = mission_fixture.MissionTests.flush
    step = mission_fixture.MissionTests.step
    until = mission_fixture.MissionTests.until

    def configure(self, right=800, outward=400):
        self.link = VelocityLink(); self.console.connection = self.link
        route = copy.deepcopy(NAV_DEFAULT)
        for station in route['stations']:
            station['x'] //= 7; station['y'] //= 7; station['arrival_enabled'] = True
        route['grid'] = {k:[v//7 for v in values] for k,values in route['grid'].items()}
        route['stations'][2].update(x=2000,y=2000)
        route['route'] = [dict(station=1,hold=1,task=1,outward_steps=outward,right_steps=right),dict(station=2,hold=0)]
        route.update(sps=5000,dwell_ms=0)
        self.console.save_navigation(route)
        self.a = self.console.camera_alignment
        self.a.hub = SimpleNamespace(snapshot=lambda:{'state':'ready'},start_tracking=Mock(),stop_tracking=Mock())
        self.a.settings.value['ring_sequence'].update(range_steps=200,scan_step=100,sps=1000)
        def capture(image=None):
            self.a.capturing = True
        self.a.capture = capture
        self.console.save_task(copy.deepcopy(DEFAULT))

    def start(self):
        ok,why = self.console.navigation_action('start')
        self.assertTrue(ok,why); self.flush()
        self.until(lambda:self.console.task.state=='ring_vision')

    def frame(self, found=True):
        seq=self.a.ring_sequence
        if self.a.active and self.a.capturing:
            self.clock[0]=max(self.clock[0],seq.after+.01)
            report=dict(kind='ring',target='ring',frame_id=self.a.last_frame_id+1,
                        captured_monotonic=self.clock[0],found=found,
                        center=[320,240],control_point=[320,240],reference=[320,240],
                        digit=str(seq.expected) if found else None,error_dx=0,error_dy=0,
                        row_line={'error_deg':0,'points':[[100,200],[500,200]]} if found else None)
            self.a.results.put((self.a.generation,report,None,None))
        self.step()
        self.assertNotEqual(self.console.task.state,'fault',self.console.task.error)

    def finish(self, limit=1400):
        for _ in range(limit):
            if self.console.task.state=='done':return
            self.frame()
        self.fail(self.console.task.snapshot())

    def test_full_order_returns_to_two_exits_and_shortens_next_leg(self):
        self.configure(); self.start()
        self.assertEqual(self.console.navigation['state'],'suspended')
        self.assertFalse(self.console.submit('CHASSIS MOVE FWD 1 100 1000')[0])
        self.finish()
        self.assertEqual([r['digit'] for r in self.a.ring_sequence.recognized],[1,2,3,2])
        # 右移脉冲不再是退出方式：任务结束后按实测里程回到到站点，再声明坐标。
        self.assertFalse(any(b.startswith('CHASSIS STRAIGHT RIGHT 800') for b in self.link.sent))
        self.assertTrue(any(b.startswith('CHASSIS STRAIGHT ') for b in self.link.sent))
        begins=[b for b in self.link.sent if b.startswith('NAV BEGIN ')]
        self.assertEqual(begins[-1].split()[-2:],['0','2000'])
        # 回到到站点后重新规划的路线从 (0,2000) 出发，最终停在下一个站点。
        self.assertEqual(self.console.navigation['x'],'2000')
        self.assertEqual(self.console.navigation['y'],'2000')
        self.assertEqual(self.console.task.leg,2)
        self.assertTrue(self.board.armed)

    def test_failed_initial_sweep_moves_outward_once_then_retries(self):
        self.configure(outward=450);self.start()
        for _ in range(500):
            self.frame(False)
            if self.a.ring_sequence.outward_used and self.a.ring_sequence.phase=='search':break
        else:self.fail('no outward retry')
        self.assertIn('CHASSIS STRAIGHT LEFT 450 1000 6000',self.link.sent)
        self.finish()
        self.assertEqual(sum(b.startswith('CHASSIS STRAIGHT LEFT 450') for b in self.link.sent),1)
        # 搜索期间的位移由板端积分记录，任务结束后要被回位动作抵消，
        # 所以重新规划的路线仍然从到站点出发。
        uploads=[b for b in self.link.sent if b.startswith('NAV BEGIN ')]
        self.assertEqual(uploads[-1].split()[-2:],['0','2000'])

    def test_stop_during_camera_cancels_both_owners_and_no_restart(self):
        self.configure();self.start();self.frame()
        self.assertTrue(self.console.task_action('stop')[0]);self.flush()
        self.assertFalse(self.a.active);self.assertFalse(self.console.task.active)
        before=len(self.link.sent)
        for _ in range(40):self.step()
        self.assertFalse(self.board.armed)
        self.assertFalse(any(b.startswith(('WHEELS ','CHASSIS STRAIGHT','NAV START','NAV SUSPEND')) for b in self.link.sent[before:]))

    def test_second_task_keeps_original_leg_numbers_after_reupload(self):
        self.configure()
        route=self.console.mission.snapshot()
        route['stations'][2]['heading']=180
        route['route'][1].update(hold=1,task=1,outward_steps=300,right_steps=600)
        route['route'].append(dict(station=0,hold=0))
        self.console.save_navigation(route)
        self.start();self.finish(limit=2400)
        begins=[b for b in self.link.sent if b.startswith('NAV SUSPEND ')]
        self.assertEqual(begins,['NAV SUSPEND 2 0','NAV SUSPEND 3 0'])
        uploads=[b for b in self.link.sent if b.startswith('NAV BEGIN ')]
        self.assertEqual(uploads[-1].split()[-2:],['2000','2000'])
        self.assertEqual(self.console.task.leg,3)

    def test_moving_search_halts_before_alignment_without_losing_work(self):
        self.configure();self.start()
        for _ in range(5):self.frame(False)
        for _ in range(8):self.frame()
        self.assertEqual(self.console.navigation['state'],'suspended')
        self.finish()

    def test_lost_host_owner_cannot_reuse_suspended_navigation(self):
        self.configure();self.start()
        self.console.task.stop(send=False)
        self.assertEqual(self.console.navigation['state'],'suspended')
        with self.assertRaises(ValueError):self.a.start()
        self.assertFalse(self.console.navigation_action('start')[0])

    def test_old_firmware_without_suspend_fails_before_upload(self):
        self.configure();self.console.nav_align.pop('suspend')
        before=len(self.link.sent)
        ok,why=self.console.navigation_action('start')
        self.assertFalse(ok);self.assertIn('NAV SUSPEND',why)
        self.assertEqual(len(self.link.sent),before)

    def test_exit_operation_id_change_stops_before_remaining_route(self):
        self.configure();self.start()
        for _ in range(800):
            self.frame()
            if self.console.task.state=='ring_return_move':break
        else:self.fail('return move not reached')
        self.board.chassis_id+=1
        for _ in range(10):
            self.step()
            if self.console.task.state=='fault':break
        self.assertEqual(self.console.task.state,'fault')
        self.assertFalse(self.board.armed)
        self.assertEqual(sum(b.startswith('NAV START ') for b in self.link.sent),1)

    def test_visual_turn_does_not_reset_map_yaw_or_start_wrong_exit(self):
        self.configure();self.start()
        initial=self.board.navigation.initial_heading
        # A real visual turn changes the absolute sensor heading while the
        # initial map heading remains fixed. Synthetic images still align.
        self.board.navigation.absolute_heading=initial+10
        for _ in range(900):
            if self.console.task.state=='fault':break
            seq=self.a.ring_sequence
            if self.a.active and self.a.capturing:
                try:self.frame()
                except AssertionError:
                    if self.console.task.state!='fault':raise
            else:self.step()
        self.assertEqual(self.console.task.state,'fault')
        # 视觉平行与板端航向差超过 ±6°，按航向偏置上限拒绝，而不是继续用错误基准。
        self.assertIn('航向偏差',self.console.task.error)
        self.assertEqual(self.board.navigation.initial_heading,initial)
        self.assertEqual(self.console.navigator.bias_cd,0)
        self.assertFalse(any(b.startswith('CHASSIS STRAIGHT RIGHT 800') for b in self.link.sent))

    def test_exit_odometry_mismatch_cannot_restore_or_resume_route(self):
        self.configure();self.start()
        for _ in range(800):
            self.frame()
            if self.console.task.state=='ring_return_move':break
        self.board.chassis_id+=1
        for _ in range(200):
            if self.console.task.state=='fault':break
            self.step()
        self.assertEqual(self.console.task.state,'fault')
        self.assertIn('回位移动',self.console.task.error)
        self.assertFalse(any(b.startswith('NAV RESTORE ') for b in self.link.sent))

    def test_vision_heading_bias_is_adopted_for_later_turn_targets(self):
        self.configure(); self.start()
        # 板端航向账本整体偏 3°，视觉平行仍然成立，于是偏置应被采纳而不是报错。
        self.board.navigation.absolute_heading += 3.
        self.finish()
        self.assertEqual(self.console.task.state, 'done')
        self.assertEqual(self.console.navigator.bias_cd, 300)
        # NAV EXEC <path> <op> <kind> <x> <y> <heading> <sps> <accel>
        headings = [int(b.split()[7]) for b in self.link.sent if b.startswith('NAV EXEC ')]
        self.assertIn(9300, headings, headings)   # 配置的 90° + 3° 偏置

    def test_new_origin_clears_the_vision_heading_bias(self):
        self.configure(); self.start()
        self.board.navigation.absolute_heading += 3.
        self.finish()
        self.assertEqual(self.console.navigator.bias_cd, 300)
        self.console.submit('ARM 0'); self.flush()
        ok, why = self.console.navigation_action('origin')
        self.assertTrue(ok, why); self.flush()
        # 重建坐标系会重新采集 yaw 原点，旧偏置必须作废。
        self.assertEqual(self.console.navigator.bias_cd, 0)

    def test_slots_and_parameter_validation(self):
        self.configure()
        config=self.console.mission.snapshot()
        self.assertEqual(validated(config)['route'][0]['right_steps'],800)
        for key,value in [('task',5),('outward_steps',True),('right_steps',-1)]:
            bad=copy.deepcopy(config);bad['route'][0][key]=value
            with self.assertRaises(ValueError):validated(bad)
        config['route'][0]['task']=2;self.console.save_navigation(config)
        ok,why=self.console.navigation_action('start')
        self.assertFalse(ok);self.assertIn('尚未配置',why)


if __name__=='__main__':unittest.main()
