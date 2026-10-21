"""Competition mapping, real mission arrival barriers and camera handoff races."""
import copy
import threading
import time
import unittest
from types import SimpleNamespace

from host.task_code import parse_task_code
from host.task_camera import TaskCameras
from host.mission import DEFAULT
from tests import test_mission as mission_fixture


class TaskCodeTests(unittest.TestCase):
    def test_both_batches_keep_pickup_color_and_destination_paired_on_retrieval(self):
        plan = parse_task_code('156+231+516+123')
        first, second = plan['batches']
        self.assertEqual([(i['angle_deg'], i['color_name'], i['target']) for i in first['items']],
                         [(0, '浅蓝', 1), (135, '黑', 3), (270, '红', 2)])
        self.assertEqual([(i['color'], i['target']) for i in second['items']],
                         [('light_blue', 3), ('red', 2), ('black', 1)])
        self.assertEqual([(i['angle_deg'], i['color_id']) for i in first['pickup_items']],
                         [(270, 1), (135, 5), (0, 6)])
        self.assertEqual([(i['color_id'], i['target']) for i in first['pickup_items']],
                         [(1, 2), (5, 3), (6, 1)])
        self.assertEqual([(i['color_id'], i['target']) for i in second['pickup_items']],
                         [(5, 1), (1, 2), (6, 3)])

    def test_color_table_and_canonical_spaces(self):
        plan = parse_task_code(' 123 + 321 + 456 + 213 ')
        self.assertEqual(plan['code'], '123+321+456+213')
        self.assertEqual([i['color'] for b in plan['batches'] for i in b['items']],
                         ['blue', 'yellow', 'red', 'light_blue', 'black', 'green'])

    def test_rejects_legacy_short_zero_unknown_and_duplicate_destinations(self):
        for text in (None, 123, '123321', '123+321', '156+231+516', '156+231+516+123+456',
                     '056+231+516+123', '176+231+516+123', '156+221+516+123',
                     '156+231+516+124', '１５６+231+516+123', '156+231+516+12'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_task_code(text)


class FakeSlots:
    """A controllable camera resource model; no CV, serial or physical devices."""
    def __init__(self):
        self.events = []
        self.states = {s:dict(state='idle') for s in ('1', '2')}
        self.hubs = {s:SimpleNamespace(snapshot=lambda s=s:copy.deepcopy(self.states[s])) for s in self.states}
        self.bindings = {'1':{'id':'cam-one'}, '2':{'id':'cam-two'}}
        self.block = None
        self.entered = threading.Event()

    def stop(self, slot):
        self.events.append(('stop', slot))
        self.states[slot] = dict(state='idle')

    def listing(self, slot):
        binding = self.bindings.get(slot)
        device = dict(id='video'+slot, resolutions=['1280x720'])
        return dict(binding=binding, bound_device=device['id'] if binding else None,
                    devices=[device] if binding else [])

    def start(self, slot, device, resolution):
        self.entered.set()
        if self.block: self.block.wait(3)
        self.events.append(('start', slot))
        self.states[slot] = dict(state='ready', device=device)


class CameraSwitchTests(unittest.TestCase):
    def setUp(self):
        self.slots = FakeSlots()
        self.cameras = TaskCameras(self.slots)
        self.addCleanup(self.cameras.close)

    def until(self, predicate):
        deadline = time.monotonic()+4
        while not predicate() and time.monotonic()<deadline: time.sleep(.01)
        self.assertTrue(predicate(), self.cameras.snapshot())

    def test_close_two_before_open_one_and_reverse_for_new_run(self):
        self.slots.states['2'] = dict(state='ready', device='video2')
        self.cameras.request('1')
        self.until(lambda:self.cameras.snapshot()['state']=='ready')
        self.assertEqual(self.slots.events, [('stop','2'), ('start','1')])
        self.cameras.request('2')
        self.until(lambda:self.cameras.snapshot()['state']=='ready')
        self.assertEqual(self.slots.events[-2:], [('stop','1'), ('start','2')])

    def test_missing_binding_never_claims_another_camera(self):
        del self.slots.bindings['1']
        self.cameras.request('1')
        self.until(lambda:self.cameras.snapshot()['state']=='error')
        self.assertEqual(self.slots.events, [('stop','2')])
        self.assertIn('固定ID', self.cameras.snapshot()['error'])

    def test_route_release_closes_one_without_opening_two(self):
        self.cameras.request('1', timeout=None)
        self.until(lambda:self.cameras.snapshot()['state']=='ready')
        before = len(self.slots.events)
        token = self.cameras.release('1')
        self.until(lambda:self.cameras.snapshot()['state']=='idle')
        self.assertEqual(self.cameras.snapshot()['token'], token)
        self.assertEqual(self.slots.events[before:], [('stop','1')])
        self.assertEqual(self.slots.states['1']['state'], 'idle')

    def test_cancel_during_driver_open_closes_late_camera(self):
        self.slots.block = threading.Event()
        self.cameras.request('1')
        self.assertTrue(self.slots.entered.wait(2))
        self.cameras.cancel()
        self.slots.block.set()
        self.until(lambda:('stop','1') in self.slots.events)
        self.assertEqual(self.cameras.snapshot()['state'], 'idle')
        self.assertEqual(self.slots.states['1']['state'], 'idle')

    def test_unbounded_opening_survives_elapsed_time_and_can_be_canceled(self):
        from unittest.mock import patch
        elapsed = [0.]
        observed = threading.Event()
        def opening(slot, device, resolution):
            self.slots.states[slot] = dict(state='opening', device=device)
        def snapshot():
            elapsed[0] += 100.
            if elapsed[0] > 1000: observed.set()
            return dict(self.slots.states['1'])
        self.slots.start = opening
        self.slots.hubs['1'].snapshot = snapshot
        with patch('host.task_camera.time', SimpleNamespace(monotonic=lambda:elapsed[0])):
            self.cameras.request('1', timeout=None)
            self.assertTrue(observed.wait(2))
            self.assertEqual(self.cameras.snapshot()['state'], 'opening')
            self.cameras.cancel()
            self.until(lambda:('stop','1') in self.slots.events)
        self.assertEqual(self.cameras.snapshot()['state'], 'idle')


class FakeCameraControl:
    """Immediate QR startup; camera-one completion remains test-controlled."""
    def __init__(self):
        self.slots = FakeSlots()
        self.requests = []
        self.timeouts = []
        self.token = 0
        self.result = dict(state='idle')

    def request(self, slot, *, timeout=10):
        self.requests.append(slot)
        self.timeouts.append(timeout)
        self.token += 1
        self.result = dict(token=self.token, slot=slot, state='ready' if slot=='2' else 'opening', error='')
        if slot == '1': self.slots.stop('2')
        else: self.slots.states['2'] = dict(state='ready', device='video2')
        return self.token

    def snapshot(self): return dict(self.result)

    def release(self, slot):
        self.slots.stop(slot)
        self.token += 1
        self.result = dict(state='idle', slot=slot, token=self.token)
        return self.token

    def cancel(self):
        self.token += 1
        self.result = dict(state='idle')

    def finish(self):
        self.slots.states['1'] = dict(state='ready', device='video1')
        self.result['state'] = 'ready'


class TaskCodeMissionTests(unittest.TestCase):
    setUp = mission_fixture.MissionTests.setUp
    send = mission_fixture.MissionTests.send
    flush = mission_fixture.MissionTests.flush
    step = mission_fixture.MissionTests.step
    until = mission_fixture.MissionTests.until

    def configure(self):
        self.control = FakeCameraControl()
        self.console.task_cameras = self.control
        route = self.console.mission.snapshot()
        route['route'] = [dict(station=1, hold=1, task=4), dict(station=2, hold=0)]
        self.console.save_navigation(route)
        self.console.save_task(copy.deepcopy(DEFAULT))
        ok, why = self.console.navigation_action('start')
        self.assertTrue(ok, why)

    def arrive(self):
        self.configure()
        self.until(lambda:self.console.task.state=='qr_wait')
        self.assertEqual(self.console.navigation['held'], '1')

    def frame(self, number, code='156+231+516+123', **fields):
        report = dict(frame_id=number, status='valid', code=code,
                      captured_at=1700000000+self.clock[0], last_code=code)
        report.update(fields)
        self.control.slots.states['2']['qr'] = report
        self.step()
        self.step()

    def test_arrival_then_three_frames_then_open_ready_before_departing(self):
        self.configure()
        self.frame(1)
        self.assertIsNone(self.console.task.task_plan)
        self.until(lambda:self.console.task.state=='qr_wait')
        self.assertEqual(self.control.requests, ['2'])
        self.frame(2); self.frame(3)
        self.assertIsNone(self.console.task.task_plan)
        self.frame(4)
        self.assertEqual(self.console.task.state, 'qr_switch')
        self.assertEqual(self.control.requests, ['2', '1'])
        locked = self.console.task.snapshot()['task_plan']
        self.assertEqual(locked['batches'][0]['items'][0]['target'], 1)
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))
        for _ in range(20): self.step()
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))
        self.control.finish()
        self.until(lambda:self.console.task.state=='done')
        self.assertEqual(self.console.task.task_plan, locked)
        self.assertTrue(any(c.startswith('NAV GO ') for c in self.link.sent))

    def test_old_duplicate_invalid_and_changed_codes_cannot_lock(self):
        self.arrive()
        self.frame(1, captured_at=self.console.task.qr_after-1)
        self.frame(2)
        for _ in range(3): self.frame(2)
        self.assertIsNone(self.console.task.task_plan)
        self.frame(3, '056+231+516+123')
        self.frame(4)
        self.frame(5, '123+321+456+123')
        self.frame(6)
        self.assertIsNone(self.console.task.task_plan)
        self.frame(7); self.frame(8)
        self.assertEqual(self.console.task.task_plan['code'], '156+231+516+123')

    def test_open_failure_keeps_plan_but_never_releases_A(self):
        self.arrive()
        for i in range(1,4): self.frame(i)
        self.control.result.update(state='error', error='摄像头1未连接')
        self.step(); self.step()
        self.assertEqual(self.console.task.state, 'fault')
        self.assertIsNotNone(self.console.task.task_plan)
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))
        self.assertFalse(self.board.armed)

    def test_camera_one_wait_exceeds_all_mission_budgets_until_stop(self):
        self.arrive()
        for i in range(1,4): self.frame(i)
        task = self.console.task
        self.assertEqual(task.state, 'qr_switch')
        elapsed = max(task.config[k] for k in ('timeout_s','leg_timeout_s','action_timeout_s'))+100
        for _ in range(int(elapsed/.05)+2): self.step()
        self.assertEqual(task.state, 'qr_switch', task.error)
        self.assertEqual(self.control.timeouts[-1], None)
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))
        self.console.task_action('stop'); self.flush()
        self.control.finish()
        for _ in range(10): self.step()
        self.assertEqual(task.state, 'stopped')
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))

    def test_decoding_next_frame_does_not_erase_previous_vote(self):
        self.arrive()
        # At 720p the next decode may arrive after the cached report ages past
        # 500 ms. Each newly delivered frame is fresh, so the votes still count.
        for i in range(1,4):
            self.frame(i)
            if i < 3:
                for _ in range(10): self.step()
        self.assertEqual(self.console.task.state, 'qr_switch')

    def test_stop_retires_handoff_and_next_run_requires_new_frames(self):
        self.arrive()
        for i in range(1,4): self.frame(i)
        self.console.task_action('stop'); self.flush()
        self.control.finish()
        for _ in range(20): self.step()
        self.assertEqual(self.console.task.state, 'stopped')
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))
        self.console.submit('ARM 1'); self.flush()
        ok, why = self.console.navigation_action('start')
        self.assertTrue(ok, why)
        self.assertIsNone(self.console.task.task_plan)
        self.assertEqual(self.control.requests[-1], '2')

    def test_missing_camera_frames_fault_without_departure(self):
        self.arrive()
        for _ in range(700):
            self.step()
            if self.console.task.state == 'fault': break
        self.assertEqual(self.console.task.state, 'fault')
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))


class QRSearchMissionTests(unittest.TestCase):
    setUp = mission_fixture.MissionTests.setUp
    send = mission_fixture.MissionTests.send
    flush = mission_fixture.MissionTests.flush
    step = mission_fixture.MissionTests.step
    until = mission_fixture.MissionTests.until
    configure = TaskCodeMissionTests.configure
    arrive = TaskCodeMissionTests.arrive

    def camera_step(self, code=None):
        """Feed current raw-camera metadata while the simulated board moves."""
        report = dict(frame_id=getattr(self, 'frame_id', 0)+1,
                      status='valid' if code else 'searching', code=code,
                      captured_at=1700000000+self.clock[0])
        self.frame_id = report['frame_id']
        self.control.slots.states['2']['qr'] = report
        self.step()

    def search_until(self, predicate, code=None, limit=2000):
        for _ in range(limit):
            if predicate(): return
            if self.console.task.state == 'fault': self.fail(self.console.task.error)
            self.camera_step(code)
        self.fail(str(self.console.task.snapshot()))

    def test_repeated_sweeps_exact_extent_speed_step_and_no_release_without_code(self):
        self.arrive()
        positions = []
        for _ in range(4000):
            positions.append(self.console.task.qr_search.position)
            self.camera_step()
        self.assertEqual((min(positions), max(positions)), (-3200, 3200))
        self.assertEqual(self.console.task.state, 'qr_wait')
        self.assertEqual(self.console.task.error, '')
        commands = [c.split() for c in self.link.sent if c.startswith('CHASSIS STRAIGHT')]
        self.assertGreater(len(commands), 32)
        self.assertTrue(all(c[3]=='800' and c[4]=='2500' for c in commands))
        cycle = ['BACK']*4 + ['FWD']*8 + ['BACK']*4
        self.assertEqual([c[2] for c in commands[:32]], cycle*2)
        self.assertFalse(any(c.startswith('NAV GO ') for c in self.link.sent))
        self.assertEqual(self.control.requests, ['2'])

    def test_search_beyond_all_elapsed_limits_then_code_can_finish_mission(self):
        self.arrive()
        task = self.console.task
        began = self.clock[0]
        total_used = began-task.started
        leg_used = began-task.leg_started
        duration = max(90, task.config['timeout_s'], task.config['leg_timeout_s'],
                       task.config['action_timeout_s'])+10
        self.search_until(lambda:self.clock[0]-began >= duration,
                          limit=int(duration/.05)+2)
        self.assertEqual(task.state, 'qr_wait')
        self.assertAlmostEqual(self.clock[0]-task.started, total_used)
        self.assertAlmostEqual(self.clock[0]-task.leg_started, leg_used)
        self.assertEqual(self.control.requests, ['2'])
        self.search_until(lambda:task.state=='qr_switch', '156+231+516+123')
        self.control.finish()
        self.until(lambda:task.state=='done')
        self.assertEqual(task.qr_search.position, 0)
        self.assertEqual(task.leg, 2)
        self.assertEqual(task.task_plan['code'], '156+231+516+123')

    def test_valid_code_at_offset_returns_physically_then_replans_from_A(self):
        self.arrive()
        q = self.console.task.qr_search
        self.search_until(lambda:q.position == -800 and q.phase == 'watch')
        self.search_until(lambda:self.console.task.state=='qr_switch', '156+231+516+123')
        self.assertEqual(self.control.requests, ['2', '1'])
        self.assertEqual(q.position, -800)
        self.control.finish()
        self.until(lambda:self.console.task.state=='done')
        self.assertEqual(q.position, 0)
        commands = self.link.sent
        back = next(i for i,c in enumerate(commands) if c.startswith('CHASSIS STRAIGHT BACK'))
        forward = next(i for i,c in enumerate(commands) if c.startswith('CHASSIS STRAIGHT FWD'))
        restore = next(i for i,c in enumerate(commands) if c.startswith('NAV RESTORE'))
        self.assertLess(back, forward); self.assertLess(forward, restore)
        self.assertEqual(commands[restore].split()[-2:], ['0', '14'])
        self.assertEqual(self.console.task.leg, 2)
        self.assertEqual(self.console.task.task_plan['code'], '156+231+516+123')

    def test_moving_sighting_halts_partial_step_and_requires_new_stopped_frames(self):
        self.arrive()
        q = self.console.task.qr_search
        self.search_until(lambda:q.phase=='moving')
        self.search_until(lambda:q.phase=='watch' and q.position<0, '156+231+516+123')
        self.assertIn('CHASSIS HALT', self.link.sent)
        self.assertTrue(-800 < q.position < 0, q.position)
        self.assertIsNone(self.console.task.task_plan)
        stopped = q.position
        self.search_until(lambda:self.console.task.state=='qr_switch', '156+231+516+123')
        self.control.finish()
        self.until(lambda:self.console.task.state=='done')
        self.assertIn('CHASSIS STRAIGHT FWD {} 2500 {}'.format(-stopped,self.console.task.route['accel']),self.link.sent)

    def test_stop_during_search_never_restarts_or_switches_camera(self):
        self.arrive()
        # STOP still wins after a complete unsuccessful sweep has repeated.
        self.search_until(lambda:sum(c.startswith('CHASSIS STRAIGHT') for c in self.link.sent)>16)
        self.search_until(lambda:self.console.task.qr_search.phase=='moving')
        self.console.task_action('stop'); self.flush()
        count=len(self.link.sent)
        for _ in range(80): self.camera_step('156+231+516+123')
        self.assertEqual(self.console.task.state, 'stopped')
        self.assertEqual(self.control.requests, ['2'])
        self.assertFalse(any(c.startswith(('CHASSIS STRAIGHT','NAV RESTORE','NAV START')) for c in self.link.sent[count:]))

    def test_other_move_identity_cannot_complete_search_step(self):
        self.arrive()
        self.search_until(lambda:self.console.task.qr_search.phase=='moving')
        self.board.chassis_id += 1
        for _ in range(20):
            self.camera_step()
            if self.console.task.state == 'fault': break
        self.assertEqual(self.console.task.state, 'fault')
        self.assertIn('编号', self.console.task.error)
        self.assertEqual(self.control.requests, ['2'])

    def test_camera_without_frames_never_starts_blind_sweep(self):
        self.arrive()
        for _ in range(100): self.step()
        self.assertEqual(self.console.task.state, 'fault')
        self.assertFalse(any(c.startswith('CHASSIS STRAIGHT') for c in self.link.sent))

    def test_return_pose_mismatch_cannot_be_written_as_A(self):
        self.arrive()
        q=self.console.task.qr_search
        q.reply('qr_search_origin', dict(state='suspended',ready='1',imu='1',
            path_id=self.console.task.path_id,x='200',y='14'))
        self.assertEqual(self.console.task.state, 'fault')
        self.assertFalse(any(c.startswith('NAV RESTORE') for c in self.link.sent))


if __name__ == '__main__': unittest.main()
