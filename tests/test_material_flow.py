"""Two complete batches through real executors/protocol; only images/plant are fake."""
import copy
import unittest
import itertools
from types import SimpleNamespace
from unittest.mock import Mock, patch

from host.material_flow import build_plan, OFFSETS
from tests import test_mission as fixture
from tests.test_camera_servo import VelocityLink
from tests.test_task_code import FakeCameraControl
from host.auxiliary import AuxiliaryConfig
from pathlib import Path


class MaterialFlowTests(unittest.TestCase):
    setUp = fixture.MissionTests.setUp
    send = fixture.MissionTests.send
    flush = fixture.MissionTests.flush
    step = fixture.MissionTests.step
    until = fixture.MissionTests.until

    def configure(self, code='156+231+516+312'):
        self.console.aux_settings = AuxiliaryConfig(Path(self.temp.name)/'auxiliary.json')
        self.code = code
        self.link = VelocityLink()
        self.console.connection = self.link
        self.control = FakeCameraControl()
        self.console.task_cameras = self.control
        self.a = self.console.camera_alignment
        self.a.hub = SimpleNamespace(snapshot=lambda: self.control.slots.states['1'],
            start_tracking=Mock(), stop_tracking=Mock())
        self.a.capture = lambda image=None: setattr(self.a, 'capturing', True)
        self.a.settings.value['ring_sequence'].update(sps=2000)
        self.a.settings.value['tracking']['camera_yaw_deg'] = 0
        cfg = self.console.pick_place.settings.snapshot()
        cfg.update(pick_down=40, place_take_down=50, place_drop_down=60, d_second_drop_down=25,
                   pick_rotation=-30, place_rotation=-30, lift_sps=3000,
                   rotation_sps=3000, s1_settle_ms=100, s2_settle_ms=100)
        self.console.pick_place.save(cfg)
        self.console.b_pickup.save(dict(self.console.b_pickup.snapshot(),
            **{key: value for key, value in cfg.items() if key in self.console.b_pickup.snapshot()}))
        ranges = self.console.aux_settings.snapshot()
        ranges['5']['max'] = 10000
        ranges['6'].update(min=-2000, max=0, presets=[0,-30])
        self.console.aux_settings.save(ranges)
        for body in ('STOP', 'LIFT INIT', 'LIFT DIR 1', 'AUX ZERO 6',
                     'AUX CONFIG 5 0 10000', 'AUX CONFIG 6 -2000 0'):
            self.console._reply(body, self.send(body))
        self.flush()
        self.saved_route = self.console.mission.snapshot()
        self.saved_config = self.console.task_settings.snapshot()
        self.jobs, self.visits, self.cameras, self.moves, self.depths = [], [], [], [], []
        self.last_job = None
        self.qr_number = 0
        ok, why = self.console.competition.start(full_flow=True)
        self.assertTrue(ok, why)

    def frame(self, missing=False, dx=0, qr=True):
        task = self.console.task
        if task.state == 'qr_wait' and qr:
            self.qr_number += 1
            self.control.slots.states['2']['qr'] = dict(status='valid', code=self.code,
                frame_id=self.qr_number, captured_at=1700000000+self.clock[0])
        if self.control.result.get('slot') == '1' and self.control.result['state'] == 'opening':
            self.cameras.append(task.leg)
            self.control.finish()
        if self.a.active and self.a.capturing:
            seq = self.a.ring_sequence
            if seq:
                self.clock[0] = max(self.clock[0], seq.after+.01)
            report = dict(kind='ring' if seq else 'block',
                target='ring' if seq else self.a.config['target'],
                frame_id=self.a.last_frame_id+1, captured_monotonic=self.clock[0],
                captured_at=1700000000+self.clock[0],
                found=not missing, center=[320+dx,240], control_point=[320+dx,240],
                reference=[320,240], error_dx=dx, error_dy=0,
                width=640, height=480, roi=[224,168,192,144],
                digit='1', row_line=dict(error_deg=0, points=[[100,200],[500,200]]))
            if seq and 'stack_layout' in self.a.config:
                # One uniquely colored ring at a time after pair-based yaw alignment.
                layout = task.d_layout
                point = next(p for p, item in sorted(layout.items())
                    if sum(other['color'] == item['color'] for other in layout.values()) == 1)
                if seq.confirming_middle:
                    point = 2
                report.update(digit=None, anchor_point=point, identity_source='remembered_color',
                              rings=[[320+dx,240]])
            self.a.results.put((self.a.generation, report, None, None))
        material = task.material
        if task.state.startswith('material_') and material:
            visit = (material.batch, material.station)
            if not self.visits or self.visits[-1] != visit:
                self.visits.append(visit)
            job = self.console.pick_place
            if job.active and job.job_id != self.last_job:
                self.last_job = job.job_id
                self.jobs.append((material.batch, material.station, job.context['mode'],
                    job.context['slot'], job.context['color_id'], job.context['target']))
                self.depths.append((material.batch, material.station, job.context['mode'],
                                    job.config['place_drop_down']))
        self.step()
        self.assertNotEqual(task.state, 'fault', task.error)
        self.assertNotEqual(self.console.competition.state, 'fault', self.console.competition.error)

    def finish(self, limit=20000):
        for _ in range(limit):
            if self.console.competition.state == 'done':
                return
            self.frame()
        self.fail(self.console.task.snapshot())

    def test_two_batches_all_jobs_and_correct_recollection_slots(self):
        self.configure()
        self.finish()
        self.assertEqual(self.visits, [(1,'B'),(1,'C'),(1,'D'),(2,'B'),(2,'C'),(2,'D')])
        self.assertEqual(self.cameras, [1], 'camera1 opens at A and stays open across both B visits')
        self.assertEqual(len(self.jobs), 24)
        self.assertEqual([(j[2],j[3],j[5]) for j in self.jobs if j[:2] == (1,'C')],
                         [('place',1,1),('place',2,3),('place',3,2),
                          ('pick',1,1),('pick',3,2),('pick',2,3)])
        for batch in (1,2):
            c_places = [(j[3:]) for j in self.jobs if j[:3] == (batch,'C','place')]
            d_places = [(j[3:]) for j in self.jobs if j[:3] == (batch,'D','place')]
            self.assertEqual(c_places, d_places)
            self.assertFalse(any(j[:3] == (batch,'D','pick') for j in self.jobs))
        self.assertFalse(self.console.task.tray)
        self.assertEqual((self.console.navigation['x'],self.console.navigation['y']), ('0','0'))
        self.assertEqual(self.console.mission.snapshot(), self.saved_route)
        self.assertEqual(self.console.task_settings.snapshot(), self.saved_config)
        moves = [b.split()[2:4] for b in self.link.sent if b.startswith('CHASSIS STRAIGHT')]
        self.assertIn(['FWD','2150'], moves)
        self.assertIn(['FWD','2080'], moves)
        self.assertIn(['BACK','4230'], moves)
        self.assertNotIn(['FWD','4300'], moves)
        expected_moves = []
        for batch_index, batch in enumerate(self.console.task.task_plan['batches'], 1):
            targets = [i['target'] for i in batch['items']]
            for visits in (targets+[1,2,3,2], targets+[2]):
                second_d = batch_index == 2 and visits == targets+[2]
                if second_d:
                    # Fixture first sees point 1, then confirms middle after transfer.
                    expected_moves.append(['FWD', str(OFFSETS[2])])
                position = OFFSETS[2] if second_d else 0
                for target in visits:
                    delta = OFFSETS[target]-position
                    if delta:
                        expected_moves.append(['FWD' if delta > 0 else 'BACK', str(abs(delta))])
                    position = OFFSETS[target]
        self.assertEqual(moves, expected_moves)
        self.assertTrue(any(b == 'AUX GOTO 5 60 3000 1000' for b in self.link.sent))
        self.assertEqual(self.link.sent.count('AUX ZERO 6'), 1)
        self.assertEqual(self.board.lift_down_sign, -1)
        self.assertEqual(len([d for d in self.depths if d == (2,'D','place',25)]), 3)
        self.assertTrue(all(d[3] == (25 if d[:3] == (2,'D','place') else 60) for d in self.depths))
        first = self.console.task.task_plan['batches'][0]['items']
        self.assertEqual(self.console.task.d_layout, {item['target']:item for item in first})

    def test_unconfigured_stacking_rejects_start_before_any_commands(self):
        self.configure()
        self.console.competition.stop()
        self.console.pick_place.settings.value['d_second_drop_down'] = -1
        before = len(self.link.sent)
        ok, why = self.console.competition.start(full_flow=True)
        self.assertFalse(ok)
        self.assertIn('第二次D点', why)
        self.assertEqual(len(self.link.sent), before)

    def test_first_layer_memory_is_committed_only_after_success_and_reset_next_run(self):
        self.configure()
        for _ in range(5000):
            self.frame()
            material = self.console.task.material
            if material and material.station == 'D' and material.phase == 'cycle':break
        else:self.fail('no first D job')
        self.assertEqual(self.console.task.d_layout, {})
        first = copy.deepcopy(material.current)
        for _ in range(1000):
            self.frame()
            if self.console.task.d_layout:break
        self.assertEqual(self.console.task.d_layout, {first['target']:first})
        self.console.task_action('stop')
        self.flush()
        self.configure()
        self.until(lambda:self.console.task.active)
        self.assertEqual(self.console.task.d_layout, {})

    def test_second_d_rejects_incomplete_layout_or_reversed_arrival(self):
        from host.material_flow import MaterialStation
        self.configure()
        for _ in range(14000):
            self.frame()
            material = self.console.task.material
            if material and material.stacked and material.phase == 'align':break
        else:self.fail('no second D')
        task = self.console.task
        action = dict(station='D', batch=2, right_steps=0, outward_steps=0)
        saved = task.d_layout.pop(1)
        with self.assertRaisesRegex(ValueError, '完整颜色布局'):
            MaterialStation(task, action)
        task.d_layout[1] = saved
        task.d_heading += 180
        with self.assertRaisesRegex(ValueError, '朝向须一致'):
            MaterialStation(task, action)

    def test_stop_during_cycle_cancels_all_owners_and_pending_moves(self):
        self.configure()
        for _ in range(1500):
            self.frame()
            if self.console.task.state == 'material_cycle' and self.console.pick_place.index >= 2:
                break
        else:self.fail('no mechanism cycle')
        self.console.task_action('stop')
        self.flush()
        before = len(self.link.sent)
        for _ in range(50):self.step()
        self.assertFalse(self.console.task.active)
        self.assertFalse(self.console.pick_place.active)
        self.assertFalse(self.console.camera_alignment.active)
        self.assertFalse(self.board.armed)
        self.assertFalse(any(b.startswith(('AUX GOTO','SERVO ','CHASSIS STRAIGHT','NAV START'))
                             and b != 'SERVO STAT' for b in self.link.sent[before:]))

    def test_saved_distances_drive_both_rounds_and_remain_fixed_during_run(self):
        self.console.material_settings.save(dict(point2_steps=1800, point3_steps=3900))
        self.configure()
        with self.assertRaises(ValueError):
            self.console.material_settings.save(dict(point2_steps=2200, point3_steps=4400))
        for _ in range(1000):
            self.frame()
            if self.console.task.active:break
        self.assertTrue(self.console.task.active)
        self.assertEqual(self.console.task.material_offsets, {1:0, 2:1800, 3:3900})
        # Even accidental in-memory changes cannot alter the pinned run geometry.
        self.console.material_settings.value.update(point2_steps=2200, point3_steps=4400)
        self.finish()
        moves = [b.split()[2:4] for b in self.link.sent if b.startswith('CHASSIS STRAIGHT')]
        self.assertEqual(moves, [[direction, str(steps)] for direction, steps in [
            # First C: place at 1/3/2, collect at 1/2/3, finish at 2.
            ('FWD',3900), ('BACK',2100), ('BACK',1800),
            ('FWD',1800), ('FWD',2100), ('BACK',2100),
            # First D: place at 1/3/2, already at the final reference.
            ('FWD',3900), ('BACK',2100),
            # Second C: place at 2/1/3, collect at 1/2/3, finish at 2.
            ('FWD',1800), ('BACK',1800), ('FWD',3900), ('BACK',3900),
            ('FWD',1800), ('FWD',2100), ('BACK',2100),
            # Second D: identify 1, calibrate 2, place at 2/1/3, finish at 2.
            ('FWD',1800), ('BACK',1800), ('FWD',3900), ('BACK',2100)]])
        self.assertEqual(len(self.jobs), 24)

    def test_replaced_row_movement_cannot_start_next_mechanism(self):
        self.configure()
        for _ in range(3000):
            self.frame()
            m = self.console.task.material
            if self.console.task.state == 'material_move' and m.station == 'C' and m.move_id:
                break
        else:self.fail('no row move')
        before = len(self.jobs)
        self.board.chassis_id += 1
        for _ in range(20):
            self.step()
            if self.console.task.state == 'fault':break
        self.assertEqual(self.console.task.state, 'fault')
        self.assertFalse(self.board.armed)
        self.assertEqual(len(self.jobs), before)

    def test_plan_preserves_geometry_and_rejects_missing_stations(self):
        route = self.console.mission.snapshot()
        config = self.console.task_settings.snapshot()
        result, actions = build_plan(route, config)
        self.assertEqual(result['stations'], route['stations'])
        self.assertEqual([result['stations'][s['station']]['name'] for s in result['route']],
                         list('ABCDBCDO'))
        self.assertEqual(OFFSETS[3]-OFFSETS[2], 2080)
        route['stations'][2]['name'] = 'X'
        with self.assertRaises(ValueError):build_plan(route, config)

    def test_every_placement_permutation_and_repeated_colors(self):
        for targets in itertools.permutations('123'):
            with self.subTest(targets=targets):
                placement = ''.join(targets)
                self.configure('116+'+placement+'+445+'+placement[::-1])
                self.finish()
                for batch in (1,2):
                    c = [j[3:] for j in self.jobs if j[:3] == (batch,'C','place')]
                    d = [j[3:] for j in self.jobs if j[:3] == (batch,'D','place')]
                    self.assertEqual(c, d)
                    pickup = '116' if batch == 1 else '445'
                    destinations = placement if batch == 1 else placement[::-1]
                    # Compare physical C/D jobs against the QR's original
                    # per-item pairs, including indistinguishable colors.
                    self.assertEqual(c, [(slot, int(pickup[3-slot]), int(destinations[3-slot]))
                                         for slot in (1,2,3)])
                    self.assertEqual([j[5] for j in self.jobs if j[:3] == (batch,'C','pick')], [1,2,3])

    def test_legacy_right_exit_is_replaced_by_inward_turn_on_both_rounds(self):
        # Scaled field geometry: D's former right exit would leave every lane.
        route = self.console.mission.snapshot()
        route['stations'][3].update(x=14, y=27, heading=90)
        route['stations'][4].update(x=28, y=13, heading=-180)
        route.update(turn_inset_enabled=True, turn_inset_steps=1)
        for leg in route['route']:
            if leg['station'] in (3,4):
                leg.update(task=1, outward_steps=2, right_steps=8)
        self.console.save_navigation(route)
        self.configure()
        planned = self.console.competition
        self.assertTrue(planned.route['turn_inset_enabled'])
        self.assertEqual(planned.route['turn_inset_steps'], 1)
        for index in ('3','4','6','7'):
            action = planned.task_config['legs'][index][0]
            self.assertEqual((action['right_steps'],action['outward_steps']), (0,2))
        phases = set()
        for _ in range(20000):
            phases.add((self.console.task.leg_base, self.console.navigator.phase))
            if planned.state == 'done':break
            self.frame()
        else:self.fail(self.console.task.snapshot())
        for leg in (3,4,6,7):
            self.assertIn((leg,'inward'), phases)
            self.assertIn((leg,'return'), phases)
        self.assertFalse(any(b.startswith('CHASSIS STRAIGHT RIGHT ') for b in self.link.sent))
        self.assertEqual(self.visits, [(1,'B'),(1,'C'),(1,'D'),(2,'B'),(2,'C'),(2,'D')])
        self.assertEqual((self.console.navigation['x'],self.console.navigation['y']), ('0','0'))
        self.assertEqual(self.console.mission.snapshot(), self.saved_route)

    def test_missing_color_waits_lowered_then_grabs_without_chassis_motion(self):
        self.configure()
        for _ in range(1000):
            self.frame()
            if self.console.task.state == 'material_align' and self.a.state == 'pickup_wait':break
        else:self.fail('no color tracking')
        task = self.console.task
        material = task.material
        # Keep live frames/feedback flowing past the former 60 s, 30 min and
        # 2 hour limits; the first target must still be eligible for pickup.
        budgets = [self.clock[0]-start for start in
                   (task.started, task.leg_started, material.started)]
        step = self.step
        with patch.object(self, 'step', side_effect=lambda:step(dt=.1)):
            for _ in range(72500):self.frame(missing=True)
        for start, budget in zip((task.started, task.leg_started, material.started), budgets):
            self.assertAlmostEqual(self.clock[0]-start, budget)
        self.assertEqual(self.console.task.state, 'material_align')
        self.assertEqual(self.console.pick_place.state, 'pickup_wait')
        self.assertEqual(self.board.axes[4]['pos'], -40)
        self.assertFalse(any(self.a.wheels))
        # Even a matching color outside the crop must not close the claw.
        for _ in range(8):self.frame(dx=100)
        self.assertEqual(self.console.pick_place.state, 'pickup_wait')
        self.assertFalse(any(self.a.wheels))
        self.assertFalse(any(b.startswith('WHEELS ') for b in self.link.sent))
        self.finish()
        self.assertEqual(self.console.navigation['x'], '0')

    def test_b_frame_pause_waits_without_closing_and_recovers(self):
        """A silent B camera pauses pickup until three new matching frames arrive."""
        self.configure()
        for _ in range(1000):
            self.frame()
            if self.console.task.state == 'material_align' and self.a.state == 'pickup_wait':break
        else:self.fail('no color tracking')
        for _ in range(8):self.frame(missing=True)
        for _ in range(80):
            self.step()
            if not self.console.task.active:break
        self.assertEqual(self.console.task.state, 'material_align', self.console.task.error)
        self.assertEqual(self.console.pick_place.state, 'pickup_wait')
        self.assertTrue(self.board.armed)
        self.finish()

    def test_start_establishes_negative_lift_and_current_rotation_zero(self):
        self.configure()
        # Fixture setup can restore AUX ranges before competition startup;
        # assert ordering only for the run being tested.
        self.link.sent.clear()
        self.board.lift_down_sign = 0
        self.board.rotation_referenced = False
        self.board.rotation_configured = False
        self.board.axes[5]['pos'] = -1234
        self.until(lambda: self.console.competition.state == 'running')
        self.assertEqual(self.board.lift_down_sign, -1)
        self.assertEqual(self.console.control_settings.snapshot()['lift'], [-1])
        self.assertTrue(self.board.rotation_referenced)
        self.assertEqual(self.board.axes[5]['pos'], 0)
        commands = self.link.sent
        self.assertLess(commands.index('ARM 0'), commands.index('LIFT DIR -1'))
        self.assertLess(commands.index('LIFT DIR -1'), commands.index('AUX ZERO 6'))
        self.assertLess(commands.index('AUX ZERO 6'), commands.index('AUX CONFIG 6 -2000 0'))
        self.assertLess(commands.index('AUX CONFIG 6 -2000 0'), commands.index('ARM 1'))
        self.assertFalse(any(b.startswith(('MOVE 6', 'AUX GOTO 6')) for b in commands))
        self.assertNotIn('AUX ZERO 5', commands)
        self.console.task_action('stop'); self.flush()
        before = len(commands)
        self.board.axes[5]['pos'] = -777
        self.assertTrue(self.console.competition.start(full_flow=True)[0])
        self.until(lambda: self.console.competition.state == 'running')
        self.assertEqual(commands[before:].count('AUX ZERO 6'), 1)
        self.assertEqual(self.board.axes[5]['pos'], 0)

    def test_rejected_rotation_zero_prevents_arming_and_departure(self):
        self.configure()
        # A range excluding zero makes the board reject the reference command.
        self.board.aux_ranges[1] = [-2000, -1]
        for _ in range(40):
            self.step()
            if self.console.competition.state == 'fault':break
        self.assertEqual(self.console.competition.state, 'fault')
        self.assertIn('AUX_ZERO', self.console.competition.error)
        self.assertNotIn('ARM 1', self.link.sent)
        self.assertFalse(any(b.startswith('NAV START') for b in self.link.sent))

    def test_stop_retires_pending_zero_ack(self):
        self.configure()
        self.console.competition.tick()
        for _ in range(100):
            body, tag = self.link.commands.get_nowait()
            if body == 'AUX ZERO 6':break
            self.console._reply(body, self.send(body), tag)
        else:self.fail('startup did not request rotation zero')
        self.assertEqual(body, 'AUX ZERO 6')
        reply = self.send(body)
        self.console.task_action('stop'); self.flush()
        self.console._reply(body, reply, tag)
        for _ in range(20):self.step()
        self.assertEqual(self.console.competition.state, 'stopped')
        self.assertNotIn('ARM 1', self.link.sent)

    def test_manual_mechanism_start_cannot_bypass_suspended_owner(self):
        self.configure()
        for _ in range(1500):
            self.frame()
            if self.console.task.state == 'material_align':break
        before = len(self.link.sent)
        self.assertFalse(self.console.pick_place_action('pick')[0])
        self.assertEqual(len(self.link.sent), before)

    def test_qr_search_switches_camera_after_code_before_returning_to_a(self):
        self.configure()
        for _ in range(500):
            task = self.console.task
            if task.state == 'qr_wait':
                self.qr_number += 1
                self.control.slots.states['2']['qr'] = dict(status='none', frame_id=self.qr_number,
                    captured_at=1700000000+self.clock[0])
            self.frame(qr=False)
            if task.qr_search and abs(task.qr_search.position) >= 800:break
        else:self.fail('no QR search movement')
        self.assertNotIn('1', self.control.requests)
        self.finish()
        self.assertEqual(self.cameras, [1])
        self.assertEqual(self.console.navigation['x'], '0')

    def test_stop_during_startup_cancels_before_arming(self):
        self.configure()
        self.assertTrue(self.console.task_action('stop')[0])
        self.flush()
        before = len(self.link.sent)
        for _ in range(20):self.step()
        self.assertFalse(self.console.competition.active)
        self.assertNotIn('ARM 1', self.link.sent[before:])

    def test_b_handoff_rechecks_arrival_after_yaw_recovery_race(self):
        """A drift after arrival confirmation must not abort the held B visit."""
        self.configure()
        original = self.send
        rejected = []
        injected = []

        def send(body):
            if body.startswith('NAV SUSPEND ') and self.console.task.leg == 2 and not injected:
                injected.append(body)
                nav = self.board.navigation
                nav.feedback(nav.absolute_heading+12, True, nav.imu_epoch, self.clock[0])
                nav.update(self.clock[0], 0)
            reply = original(body)
            if reply.fields.get('code') == 'NAV_HANDOFF_NOT_READY':
                rejected.append(body)
            return reply

        self.send = send
        self.finish()
        self.assertEqual(len(injected), 1)
        self.assertEqual(len(rejected), 1)
        self.assertEqual(self.console.task.state, 'done')

    def test_camera_one_opens_after_a_code_and_stays_open_until_o(self):
        """The full route owns one camera stream across both material batches."""
        self.configure()
        for _ in range(1000):
            if self.console.task.task_plan:
                break
            self.frame()
        self.assertEqual(self.control.requests, ['2', '1'])
        self.assertIsNone(self.control.timeouts[-1])
        self.assertEqual(self.console.task.leg, 1)
        self.finish()
        self.assertEqual(self.control.requests, ['2', '1'])
        self.assertEqual(self.control.slots.states['1']['state'], 'idle')
        self.assertEqual(self.control.slots.states['2']['state'], 'idle')


if __name__ == '__main__':
    unittest.main()
