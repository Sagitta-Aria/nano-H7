"""Verify B-only persistence, HTTP permissions, and two-batch parameter routing."""
import json
from pathlib import Path
import tempfile
import unittest
import urllib.request
import urllib.error
from types import SimpleNamespace
from unittest.mock import patch

from host.webapp import Console
from host.pick_place import PickPlaceSettings, DEFAULT
from host.b_pickup import MECHANISM_KEYS, ROI_DEFAULT, DELAY_DEFAULT
from tests import test_material_flow as flow
from tests import test_pick_place as pickup
from tests.test_camera_servo import VelocityLink
from tests.test_task_code import FakeCameraControl


class BPickupConfigTests(unittest.TestCase):
    def test_legacy_b_config_gets_crop_without_rewriting_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root/'b_pickup.json'
            old = {key: DEFAULT[key] for key in MECHANISM_KEYS}
            old['pick_down'] = 7654
            path.write_text(json.dumps(old), encoding='utf-8')
            before = path.read_bytes()
            c = Console(navigation_path=root/'navigation.json')
            self.assertEqual(c.b_pickup.snapshot(), dict(old, **ROI_DEFAULT, **DELAY_DEFAULT))
            self.assertEqual(path.read_bytes(), before)
            for update in ({'roi_x_pct':80}, {'roi_width_pct':0},
                           {'roi_height_pct':True}, {'roi_y_pct':-1},
                           {'recognition_delay_ms':-1}, {'recognition_delay_ms':60001},
                           {'recognition_delay_ms':True}, {'recognition_delay_ms':2.5}):
                with self.assertRaises(ValueError):
                    c.b_pickup.save(dict(c.b_pickup.snapshot(), **update))
            self.assertEqual(path.read_bytes(), before)

    def test_seed_once_and_keep_independent_after_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            PickPlaceSettings(root/'pick_place.json').save(dict(DEFAULT, pick_down=8765))
            c = Console(navigation_path=root/'navigation.json')
            self.assertEqual(c.b_pickup.snapshot()['pick_down'], 8765)
            original = c.pick_place.settings.snapshot()
            c.b_pickup.save(dict(c.b_pickup.snapshot(), pick_down=7654, recognition_delay_ms=3456))
            self.assertEqual(c.pick_place.settings.snapshot(), original)
            c.pick_place.save(dict(original, pick_down=6543))
            restarted = Console(navigation_path=root/'navigation.json')
            self.assertEqual(restarted.b_pickup.snapshot()['pick_down'], 7654)
            self.assertEqual(restarted.b_pickup.snapshot()['recognition_delay_ms'], 3456)
            before = restarted.b_pickup.path.read_bytes()
            for update in ({'pick_down':True}, {'lift_sps':0}, {'s1_open':181}, {'s2_slot_1':90}):
                with self.assertRaises(ValueError):
                    restarted.b_pickup.save(dict(restarted.b_pickup.snapshot(), **update))
            self.assertEqual(restarted.b_pickup.path.read_bytes(), before)


class BPickupFlowTests(unittest.TestCase):
    setUp = flow.MaterialFlowTests.setUp
    send = flow.MaterialFlowTests.send
    flush = flow.MaterialFlowTests.flush
    step = flow.MaterialFlowTests.step
    until = flow.MaterialFlowTests.until
    configure = flow.MaterialFlowTests.configure
    frame = flow.MaterialFlowTests.frame
    finish = flow.MaterialFlowTests.finish

    def test_cold_camera_first_frame_can_take_five_seconds_at_b(self):
        """Replay B arrival with a ready camera whose recognition is still starting."""
        self.configure(code='321+123+546+231')
        for _ in range(1500):
            self.frame()
            if self.a.state == 'pickup_wait':
                break
        else:
            self.fail('no B color wait')
        before = len(self.link.sent)
        for _ in range(100):
            self.step()
        self.assertEqual(self.a.state, 'pickup_wait', self.a.error)
        self.assertEqual(self.console.task.state, 'material_align', self.console.task.error)
        self.assertEqual(self.a.config['target'], 'blue')
        self.assertFalse(any(body.startswith(('SERVO ', 'AUX GOTO', 'WHEELS '))
                             and body != 'SERVO STAT' for body in self.link.sent[before:]))
        for _ in range(3):
            self.frame()
        self.assertEqual(self.a.state, 'done', self.a.error)
        self.finish()

    def test_camera_opening_after_a_has_no_elapsed_limit(self):
        """Camera handoff may finish after the former opening/station/run limits."""
        self.configure()
        for _ in range(1500):
            self.frame()
            if self.console.task.state == 'qr_switch':
                break
        else:
            self.fail('no camera1 opening after A code')
        for _ in range(700):
            self.step(.05)
        self.assertEqual(self.console.task.state, 'qr_switch', self.console.task.error)
        self.assertEqual(self.control.timeouts[-1], None)
        self.finish()

    def test_arrival_ignores_stale_wheel_cache_and_checks_live_axes(self):
        """Held navigation is fresh; AXIS cache can still describe travel."""
        self.configure()
        for _ in range(1500):
            self.frame()
            material = self.console.task.material
            if material and material.station == 'B' and material.phase == 'camera':
                break
        else:
            self.fail('no B camera handoff')
        self.assertEqual(self.console.navigation['state'], 'suspended')
        self.assertFalse(any(axis['active'] for axis in self.board.axes))
        for axis in range(1, 5):
            self.console.axes[str(axis)] = dict(axis=str(axis), active='1')
        for _ in range(200):
            self.frame()
            if self.a.state == 'pickup_wait':
                break
        else:
            self.fail('stale travel telemetry prevented pickup')
        self.assertEqual(self.board.axes[4]['pos'], -40)
        self.assertTrue(all('AXIS '+str(axis) in self.link.sent for axis in range(1, 8)))

    def test_b_color_inside_crop_grabs_without_center_alignment(self):
        """A preset pickup position needs color confirmation, not PID convergence."""
        self.configure()
        for _ in range(1500):
            self.frame()
            if self.a.state == 'pickup_wait':
                break
        else:
            self.fail('no B color wait')
        before = len(self.link.sent)
        self.assertGreater(60, self.a.config['tracking']['deadzone_px'])
        for _ in range(3):
            self.frame(dx=60)
        self.assertEqual(self.a.state, 'done')
        for _ in range(60):
            self.step()
            if 'SERVO 2 75' in self.link.sent[before:]:
                break
        self.assertIn('SERVO 2 75', self.link.sent[before:])
        self.assertFalse(any(body.startswith(('WHEELS ', 'TURN ZERO', 'TURN MOVE', 'CHASSIS STRAIGHT', 'NAV NUDGE'))
                             for body in self.link.sent[before:]))

    def test_station_preflight_still_rejects_actual_axis_motion(self):
        """Fresh board motion must stop pickup even when cached axes say idle."""
        self.configure()
        for _ in range(1500):
            self.frame()
            material = self.console.task.material
            if material and material.station == 'B' and material.phase == 'camera':
                break
        else:
            self.fail('no B camera handoff')
        self.assertTrue(self.send('MOVE 7 5000 100 1000').ok)
        self.control.finish()
        before = len(self.link.sent)
        for _ in range(100):
            self.step()
            if self.console.task.state == 'fault':
                break
        self.assertEqual(self.console.task.state, 'fault')
        self.assertIn('请先停止所有轴', self.console.task.error)
        self.assertIn('AXIS 7', self.link.sent[before:])
        self.assertFalse(any(body.startswith(('AUX GOTO', 'SERVO ')) and body != 'SERVO STAT'
                             for body in self.link.sent[before:]))
        self.assertFalse(self.board.armed)

    def test_each_b_pick_opens_lowers_waits_closes_and_returns_home(self):
        self.configure()
        confirmed = set()
        for _ in range(20000):
            if self.console.competition.state == 'done':break
            job = self.console.pick_place
            if self.a.state == 'pickup_wait' and job.job_id not in confirmed:
                self.assertEqual(job.state, 'pickup_wait')
                self.assertEqual(self.board.axes[4]['pos'], -40)
                self.assertFalse(self.board.axes[4]['active'])
                self.assertEqual(self.board.axes[5]['pos'], 0)
                before = len(self.link.sent)
                # No color: neither jaw closure nor chassis motion is allowed.
                for _ in range(5):self.frame(missing=True)
                self.assertEqual(job.state, 'pickup_wait')
                self.assertEqual(self.a.stable, 0)
                self.assertFalse(any(b.startswith(('SERVO ', 'AUX GOTO', 'WHEELS '))
                                     for b in self.link.sent[before:]))
                with self.assertRaises(ValueError):
                    job.vision(dict(job.request, ready=True, captured_at=1700000000+self.clock[0]))
                for expected in (1, 2):
                    self.frame()
                    self.assertEqual(self.a.stable, expected)
                    self.assertEqual(job.state, 'pickup_wait')
                self.frame(missing=True)
                self.assertEqual(self.a.stable, 0)
                for _ in range(3):self.frame()
                self.assertEqual(self.a.state, 'done')
                self.until(lambda:job.state == 'pickup_delay')
                before = len(self.link.sent)
                started = self.clock[0]
                for _ in range(39):self.step()
                self.assertEqual(job.state, 'pickup_delay')
                self.assertFalse(any(b.startswith(('SERVO ', 'AUX GOTO', 'WHEELS '))
                                     and b != 'SERVO STAT' for b in self.link.sent[before:]))
                self.step(.051)
                self.assertGreaterEqual(self.clock[0]-started, 2)
                self.assertIn('SERVO 2 75', self.link.sent[before:])
                confirmed.add(job.job_id)
            self.frame()
        else:self.fail(self.console.task.snapshot())
        self.assertEqual(len(confirmed), 6)
        self.assertEqual(self.board.axes[4]['pos'], 0)
        self.assertEqual(self.board.axes[5]['pos'], 0)

    def test_stop_while_lowered_does_not_close_or_resume_claw(self):
        self.configure()
        for _ in range(1500):
            self.frame()
            if self.a.state == 'pickup_wait':break
        else:self.fail('no lowered color wait')
        self.console.task_action('stop')
        self.flush()
        before = len(self.link.sent)
        for _ in range(30):self.step()
        self.assertFalse(self.console.pick_place.active)
        self.assertFalse(self.a.active)
        self.assertFalse(self.board.armed)
        self.assertFalse(any(b.startswith(('SERVO ', 'AUX GOTO', 'WHEELS ')) and b != 'SERVO STAT'
                             for b in self.link.sent[before:]))

    def test_both_b_visits_use_b_parameters_and_second_d_only_changes_drop_depth(self):
        self.configure()
        self.console.competition.stop()
        b = dict(self.console.b_pickup.snapshot(), pick_down=37, pick_rotation=-17, s1_closed=65)
        self.console.b_pickup.save(b)
        self.assertTrue(self.console.competition.start(full_flow=True)[0])
        original = self.console.pick_place.settings.snapshot()
        seen = []
        start = self.console.pick_place.start

        def capture(*args, **kwargs):
            result = start(*args, **kwargs)
            job = self.console.pick_place
            seen.append((self.console.task.material.station, job.context['mode'],
                         job.context['batch'], dict(job.config)))
            return result

        with patch.object(self.console.pick_place, 'start', side_effect=capture):
            self.finish()
        self.assertEqual(len(seen), 24)
        self.assertEqual(sum(s == 'B' for s, _, _, _ in seen), 6)
        for station, mode, batch, config in seen:
            if station == 'B':
                for key in MECHANISM_KEYS:
                    self.assertEqual(config[key], b[key])
            else:
                expected = dict(original)
                if station == 'C' and mode == 'pick':expected['pick_down'] = original['place_drop_down']
                if station == 'D' and batch == 2:
                    expected['place_drop_down'] = original['d_second_drop_down']
                self.assertEqual(config, expected)
        self.assertIn('AUX GOTO 5 37 3000 1000', self.link.sent)
        self.assertEqual(self.console.pick_place.settings.snapshot(), original)

    def test_b_target_outside_range_fails_before_departure(self):
        self.configure()
        self.console.competition.stop()
        self.console.b_pickup.save(dict(self.console.b_pickup.snapshot(), pick_down=10001))
        self.assertTrue(self.console.competition.start(full_flow=True)[0])
        for _ in range(40):
            self.step()
            if self.console.competition.state == 'fault':break
        self.assertEqual(self.console.competition.state, 'fault')
        self.assertIn('10001', self.console.competition.error)
        self.assertNotIn('ARM 1', self.link.sent)


class BPickupStandaloneTests(unittest.TestCase):
    """One-click B tests exercise the real mechanism and camera ownership barriers."""
    send = pickup.PickPlaceTests.send
    flush = pickup.PickPlaceTests.flush
    step = pickup.PickPlaceTests.step
    until = pickup.PickPlaceTests.until
    start = pickup.PickPlaceTests.start
    actions = pickup.PickPlaceTests.actions

    def setUp(self):
        pickup.PickPlaceTests.setUp(self)
        self.console.connection = self.link = VelocityLink()
        self.console.task_cameras = self.control = FakeCameraControl()
        self.a = self.console.camera_alignment
        self.a.hub = SimpleNamespace(snapshot=lambda:self.control.slots.states['1'],
            start_tracking=lambda config:None, stop_tracking=lambda:None)
        self.a.capture = lambda image=None:setattr(self.a, 'capturing', True)
        self.b = dict(self.console.b_pickup.snapshot(), pick_down=123, pick_rotation=-45,
                      lift_sps=444, rotation_sps=555, s1_closed=80,
                      s1_settle_ms=100, s2_settle_ms=100)
        self.console.b_pickup.save(self.b)

    def lowered(self, code='321+123+546+231'):
        """Start from a closed camera and reach the verified lowered color gate."""
        self.start(profile='b_pickup', task_code=code)
        self.assertEqual(self.control.requests, ['1'])
        self.assertEqual(self.actions(), [])
        self.control.finish()
        self.until(lambda:self.a.state == 'pickup_wait')

    def frame(self, found=True, target=None):
        """Inject fresh control reports, preserving actual target and crop validation."""
        self.a.results.put((self.a.generation, dict(kind='block',
            target=target or self.a.config['target'], found=found,
            frame_id=self.a.last_frame_id+1, captured_monotonic=self.clock[0],
            captured_at=1700000000+self.clock[0], width=640, height=480,
            center=[320,240], roi=[224,168,192,144]), None, None))
        self.step()

    def test_opens_camera_and_grabs_only_selected_color_with_b_parameters(self):
        generic = self.module.settings.snapshot()
        self.lowered()
        self.assertEqual(self.module.context['color_id'], 3)
        self.assertEqual(self.a.config['target'], 'blue')
        self.assertEqual(self.module.context['profile'], 'b_pickup')
        self.assertEqual(self.module.config['lift_sps'], 444)
        self.assertIn('SERVO 2 170', self.actions())
        self.assertLess(self.actions().index('AUX GOTO 5 123 444 1000'),
                        self.actions().index('SERVO 2 170'))
        for _ in range(100):self.step()
        self.assertEqual(self.module.state, 'pickup_wait', self.module.error)
        for _ in range(5):self.frame(found=False)
        self.assertNotIn('SERVO 2 80', self.actions())
        self.frame(); self.frame()
        for _ in range(80):self.step()
        self.assertEqual(self.module.state, 'pickup_wait', self.module.error)
        self.frame()
        self.assertEqual(self.a.stable, 1)
        self.frame(); self.frame()
        self.until(lambda:self.module.state == 'done')
        self.assertIn('SERVO 2 80', self.actions())
        self.assertIn('AUX GOTO 5 123 444 1000', self.actions())
        self.assertIn('AUX GOTO 6 -45 555 1000', self.actions())
        self.assertEqual(self.module.settings.snapshot(), generic)
        self.assertFalse(any(b.startswith(('WHEELS ', 'CHASSIS STRAIGHT', 'TURN ZERO', 'NAV START'))
                             for b in self.link.sent))

    def test_opening_can_wait_and_operator_stop_cancels_camera_request(self):
        self.start(profile='b_pickup')
        for _ in range(700):self.step()
        self.assertEqual(self.module.state, 'camera', self.module.error)
        self.assertEqual(self.control.timeouts[-1], None)
        self.assertEqual(self.actions(), [])
        self.console.pick_place_action('stop')
        self.assertEqual(self.control.snapshot()['state'], 'idle')
        self.control.finish()
        for _ in range(10):self.step()
        self.assertEqual(self.module.state, 'stopped')
        self.assertEqual(self.actions(), [])

    def test_default_delay_starts_after_color_confirmation(self):
        self.lowered()
        # Time spent waiting for color cannot consume the post-recognition delay.
        for _ in range(80):self.step()
        self.frame(); self.frame()
        self.assertEqual(self.module.state, 'pickup_wait')
        self.frame()
        self.until(lambda:self.module.state == 'pickup_delay')
        self.assertTrue(self.module.active)
        self.assertIn('2000 ms', self.module.snapshot()['label'])
        before = len(self.actions())
        started = self.clock[0]
        for _ in range(38):self.step()
        self.step(.099)
        self.assertEqual(self.module.state, 'pickup_delay')
        self.assertEqual(len(self.actions()), before)
        self.step(.0011)
        self.assertGreaterEqual(self.clock[0]-started, 2)
        self.assertEqual(self.actions()[-1], 'SERVO 2 80')
        self.until(lambda:self.module.state == 'done')

    def test_adjustable_delay_can_exceed_mechanism_timeout_or_be_zero(self):
        for delay in (2500, 0):
            with self.subTest(delay=delay):
                self.console.b_pickup.save(dict(self.b, recognition_delay_ms=delay,
                                                step_timeout_ms=1000))
                self.control.requests.clear()
                self.link.sent.clear()
                self.lowered()
                self.frame(); self.frame(); self.frame()
                self.until(lambda:self.module.state == 'pickup_delay')
                before = len(self.actions())
                if delay:
                    for _ in range(49):self.step()
                    self.step(.049)
                    self.assertEqual(self.module.state, 'pickup_delay', self.module.error)
                    self.assertEqual(len(self.actions()), before)
                self.step(.002)
                self.assertEqual(self.actions()[-1], 'SERVO 2 80')
                self.until(lambda:self.module.state == 'done')

    def test_stop_during_delay_prevents_late_closure(self):
        self.lowered()
        self.frame(); self.frame(); self.frame()
        self.until(lambda:self.module.state == 'pickup_delay')
        with self.assertRaises(ValueError):
            self.console.b_pickup.save(self.b)
        self.console.pick_place_action('stop')
        self.flush()
        before = len(self.actions())
        for _ in range(60):self.step()
        self.assertEqual(self.module.state, 'stopped')
        self.assertEqual(len(self.actions()), before)
        self.assertNotIn('SERVO 2 80', self.actions())

    def test_lost_arm_during_delay_faults_without_closing(self):
        self.lowered()
        self.frame(); self.frame(); self.frame()
        self.until(lambda:self.module.state == 'pickup_delay')
        self.console.armed = False
        self.step()
        self.assertEqual(self.module.state, 'fault')
        self.assertNotIn('SERVO 2 80', self.actions())

    def test_stop_while_waiting_cancels_recognition_and_late_frames(self):
        self.lowered()
        generation = self.a.generation
        self.console.pick_place_action('stop')
        self.assertFalse(self.a.active)
        self.assertGreater(self.a.generation, generation)
        for _ in range(3):
            self.a.results.put((generation, dict(found=True), None, None))
            self.step()
        self.assertEqual(self.module.state, 'stopped')
        self.assertNotIn('SERVO 2 80', self.actions())

    def test_explicit_camera_error_faults_without_closing(self):
        self.lowered()
        self.a.results.put((self.a.generation, None, None, 'camera disconnected'))
        self.step(); self.step()
        self.assertEqual(self.module.state, 'fault')
        self.assertIn('camera disconnected', self.module.error)
        self.assertNotIn('SERVO 2 80', self.actions())

    def test_external_confirmation_cannot_bypass_camera_color_gate(self):
        self.lowered()
        with self.assertRaises(ValueError):
            self.module.vision(dict(self.module.request, ready=True,
                captured_at=1700000000+self.clock[0]))
        self.assertFalse(self.module.vision_ready)

    def test_b_wait_uses_cancelable_unbounded_hub_wait(self):
        from unittest.mock import Mock
        from host.camera_alignment import CameraAlignment
        self.lowered()
        self.a.capturing = False
        self.a.hub.tracking_sample = Mock(side_effect=ValueError('canceled'))
        with patch('host.camera_alignment.threading.Thread',
                   side_effect=lambda **kw:SimpleNamespace(start=kw['target'])):
            CameraAlignment.capture(self.a)
        self.a.hub.tracking_sample.assert_called_once_with(
            self.a.capture_after, self.a.last_frame_id, timeout=None)


class BPickupHttpTests(unittest.TestCase):
    setUp = pickup.PickPlaceHttpTests.setUp
    tearDown = pickup.PickPlaceHttpTests.tearDown

    def call(self, payload=None, token='writer'):
        request = urllib.request.Request('http://127.0.0.1:{}/api/b-pickup/config'.format(self.server.server_port),
            data=None if payload is None else json.dumps(payload).encode(), headers={'X-Session':token})
        try:
            with urllib.request.urlopen(request, timeout=3) as response:return response.status, json.load(response)
        except urllib.error.HTTPError as error:return error.code, json.load(error)

    def test_save_permissions_invalid_data_and_active_owner(self):
        config = self.call()[1]['config']
        config['pick_down'] = 5432
        self.assertEqual(self.call({'config':config}, 'reader')[0], 409)
        self.assertEqual(self.call({'config':config})[1]['config'], config)
        self.assertEqual(self.call({'config':{}})[0], 400)
        for owner, state in ((self.console.competition,'connecting'),
                             (self.console.task,'travel'), (self.console.pick_place,'running')):
            previous = owner.state; owner.state = state
            self.assertEqual(self.call({'config':dict(config, pick_down=1111)})[0], 400)
            owner.state = previous
        self.assertEqual(self.call()[1]['config'], config)
        self.assertEqual(self.console.pick_place.settings.snapshot(), DEFAULT)


if __name__ == '__main__':unittest.main()
