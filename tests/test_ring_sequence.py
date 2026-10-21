"""Real protocol/SIM checks of bounded search, gyro turns and OCR ordering."""
import copy
import importlib.util
import math
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from host.camera_vision import DEFAULT, VisionSettings, validated, detect
from host.ring_sequence import row_line
from host.ring_perception import RingOCR
from tests import test_camera_servo as servo_fixture


class GeometryTests(unittest.TestCase):
    def test_spacing_upgrades_old_file_and_persists_adjustment(self):
        config = copy.deepcopy(DEFAULT)
        config['ring_sequence'].pop('spacing_steps')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'camera.json'
            path.write_text(json.dumps(config), encoding='utf-8')
            original = path.read_bytes()
            settings = VisionSettings(path)
            self.assertEqual(settings.snapshot()['ring_sequence']['spacing_steps'], 2200)
            self.assertEqual(path.read_bytes(), original)
            config = settings.snapshot()
            config['ring_sequence']['spacing_steps'] = 1875
            settings.save(config)
            self.assertEqual(VisionSettings(path).snapshot(), config)
        for bad in (0, -1, True, 2.5, 100001):
            config['ring_sequence']['spacing_steps'] = bad
            with self.assertRaises(ValueError): validated(config)

    def test_mount_rotation_and_shortest_parallel_turn(self):
        self.assertAlmostEqual(row_line([[50, 100], [250, 100]], 90)['error_deg'], 0)
        self.assertAlmostEqual(row_line([[50, 100], [250, 200]], 90)['error_deg'], math.degrees(math.atan(.5)))
        self.assertAlmostEqual(row_line([[250, 200], [50, 100]], 90)['error_deg'], math.degrees(math.atan(.5)))
        self.assertAlmostEqual(row_line([[100, 50], [100, 250]], 0)['error_deg'], 0)
        self.assertIsNone(row_line([[100, 100]], 90))
        self.assertIsNone(row_line([[100, 100], [102, 102]], 90))

    def test_legacy_config_upgrade_and_search_boundaries(self):
        config = copy.deepcopy(DEFAULT)
        config.pop('ring_sequence')
        self.assertEqual(validated(config)['ring_sequence']['range_steps'], 2000)
        for key, value in [('range_steps', 0), ('scan_step', True), ('sps', 2001),
                           ('angle_tolerance_deg', float('nan')), ('max_turn_deg', 31)]:
            config = copy.deepcopy(DEFAULT)
            config['ring_sequence'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validated(config)

    def test_ocr_never_substitutes_the_requested_digit_for_the_actual_result(self):
        reader = RingOCR.__new__(RingOCR)
        reader.configure({'confidence': .6, 'confidence_one': .3})
        reader.engine = object()
        reader.algorithm = SimpleNamespace(extract_ring_interior=Mock(return_value=object()),
                                          recognize=Mock(return_value=('3', .95, 12)))
        rings = [{'center': (100, 100), 'area': 500}]
        self.assertIsNone(reader.select(None, rings, '1'))
        match = reader.select(None, rings, '3')
        self.assertEqual(match['digit'], '3')
        self.assertEqual(match['center'], (100, 100))

    @unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV unavailable')
    def test_same_frame_distinct_rings_and_concentric_rejection(self):
        import cv2
        import numpy as np
        config = copy.deepcopy(DEFAULT); config['kind'] = 'ring'; config['detector']['ring']['min_area'] = 500
        image = np.full((480, 640, 3), 255, np.uint8)
        cv2.circle(image, (160, 180), 60, (0, 0, 0), 6)
        cv2.circle(image, (160, 180), 35, (0, 0, 0), 6)
        one = detect(image.copy(), config, 1.)
        self.assertEqual(len(one['rings']), 1)
        self.assertIsNone(one['row_line'])
        cv2.circle(image, (450, 240), 55, (0, 0, 0), 6)
        two = detect(image, config, 2.)
        self.assertEqual(len(two['rings']), 2)
        self.assertIsNotNone(two['row_line'])
        self.assertIsNone(two['digit'])


class SequenceTests(unittest.TestCase):
    # Reuse only the protocol fixture, without inheriting its unrelated tests.
    setUp = servo_fixture.ServoTests.setUp
    send = servo_fixture.ServoTests.send
    flush = servo_fixture.ServoTests.flush
    step = servo_fixture.ServoTests.step
    until = servo_fixture.ServoTests.until

    def start(self, extent=250):
        self.a.settings.value['kind'] = 'ring'
        self.a.settings.value['tracking']['camera_yaw_deg'] = 90
        self.a.settings.value['ring_sequence'].update(range_steps=extent, scan_step=100, sps=300)
        self.a.start(); self.flush(); self.until(lambda:self.a.state=='ring_sequence')
        self.assertEqual(self.a.state, 'ring_sequence', self.a.error)

    def sample(self, angle=None, digit=None, **extra):
        self.clock[0] = max(self.clock[0], self.a.ring_sequence.after+.01)
        report = dict(kind='ring', target='ring', found=digit is not None or angle is not None,
                      frame_id=self.a.last_frame_id+1, captured_monotonic=self.clock[0],
                      center=[320, 240], control_point=[320, 270], reference=[320, 240],
                      error_dx=0, error_dy=0,
                      digit=digit, row_line=None if angle is None else {'error_deg':angle,'points':[[100,200],[500,200]]})
        report.update(extra)
        self.a.results.put((self.capture_generation, report, None, None))
        self.step()

    def complete_motion(self):
        self.until(lambda:not self.a.ring_sequence.operation, limit=300)

    def align(self):
        for _ in range(3): self.sample(angle=0)
        self.assertEqual(self.a.ring_sequence.phase, 'seek')
        self.assertEqual(self.a.ring_sequence.expected, 1)

    def test_search_stays_inside_plus_minus_x_and_fails_after_one_sweep(self):
        self.start()
        positions = [0]
        for _ in range(30):
            self.sample()
            if not self.a.active: break
            self.complete_motion()
            positions.append(self.a.ring_sequence.position)
        self.assertEqual(self.a.state, 'fault')
        self.assertIn(-250, positions); self.assertIn(250, positions)
        self.assertTrue(all(-250 <= p <= 250 for p in positions))
        self.assertEqual(self.a.ring_sequence.total, 1000)
        self.assertFalse(self.board.armed)
        self.assertFalse(any(cmd.startswith('TURN MOVE') for cmd in self.link.sent))

    def test_same_frame_line_rotates_before_any_digit_search(self):
        self.start()
        for _ in range(3): self.sample(angle=12)
        self.assertEqual(self.a.ring_sequence.operation[0], 'turn')
        self.assertIn('TURN MOVE REL 1200 300 6000 100', self.link.sent)
        self.assertFalse(any(call.kwargs.get('phase') for call in self.a.hub.start_tracking.call_args_list))
        self.complete_motion()
        self.align()
        self.assertEqual(self.a.hub.start_tracking.call_args.kwargs['phase'], '1')

    def test_five_degrees_is_accepted_only_when_inside_saved_tolerance(self):
        """Replay the saved 10-degree setting, then the requested 2-degree limit."""
        self.a.settings.value['ring_sequence']['angle_tolerance_deg'] = 10
        self.start()
        for _ in range(3): self.sample(angle=-5.3)
        self.assertEqual(self.a.ring_sequence.phase, 'seek')
        self.assertFalse(any(c.startswith('TURN MOVE') for c in self.link.sent))
        self.c.submit('STOP'); self.flush()
        self.c.submit('ARM 1'); self.flush()
        self.a.settings.value['ring_sequence']['angle_tolerance_deg'] = 2
        self.start()
        for _ in range(3): self.sample(angle=-5.3)
        self.assertIn('TURN MOVE REL -530 300 6000 100', self.link.sent)

    def test_parallel_acceptance_requires_three_frames_inside_strict_band(self):
        self.a.settings.value['ring_sequence']['angle_tolerance_deg'] = .4
        self.start()
        for angle in (.7, .4, .2): self.sample(angle=angle)
        self.assertNotEqual(self.a.ring_sequence.phase, 'seek')
        self.sample(angle=.1)
        self.assertEqual(self.a.ring_sequence.phase, 'seek')
        self.assertFalse(any(c.startswith('TURN MOVE') for c in self.link.sent))

    def test_row_drift_while_seeking_realigns_before_accepting_digit(self):
        self.start(); self.align()
        for _ in range(5): self.sample(angle=5.3, digit='1')
        self.assertEqual(self.a.ring_sequence.operation, ('turn', 530))
        self.assertEqual(self.a.ring_sequence.recognized, [])
        self.assertEqual(self.link.sent.count('TURN ZERO'), 1)

    def test_approach_row_drift_brakes_then_realigns_before_accepting_anchor(self):
        """A live 5-degree row must not be ignored after OCR selected digit 1."""
        self.start(); self.align()
        for _ in range(3): self.sample(angle=0, digit='1')
        self.sample(angle=0, digit='1', error_dx=100)
        for _ in range(4): self.step()
        self.assertTrue(any(a['active'] for a in self.board.axes[:4]))
        before = len(self.link.sent)
        self.sample(angle=-5.3, digit='1')
        self.assertEqual(self.a.ring_sequence.phase, 'realign_brake')
        self.assertEqual(self.a.wheels, [0]*4)
        self.assertEqual(self.a.ring_sequence.line['error_deg'], -5.3)
        self.assertFalse(any(c.startswith('TURN MOVE') for c in self.link.sent[before:]))
        self.until(lambda:self.a.state == 'ring_sequence')
        self.assertFalse(any(a['active'] for a in self.board.axes[:4]))
        self.assertIsNone(self.link.velocity)
        for _ in range(3): self.sample(angle=-5.3, digit='1')
        self.assertEqual(self.a.ring_sequence.operation, ('turn', -530))
        self.complete_motion()
        self.align()
        for _ in range(20):
            if self.a.ring_sequence.recognized: break
            self.sample(angle=0, digit='1')
        self.assertEqual(self.a.ring_sequence.phase, 'dwell')
        self.assertTrue(self.a.ring_sequence.recognized[0]['aligned'])

    def test_stop_during_row_realign_braking_cancels_later_turns(self):
        self.start(); self.align()
        for _ in range(3): self.sample(digit='1')
        self.sample(angle=5.3, digit='1', error_dx=100)
        self.c.submit('STOP'); self.flush()
        before = len(self.link.sent)
        for _ in range(20): self.step()
        self.assertEqual(self.a.state, 'stopped')
        self.assertFalse(any(c.startswith(('TURN MOVE', 'WHEELS ', 'CHASSIS STRAIGHT'))
                             for c in self.link.sent[before:]))

    def test_one_uses_ocr_then_two_and_three_use_full_spacing_without_frames(self):
        self.start(); self.align()
        for _ in range(5): self.sample(digit='3')
        self.assertEqual(self.a.ring_sequence.expected, 1)
        self.assertEqual(self.a.ring_sequence.operation, ('move', -100))
        self.complete_motion()
        self.sample(digit='1'); self.sample(digit='1')
        self.assertEqual(self.a.ring_sequence.recognized, [])
        self.sample(digit='1')
        self.assertEqual(self.a.ring_sequence.phase, 'approach')
        for _ in range(3): self.sample(digit='1')
        self.until(lambda:self.a.ring_sequence.phase == 'dwell')
        before = len(self.link.sent)
        self.until(lambda:self.a.state == 'done', limit=700)
        self.assertEqual(self.a.state, 'done')
        self.assertEqual([v['digit'] for v in self.a.ring_sequence.recognized], [1, 2, 3])
        self.assertFalse(any(axis['active'] for axis in self.board.axes[:4]))
        self.assertTrue(any(cmd.startswith('WHEELS ') for cmd in self.link.sent))
        self.assertTrue(self.a.ring_sequence.recognized[0]['aligned'])
        commands = self.link.sent[before:]
        self.assertEqual([c for c in commands if c.startswith('CHASSIS STRAIGHT')],
                         ['CHASSIS STRAIGHT FWD 2200 300 6000']*2)
        self.assertFalse(any(c.startswith(('WHEELS ', 'TURN ZERO', 'CHASSIS HALT')) for c in commands))
        self.assertEqual([r['positioning'] for r in self.a.ring_sequence.recognized],
                         ['vision', 'fixed_spacing', 'fixed_spacing'])
        self.assertEqual([r['position_steps'] for r in self.a.ring_sequence.recognized[1:]], [2200,4400])
        self.assertEqual([c.kwargs.get('phase') for c in self.a.hub.start_tracking.call_args_list], [None,'1'])

    def calibrated_one(self, spacing=2200):
        """Reach the visual anchor through real protocol before fixed travel."""
        self.a.settings.value['ring_sequence']['spacing_steps'] = spacing
        self.start(); self.align()
        for _ in range(6): self.sample(digit='1')
        self.until(lambda:self.a.ring_sequence.phase == 'dwell')

    def test_adjusted_spacing_advances_after_completion_without_dwell(self):
        self.calibrated_one(1875)
        sequence = self.a.ring_sequence
        # Edits to saved settings do not change the active run's pinned distance.
        self.a.settings.value['ring_sequence']['spacing_steps'] = 99
        for digit in (1, 2):
            self.assertEqual(sequence.expected, digit)
            self.assertTrue(sequence.recognized[-1]['aligned'])
            self.assertFalse(any(axis['active'] for axis in self.board.axes[:4]))
            self.assertEqual(sequence.phase, 'dwell')
            before = len(self.link.sent)
            self.step()
            self.assertEqual(sequence.expected, digit+1)
            self.assertEqual([cmd for cmd in self.link.sent[before:] if cmd.startswith('CHASSIS STRAIGHT')],
                             ['CHASSIS STRAIGHT FWD 1875 300 6000'])
            self.assertEqual(sequence.phase, 'spacing')
            self.assertEqual(sequence.operation, ('move', 1875))
            self.assertEqual(len(sequence.recognized), digit)
            self.complete_motion()
        self.assertEqual(self.a.state, 'done')
        self.assertEqual(sequence.position, 3750)

    def test_spacing_ignores_late_vision_and_stop_cancels_following_points(self):
        self.calibrated_one()
        self.until(lambda:self.a.ring_sequence.phase == 'spacing')
        self.a.results.put((self.a.generation-1, None, None, 'retired OCR stream'))
        self.step()
        self.assertTrue(self.a.active, self.a.error)
        self.c.submit('STOP'); self.flush()
        before = len(self.link.sent)
        for _ in range(100): self.step()
        self.assertEqual(self.a.state, 'stopped')
        self.assertEqual(len(self.a.ring_sequence.recognized), 1)
        self.assertFalse(any(c.startswith('CHASSIS STRAIGHT') for c in self.link.sent[before:]))

    def test_spacing_does_not_accept_wrong_id_or_partial_completion(self):
        for changes in ({'move_id':'other'}, {'remaining':'1'}):
            with self.subTest(changes=changes):
                self.calibrated_one()
                sequence = self.a.ring_sequence
                self.until(lambda:sequence.phase == 'spacing')
                fields = dict(state='done', active='0', move_id=sequence.operation_id, remaining='0')
                fields.update(changes)
                sequence.reply('ring_status', fields)
                self.flush()
                self.assertEqual(self.a.state, 'fault')
                self.assertEqual(len(sequence.recognized), 1)
                self.c.submit('ARM 1'); self.flush()

    def test_stale_frame_and_single_rings_do_not_form_a_line(self):
        self.start()
        self.sample(angle=12, captured_monotonic=self.clock[0]-1)
        self.assertIsNone(self.a.ring_sequence.operation)
        self.sample(rings=[[100, 200]])
        self.assertEqual(self.a.ring_sequence.operation[0], 'move')
        self.assertIsNone(self.a.ring_sequence.line)
        self.complete_motion()
        self.sample(rings=[[400, 250]])
        self.assertFalse(any(cmd.startswith('TURN MOVE') for cmd in self.link.sent))

    def test_stale_ocr_breaks_consecutive_confirmation(self):
        self.start(); self.align()
        self.sample(digit='1'); self.sample(digit='1')
        self.sample(digit='1', captured_monotonic=self.clock[0]-1)
        self.sample(digit='1')
        self.assertEqual(self.a.ring_sequence.expected, 1)
        self.assertEqual(self.a.ring_sequence.stable, 1)

    def test_stop_cancels_search_and_delayed_completion(self):
        self.start(); self.sample()
        self.assertFalse(self.c.submit('CHASSIS MOVE FWD 100 300 6000')[0])
        self.c.submit('STOP'); self.flush()
        self.assertEqual(self.a.state, 'stopped')
        self.assertFalse(self.board.armed)
        before = len(self.link.sent)
        self.sample(angle=0)
        self.assertFalse(any(cmd.startswith(('CHASSIS STRAIGHT', 'TURN MOVE')) for cmd in self.link.sent[before:]))

    def test_wrong_operation_or_imu_failure_stops(self):
        self.start(); self.sample()
        sequence = self.a.ring_sequence
        sequence.reply('ring_status', dict(state='done', active='0', move_id='other'))
        self.flush()
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(self.board.armed)

    def test_digit_search_exhausts_its_own_x_limit_without_skipping_one(self):
        self.start(extent=150); self.align()
        for _ in range(5): self.sample(digit='2')
        self.complete_motion()
        self.assertEqual(self.a.ring_sequence.position, -100)
        for _ in range(5): self.sample()
        self.complete_motion()
        self.assertEqual(self.a.ring_sequence.position, -150)
        for _ in range(5): self.sample(digit='3')
        self.flush()
        self.assertEqual(self.a.state, 'fault')
        self.assertEqual(self.a.ring_sequence.recognized, [])
        self.assertFalse(self.board.armed)

    def test_lost_gyro_during_search_stops_instead_of_accepting_completion(self):
        self.start(); self.sample()
        self.board.navigation.imu_valid = False
        self.until(lambda:self.a.state == 'fault', limit=100)
        self.assertFalse(self.board.armed)

    def test_no_fresh_camera_result_times_out_before_starting_search(self):
        self.start()
        self.clock[0] += 3.1
        self.a.tick(); self.flush()
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(any(cmd.startswith('CHASSIS STRAIGHT') for cmd in self.link.sent))

    def test_two_circles_during_search_stop_the_slice_before_any_turn(self):
        self.start(extent=2000)
        self.a.ring_sequence.config['scan_step'] = 1000
        self.sample()
        for _ in range(3): self.sample(angle=12)
        self.assertIn('CHASSIS HALT', self.link.sent)
        self.assertFalse(any(cmd.startswith('TURN MOVE') for cmd in self.link.sent))
        self.complete_motion()
        self.assertLess(abs(self.a.ring_sequence.position), 1000)
        for _ in range(3): self.sample(angle=12)
        self.assertTrue(any(cmd.startswith('TURN MOVE') for cmd in self.link.sent))

    def test_one_requires_approach_before_the_next_digit(self):
        self.start(); self.align()
        for _ in range(3): self.sample(digit='1')
        self.assertEqual(self.a.ring_sequence.expected, 1)
        self.assertEqual(self.a.ring_sequence.phase, 'approach')
        self.assertEqual(self.a.ring_sequence.recognized, [])

    def test_digit_seen_mid_search_waits_for_idle_before_servo_handoff(self):
        self.start(extent=2000); self.align()
        self.a.ring_sequence.config['scan_step']=1000
        for _ in range(5): self.sample()
        self.assertEqual(self.a.ring_sequence.operation,('move',-1000))
        before=len(self.link.sent)
        for _ in range(3): self.sample(digit='1')
        self.assertIn('CHASSIS HALT',self.link.sent[before:])
        self.assertNotIn('TURN ZERO',self.link.sent[before:])
        self.assertFalse(any(cmd.startswith('WHEELS ') for cmd in self.link.sent[before:]))
        self.complete_motion()
        self.assertFalse(any(axis['active'] for axis in self.board.axes[:4]))
        for _ in range(3): self.sample(digit='1')
        self.assertEqual(self.a.ring_sequence.phase,'approach')
        self.sample(digit='1',error_dx=100)
        commands=self.link.sent[before:]
        self.assertLess(commands.index('CHASSIS HALT'),commands.index('TURN ZERO'))
        self.assertLess(commands.index('TURN ZERO'),next(i for i,c in enumerate(commands) if c.startswith('WHEELS ')))
        self.assertTrue(self.a.active,self.a.error)

    def test_lost_line_after_turn_restarts_bounded_search(self):
        self.start(extent=800)
        for _ in range(3): self.sample(angle=12)
        self.complete_motion()
        for _ in range(3): self.sample()
        self.assertEqual(self.a.ring_sequence.phase, 'recover')
        self.assertEqual(self.a.ring_sequence.operation, ('move', -100))
        self.assertFalse(any(call.kwargs.get('phase') for call in self.a.hub.start_tracking.call_args_list))

    def test_lost_line_during_turn_halts_before_recovery(self):
        self.start(extent=800)
        for _ in range(3): self.sample(angle=30)
        for _ in range(3): self.sample()
        self.assertIn('CHASSIS HALT', self.link.sent)
        self.complete_motion()
        self.assertEqual(self.a.ring_sequence.phase, 'recover')
        self.assertFalse(any(axis['active'] for axis in self.board.axes[:4]))

    def test_turn_continues_capturing_and_one_missing_frame_does_not_restart(self):
        self.start(extent=800)
        for _ in range(3): self.sample(angle=30)
        self.a.capturing = False
        self.step()
        self.assertTrue(self.a.capturing)
        self.sample(); self.sample(angle=20); self.sample(); self.sample()
        self.assertEqual(self.a.ring_sequence.phase, 'align')
        self.assertNotIn('CHASSIS HALT', self.link.sent)

    def test_recovery_exhaustion_stops_within_its_range(self):
        self.start(extent=150)
        for _ in range(3): self.sample(angle=12)
        self.complete_motion()
        for _ in range(3): self.sample()
        sequence = self.a.ring_sequence
        for _ in range(30):
            if not self.a.active: break
            if sequence.operation: self.complete_motion()
            self.assertLessEqual(abs(sequence.position), 150)
            self.sample()
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(self.board.armed)
        self.assertEqual(sequence.expected, 1)

    def test_stopped_forward_sighting_also_requires_500_and_stop_cancels_it(self):
        self.start(extent=800)
        for _ in range(3): self.sample(angle=12)
        self.complete_motion()
        for _ in range(3): self.sample()
        self.complete_motion()
        sequence = self.a.ring_sequence
        while sequence.position > -800:
            self.sample(); self.complete_motion()
        self.sample(); self.complete_motion()
        for _ in range(3): self.sample(angle=0)
        self.assertEqual(sequence.phase, 'advance')
        self.assertEqual(sequence.operation, ('move', 500))
        self.c.submit('STOP'); self.flush()
        self.assertEqual(self.a.state, 'stopped')
        self.assertFalse(self.board.armed)

    def test_forward_reacquisition_advances_500_before_fresh_alignment(self):
        self.start(extent=800)
        for _ in range(3): self.sample(angle=12)
        self.complete_motion()
        for _ in range(3): self.sample()
        sequence = self.a.ring_sequence
        self.assertEqual(sequence.phase, 'recover')
        # Seeing the row while backing up must not resume alignment.
        for _ in range(3): self.sample(angle=12)
        self.assertNotIn('CHASSIS HALT', self.link.sent)
        self.complete_motion()
        while sequence.position > -800:
            self.sample(); self.complete_motion()
        self.sample()
        self.assertEqual(sequence.operation, ('move', 100))
        for _ in range(3): self.sample(angle=12)
        self.assertIn('CHASSIS HALT', self.link.sent)
        self.complete_motion()
        before = sequence.position
        self.until(lambda:sequence.operation == ('move', 500))
        # The clearance move cannot be shortened by double-circle detections.
        for _ in range(3): self.sample(angle=0)
        self.assertEqual(sequence.phase, 'advance')
        self.complete_motion()
        self.assertEqual(sequence.position-before, 500)
        self.assertEqual(sequence.phase, 'align')
        self.assertEqual(sequence.expected, 1)
        for _ in range(3): self.sample(angle=8)
        self.assertEqual(sequence.operation, ('turn', 800))

    def test_old_stream_error_does_not_fault_a_new_digit_stream(self):
        self.start()
        old_generation = self.a.generation
        self.align()
        self.a.results.put((old_generation, None, None, 'old stream stopped'))
        self.step()
        self.assertTrue(self.a.active, self.a.error)
        self.assertTrue(self.a.capturing)

    def test_one_approach_tracks_pixels_and_brakes_on_wrong_digit(self):
        self.start(); self.align()
        for _ in range(3): self.sample(digit='1')
        self.sample(digit='1', error_dx=100, error_dy=0)
        self.assertGreater(self.a.vy, 0)
        self.assertTrue(any(self.a.wheels))
        self.sample(digit='2', error_dx=100)
        self.assertEqual((self.a.vx, self.a.vy, self.a.yaw), (0, 0, 0))
        self.assertEqual(self.a.ring_sequence.expected, 1)
        self.assertEqual(self.a.ring_sequence.recognized, [])
        for _ in range(65):
            if not self.a.active: break
            self.sample(digit='2')
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(self.board.armed)

    def test_one_approach_converges_using_real_simulated_wheel_displacement(self):
        self.start(); self.align()
        for _ in range(3): self.sample(digit='1')
        error = [90., -40.]
        positions = [axis['pos'] for axis in self.board.axes[:4]]
        for index in range(1000):
            if index % 6 == 0:
                self.sample(digit='1', error_dx=error[0], error_dy=error[1])
            else:
                self.step()
            current = [axis['pos'] for axis in self.board.axes[:4]]
            physical = [(v-p)*(-1 if self.board.invert & (1<<i) else 1)
                        for i,(v,p) in enumerate(zip(current,positions))]
            positions = current
            right = (-physical[0]+physical[1]+physical[2]-physical[3])/4
            forward = sum(physical)/4
            # A +90 degree camera sees body forward as camera right.
            error[0] -= forward*.025
            error[1] -= right*.025
            if self.a.ring_sequence.phase == 'dwell': break
            self.assertNotEqual(self.a.state, 'fault', self.a.error)
        self.assertEqual(self.a.ring_sequence.phase, 'dwell', self.a.snapshot())
        self.assertLessEqual(max(map(abs,error)), 5)
        self.assertTrue(self.a.ring_sequence.recognized[0]['aligned'])
        self.assertFalse(any(axis['active'] for axis in self.board.axes[:4]))


if __name__ == '__main__':
    unittest.main()
