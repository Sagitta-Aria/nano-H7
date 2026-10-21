"""No camera/COM access: synthetic color images and real protocol/SIM barriers."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from host.camera_vision import DEFAULT as VISION_DEFAULT, detect
from host.pick_place import DEFAULT, PickPlaceSettings, stacked_config
from host.stack_ring import StackRingSequence
from tests import test_ring_sequence as sequence_fixture
from tests import test_camera_servo as servo_fixture


class StackSettingsTests(unittest.TestCase):
    def test_old_file_is_preserved_and_requires_explicit_calibration(self):
        old = dict(DEFAULT)
        old.pop('d_second_drop_down')
        old['place_drop_down'] = 8123
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'pick.json'
            path.write_text(json.dumps(old), encoding='utf-8')
            raw = path.read_bytes()
            settings = PickPlaceSettings(path)
            self.assertEqual(settings.value['place_drop_down'], 8123)
            self.assertEqual(settings.value['d_second_drop_down'], -1)
            self.assertEqual(path.read_bytes(), raw)
            with self.assertRaises(ValueError):
                stacked_config(settings.snapshot())
            settings.save(dict(settings.snapshot(), d_second_drop_down=6100))
            self.assertEqual(stacked_config(PickPlaceSettings(path).snapshot())['place_drop_down'], 6100)
            self.assertEqual(settings.value['place_drop_down'], 8123)
        for depth in (-1, 8123, 9000, True, 2.5):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                stacked_config(dict(old, d_second_drop_down=depth))


@unittest.skipUnless(sequence_fixture.importlib.util.find_spec('cv2'), 'OpenCV unavailable')
class StackPerceptionTests(unittest.TestCase):
    def image_report(self, colors, layout, expected=None):
        """Supply actual ring contours, with offset block tops inside each hole."""
        import cv2
        import numpy as np
        from host import vision_tuning
        image = np.full((480, 640, 3), 255, np.uint8)
        rings = []
        palette = {'red':(0,0,255), 'blue':(255,0,0), 'yellow':(0,255,255),
                   'black':(0,0,0), None:None}
        for index, color in enumerate(colors):
            center = (320, 350-index*200)
            cv2.circle(image, center, 65, (0,0,0), 6)
            contour = cv2.ellipse2Poly(center, (60,60), 0, 0, 360, 5).reshape(-1,1,2)
            rings.append(dict(center=center, area=1000, inner=contour, outer=contour))
            if color:
                cv2.rectangle(image, (center[0]-12,center[1]-23),
                              (center[0]+28,center[1]+17), palette[color], -1)
        cfg = copy.deepcopy(VISION_DEFAULT)
        cfg.update(kind='ring', stack_layout={str(p):dict(color=c) for p,c in layout.items()},
                   stack_expected=expected)
        cfg['tracking']['camera_yaw_deg'] = 0
        cfg['detector'] = vision_tuning.legacy_settings(100, 140)
        ocr = Mock()
        report = detect(image, cfg, 10., ocr=ocr,
                        ring_detector=lambda *args:(rings, None))
        ocr.select.assert_not_called()
        return report

    def test_single_unique_color_uses_ring_center_and_never_ocr(self):
        report = self.image_report(['red'], {1:'red',2:'blue',3:'yellow'})
        self.assertEqual(report['anchor_point'], 1)
        self.assertEqual(report['center'], [320,350])
        self.assertIsNone(report['digit'])
        self.assertEqual(report['digits'], [])
        self.assertIsNone(report['row_line'])

    def test_repeated_color_requires_unique_ordered_pair(self):
        layout = {1:'red',2:'blue',3:'red'}
        self.assertFalse(self.image_report(['red'], layout)['found'])
        report = self.image_report(['red','blue'], layout, expected=2)
        self.assertEqual(report['anchor_point'], 2)
        self.assertEqual([o['point'] for o in report['color_rings']], [1,2])
        self.assertFalse(self.image_report(['red','red'], {1:'red',2:'red',3:'red'})['found'])

    def test_black_rim_does_not_create_black_block_identity(self):
        layout = {1:'black',2:'blue',3:'red'}
        self.assertFalse(self.image_report([None], layout)['found'])
        self.assertEqual(self.image_report(['black'], layout)['anchor_point'], 1)
        self.assertFalse(self.image_report(['yellow'], layout)['found'])

    def test_expected_point_is_not_invented_from_other_color(self):
        self.assertFalse(self.image_report(['red'], {1:'red',2:'blue',3:'yellow'}, expected=2)['found'])


class StackSequenceTests(unittest.TestCase):
    setUp = servo_fixture.ServoTests.setUp
    send = servo_fixture.ServoTests.send
    flush = servo_fixture.ServoTests.flush
    step = servo_fixture.ServoTests.step
    until = servo_fixture.ServoTests.until
    complete_motion = sequence_fixture.SequenceTests.complete_motion

    def start(self):
        """Use the real camera owner/serial controller with a simulated held visit."""
        self.a.settings.value['kind'] = 'ring'
        self.a.settings.value['ring_sequence'].update(range_steps=100, scan_step=50, sps=2000)
        self.a.start()
        self.a.task_options = dict(anchor_only=True, material_offsets={1:0,2:180,3:390},
            stack_layout={1:dict(color='red'),2:dict(color='blue'),3:dict(color='yellow')})
        self.a.config['stack_layout'] = self.a.task_options['stack_layout']
        self.a.ring_sequence = StackRingSequence(self.a)
        self.flush()
        self.until(lambda:self.a.state == 'ring_sequence')
        for _ in range(3):
            self.sample(angle=0)
        self.assertEqual(self.a.ring_sequence.phase, 'seek')

    def sample(self, point=None, angle=None, **extra):
        sequence_fixture.SequenceTests.sample(self, angle=angle, digit=None,
            found=point is not None, anchor_point=point, identity_source='remembered_color', **extra)

    def anchor(self, point):
        for _ in range(25):
            self.sample(point)
            if self.a.ring_sequence.phase in ('transfer', 'complete'):
                return
        self.fail(self.a.snapshot())

    def test_one_ring_anchor_transfer_and_fresh_middle_confirmation(self):
        self.start()
        self.anchor(1)
        self.assertEqual(self.a.ring_sequence.phase, 'transfer')
        self.assertIn('CHASSIS STRAIGHT FWD 180 2000 6000', self.link.sent)
        self.complete_motion()
        self.assertEqual(self.a.ring_sequence.phase, 'seek')
        self.assertTrue(self.a.ring_sequence.confirming_middle)
        # Other point after the finite move cannot be accepted as middle.
        for _ in range(3):self.sample(3)
        self.assertEqual(self.a.ring_sequence.phase, 'seek')
        self.anchor(2)
        self.assertEqual(self.a.state, 'done')
        self.assertEqual([r['point'] for r in self.a.ring_sequence.recognized], [1,2])
        phases = [call.kwargs.get('phase') for call in self.a.hub.start_tracking.call_args_list]
        self.assertFalse(any(p in ('1','2','3') for p in phases))

    def test_ambiguous_identity_searches_both_directions_then_stops(self):
        self.start()
        positions = []
        for _ in range(60):
            self.sample()
            if not self.a.active:break
            self.complete_motion()
            positions.append(self.a.ring_sequence.position)
        self.assertEqual(self.a.state, 'fault')
        self.assertIn(-100, positions)
        self.assertIn(100, positions)
        self.assertFalse(self.a.ring_sequence.recognized)

    def test_stale_and_changing_identity_cannot_release_approach(self):
        self.start()
        for point in (1,2,1,2,1,2):
            self.sample(point)
        self.assertEqual(self.a.ring_sequence.phase, 'seek')
        self.sample(2, captured_monotonic=self.clock[0]-2)
        self.assertEqual(self.a.ring_sequence.stable, 0)
        self.assertFalse(self.a.ring_sequence.recognized)

    def test_changed_transfer_move_id_faults(self):
        self.start()
        self.anchor(3)
        self.assertIn('CHASSIS STRAIGHT BACK 210 2000 6000', self.link.sent)
        self.board.chassis_id += 1
        for _ in range(10):
            self.step()
            if self.a.state == 'fault':break
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(self.board.armed)
        self.assertNotEqual(self.a.ring_sequence.phase, 'complete')

    def test_incomplete_transfer_cannot_confirm_middle(self):
        self.start()
        self.anchor(1)
        sequence = self.a.ring_sequence
        self.assertIsNotNone(sequence.operation)
        sequence.reply('ring_status', dict(state='done', active='0', remaining='1',
            move_id=sequence.operation_id, fault='none'))
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(sequence.confirming_middle)
        self.assertEqual([item['point'] for item in sequence.recognized], [1])

    def test_stop_during_transfer_cannot_resume_from_late_frames(self):
        self.start()
        self.anchor(1)
        self.a.stop('操作员停止')
        for _ in range(5):
            self.sample(2)
            self.step()
        self.assertEqual(self.a.state, 'stopped')
        self.assertFalse(self.a.active)
        self.assertFalse(self.board.armed)
        self.assertEqual([item['point'] for item in self.a.ring_sequence.recognized], [1])


if __name__ == '__main__':
    unittest.main()
