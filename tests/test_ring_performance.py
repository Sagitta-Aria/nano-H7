"""Ring geometry and asynchronous multi-target OCR regression, without hardware."""
import importlib
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

ring = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.black_ring')
digit = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.ring_digit')
core = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.digit_core')


class ImmediatePool:
    """Complete inference now while the camera consumes results on its next frame."""
    def submit(self, function, *args):
        result = Future()
        result.set_result(function(*args))
        return result

    def shutdown(self, **kwargs):
        pass


class RingPerformanceTests(unittest.TestCase):
    def test_spawn_discovery_returns_geometry_and_shuts_down(self):
        """The real spawn boundary must work without pickling the pool or GPU state."""
        from host.ring_detector import RingDetector
        from host.vision_tuning import defaults
        settings = defaults()
        settings['ring']['min_area'] = 300
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.circle(image, (160,120), 60, (20,20,20), 6)
        detector = RingDetector()
        try:
            detector.detect(image, settings)
            signature, rings = detector.future.result(timeout=15)
            self.assertEqual(signature, detector.signature)
            self.assertEqual(len(rings), 1)
            self.assertLess(np.linalg.norm(np.asarray(rings[0]['center'])-(160,120)), 3)
            self.assertEqual(detector.detect(np.full_like(image,220), settings)[0], [])
        finally:
            detector.close()

    def test_roi_tracking_follows_motion_and_discovers_additional_ring(self):
        """Moving centers use current pixels; background finds targets outside ROIs."""
        from host.ring_detector import RingDetector
        from host.vision_tuning import defaults
        settings = defaults()
        settings['ring'].update(min_area=300, detect_width=800)
        image = np.full((480, 1024, 3), 220, np.uint8)
        cv2.circle(image, (200, 240), 60, (20, 20, 20), 6)
        with patch('host.ring_detector.ProcessPoolExecutor', return_value=ImmediatePool()):
            detector = RingDetector()
            self.assertEqual(len(detector.detect(image, settings)[0]), 1)
            moved = np.full_like(image, 220)
            cv2.circle(moved, (212, 244), 60, (20, 20, 20), 6)
            cv2.circle(moved, (800, 240), 60, (20, 20, 20), 6)
            detector.next_scan = 0
            current, _ = detector.detect(moved, settings)
            self.assertLess(np.linalg.norm(np.asarray(current[0]['center'])-(212,244)), 3)
            current, _ = detector.detect(moved, settings)
            self.assertEqual(len(current), 2)
            self.assertLess(np.linalg.norm(np.asarray(current[1]['center'])-(800,240)), 3)
            self.assertEqual(detector.detect(np.full_like(image, 220), settings)[0], [])
            detector.close()

    def test_background_hint_is_revalidated_and_config_change_invalidates_it(self):
        """A stale discovery can locate an ROI but cannot report an absent ring."""
        from host.ring_detector import RingDetector
        from host.vision_tuning import defaults
        settings = defaults()
        settings['ring'].update(min_area=300, detect_width=800)
        image = np.full((360, 640, 3), 220, np.uint8)
        cv2.ellipse(image, (300,180), (80,60), 0, 20, 350, (20,20,20), 7)
        with patch('host.ring_detector.ProcessPoolExecutor', return_value=ImmediatePool()):
            detector = RingDetector()
            detector.detect(image, settings)
            targets, _ = detector.detect(image, settings)
            self.assertEqual(len(targets), 1)
            self.assertLess(np.linalg.norm(np.asarray(targets[0]['center'])-(300,180)), 3)
            blank = np.full_like(image,220)
            self.assertEqual(detector.detect(blank, settings)[0], [])
            settings['ring']['min_area'] = 100000
            self.assertEqual(detector.detect(image, settings)[0], [])
            detector.close()

    def test_thin_digit_compensation_requires_two_strong_model_results(self):
        """One confident alternate view must never manufacture a confirmed digit."""
        from unittest.mock import Mock
        crop = np.full((120, 25, 3), 255, np.uint8)
        with patch.object(core, 'prepare_digit', return_value=crop):
            engine = Mock(side_effect=[(None, 0), ('1', .8), ('1', .9)])
            self.assertEqual(core.recognize(engine, crop)[0], '1')
            engine = Mock(side_effect=[(None, 0), ('1', .8)] + [(None, 0)]*6)
            self.assertIsNone(core.recognize(engine, crop)[0])

    def test_ocr_ignores_small_flecks_but_keeps_both_digits(self):
        """Small print debris must not widen 1; a second full digit must survive."""
        image = np.full((192, 192, 3), 255, np.uint8)
        cv2.rectangle(image, (90, 35), (101, 160), (0, 0, 0), -1)
        clean = core.prepare_digit(image)
        cv2.rectangle(image, (48, 65), (52, 90), (0, 0, 0), -1)
        cv2.rectangle(image, (59, 65), (63, 90), (0, 0, 0), -1)
        np.testing.assert_array_equal(core.prepare_digit(image), clean)
        cv2.rectangle(image, (135, 35), (146, 160), (0, 0, 0), -1)
        self.assertGreater(core.prepare_digit(image).shape[1], clean.shape[1] * 2)

    def test_broken_inner_ring_is_removed_from_digit_crop(self):
        """An open inner ring must not become OCR foreground alongside the digit."""
        image = np.full((260, 260, 3), 210, np.uint8)
        cv2.circle(image, (130, 130), 100, (15, 15, 15), 5)
        cv2.ellipse(image, (130, 130), (60, 60), 0, 20, 350, (15, 15, 15), 5)
        cv2.putText(image, '2', (113, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (15, 15, 15), 3)
        inner = cv2.ellipse2Poly((130, 130), (96, 96), 0, 0, 360, 2).reshape(-1, 1, 2)
        crop = digit.extract_ring_center(image, dict(inner=inner))
        yy, xx = np.indices(crop.shape[:2])
        radius = np.hypot(xx - 96, yy - 96)
        dark = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) < 100
        self.assertLess(np.count_nonzero(dark & (radius > 65)), 100)
        self.assertGreater(np.count_nonzero(dark & (radius < 65)), 200)

    def test_all_visible_digits_confirm_at_two_fps_without_stale_reuse(self):
        """Two fresh rings must accumulate votes even when geometry takes 500 ms."""
        args = SimpleNamespace(digit_confirm=3, digit_window=7, digit_hold=.6,
                               digit_roi_scale=.75, confidence=.6, confidence_one=.3,
                               digit_interval=.06)
        targets = [dict(id=n, center=(n*150, 100), radius=50, area=7800)
                   for n in (1, 2)]
        with patch.object(digit, 'ThreadPoolExecutor', return_value=ImmediatePool()), \
             patch.object(digit, 'extract_ring_center', side_effect=lambda frame, item, scale: str(item['id'])), \
             patch.object(digit, 'recognize', side_effect=lambda engine, crop, *args: (crop, .95, 10)), \
             patch.object(digit.time, 'monotonic') as clock:
            worker = digit.RingDigitRecognizer(args, engine=object())
            for step in range(8):
                clock.return_value = step * .5
                reports = worker.update(None, targets)
            self.assertEqual([reports[n]['digit'] for n in (1, 2)], ['1', '2'])
            clock.return_value = 3.6
            self.assertEqual(worker.update(None, []), {})
            clock.return_value = 3.7
            reports = worker.update(None, targets)
            self.assertTrue(all(item['digit'] is None for item in reports.values()))
            worker.close()

    def test_late_ocr_batch_cannot_become_fresh_evidence(self):
        """Results from frames older than the hold limit are discarded."""
        args = SimpleNamespace(digit_confirm=1, digit_window=3, digit_hold=.6,
                               digit_roi_scale=.75, confidence=.6, confidence_one=.3,
                               digit_interval=.06)
        targets = [dict(id=1, center=(100, 100), radius=50, area=7800)]
        with patch.object(digit, 'ThreadPoolExecutor', return_value=ImmediatePool()), \
             patch.object(digit, 'extract_ring_center', return_value='1'), \
             patch.object(digit, 'recognize', return_value=('1', .95, 10)), \
             patch.object(digit.time, 'monotonic') as clock:
            worker = digit.RingDigitRecognizer(args, engine=object())
            clock.return_value = 0
            worker.update(None, targets)
            clock.return_value = 1
            self.assertIsNone(worker.update(None, targets)[1]['digit'])
            worker.close()

    def test_concentric_layer_switch_preserves_track_but_not_distant_target(self):
        """A ring layer changes area, while a genuinely new center resets votes."""
        args = SimpleNamespace(digit_confirm=3, digit_window=7, digit_hold=.6)
        with patch.object(digit, 'ThreadPoolExecutor', return_value=ImmediatePool()):
            worker = digit.RingDigitRecognizer(args, engine=object())
            initial = worker._associate([dict(id=1,center=(100,100),radius=100,area=31000)])
            inner = worker._associate([dict(id=1,center=(108,103),radius=65,area=13000)])
            self.assertEqual(initial, inner)
            moved = worker._associate([dict(id=1,center=(250,100),radius=65,area=13000)])
            self.assertNotEqual(inner, moved)
            worker.close()

    def test_broken_rings_small_targets_and_solid_rejection(self):
        """Keep the edge fallback on broken rings and do not promote solid discs."""
        image = np.full((720, 1280, 3), 220, np.uint8)
        cv2.ellipse(image, (250, 300), (100, 80), 18, 20, 350, (20, 20, 20), 10)
        cv2.circle(image, (600, 330), 24, (20, 20, 20), 5)
        cv2.circle(image, (950, 330), 75, (20, 20, 20), -1)
        for width in (640, 960):
            targets, _ = ring.analyze_ring_frame(image, min_area=300, detect_width=width)
            centers = np.array([target['center'] for target in targets])
            self.assertEqual(len(centers), 2)
            for expected in ((250, 300), (600, 330)):
                self.assertLessEqual(np.linalg.norm(centers - expected, axis=1).min(), 3)


if __name__ == '__main__':
    unittest.main()
