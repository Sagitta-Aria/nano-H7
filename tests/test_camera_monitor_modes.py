"""Ensure mode switches cannot mix detector calls, results or FPS windows."""
import copy
import unittest
from unittest.mock import Mock, patch

import numpy as np

from host.camera_monitor import RecognitionMonitor
from host.camera_vision import DEFAULT


class MonitorModeTests(unittest.TestCase):
    def test_idle_color_respects_selected_target_and_never_falls_back_to_black(self):
        """Target red with a black distractor must either find red or report missing."""
        import cv2
        from host.vision_tuning import legacy_settings
        config = copy.deepcopy(DEFAULT)
        config['detector'] = legacy_settings(500, 140)
        monitor = RecognitionMonitor()
        monitor.select('color')
        monitor.controls.compose = lambda image, mode: image
        monitor.controls.take_mode = lambda: None
        try:
            for name, bgr in [('red', (0,0,255)), ('green', (0,255,0)),
                              ('blue', (255,0,0)), ('yellow', (0,255,255)),
                              ('light_blue', (255,255,0)), ('black', (0,0,0)), (None, None)]:
                with self.subTest(color=name):
                    config['target'] = name or 'red'
                    frame = np.full((240,320,3),255,np.uint8)
                    if name != 'black': cv2.rectangle(frame,(10,10),(70,70),(0,0,0),-1)
                    if bgr is not None: cv2.rectangle(frame,(100,80),(180,160),bgr,-1)
                    monitor.process(frame, config)
                    self.assertEqual(monitor.report['color'], name)
                    self.assertEqual(monitor.report['found'], name is not None)
                    with patch('cv2.putText', wraps=cv2.putText) as text:
                        monitor.draw(frame)
                    self.assertTrue(any('Color: '+(name or '--') in call.args[1] for call in text.call_args_list))
            self.assertEqual(config['target'], 'red')
            config['detector']['colors']['red']['min_area'] = 100000
            monitor.process(frame, config)
            self.assertFalse(monitor.report['found'])
        finally:
            monitor.close()

    def test_color_switch_resets_slow_digit_fps_and_skips_ring_and_ocr(self):
        """At a 30 Hz color input, old 2 Hz digit completions must not lower FPS."""
        frame = np.full((240,320,3),255,np.uint8)
        config = copy.deepcopy(DEFAULT)
        config['kind'] = 'ring'
        detector, ocr = Mock(), Mock()
        ocr.select.return_value = None
        ocr.observations = []
        detector.detect.return_value = ([], None)
        with patch('host.ring_detector.RingDetector', return_value=detector), \
             patch('host.ring_perception.PreviewOCR', return_value=ocr), \
             patch('host.camera_monitor.time.monotonic', return_value=10.) as clock:
            monitor = RecognitionMonitor()
            try:
                monitor.process(frame, config)
                clock.return_value = 10.5
                monitor.process(frame, config)
                detector.detect.reset_mock(); ocr.select.reset_mock()
                monitor.select('color')
                for value in (11., 11.+1/30, 11.+2/30):
                    clock.return_value = value
                    monitor.process(frame, config)
                self.assertAlmostEqual(monitor.report['fps'], 30., places=5)
                detector.detect.assert_not_called()
                ocr.select.assert_not_called()
                self.assertEqual(monitor.report['mode'], 'color')
                self.assertNotIn('digits', monitor.report)
                self.assertEqual(config['kind'], 'ring')
            finally:
                monitor.close()

    def test_color_only_does_not_load_digit_engine(self):
        """Opening color mode must not start expensive GPU model initialization."""
        with patch('host.ring_perception.PreviewOCR') as create_ocr:
            monitor = RecognitionMonitor()
            try:
                monitor.select('color')
                monitor.process(np.full((240,320,3),255,np.uint8),copy.deepcopy(DEFAULT))
                create_ocr.assert_not_called()
                self.assertIsNone(monitor.detector.future)
            finally:
                monitor.close()

if __name__ == '__main__':
    unittest.main()
