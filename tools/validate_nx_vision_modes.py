"""Exercise real NX mode transitions, GPU OCR and inactive-detector isolation."""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def main():
    """Use a physical digit fixture and synthetic color target without a camera."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True)
    parser.add_argument('--real-one', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    sys.path.insert(0, args.source)
    import nx_vision_live as live
    cv2.setNumThreads(1)
    options = live.parse_args([args.source, '--mode', 'color'])
    runtime = live.VisionModes(options)
    evidence = {}

    def forbidden(*unused, **kwargs):
        """Make accidental execution of the inactive algorithm fail the check."""
        raise AssertionError('Inactive detector was called')

    def confirm_one(frame):
        """Feed enough independent GPU results to exercise original voting."""
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            runtime.process(frame)
            if runtime.digit == '1':
                return
            if runtime.future is not None:
                runtime.future.result()
            time.sleep(.035)
        raise AssertionError('Real digit 1 failed to stabilize')

    try:
        assert runtime.engine is None and runtime.pool is None
        assert 'nx_vision_trt' not in sys.modules and 'ring_digit' not in sys.modules
        frame = np.full((480, 640, 3), 210, np.uint8)
        cv2.circle(frame, (333, 273), 28, (0, 210, 0), -1, cv2.LINE_AA)
        before = frame.copy()
        runtime.process(frame)
        first = runtime.report(frame)
        assert first['color_block']['color'] == 'green'
        assert (first['dx'], first['dy'], first['error_dx'], first['error_dy']) == (13, 33, 13, 3)
        assert first['ring_frames'] == first['ocr_submitted'] == 0
        assert first['roi'] == (140, 60, 360, 330)
        runtime.annotate(frame)
        assert np.array_equal(frame, before)
        evidence['cold_color_no_ocr'] = first
        runtime.process(np.full_like(frame, 210))
        assert runtime.color is None and runtime.report(frame)['center'] is None

        color_detector = runtime.color_module.find_color_block
        runtime.color_module.find_color_block = forbidden
        runtime.switch('digit')
        real = cv2.imread(args.real_one)
        assert real is not None
        confirm_one(real)
        evidence['digit_only'] = runtime.report(real)
        assert runtime.color_frames == 0 and runtime.ocr_submitted >= options.confirm
        assert runtime.color is None
        engine = runtime.engine
        runtime._drain()
        runtime.next_run = 0
        runtime.process(real)
        pending = runtime.future
        assert pending is not None
        runtime.switch('color')
        assert pending.done() and runtime.future is None
        assert runtime.digit is None and runtime.crop is None and runtime.ring is None
        assert runtime.target.value() is None and runtime.target.candidate is None
        runtime.color_module.find_color_block = color_detector
        ring_detector, extract, recognize = runtime.detect_ring, runtime.extract, runtime.recognize
        runtime.detect_ring = runtime.extract = runtime.recognize = forbidden
        for _ in range(10):
            runtime.process(frame)
        evidence['color_after_digit'] = runtime.report(frame)
        assert runtime.ring_frames == runtime.ocr_submitted == 0
        assert runtime.color_frames == 10 and runtime.color['color'] == 'green'
        assert runtime.engine is engine
        runtime.detect_ring, runtime.extract, runtime.recognize = ring_detector, extract, recognize
        runtime.switch('digit')
        assert runtime.digit is None and runtime.ring_frames == runtime.color_frames == 0
        assert runtime.engine is engine
        runtime.color_module.find_color_block = forbidden
        confirm_one(real)
        evidence['digit_reentry'] = runtime.report(real)
        assert runtime.color_frames == 0
    finally:
        runtime.close()
    evidence['passed'] = True
    Path(args.output).write_text(json.dumps(evidence, indent=2)+'\n')
    print(json.dumps(evidence), flush=True)


if __name__ == '__main__':
    main()
