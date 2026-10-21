"""Validate shared ROI/color/grasp geometry and ring+GPU OCR coexistence on NX.

Run with vision-env and the deployed demo sources. Generated colors verify
thresholds and mapping; physical camera lighting still needs user testing.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def main():
    """Replay six-color and boundary cases, then process both targets in one frame."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--real-ring', nargs='*', default=[])
    args = parser.parse_args()
    sys.path.insert(0, args.source)
    import nx_vision_color as color
    import nx_vision_fast as fast
    import nx_vision_roi as roi
    from nx_vision_trt import accelerate
    import digit_core
    cv2.setNumThreads(1)

    def canvas():
        """Create a neutral camera-sized background outside all color thresholds."""
        return np.full((480, 640, 3), 210, np.uint8)

    def bgr(hsv):
        """Convert an independent test HSV sample into a real BGR camera pixel."""
        return tuple(int(v) for v in cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0])

    cases = []
    samples = [('red', (0, 220, 220)), ('red', (175, 220, 220)),
               ('green', (60, 220, 220)), ('blue', (115, 220, 220)),
               ('blue', (115, 100, 120)), ('yellow', (25, 220, 220)),
               ('black', (0, 0, 20)), ('light_blue', (90, 220, 220)),
               ('light_blue', (115, 100, 220))]
    for index, (expected, hsv) in enumerate(samples):
        image = canvas()
        cv2.circle(image, (333, 273), 28, bgr(hsv), -1, cv2.LINE_AA)
        cases.append(('solid-%s-%s' % (expected, index), image, expected, (333, 273)))
    for name in ('blank', 'rectangle', 'square', 'tiny', 'black-ring', 'red-ring',
                 'outside', 'cut-left', 'cut-bottom'):
        image = canvas()
        red = bgr((0, 220, 220))
        if name == 'rectangle':
            cv2.rectangle(image, (260, 190), (360, 240), red, -1)
        elif name == 'square':
            cv2.rectangle(image, (270, 200), (330, 260), red, -1)
        elif name == 'tiny':
            cv2.circle(image, (320, 270), 8, red, -1)
        elif name in ('black-ring', 'red-ring'):
            cv2.circle(image, (320, 270), 45, (20, 20, 20) if name == 'black-ring' else red, 8)
        elif name in ('outside', 'cut-left', 'cut-bottom'):
            center = {'outside': (55, 240), 'cut-left': (145, 240), 'cut-bottom': (320, 400)}[name]
            cv2.circle(image, center, 28, red, -1)
        cases.append((name, image, None, None))
    image = canvas()
    cv2.circle(image, (220, 180), 35, bgr((0, 220, 220)), -1)
    cv2.circle(image, (400, 300), 22, bgr((115, 220, 220)), -1)
    cv2.circle(image, (320, 240), 8, bgr((25, 220, 220)), -1)
    cases.append(('smallest-valid', image, 'blue', (400, 300)))
    image = canvas()
    cv2.circle(image, (240, 270), 32, bgr((0, 220, 220)), -1)
    cv2.circle(image, (400, 240), 23, (20, 20, 20), 6)
    cases.append(('solid-next-to-ring', image, 'red', (240, 270)))
    image = canvas()
    cv2.circle(image, (320, 270), 28, bgr((60, 220, 220)), -1)
    cases.append(('aligned-with-grasp', image, 'green', (320, 270)))
    for path in args.real_ring:
        image = cv2.imread(path)
        if image is None:
            raise RuntimeError('Cannot read ' + path)
        cases.append(('real-ring-'+Path(path).name, image, None, None))
    rows = []
    for name, image, expected, center in cases:
        original = image.copy()
        timings = []
        for repeat in range(6):
            start = time.perf_counter()
            result = color.find_color_block(image)
            if repeat:
                timings.append((time.perf_counter()-start)*1000)
        assert np.array_equal(image, original), 'Detector modified the OCR input'
        actual = result['color'] if result is not None else None
        assert actual == expected, (name, actual, expected)
        if center is not None:
            assert result['center'] == center, (name, result)
            assert (result['error_dx'], result['error_dy']) == (center[0]-320, center[1]-270)
            assert (result['dx'], result['dy']) == (center[0]-320, center[1]-240)
        rows.append(dict(name=name, passed=True, result=color.report(result),
                         median_ms=round(float(np.median(timings)), 2)))
        print(json.dumps(rows[-1]), flush=True)

    image = canvas()
    cv2.circle(image, (240, 270), 26, bgr((60, 220, 220)), -1)
    cv2.circle(image, (415, 240), 60, (15, 15, 15), 4)
    cv2.putText(image, '3', (393, 265), cv2.FONT_HERSHEY_SIMPLEX, 1.8,
                (15, 15, 15), 3, cv2.LINE_AA)
    ring = roi.find_ring(image, fast.find_smallest_ring, 360, bottom_trim=30)
    result = color.find_color_block(image)
    assert ring is not None and result is not None and result['color'] == 'green'
    assert result['center'] == (240, 270) and ring['center'] == (415, 240)
    engine = accelerate(digit_core.make_engine())
    try:
        digit, score, latency = fast.recognize(engine, fast.extract_ring_interior(image, ring), .60)
        assert digit == '3' and score >= .60, (digit, score)
    finally:
        engine.nx_trt_session.close()
    rendered = roi.annotate(image, ring, digit, score, latency, 0, 360, 30)
    color.annotate(rendered, result)
    cv2.imwrite(str(Path(args.output).with_suffix('.png')), rendered)
    report = dict(passed=True, cases=rows, simultaneous=dict(digit=digit, score=float(score),
                  color=color.report(result), grasp_point=[320, 270]))
    Path(args.output).write_text(json.dumps(report, indent=2))
    print('Color cases: {} passed; simultaneous GPU digit 3 and green passed.'.format(len(rows)))


if __name__ == '__main__':
    main()
