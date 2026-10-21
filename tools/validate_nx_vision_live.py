"""Replay real digit-1 failures and generated targets on the NX GPU.

Run inside vision-env with --source pointing to the unchanged demo modules.
Use --backend original before a fix and fast afterwards. The generated cases
cover other digits and empty/solid targets; they do not establish field accuracy.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def main():
    """Fail on incorrect labels and record warmed per-stage replay timings."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True)
    parser.add_argument('--backend', choices=['original', 'fast'], required=True)
    parser.add_argument('--real-one', nargs='*', default=[])
    parser.add_argument('--output', required=True)
    parser.add_argument('--roi-size', type=int, default=0,
                        help='Centered detection square in pixels; 0 uses the full frame')
    parser.add_argument('--roi-bottom-trim', type=int, default=0)
    args = parser.parse_args()
    sys.path.insert(0, args.source)
    import black_ring
    import digit_core
    import ring_digit
    from nx_vision_trt import accelerate
    if args.backend == 'fast':
        import nx_vision_fast as backend
    else:
        backend = None
    detect = backend.find_smallest_ring if backend else black_ring.find_smallest_ring
    extract = backend.extract_ring_interior if backend else ring_digit.extract_ring_interior
    recognize = backend.recognize if backend else digit_core.recognize
    import nx_vision_roi
    if args.roi_size < 0:
        parser.error('ROI size cannot be negative')
    full_detector = detect
    if args.roi_size:
        def detect(frame):
            """Exercise the same crop/coordinate mapping used by the live launcher."""
            return nx_vision_roi.find_ring(frame, full_detector, args.roi_size,
                                           bottom_trim=args.roi_bottom_trim)
    cv2.setNumThreads(1)
    cases = []
    for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX, cv2.FONT_HERSHEY_TRIPLEX):
        for digit in ('1', '2', '3'):
            image = np.full((480, 640, 3), 210, np.uint8)
            cv2.circle(image, (320, 260), 100, (15, 15, 15), 5)
            (w, h), _ = cv2.getTextSize(digit, font, 2.7, 4)
            cv2.putText(image, digit, (320-w//2, 260+h//2), font, 2.7, (15, 15, 15), 4, cv2.LINE_AA)
            cases.append(('font-%s-digit-%s' % (font, digit), image, digit))
    for name in ('blank', 'solid', 'empty-ring', 'digit-4', 'digit-7', 'digit-12'):
        image = np.full((480, 640, 3), 210, np.uint8)
        if name != 'blank':
            cv2.circle(image, (320, 260), 100, (15, 15, 15), -1 if name == 'solid' else 5)
        if name.startswith('digit-'):
            text = name.split('-')[1]
            (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 2.1, 4)
            cv2.putText(image, text, (320-w//2, 260+h//2), cv2.FONT_HERSHEY_SIMPLEX,
                        2.1, (15, 15, 15), 4, cv2.LINE_AA)
        cases.append((name, image, None))
    if args.roi_size == 360:
        for name in ('roi-outside', 'roi-cut-ring', 'roi-center-and-outside'):
            image = np.full((480, 640, 3), 210, np.uint8)
            center = (145, 240) if name == 'roi-cut-ring' else (55, 240)
            cv2.circle(image, center, 42, (15, 15, 15), 4)
            cv2.putText(image, '2', (center[0]-15, center[1]+17), cv2.FONT_HERSHEY_SIMPLEX,
                        1.3, (15, 15, 15), 3, cv2.LINE_AA)
            expected = None
            if name == 'roi-center-and-outside':
                cv2.circle(image, (320, 260), 100, (15, 15, 15), 5)
                cv2.putText(image, '1', (293, 290), cv2.FONT_HERSHEY_SIMPLEX,
                            2.7, (15, 15, 15), 4, cv2.LINE_AA)
                expected = '1'
            cases.append((name, image, expected))
        if args.roi_bottom_trim == 30:
            image = np.full((480, 640, 3), 210, np.uint8)
            cv2.circle(image, (320, 402), 12, (15, 15, 15), 2)
            cases.append(('roi-raised-bottom', image, None))
    for path in args.real_one:
        image = cv2.imread(path)
        if image is None:
            raise RuntimeError('Cannot read ' + path)
        cases.append((Path(path).name, image, '1'))
    engine = accelerate(digit_core.make_engine())
    rows = []
    try:
        for name, image, expected in cases:
            timings, result = [], (None, 0, 0)
            for repeat in range(4):
                start = time.perf_counter()
                ring = detect(image)
                if args.roi_size and ring is not None:
                    x, y, width, height = nx_vision_roi.roi_bounds(image, args.roi_size,
                                                                 args.roi_bottom_trim)
                    for contour in (ring['outer'], ring['inner']):
                        assert (contour[:, 0, 0] > x).all() and (contour[:, 0, 0] < x+width).all()
                        assert (contour[:, 0, 1] > y).all() and (contour[:, 0, 1] < y+height).all()
                    assert np.linalg.norm(np.asarray(ring['center'])-ring['ellipse'][0]) < 1
                    if name == 'roi-center-and-outside':
                        assert np.linalg.norm(np.asarray(ring['center'])-[320, 260]) < 2
                if name in ('roi-outside', 'roi-cut-ring', 'roi-raised-bottom'):
                    assert ring is None
                after_detect = time.perf_counter()
                crop = extract(image, ring)
                after_crop = time.perf_counter()
                result = recognize(engine, crop, .60, .30) if crop is not None else (None, 0, 0)
                end = time.perf_counter()
                if repeat:
                    timings.append([(after_detect-start)*1000, (after_crop-after_detect)*1000,
                                    (end-after_crop)*1000, (end-start)*1000])
            row = dict(name=name, expected=expected, actual=result[0], score=round(float(result[1]), 3),
                       passed=(result[0] == expected and
                               (args.backend != 'fast' or expected is None or result[1] >= .60)),
                       ring=ring is not None,
                       median_ms=dict(zip(('ring', 'crop', 'ocr', 'total'),
                                          np.median(timings, axis=0).round(2).tolist())))
            rows.append(row)
            print(json.dumps(row), flush=True)
    finally:
        engine.nx_trt_session.close()
    report = dict(backend=args.backend, roi_size=args.roi_size, roi_bottom_trim=args.roi_bottom_trim,
                  passed=all(row['passed'] for row in rows), cases=rows)
    Path(args.output).write_text(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
