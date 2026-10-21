"""Mission perception runner using the NX's existing detectors and one camera.

Run instead of nx_vision_live while executing a mission, so there is one camera
owner. No serial handle is opened. See docs/MISSION.md for calibration and usage.
"""
import sys
from pathlib import Path
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from host.mission_vision import VisionPublisher, sample_for


def main():
    """Select QR/color/ring from host requests and publish capture-stamped results."""
    import os
    import cv2
    import nx_vision_live as live
    args = live.parse_args()
    if args.image:
        raise SystemExit('Use nx_vision_live for offline images; mission input requires live capture.')
    sys.path.insert(0, str(args.source))
    publisher = VisionPublisher(os.environ.get('MISSION_HOST', 'http://127.0.0.1:8770'),
                                os.environ.get('MISSION_TOKEN'))
    cap = runtime = None
    identity = None
    frame_id = 0
    cv2.setNumThreads(1)
    try:
        # Start in color mode, keeping GPU/OCR lazy until a ring request arrives.
        args.mode = 'color'
        runtime = live.VisionModes(args)
        qr = cv2.QRCodeDetector()
        cap = live.open_camera(args.camera)
        while True:
            captured_at = time.time()
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera read failed')
            frame_id += 1
            request = publisher.request()
            if not request:
                time.sleep(.02)
                continue
            current = (request['run_id'], request['request_id'])
            if current != identity:
                runtime._drain()
                if runtime.target is not None:
                    runtime.target.observe(None)
                runtime.digit = None
                runtime.next_run = 0.
                identity = current
                if request['kind'] != 'qr':
                    runtime.switch('color' if request['kind'] == 'block' else 'digit')
                # The frame captured before this switch cannot confirm this request.
                continue
            if request['kind'] == 'qr':
                text, _, _ = qr.detectAndDecode(frame)
                report = {'qr_text': text}
            else:
                if request['kind'] == 'ring':
                    # Mission confirmation must identify this captured frame.
                    # UI OCR voting can retain a digit briefly; stamping that
                    # cached value with a new frame time would fake freshness.
                    ring = live.roi.find_ring(frame, runtime.detect_ring, runtime.roi_size,
                                              args.black_max, args.min_area, runtime.bottom_trim)
                    crop = runtime.extract(frame, ring)
                    digit = None
                    if crop is not None:
                        digit, _, _ = runtime.recognize(runtime.engine, crop,
                                                       args.confidence, args.confidence_one)
                    report = dict(mode='digit', ring=ring is not None, digit=digit,
                                  **live.roi.grasp_metrics(frame, ring, runtime.grasp_offset))
                else:
                    runtime.process(frame)
                    report = runtime.report(frame)
            publisher.publish(sample_for(request, report, 'nx-mission-camera', frame_id, captured_at))
    finally:
        publisher.close()
        if cap is not None:
            cap.release()
        if runtime is not None:
            runtime.close()


if __name__ == '__main__':
    main()
