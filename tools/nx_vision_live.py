"""Mutually exclusive digit/GPU and solid-color modes on one NX camera.

Use with vision-env and deployed demo modules. Touch buttons or keys 1/2 switch
algorithms without reopening the camera; this module never controls the robot.
"""

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np

import nx_vision_roi as roi


def parse_args(argv=None):
    """Accept existing camera/OCR settings plus an explicit initial mode."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--mode', choices=('digit', 'color'), default='digit')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--black-max', type=int, default=140)
    parser.add_argument('--min-area', type=float, default=200)
    parser.add_argument('--confidence', type=float, default=.60)
    parser.add_argument('--confidence-one', type=float, default=.30)
    parser.add_argument('--interval', type=float, default=.03)
    parser.add_argument('--confirm', type=int, default=3)
    parser.add_argument('--window', type=int, default=7)
    parser.add_argument('--hold', type=float, default=.6)
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--image')
    parser.add_argument('--output')
    args = parser.parse_args(argv)
    args.source = args.source.resolve(strict=True)
    if (not 0 <= args.black_max < 255 or args.min_area <= 0
            or not 0 <= args.confidence <= 1 or not 0 <= args.confidence_one <= 1
            or args.interval < 0 or args.confirm < 1 or args.window < args.confirm
            or args.hold <= 0):
        parser.error('Invalid thresholds or voting parameters')
    if args.output and not args.image:
        parser.error('--output requires --image')
    return args


def open_camera(index):
    """Request MJPEG/640x480 and a short V4L2 queue; log negotiated settings."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError('Cannot open camera {}'.format(index))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 60)
    buffer_set = cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print(json.dumps(dict(camera=index, width=cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                          height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                          capture_fps=cap.get(cv2.CAP_PROP_FPS),
                          buffer_count=cap.get(cv2.CAP_PROP_BUFFERSIZE),
                          buffer_set=buffer_set)), flush=True)
    return cap


def save_image(path, image):
    """Encode captures with Unicode-safe paths; fail visibly on encoding errors."""
    ok, encoded = cv2.imencode(path.suffix or '.png', image)
    if not ok:
        raise RuntimeError('Cannot encode image')
    encoded.tofile(str(path))


class VisionModes:
    """Own mode state and lazy OCR resources, independently of the camera/UI.

    Call switch/process on the UI thread. A switch drains the single pending
    OCR job before color processing starts and discards all prior target votes.
    A previously loaded GPU engine stays cached, idle in color mode, until close.
    """

    def __init__(self, args):
        self.args = args
        self.backend = os.environ.get('NX_VISION_BACKEND', 'fast')
        if self.backend not in ('fast', 'original'):
            raise ValueError('NX_VISION_BACKEND must be fast or original')
        # Search the entire camera image. Old ROI environment variables must
        # not silently restore the central crop after a service restart.
        self.roi_size = 0
        self.bottom_trim = 0
        self.grasp_offset = (int(os.environ.get('NX_VISION_GRASP_DX', '0')),
                             int(os.environ.get('NX_VISION_GRASP_DY', '30')))
        self.color_min_area = float(os.environ.get('NX_VISION_COLOR_MIN_AREA', '500'))
        if self.roi_size != 0 and self.roi_size < 80:
            raise ValueError('ROI size must be 0 or at least 80')
        if self.bottom_trim < 0 or (self.roi_size and self.bottom_trim >= self.roi_size):
            raise ValueError('ROI bottom trim must leave a positive height')
        if self.color_min_area <= 0:
            raise ValueError('Color minimum area must be positive')
        self.engine = self.pool = self.future = self.target = None
        self.ring_module = self.color_module = self.mode = None
        self.frame_times = deque()
        try:
            self.switch(args.mode)
        except BaseException:
            self.close()
            raise

    def _load_digit(self):
        """Load GPU OCR only on first entry into digit mode; reuse original voting."""
        if self.engine is not None:
            return
        import ring_digit
        from nx_vision_trt import accelerate
        self.ring_module = ring_digit
        algorithm = ring_digit
        self.detect_ring = algorithm.find_smallest_ring
        self.extract = algorithm.extract_ring_interior
        self.recognize = algorithm.recognize
        self.engine = accelerate(ring_digit.make_engine())
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.target = ring_digit.TargetDigit(self.args.confirm, self.args.window, self.args.hold)
        print('OCR backend: TensorRT GPU (FP16-enabled engine)', flush=True)

    def _drain(self):
        """Finish and discard outstanding inference before a switch or cleanup."""
        if self.future is not None:
            future, self.future = self.future, None
            future.result()

    def switch(self, mode):
        """Select exactly one detector and clear stale targets, FPS and counters."""
        if mode not in ('digit', 'color'):
            raise ValueError('Unknown mode: ' + mode)
        if mode == self.mode:
            return
        self._drain()
        if self.target is not None:
            self.target.observe(None)
        if mode == 'digit':
            self._load_digit()
        elif self.color_module is None:
            import nx_vision_color
            self.color_module = nx_vision_color
        self.mode = mode
        self.ring = self.color = self.crop = self.digit = None
        self.score = self.latency = self.color_ms = self.fps = 0.0
        self.next_run = 0.0
        self.ring_frames = self.color_frames = self.ocr_submitted = 0
        self.frame_times.clear()
        print(json.dumps(dict(event='mode_changed', mode=mode,
                              ocr_loaded=self.engine is not None,
                              roi_size=self.roi_size, bottom_trim=self.bottom_trim,
                              grasp_offset=self.grasp_offset)), flush=True)

    def process(self, frame):
        """Process one unmodified raw frame using only the selected algorithm."""
        now = time.monotonic()
        if self.mode == 'digit':
            self.ring = roi.find_ring(frame, self.detect_ring, self.roi_size,
                                      self.args.black_max, self.args.min_area, self.bottom_trim)
            self.ring_frames += 1
            self.target.observe(self.ring)
            if self.future is not None and self.future.done():
                self.target.accept(self.submitted_generation, self.future.result(), now)
                self.future = None
            self.crop = self.extract(frame, self.ring)
            if self.future is None and self.crop is not None and now >= self.next_run:
                self.submitted_generation = self.target.generation
                self.future = self.pool.submit(self.recognize, self.engine, self.crop,
                                               self.args.confidence, self.args.confidence_one)
                self.ocr_submitted += 1
                self.next_run = now + self.args.interval
            self.digit = self.target.value(now)
            self.score, self.latency = self.target.score, self.target.latency
        else:
            start = time.perf_counter()
            self.color = self.color_module.find_color_block(
                frame, self.color_min_area, self.roi_size, self.bottom_trim, self.grasp_offset)
            self.color_ms = (time.perf_counter()-start)*1000
            self.color_frames += 1
        now = time.monotonic()
        self.frame_times.append(now)
        while len(self.frame_times) > 2 and now-self.frame_times[0] > 2:
            self.frame_times.popleft()
        self.fps = ((len(self.frame_times)-1)/(now-self.frame_times[0])
                    if len(self.frame_times) > 1 else 0.0)

    def report(self, frame):
        """Expose selected-mode telemetry and counters to verify detector isolation."""
        target = self.ring if self.mode == 'digit' else self.color
        return dict(mode=self.mode, center=target['center'] if target is not None else None,
                    processing_fps=round(self.fps, 1),
                    ring=self.ring is not None, digit=self.digit,
                    confidence=round(float(self.score), 3), ocr_ms=round(self.latency, 1),
                    color_block=self.color_module.report(self.color) if self.color is not None else None,
                    color_ms=round(self.color_ms, 2),
                    ring_frames=self.ring_frames, color_frames=self.color_frames,
                    ocr_submitted=self.ocr_submitted,
                    roi=roi.roi_bounds(frame, self.roi_size, self.bottom_trim),
                    **roi.grasp_metrics(frame, target, self.grasp_offset))

    def annotate(self, frame):
        """Draw only the selected result plus shared references and switching keys."""
        if self.mode == 'digit':
            image = roi.annotate(frame, self.ring, self.digit, self.score, self.latency,
                                 self.fps, self.roi_size, self.bottom_trim, self.grasp_offset)
            label = 'DIGIT / GPU'
        else:
            image = roi.annotate_reference(frame, self.roi_size, self.bottom_trim, self.grasp_offset)
            self.color_module.annotate(image, self.color)
            cv2.rectangle(image, (0, 0), (image.shape[1]-1, 53), (0, 0, 0), -1)
            name = self.color['color'] if self.color is not None else '--'
            cv2.putText(image, 'Color: {} | FPS {:.1f}'.format(name, self.fps), (10, 23),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 200, 0), 2)
            _, _, width, height = roi.roi_bounds(frame, self.roi_size, self.bottom_trim)
            cv2.putText(image, 'ROI {}x{} | detection {:.1f}ms'.format(width, height, self.color_ms),
                        (10, 46), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
            label = 'COLOR'
        cv2.putText(image, '{} | Tap side buttons to switch'.format(label),
                    (10, image.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 255, 255), 1)
        return image

    def close(self):
        """Wait for the worker before releasing its GPU session, including on errors."""
        try:
            self._drain()
        finally:
            if self.pool is not None:
                self.pool.shutdown(wait=True)
            if self.engine is not None:
                self.engine.nx_trt_session.close()
                self.engine = None


def main():
    """Run a single persistent camera/window, or one offline frame for validation."""
    import sys
    args = parse_args()
    sys.path.insert(0, str(args.source))
    cv2.setNumThreads(1)
    runtime, cap = None, None
    window = 'NX Vision'
    try:
        runtime = VisionModes(args)
        if args.image:
            frame = cv2.imdecode(np.fromfile(args.image, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError('Cannot read image')
            runtime.process(frame)
            if runtime.future is not None:
                runtime.digit, runtime.score, runtime.latency = runtime.future.result()
                runtime.future = None
            print(json.dumps(runtime.report(frame)), flush=True)
            image = runtime.annotate(frame)
            if args.output:
                save_image(Path(args.output), image)
            else:
                cv2.imshow(window, image)
                cv2.waitKey(0)
            return
        cap = open_camera(args.camera)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        from nx_vision_ui import TouchModePanel
        controls = TouchModePanel()
        cv2.setMouseCallback(window, controls.on_mouse)
        failures, last_report = 0, 0.0
        debug_shown = False
        print('Touch side buttons: digit/color; 1/2 also switch; Q/Esc: exit; S: save; C: camera', flush=True)
        while True:
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures >= 10:
                    raise RuntimeError('Camera read failed')
                key = cv2.waitKey(100) & 0xFF
                if key in (27, ord('q')):
                    break
                continue
            failures = 0
            runtime.process(frame)
            cv2.imshow(window, controls.compose(runtime.annotate(frame), runtime.mode))
            if args.debug and runtime.crop is not None:
                cv2.imshow('Ring interior OCR', runtime.crop)
                debug_shown = True
            elif debug_shown:
                cv2.destroyWindow('Ring interior OCR')
                debug_shown = False
            now = time.monotonic()
            if now-last_report >= .5:
                print(json.dumps(runtime.report(frame)), flush=True)
                last_report = now
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            requested = controls.take_mode()
            if requested is not None:
                print(json.dumps(dict(event='touch_mode_requested', mode=requested)), flush=True)
                runtime.switch(requested)
            elif key in (ord('1'), ord('2')):
                runtime.switch('digit' if key == ord('1') else 'color')
            elif key == ord('s'):
                folder = args.source/'captures'
                folder.mkdir(exist_ok=True)
                stamp = str(time.time_ns())
                save_image(folder/(stamp+'_frame.png'), frame)
                if runtime.crop is not None:
                    save_image(folder/(stamp+'_interior.png'), runtime.crop)
            elif key == ord('c'):
                cap.release()
                candidate = 0 if args.camera == 1 else 1
                try:
                    cap = open_camera(candidate)
                    args.camera = candidate
                except RuntimeError as exc:
                    print(exc, flush=True)
                    cap = open_camera(args.camera)
                runtime._drain()
                if runtime.target is not None:
                    runtime.target.observe(None)
                runtime.frame_times.clear()
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if runtime is not None:
            runtime.close()


if __name__ == '__main__':
    main()
