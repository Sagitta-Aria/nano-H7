"""Same-frame ring geometry and lazily loaded 1/2/3 OCR for camera-one runs.

No camera or motor I/O is owned here. Geometry works without OCR dependencies;
the digit engine is created only after the chassis has become parallel.
"""
import contextlib
import importlib
import platform
import sys
from pathlib import Path


def find_rings(frame, black_max, min_area):
    """Return distinct ring centers, merging masks and concentric nested rings."""
    from .vision_tuning import defaults, ring_results
    settings = defaults()
    settings['ring'].update(black_max=black_max, min_area=min_area)
    return ring_results(frame, settings)[0]


class RingOCR:
    """Reuse the contest OCR on ring interiors; NX uses its TensorRT adapter.

    Only use on the camera perception thread. Results are raw OCR evidence; the
    motion owner requires multiple fresh stationary frames before accepting them.
    Missing OCR dependencies raise a stream error and never fabricate a digit.
    """
    def __init__(self):
        from .vision_tuning import OCR_DEFAULT
        self.thresholds = dict(OCR_DEFAULT)
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                               '工创赛视觉识别' / '黑色圆环识别与数字识别'))
        with contextlib.redirect_stdout(sys.stderr):
            import ring_digit
            self.algorithm = ring_digit
            self.engine = ring_digit.make_engine()
            if platform.machine().lower() in ('aarch64', 'arm64') and sys.platform == 'linux':
                from nx_vision_trt import accelerate
                self.engine = accelerate(self.engine)


    def configure(self, settings):
        """Use the validated saved thresholds for fresh control-frame inference."""
        self.thresholds = dict(settings)

    def select(self, frame, rings, expected):
        """Read actual interiors, returning only an independently recognized target."""
        matches = []
        self.observations = []
        with contextlib.redirect_stdout(sys.stderr):
            for ring in rings:
                crop = self.algorithm.extract_ring_interior(frame, ring)
                if crop is None:
                    continue
                digit, confidence, latency = self.algorithm.recognize(
                    self.engine, crop, self.thresholds['confidence'], self.thresholds['confidence_one'])
                if digit is not None and str(digit) in ('1', '2', '3'):
                    self.observations.append(dict(digit=str(digit), center=list(ring['center']),
                                                  confidence=float(confidence)))
                if digit is not None and (expected is None or str(digit) == expected):
                    matches.append(dict(ring, digit=str(digit), confidence=confidence, ocr_ms=latency))
        return max(matches, key=lambda item: item['confidence']) if matches else None

    def close(self):
        """Release TensorRT resources when the enclosing recognition stream ends."""
        # accelerate() installs a GPU session on the OCR text recognizer.
        session = getattr(self.engine, 'nx_trt_session', None)
        if hasattr(session, 'close'):
            session.close()


class PreviewOCR:
    """Asynchronous per-ring voting for display only, never motion evidence.

    The control reader above returns fresh inference on the current frame;
    cached display votes must not satisfy the controller's stationary checks.
    """
    def __init__(self):
        from types import SimpleNamespace
        self.reader = RingOCR()
        args = SimpleNamespace(digit_confirm=3, digit_window=7, digit_hold=.6,
                               digit_roi_scale=.75, confidence=.6,
                               confidence_one=.3, digit_interval=.06)
        self.worker = self.reader.algorithm.RingDigitRecognizer(args, self.reader.engine)
        self.observations = []

    def configure(self, settings):
        """Apply saved preview thresholds, draining old work and clearing old votes.

        Only the perception thread calls this; no GPU engine is reloaded. Old
        threshold results must not confirm digits under the newly saved values.
        """
        if self.reader.thresholds == settings:
            return
        self.worker.close()
        self.reader.configure(settings)
        self.worker.args.confidence = settings['confidence']
        self.worker.args.confidence_one = settings['confidence_one']
        self.worker = self.reader.algorithm.RingDigitRecognizer(self.worker.args, self.reader.engine)
        self.observations = []

    def select(self, frame, rings, expected=None):
        """Expose confirmed display votes attached only to matched current targets."""
        results = self.worker.update(frame, rings)
        matches = [dict(ring, **results[ring['id']]) for ring in rings
                   if results.get(ring['id'], {}).get('digit') is not None]
        self.observations = [dict(digit=r['digit'], center=list(r['center']),
                                  confidence=r['confidence']) for r in matches]
        return max(matches, key=lambda r: r['confidence']) if matches else (rings[0] if rings else None)

    def close(self):
        """Drain the single worker before releasing device inference buffers."""
        self.worker.close()
        self.reader.close()
