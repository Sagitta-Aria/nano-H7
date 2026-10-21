"""Fresh-frame ring geometry with bounded background discovery, no camera I/O.

Background results are search hints only. Every returned center is detected in
the caller's current image; old centers never become motion evidence.
"""
from concurrent.futures import ProcessPoolExecutor
import copy
import importlib
import multiprocessing
import os
import sys
import time


def _discovery_worker(parent_pid):
    """Bound OpenCV threads and exit with the camera when its parent is terminated."""
    import cv2
    cv2.setNumThreads(1)
    if sys.platform.startswith('linux'):
        import ctypes
        import signal
        # CameraHub can terminate a blocked camera process. Its CPU discovery
        # child must not survive that stop and leak a process on every restart.
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), 'Cannot bind ring discovery to camera lifetime')
        if os.getppid() != parent_pid:
            raise RuntimeError('Camera exited before ring discovery initialized')


class RingDetector:
    """Re-detect current ROIs each frame; search the full image in the background."""
    def __init__(self):
        self.algorithm = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.black_ring')
        # Full arc search is Python-heavy; a thread competes with current-frame
        # geometry for the GIL. Spawn also avoids inheriting a live CUDA context.
        self.pool = ProcessPoolExecutor(max_workers=1,
                                       mp_context=multiprocessing.get_context('spawn'),
                                       initializer=_discovery_worker, initargs=(os.getpid(),))
        self.future = None
        self.hints = []
        self.previous = []
        self.signature = None
        self.next_scan = 0.
        self.submitted = 0.

    def detect(self, frame, settings, debug=False):
        """Return only same-frame targets; discover unseen broken rings at 2 Hz."""
        import cv2
        import numpy as np
        config = settings['ring']
        signature = (frame.shape, tuple(sorted(config.items())))
        now = time.monotonic()
        if signature != self.signature:
            self.hints = []
            self.previous = []
            self.signature = signature
            self.next_scan = 0.
        if debug:
            return self.algorithm.analyze_ring_frame(frame, debug=True, **config)
        if self.future is not None and self.future.done():
            old_signature, rings = self.future.result()
            self.future = None
            if old_signature == signature and now-self.submitted < 1.5:
                self.hints = rings
        # All ROIs use one sampling grid. Resizing each crop independently changes
        # subpixel phase and can break thin printed rings as the crop moves.
        work = frame
        width = config['detect_width']
        if width > 0 and frame.shape[1] > width:
            work = cv2.resize(frame, (width, max(1, round(frame.shape[0]*width/frame.shape[1]))),
                              interpolation=cv2.INTER_LINEAR)
        sx, sy = work.shape[1]/frame.shape[1], work.shape[0]/frame.shape[0]
        local_config = dict(config, detect_width=0, min_area=config['min_area']*sx*sy,
                            nearby_distance=0)
        cutoff, _ = cv2.threshold(cv2.cvtColor(work, cv2.COLOR_BGR2GRAY), 0, 255,
                                  cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        fast_config = dict(local_config, cutoff=cutoff)
        seeds = self.previous + self.hints
        targets = [] if seeds else self.algorithm.restore_ring_targets(
            self.algorithm.fast_ring_frame(work, **fast_config), frame.shape, work.shape)
        global_targets = None
        for hint in seeds:
            # A fast current-frame result already validates this station, regardless
            # of which concentric layer the background happened to discover.
            if any(np.linalg.norm(np.asarray(hint['center'])-r['center']) <
                   max(12, min(hint['radius'], r['radius'])*.4) for r in targets):
                continue
            x, y, width, height = cv2.boundingRect(hint['outer'])
            margin = max(20, round(max(width, height)*.2))
            x0, y0 = max(0, int((x-margin)*sx)), max(0, int((y-margin)*sy))
            x1 = min(work.shape[1], int(np.ceil((x+width+margin)*sx)))
            y1 = min(work.shape[0], int(np.ceil((y+height+margin)*sy)))
            local = work[y0:y1, x0:x1]
            rings = self.algorithm.fast_ring_frame(local, **fast_config)
            if not rings:
                # A crop changes Otsu's brightness distribution. Before taking
                # the expensive arc path, try one shared whole-frame mask.
                if global_targets is None:
                    global_targets = self.algorithm.restore_ring_targets(
                        self.algorithm.fast_ring_frame(work, **fast_config), frame.shape, work.shape)
                    targets.extend(global_targets)
                if any(np.linalg.norm(np.asarray(hint['center'])-r['center']) <
                       max(12, min(hint['radius'], r['radius'])*.4) for r in global_targets):
                    continue
                rings, _ = self.algorithm.analyze_ring_frame(local, **local_config)
            for ring in rings:
                ring['center'] = (ring['center'][0]+x0, ring['center'][1]+y0)
                for name in ('outer', 'inner'):
                    ring[name] = ring[name] + np.array([x0,y0], np.int32)
                center, axes, angle = ring['ellipse']
                ring['ellipse'] = ((center[0]+x0,center[1]+y0),axes,angle)
            targets.extend(self.algorithm.restore_ring_targets(rings, frame.shape, work.shape))
        if self.future is None and now >= self.next_scan:
            self.submitted = now
            self.next_scan = now + .5
            self.future = self.pool.submit(self._discover, frame.copy(), copy.deepcopy(config), signature)
        targets = self.algorithm.merge_concentric_rings(targets)
        if config['nearby_distance'] > 0:
            targets = self.algorithm.suppress_nearby_rings(targets, config['nearby_distance'])
        targets.sort(key=lambda r:r['center'])
        for index, target in enumerate(targets, 1):
            target['id'] = index
        self.previous = targets
        return targets, None

    @staticmethod
    def _discover(frame, config, signature):
        """Run one bounded full search; result coordinates are never returned directly."""
        import cv2
        cv2.setNumThreads(1)
        algorithm = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.black_ring')
        return signature, algorithm.analyze_ring_frame(frame, **config)[0]

    def close(self):
        """Join discovery before the enclosing camera worker releases its resources."""
        self.pool.shutdown(wait=True)
