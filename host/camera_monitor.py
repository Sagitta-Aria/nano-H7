"""NX touchscreen recognition display using the camera worker's raw frames.

The shared detector and asynchronous digit reader own no camera here. Local mode buttons
select preview perception only; alignment always uses its saved configuration.
All coordinates displayed below are converted back to native camera pixels.
"""
import contextlib
import sys
import threading
import time
from collections import deque
from pathlib import Path

# Competition task code card:每个字符的字高不得低于 12 像素。
TASK_CODE_MIN_PX = 12
TASK_CODE_MARGIN = 24


def task_code(state):
    """Return the competition task code only while a task is running.

    Reads one host state snapshot; never opens hardware, camera or motion. An
    inactive task returns nothing even if a stale plan is still cached, so the
    card disappears the moment the two-round flow finishes or stops.
    """
    if not isinstance(state, dict):
        return ''
    plan_owner = state.get('task')
    if not isinstance(plan_owner, dict) or not plan_owner.get('active'):
        return ''
    plan = plan_owner.get('task_plan')
    code = plan.get('code') if isinstance(plan, dict) else None
    return code.strip() if isinstance(code, str) else ''


def task_code_frame(code, size=(480, 800)):
    """Full-screen pure white card with one centred black line of the code.

    The string is never wrapped or truncated: the glyph scale shrinks until the
    whole code fits the screen, and the 12 px minimum glyph height is enforced
    by raising instead of silently drawing something unreadable.
    """
    import cv2
    import numpy as np
    height, width = size
    frame = np.full((height, width, 3), 255, np.uint8)
    text = '' if code is None else str(code).strip()
    if not text:
        return frame
    scale, thickness = 8.0, 9
    text_width = text_height = 0
    while scale > 0.35:
        thickness = max(1, int(round(scale * 1.15)))
        (text_width, text_height), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_DUPLEX, scale, thickness)
        if (text_width <= width - 2 * TASK_CODE_MARGIN
                and text_height <= height - 2 * TASK_CODE_MARGIN):
            break
        scale -= .1
    if (text_height < TASK_CODE_MIN_PX
            or text_width > width - 2 * TASK_CODE_MARGIN):
        raise ValueError('任务码过长，无法以不低于 {} px 的字高全屏显示。'.format(TASK_CODE_MIN_PX))
    origin = ((width - text_width) // 2, (height + text_height) // 2)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_DUPLEX, scale,
                (0, 0, 0), thickness, cv2.LINE_AA)
    return frame


class RecognitionMonitor:
    """Run perception off the GUI thread and render an 800x480 touch display."""

    def __init__(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
        from nx_vision_ui import TouchModePanel
        self.controls = TouchModePanel()
        self.lock = threading.Lock()
        self.runtime = None
        from .ring_detector import RingDetector
        self.detector = RingDetector()
        self.requested = 'digit'
        self.report = None
        self.error = ''
        self.frame_times = deque()
        self.processed_mode = None
        self.task_code = ''

    def update_task_code(self, code):
        """Publish the card text; an empty string returns the camera preview."""
        with self.lock:
            self.task_code = code or ''

    def on_mouse(self, *args):
        self.controls.on_mouse(*args)

    def select(self, mode):
        """Queue a display-only mode switch; never change alignment or motion."""
        with self.lock:
            self.requested = mode

    def process(self, raw, config, captured_monotonic=None):
        """Use saved tuning on native pixels; OCR votes are display-only."""
        from .camera_vision import detect
        from .ring_perception import PreviewOCR
        import copy
        start = time.monotonic()
        with self.lock:
            mode = self.requested
        if mode != self.processed_mode:
            # Completion rates belong to one mode. A slow digit frame must not
            # depress the color rate for two seconds after a touchscreen switch.
            # This is owned by the perception thread, never mutated by the GUI.
            self.frame_times.clear()
            self.processed_mode = mode
        settings = copy.deepcopy(config)
        settings['kind'] = 'ring' if mode == 'digit' else 'block'
        with contextlib.redirect_stdout(sys.stderr):
            if mode == 'digit' and self.runtime is None:
                self.runtime = PreviewOCR()
            report = detect(raw.copy(), settings, time.time(),
                            ocr=self.runtime if mode == 'digit' else None,
                            ring_detector=self.detector.detect)
        finished = time.monotonic()
        self.frame_times.append(finished)
        while len(self.frame_times) > 1 and finished-self.frame_times[0] > 2.:
            self.frame_times.popleft()
        fps = ((len(self.frame_times)-1)/max(.001, finished-self.frame_times[0])
               if len(self.frame_times)>1 else 0.)
        report.update(captured_monotonic=captured_monotonic if captured_monotonic is not None else start,
                      mode=mode, digit=report.get('digit'), confidence=report.get('confidence') or 0.,
                      fps=fps, ocr_ms=0.,
                      color=report['target'] if mode == 'color' and report['found'] else None,
                      roi=(0, 0, raw.shape[1], raw.shape[0]))
        with self.lock:
            self.report, self.error = report, ''

    def failed(self, error):
        with self.lock:
            self.error = str(error)

    def draw(self, raw, alignment_report=None, tracking=False):
        """Paint native coordinates and recognition results; never infer on GUI."""
        import cv2
        import numpy as np
        mode = self.controls.take_mode()
        if mode and not tracking:
            self.select(mode)
        with self.lock:
            report = alignment_report if tracking else self.report
            selected, error = self.requested, self.error
            card = self.task_code
        if card:
            # Pure white full-screen card: the whole 800x480 display goes to the
            # task code. The camera keeps streaming, so finishing the task
            # restores the live preview on the next frame with no restart.
            return task_code_frame(card, (480, 800))
        height, width = raw.shape[:2]
        image = cv2.resize(raw, (640, 360))
        canvas = np.zeros((480, 640, 3), np.uint8)
        now = time.monotonic()
        age = now-report['captured_monotonic'] if report else None
        fresh = report is not None and age < 1.2 and (tracking or report['mode'] == selected)
        def position(point):
            return (round(point[0]*640/width), round(point[1]*360/height))
        reference = [width//2, height//2]
        cv2.drawMarker(image, position(reference), (0,255,255), cv2.MARKER_CROSS, 20, 1)
        if fresh and tracking and report.get('pickup_only'):
            x, y, w, h = report['roi']
            cv2.rectangle(image, position([x, y]), position([x+w-1, y+h-1]), (255,255,0), 2)
        if fresh and report['found']:
            cv2.circle(image, position(report['center']), 5, (0,255,0), 2)
            cv2.drawMarker(image, position(report['control_point']), (255,0,255), cv2.MARKER_CROSS, 20, 2)
            cv2.line(image, position(reference), position(report['control_point']), (0,255,255), 1)
        if fresh and selected == 'digit' and not tracking:
            for observation in report.get('digits', []):
                x, y = position(observation['center'])
                cv2.circle(image, (x, y), 5, (0, 255, 0), 2)
                label = '{} {:.2f}'.format(observation['digit'], observation['confidence'])
                cv2.putText(image, label, (max(0, x-25), max(18, y-20)),
                            cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2, cv2.LINE_AA)
        if fresh and tracking:
            for center in report.get('rings', []):
                cv2.circle(image, position(center), 5, (0,255,0), 2)
            line = report.get('row_line')
            if line:
                cv2.line(image, position(line['points'][0]), position(line['points'][1]), (255,255,0), 2)
        if tracking:
            title = 'ALIGN / '+(report.get('target','waiting') if report else 'waiting')
            detail = 'NX visual PID | {:.0f} ms'.format(age*1000) if age is not None else 'Waiting for alignment frame'
            if report and report.get('pickup_only'):
                title = 'B PICK / '+report['target']
                detail = 'Color inside box | chassis stopped'
            if report and report.get('row_line'):
                detail = 'Row CW error: {:.1f} deg'.format(report['row_line']['error_deg'])
            if report and report.get('expected_digit') and report.get('digit'):
                detail += ' | Digit: '+str(report['digit'])
        else:
            title = 'DIGIT / OCR GPU' if selected == 'digit' else 'COLOR'
            if fresh:
                detail = ('Digit: {}  conf {:.2f}'.format(report['digit'] or '--', report['confidence'])
                          if selected == 'digit' else 'Target: {} | Color: {}'.format(report['target'], report['color'] or '--'))
                detail += ' | {:.1f} FPS | {:.0f} ms'.format(report['fps'], age*1000)
            else:
                detail = ('ERROR: '+error) if error else 'Loading / waiting for fresh recognition'
        found = fresh and report['found']
        lines = [title+' | camera {}x{}'.format(width,height), detail,
                 'Center {}  Point {}  Ref {}'.format(report['center'] if found else '--',
                     report['control_point'] if found else '--', reference),
                 'Error X={}  Y={} px | {}'.format(report['error_dx'] if found else '--',
                     report['error_dy'] if found else '--', 'FOUND' if found else 'NO TARGET')]
        for i,line in enumerate(lines):
            cv2.putText(canvas, line, (8,22+i*28), cv2.FONT_HERSHEY_SIMPLEX,
                        .50, (255,255,255) if i else (0,230,255), 1, cv2.LINE_AA)
        canvas[120:480] = image
        return self.controls.compose(canvas, selected)

    def close(self):
        """Release the cached GPU runtime after the perception thread exits."""
        self.detector.close()
        if self.runtime:
            with contextlib.redirect_stdout(sys.stderr):
                self.runtime.close()
