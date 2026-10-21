"""Camera-two task QR adapter; no serial, motion, OCR or camera ownership.

Reuse the competition detector and parser without starting its standalone GUI.
Results distinguish the current frame from the last valid code in this session.
"""
import importlib.util
from pathlib import Path
from .task_code import parse_task_code


class TaskQR:
    """Decode native camera frames and retain grouped digits, including zeros."""

    def __init__(self):
        source = Path(__file__).resolve().parents[1] / '工创赛视觉识别' / '二维码识别' / 'qr_task.py'
        spec = importlib.util.spec_from_file_location('competition_qr_task', str(source))
        self.algorithm = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.algorithm)
        self.detector = self.algorithm.cv2.QRCodeDetector()
        self.last_code = None
        self.last_seen_at = None
        self.last_plan = None

    def process(self, frame, captured_at):
        """Decode one frame; invalid/missing codes never replace the last valid one."""
        text, points, failed = self.algorithm.decode_frame(self.detector, frame)
        # Keep protocol messages bounded even for arbitrary non-task QR payloads.
        result = dict(status='searching', text=text[:512], code=None, groups=[], digits=[],
                      captured_at=captured_at, task_plan=None, task_error='')
        if failed:
            result['status'] = 'decode_error'
        elif text:
            try:
                groups = self.algorithm.parse_groups(text)
                if len(text) > 512:
                    raise ValueError('任务码过长')
            except ValueError:
                result['status'] = 'invalid'
            else:
                code = '+'.join(groups)
                result.update(status='valid', code=code, groups=groups,
                              digits=[int(d) for group in groups for d in group])
                self.last_code, self.last_seen_at = code, captured_at
                try:
                    result['task_plan'] = parse_task_code(code)
                    self.last_plan = result['task_plan']
                except ValueError as error:
                    result['task_error'] = str(error)
        elif points is not None:
            result['status'] = 'located'
        result.update(last_code=self.last_code, last_seen_at=self.last_seen_at, last_plan=self.last_plan)
        return result
