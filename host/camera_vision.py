"""Reusable camera-one perception and bounded alignment decisions.

Importing this module opens no camera/serial resource and does not import CV.
Detection runs in a camera subprocess. Pixel errors are suitable for a future
held-station adapter; they do not establish an absolute vehicle coordinate.
"""
import copy
import json
import math
import os
import tempfile
from pathlib import Path

from .vision_alignment import TRACKING_DEFAULT, validate_tracking
from . import vision_tuning

SEQUENCE_DEFAULT = {'range_steps': 2000, 'scan_step': 200, 'sps': 300, 'spacing_steps': 2200,
                    'angle_tolerance_deg': 2., 'max_turn_deg': 15.}

DEFAULT = {'kind': 'block', 'target': 'red', 'target_offset_px': [0, 30],
           'min_area': 500, 'black_max': 140,
           'tracking': copy.deepcopy(TRACKING_DEFAULT),
           'ring_sequence': copy.deepcopy(SEQUENCE_DEFAULT),
           'detector': vision_tuning.defaults()}
COLORS = ('red', 'green', 'blue', 'yellow', 'black', 'light_blue')


def validated(value):
    """Validate recognition, target offset and direct visual tracking controls."""
    if isinstance(value, dict) and 'detector' not in value:
        value = copy.deepcopy(value)
        value['detector'] = vision_tuning.legacy_settings(value.get('min_area', 500),
                                                        value.get('black_max', 140))
    if isinstance(value, dict) and 'ring_sequence' not in value:
        value = dict(value, ring_sequence=copy.deepcopy(SEQUENCE_DEFAULT))
    if isinstance(value, dict) and isinstance(value.get('ring_sequence'), dict):
        # Older saved settings gain the fixed spacing without rewriting disk.
        value = dict(value, ring_sequence=dict({'spacing_steps': 2200}, **value['ring_sequence']))
    if not isinstance(value, dict) or set(value) != set(DEFAULT):
        raise ValueError('识别配置字段不完整。')
    if value['kind'] not in ('block', 'ring') or value['target'] not in COLORS:
        raise ValueError('请选择色块/圆环及有效颜色。')
    offset = value['target_offset_px']
    if not isinstance(offset, list) or len(offset) != 2 or any(type(v) is not int or abs(v) > 10000 for v in offset):
        raise ValueError('目标偏移必须是两个 ±10000 内的整数像素。')
    for key, low, high in (('min_area', 40, 1000000), ('black_max', 1, 255)):
        if type(value[key]) is not int or not low <= value[key] <= high:
            raise ValueError('识别面积或黑色阈值越界。')
    value = dict(value, detector=vision_tuning.validate(value['detector']))
    validate_tracking(value['tracking'])
    sequence = value['ring_sequence']
    if not isinstance(sequence, dict) or set(sequence) != set(SEQUENCE_DEFAULT):
        raise ValueError('请提供完整的圆列搜索配置。')
    for key, low, high in (('range_steps', 1, 100000), ('scan_step', 1, 1000), ('sps', 1, 2000),
                           ('spacing_steps', 1, 100000)):
        if type(sequence[key]) is not int or not low <= sequence[key] <= high:
            raise ValueError('圆列 {} 必须是 {}～{} 的整数。'.format(key, low, high))
    for key, low, high in (('angle_tolerance_deg', .2, 10), ('max_turn_deg', 1, 30)):
        if type(sequence[key]) not in (int, float) or not math.isfinite(sequence[key]) or not low <= sequence[key] <= high:
            raise ValueError('圆列角度配置无效。')
    return copy.deepcopy(value)


class VisionSettings:
    """Persist camera-one settings only; saving never arms or moves the car."""
    def __init__(self, path):
        self.path = Path(path)
        value = json.loads(self.path.read_text(encoding='utf-8')) if self.path.exists() else copy.deepcopy(DEFAULT)
        # Retain saved recognition/offset/speed when upgrading the first camera
        # release. Loading does not rewrite the user's file or change task config.
        if isinstance(value, dict) and 'calibration' in value and 'reference_offset_px' in value:
            value = copy.deepcopy(value)
            previous = value.pop('calibration')
            value['target_offset_px'] = value.pop('reference_offset_px')
            value['tracking'] = dict(TRACKING_DEFAULT, sps=previous['sps'], deadzone_px=previous['tolerance_px'])
        if isinstance(value, dict) and isinstance(value.get('tracking'), dict):
            # Existing settings gain PID fields without rewriting them on load.
            value['tracking'] = dict(TRACKING_DEFAULT, **value['tracking'])
            value['tracking']['min_sps'] = min(value['tracking']['min_sps'], value['tracking']['sps'])
        self.value = validated(value)

    def snapshot(self):
        return copy.deepcopy(self.value)

    def save(self, value):
        value = validated(value)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        name = None
        try:
            with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=self.path.parent, delete=False) as stream:
                name = stream.name
                json.dump(value, stream, ensure_ascii=False, indent=2)
                stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
            os.replace(name, self.path)
        finally:
            if name and os.path.exists(name): os.unlink(name)
        self.value = value
        return self.snapshot()


def pickup_roi_bounds(width, height, percentages):
    """Convert B's percentage crop to native pixels; reject empty/outside boxes."""
    if (not isinstance(percentages, (list, tuple)) or len(percentages) != 4 or
            any(type(v) is not int for v in percentages)):
        raise ValueError('B点识别框必须包含四个整数百分比。')
    x, y, w, h = percentages
    if min(x, y) < 0 or min(w, h) < 1 or x+w > 100 or y+h > 100:
        raise ValueError('B点识别框不能超出画面范围。')
    left, top = width*x//100, height*y//100
    right, bottom = width*(x+w)//100, height*(y+h)//100
    if right <= left or bottom <= top:
        raise ValueError('B点识别框像素范围为空。')
    return [left, top, right-left, bottom-top]


def detect(frame, config, captured_at, ocr=None, expected=None, diagnostic=False, ring_detector=None):
    """Detect a selected color or the smallest black ring in original pixels.

    Reuses the updated contest detectors and the saved per-color/ring tuning.
    Ring geometry supplies a same-frame center baseline, never ellipse yaw.
    Ordered digit recognition is enabled only by the sequence's OCR phase.
    Optional preview_roi_pct crops only an explicit diagnostic request. The
    saved task settings and native coordinate system remain unchanged.
    """
    import cv2
    config = dict(config)
    stack_layout = config.pop('stack_layout', None)
    stack_expected = config.pop('stack_expected', None)
    pickup_roi_pct = config.pop('pickup_roi_pct', None)
    preview_roi_pct = config.pop('preview_roi_pct', None)
    if preview_roi_pct is not None and (not diagnostic or pickup_roi_pct is not None):
        raise ValueError('调试框只能用于独立调参预览。')
    config = validated(config)
    height, width = frame.shape[:2]
    ox, oy = config['target_offset_px']
    reference = [width // 2, height // 2]
    crop = preview_roi_pct if preview_roi_pct is not None else pickup_roi_pct
    roi = pickup_roi_bounds(width, height, crop) if crop is not None else None
    if roi and config['kind'] != 'block':
        raise ValueError('B点识别框仅用于色块抓取。')
    if config['kind'] == 'block':
        x, y, w, h = roi if roi else [0, 0, width, height]
        result, debug = vision_tuning.color_result(frame[y:y+h, x:x+w], config['target'], config['detector'])
        center = list(result[1]) if result else None
        if result:
            center = [center[0]+x, center[1]+y]
            cv2.drawContours(frame, [result[2]], -1, (0, 255, 0), 2, offset=(x, y))
    else:
        from .ring_sequence import row_line
        if stack_layout is None and ocr is not None and hasattr(ocr, 'configure'):
            ocr.configure(config['detector']['ocr'])
        rings, debug = (ring_detector or vision_tuning.ring_results)(frame, config['detector'], diagnostic)
        line = row_line([ring['center'] for ring in rings], config['tracking']['camera_yaw_deg'])
        if stack_layout is not None:
            from .stack_ring import select_anchor
            result, anchor_point, color_rings = select_anchor(frame, rings, config['detector'],
                stack_layout, config['tracking']['camera_yaw_deg'], stack_expected)
        else:
            result = ocr.select(frame, rings, expected) if ocr else (min(rings, key=lambda r: r['area']) if rings else None)
        center = list(result['center']) if result else None
        for ring in rings:
            cv2.drawContours(frame, [ring['outer'], ring['inner']], -1, (0, 255, 0), 2)
        if line:
            cv2.line(frame, tuple(line['points'][0]), tuple(line['points'][1]), (255, 255, 0), 2)
    control_point = [center[0] + ox, center[1] + oy] if center else None
    report = {'kind': config['kind'], 'target': config['target'] if config['kind'] == 'block' else 'ring',
              'found': center is not None, 'center': center, 'reference': reference,
              'control_point': control_point,
              'width': width, 'height': height, 'captured_at': captured_at,
              'error_dx': control_point[0] - reference[0] if center else None,
              'error_dy': control_point[1] - reference[1] if center else None}
    if roi:
        report.update(roi=roi, pickup_only=pickup_roi_pct is not None)
        cv2.rectangle(frame, (roi[0], roi[1]), (roi[0]+roi[2]-1, roi[1]+roi[3]-1), (255, 255, 0), 2)
    if config['kind'] == 'ring':
        report.update(rings=[list(ring['center']) for ring in rings], row_line=line,
                      digits=copy.deepcopy(getattr(ocr, 'observations', [])),
                      expected_digit=expected,
                      digit=result.get('digit') if result else None,
                      confidence=result.get('confidence') if result else None)
        if stack_layout is not None:
            report.update(anchor_point=anchor_point, identity_source='remembered_color',
                          color_rings=color_rings, digit=None, digits=[], expected_digit=None)
    cv2.drawMarker(frame, tuple(reference), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
    deadzone = config['tracking']['deadzone_px']
    cv2.rectangle(frame, (reference[0]-deadzone, reference[1]-deadzone),
                  (reference[0]+deadzone, reference[1]+deadzone), (0, 255, 255), 1)
    if center:
        cv2.line(frame, tuple(reference), tuple(control_point), (0, 255, 255), 2)
        cv2.line(frame, tuple(center), tuple(control_point), (255, 0, 255), 2)
        cv2.drawMarker(frame, tuple(control_point), (255, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.circle(frame, tuple(center), 4, (0, 0, 255), -1)
    cv2.putText(frame, '{} dx={} dy={}'.format(report['target'], report['error_dx'], report['error_dy']),
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 180, 255), 2)
    if diagnostic:
        if config['kind'] == 'block':
            report['color_settings'] = copy.deepcopy(config['detector']['colors'][config['target']])
        report['_debug'] = debug
    return report


