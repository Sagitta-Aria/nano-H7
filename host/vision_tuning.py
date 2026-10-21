"""Shared detector configuration and diagnostics; no camera or motion ownership.

OpenCV is imported only inside image functions so the web server can run in a
lightweight environment. Camera workers receive complete validated snapshots.
"""
import copy
import importlib
import json
import math
from pathlib import Path

COLORS = ('red', 'green', 'blue', 'yellow', 'black', 'light_blue')
SOURCE = Path(__file__).resolve().parents[1] / '工创赛视觉识别'
RING_DEFAULT = dict(black_max=140, min_area=300, canny_low=40, canny_high=120,
                    min_coverage=.6, shape_tolerance=.2, detect_width=800,
                    nearby_distance=100)
OCR_DEFAULT = dict(confidence=.6, confidence_one=.3)


def defaults():
    """Import only interval one from the supplied color calibration files."""
    return validate({'colors': {color: json.loads((SOURCE / '色块识别' /
            (color + '_thresholds.json')).read_text(encoding='utf-8-sig')) for color in COLORS},
            'ring': copy.deepcopy(RING_DEFAULT)})


def validate(value):
    """Validate and return a single-interval snapshot without mutating the input.

    Historical two-interval colors remain readable; only interval one is kept
    for saving, preview and detection. Callers must use the returned snapshot.
    """
    value = copy.deepcopy(value)
    if isinstance(value, dict) and 'ocr' not in value:
        value['ocr'] = copy.deepcopy(OCR_DEFAULT)
    if not isinstance(value, dict) or set(value) != {'colors', 'ring', 'ocr'}:
        raise ValueError('视觉阈值必须包含 colors、ring 和 ocr。')
    ocr = value['ocr']
    if not isinstance(ocr, dict) or set(ocr) != set(OCR_DEFAULT):
        raise ValueError('数字置信度参数字段不完整。')
    for key in OCR_DEFAULT:
        if type(ocr[key]) not in (int, float) or not math.isfinite(ocr[key]) or not .01 <= ocr[key] <= 1:
            raise ValueError('数字置信度须为 0.01～1.00：' + key)
    colors = value['colors']
    if not isinstance(colors, dict) or set(colors) != set(COLORS):
        raise ValueError('请提供六种颜色的完整阈值。')
    for name in COLORS:
        item = colors[name]
        if not isinstance(item, dict) or set(item) != {'color', 'min_area', 'ranges'} or item['color'] != name:
            raise ValueError('颜色配置字段不正确：' + name)
        if type(item['min_area']) is not int or not 1 <= item['min_area'] <= 10000000:
            raise ValueError('色块面积须为 1～10000000 的整数。')
        ranges = item['ranges']
        if not isinstance(ranges, list) or len(ranges) not in ((1, 2) if name in ('red', 'blue', 'light_blue') else (1,)):
            raise ValueError('HSV 区间数量不正确：' + name)
        for pair in ranges:
            if not isinstance(pair, list) or len(pair) != 2:
                raise ValueError('HSV 区间必须包含上下限。')
            for bound in pair:
                if not isinstance(bound, list) or len(bound) != 3 or any(
                        type(n) is not int or not 0 <= n <= limit for n, limit in zip(bound, (179, 255, 255))):
                    raise ValueError('HSV 须为整数：H 0～179，S/V 0～255。')
            if any(a > b for a, b in zip(*pair)):
                raise ValueError('HSV 下限不能大于上限：' + name)
        item['ranges'] = ranges[:1]
    ring = value['ring']
    if not isinstance(ring, dict) or set(ring) != set(RING_DEFAULT):
        raise ValueError('圆环参数字段不完整。')
    for key, low, high in [('black_max', 0, 254), ('min_area', 1, 10000000),
                           ('canny_low', 0, 254), ('canny_high', 1, 255),
                           ('detect_width', 0, 4096), ('nearby_distance', 0, 2000)]:
        if type(ring[key]) is not int or not low <= ring[key] <= high:
            raise ValueError('圆环参数越界：' + key)
    if ring['canny_low'] >= ring['canny_high'] or 0 < ring['detect_width'] < 160:
        raise ValueError('Canny 上限须大于下限；检测宽度须为 0 或 160～4096。')
    for key, low, high in [('min_coverage', .4, 1.), ('shape_tolerance', .05, .3)]:
        if type(ring[key]) not in (float, int) or not math.isfinite(ring[key]) or not low <= ring[key] <= high:
            raise ValueError('圆环参数越界：' + key)
    return value


def legacy_settings(min_area, black_max):
    """Expose pre-console thresholds without replacing existing calibration.

    Only files without a detector section use this migration. New installations
    and the explicit defaults button continue to use the supplied source JSONs.
    """
    ranges = {
        'red': [[[0, 80, 46], [10, 255, 255]], [[170, 80, 46], [179, 255, 255]]],
        'green': [[[35, 50, 46], [84, 255, 255]]],
        'blue': [[[100, 160, 46], [135, 255, 255]], [[100, 50, 46], [135, 159, 159]]],
        'yellow': [[[15, 80, 46], [34, 255, 255]]],
        'black': [[[0, 0, 0], [179, 255, 45]]],
        'light_blue': [[[85, 50, 46], [99, 255, 255]], [[100, 50, 160], [135, 159, 255]]],
    }
    value = defaults()
    for name in COLORS:
        value['colors'][name].update(min_area=min_area, ranges=ranges[name][:1])
    value['ring'].update(min_area=min_area, black_max=min(254, black_max))
    return value


def color_result(frame, name, settings):
    """Use the new unrestricted contour detector; black selects the largest blob."""
    module = importlib.import_module('工创赛视觉识别.色块识别.color_block_center')
    item = settings['colors'][name]
    mask = module.make_mask(frame, item['ranges'][:1])
    return module.find_block(mask, item['min_area'], largest=name == 'black'), mask


def ring_results(frame, settings, debug=False):
    """Run both edge and mask paths once, returning native-pixel ring geometry."""
    module = importlib.import_module('工创赛视觉识别.黑色圆环识别与数字识别.black_ring')
    return module.analyze_ring_frame(frame, debug=debug, **settings['ring'])


def preview(frame, config, captured_at):
    """Render a diagnostic frame and segmentation from the actual control detector.

    This single-frame path does not run OCR, vote on targets, or move hardware.
    The caller owns image encoding and limits preview request frequency.
    """
    from .camera_vision import detect
    report = detect(frame, config, captured_at, diagnostic=True)
    mask = report.pop('_debug')
    return report, frame, mask
