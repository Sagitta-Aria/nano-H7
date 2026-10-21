"""Legacy all-color preview wrapper for the calibrated contour detector.

Per-color area limits and HSV ranges come from the supplied configuration files.
The camera worker uses saved VisionSettings directly for task and monitor paths.
No camera, inference engine, or robot resource is opened by this module.
"""

import cv2
import numpy as np

from color_block_center import COLOR_RANGES
from nx_vision_roi import grasp_metrics, roi_bounds


KERNEL = np.ones((5, 5), np.uint8)


def find_color_block(frame, min_area=500, size=0, bottom_trim=0, grasp_offset=(0, 30)):
    """Select one result across calibrated colors; black prefers its largest blob.

    min_area is retained for old callers; calibrated per-color limits take
    precedence. Coordinates are translated from the optional ROI to the frame.
    """
    from host.vision_tuning import defaults, color_result
    x0, y0, width, height = roi_bounds(frame, size, bottom_trim)
    settings = defaults()
    candidates = []
    for name in settings['colors']:
        result, _ = color_result(frame[y0:y0+height, x0:x0+width], name, settings)
        if result:
            area, center, contour = result
            candidates.append(dict(color=name, center=(center[0]+x0, center[1]+y0), area=area,
                contour=contour+np.array([[[x0,y0]]], dtype=np.int32)))
    best = min(candidates, key=lambda item: item['area']) if candidates else None
    if best is not None:
        best.update(grasp_metrics(frame, best, grasp_offset))
    return best


def report(result):
    """Return JSON-safe detection telemetry without contour arrays or fake confidence."""
    if result is None:
        return None
    return {key: value for key, value in result.items() if key != 'contour'}


def annotate(image, result):
    """Draw a separate color target and its error on the shared preview in place."""
    if result is None:
        status = 'Color: -- (color contours)'
    else:
        cv2.drawContours(image, [result['contour']], -1, (255, 200, 0), 2)
        cv2.drawMarker(image, result['center'], (255, 200, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.line(image, result['grasp_point'], result['center'], (255, 200, 0), 1)
        status = 'Color: {} | dx{:+d} dy{:+d} | ex{:+d} ey{:+d}'.format(
            result['color'], result['dx'], result['dy'], result['error_dx'], result['error_dy'])
    y = image.shape[0]-61
    cv2.rectangle(image, (0, y-18), (image.shape[1]-1, y+6), (0, 0, 0), -1)
    cv2.putText(image, status, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 200, 0), 1)
    return image
