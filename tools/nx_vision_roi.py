"""Central detection ROI and full-frame overlays for the NX live test.

This crops processing, not camera capture, and does not calibrate lens distortion.
Coordinates returned to the demo remain in the original camera pixel system.
"""

import cv2


def grasp_metrics(frame, ring=None, offset=(0, 30)):
    """Measure ring position and alignment error against the calibrated grasp point.

    Offsets use full-camera pixels: right/down are positive. dx/dy remain
    relative to image center; error_dx/error_dy are ring minus grasp point.
    ROI changes do not move this reference. No actuator command is generated.
    """
    height, width = frame.shape[:2]
    origin = (width//2, height//2)
    point = (origin[0]+offset[0], origin[1]+offset[1])
    if not (0 <= point[0] < width and 0 <= point[1] < height):
        raise ValueError('Grasp point must lie inside the camera image')
    center = ring['center'] if ring is not None else None
    return dict(grasp_point=point, grasp_offset=offset,
                dx=center[0]-origin[0] if center is not None else None,
                dy=center[1]-origin[1] if center is not None else None,
                error_dx=center[0]-point[0] if center is not None else None,
                error_dy=center[1]-point[1] if center is not None else None)


def roi_bounds(frame, size=360, bottom_trim=0):
    """Clip a central square then raise only its bottom; size=0 uses the full image."""
    height, width = frame.shape[:2]
    if size < 0:
        raise ValueError('ROI size cannot be negative')
    if size == 0:
        return 0, 0, width, height
    side = min(size, width, height)
    if not 0 <= bottom_trim < side:
        raise ValueError('ROI bottom trim must leave a positive height')
    return (width-side)//2, (height-side)//2, side, side-bottom_trim


def find_ring(frame, detector, size=360, black_max=140, min_area=200, bottom_trim=0):
    """Run detection only inside the ROI and translate all geometry to the frame.

    The detector must reject rings cut by its input boundary. Ring areas and
    radii retain their pixel units because the image is cropped without scaling.
    """
    x, y, width, height = roi_bounds(frame, size, bottom_trim)
    ring = detector(frame[y:y+height, x:x+width], black_max, min_area)
    if ring is None:
        return None
    mapped = dict(ring)
    mapped['center'] = (ring['center'][0]+x, ring['center'][1]+y)
    for key in ('outer', 'inner'):
        mapped[key] = ring[key] + (x, y)
    center, axes, angle = ring['ellipse']
    mapped['ellipse'] = ((center[0]+x, center[1]+y), axes, angle)
    return mapped


def annotate_reference(frame, size=360, bottom_trim=0, grasp_offset=(0, 30)):
    """Draw shared ROI/grasp references without running or naming a detector."""
    x, y, width, height = roi_bounds(frame, size, bottom_trim)
    image = cv2.convertScaleAbs(frame, alpha=.4)
    image[y:y+height, x:x+width] = frame[y:y+height, x:x+width]
    if size:
        cv2.rectangle(image, (x, y), (x+width-1, y+height-1), (0, 255, 255), 2)
    origin = (frame.shape[1]//2, frame.shape[0]//2)
    metrics = grasp_metrics(frame, None, grasp_offset)
    point = metrics['grasp_point']
    cv2.drawMarker(image, origin, (160, 160, 160), cv2.MARKER_CROSS, 12, 1)
    cv2.drawMarker(image, point, (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.circle(image, point, 7, (255, 0, 255), 1)
    cv2.putText(image, 'GRAB', (point[0]+13, point[1]+5),
                cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 0, 255), 1)
    cv2.putText(image, 'GRAB {} | target dx={:+d} dy={:+d} | ex/ey: target - grab'.format(
        point, grasp_offset[0], grasp_offset[1]), (10, frame.shape[0]-33),
        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 180, 255), 1)
    return image


def annotate(frame, ring, digit, score, latency, fps, size=360, bottom_trim=0,
             grasp_offset=(0, 30)):
    """Draw digit telemetry over the same references used in color-only mode."""
    image = annotate_reference(frame, size, bottom_trim, grasp_offset)
    _, _, width, height = roi_bounds(frame, size, bottom_trim)
    metrics = grasp_metrics(frame, ring, grasp_offset)
    point = metrics['grasp_point']
    if ring is not None:
        center = ring['center']
        cv2.drawContours(image, [ring['outer'], ring['inner']], -1, (0, 255, 0), 2)
        cv2.circle(image, center, 3, (0, 0, 255), -1)
        cv2.line(image, point, center, (0, 180, 255), 1)
        position = 'dx{:+d} dy{:+d} | ex{:+d} ey{:+d}'.format(
            metrics['dx'], metrics['dy'], metrics['error_dx'], metrics['error_dy'])
    else:
        position = 'No complete ring inside ROI'
    cv2.rectangle(image, (0, 0), (frame.shape[1]-1, 53), (0, 0, 0), -1)
    status = 'Digit: {}  conf {:.2f}'.format(digit, score) if digit else 'Digit: --'
    cv2.putText(image, '{} | FPS {:.1f}'.format(status, fps), (10, 23),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
    cv2.putText(image, 'ROI {}x{} | {} | OCR {:.1f}ms'.format(width, height, position, latency),
                (10, 46), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    return image
