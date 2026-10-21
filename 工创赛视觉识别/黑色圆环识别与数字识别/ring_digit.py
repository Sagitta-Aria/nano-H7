"""给已检测圆环的中心区域添加数字识别；相机与圆环检测统一由 black_ring 管理。"""
from concurrent.futures import ThreadPoolExecutor
import time

import cv2
import numpy as np

try:
    from .digit_core import StableDigit, make_engine, recognize
except ImportError:  # Standalone source script entry.
    from digit_core import StableDigit, make_engine, recognize


def inner_ring_boundary(dark, ellipse):
    """在已定位圆环内沿椭圆采样找断开的内圈；只缩小 OCR 裁剪，不改控制圆心。

    单个数字不会覆盖一整圈。候选须覆盖至少 60% 角度，并在紧邻内侧具有
    亮间隙；不把单根线、数字笔画或实心黑块当作裁剪边界。
    """
    (cx, cy), axes, angle = ellipse
    minor_radius = min(axes) / 2
    radii = np.arange(.35, 1.08, 1 / max(1., minor_radius))
    angles = np.linspace(0, 2*np.pi, 180, endpoint=False)
    theta = np.deg2rad(angle)
    x = np.cos(angles) * axes[0] / 2
    y = np.sin(angles) * axes[1] / 2
    map_x = (cx + radii[:, None] * (x*np.cos(theta)-y*np.sin(theta))).astype(np.float32)
    map_y = (cy + radii[:, None] * (x*np.sin(theta)+y*np.cos(theta))).astype(np.float32)
    samples = cv2.remap(dark, map_x, map_y, cv2.INTER_NEAREST,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    coverage = np.mean(samples > 0, axis=1)
    active = coverage >= .6
    for index in range(1, len(radii)-1):
        if active[index-1] or not (active[index] and active[index+1]):
            continue
        inside = (radii >= radii[index]*.8) & (radii <= radii[index]*.94)
        if np.any(inside) and np.mean(coverage[inside]) < .25:
            return min(1., float(radii[index]))
    return 1.


def extract_ring_center(frame, ring, scale=0.75, output_size=192):
    """取孔洞中心附近，屏蔽环线；椭圆仅在 OCR 小图中近似校正。"""
    inner = ring['inner']
    if len(inner) < 5:
        return None
    ellipse = cv2.fitEllipse(inner)
    if not np.isfinite((*ellipse[0], *ellipse[1], ellipse[2])).all() or min(ellipse[1]) < 12:
        return None
    x, y, width, height = cv2.boundingRect(inner)
    x0, y0 = max(0, x - 3), max(0, y - 3)
    x1, y1 = min(frame.shape[1], x + width + 3), min(frame.shape[0], y + height + 3)
    if x1 <= x0 or y1 <= y0:
        return None
    interior = frame[y0:y1, x0:x1].copy()
    offset = np.array([x0, y0], dtype=np.int32).reshape(1, 1, 2)
    local_inner = inner - offset
    mask = np.zeros(interior.shape[:2], np.uint8)
    cv2.drawContours(mask, [local_inner], -1, 255, -1)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    interior[mask == 0] = 255
    center = np.asarray(ellipse[0]) - (x0, y0)

    # 圆环检测仍按整体反馈；这里只找中心内层孔洞，避免同心线进入 OCR。
    gray = cv2.cvtColor(interior, cv2.COLOR_BGR2GRAY)
    # 阈值只从真实孔洞像素估计。外部补白不能参与 Otsu，否则阴影中的
    # 灰纸会与白边分成两类，整张纸被当成黑色，内层环线便无法排除。
    valid_pixels = gray[mask > 0]
    if valid_pixels.size == 0:
        return None
    cutoff, _ = cv2.threshold(valid_pixels, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark = cv2.bitwise_and(cv2.inRange(gray, 0, cutoff), mask)
    # 把纸面统一成白底，避免灰纸与外部补白的边界再次成为数字前景。
    interior[dark == 0] = 255
    contours, hierarchy = cv2.findContours(dark, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    best_area = cv2.contourArea(inner)
    chosen = (tuple(center), ellipse[1], ellipse[2])
    if hierarchy is not None:
        for index, contour in enumerate(contours):
            if hierarchy[0][index][3] < 0 or len(contour) < 12:
                continue
            area = cv2.contourArea(contour)
            if not 100 < area < best_area or cv2.pointPolygonTest(contour, tuple(center), False) < 0:
                continue
            candidate = cv2.fitEllipse(contour)
            (cx, cy), axes, angle = candidate
            if (min(axes) < 16 or np.linalg.norm(np.asarray((cx, cy)) - center) > max(3, min(axes) * 0.08)
                    or abs(min(axes) / max(axes) - min(ellipse[1]) / max(ellipse[1])) > 0.18):
                continue
            theta = np.deg2rad(angle)
            points = contour[:, 0].astype(float) - (cx, cy)
            u = (points[:, 0] * np.cos(theta) + points[:, 1] * np.sin(theta)) / (axes[0] / 2)
            v = (-points[:, 0] * np.sin(theta) + points[:, 1] * np.cos(theta)) / (axes[1] / 2)
            if np.percentile(abs(np.hypot(u, v) - 1), 90) < 0.18:
                chosen, best_area = candidate, area

    boundary = inner_ring_boundary(dark, chosen)
    (cx, cy), (a, b), angle = chosen
    a, b = a * boundary * scale, b * boundary * scale
    theta = np.deg2rad(angle)
    cosine, sine = np.cos(theta), np.sin(theta)
    mask[:] = 0
    cv2.ellipse(mask, ((cx, cy), (a, b), angle), 255, -1)
    interior[mask == 0] = 255
    source = np.float32([[cx, cy], [cx + a * cosine / 2, cy + a * sine / 2],
                         [cx - b * sine / 2, cy + b * cosine / 2]])
    middle, radius = output_size / 2, output_size * 0.44
    destination = np.float32([[middle, middle],
                              [middle + radius * cosine, middle + radius * sine],
                              [middle - radius * sine, middle + radius * cosine]])
    # 不按椭圆的任意拟合角旋转文字，实时原画面也不做变换。
    transform = cv2.getAffineTransform(source, destination)
    return cv2.warpAffine(interior, transform, (output_size, output_size),
                          flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                          borderValue=(255, 255, 255))


class _Track:
    def __init__(self, ring, args):
        self.ring = ring
        self.stable = StableDigit(args.digit_confirm, args.digit_window, args.digit_hold)
        self.last_submit = float('-inf')
        self.last_completed = float('-inf')
        self.candidate, self.score, self.confirmed_score, self.latency = None, 0.0, 0.0, 0.0

    def accept(self, result, now):
        self.candidate, self.score, self.latency = result
        value = self.stable.update(self.candidate, now)
        if value is not None and value == self.candidate:
            self.confirmed_score = self.score
        self.last_completed = now

    def report(self, now):
        value = self.stable.current(now)
        recent = now - self.last_completed < self.stable.hold
        return dict(digit=value, confidence=round(self.confirmed_score if value else 0.0, 3),
                    candidate=self.candidate if recent else None, ocr_ms=round(self.latency, 1))


class RingDigitRecognizer:
    """一个后台任务处理同帧全部目标；丢失目标和过期帧的结果立即失效。"""
    def __init__(self, args, engine=None):
        self.args = args
        self.engine = make_engine() if engine is None else engine
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ring-digit')
        self.tracks, self.next_key = {}, 0
        self.future = None
        self.next_run, self.latency = 0.0, 0.0

    def reset(self):
        # 序号不重用；切换摄像头后正在运行的旧结果也不能落到新目标上。
        self.tracks.clear()
        self.next_run = 0.0

    def _associate(self, targets):
        pairs = []
        for key, track in self.tracks.items():
            old = track.ring
            for index, ring in enumerate(targets):
                distance = np.linalg.norm(np.asarray(old['center']) - ring['center'])
                ratio = ring['area'] / old['area']
                # 同心环的外层可能因断口消失。仅圆心极近时允许层级面积变化；
                # 普通运动关联仍使用原面积约束，丢失帧立即删除轨迹。
                same_center = distance <= max(6, min(old['radius'], ring['radius']) * .15)
                compatible = 0.55 <= ratio <= 1.8 or (same_center and .25 <= ratio <= 4.)
                if distance <= max(15, min(old['radius'], ring['radius']) * 0.4) and compatible:
                    pairs.append((distance / max(1, old['radius']) + abs(np.log(ratio)) * 0.2, key, index))
        assigned, used = {}, set()
        for _, key, index in sorted(pairs):
            if index not in assigned and key not in used:
                assigned[index] = key
                used.add(key)
        current, ids = {}, {}
        for index, ring in enumerate(targets):
            if index in assigned:
                key = assigned[index]
                track = self.tracks[key]
                track.ring = ring
            else:
                self.next_key += 1
                key = self.next_key
                track = _Track(ring, self.args)
            current[key] = track
            ids[ring.get('id', index + 1)] = key
        self.tracks = current
        return ids

    def _recognize_frame(self, frame, jobs):
        """在唯一 OCR 线程依次处理同帧原图裁剪；不碰跟踪状态或并发调用 GPU。"""
        results = {}
        for key, ring in jobs:
            crop = extract_ring_center(frame, ring, self.args.digit_roi_scale)
            results[key] = recognize(self.engine, crop, self.args.confidence,
                                     self.args.confidence_one)
        return results

    def update(self, frame, targets, *, synchronous=False):
        ids = self._associate(targets)
        if synchronous:
            # 单张离线图片不做时间投票；每个目标只识别一次。
            results = {}
            for ring_id, key in ids.items():
                crop = extract_ring_center(frame, self.tracks[key].ring, self.args.digit_roi_scale)
                text, score, latency = recognize(self.engine, crop, self.args.confidence, self.args.confidence_one)
                self.latency = latency
                results[ring_id] = dict(digit=text, confidence=round(score if text else 0.0, 3),
                                        candidate=None, ocr_ms=round(latency, 1))
            return results
        now = time.monotonic()
        if self.future is not None and self.future.done():
            results = self.future.result()
            for key, result in results.items():
                if key in self.tracks and now - self.tracks[key].last_submit < self.args.digit_hold:
                    self.tracks[key].accept(result, now)
                    self.latency = result[2]
            self.future = None
        if self.future is None and self.tracks and now >= self.next_run:
            jobs = [(key, track.ring) for key, track in self.tracks.items()]
            for track in self.tracks.values():
                track.last_submit = now
            # detect() 会在返回前绘制彩色轮廓，后台只能读取此处保存的未绘制图。
            self.future = self.pool.submit(self._recognize_frame,
                                          frame.copy() if frame is not None else None, jobs)
            self.next_run = now + self.args.digit_interval
        return {ring_id: self.tracks[key].report(now) for ring_id, key in ids.items()}

    def close(self):
        self.pool.shutdown(wait=True)  # Python 3.8 on Xavier NX; only one future is submitted.


def extract_ring_interior(frame, ring, output_size=192):
    """Compatibility crop for old tools; uses the updated center-only extraction."""
    return None if ring is None else extract_ring_center(frame, ring, output_size=output_size)


try:
    from .black_ring import find_smallest_ring
except ImportError:
    from black_ring import find_smallest_ring


class TargetDigit:
    """投票只属于当前最小圆环，忽略切换目标前提交的后台结果。"""
    def __init__(self, confirm=3, window=7, hold=0.6):
        self.stable = StableDigit(confirm, window, hold)
        self.generation = 0
        self.signature = None
        self.clear()

    def clear(self):
        self.stable.reset()
        self.candidate, self.score, self.latency = None, 0.0, 0.0
        self.last_completed = None

    def observe(self, ring):
        if ring is None:
            if self.signature is not None:
                self.generation += 1
            self.signature = None
            self.clear()
            return
        center, area, radius = ring['center'], ring['area'], ring['radius']
        changed = self.signature is None
        if self.signature is not None:
            old_center, old_area, old_radius = self.signature
            changed = (np.linalg.norm(np.asarray(center) - old_center) > max(12, old_radius * 0.45)
                       or not 0.55 <= area / old_area <= 1.8)
        if changed:
            self.generation += 1
            self.clear()
        self.signature = (np.asarray(center), area, radius)

    def accept(self, generation, result, now=None):
        if generation != self.generation or self.signature is None:
            return False
        now = time.monotonic() if now is None else now
        self.candidate, score, self.latency = result
        value = self.stable.update(self.candidate, now)
        if value is not None and value == self.candidate:
            self.score = score
        self.last_completed = now
        return True

    def value(self, now=None):
        now = time.monotonic() if now is None else now
        if ((self.stable.value is not None and now - self.stable.last_support >= self.stable.hold)
                or (self.last_completed is not None and now - self.last_completed > 0.5)):
            self.clear()
        return self.stable.value


if __name__ == '__main__':
    from black_ring import main
    main()
