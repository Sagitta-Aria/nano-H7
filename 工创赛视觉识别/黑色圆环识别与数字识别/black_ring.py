"""工位多黑色圆环：同心环合并，独立圆环分别反馈，允许轻微轮廓畸变。

参考 https://gitee.com/linsen06/cyqm_vision_gongxun 的
code/ring_station_detector.py：Canny、椭圆观测和同心聚类。
本机适配不使用原仓库的相机内参、工位零点、串口或毫米换算。
"""
import argparse
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np


DEFAULT_MIN_AREA = 50000
NEARBY_RING_DISTANCE = 100  # 原始画面像素；圆心距离不超过此值时保留面积较大的目标。
RING_DEFAULTS = dict(black_max=140, min_area=DEFAULT_MIN_AREA, canny_low=40,
                     canny_high=120, min_coverage=0.60, shape_tolerance=0.20)
DEFAULT_CONFIG = Path(__file__).resolve().with_name('ring_thresholds.json')


def residual_p90(values):
    """有限一维轮廓误差的线性 90 分位；避免通用 percentile 的调度开销。"""
    position = (values.size - 1) * .9
    low = int(position)
    high = min(low + 1, values.size - 1)
    ordered = np.partition(values, (low, high))
    return float(ordered[low] + (ordered[high] - ordered[low]) * (position - low))


def fit_contour(contour):
    """等距抽取至多 128 个边界点拟合；原轮廓仍用于完整形状与面积核验。"""
    stride = max(1, (len(contour) + 127) // 128)
    return cv2.fitEllipseAMS(np.ascontiguousarray(contour[::stride]))


def circle_geometry(contour, min_area, shape_tolerance=0.20):
    """以椭圆作近似，允许透视、镜头造成的轻微非椭圆变形。"""
    area = cv2.contourArea(contour)
    if area < min_area or len(contour) < 5:
        return None
    try:
        ellipse = fit_contour(contour)
    except cv2.error:
        return None
    center, axes, angle = ellipse
    if (not np.isfinite((*center, *axes, angle)).all() or min(axes) < 6
            or min(axes) / max(axes) < 0.25):
        return None
    points = contour[:, 0, :].astype(float) - np.asarray(center)
    theta = np.deg2rad(angle)
    u = points[:, 0] * np.cos(theta) + points[:, 1] * np.sin(theta)
    v = -points[:, 0] * np.sin(theta) + points[:, 1] * np.cos(theta)
    radial = np.sqrt((u / (axes[0] / 2)) ** 2 + (v / (axes[1] / 2)) ** 2)
    fill = area / (np.pi * axes[0] * axes[1] / 4)
    polygon = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
    if (len(polygon) <= 4 or not 0.72 <= fill <= 1.28
            or residual_p90(abs(radial - 1)) > shape_tolerance):
        return None
    return area, center, ellipse


def neutral_mask(frame):
    """等价的 OpenCV 通道运算，避免整帧转 int16 和反复做 NumPy 归约。"""
    blue, green, red = cv2.split(frame)
    maximum = cv2.max(cv2.max(blue, green), red)
    minimum = cv2.min(cv2.min(blue, green), red)
    return cv2.bitwise_or(cv2.inRange(cv2.subtract(maximum, minimum), 0, 65),
                          cv2.inRange(maximum, 0, 60))


def make_masks(frame, black_max=140, *, gray=None, neutral=None):
    if gray is None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if neutral is None:
        neutral = neutral_mask(frame)
    # 允许黑色在曝光后呈灰色，排除明显饱和的彩色底板。
    ceiling = cv2.bitwise_and(cv2.inRange(gray, 0, black_max), neutral)
    _, otsu_mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    global_mask = cv2.bitwise_and(ceiling, otsu_mask)
    local_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 41, 7)
    local_mask = cv2.bitwise_and(local_mask, ceiling)
    # 小幅闭运算连接细小断点，避免较大核吞掉小环孔洞。
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return [cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel) for m in (global_mask, local_mask)]


def rings_from_mask(mask, min_area, gray, shape_tolerance=0.20):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]
    height, width = mask.shape
    candidates = []
    for index, outer in enumerate(contours):
        # 没有子轮廓便不可能有孔洞，先跳过以免对纹理和实心块做椭圆拟合。
        if hierarchy[index][2] == -1:
            continue
        # 偶数深度是黑色区域，奇数深度是孔洞；允许套环。
        depth, parent = 0, hierarchy[index][3]
        while parent != -1:
            depth += 1
            parent = hierarchy[parent][3]
        if depth % 2:
            continue
        geometry = circle_geometry(outer, min_area, shape_tolerance)
        if geometry is None:
            continue
        area, center, ellipse = geometry
        radius = max(ellipse[1]) / 2
        x, y, w, h = cv2.boundingRect(outer)
        if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
            continue
        children = []
        child = hierarchy[index][2]
        while child != -1:
            children.append(child)
            child = hierarchy[child][0]
        if not children:
            continue  # 实心黑圆没有孔洞。
        inner = contours[max(children, key=lambda i: cv2.contourArea(contours[i]))]
        hole_geometry = circle_geometry(inner, max(40, area * 0.08), shape_tolerance)
        if hole_geometry is None:
            continue
        hole_area, hole_center, hole_ellipse = hole_geometry
        # 孔洞必须足够大、不能几乎等于外圆，并且内外圆近似同心。
        if not 0.10 <= hole_area / area <= 0.98:
            continue
        if np.hypot(center[0] - hole_center[0], center[1] - hole_center[1]) > 0.18 * min(ellipse[1]) / 2:
            continue
        ratios = sorted(ellipse[1])
        hole_ratios = sorted(hole_ellipse[1])
        if abs(ratios[0] / ratios[1] - hole_ratios[0] / hole_ratios[1]) > 0.18:
            continue
        # 孔洞靠近边缘的区域应非黑色，中心允许有内层圆环或靶心。
        # 亮孔/环带的核验只涉及当前环的包围框，避免每个候选分配整帧掩膜。
        offset = np.array([x, y], dtype=np.int32).reshape(1, 1, 2)
        local_mask = mask[y:y + h, x:x + w]
        local_gray = gray[y:y + h, x:x + w]
        hole_strip = np.zeros((h, w), dtype=np.uint8)
        hole_origin = (hole_ellipse[0][0] - x, hole_ellipse[0][1] - y)
        for scale, value in ((0.88, 255), (0.68, 0)):
            scaled = (hole_origin, tuple(a * scale for a in hole_ellipse[1]), hole_ellipse[2])
            cv2.ellipse(hole_strip, scaled, value, -1)
        core_pixels = local_mask[hole_strip > 0]
        if core_pixels.size == 0 or np.mean(core_pixels == 0) < 0.65:
            continue
        # 核验外轮廓与孔洞间主要为黑色环带。
        band = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(band, [outer - offset], -1, 255, -1)
        cv2.drawContours(band, [inner - offset], -1, 0, -1)
        if not np.any(band) or np.mean(local_mask[band > 0] > 0) < 0.75:
            continue
        # 自适应阈值会把实心黑圆也分割成边缘环：用原图亮度再次核验。
        # 真圆环的孔洞边缘应明显比环带亮，不能仅依赖二值孔洞。
        if np.median(local_gray[hole_strip > 0]) - float(np.median(local_gray[band > 0])) < 12:
            continue
        candidates.append(dict(area=area, center=tuple(round(v) for v in center),
                               radius=radius, outer=outer, inner=inner, ellipse=ellipse))
    return candidates


def make_edges(frame, canny_low=40, canny_high=120, *, gray=None):
    if gray is None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
    return cv2.Canny(blurred, canny_low, canny_high)


def edge_observations(edges, min_area, shape_tolerance=0.20):
    """闭环和有断口的圆弧都可产生观测，不要求 findContours 找到孔洞。"""
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    observations = []
    for contour in contours:
        if len(contour) < 24:
            continue
        try:
            center, axes, angle = fit_contour(contour)
        except cv2.error:
            continue
        major, minor = max(axes), min(axes)
        if (not np.isfinite((*center, *axes, angle)).all() or minor < 8
                or minor / major < 0.25 or major > 2 * max(edges.shape)
                or np.pi * major * minor / 4 < min_area * 0.4):
            continue
        theta = np.deg2rad(angle)
        points = contour[:, 0, :].astype(float) - center
        u = (points[:, 0] * np.cos(theta) + points[:, 1] * np.sin(theta)) / (axes[0] / 2)
        v = (-points[:, 0] * np.sin(theta) + points[:, 1] * np.cos(theta)) / (axes[1] / 2)
        residual = residual_p90(abs(np.hypot(u, v) - 1))
        sectors = np.floor((np.arctan2(v, u) + np.pi) * 36 / (2 * np.pi)).astype(int) % 36
        coverage = len(np.unique(sectors)) / 36
        if residual > shape_tolerance or coverage < 0.4:
            continue
        observations.append(dict(center=np.asarray(center), axes=axes, angle=angle,
                                 major=major, aspect=minor / major, coverage=coverage))
    return observations


def cluster_observations(observations):
    """合并同一工位的内外边缘/多重同心环，尺度随画面中的环大小变化。"""
    groups = []
    for observation in sorted(observations, key=lambda item: item['coverage'], reverse=True):
        for group in groups:
            reference = group[0]
            tolerance = max(3.0, min(observation['major'], reference['major']) * 0.05)
            if (np.linalg.norm(observation['center'] - reference['center']) <= tolerance
                    and abs(observation['aspect'] - reference['aspect']) <= 0.15):
                group.append(observation)
                break
        else:
            groups.append([observation])
    return groups


def rings_from_edges(frame, edges, black_max, min_area, min_coverage, shape_tolerance=0.20,
                     *, gray=None, neutral=None):
    """沿拟合椭圆采样寻找黑色环带，可跨越断口，并核验孔内比环带亮。

    仅边缘像圆不足以判为黑环：每个候选必须同时具有黑色环带、亮的内侧
    和外侧。实心圆的内侧不亮，因此不能通过这条检测路径。
    """
    observations = edge_observations(edges, min_area, shape_tolerance)
    if gray is None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if neutral is None:
        neutral = neutral_mask(frame)
    height, width = gray.shape
    angles = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    candidates = []
    for group in cluster_observations(observations):
        center = np.median([item['center'] for item in group], axis=0)
        reference = max(group, key=lambda item: item['major'])
        theta = np.deg2rad(reference['angle'])
        rotation = np.array([[np.cos(theta), -np.sin(theta)],
                             [np.sin(theta), np.cos(theta)]])
        # 半径以长轴像素计，短轴与旋转由椭圆观测决定。
        directions = (np.column_stack((np.cos(angles), np.sin(angles)))
                      * (np.asarray(reference['axes']) / reference['major'])) @ rotation.T
        maximum_radius = max(item['major'] for item in group) / 2
        minimum_radius = np.sqrt(min_area / (np.pi * reference['aspect']))
        # 面积阈值变化时保持整像素采样位置一致，避免细同心环的明暗间隙跳变。
        radii = np.arange(max(2.0, float(np.floor(minimum_radius * 0.25))),
                          maximum_radius + max(12, maximum_radius * 0.25), 1.0)
        points = center + radii[:, None, None] * directions[None, :, :]
        map_x, map_y = points[:, :, 0].astype(np.float32), points[:, :, 1].astype(np.float32)
        samples = cv2.remap(gray, map_x, map_y, cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=255)
        neutral_samples = cv2.remap(neutral, map_x, map_y, cv2.INTER_NEAREST,
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        # 局部 Otsu 适应灰色/阴影背景；黑色上限仍限制可接受的环带亮度。
        otsu, _ = cv2.threshold(samples, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cutoff = min(black_max, otsu)
        dark = (samples <= cutoff) & (neutral_samples > 0)
        active = np.mean(dark, axis=1) >= min_coverage
        transitions = np.diff(np.r_[False, active, False].astype(np.int8))
        starts, ends = np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)
        for band_index, (start, end) in enumerate(zip(starts, ends)):
            inner_radius, outer_radius = radii[start] - 0.5, radii[end - 1] + 0.5
            band_width = outer_radius - inner_radius
            area = np.pi * outer_radius ** 2 * reference['aspect']
            if (end - start < 2 or area < min_area
                    or 2 * inner_radius * reference['aspect'] < 8
                    or not 0.10 <= (inner_radius / outer_radius) ** 2 <= 0.98):
                continue
            inside = (radii >= inner_radius * 0.70) & (radii <= inner_radius * 0.90)
            if band_index > 0:
                # 同心线靠近时，在相邻环带间的真实亮间隙核验内侧，避免
                # 将另一层黑线采进亮度统计而漏掉最外环。
                previous_outer = radii[ends[band_index - 1] - 1] + 0.5
                inside = ((radii > max(previous_outer + 0.5, inner_radius * 0.70))
                          & (radii < inner_radius - 0.5))
            outside = ((radii >= outer_radius + max(1.5, band_width * 0.25))
                       & (radii <= outer_radius + max(4, band_width)))
            if band_index + 1 < len(starts):
                outside &= radii < radii[starts[band_index + 1]] - 0.5
            if not np.any(inside) or not np.any(outside):
                continue
            band_gray = np.median(samples[start:end], axis=0).astype(float)
            inner_gray = np.median(samples[inside], axis=0).astype(float)
            outer_gray = np.median(samples[outside], axis=0).astype(float)
            supported = ((np.mean(dark[start:end], axis=0) >= 0.6)
                         & (inner_gray - band_gray >= 12) & (outer_gray - band_gray >= 8))
            coverage = float(np.mean(supported))
            if coverage < min_coverage:
                continue
            outer_points = center + outer_radius * directions
            if (outer_points[:, 0].min() <= 0 or outer_points[:, 1].min() <= 0
                    or outer_points[:, 0].max() >= width - 1
                    or outer_points[:, 1].max() >= height - 1):
                continue
            outer = np.rint(outer_points).astype(np.int32).reshape(-1, 1, 2)
            inner = np.rint(center + inner_radius * directions).astype(np.int32).reshape(-1, 1, 2)
            axes = tuple(float(a * 2 * outer_radius / reference['major']) for a in reference['axes'])
            ellipse = (tuple(float(v) for v in center), axes, reference['angle'])
            candidates.append(dict(area=float(area), center=tuple(round(v) for v in center),
                                   radius=float(outer_radius), outer=outer, inner=inner,
                                   ellipse=ellipse, method='edges', coverage=coverage))
    return candidates


def make_debug_view(frame, black_max=140, canny_low=40, canny_high=120):
    """从左到右：全局黑色、局部黑色、用于新检测路径的 Canny 边缘。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return np.hstack([*make_masks(frame, black_max, gray=gray),
                      make_edges(frame, canny_low, canny_high, gray=gray)])


def ellipse_shape(ellipse):
    """归一化形状矩阵，避免近圆椭圆的拟合角度跳变影响同心判断。"""
    _, axes, angle = ellipse
    theta = np.deg2rad(angle)
    rotation = np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])
    return rotation @ np.diag((np.asarray(axes) / max(axes)) ** 2) @ rotation.T


def merge_concentric_rings(candidates):
    """同心环和不同检测路径的重复观测归为一个目标。

    最外环决定目标面积/边界；多层观测的圆心中位数反馈坐标。
    分组始终对照最外环，避免邻近目标被链式合并。
    """
    groups = []
    for ring in sorted(candidates, key=lambda item: item['area'], reverse=True):
        for group in groups:
            reference = group[0]
            tolerance = max(3.0, min(min(ring['ellipse'][1]),
                                     min(reference['ellipse'][1])) * 0.06)
            distance = np.linalg.norm(np.asarray(ring['ellipse'][0]) - reference['ellipse'][0])
            if (distance <= tolerance
                    and np.linalg.norm(ellipse_shape(ring['ellipse'])
                                       - ellipse_shape(reference['ellipse'])) <= 0.30):
                group.append(ring)
                break
        else:
            groups.append([ring])
    targets = []
    for group in groups:
        outer_ring = group[0]
        center = tuple(round(v) for v in np.median([item['ellipse'][0] for item in group], axis=0))
        target = dict(outer_ring)
        target['center'] = center
        target['ellipse'] = (center, outer_ring['ellipse'][1], outer_ring['ellipse'][2])
        targets.append(target)
    return targets


def restore_ring_targets(targets, source_shape, work_shape):
    """把中心、轮廓、面积和椭圆换算回原始采集像素坐标。"""
    if source_shape[:2] == work_shape[:2]:
        return targets
    scale = np.array([source_shape[1] / work_shape[1], source_shape[0] / work_shape[0]])
    restored = []
    for item in targets:
        target = dict(item)
        target['center'] = tuple(int(v) for v in np.rint((np.asarray(item['center']) + .5) * scale - .5))
        for name in ('outer', 'inner'):
            target[name] = np.rint((item[name].astype(float) + .5) * scale - .5).astype(np.int32)
        center, axes, angle = item['ellipse']
        theta = np.deg2rad(angle)
        rotation = np.array([[np.cos(theta), -np.sin(theta)],
                             [np.sin(theta), np.cos(theta)]])
        # 宽高独立换算，处理奇数图像尺寸四舍五入造成的轻微比例差异。
        matrix = scale[:, None] * rotation @ np.diag(np.asarray(axes) ** 2) @ rotation.T * scale[None, :]
        eigenvalues, directions = np.linalg.eigh(matrix)
        new_axes = tuple(float(v) for v in np.sqrt(np.maximum(eigenvalues, 0)))
        new_angle = float(np.rad2deg(np.arctan2(directions[1, 0], directions[0, 0])))
        new_center = tuple(float(v) for v in (np.asarray(center) + .5) * scale - .5)
        target['ellipse'] = (new_center, new_axes, new_angle)
        target['radius'] = max(new_axes) / 2
        target['area'] = float(item['area'] * scale.prod())
        restored.append(target)
    return restored


def suppress_nearby_rings(targets, max_distance=NEARBY_RING_DISTANCE):
    """按面积从大到小筛选；距已保留圆心不超过阈值的小环不再输出。"""
    kept = []
    limit_squared = max_distance ** 2
    for ring in sorted(targets, key=lambda item: (-item['area'], *item['center'])):
        x, y = ring['center']
        if all((float(x) - other['center'][0]) ** 2 + (float(y) - other['center'][1]) ** 2
               > limit_squared for other in kept):
            kept.append(ring)
    return kept


def fast_ring_frame(frame, black_max=140, min_area=DEFAULT_MIN_AREA, *,
                    detect_width=800, shape_tolerance=.2, nearby_distance=NEARBY_RING_DISTANCE,
                    cutoff=None, **unused):
    """全幅当前帧的快速闭环检测；断环与不均匀光照由调用者的完整搜索补充。"""
    height, width = frame.shape[:2]
    work = frame
    if detect_width > 0 and width > detect_width:
        work = cv2.resize(frame, (detect_width, round(height*detect_width/width)),
                          interpolation=cv2.INTER_LINEAR)
    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
    neutral = neutral_mask(work)
    if cutoff is None:
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    else:
        # ROI callers share the current full-image Otsu threshold, so cropping
        # does not turn a bright paper background into a different segmentation.
        mask = cv2.inRange(gray, 0, cutoff)
    mask = cv2.bitwise_and(mask, cv2.bitwise_and(cv2.inRange(gray, 0, black_max), neutral))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    scaled_area = min_area * work.shape[1]/width * work.shape[0]/height
    targets = rings_from_mask(mask, scaled_area, gray, shape_tolerance)
    targets = restore_ring_targets(merge_concentric_rings(targets), frame.shape, work.shape)
    if nearby_distance > 0:
        targets = suppress_nearby_rings(targets, nearby_distance)
    targets.sort(key=lambda item: item['center'])
    for index, target in enumerate(targets, 1):
        target.update(id=index, method='fast-mask', coverage=1.)
    return targets


def analyze_ring_frame(frame, black_max=140, min_area=DEFAULT_MIN_AREA, *,
                       canny_low=40, canny_high=120, min_coverage=0.60, shape_tolerance=0.20,
                       detect_width=0, debug=False, nearby_distance=NEARBY_RING_DISTANCE):
    """单次检测返回原图坐标及可选调试图；所有路径共用同一组中间图。"""
    work = frame
    height, width = frame.shape[:2]
    if detect_width > 0 and width > detect_width:
        work = cv2.resize(frame, (detect_width, max(1, round(height * detect_width / width))),
                           interpolation=cv2.INTER_AREA)
    scaled_area = min_area * (work.shape[1] / width) * (work.shape[0] / height)
    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
    neutral = neutral_mask(work)
    masks = make_masks(work, black_max, gray=gray, neutral=neutral)
    joined = cv2.morphologyEx(masks[0], cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    fast = rings_from_mask(joined, scaled_area, gray, shape_tolerance)
    # 已通过黑带/亮孔核验的完整圆环无需再拟合每一层 Canny 边缘。
    # 只屏蔽它自身的椭圆内区，其他位置仍逐帧走断环与局部阈值路径。
    search_gray = gray.copy()
    remaining = [mask.copy() for mask in masks]
    for ring in merge_concentric_rings(fast):
        center, axes, angle = ring['ellipse']
        exclusion = (center, tuple(a + 10 for a in axes), angle)
        cv2.ellipse(search_gray, exclusion, 255, -1)
        for mask in remaining:
            cv2.ellipse(mask, exclusion, 0, -1)
    edges = make_edges(work, canny_low, canny_high, gray=search_gray)
    candidates = rings_from_edges(work, edges, black_max, scaled_area, min_coverage,
                                  shape_tolerance, gray=gray, neutral=neutral)
    # 保留原来的闭合孔洞方法处理低分辨率、细环等边缘观测不足的情形。
    # 两条路径通过黑色环带/孔内亮度核验后，统一合并同心观测。
    legacy = fast + [ring for mask in remaining
              for ring in rings_from_mask(mask, scaled_area, gray, shape_tolerance)]
    for ring in legacy:
        ring['method'] = 'mask'
        ring['coverage'] = 1.0
    candidates.extend(legacy)
    targets = restore_ring_targets(merge_concentric_rings(candidates), frame.shape, work.shape)
    # 使用换算后的原图坐标判断 100 像素距离，不随检测工作图缩放改变。
    if nearby_distance > 0:
        targets = suppress_nearby_rings(targets, nearby_distance)
    targets.sort(key=lambda item: item['center'])
    for index, ring in enumerate(targets, start=1):
        ring['id'] = index
    debug_image = None
    if debug:
        panels = [*masks, edges]
        if work.shape[1] > 333:
            size = (333, max(1, round(work.shape[0] * 333 / work.shape[1])))
            panels = [cv2.resize(panel, size, interpolation=cv2.INTER_NEAREST) for panel in panels]
        debug_image = np.hstack(panels)
    return targets, debug_image


def find_ring_targets(frame, black_max=140, min_area=DEFAULT_MIN_AREA, *,
                      canny_low=40, canny_high=120, min_coverage=0.60, shape_tolerance=0.20,
                      detect_width=0):
    """返回原图坐标下的全部独立圆环；接口默认原尺寸检测。"""
    return analyze_ring_frame(frame, black_max, min_area, canny_low=canny_low,
                               canny_high=canny_high, min_coverage=min_coverage,
                               shape_tolerance=shape_tolerance, detect_width=detect_width)[0]


def select_smallest_target(targets):
    """兼容按面积挑选最小独立圆环的调用。"""
    return min(targets, key=lambda ring: ring['area']) if targets else None


def find_smallest_ring(frame, black_max=140, min_area=DEFAULT_MIN_AREA, *,
                       canny_low=40, canny_high=120, min_coverage=0.60, shape_tolerance=0.20):
    targets = find_ring_targets(frame, black_max, min_area, canny_low=canny_low,
                                canny_high=canny_high, min_coverage=min_coverage,
                                shape_tolerance=shape_tolerance)
    return select_smallest_target(targets)


def add_ring_arguments(parser):
    parser.add_argument('--width', type=int, default=1280, help='摄像头请求宽度，默认 1280')
    parser.add_argument('--height', type=int, default=720, help='摄像头请求高度，默认 720')
    parser.add_argument('--fps', type=float, default=30, help='摄像头请求帧率，默认 30，以设备支持为准')
    parser.add_argument('--detect-width', type=int, default=800,
                        help='检测工作图最大宽度，默认 800；0 使用原始尺寸，采集分辨率不变')
    parser.add_argument('--debug-fps', type=float, default=10,
                        help='分割预览最高刷新频率，默认 10；圆环检测仍逐帧运行')
    parser.add_argument('--black-max', type=int, default=140, help='黑色灰度上限 0..254')
    parser.add_argument('--min-area', type=float, default=DEFAULT_MIN_AREA,
                        help=f'外环最小面积（像素平方），默认 {DEFAULT_MIN_AREA}；增大可过滤小目标')
    parser.add_argument('--canny-low', type=int, default=40, help='Canny 边缘低阈值')
    parser.add_argument('--canny-high', type=int, default=120, help='Canny 边缘高阈值')
    parser.add_argument('--min-coverage', type=float, default=0.60,
                        help='边缘路径中黑环与内外明暗关系的最小圆周支持比例 0.40..1.00')
    parser.add_argument('--shape-tolerance', type=float, default=0.20,
                        help='允许的椭圆归一化径向误差 0.05..0.30，增大可容忍更多畸变')


def validate_ring_arguments(parser, args):
    if args.width <= 0 or args.height <= 0:
        parser.error('--width and --height must be positive integers')
    if not np.isfinite(args.fps) or args.fps <= 0:
        parser.error('--fps must be a positive finite number')
    if args.detect_width != 0 and args.detect_width < 160:
        parser.error('--detect-width must be 0 or at least 160')
    if not np.isfinite(args.debug_fps) or args.debug_fps <= 0:
        parser.error('--debug-fps must be a positive finite number')
    if (not 0 <= args.black_max < 255 or not np.isfinite(args.min_area) or args.min_area <= 0
            or not 0 <= args.canny_low < args.canny_high <= 255
            or not 0.40 <= args.min_coverage <= 1.0
            or not 0.05 <= args.shape_tolerance <= 0.30):
        parser.error('black-max: 0..254; min-area: positive; '
                     '0 <= canny-low < canny-high <= 255; min-coverage: 0.40..1.00; '
                     'shape-tolerance: 0.05..0.30')


def ring_options(args):
    return dict(black_max=args.black_max, min_area=args.min_area, canny_low=args.canny_low,
                canny_high=args.canny_high, min_coverage=args.min_coverage,
                shape_tolerance=args.shape_tolerance)


def validate_ring_settings(settings):
    """核验 JSON 参数；允许只填写部分检测阈值，其余使用默认值。"""
    if not isinstance(settings, dict):
        raise ValueError('Threshold configuration must be a JSON object')
    unknown = set(settings) - set(RING_DEFAULTS)
    if unknown:
        raise ValueError('Unknown threshold keys: ' + ', '.join(sorted(unknown)))
    values = {**RING_DEFAULTS, **settings}
    for key in ('black_max', 'canny_low', 'canny_high'):
        if type(values[key]) is not int:
            raise ValueError(f'{key} must be an integer')
    for key in ('min_area', 'min_coverage', 'shape_tolerance'):
        if type(values[key]) not in (int, float) or not np.isfinite(values[key]):
            raise ValueError(f'{key} must be a finite number')
    if not 0 <= values['black_max'] <= 254:
        raise ValueError('black_max must be 0..254')
    if values['min_area'] <= 0:
        raise ValueError('min_area must be positive')
    if not 0 <= values['canny_low'] < values['canny_high'] <= 255:
        raise ValueError('Require 0 <= canny_low < canny_high <= 255')
    if not 0.40 <= values['min_coverage'] <= 1.0:
        raise ValueError('min_coverage must be 0.40..1.00')
    if not 0.05 <= values['shape_tolerance'] <= 0.30:
        raise ValueError('shape_tolerance must be 0.05..0.30')
    return values


def load_ring_settings(path):
    path = Path(path)
    if not path.exists():
        return {}
    return validate_ring_settings(json.loads(path.read_text(encoding='utf-8-sig')))


def save_ring_settings(path, settings):
    values = validate_ring_settings(settings)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(values, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def create_debug_controls(args):
    cv2.namedWindow('Black Mask', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Black Mask', 1000, 480)
    for name, value, maximum in [('Black max', args.black_max, 254),
                                  ('Min area px2', max(1, round(args.min_area)),
                                   max(100000, round(args.min_area))),
                                  ('Canny low', args.canny_low, 254),
                                  ('Canny high', args.canny_high, 255),
                                  ('Coverage %', round(args.min_coverage * 100), 100),
                                  ('Shape tol %', round(args.shape_tolerance * 100), 30)]:
        cv2.createTrackbar(name, 'Black Mask', value, maximum, lambda _: None)


def read_debug_controls(args):
    names = ('Black max', 'Min area px2', 'Canny low', 'Canny high', 'Coverage %', 'Shape tol %')
    positions = {name: cv2.getTrackbarPos(name, 'Black Mask') for name in names}
    args.black_max = positions['Black max']
    args.min_area = max(1, positions['Min area px2'])
    args.canny_low = positions['Canny low']
    args.canny_high = max(args.canny_low + 1, positions['Canny high'])
    args.min_coverage = max(0.40, positions['Coverage %'] / 100)
    args.shape_tolerance = max(0.05, positions['Shape tol %'] / 100)
    # 只在纠正越界值时设置滑块，避免每帧触发原生窗口重复刷新。
    for name, value in [('Min area px2', args.min_area), ('Canny high', args.canny_high),
                         ('Coverage %', round(args.min_coverage * 100)),
                         ('Shape tol %', round(args.shape_tolerance * 100))]:
        if positions[name] != value:
            cv2.setTrackbarPos(name, 'Black Mask', value)


def set_debug_controls(args):
    for name, value in [('Black max', args.black_max),
                         ('Min area px2', max(1, round(args.min_area))),
                         ('Canny low', args.canny_low), ('Canny high', args.canny_high),
                         ('Coverage %', round(args.min_coverage * 100)),
                         ('Shape tol %', round(args.shape_tolerance * 100))]:
        cv2.setTrackbarPos(name, 'Black Mask', value)


def ring_reports(frame, targets, digit_results=None):
    """每个独立目标各反馈一组坐标；编号按当前帧从左到右排列。"""
    height, width = frame.shape[:2]
    reports = []
    for index, ring in enumerate(targets, start=1):
        info = dict(id=ring.get('id', index), center=ring['center'],
                    dx=ring['center'][0] - width // 2, dy=ring['center'][1] - height // 2,
                    area=round(ring['area'], 1), diameter=round(ring['radius'] * 2, 1))
        if digit_results is not None:
            info.update(digit_results.get(info['id'], dict(digit=None, confidence=0.0, candidate=None, ocr_ms=0.0)))
        reports.append(info)
    return reports


def draw_ring_targets(frame, targets, selected=None, extra_lines=(), digit_results=None):
    """在原始大小的画面上标出全部目标、编号、圆心和每个目标的 dx/dy。"""
    height, width = frame.shape[:2]
    origin = (width // 2, height // 2)
    header_lines = [f'Rings: {len(targets)} | Frame: {width}x{height} | Center: {origin}', *extra_lines]
    if not targets:
        header_lines.append('No black ring')
    header_height = 8 + 26 * len(header_lines)
    palette = ((0, 255, 0), (255, 200, 0), (255, 100, 255), (0, 180, 255))
    cv2.drawMarker(frame, origin, (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
    for index, (ring, info) in enumerate(zip(targets, ring_reports(frame, targets))):
        is_selected = selected is ring
        color = (0, 255, 255) if is_selected else palette[index % len(palette)]
        center = ring['center']
        # 每组同心环只绘制一次最外边界。
        cv2.drawContours(frame, [ring['outer']], -1, color, 2)
        cv2.circle(frame, center, 4, color, -1)
        cv2.line(frame, origin, center, color, 1)
        marker = '*' if is_selected else ''
        labels = [f'R{info["id"]}{marker} center: {center}',
                  f'dx: {info["dx"]:+d}   dy: {info["dy"]:+d} px']
        if digit_results is not None:
            result = digit_results.get(info['id'], {})
            if result.get('digit') is not None:
                labels.append(f'Digit: {result["digit"]}  confidence: {result["confidence"]:.2f}')
            else:
                labels.append('Digit: confirming...' if result.get('candidate') else 'Digit: --')
        font_scale = min(0.5, max(0.2, (width - 16) / max(
            cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0] for text in labels)))
        text_width = max(cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX,
                                        font_scale, 1)[0][0] for text in labels)
        x, y, _, _ = cv2.boundingRect(ring['outer'])
        label_x = max(4, min(x, width - text_width - 10))
        label_y = max(header_height + 18, min(y - 26, height - 8 - 20 * (len(labels) - 1)))
        cv2.rectangle(frame, (label_x - 3, label_y - 17),
                      (label_x + text_width + 4, min(height - 1, label_y + 20 * (len(labels) - 1) + 4)), (0, 0, 0), -1)
        for row, text in enumerate(labels):
            cv2.putText(frame, text, (label_x, label_y + 20 * row),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1, cv2.LINE_AA)
    # 只在顶部叠加文字，不附加或缩放画面。
    cv2.rectangle(frame, (0, 0), (width - 1, header_height), (0, 0, 0), -1)
    for row, text in enumerate(header_lines):
        font_scale = min(0.58, max(0.2, (width - 16) / max(
            1, cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0])))
        cv2.putText(frame, text, (8, 24 + row * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


def draw_result(frame, ring):
    """兼容旧的单目标调用；实时入口使用 draw_ring_targets 显示全部目标。"""
    return draw_ring_targets(frame, [] if ring is None else [ring], selected=ring)


def open_camera(index, width=1280, height=720, fps=30):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW if os.name == 'nt' else cv2.CAP_ANY)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'Cannot open camera {index}')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    # 部分后端不支持此属性；支持时减少待处理旧帧。
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # DirectShow 在修改尺寸或帧率时会重新协商格式，可能把 MJPG 重置为
    # YUY2。必须最后设置 FOURCC；本机 1080p 的实测由约 5 FPS 恢复到 30 FPS。
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    return cap


def camera_fourcc(cap):
    code = int(cap.get(cv2.CAP_PROP_FOURCC))
    return ''.join(chr((code >> (8 * index)) & 255) for index in range(4)).strip('\x00') or 'unknown'


def parse_arguments(argv=None, *, tune=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=1)
    add_ring_arguments(parser)
    controls = parser.add_mutually_exclusive_group()
    controls.add_argument('--debug', dest='debug', action='store_true',
                          help='显示圆环阈值滑块及分割预览（默认开启）')
    controls.add_argument('--no-debug', dest='debug', action='store_false',
                          help='隐藏阈值滑块及分割预览，仅显示识别画面')
    parser.set_defaults(debug=True)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                        help='阈值 JSON 路径，默认读取本目录 ring_thresholds.json')
    parser.add_argument('--no-config', action='store_true', help='跳过读取已保存的阈值')
    digits = parser.add_mutually_exclusive_group()
    digits.add_argument('--digits', dest='digits', action='store_true', help='启用圆环中心数字识别（默认开启）')
    digits.add_argument('--no-digits', dest='digits', action='store_false', help='只识别圆环，跳过数字模型')
    parser.set_defaults(digits=True)
    parser.add_argument('--confidence', type=float, default=0.60, help='数字 2/3 的最低置信度，默认 0.60')
    parser.add_argument('--confidence-one', type=float, default=0.30, help='数字 1 的最低置信度，默认 0.30')
    parser.add_argument('--digit-roi-scale', type=float, default=0.75,
                        help='中心识别区相对孔洞直径的比例，0.30..0.95，默认 0.75')
    parser.add_argument('--digit-interval', type=float, default=0.06, help='后台数字任务最小启动间隔（秒），默认 0.06')
    parser.add_argument('--digit-confirm', type=int, default=3, help='数字确认所需的最少匹配次数，默认 3')
    parser.add_argument('--digit-window', type=int, default=7, help='数字投票窗口长度，默认 7')
    parser.add_argument('--digit-hold', type=float, default=0.6, help='数字缺少匹配结果时的最长保留秒数，默认 0.6')
    parser.add_argument('--image', help='离线图片路径')
    parser.add_argument('--output', help='与 --image 一起使用，保存结果后退出，不弹窗')
    args = parser.parse_args(argv)
    if not args.no_config:
        try:
            settings = load_ring_settings(args.config)
        except (OSError, ValueError) as exc:
            parser.error(f'Cannot load thresholds from {args.config}: {exc}')
        # 命令行显式指定的值优先于已保存的阈值；--help 不依赖配置能否读取。
        parser.set_defaults(**settings)
        args = parser.parse_args(argv)
    validate_ring_arguments(parser, args)
    numeric = (args.confidence, args.confidence_one, args.digit_roi_scale, args.digit_interval, args.digit_hold)
    if (not np.isfinite(numeric).all() or not 0 <= args.confidence <= 1
            or not 0 <= args.confidence_one <= 1 or not 0.30 <= args.digit_roi_scale <= 0.95
            or args.digit_interval < 0 or args.digit_confirm < 1 or args.digit_window < args.digit_confirm
            or args.digit_hold <= 0):
        parser.error('Invalid digit confidence, ROI scale, interval, vote window or hold time')
    if args.output and not args.image:
        parser.error('--output 需要和 --image 一起使用')
    return args


def save_image(path, image):
    path = Path(path)
    ok, encoded = cv2.imencode(path.suffix or '.png', image)
    if not ok:
        raise RuntimeError('Cannot encode image')
    encoded.tofile(str(path))


def main(argv=None, *, tune=False):
    args = parse_arguments(argv, tune=tune)
    cv2.setNumThreads(max(1, min(4, os.cpu_count() or 1)))
    source = None
    if args.image:
        source = cv2.imdecode(np.fromfile(args.image, dtype=np.uint8), cv2.IMREAD_COLOR)
        if source is None:
            raise RuntimeError('Cannot read image')
    cap, digit_worker = None, None
    window = 'Black Ring Targets'
    failures = 0
    resolution_reported = False
    last_tick, fps, last_report = time.monotonic(), 0.0, 0.0
    status, status_until = '', 0.0
    next_debug, last_debug_options = 0.0, None
    try:
        if args.digits:
            # 识别与调参共用入口；显式 --no-digits 时不加载已有 OCR 模型。
            from ring_digit import RingDigitRecognizer
            digit_worker = RingDigitRecognizer(args)
        if args.output:
            targets = find_ring_targets(source, **ring_options(args), detect_width=args.detect_width)
            digits = digit_worker.update(source, targets, synchronous=True) if digit_worker is not None else None
            print(dict(ring_count=len(targets), rings=ring_reports(source, targets, digits)), flush=True)
            save_image(args.output, draw_ring_targets(source.copy(), targets, digit_results=digits))
            return
        cap = open_camera(args.camera, args.width, args.height, args.fps) if source is None else None
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(window, 1280, 720)
        if args.debug:
            create_debug_controls(args)
        shortcuts = 'S/P: save thresholds; F: snapshot; R: defaults'
        print(f'{shortcuts}; C: camera 0/1; Q/Esc: exit.', flush=True)
        print(f'Threshold file: {args.config.resolve()}\nThresholds: {ring_options(args)}\n'
              f'Detection max width: {args.detect_width or "original"}; coordinates use original frame.', flush=True)
        if digit_worker is not None:
            print(f'Digits 1/2/3 near each ring center; confidence 1={args.confidence_one:g}, '
                  f'2/3={args.confidence:g}; background OCR, center scale={args.digit_roi_scale:g}.', flush=True)
        while True:
            ok, frame = cap.read() if cap is not None else (True, source)
            if not ok:
                failures += 1
                if failures >= 10:
                    raise RuntimeError('Camera read failed: check USB connection or other camera programs.')
                if cv2.waitKey(100) & 0xFF in (27, ord('q')):
                    break
                continue
            failures = 0
            if cap is not None and not resolution_reported:
                codec = camera_fourcc(cap)
                print(f'Camera {args.camera}: requested {args.width}x{args.height}, '
                      f'actual frame {frame.shape[1]}x{frame.shape[0]}; '
                      f'format {codec}; requested FPS {args.fps:g}, '
                      f'driver FPS {cap.get(cv2.CAP_PROP_FPS):g}', flush=True)
                if codec not in ('MJPG', 'JPEG'):
                    print(f'Camera did not select MJPG (actual {codec}); USB bandwidth may limit FPS.', flush=True)
                resolution_reported = True
            if args.debug:
                if cv2.getWindowProperty('Black Mask', cv2.WND_PROP_VISIBLE) < 1:
                    break
                read_debug_controls(args)
            options = ring_options(args)
            tick = time.monotonic()
            refresh_debug = args.debug and (tick >= next_debug or options != last_debug_options)
            targets, debug_image = analyze_ring_frame(frame, **options, detect_width=args.detect_width,
                                                       debug=refresh_debug)
            digits = (digit_worker.update(frame, targets, synchronous=source is not None)
                      if digit_worker is not None else None)
            if debug_image is not None:
                cv2.imshow('Black Mask', debug_image)
                next_debug = time.monotonic() + 1 / args.debug_fps
                last_debug_options = options
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(0.001, now - last_tick)
            last_tick = now
            work_width = min(frame.shape[1], args.detect_width) if args.detect_width else frame.shape[1]
            lines = [f'FPS {fps:.1f} | {codec} | Detect width {work_width}' if cap is not None else
                     f'Offline image | Detect width {work_width}']
            if digit_worker is not None:
                lines.append(f'Digits 1/2/3 | Center ROI {args.digit_roi_scale:.2f} | '
                             f'OCR {digit_worker.latency:.1f} ms | background')
            if args.debug:
                lines.extend((f'Black max {args.black_max} | Min area {args.min_area:g} px2 | '
                              f'Canny {args.canny_low}/{args.canny_high}',
                              f'Coverage {args.min_coverage:.2f} | Shape tol {args.shape_tolerance:.2f}'))
            lines.append(status if now < status_until else shortcuts + '; Q/Esc: exit')
            cv2.imshow(window, draw_ring_targets(frame.copy(), targets, extra_lines=lines, digit_results=digits))
            if now - last_report >= 0.5:
                print(dict(ring_count=len(targets), rings=ring_reports(frame, targets, digits)), flush=True)
                last_report = now
            # 实时相机主动轮询窗口事件；离线图片保留等待/滑块刷新逻辑。
            key = (cv2.pollKey() if cap is not None else cv2.waitKey(30 if args.debug else 0)) & 0xFF
            if key in (27, ord('q')) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord('p'), ord('s')):
                try:
                    save_ring_settings(args.config, ring_options(args))
                    status = 'Thresholds saved'
                    print(f'Saved thresholds: {args.config.resolve()}', flush=True)
                except (OSError, ValueError) as exc:
                    status = 'Save failed; see terminal'
                    print(f'Cannot save thresholds: {exc}', flush=True)
                status_until = time.monotonic() + 5
            if key == ord('r'):
                for name, value in RING_DEFAULTS.items():
                    setattr(args, name, value)
                if args.debug:
                    set_debug_controls(args)
                status, status_until = 'Defaults restored (not saved)', time.monotonic() + 5
            if key == ord('f'):
                folder = Path(__file__).resolve().parent / 'captures'
                folder.mkdir(exist_ok=True)
                stamp = str(time.time_ns())
                for name, image in [('frame', frame), ('mask', make_debug_view(
                        frame, args.black_max, args.canny_low, args.canny_high))]:
                    save_image(folder / (stamp + '_' + name + '.png'), image)
                print('Saved frame/masks to captures.', flush=True)
            if key == ord('c') and cap is not None:
                if digit_worker is not None:
                    digit_worker.reset()
                cap.release()
                candidate = 0 if args.camera == 1 else 1
                try:
                    cap = open_camera(candidate, args.width, args.height, args.fps)
                    args.camera = candidate
                except RuntimeError as exc:
                    print(exc, flush=True)
                    cap = open_camera(args.camera, args.width, args.height, args.fps)
                resolution_reported = False
                last_tick, fps = time.monotonic(), 0.0
                next_debug = 0.0
    finally:
        if cap is not None:
            cap.release()
        if digit_worker is not None:
            digit_worker.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
