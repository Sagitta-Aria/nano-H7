"""六个单颜色程序共用的 HSV 检测、滑块调参与参数保存功能。"""
import argparse
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np


def validate_settings(settings, color, range_count):
    if not isinstance(settings, dict) or settings.get("color") != color:
        raise ValueError(f"参数文件的 color 必须是 {color}")
    min_area = settings.get("min_area")
    if type(min_area) is not int or min_area < 1:
        raise ValueError("min_area 必须是大于 0 的整数")
    ranges = settings.get("ranges")
    if not isinstance(ranges, (list, tuple)) or len(ranges) != range_count:
        raise ValueError(f"ranges 必须有 {range_count} 组 HSV 上下限")
    for pair in ranges:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError("每组阈值应为 [[H下限,S下限,V下限],[H上限,S上限,V上限]]")
        for bound in pair:
            if not isinstance(bound, (list, tuple)) or len(bound) != 3:
                raise ValueError("每个 HSV 阈值必须有三个整数")
            if any(type(v) is not int or not 0 <= v <= limit
                   for v, limit in zip(bound, (179, 255, 255))):
                raise ValueError("H 范围为 0–179，S/V 范围为 0–255")
        if any(low > high for low, high in zip(*pair)):
            raise ValueError("HSV 下限不能大于上限")
    return settings


def default_settings(color, ranges, min_area):
    return {"color": color, "min_area": min_area,
            "ranges": [[list(lower), list(upper)] for lower, upper in ranges]}


def load_settings(path, color, ranges, min_area):
    if not path.exists():
        return default_settings(color, ranges, min_area)
    settings = json.loads(path.read_text(encoding="utf-8-sig"))
    return validate_settings(settings, color, len(ranges))


def save_settings(path, settings):
    validate_settings(settings, settings["color"], len(settings["ranges"]))
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def make_mask(frame, ranges):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def find_block(mask, min_area, *, largest=False):
    """在达到面积下限的色块中选择最大或最小目标。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        if best is not None and (area <= best[0] if largest else area >= best[0]):
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        center = (round(moments["m10"] / moments["m00"]),
                  round(moments["m01"] / moments["m00"]))
        best = (area, center, contour)
    return best


def draw_result(frame, result, color, status):
    height, width = frame.shape[:2]
    frame_center = (width // 2, height // 2)
    cv2.drawMarker(frame, frame_center, (255, 255, 255), cv2.MARKER_CROSS, 25, 2)
    if result is None:
        target_text, offset_text = "No color block", "dx: --   dy: --"
    else:
        area, center, contour = result
        dx, dy = center[0] - frame_center[0], center[1] - frame_center[1]
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
        cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.line(frame, frame_center, center, (0, 255, 255), 2)
        target_text = f"center: {center}  area: {area:.0f}px^2"
        offset_text = f"dx: {dx:+d} px   dy: {dy:+d} px"
    # 文字直接叠加，不拼接面板或改变原始画面的宽高。
    cv2.rectangle(frame, (0, 0), (width - 1, 125), (0, 0, 0), -1)
    lines = [f"{color} | frame center: {frame_center}", target_text, offset_text, status]
    for row, text in enumerate(lines):
        scale = min(0.62, max(0.2, (width - 20) / max(1, cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0])))
        cv2.putText(frame, text, (10, 25 + row * 29), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 255, 0) if row in (1, 2) else (255, 255, 255), 1,
                    cv2.LINE_AA)
    return frame


class ThresholdControls:
    CHANNELS = ("H min", "S min", "V min", "H max", "S max", "V max")

    def __init__(self, window, settings):
        self.window = window
        self.settings = settings
        self.active_range = 0
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 540, 560)
        if len(settings["ranges"]) > 1:
            cv2.createTrackbar("Range 0/1", window, 0, len(settings["ranges"]) - 1,
                               lambda _: None)
        values = settings["ranges"][0][0] + settings["ranges"][0][1]
        for name, value, limit in zip(self.CHANNELS, values, (179, 255, 255) * 2):
            cv2.createTrackbar(name, window, value, limit, lambda _: None)
        cv2.createTrackbar("Min area", window, settings["min_area"],
                           max(20000, settings["min_area"]), lambda _: None)

    def show_range(self):
        lower, upper = self.settings["ranges"][self.active_range]
        for name, value in zip(self.CHANNELS, lower + upper):
            cv2.setTrackbarPos(name, self.window, value)

    def read(self):
        values = [cv2.getTrackbarPos(name, self.window) for name in self.CHANNELS]
        lower, upper = values[:3], values[3:]
        # 滑块交叉时将上限抬到下限，避免得到上下限颠倒的无效区间。
        for index in range(3):
            if upper[index] < lower[index]:
                upper[index] = lower[index]
                cv2.setTrackbarPos(self.CHANNELS[index + 3], self.window, upper[index])
        self.settings["ranges"][self.active_range] = [lower, upper]
        if len(self.settings["ranges"]) > 1:
            selected = cv2.getTrackbarPos("Range 0/1", self.window)
            if selected != self.active_range:
                self.active_range = selected
                self.show_range()
        min_area = max(1, cv2.getTrackbarPos("Min area", self.window))
        self.settings["min_area"] = min_area
        cv2.setTrackbarPos("Min area", self.window, min_area)
        return self.settings

    def reset(self, ranges, min_area):
        self.settings = default_settings(self.settings["color"], ranges, min_area)
        self.show_range()
        cv2.setTrackbarPos("Min area", self.window, min_area)

    def display(self):
        panel = np.zeros((85, 540, 3), dtype=np.uint8)
        lines = [f"{self.settings['color']} | editing range {self.active_range}",
                 "All ranges are combined (OR). White mask = selected.",
                 "S: save  R: reset  C: camera  Q/Esc: quit"]
        for row, text in enumerate(lines):
            cv2.putText(panel, text, (10, 22 + 25 * row), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(self.window, panel)


def open_camera(index):
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"无法打开摄像头 {index}")
    return cap


def run_color(color, ranges, default_min_area=500, *, largest=False):
    selection = "最大" if largest else "最小"
    parser = argparse.ArgumentParser(description=f"{color} {selection}色块识别与 HSV 滑块调参")
    parser.add_argument("--camera", type=int, default=1, help="USB 摄像头编号，默认 1")
    parser.add_argument("--min-area", type=int, help="覆盖已保存的最小面积，须大于 0")
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).with_name(f"{color}_thresholds.json"),
                        help="参数文件路径；默认使用本颜色独立的 JSON 文件")
    args = parser.parse_args()
    if args.min_area is not None and args.min_area < 1:
        parser.error("--min-area 必须大于 0")
    try:
        settings = load_settings(args.config, color, ranges, default_min_area)
    except (OSError, ValueError) as exc:
        parser.error(f"无法读取阈值参数 {args.config}: {exc}")
    if args.min_area is not None:
        settings["min_area"] = args.min_area

    preview, mask_window, controls_window = f"{color} - Camera", f"{color} - Mask", f"{color} - HSV"
    cap = open_camera(args.camera)
    try:
        cv2.namedWindow(preview, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(mask_window, cv2.WINDOW_AUTOSIZE)
        controls = ThresholdControls(controls_window, settings)
        print(f"当前颜色：{color}；参数文件：{args.config.resolve()}", flush=True)
        print(f"目标选择：面积{selection}的有效色块。", flush=True)
        print("拖动 HSV / Min area 滑块实时调参。S 保存，R 恢复默认，C 切换摄像头，Q/Esc 退出。", flush=True)
        notice, notice_until, failures = "", 0, 0
        while True:
            if any(cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1
                   for window in (preview, mask_window, controls_window)):
                break
            settings = controls.read()
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures >= 10:
                    raise RuntimeError("连续读取画面失败，请检查摄像头连接或是否被占用")
            else:
                failures = 0
                mask = make_mask(frame, settings["ranges"])
                result = find_block(mask, settings["min_area"], largest=largest)
                status = notice if time.monotonic() < notice_until else "S: save  R: reset  C: camera  Q: quit"
                cv2.imshow(preview, draw_result(frame, result, color, status))
                cv2.imshow(mask_window, mask)
            controls.display()
            key = cv2.waitKey(1 if ok else 100) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                settings = controls.read()
                try:
                    save_settings(args.config, settings)
                    notice = "Thresholds saved"
                    print(f"已保存：{args.config.resolve()}", flush=True)
                except OSError as exc:
                    notice = "Save failed - see console"
                    print(f"保存失败：{exc}", flush=True)
                notice_until = time.monotonic() + 3
            elif key in (ord("r"), ord("R")):
                controls.reset(ranges, default_min_area)
                notice, notice_until = "Defaults restored (press S to save)", time.monotonic() + 3
            elif key in (ord("c"), ord("C")):
                cap.release()
                candidate = 0 if args.camera == 1 else 1
                try:
                    cap = open_camera(candidate)
                    args.camera = candidate
                except RuntimeError as exc:
                    print(exc, flush=True)
                    cap = open_camera(args.camera)
                failures = 0
                print(f"当前摄像头：{args.camera}", flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("请运行 red_block.py、yellow_block.py、blue_block.py、green_block.py、black_block.py 或 light_blue_block.py。")


# Compatibility for offline tools; calibrated JSON is the source of each range.
COLOR_RANGES = {name: json.loads(Path(__file__).with_name(name + '_thresholds.json').read_text(
    encoding='utf-8-sig'))['ranges'] for name in ('red','green','blue','yellow','black','light_blue')}


def find_color_block(frame, min_area=500):
    """Legacy all-color result shape; new contour selection is preserved per color."""
    candidates = []
    for name, ranges in COLOR_RANGES.items():
        found = find_block(make_mask(frame, ranges), min_area, largest=name == 'black')
        if found:
            candidates.append((found[0], name, found[1], found[2]))
    return min(candidates, key=lambda item: item[0]) if candidates else None
