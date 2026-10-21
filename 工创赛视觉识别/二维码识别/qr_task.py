"""二维码识别与屏幕显示。

支持 ``123+321+456+654`` 等多组三位数字，保留分组和前导零。
仅识别显示，不包含串口通信。
"""
import argparse
import os
import re
import time
from typing import List

import cv2
import numpy as np


def parse_groups(text: str) -> List[str]:
    text = text.strip()
    if re.fullmatch(r"[0-9]{3}(?:\s*\+\s*[0-9]{3})+", text):
        return [group.strip() for group in text.split('+')]
    # 兼容此前的六位/逗号分隔颜色任务。
    if re.fullmatch(r"(?:[1-3]{6}|[1-3](?:\s*,\s*[1-3]){5})", text):
        digits = ''.join(re.findall(r'[1-3]', text))
        return [digits[:3], digits[3:]]
    raise ValueError("任务应为以 + 分隔的多组三位数字，例如 123+321+456+654")


def parse_task(text: str) -> List[int]:
    return [int(digit) for group in parse_groups(text) for digit in group]


def decode_frame(detector, frame):
    try:
        text, points, _ = detector.detectAndDecode(frame)
        return text, points, False
    except cv2.error:
        # OpenCV 可能在特定帧的候选轮廓上异常；尝试灰度，仍失败则等待下一帧。
        try:
            text, points, _ = detector.detectAndDecode(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            return text, points, False
        except cv2.error:
            return '', None, True


def render_result(frame, text, points, last_result, decode_error=False):
    """状态每帧更新；最后成功结果单独保留，避免将历史结果当作当前检测。"""
    if text:
        try:
            groups = parse_groups(text)
        except ValueError:
            status, color = 'QR decoded / INVALID task format', (0, 0, 255)
            result_line = 'Text: ' + text
        else:
            status, color = 'QR decoded / VALID', (0, 255, 0)
            result_line = 'Result: ' + '+'.join(groups)
    else:
        status = 'QR located / decoding...' if points is not None else 'No QR detected'
        color = (0, 200, 255)
        result_line = 'Last valid: ' + last_result if last_result else 'Result: --'
    if decode_error:
        status, color = 'Decode error / retrying next frame', (0, 0, 255)
    preview = frame.copy()
    if points is not None:
        cv2.polylines(preview, [points.reshape(-1, 2).astype(np.int32)], True, color, 2)
    # 直接叠加在原始帧顶部，不拼接面板，也不改变帧尺寸或比例。
    height, width = preview.shape[:2]
    top = min(70, height)
    preview[:top] = cv2.addWeighted(preview[:top], 0.35, np.zeros_like(preview[:top]), 0.65, 0)
    for row, line in enumerate((status, result_line)):
        line = line.encode('ascii', 'replace').decode('ascii')
        text_width = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0][0]
        scale = 0.7 * min(1.0, max(1, width - 20) / max(1, text_width))
        cv2.putText(preview, line, (10, 26 + 32 * row), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color if row == 0 else (255, 255, 255), 2, cv2.LINE_AA)
    return preview


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="0", help="摄像头编号或设备路径")
    ap.add_argument("--image", help="离线二维码图片，显示识别情况与结果")
    args = ap.parse_args()
    detector = cv2.QRCodeDetector()
    if args.image:
        frame = cv2.imdecode(np.fromfile(args.image, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise SystemExit("无法读取图片")
        text, points, decode_error = decode_frame(detector, frame)
        print("二维码内容:", text, flush=True)
        try:
            last_result = '+'.join(parse_groups(text)) if text else None
        except ValueError:
            last_result = None
        cv2.namedWindow('qr_task', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('qr_task', render_result(frame, text, points, last_result, decode_error))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return
    source = int(args.camera) if args.camera.isdigit() else args.camera
    backend = cv2.CAP_DSHOW if os.name == "nt" and isinstance(source, int) else cv2.CAP_ANY
    cap = cv2.VideoCapture(source, backend)
    if not cap.isOpened():
        cap.release()
        raise SystemExit(f"无法打开摄像头: {args.camera}")
    last = None
    failures = 0
    last_result = None
    cv2.namedWindow('qr_task', cv2.WINDOW_AUTOSIZE)
    print(f'Camera {source}; Q/Esc: exit.', flush=True)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures >= 10:
                    raise RuntimeError("摄像头连续读取失败，请关闭其他占用摄像头的程序")
                if cv2.waitKey(100) & 0xFF in (27, ord('q')):
                    break
                continue
            failures = 0
            text, points, decode_error = decode_frame(detector, frame)
            if text and text != last:
                print("二维码内容:", text, flush=True)
                try:
                    groups = parse_groups(text)
                except ValueError as exc:
                    print(exc, flush=True)
                else:
                    last_result = '+'.join(groups)
                    print("分组结果:", groups, flush=True)
                last = text
            cv2.imshow("qr_task", render_result(frame, text, points, last_result, decode_error))
            if cv2.waitKey(1) & 0xFF in (27, ord('q')) or cv2.getWindowProperty('qr_task', cv2.WND_PROP_VISIBLE) < 1:
                break
            time.sleep(0.01)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
