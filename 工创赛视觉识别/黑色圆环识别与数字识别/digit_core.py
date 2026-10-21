"""只识别单个 1/2/3，使用 RapidOCR 随包提供的 PP-OCRv3 ONNX 模型。"""
from collections import Counter, deque
from importlib.util import find_spec
from pathlib import Path
import time

import cv2
import numpy as np


class DigitEngine:
    """只加载文字识别模型，限制 CPU 线程，避免与圆环检测争用全部核心。"""
    def __init__(self):
        import onnxruntime as ort

        package = find_spec('rapidocr_onnxruntime')
        if package is None:
            raise RuntimeError('Install requirements.txt in the local .venv first.')
        model = Path(package.origin).parent / 'models' / 'ch_PP-OCRv3_rec_infer.onnx'
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        options.add_session_config_entry('session.inter_op.allow_spinning', '0')
        self.session = ort.InferenceSession(str(model), sess_options=options,
                                           providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        characters = self.session.get_modelmeta().custom_metadata_map['character'].splitlines()
        self.characters = ['blank', *characters, ' ']

    def __call__(self, image):
        # 与原模型预处理保持一致；保留宽高比例，不把细长的 1 拉成方形。
        width = max(1, int(48 * image.shape[1] / image.shape[0]))
        resized = cv2.resize(image, (width, 48)).astype(np.float32)
        tensor = ((resized.transpose(2, 0, 1) / 255 - 0.5) / 0.5)[None]
        sequence = self.session.run(None, {self.input_name: tensor})[0][0]
        indices = sequence.argmax(axis=1)
        keep = indices != 0
        keep[1:] &= indices[1:] != indices[:-1]
        tokens = indices[keep]
        text = ''.join(self.characters[int(index)] for index in tokens)
        if text not in ('1', '2', '3'):
            return None, 0.0
        # 仅平均真正解码出的字符概率；不把其他字母强制映射成数字。
        return text, float(sequence[np.flatnonzero(keep), tokens].mean())


def make_engine():
    return DigitEngine()


def prepare_digit(image):
    """提取完整前景并统一为浅底深字，不从多位数中单独挑一个数字。"""
    if image is None or image.size == 0:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if int(gray.max()) - int(gray.min()) < 25:
        return None
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    border = np.concatenate((binary[0], binary[-1], binary[:, 0], binary[:, -1]))
    light_background = np.mean(border) >= 127
    foreground = 255 - binary if light_background else binary
    _, _, stats, _ = cv2.connectedComponentsWithStats(foreground)
    height, width = gray.shape
    boxes = []
    for x, y, w, h, area in stats[1:]:
        if area < max(12, height * width * 0.0008) or h < height * 0.12:
            continue
        if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
            continue
        boxes.append((int(x), int(y), int(x + w), int(y + h), int(area)))
    if not boxes:
        return None
    # 单字旁边的细小印刷碎屑不应扩大 OCR 包围框。高度和面积同时很小
    # 才剔除；完整的第二位数字仍保留，避免把 12 裁成一个可接受的 1/2。
    tallest = max(b[3] - b[1] for b in boxes)
    largest = max(b[4] for b in boxes)
    boxes = [b for b in boxes if b[3] - b[1] >= tallest * .3 or b[4] >= largest * .15]
    x0, y0 = min(b[0] for b in boxes), min(b[1] for b in boxes)
    x1, y1 = max(b[2] for b in boxes), max(b[3] for b in boxes)
    normalized = gray if light_background else 255 - gray
    crop = normalized[y0:y1, x0:x1]
    pad = max(3, round(crop.shape[0] * 0.06))
    crop = cv2.copyMakeBorder(crop, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)


def recognize(engine, image, threshold=0.60, one_threshold=0.30):
    started = time.perf_counter()
    crop = prepare_digit(image)
    if crop is None:
        return None, 0.0, (time.perf_counter() - started) * 1000
    text, score = engine(crop)
    required = one_threshold if text == '1' else threshold
    if (text is None or score < threshold) and crop.shape[1] / crop.shape[0] < .75:
        # 细字在高度 48 的模型中容易退化。灰度、二值和一像素水平扩展
        # 只是同一真实裁剪的归一化视图；至少两次达到普通阈值才接受补偿。
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        thick = cv2.erode(crop, np.ones((1, 2), np.uint8))
        votes = []
        for view, vertical, horizontal in ((crop, .1, .3),
                (cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR), .1, .3),
                (thick, .1, .3), (thick, .15, .2)):
            top, side = round(view.shape[0] * vertical), round(view.shape[0] * horizontal)
            padded = cv2.copyMakeBorder(view, top, top, side, side,
                                       cv2.BORDER_CONSTANT, value=(255, 255, 255))
            candidate, confidence = engine(padded)
            if candidate == '1' and confidence >= max(threshold, one_threshold):
                votes.append(confidence)
                if len(votes) >= 2:
                    text, score = '1', float(np.mean(votes))
                    break
    # 沿用原来针对细长 1 的留白补偿，仅主识别失败时使用。
    if (text is None or score < required) and crop.shape[1] / crop.shape[0] < 0.75:
        votes = []
        for vertical, horizontal in ((0.25, 0.50), (0.25, 0.40), (0.20, 0.40)):
            top, side = round(crop.shape[0] * vertical), round(crop.shape[0] * horizontal)
            padded = cv2.copyMakeBorder(crop, top, top, side, side,
                                        cv2.BORDER_CONSTANT, value=(255, 255, 255))
            candidate, confidence = engine(padded)
            if candidate == '1' and confidence >= one_threshold:
                votes.append(confidence)
        if len(votes) >= 2:
            text, score = '1', float(np.mean(votes))
    required = one_threshold if text == '1' else threshold
    if text not in ('1', '2', '3') or score < required:
        text = None
    return text, score, (time.perf_counter() - started) * 1000


class StableDigit:
    def __init__(self, frames=3, window=7, hold=0.6):
        self.frames, self.window, self.hold = frames, window, hold
        self.reset()

    def reset(self):
        self.history = deque(maxlen=self.window)
        self.value, self.last_support, self.last_observation = None, None, None

    def current(self, now):
        reference = self.last_support if self.value is not None else self.last_observation
        if reference is not None and now - reference >= self.hold:
            self.reset()
        return self.value

    def update(self, text, now):
        self.current(now)
        self.history.append(text)
        self.last_observation = now
        counts = Counter(value for value in self.history if value is not None)
        if counts:
            winner, votes = counts.most_common(1)[0]
            if winner == text and votes >= self.frames and votes / len(self.history) >= 0.65:
                self.value, self.last_support = winner, now
        if text is not None and text == self.value:
            self.last_support = now
        return self.value
