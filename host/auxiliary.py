"""Saved M5/M6 operator settings; saving never moves or re-references hardware.

Presets are absolute logical pulse positions, not angles/mm or encoder readings.
Firmware owns the live limits. Reapply saved ranges explicitly after MCU reset.
"""
import copy
import json
import os
import tempfile
from pathlib import Path

CONFIG_PATH = Path(__file__).with_name("auxiliary.json")
DEFAULT = {"5": {"min": 0, "max": 8000, "sps": 300, "accel": 1000,
                 "step": 100, "presets": [0, 0]},
           "6": {"min": 0, "max": 0, "sps": 300, "accel": 1000,
                 "step": 100, "presets": [0, 0]}}


def validated(config):
    """Validate both axes as one snapshot; reject booleans and partial numbers."""
    if not isinstance(config, dict) or set(config) != {"5", "6"}:
        raise ValueError("必须提供 M5、M6 两轴参数。")
    for axis in ("5", "6"):
        value = config[axis]
        if not isinstance(value, dict) or set(value) != set(DEFAULT[axis]):
            raise ValueError("每轴需要范围、速度、加速度、单次步数和两个常用位置。")
        for key, low, high in (("min", 0 if axis == "5" else -1000000, 1000000),
                               ("max", 0, 1000000), ("sps", 1, 3000),
                               ("accel", 1, 20000), ("step", 1, 2000000)):
            # M6 may use an entirely negative interval after a deliberate setup.
            if key == "max" and axis == "6":
                low = -1000000
            if type(value[key]) is not int or not low <= value[key] <= high:
                raise ValueError("M{} {} 范围 {}～{}，必须为整数。".format(axis, key, low, high))
        if value["min"] > value["max"] or (axis == "5" and value["min"] == value["max"]):
            raise ValueError("下限不能大于上限；M5 必须保留有效行程。")
        presets = value["presets"]
        if not isinstance(presets, list) or len(presets) != 2 or any(
                type(p) is not int or not value["min"] <= p <= value["max"] for p in presets):
            raise ValueError("M{} 的两个常用位置必须在设定范围内。".format(axis))
    return copy.deepcopy(config)


class AuxiliaryConfig:
    """Atomically persist operator settings without sending any board command."""
    def __init__(self, path=CONFIG_PATH):
        self.path = Path(path)
        self.value = validated(json.loads(self.path.read_text(encoding="utf-8"))) \
            if self.path.exists() else copy.deepcopy(DEFAULT)

    def snapshot(self):
        return copy.deepcopy(self.value)

    def save(self, config):
        value = validated(config)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                             suffix=".tmp", delete=False) as output:
                temporary = output.name
                json.dump(value, output, ensure_ascii=False, indent=2)
                output.write("\n")
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        self.value = value
        return self.snapshot()
