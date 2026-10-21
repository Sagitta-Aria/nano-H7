"""Independent B-station pickup settings; the shared executor still owns motors."""
import json
from pathlib import Path

from .pick_place import DEFAULT, PickPlaceSettings, validated

MECHANISM_KEYS = ('pick_down', 'pick_rotation', 'lift_sps', 'lift_accel',
        'rotation_sps', 'rotation_accel', 's1_open', 's1_closed',
        's1_settle_ms', 's2_settle_ms', 'step_timeout_ms', 'ack_timeout_ms')
ROI_DEFAULT = dict(roi_x_pct=35, roi_y_pct=35, roi_width_pct=30, roi_height_pct=30)
DELAY_DEFAULT = dict(recognition_delay_ms=2000)
KEYS = MECHANISM_KEYS + tuple(ROI_DEFAULT) + tuple(DELAY_DEFAULT)


class BPickupSettings(PickPlaceSettings):
    """Seed once from existing pickup values, then persist independently.

    Tray slot angles remain shared so downstream pickup addresses the same
    physical pockets. This module configures B only; it never moves hardware.
    """
    @staticmethod
    def validate(config):
        """Reject partial/unknown fields and reuse the mechanism's value limits."""
        if not isinstance(config, dict) or set(config) != set(KEYS):
            raise ValueError('B点取料参数不完整，或含有未知参数。')
        delay = config['recognition_delay_ms']
        if type(delay) is not int or not 0 <= delay <= 60000:
            raise ValueError('B点识别后抓取延迟必须是 0～60000 范围内的整数毫秒。')
        for key in ROI_DEFAULT:
            low = 1 if key in ('roi_width_pct', 'roi_height_pct') else 0
            if type(config[key]) is not int or not low <= config[key] <= 100:
                raise ValueError('B点识别框须填写有效的整数百分比。')
        if (config['roi_x_pct']+config['roi_width_pct'] > 100 or
                config['roi_y_pct']+config['roi_height_pct'] > 100):
            raise ValueError('B点识别框不能超出画面范围。')
        full = validated(dict(DEFAULT, **{key: config[key] for key in MECHANISM_KEYS}))
        return dict({key: full[key] for key in MECHANISM_KEYS},
                    **{key: config[key] for key in ROI_DEFAULT},
                    recognition_delay_ms=delay)

    def __init__(self, owner, path):
        self.owner = owner
        initial = owner.pick_place.settings.snapshot()
        # Older B files gain a recognition box and 2s pickup delay in memory, without
        # rewriting the operator's saved mechanism calibration on load.
        self.path = Path(path)
        if self.path.exists():
            initial = json.loads(self.path.read_text(encoding='utf-8'))
        else:
            initial = {key: initial[key] for key in MECHANISM_KEYS}
        self.value = self.validate({**ROI_DEFAULT, **DELAY_DEFAULT, **initial})
        if not self.path.exists():
            # Persist the initial copy once, so later generic edits stay separate.
            super().save(self.value)

    def save(self, config):
        """The console lock protects edits against all active mission owners."""
        c = self.owner
        if c.pick_place.active or c.task.active or c.competition.active:
            raise ValueError('请先停止自动任务和抓放动作，再保存B点取料参数。')
        return super().save(config)

    def cycle_config(self, shared):
        """Overlay B-only fields onto shared tray geometry for one pickup."""
        values = self.snapshot()
        return dict(shared, **{key: values[key] for key in MECHANISM_KEYS})

    def recognition_roi(self):
        """Return the B-only crop in percentages; no camera or motion I/O."""
        return [self.value[key] for key in ROI_DEFAULT]
