"""Persist acknowledged MCU tuning, excluding positions and movement commands."""
import copy
import json
import os
import tempfile
from pathlib import Path


class ControlSettings:
    """Replay only configuration that the board previously accepted."""
    def __init__(self, path):
        self.path = Path(path)
        self.value = json.loads(self.path.read_text(encoding='utf-8')) if self.path.exists() else {}
        self.commands()  # Validate the on-disk schema before any serial replay.

    def commands(self, include_lift=True):
        """Build a small allowlist; a settings file can never store ARM or MOVE."""
        result = []
        for key, values in self.value.items():
            ranges = {'turn': [(0, 2147483647), (1, 1000), (1, 5000)],
                      'chassis': [(0, 15)], 'lift': [(-1, 1)]}.get(key)
            if ranges is None or not isinstance(values, list) or len(values) != len(ranges):
                raise ValueError('控制参数文件格式错误：' + key)
            if any(type(v) is not int or not low <= v <= high for v, (low, high) in zip(values, ranges)):
                raise ValueError('控制参数文件数值错误：' + key)
            if key == 'lift' and values[0] == 0:
                raise ValueError('升降方向必须是 -1 或 1。')
            if key == 'lift' and not include_lift: continue
            result.append({'turn':'TURN CONFIG ', 'chassis':'CHASSIS CONFIG ', 'lift':'LIFT DIR '}[key] + ' '.join(map(str, values)))
        return result

    def snapshot(self):
        """Return saved values, not browser drafts or live defaults."""
        return copy.deepcopy(self.value)

    def accept(self, command, fields):
        """Save only successful config replies, with the complete board readback."""
        value = self.snapshot()
        if command.startswith('TURN CONFIG '):
            value['turn'] = [int(fields[k]) for k in ('kp_centi', 'tolerance_cd', 'min_sps')]
        elif command.startswith('CHASSIS CONFIG '):
            value['chassis'] = [int(command.split()[2])]
        elif command.startswith('LIFT DIR '):
            value['lift'] = [int(fields['down_sign'])]
        else:
            return False
        if value == self.value:
            return False
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             suffix='.tmp', delete=False) as output:
                temporary = output.name
                json.dump(value, output, indent=2)
                output.write('\n'); output.flush(); os.fsync(output.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary): os.unlink(temporary)
        self.value = value
        return True
