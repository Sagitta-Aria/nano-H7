"""Persist explicit flow calibration; all live totals and recording marks are MCU-owned.

Saving does not apply to hardware. Applying affects future increments only and
must never reset a boot total or reinterpret old history on a host reconnect.
"""
import copy
import json
import math
import os
import tempfile
from decimal import Decimal, InvalidOperation
from pathlib import Path

CONFIG_PATH = Path(__file__).with_name("optical_flow.json")
DEFAULT = dict(height_mm=30.0, gain_x=1.0, gain_y=1.0, map=0,
               angle_deg=0.0, min_quality=1)


def wire_values(config):
    """Validate a complete calibration and return six integer protocol fields.

    Decimal conversion rejects NaN, booleans and unrepresentable precision;
    gains are multipliers after axis swap, and map bits are X-/Y-/swap.
    """
    if not isinstance(config, dict) or set(config) != set(DEFAULT):
        raise ValueError("需要完整的高度、X/Y 比例、轴向映射、安装角和最低置信度。")
    values = []
    for key, scale, low, high in (("height_mm", 1000, 1000, 1000000),
                                 ("gain_x", 1000000, 10000, 10000000),
                                 ("gain_y", 1000000, 10000, 10000000)):
        try:
            if isinstance(config[key], bool):
                raise ValueError()
            value = Decimal(str(config[key])) * scale
            if not value.is_finite() or value != value.to_integral_value() or not low <= value <= high:
                raise ValueError()
            values.append(int(value))
        except (InvalidOperation, ValueError, TypeError):
            raise ValueError("高度限 1～1000 mm、最多三位小数；比例限 0.01～10、最多六位小数。") from None
    mapping = config["map"]
    quality = config["min_quality"]
    if type(mapping) is not int or not 0 <= mapping <= 7 or type(quality) is not int or not 1 <= quality <= 255:
        raise ValueError("轴向映射为 0～7，最低置信度为 1～255 的整数。")
    try:
        angle = Decimal(str(config["angle_deg"])) * 100
        if isinstance(config["angle_deg"], bool) or not angle.is_finite() or angle != angle.to_integral_value() or not -18000 <= angle <= 18000:
            raise ValueError()
    except (InvalidOperation, ValueError):
        raise ValueError("安装角限 −180～180°，最多两位小数，顺时针为正。") from None
    return [*values, mapping, int(angle), quality]


def config_command(config):
    """Build the existing CRC/request-protected command, without opening a port."""
    return "FLOW CONFIG " + " ".join(map(str, wire_values(config)))


class FlowSettings:
    """Atomic local preferences shared by Web/Tk; explicit apply is separate."""
    def __init__(self, path=CONFIG_PATH):
        self.path = Path(path)
        self.value = json.loads(self.path.read_text(encoding="utf-8")) if self.path.exists() else copy.deepcopy(DEFAULT)
        wire_values(self.value)

    def snapshot(self):
        """Return a copy so a draft form cannot mutate persisted settings."""
        return copy.deepcopy(self.value)

    def save(self, config):
        """Validate before atomic replacement; saving never sends a command."""
        wire_values(config)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                             suffix=".tmp", delete=False) as stream:
                name = stream.name
                json.dump(config, stream, ensure_ascii=False, indent=2)
                stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
            os.replace(name, self.path)
        finally:
            if name and os.path.exists(name):
                os.unlink(name)
        self.value = copy.deepcopy(config)
        return self.snapshot()


class FlowDemo:
    """Synthetic vectors for explicit SIM sessions; never used for hardware I/O."""
    def __init__(self, board):
        self.board = board
        self.config = copy.deepcopy(DEFAULT)
        self.total = [0., 0.]
        self.world = [0., 0.]
        self.world_base = [0., 0.]
        self.world_frozen = [0., 0.]
        self.initial_yaw = board.navigation.absolute_heading
        self.world_epoch = board.navigation.imu_epoch
        self.world_locked = False
        self.gaps = self.mark_gap = self.frozen_gap = 0
        self.baseline = [0., 0.]
        self.frozen = [0., 0.]
        self.last = 0.
        self.marked = self.recording = False
        self.start_ms = self.end_ms = self.revision = 0

    def update(self):
        """Integrate a deterministic synthetic path using the current calibration."""
        t = self.board.clock() - self.board.start
        a, b = ((math.sin(t) - math.sin(self.last)) * 3000,
                (math.cos(t) - math.cos(self.last)) * 3000)
        self.last = t
        c = self.config
        if c["map"] & 4:
            a, b = b, a
        a *= c["height_mm"] * c["gain_x"] / 10 * (-1 if c["map"] & 1 else 1)
        b *= c["height_mm"] * c["gain_y"] / 10 * (-1 if c["map"] & 2 else 1)
        angle = math.radians(c["angle_deg"])
        dx, dy = math.cos(angle) * a + math.sin(angle) * b, -math.sin(angle) * a + math.cos(angle) * b
        self.total[0] += dx; self.total[1] += dy
        nav = self.board.navigation
        self.world_locked |= nav.imu_epoch != self.world_epoch
        if not self.world_locked and nav.fresh(self.board.clock()):
            yaw = math.radians(nav.absolute_heading - self.initial_yaw)
            self.world[0] += math.cos(yaw)*dx + math.sin(yaw)*dy
            self.world[1] += -math.sin(yaw)*dx + math.cos(yaw)*dy
        elif dx or dy:
            self.gaps += 1

    def dispatch(self, args):
        """Mirror FLOW commands; firmware tests separately verify real parsing."""
        self.update()
        now = int(self.last * 1000)
        if args == ["START"]:
            self.baseline = self.total.copy(); self.world_base = self.world.copy()
            self.mark_gap = self.gaps; self.marked = self.recording = True
            self.start_ms, self.end_ms = now, 0
        elif args == ["STOP"]:
            if self.recording:
                self.frozen = [a - b for a, b in zip(self.total, self.baseline)]
                self.world_frozen = [a - b for a, b in zip(self.world, self.world_base)]
                self.frozen_gap = self.gaps - self.mark_gap
                self.recording = False; self.end_ms = now
        elif len(args) == 7 and args[0] == "CONFIG":
            try:
                h, gx, gy, mapping, angle, q = map(int, args[1:])
                c = dict(height_mm=h/1000, gain_x=gx/1e6, gain_y=gy/1e6,
                         map=mapping, angle_deg=angle/100, min_quality=q)
                wire_values(c)
            except (TypeError, ValueError):
                return "ERR code=FLOW_ARGUMENT"
            self.config = c; self.revision += 1; args = ["CONFIG"]
        elif args not in (["STAT"], ["WORLD"], ["CONFIG"]):
            return "ERR code=FLOW_ARGUMENT"
        relative = [a - b for a, b in zip(self.total, self.baseline)] if self.recording else self.frozen
        x, y = map(round, self.total); rx, ry = map(round, relative)
        if args == ["CONFIG"]:
            h, gx, gy, mapping, angle, q = wire_values(self.config)
            return f"OK cmd=FLOW_CONFIG height_um={h} gx_ppm={gx} gy_ppm={gy} map={mapping} angle_cd={angle} minq={q} rev={self.revision}"
        if args == ["WORLD"]:
            wx, wy = map(round, self.world)
            relative = [a-b for a,b in zip(self.world, self.world_base)] if self.recording else self.world_frozen
            wrx, wry = map(round, relative)
            fresh = not self.world_locked and self.board.navigation.fresh(self.board.clock())
            gaps = self.gaps-self.mark_gap if self.recording else self.frozen_gap
            return (f"OK cmd=FLOW_WORLD wx_um={wx} wy_um={wy} wrx_um={wrx} wry_um={wry} heading={int(fresh)} "
                    f"locked={int(self.world_locked)} gaps={self.gaps} rgaps={gaps} world_ms=0 epoch={self.board.navigation.imu_epoch}")
        return (f"OK cmd=FLOW_STAT x_um={x} y_um={y} rx_um={rx} ry_um={ry} marked={int(self.marked)} recording={int(self.recording)} "
                f"start_ms={self.start_ms} end_ms={self.end_ms} now_ms={now} online=1 valid=200 dt_us=5000 fx=0 fy=0 "
                f"frames={int(self.last*200)} accepted={int(self.last*200)} bad=0 low=0 uart=0 sat=0 rev={self.revision}")
