"""Tk M5/M6 cards: saved operator settings, board-owned positions and limits.

Saving never moves hardware. Reconnect never re-zeros M6. Presets are absolute
logical pulses; the MCU resolves each target atomically against its live pose.
"""
import time
from .auxiliary import validated
import tkinter as tk
from tkinter import ttk, messagebox


class AuxAxisPanel:
    """Dedicated operator controls; use only for M5 lift and M6 rotation."""
    def __init__(self, app, parent, axis, settings):
        self.app, self.axis, self.settings = app, axis, settings
        self.supported = self.armed = False
        self.live_config = False
        self.pending = None
        self.generation = 0
        self.fields, self.telemetry = {}, {}
        self.last_poll = 0.
        self.last_rendered_sign = None
        self.state = tk.StringVar(value="等待连接 v0.2.8 固件。")
        self.position = tk.StringVar(value="— 脉冲")
        p = settings.snapshot()[str(axis)]
        self.variables = {k: tk.StringVar(value=str(p[k])) for k in ("min", "max", "sps", "accel", "step")}
        self.presets = [tk.StringVar(value=str(v)) for v in p["presets"]]
        self.steps, self.speed, self.accel = [self.variables[k] for k in ("step", "sps", "accel")]
        self.direction = tk.StringVar(value="未确认")
        frame = ttk.LabelFrame(parent, text="M5 升降 · 最高点 0，向下为正" if axis == 5 else "M6 旋转 · 自定零位，正反转脉冲", padding=12)
        frame.pack(fill="x", pady=(0, 14))
        ttk.Label(frame, textvariable=self.position, font=("Consolas", 20, "bold")).pack(anchor="w")
        ttk.Label(frame, textvariable=self.state, wraplength=840).pack(anchor="w", pady=5)
        row = ttk.Frame(frame); row.pack(fill="x", pady=4)
        if axis == 5:
            self.selector = ttk.Combobox(row, textvariable=self.direction, values=("未确认", "正步数向下", "负步数向下"), state="readonly", width=15)
            self.selector.pack(side="left")
            self.configure_button = ttk.Button(row, text="应用方向", command=self.configure_direction)
            self.configure_button.pack(side="left", padx=8)
        else:
            self.zero_button = ttk.Button(row, text="当前位置设为零", command=lambda: app.send("AUX ZERO 6"))
            self.zero_button.pack(side="left")
            ttk.Label(row, text="解除使能后设置；不产生运动。").pack(side="left", padx=10)
        grid = ttk.Frame(frame); grid.pack(fill="x", pady=8)
        for col in range(3): grid.columnconfigure(col, weight=1)
        for i, (key, label) in enumerate((("min", "下限 / 脉冲"), ("max", "上限 / 脉冲"), ("step", "单次移动 / 脉冲"), ("sps", "速度 / step/s"), ("accel", "加速度 / step/s²"))):
            cell = ttk.Frame(grid); cell.grid(row=i//3, column=i%3, sticky="ew", padx=(0, 12), pady=4)
            ttk.Label(cell, text=label).pack(anchor="w")
            ttk.Entry(cell, textvariable=self.variables[key], width=15).pack(fill="x")
        row = ttk.Frame(frame); row.pack(fill="x", pady=6)
        ttk.Button(row, text="保存并应用本轴参数", command=self.save).pack(side="left")
        self.apply_button = ttk.Button(row, text="应用本轴范围", command=self.apply_range)
        self.apply_button.pack(side="left", padx=8)
        row = ttk.Frame(frame); row.pack(fill="x", pady=6)
        self.buttons = {}
        actions = (("向上", "UP", False), ("向下", "DOWN", False), ("返回最高点", "UP", True)) if axis == 5 else (("反转一步 −", "UP", False), ("正转一步 +", "DOWN", False))
        for label, direction, home in actions:
            button = ttk.Button(row, text=label, command=lambda d=direction, h=home: self.move(d, h))
            button.pack(side="left", padx=(0, 8)); self.buttons[label] = button
        self.stop_button = ttk.Button(row, text="停止 M{}".format(axis), command=lambda: app.send("AUX HALT {}".format(axis)))
        self.stop_button.pack(side="left")
        self.record_buttons, self.goto_buttons = [], []
        for index in range(2):
            row = ttk.Frame(frame); row.pack(fill="x", pady=5)
            ttk.Label(row, text="常用位置 {} / 脉冲".format(index+1)).pack(side="left")
            ttk.Entry(row, textvariable=self.presets[index], width=15).pack(side="left", padx=8)
            button = ttk.Button(row, text="记录当前位置", command=lambda i=index: self.record(i))
            button.pack(side="left", padx=(0, 8)); self.record_buttons.append(button)
            button = ttk.Button(row, text="前往", command=lambda i=index: self.goto(i))
            button.pack(side="left"); self.goto_buttons.append(button)
        self.render()

    def save(self):
        """Apply connected bounds; persist only after matching board confirmation."""
        try:
            config = self.settings.snapshot()
            p = {k: int(v.get()) for k, v in self.variables.items()}
            p["presets"] = [int(v.get()) for v in self.presets]
            config[str(self.axis)] = p
            validated(config)
            if self.app.connected:
                f = self.telemetry
                if (not self.supported or self.pending or f.get('active') != '0' or
                        f.get('referenced') != '1' or (self.armed and not self.live_config)):
                    raise ValueError('请等待该轴静止并建立零位；旧固件还需解除使能。')
                if not p['min'] <= int(f['pos']) <= p['max']:
                    raise ValueError('当前位置必须在新范围内。')
                self.generation += 1
                tag = ('auxiliary_apply', self.axis, self.generation)
                body = 'AUX CONFIG {} {} {}'.format(self.axis, p['min'], p['max'])
                self.pending = (tag, p)
                if not self.app.connection.submit(body, tag=tag):
                    self.pending = None
                    raise ValueError('命令未能发送。')
                self.app.status.set('M{} 等待板端确认，尚未保存。'.format(self.axis))
                return p
            self.settings.save(config)
            self.app.status.set("M{} 离线参数已保存；连接后请应用。".format(self.axis))
            return p
        except (ValueError, OSError) as error:
            messagebox.showerror("辅助轴参数", str(error))
            return None

    def configure_direction(self):
        sign = {"正步数向下": 1, "负步数向下": -1}.get(self.direction.get())
        if sign is None:
            messagebox.showerror("M5 方向", "请先确认哪种原始步数表示向下。")
            return
        self.app.send("LIFT DIR {}".format(sign))

    def apply_range(self):
        """Use the same confirmation transaction as Save."""
        self.save()

    def record(self, index):
        if self.app.connected and self.telemetry.get("active") == "0" and self.telemetry.get("referenced") == "1":
            self.presets[index].set(self.telemetry["pos"])
            self.app.status.set("已记录当前位置，请保存参数。")

    def move(self, direction, top=False):
        self._motion("GOTO" if top else "STEP", 0 if top else None, direction)

    def goto(self, index):
        self._motion("GOTO", self.presets[index].get())

    def _motion(self, verb, value, direction="DOWN"):
        """Validate cached telemetry for feedback; the MCU rechecks atomically."""
        f = self.telemetry
        if not (self.app.connected and self.supported and self.armed and f.get("active") == "0" and f.get("referenced") == "1" and f.get("configured") == "1" and (self.axis != 5 or self.fields.get("down_sign") in ("1", "-1"))):
            messagebox.showerror("辅助轴未就绪", "请先建立零位、应用范围、确认方向并使能，等待该轴停止。")
            return
        try:
            if int(self.variables['min'].get()) != int(f['min']) or int(self.variables['max'].get()) != int(f['max']):
                raise ValueError('输入范围尚未生效，请先保存并应用。')
            value = int(value) if verb == "GOTO" else int(self.steps.get()) * (1 if direction == "DOWN" else -1)
            sps, accel = int(self.speed.get()), int(self.accel.get())
            target = value if verb == "GOTO" else int(f["pos"]) + value
            if not (1 <= sps <= 3000 and 1 <= accel <= 20000 and -2000000 <= value <= 2000000 and (verb != "STEP" or int(self.steps.get()) > 0) and int(f["min"]) <= target <= int(f["max"])):
                raise ValueError("目标或速度超出范围。")
        except (ValueError, KeyError) as error:
            messagebox.showerror("辅助轴范围", str(error)); return
        self.app.send("AUX {} {} {} {} {}".format(verb, self.axis, value, sps, accel))

    def reply(self, command, reply, tag=None):
        """Mirror telemetry; accepted movement never changes displayed pose."""
        if self.pending and tag == self.pending[0]:
            _, p = self.pending
            self.pending = None
            f = reply.fields
            if (reply.ok and f.get('cmd') == 'AUX_STAT' and f.get('axis') == str(self.axis)
                    and f.get('min') == str(p['min']) and f.get('max') == str(p['max'])):
                config = self.settings.snapshot(); config[str(self.axis)] = p
                try:
                    self.settings.save(config)
                    self.app.status.set('M{} 板端范围已生效，参数已保存。'.format(self.axis))
                except OSError as error:
                    self.app.status.set('板端已应用，但保存失败：{}'.format(error))
            else:
                self.app.status.set('M{} 应用失败，参数未保存。'.format(self.axis))
        if not reply.ok: return
        f = reply.fields
        if f.get("cmd") == "HELLO":
            self.supported = f.get("aux_control") == "1"
            self.live_config = f.get('aux_live') == '1'
            self.pending = None
            self.fields, self.telemetry = {}, {}
            self.armed = False
            self.last_rendered_sign = None
        elif f.get("cmd") == "LIFT_STAT" and self.axis == 5:
            self.fields = f
            if self.last_rendered_sign != f.get("down_sign"):
                self.direction.set({"1": "正步数向下", "-1": "负步数向下"}.get(f.get("down_sign"), "未确认"))
                self.last_rendered_sign = f.get("down_sign")
        elif f.get("cmd") == "AUX_STAT" and f.get("axis") == str(self.axis):
            self.telemetry = f
            if self.axis == 6: self.fields = f
        if "armed" in f: self.armed = f["armed"] == "1"
        self.render()

    def render(self):
        f = self.telemetry
        connected = self.app.connected and self.supported
        idle, referenced = f.get("active") == "0", f.get("referenced") == "1"
        ready = connected and idle and referenced and f.get("configured") == "1" and self.armed and (self.axis != 5 or self.fields.get("down_sign") in ("1", "-1"))
        self.position.set("{} 脉冲".format(f.get("pos", "—")))
        text = "请连接 v0.2.8 或更新固件。" if not connected else "请先建立零位。" if not referenced else "零位已建立，请设置并应用范围。" if f.get("configured") != "1" else "板端范围 {}～{} · {}".format(f["min"], f["max"], "静止" if idle else "运动中")
        self.state.set(text)
        self.apply_button.configure(state="normal" if connected and idle and referenced and not self.pending and (not self.armed or self.live_config) else "disabled")
        self.stop_button.configure(state="normal" if connected else "disabled")
        if self.axis == 5:
            allowed = connected and self.fields.get("initialized") == "1" and idle and not self.armed and self.fields.get("pos") == "0"
            self.configure_button.configure(state="normal" if allowed else "disabled")
            self.selector.configure(state="readonly" if allowed else "disabled")
        else: self.zero_button.configure(state="normal" if connected and idle and not self.armed else "disabled")
        for label, button in self.buttons.items():
            allowed = ready and (label != "返回最高点" or (int(f.get("min", 0)) <= 0 <= int(f.get("max", 0)) and int(f.get("pos", 0)) != 0))
            button.configure(state="normal" if allowed else "disabled")
        for button in self.record_buttons: button.configure(state="normal" if connected and idle and referenced else "disabled")
        for button in self.goto_buttons: button.configure(state="normal" if ready else "disabled")

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.app.connected and self.supported and now - self.last_poll >= .25:
            self.app.send("AUX STAT {}".format(self.axis))
            if self.axis == 5: self.app.send("LIFT STAT")
            self.last_poll = now
        self.render()

    def disconnected(self):
        """Discard confirmations from a closed connection without saving drafts."""
        self.pending = None
        self.supported = self.live_config = self.armed = False
