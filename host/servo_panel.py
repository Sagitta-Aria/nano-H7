"""Servo page for the Tk console: S1/PH6 180 degrees, S2/PH9 270 degrees.

The board holds the servo itself, so this page only sends an angle and mirrors
the reported pulse. The dial is a plain canvas arc because the served angle is a
single value; anything fancier would imply feedback the firmware does not have.
"""
import tkinter as tk
import re
from tkinter import ttk, messagebox

from .ui import BG, CARD, GREEN, INK, MUTED, FONT, NUM, MONO, ScrollArea
from .servo_config import SERVO_LAYOUT, wire_angle, display_angle

PRESETS = [0, 45, 90, 120, 135, 180, 225, 270]


class ServoPanel:
    def __init__(self, app, parent):
        self.app, self.parent = app, parent
        self.angle = tk.StringVar(value="120")
        self.pulse = tk.StringVar(value="1500")
        self.state = tk.StringVar(value="未连接 · 直接连接模拟板即可预览")
        self.travel = 180.
        self.channel = tk.StringVar(value="1")
        self.travels = {ch: layout["travel"] for ch, layout in SERVO_LAYOUT.items()}
        self.targets = {ch: layout["home"] for ch, layout in SERVO_LAYOUT.items()}
        self.selected_channel = 1
        self.device = {}
        self.frames = {}
        self.heading = tk.StringVar(value="舵机 S1 · PH6 / TIM12_CH1 · 180°")
        self.area = ScrollArea(parent)
        self.area.pack(fill="both", expand=True)
        self.content = self.area.content
        self.build()
        self.area.bind_wheel_tree()

    def build(self):
        ttk.Label(self.content, textvariable=self.heading,
                  font=(FONT, 12, "bold")).pack(anchor="w")
        ttk.Label(self.content,
                  text="连接后自动定位：S1（PH6）120°，S2（PH9）0°。舵机不受“使能输出”门控，也不会被“停止全部”切断："
                       "它是位置输出，必须持续收到脉冲才能保持位置。",
                  foreground=MUTED, wraplength=900, justify="left").pack(anchor="w", pady=(4, 10))

        row = ttk.Frame(self.content)
        row.pack(fill="x")
        dial = tk.Frame(row, bg=CARD, padx=10, pady=10)
        dial.pack(side="left")
        self.canvas = tk.Canvas(dial, width=280, height=180, background=CARD, highlightthickness=0)
        self.canvas.pack()
        self.readout = self.canvas.create_text(140, 118, text="—", fill=INK, font=(MONO, 22, "bold"))
        self.arc = self.canvas.create_arc(40, 30, 240, 230, start=0, extent=0,
                                          style="arc", width=12, outline=GREEN)

        form = ttk.LabelFrame(row, text="按角度定位")
        form.pack(side="left", fill="both", expand=True, padx=(12, 0))
        line = ttk.Frame(form)
        line.pack(fill="x", pady=(0, 6))
        ttk.Label(line, text="通道").pack(side="left", padx=(0, 6))
        self.selector = ttk.Combobox(line, textvariable=self.channel, values=("1", "2"), width=3, state="readonly")
        self.selector.pack(side="left", padx=(0, 8))
        self.selector.bind("<<ComboboxSelected>>", self.select_channel)
        ttk.Label(line, text="角度 (度)").pack(side="left", padx=(0, 6))
        ttk.Entry(line, textvariable=self.angle, width=10).pack(side="left")
        ttk.Button(line, text="定位", style="Accent.TButton", command=self.send).pack(side="left", padx=8)
        ttk.Button(line, text="查询状态", command=lambda: self.app.send("SERVO STAT")).pack(side="left")
        presets = ttk.Frame(form)
        presets.pack(fill="x", pady=(0, 8))
        self.preset_buttons = {}
        for degrees in PRESETS:
            button = ttk.Button(presets, text=f"{degrees}°", width=6,
                                command=lambda d=degrees: self.send(d))
            if degrees <= self.travel:
                button.pack(side="left", padx=(0, 4))
            button.configure(state="normal" if degrees <= self.travel else "disabled")
            self.preset_buttons[degrees] = button
        ttk.Label(form, textvariable=self.state, foreground=MUTED,
                  wraplength=520, justify="left").pack(anchor="w", pady=(6, 0))
        ttk.Label(form, text="角度写整数即为整度（135 = 135.0°）；只有出现小数点才表示小数。"
                             "越界会被板端拒绝（ERR code=ANGLE_RANGE），不会静默截断。",
                  foreground=MUTED, wraplength=520, justify="left").pack(anchor="w")

        cal = ttk.LabelFrame(self.content, text="脉宽校准")
        cal.pack(fill="x", pady=(12, 0))
        ttk.Label(cal, text="实测满行程脉宽 (µs)").pack(side="left", padx=(0, 6))
        ttk.Entry(cal, textvariable=self.pulse, width=8).pack(side="left")
        ttk.Button(cal, text="换算成角度并发送", command=self.send_pulse).pack(side="left", padx=8)
        ttk.Label(cal, text="固件没有原始脉宽命令；这里换算成角度走同一条角度路径。"
                            "若实测范围不是 500–2500 µs，改固件常量并重算脉宽表。",
                  foreground=MUTED).pack(side="left")

    # ---- actions -----------------------------------------------------------

    def select_channel(self, event=None):
        """Restore this UI servo's target and mapped status; selection sends nothing."""
        channel = int(self.channel.get())
        if channel != self.selected_channel:
            if self.selected_channel is not None:
                self.targets[self.selected_channel] = self.angle.get()
            self.angle.set(self.targets[channel])
            self.selected_channel = channel
        self.travel = self.travels[channel]
        layout = SERVO_LAYOUT[channel]
        pin, timer_channel = layout["pin"], layout["timer"]
        self.heading.set(f"舵机 S{channel} · {pin} / TIM12_CH{timer_channel} · {self.travel:g}°")
        for degrees, button in self.preset_buttons.items():
            button.pack_forget()
            if degrees <= self.travel and (degrees != 120 or channel == 1):
                button.pack(side="left", padx=(0, 4))
            button.configure(state="normal" if degrees <= self.travel else "disabled")
        frame = self.frames.get(channel)
        if frame:
            self.state.set(f"已下发 {frame['angle']}° · {frame['us']} µs（无位置反馈）")
        else:
            self.state.set("尚未收到该通道状态；选中通道不会触发运动。")
        self.draw()

    def send(self, degrees=None):
        text = str(self.angle.get() if degrees is None else degrees).strip()
        if not _valid_angle(text):
            messagebox.showerror("角度格式", "整度（135）或一位小数（13.5）。")
            return
        if float(text) > self.travel:
            messagebox.showerror("超出行程", f"当前行程 {self.travel:g}°，板端会拒绝更大角度。")
            return
        channel = int(self.channel.get())
        self.angle.set(text)
        self.targets[channel] = text
        self.app.send(f"SERVO {channel} {wire_angle(self.device, channel, text)}")

    def send_pulse(self):
        try:
            micros = int(self.pulse.get())
        except ValueError:
            messagebox.showerror("脉宽无效", "脉宽必须是 500–2500 的整数微秒。")
            return
        if not 500 <= micros <= 2500:
            messagebox.showerror("脉宽无效", "脉宽必须是 500–2500 的整数微秒。")
            return
        self.send(round((micros - 500) * self.travel / 2000, 1))

    # ---- replies -----------------------------------------------------------

    def reply(self, command, reply):
        """Mirror the board's answer; never invent a position nothing reported."""
        fields = reply.fields
        if fields.get("cmd") == "HELLO":
            self.device = dict(fields)
            channels = fields.get("servo")
            count = max(1, int(channels or 1))
            self.travels = {
                ch: layout["travel"]
                for ch, layout in SERVO_LAYOUT.items() if ch <= count
            }
            self.selector.configure(values=tuple(str(ch) for ch in self.travels))
            if int(self.channel.get()) not in self.travels:
                self.channel.set(str(next(iter(self.travels))))
            self.frames.clear()
            self.targets = {ch: layout["home"] for ch, layout in SERVO_LAYOUT.items()}
            self.selected_channel = None
            self.select_channel()
            self.state.set(f"板端声明 {channels} 路舵机，行程 {self.travel:g}°。"
                           if channels else "该固件没有舵机能力标记。")
        elif fields.get("cmd") == "SERVO_STAT" and reply.ok:
            for match in re.finditer(r"ch=(\d+) angle=([\d.]+) us=(\d+) on=(\d)", reply.body):
                channel = int(match[1])
                if channel in SERVO_LAYOUT:
                    self.frames[channel] = {"angle": display_angle(channel, match[3]), "us": match[3]}
            self.select_channel()
        elif fields.get("cmd") == "SERVO" and reply.ok:
            channel = int(fields["ch"])
            if channel in SERVO_LAYOUT:
                self.frames[channel] = {"angle": display_angle(channel, fields["us"]), "us": fields["us"]}
            self.select_channel()

    def disconnected(self):
        self.state.set("已断开；真实板舵机仍在保持最后位置。")

    def draw(self):
        frame = self.frames.get(int(self.channel.get()))
        value = frame["angle"] if frame else None
        self.canvas.itemconfigure(self.readout, text=f"{value}°" if value is not None else "—")
        try:
            fraction = max(0., min(1., float(value) / self.travel))
        except (ValueError, TypeError):
            fraction = 0.
        # Canvas arcs run counter-clockwise from start; -90..180 covers 0..270.
        self.canvas.itemconfigure(self.arc, start=-90, extent=fraction * 270)


def _valid_angle(text):
    whole, dot, fraction = text.partition(".")
    if not whole.isdigit() or len(whole) > 3:
        return False
    return not dot or (len(fraction) == 1 and fraction.isdigit())
