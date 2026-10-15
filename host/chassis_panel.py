"""Relative body motion UI. No Cartesian position estimate or path planner.

Keyboard intent exists only while the user holds a key; the chassis tab no
longer has to be the visible one, so the axis table can be watched while
jogging. The Tk loop refreshes a latest-only worker lease; no key events enter
a FIFO. Leaving the tab, losing window focus or releasing the key still stops.
"""
import tkinter as tk
from tkinter import ttk, messagebox
import time
from .ui import GREEN, MUTED, FONT, ScrollArea
from .turn_panel import TurnPanel
from .straight import straight_command

DIRECTIONS = {"前进": "FWD", "后退": "BACK", "左移": "LEFT", "右移": "RIGHT",
              "原地左旋 ↺": "CCW", "原地右旋 ↻": "CW"}
KEYS = {"Up": "FWD", "Down": "BACK", "Left": "LEFT", "Right": "RIGHT", "q": "CCW", "e": "CW"}


class ChassisPanel:
    def __init__(self, app, parent):
        self.app, self.parent = app, parent
        self.keyboard = tk.BooleanVar(value=False)
        self.direction = tk.StringVar(value="前进")
        self.steps = tk.StringVar(value="1000")
        self.speed = tk.StringVar(value="500")
        self.accel = tk.StringVar(value="2000")
        self.assist = tk.BooleanVar(value=False)
        self.assist_supported = False
        self.state = tk.StringVar(value="未连接 · 先确认四轮方向，再进行整车运动")
        self.invert = [tk.BooleanVar(value=i in (0, 2)) for i in range(4)]
        self.held = None
        self.intent = None
        self.release_timer = None
        self.last_refresh = self.last_poll = 0.
        self.supported = False
        self.config_received = False
        self.config_draft = None
        self.board_invert = None
        self.config_syncing = False
        self.config_state = tk.StringVar(value="等待板端方向配置")
        for var in self.invert:
            var.trace_add("write", self.config_edited)
        # Shared scroll body: every control stays reachable on a short screen.
        self.area = ScrollArea(parent)
        self.area.pack(fill="both", expand=True)
        self.content = self.area.content
        self.build()
        self.turn = TurnPanel(app, self.content)
        self.area.bind_wheel_tree()
        app.root.bind("<KeyPress>", self.key_down, add="+")
        app.root.bind("<KeyRelease>", self.key_up, add="+")
        app.root.bind("<FocusOut>", self.focus_out, add="+")
        app.root.bind("<ButtonRelease-1>", self.mouse_up, add="+")
        app.notebook.bind("<<NotebookTabChanged>>", self.tab_changed, add="+")

    def build(self):
        """Arrange a body-oriented pad, finite move form and wiring calibration."""
        top = ttk.Frame(self.content)
        top.pack(fill="x")
        ttk.Label(top, text="车头 ↑     M3 左前 · M4 右前 · M1 左后 · M2 右后",
                  font=(FONT, 12, "bold")).pack(anchor="w")
        move = ttk.LabelFrame(self.content, text="相对位移 · 平移步数")
        move.pack(fill="x", pady=(8, 12))
        for title, variable, width in [("方向", self.direction, 13), ("步数", self.steps, 10),
                                       ("速度 step/s", self.speed, 8), ("加速度 step/s²", self.accel, 8)]:
            ttk.Label(move, text=title).pack(side="left", padx=(0, 5))
            if variable is self.direction:
                widget = ttk.Combobox(move, textvariable=variable, values=list(DIRECTIONS), state="readonly", width=width)
            else:
                widget = ttk.Entry(move, textvariable=variable, width=width)
            widget.pack(side="left", padx=(0, 12))
        ttk.Button(move, text="执行位移", command=self.move).pack(side="right")
        ttk.Checkbutton(self.content, text="陀螺仪辅助直线（指定步数平移，起步锁定航向）",
                        variable=self.assist).pack(anchor="w")
        ttk.Label(self.content, text="前进、后退、左移、右移可纠偏；共用下方 P 和角度容差。"
                  "辅助模式步数按四轮合成平移量计程，点动和原地旋转保持原操作。",
                  foreground=MUTED, wraplength=940).pack(anchor="w", pady=(0,8))
        middle = ttk.Frame(self.content)
        middle.pack(fill="x")
        pad = ttk.LabelFrame(middle, text="按住移动 · 松开停止")
        pad.pack(side="left", fill="both", expand=True, padx=(0, 12))
        self.pad_buttons = {}
        for text, direction, row, column in [("↺ 左旋 / Q", "CCW", 0, 0), ("↑ 前进", "FWD", 0, 1),
            ("右旋 / E ↻", "CW", 0, 2), ("← 左移", "LEFT", 1, 0), ("↓ 后退", "BACK", 1, 1),
            ("右移 →", "RIGHT", 1, 2)]:
            button = ttk.Button(pad, text=text, takefocus=False)
            button.grid(row=row, column=column, padx=4, pady=4, sticky="ew")
            button.bind("<ButtonPress-1>", lambda event, d=direction: self.begin(d, "mouse"))
            self.pad_buttons[direction] = button
            pad.columnconfigure(column, weight=1)
        ttk.Checkbutton(pad, text="开启键盘控制：↑↓前后，←→平移，Q/E 旋转",
                        variable=self.keyboard, command=self.keyboard_changed).grid(row=2, column=0, columnspan=2, sticky="w", pady=8)
        ttk.Button(pad, text="停止底盘（空格）", command=lambda: self.halt(force=True)).grid(row=2, column=2, sticky="ew", pady=4)
        diagram = ttk.LabelFrame(middle, text="俯视接线约定")
        diagram.pack(side="right", fill="both")
        ttk.Label(diagram, text="↑ 车头　（每轮标注：STEP / DIR）\n\n"
                  "M1 左后　PI5 / PI8\nM2 右后　PI6 / PI3\nM3 左前　PI7 / PI4\nM4 右前　PI2 / PC13",
                  justify="left").pack()
        config = ttk.LabelFrame(self.content, text="电机方向校准 · 勾选表示反转该轮 DIR")
        config.pack(fill="x", pady=12)
        for i, title in enumerate(["M1 左后", "M2 右后", "M3 左前", "M4 右前"]):
            ttk.Checkbutton(config, text=title, variable=self.invert[i]).pack(side="left", padx=(0, 20))
        ttk.Button(config, text="静止时应用", command=self.configure).pack(side="right")
        ttk.Label(self.content, textvariable=self.config_state, foreground=MUTED).pack(anchor="w")
        ttk.Label(self.content, textvariable=self.state, foreground=GREEN, wraplength=790).pack(anchor="w", pady=6)
        ttk.Label(self.content, text="步数是每个车轮的 STEP 数，不是毫米或车体坐标；方向校准仅本次上电有效。"
                  "离开本页、窗口失焦或松键会停止点动；空格停止底盘、Esc 停止全部。",
                  foreground=MUTED, wraplength=940).pack(anchor="w")

    def selected(self):
        return self.app.notebook.select() == str(self.parent)

    def parameters(self):
        sps, accel = int(self.speed.get()), int(self.accel.get())
        if not 1 <= sps <= 5000 or not 1 <= accel <= 20000:
            raise ValueError("速度范围 1–5000，加速度范围 1–20000")
        return sps, accel

    def ready(self):
        if not self.app.connected or not self.supported:
            self.state.set("请连接支持底盘指令的 v0.2 固件；旧固件需重新烧录。")
            return False
        if self.app.values["armed"].get() != "已使能":
            self.state.set("请先点击底部的「使能输出」。")
            return False
        return True

    def move(self):
        if not self.ready():
            return
        if self.intent:
            self.state.set("请先松开点动，再提交指定步数运动。")
            return
        try:
            steps = int(self.steps.get())
            if not 1 <= steps <= 10000000:
                raise ValueError("步数范围 1–10000000")
            sps, accel = self.parameters()
            direction = DIRECTIONS[self.direction.get()]
            if self.assist.get():
                if not self.assist_supported:
                    raise ValueError("陀螺仪辅助直线需要 H743 0.2.17 固件。")
                self.app.send(straight_command(direction, steps, sps, accel))
            else:
                self.app.send(f"CHASSIS MOVE {direction} {steps} {sps} {accel}")
        except ValueError as exc:
            messagebox.showerror("运动参数无效", str(exc))

    def configure(self):
        if not self.app.connected or not self.supported:
            self.state.set("请先连接 v0.2 固件。")
            return
        mask = sum(1 << i for i, v in enumerate(self.invert) if v.get())
        self.app.send(f"CHASSIS CONFIG {mask}")

    def config_edited(self, *_args):
        """Keep operator edits separate from board reports, including late init."""
        if self.config_syncing:
            return
        self.config_draft = sum(1 << i for i, var in enumerate(self.invert) if var.get())
        self.render_config()

    def render_config(self):
        """Show confirmed and unsent masks; queued requests are not confirmation."""
        board = "等待板端方向配置" if self.board_invert is None else f"板端掩码 {self.board_invert}"
        self.config_state.set(board + (f" · 待应用 {self.config_draft}" if self.config_draft is not None else ""))

    def begin(self, direction, source):
        if getattr(getattr(self.app, "navigation", None), "uploading", False):
            self.state.set("正在下发巡航，请等待完成或先停止。")
            return
        if not self.ready():
            return
        try:
            sps, accel = self.parameters()
        except ValueError as exc:
            self.state.set(str(exc))
            return
        self.held = source
        self.intent = f"CHASSIS JOG {direction} {sps} {accel}"
        self.app.connection.set_jog(self.intent)
        self.last_refresh = time.monotonic()

    def halt(self, force=False):
        """Release local intent before queuing HALT; never auto-arm on restart."""
        had_intent = self.intent is not None
        self.held = self.intent = None
        if self.release_timer:
            self.app.root.after_cancel(self.release_timer)
            self.release_timer = None
        if self.app.connected and (had_intent or force):
            self.app.send("CHASSIS HALT")

    def editable_focus(self):
        widget = self.app.root.focus_get()
        return widget is not None and widget.winfo_class() in ("TEntry", "Entry", "TCombobox", "TSpinbox", "Spinbox", "Text")

    def key_down(self, event):
        if event.keysym == "Escape":
            self.halt()
            self.app.send("STOP")
            return "break"
        if self.editable_focus():
            return
        if event.keysym == "space":
            self.halt(force=True)
            return "break"
        key = event.keysym if event.keysym in KEYS else event.keysym.lower()
        if not self.keyboard.get() or key not in KEYS:
            return
        if self.held == key:
            if self.release_timer:
                self.app.root.after_cancel(self.release_timer)
                self.release_timer = None
            return "break"  # OS auto-repeat does not add motion commands.
        if self.held is not None:
            return "break"  # One direction at a time; no hidden diagonal blend.
        self.begin(KEYS[key], key)
        return "break"

    def key_up(self, event):
        key = event.keysym if event.keysym in KEYS else event.keysym.lower()
        if self.held == key and self.release_timer is None:
            self.release_timer = self.app.root.after(25, self.halt)
            return "break"

    def mouse_up(self, event):
        if self.held == "mouse":
            self.halt()

    def keyboard_changed(self):
        self.halt()
        self.parent.focus_set()

    def focus_out(self, event):
        def check():
            if self.app.root.focus_displayof() is None or self.editable_focus():
                self.halt()
        self.app.root.after_idle(check)

    def tab_changed(self, event):
        if not self.selected():
            self.halt()

    def tick(self, now):
        self.turn.tick(now)
        if self.intent and self.app.connected:
            if now - self.last_refresh >= .1:
                self.app.connection.set_jog(self.intent)
                self.last_refresh = now
        if self.app.connected and self.supported and self.app.connection.commands.qsize() < 3 and now - self.last_poll >= .3:
            self.app.send("CHASSIS STAT")
            self.last_poll = now

    def reply(self, command, reply):
        self.turn.reply(command, reply)
        if not reply.ok:
            if command.startswith("CHASSIS"):
                self.state.set("底盘拒绝命令：" + reply.fields.get("code", reply.body))
                self.halt()
            return
        f = reply.fields
        if f.get("cmd") == "HELLO":
            self.supported = f.get("chassis") == "1"
            self.assist_supported = f.get("straight") == "1"
            self.config_received = False
            self.config_draft = self.board_invert = None
            self.render_config()
        elif f.get("cmd") in ("CHASSIS", "CHASSIS_STAT"):
            labels = {"idle": "空闲", "moving": "指定步数运行中", "done": "步数执行完成", "stopped": "已停止",
                      "jogging": "点动中", "switching": "等待换向", "fault": "直线纠偏故障"}
            self.state.set(f"{labels.get(f['state'], f['state'])} · {f['direction']} · 命令 {f['move_id']} · "
                           f"目标 {f['total']} step / 剩余 {f['remaining']} step · 活动轮掩码 {f['active']}")
            if f.get("assist") == "1":
                self.state.set(self.state.get() + " · 锁定航向 {:.2f}° · 偏差 {:.2f}° · {}".format(
                    int(f['target_cd'])/100, int(f['error_cd'])/100, f['fault']))
            self.board_invert = int(f["invert"])
            if self.config_draft == self.board_invert:
                self.config_draft = None
            if self.config_draft is None:
                self.config_syncing = True
                try:
                    for i, var in enumerate(self.invert):
                        var.set(bool(self.board_invert & (1 << i)))
                finally:
                    self.config_syncing = False
            self.config_received = True
            self.render_config()
        if f.get("armed") == "0":
            self.halt()

    def disconnected(self):
        self.turn.disconnected()
        self.halt()
        self.keyboard.set(False)
        self.supported = False
        self.assist_supported = False
        self.config_received = False
        self.config_draft = self.board_invert = None
        self.render_config()
        self.state.set("离线 · 底盘状态未知，重连后需重新使能")
