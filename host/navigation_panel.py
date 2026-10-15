"""Tk mission editor using the same JSON parameters and MCU protocol as Web.

Nano/PC plans the full route from fresh MCU telemetry. Only acknowledged
uploads advance to START; closing/stopping cancels the transaction.
"""
import tkinter as tk
from tkinter import ttk, messagebox
from .navigation import MissionConfig, LABELS
from .ui import ScrollArea, MUTED


class NavigationPanel:
    def __init__(self, app, parent):
        self.app = app
        self.config = MissionConfig()
        self.fields = {}
        self.headings = {}
        self.grid_fields = {}
        self.pending = []
        self.generation = 0
        self.uploading = False
        self.supported = False
        self.telemetry = {}
        self.last_poll = 0.
        self.state = tk.StringVar(value="未连接")
        self.assist_state = tk.StringVar(value="等待板端直线纠偏功能状态")
        self.area = ScrollArea(parent)
        self.area.pack(fill="both", expand=True)
        body = self.area.content
        ttk.Label(body, text="O → A → B → C → D → A · 脉冲坐标 · x 右为正 / y 前为正").pack(anchor="w")
        grid = ttk.Frame(body)
        grid.pack(anchor="w", pady=15)
        for column, title in enumerate(("点", "x", "y", "到达姿态 °")):
            ttk.Label(grid, text=title).grid(row=0, column=column, padx=12, pady=6)
        values = self.config.snapshot()
        for row, name in enumerate(LABELS, 1):
            ttk.Label(grid, text=name).grid(row=row, column=0, padx=12, pady=6)
            variables = [tk.StringVar(value=str(value)) for value in values["points"][name]]
            self.fields[name] = variables
            for column, variable in enumerate(variables, 1):
                ttk.Entry(grid, textvariable=variable, width=16).grid(row=row, column=column, padx=12, pady=6)
            self.headings[name] = tk.StringVar(value=str(values["headings"][name]))
            ttk.Entry(grid, textvariable=self.headings[name], width=12,
                      state="readonly" if name == "O" else "normal").grid(row=row, column=3, padx=12, pady=6)
        lanes = ttk.Frame(body)
        lanes.pack(anchor="w", pady=8)
        for row, (axis, title) in enumerate((("x", "三条竖线 x"), ("y", "三条横线 y"))):
            ttk.Label(lanes, text=title).grid(row=row, column=0, padx=8, pady=6)
            self.grid_fields[axis] = [tk.StringVar(value=str(v)) for v in values["grid"][axis]]
            for column, variable in enumerate(self.grid_fields[axis], 1):
                ttk.Entry(lanes, textvariable=variable, width=12).grid(row=row, column=column, padx=8, pady=6)
        parameters = ttk.Frame(body)
        parameters.pack(anchor="w", pady=8)
        for column, (name, title) in enumerate((("sps", "最大轮速 step/s"), ("accel", "加速度 step/s²"), ("dwell_ms", "到点停留 ms"))):
            variable = self.fields[name] = tk.StringVar(value=str(values[name]))
            ttk.Label(parameters, text=title).grid(row=0, column=column, padx=8, sticky="w")
            ttk.Entry(parameters, textvariable=variable, width=16).grid(row=1, column=column, padx=8, pady=6)
        buttons = ttk.Frame(body)
        buttons.pack(anchor="w", pady=12)
        for text, callback in (("保存参数", self.save), ("当前位置设为 O", self.origin),
                               ("开始巡航", self.start), ("停止巡航", lambda: app.send("NAV STOP"))):
            ttk.Button(buttons, text=text, command=callback).pack(side="left", padx=(0, 10))
        ttk.Label(body, textvariable=self.state).pack(anchor="w", pady=12)
        ttk.Label(body, textvariable=self.assist_state, foreground=MUTED).pack(anchor="w")
        ttk.Label(body, text="田字格通道 · 只沿横纵线前进 · 拐点停车转向，不额外停留\n首次出发航向 = 0°，重新开始保留地图朝向；目标点调姿后停留。定位仍为脉冲估算。", foreground=MUTED).pack(anchor="w")
        self.area.bind_wheel_tree()

    def save(self):
        try:
            config = {"points": {name: [int(variable.get()) for variable in self.fields[name]] for name in LABELS}}
            config.update({name: int(self.fields[name].get()) for name in ("sps", "accel", "dwell_ms")})
            config["headings"] = {name: float(variable.get()) for name, variable in self.headings.items()}
            config["grid"] = {axis: [int(v.get()) for v in variables] for axis, variables in self.grid_fields.items()}
            self.config.save(config)
        except (ValueError, OSError) as error:
            messagebox.showerror("参数无效", str(error))
            return False
        self.app.status.set("巡航参数已保存，下次巡航生效。")
        return True

    def origin(self):
        if self.save():
            x, y = self.config.snapshot()["points"]["O"]
            self.app.send("NAV RESET {} {}".format(x, y))

    def start(self):
        if not self.supported or not self.app.connected:
            self.app.status.set("请连接支持完整路径上传的 0.2.14 固件。")
            return
        if self.uploading or self.telemetry.get("ready") != "1":
            self.app.status.set("请先建立 O 点，或等待当前下发结束。")
            return
        if self.telemetry.get("state") in ("turning", "moving", "orienting", "waiting", "blocked") or self.app.chassis.turn.fields.get("state") == "turning":
            self.app.status.set("请先停止当前巡航。")
            return
        if self.app.connection.port != "SIM" and not self.app.imu.hub.navigation_feedback()["valid"]:
            self.app.status.set("请先连接 H723 并等待校准及新航向数据。")
            return
        if self.save():
            self.generation += 1
            self.pending = ["NAV STAT"]
            self.uploading = True
            self.next_command()

    def next_command(self):
        if self.pending and not self.app.connection.submit(self.pending.pop(0), tag=("navigation", self.generation)):
            self.cancel()
            self.app.status.set("发送队列已满，巡航未启动。")

    def cancel(self):
        self.generation += 1
        self.pending = []
        self.uploading = False

    def reply(self, command, reply, tag=None):
        if tag == ("navigation", self.generation):
            if not reply.ok:
                self.cancel()
            elif command == "NAV STAT":
                try:
                    fields = reply.fields
                    if fields.get("ready") != "1" or fields.get("state") in ("turning", "moving", "orienting", "waiting", "blocked"):
                        raise ValueError("坐标未建立或车辆仍在运动。")
                    self.pending = self.config.commands((int(fields["x"]), int(fields["y"])),
                                                        self.generation % 2147483647 + 1)
                    self.next_command()
                except (ValueError, KeyError) as error:
                    self.cancel()
                    self.app.status.set("路径规划失败：" + str(error))
            elif command.startswith("NAV START"):
                self.uploading = False
            else:
                self.next_command()
        fields = reply.fields
        if reply.ok and fields.get("cmd") == "HELLO":
            self.supported = all(fields.get(key) == "1" for key in ("nav_heading", "nav_grid", "nav_path"))
            self.assist_state.set("每段直线自动进行陀螺仪纠偏 · 使用底盘页的 P 与容差" if fields.get("straight") == "1"
                                  else "巡航直线纠偏需要 H743 0.2.17 固件")
        if reply.ok and fields.get("cmd") == "NAV_STAT":
            self.telemetry = fields
            self.state.set("{} · 当前 ({}, {}) · 目标 {} ({}, {}) · 停留 {} ms".format(
                fields.get("state"), fields.get("x"), fields.get("y"), fields.get("target"),
                fields.get("tx"), fields.get("ty"), fields.get("wait_ms")))
            self.state.set(self.state.get() + "\n航向 {}° · 目标 {}° · 到达姿态 {}° · {}".format(
                int(fields.get("heading_cd", 0)) / 100, int(fields.get("target_cd", 0)) / 100,
                int(fields.get("arrival_cd", 0)) / 100, fields.get("fault", "none")))
            if fields.get("planner") == "host_grid":
                self.state.set(self.state.get() + "\n下一拐点 ({}, {}) · 小段 {}/{}".format(
                    fields.get("wx"), fields.get("wy"), fields.get("segment"), fields.get("segments")))

    def tick(self, now):
        if self.app.connected and self.supported and now - self.last_poll >= .3:
            self.app.send("NAV STAT")
            self.last_poll = now

    def disconnected(self):
        self.cancel()
        self.supported = False
        self.telemetry = {}
        self.state.set("未连接")
        self.assist_state.set("等待板端直线纠偏功能状态")
