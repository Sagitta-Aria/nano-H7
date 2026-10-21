"""Tk mission editor using the same JSON parameters and MCU protocol as Web.

Nano/PC plans the full route from fresh MCU telemetry. Only acknowledged
uploads advance to START; closing/stopping cancels the transaction.
"""
import tkinter as tk
from tkinter import ttk, messagebox
from .navigation import MissionConfig, ORIGIN, reference_command
from .navigation_runner import NavigationRunner
from .ui import ScrollArea, MUTED


class NavigationPanel:
    def __init__(self, app, parent):
        self.app = app
        self.config = MissionConfig()
        self.navigator = NavigationRunner(self)
        self.arrival_enabled = {}
        self.fields = {}
        self.headings = {}
        self.grid_fields = {}
        self.pending = []
        self.generation = 0
        self.uploading = False
        self.supported = False
        self.reference_supported = False
        self.telemetry = {}
        self.last_poll = 0.
        self.state = tk.StringVar(value="未连接")
        self.assist_state = tk.StringVar(value="等待板端直线纠偏功能状态")
        self.area = ScrollArea(parent)
        self.area.pack(fill="both", expand=True)
        body = self.area.content
        self.route_text = tk.StringVar()
        ttk.Label(body, textvariable=self.route_text).pack(anchor="w")
        grid = ttk.Frame(body)
        grid.pack(anchor="w", pady=15)
        for column, title in enumerate(("点", "x", "y", "到达姿态 °", "到点调姿")):
            ttk.Label(grid, text=title).grid(row=0, column=column, padx=12, pady=6)
        values = self.config.snapshot()
        # Station count comes from the saved config. Adding or removing stations
        # and editing the route order is done on the Web page; this panel edits
        # the coordinates and attitudes of whatever stations already exist.
        self.station_names = [station["name"] for station in values["stations"]]
        self.start_name = tk.StringVar(value=self.station_names[values['start_station']])
        ttk.Label(body, text="巡航起点（建立坐标前选择）").pack(anchor="w")
        ttk.Combobox(body, textvariable=self.start_name, state="readonly",
                     values=[name for i, name in enumerate(self.station_names) if i == ORIGIN or name == 'X']).pack(anchor="w")
        self.route_text.set(" → ".join(
            [self.start_name.get()] +
            [self.station_names[step["station"]] for step in values["route"]]) +
            " · 脉冲坐标 · x 右为正 / y 前为正")
        for row, station in enumerate(values["stations"], 1):
            name = station["name"]
            ttk.Label(grid, text=name).grid(row=row, column=0, padx=12, pady=6)
            variables = [tk.StringVar(value=str(station[axis])) for axis in ("x", "y")]
            self.fields[name] = variables
            for column, variable in enumerate(variables, 1):
                ttk.Entry(grid, textvariable=variable, width=16).grid(row=row, column=column, padx=12, pady=6)
            self.headings[name] = tk.StringVar(value=str(station["heading"]))
            ttk.Entry(grid, textvariable=self.headings[name], width=12,
                      state="readonly" if row - 1 == ORIGIN else "normal").grid(row=row, column=3, padx=12, pady=6)
            self.arrival_enabled[name] = tk.BooleanVar(value=station.get('arrival_enabled', False))
            ttk.Checkbutton(grid,variable=self.arrival_enabled[name]).grid(row=row,column=4,padx=12,pady=6)
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
        self.inset_enabled = tk.BooleanVar(value=values['turn_inset_enabled'])
        ttk.Checkbutton(parameters, text="转前向场地中心内移", variable=self.inset_enabled).grid(row=2, column=0, padx=8)
        self.fields['turn_inset_steps'] = tk.StringVar(value=str(values['turn_inset_steps']))
        ttk.Label(parameters, text="内移脉冲（转后平移回通道/站点）").grid(row=2, column=1, padx=8)
        ttk.Entry(parameters, textvariable=self.fields['turn_inset_steps'], width=16).grid(row=2, column=2, padx=8)
        buttons = ttk.Frame(body)
        buttons.pack(anchor="w", pady=12)
        for text, callback in (("保存参数", self.save), ("当前位置设为所选起点", self.origin),
                               ("开始巡航", self.start), ("停止巡航", self.stop)):
            ttk.Button(buttons, text=text, command=callback).pack(side="left", padx=(0, 10))
        ttk.Label(body, textvariable=self.state).pack(anchor="w", pady=12)
        ttk.Label(body, textvariable=self.assist_state, foreground=MUTED).pack(anchor="w")
        ttk.Label(body, text="O 出发沿通道巡航；X 出发强制内移、转至 0°，直接平移到 A。\nX 建系位置 (0, 28800)、朝向 -180°；内移采用保存的脉冲值。定位仍为脉冲估算。", foreground=MUTED).pack(anchor="w")
        self.area.bind_wheel_tree()

    @property
    def connection(self):
        return self.app.connection

    def note(self, kind, message):
        self.app.status.set(message)

    def stop(self):
        self.navigator.stop('操作员停止。')
        self.cancel()
        self.app.send('NAV STOP')

    def save(self):
        try:
            config = {"stations": [
                {"name": name,
                 "x": int(self.fields[name][0].get()), "y": int(self.fields[name][1].get()),
                 "heading": float(self.headings[name].get()), "arrival_enabled": self.arrival_enabled[name].get()}
                for name in self.station_names]}
            # The route is preserved as saved; this panel does not reorder it.
            config["route"] = self.config.snapshot()["route"]
            config['start_station'] = self.station_names.index(self.start_name.get())
            config.update({name: int(self.fields[name].get()) for name in ("sps", "accel", "dwell_ms")})
            config['turn_inset_enabled'] = self.inset_enabled.get()
            config['turn_inset_steps'] = int(self.fields['turn_inset_steps'].get())
            config["grid"] = {axis: [int(v.get()) for v in variables] for axis, variables in self.grid_fields.items()}
            self.config.save(config)
            self.route_text.set(" → ".join([self.start_name.get()] +
                [self.station_names[step['station']] for step in config['route']]))
        except (ValueError, OSError) as error:
            messagebox.showerror("参数无效", str(error))
            return False
        self.app.status.set("巡航参数已保存，下次巡航生效。")
        return True

    def origin(self):
        if self.save():
            config = self.config.snapshot()
            if config['start_station'] and not self.reference_supported:
                self.app.status.set('X 点建系需要 H743 0.2.29 的 nav_reference 接口。')
                return
            try:
                command = reference_command(config)
            except ValueError as error:
                self.app.status.set('巡航参数尚不可执行：' + str(error))
                return
            self.app.send(command)

    def start(self):
        if not self.supported or not self.app.connected:
            self.app.status.set("请连接支持完整路径上传的 0.2.14 固件。")
            return
        if self.navigator.active or self.uploading or self.telemetry.get("ready") != "1":
            self.app.status.set("请先建立所选起点，或等待当前下发结束。")
            return
        if self.telemetry.get("state") in ("turning", "moving", "orienting", "waiting", "blocked") or self.app.chassis.turn.fields.get("state") == "turning":
            self.app.status.set("请先停止当前巡航。")
            return
        if self.app.connection.port != "SIM" and not self.app.imu.hub.navigation_feedback()["valid"]:
            self.app.status.set("请先连接 H723 并等待校准及新航向数据。")
            return
        if self.save():
            self.run_config = self.config.snapshot()
            if self.run_config['start_station'] and not self.reference_supported:
                self.app.status.set('X 点建系需要 H743 0.2.29 的 nav_reference 接口。')
                return
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
        self.navigator.reply(command,reply,tag)
        if command in ('STOP','ARM 0','NAV STOP','CHASSIS HALT','TURN STOP'):
            self.navigator.stop('已停止。'); self.cancel()
        if tag == ("navigation", self.generation):
            if not reply.ok:
                self.cancel()
            elif command == 'NAV EXEC' and (reply.fields.get('cmd') != 'NAV_EXEC' or reply.fields.get('v') != '1'):
                self.cancel()
                self.app.status.set('需要 H743 通用逐步执行接口，未启动运动。')
            elif command == "NAV STAT":
                try:
                    fields = reply.fields
                    if 'stations' not in fields or 'held' not in fields:
                        raise ValueError('站点路线需要 H743 0.2.19 或更新固件。')
                    if fields.get("ready") != "1" or fields.get("state") in ("turning", "moving", "orienting", "waiting", "blocked"):
                        raise ValueError("坐标未建立或车辆仍在运动。")
                    self.pending = self.config.commands((int(fields["x"]), int(fields["y"])),
                                                        self.generation % 2147483647 + 1, config=self.run_config, host_control=True)
                    self.next_command()
                except (ValueError, KeyError) as error:
                    self.cancel()
                    self.app.status.set("路径规划失败：" + str(error))
            elif command.startswith("NAV START"):
                self.uploading = False
                self.navigator.start(self.run_config,command.split()[2])
            else:
                self.next_command()
        fields = reply.fields
        if reply.ok and fields.get('cmd') == 'INFO':
            self.reference_supported = fields.get('nav_reference') == '1'
        if reply.ok and fields.get("cmd") == "HELLO":
            self.supported = all(fields.get(key) == "1" for key in ("nav_heading", "nav_grid", "nav_path"))
            self.assist_state.set("每段直线自动进行陀螺仪纠偏 · 使用底盘页的 P 与容差" if fields.get("straight") == "1"
                                  else "巡航直线纠偏需要 H743 0.2.17 固件")
        if reply.ok and fields.get("cmd") == "NAV_STAT":
            self.telemetry = fields
            self.state.set("{} · 当前 ({}, {}) · 目标 {} ({}, {}) · 停留 {} ms".format(
                fields.get("state"), fields.get("x"), fields.get("y"), self.config.station_name(int(fields.get("station", -1))),
                fields.get("tx"), fields.get("ty"), fields.get("wait_ms")))
            self.state.set(self.state.get() + "\n航向 {}° · 目标 {}° · 到达姿态 {}° · {}".format(
                int(fields.get("heading_cd", 0)) / 100, int(fields.get("target_cd", 0)) / 100,
                int(fields.get("arrival_cd", 0)) / 100, fields.get("fault", "none")))
            if fields.get("planner") == "host_grid":
                self.state.set(self.state.get() + "\n下一拐点 ({}, {}) · 小段 {}/{}".format(
                    fields.get("wx"), fields.get("wy"), fields.get("segment"), fields.get("segments")))

    def tick(self, now):
        self.navigator.tick()
        if self.app.connected and self.supported and now - self.last_poll >= .3:
            self.app.send("NAV STAT")
            self.last_poll = now

    def disconnected(self):
        self.navigator.stop("串口断开。",fault=True)
        self.cancel()
        self.supported = False
        self.reference_supported = False
        self.telemetry = {}
        self.state.set("未连接")
        self.assist_state.set("等待板端直线纠偏功能状态")
