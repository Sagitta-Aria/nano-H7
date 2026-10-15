"""Tk desktop console. Run from repository root: python -m host.gui.

The default demo mode never opens a serial port. Real motion needs both an
explicit GUI acknowledgement and an ARM command; LED/resource tests do not.

Every tab body lives in a ScrollArea, so a short screen scrolls instead of
silently cutting off the last controls. Arm/acknowledge/STOP stay in one
always-visible action bar rather than being repeated per page.
"""
import argparse
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox
from .transport import Connection, list_ports
from .chassis_panel import ChassisPanel
from .servo_panel import ServoPanel
from .lift_panel import LiftPanel
from .aux_panel import AuxAxisPanel
from .auxiliary import AuxiliaryConfig
from .navigation_panel import NavigationPanel
from .imu_panel import ImuPanel
from .flow_panel import FlowPanel
from .ui import (BG, CARD, INK, MUTED, HINT, HDR, LOG_BG, LOG_FG, FONT, NUM, MONO,
                 LED_FIXED, LED_OFF, LED_GREEN, LED_BLUE,
                 apply_theme, ScrollArea, Prefs)

CARDS = [("运行时间", "uptime"), ("内核时钟", "cpu"), ("RTOS 空闲堆", "heap"), ("电机输出", "armed")]
DEVICE = [("固件 / 协议", "firmware"), ("板卡", "board"), ("芯片 UID", "uid"),
          ("Flash / RTOS", "flash"), ("复位标志", "reset")]
RESOURCES = [("任务数 / 历史最小空闲堆", "tasks"), ("协议坏帧 / 重放次数", "protocol"),
             ("任务栈最小剩余：通信 / 运动 / 监督", "stacks"), ("心跳超时 / 运动时序故障", "faults"),
             ("串口接收字节 / 接收错误 / 发送超时", "serial")]


class Console:
    def __init__(self, root):
        self.root = root
        self.connection = None
        self.events = queue.Queue()
        self.connected = False
        self.closing = False
        self.last_poll = 0.
        self.axis_poll = 1
        self.last_stat = 0.
        self.values = {}
        self.scroll_areas = []
        self.prefs = Prefs()
        self.aux_settings = AuxiliaryConfig()
        self.port = tk.StringVar(value="模拟板（无需硬件）")
        self.status = tk.StringVar(value="未连接 · 可先连接模拟板体验")
        self.motion_ack = tk.BooleanVar(value=False)
        self.link_mode = tk.StringVar(value="DEMO / 离线预览")
        self.setup_style()
        self.build()
        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.pump_job = root.after(50, self.pump)

    def setup_style(self):
        self.root.title("H743 Robot Console · 调试台")
        self.root.geometry(self.prefs.geometry("1080x800"))
        self.root.minsize(900, 600)
        apply_theme(self.root)

    def value(self, key, initial="—"):
        self.values[key] = tk.StringVar(value=initial)
        return self.values[key]

    def scroll_tab(self, parent, height=300):
        area = ScrollArea(parent, height=height)
        area.pack(fill="both", expand=True)
        self.scroll_areas.append(area)
        return area.content

    def build(self):
        header = tk.Frame(self.root, bg=HDR, padx=22, pady=12)
        header.pack(fill="x")
        tk.Label(header, text="H743  /  ROBOT CONSOLE", bg=HDR, fg="white",
                 font=(NUM, 16, "bold")).pack(side="left")
        tk.Label(header, textvariable=self.link_mode, bg=HDR, fg=HINT,
                 font=(FONT, 10)).pack(side="right")

        bar = ttk.Frame(self.root, padding=(20, 10))
        bar.pack(fill="x")
        ttk.Label(bar, text="连接设备").pack(side="left", padx=(0, 10))
        self.port_box = ttk.Combobox(bar, textvariable=self.port, state="readonly", width=40)
        self.port_box.pack(side="left")
        ttk.Button(bar, text="刷新", command=self.refresh_ports).pack(side="left", padx=8)
        self.connect_button = ttk.Button(bar, text="连接", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        ttk.Label(bar, text="115200 · 8N1 · 有线 / 无线", foreground=MUTED).pack(side="right")

        strip = ttk.Frame(self.root, padding=(20, 0, 20, 10))
        strip.pack(fill="x")
        card = tk.Frame(strip, bg=CARD, padx=16, pady=8)
        card.pack(fill="x")
        for column, (label, key) in enumerate(CARDS):
            cell = tk.Frame(card, bg=CARD)
            cell.grid(row=0, column=column, sticky="w", padx=(0, 40))
            tk.Label(cell, text=label, bg=CARD, fg=MUTED, font=(FONT, 9)).pack(side="left", padx=(0, 8))
            tk.Label(cell, textvariable=self.value(key), bg=CARD, fg=INK,
                     font=(NUM, 13, "bold")).pack(side="left")

        notebook = self.notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=20, pady=(0, 6))
        overview, chassis, motion, logs = [ttk.Frame(notebook) for _ in range(4)]
        servo = ttk.Frame(notebook)
        notebook.add(overview, text="板级测试与资源")
        notebook.add(servo, text="舵机")
        notebook.add(chassis, text="底盘运动")
        navigation = ttk.Frame(notebook)
        notebook.add(navigation, text="坐标巡航")
        auxiliary = ttk.Frame(notebook)
        notebook.add(auxiliary, text="升降与旋转")
        notebook.add(motion, text="七轴运动调试")
        notebook.add(logs, text="通信记录")
        imu_tab = ttk.Frame(notebook)
        notebook.add(imu_tab, text="陀螺仪 · H723")
        self.imu = ImuPanel(self, imu_tab)
        notebook.bind("<<NotebookTabChanged>>", lambda event: self.imu.draw(), add="+")
        self.build_overview(self.scroll_tab(overview))
        self.build_motion(self.scroll_tab(motion))
        aux_body = self.scroll_tab(auxiliary)
        ttk.Label(aux_body, text="位置与常用位置均为脉冲计数；保存不运动，范围需静止且解除使能后应用。", wraplength=840).pack(anchor="w", pady=(0, 10))
        self.lift = LiftPanel(self, aux_body, self.aux_settings)
        self.rotation = AuxAxisPanel(self, aux_body, 6, self.aux_settings)
        self.servo = ServoPanel(self, servo)
        self.chassis = ChassisPanel(self, chassis)
        self.navigation = NavigationPanel(self, navigation)
        flow_tab = ttk.Frame(notebook)
        notebook.add(flow_tab, text="光流位移")
        self.flow = FlowPanel(self, self.scroll_tab(flow_tab))
        self.build_logs(logs)
        for area in self.scroll_areas:
            area.bind_wheel_tree()

        footer = ttk.Frame(self.root, padding=(20, 6, 20, 10))
        footer.pack(side="bottom", fill="x", before=notebook)
        ttk.Checkbutton(footer, text="已核对驱动器接线与运动区域", variable=self.motion_ack).pack(side="left")
        ttk.Button(footer, text="使能输出", style="Accent.TButton", command=self.arm).pack(side="left", padx=10)
        ttk.Label(footer, textvariable=self.status, foreground=MUTED).pack(side="left", padx=(14, 10),
                                                                            fill="x", expand=True)
        ttk.Button(footer, text="停止全部 / 解除使能", style="Stop.TButton",
                   command=lambda: self.send("STOP")).pack(side="right")

    def build_overview(self, parent):
        parent.columnconfigure(0, weight=3)
        parent.columnconfigure(1, weight=5)
        leds = ttk.LabelFrame(parent, text="板载 LED · 回路测试")
        leds.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        self.led_canvas = tk.Canvas(leds, height=82, background=BG, highlightthickness=0)
        self.led_canvas.pack(fill="x")
        self.led_shapes, self.led_labels = [], []
        for name, colour in (("红 / 电源固定", LED_FIXED), ("绿 / PB0", LED_OFF), ("蓝 / PB1", LED_OFF)):
            self.led_shapes.append(self.led_canvas.create_oval(0, 0, 0, 0, fill=colour, outline=""))
            self.led_labels.append(self.led_canvas.create_text(0, 0, text=name, fill=MUTED, font=(FONT, 9)))
        self.led_canvas.bind("<Configure>", self.layout_leds)
        buttons = ttk.Frame(leds)
        buttons.pack(fill="x", pady=(6, 0))
        self.led_buttons = []
        for label, body in [("绿灯", "LED 1 0"), ("蓝灯", "LED 0 1"),
                            ("同时点亮", "LED 1 1"), ("关闭绿蓝", "LED 0 0")]:
            button = ttk.Button(buttons, text=label, command=lambda b=body: self.send(b))
            button.pack(side="left", padx=(0, 6))
            self.led_buttons.append(button)
        ttk.Label(leds, text="红灯不受软件控制；圆点显示已确认的通道状态。",
                  foreground=MUTED, wraplength=340).pack(anchor="w", pady=(10, 0))

        device = ttk.LabelFrame(parent, text="设备信息")
        device.grid(row=0, column=1, sticky="nsew")
        for row, (label, key) in enumerate(DEVICE):
            ttk.Label(device, text=label, foreground=MUTED).grid(row=row, column=0, sticky="w", pady=4, padx=(0, 14))
            ttk.Label(device, textvariable=self.value(key)).grid(row=row, column=1, sticky="w", pady=4)

        resources = ttk.LabelFrame(parent, text="实时资源 · 1 秒刷新")
        resources.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        for index, (label, key) in enumerate(RESOURCES):
            row, column = divmod(index, 2)
            column *= 2
            ttk.Label(resources, text=label, foreground=MUTED).grid(row=row, column=column, sticky="w",
                                                                    pady=4, padx=(0, 12))
            ttk.Label(resources, textvariable=self.value(key)).grid(row=row, column=column + 1, sticky="w",
                                                                    pady=4, padx=(0, 30))
        ttk.Label(parent, text="模拟连接中的资源数值为演示值。真实连接读取板端数据；空闲堆不等于芯片全部剩余 RAM。",
                  foreground=MUTED, wraplength=940).grid(row=2, column=0, columnspan=2, sticky="w", pady=(12, 0))

    def layout_leds(self, _event=None):
        """Keep the three lamps evenly spread when the window is resized."""
        step = max(self.led_canvas.winfo_width(), 240) / 3.
        for index, shape in enumerate(self.led_shapes):
            centre = step * (index + .5)
            self.led_canvas.coords(shape, centre - 17, 6, centre + 17, 40)
            self.led_canvas.coords(self.led_labels[index], centre, 60)

    def build_motion(self, parent):
        ttk.Label(parent, text="位置来自脉冲计数，不是编码器反馈。运动中反向需先减速到零；整车 m/s 换算待轮径、减速比与细分确定。",
                  foreground=MUTED, wraplength=940).pack(anchor="w", pady=(0, 10))
        self.axis_table = ttk.Treeview(parent, columns=("axis", "limit", "pos", "sps", "left", "state", "id"),
                                       show="headings", height=7)
        for key, title, width in [("axis", "轴", 90), ("limit", "上限 step/s", 105),
            ("pos", "已发脉冲位置", 125), ("sps", "计划 step/s", 110), ("left", "剩余步数", 105),
            ("state", "状态", 95), ("id", "命令编号", 145)]:
            self.axis_table.heading(key, text=title)
            self.axis_table.column(key, width=width, anchor="center")
        self.axis_table.pack(fill="x")
        for i in range(1, 8):
            self.axis_table.insert("", "end", iid=str(i), values=(f"M{i}", 5000 if i <= 4 else 3000, "—", "—", "—", "未知", "—"))

        forms = ttk.Frame(parent)
        forms.pack(fill="x", pady=(12, 0))
        forms.columnconfigure(0, weight=1, uniform="form")
        forms.columnconfigure(1, weight=1, uniform="form")
        move = ttk.LabelFrame(forms, text="单轴有限行程")
        move.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        self.move_fields = {}
        for index, (label, key, initial, width) in enumerate([("轴 (1–7)", "axis", "5", 5),
                ("步数（负数反向）", "steps", "200", 9), ("最高 step/s", "sps", "500", 7),
                ("加速度 step/s²", "accel", "2000", 7)]):
            row, column = divmod(index, 2)
            cell = ttk.Frame(move)
            cell.grid(row=row, column=column, sticky="w", pady=3, padx=(0, 14))
            ttk.Label(cell, text=label).pack(side="left", padx=(0, 6))
            var = tk.StringVar(value=initial)
            self.move_fields[key] = var
            ttk.Entry(cell, textvariable=var, width=width).pack(side="left")
        ttk.Button(move, text="提交运动", command=self.move_axis).grid(row=2, column=0, columnspan=2,
                                                                      sticky="ew", pady=(8, 0))

        wheels = ttk.LabelFrame(forms, text="四轮速度 · M1 / M2 / M3 / M4（正负表示方向）")
        wheels.grid(row=0, column=1, sticky="nsew")
        self.wheel_fields = []
        row = ttk.Frame(wheels)
        row.pack(fill="x")
        for i in range(4):
            ttk.Label(row, text=f"M{i+1}").pack(side="left", padx=(0, 4))
            var = tk.StringVar(value="0")
            self.wheel_fields.append(var)
            ttk.Entry(row, textvariable=var, width=7).pack(side="left", padx=(0, 12))
        buttons = ttk.Frame(wheels)
        buttons.pack(fill="x", pady=(8, 0))
        ttk.Button(buttons, text="应用速度", command=self.set_wheels).pack(side="left")
        ttk.Button(buttons, text="四轮减速到零",
                   command=lambda: self.send("WHEELS 0 0 0 0 2000")).pack(side="left", padx=8)

    def build_logs(self, parent):
        parent.configure(padding=16)
        self.log_text = tk.Text(parent, bg=LOG_BG, fg=LOG_FG, insertbackground="white",
                                font=(MONO, 10), wrap="word", state="disabled", height=15)
        self.log_text.pack(fill="both", expand=True)
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(8, 0))
        ttk.Label(row, text="只记录命令应答与连接错误；心跳、状态轮询与点动不刷屏。",
                  foreground=MUTED).pack(side="left")
        ttk.Button(row, text="清空记录", command=self.clear_log).pack(side="right")

    def refresh_ports(self):
        self.ports = {"模拟板（无需硬件）": "SIM"}
        self.ports.update({f"{port} · {description}": port for port, description in list_ports()})
        self.port_box["values"] = list(self.ports)
        remembered = self.prefs.text("port")
        if self.port.get() not in self.ports and remembered in self.ports:
            self.port.set(remembered)
        if self.port.get() not in self.ports:
            self.port.set("模拟板（无需硬件）")

    def toggle_connection(self):
        if self.connection:
            self.chassis.halt()
            self.connection.close()
            self.connect_button.configure(state="disabled")
            self.status.set("正在断开，发送停止命令…")
            return
        self.connected = False
        self.motion_ack.set(False)
        port = self.ports[self.port.get()]
        self.prefs.put("port", self.port.get())
        self.link_mode.set("模拟模式 · 所有数据仅作演示" if port == "SIM" else f"真实串口 / {port}")
        self.status.set("正在握手…")
        self.port_box.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.connection = Connection(port, self.events, imu_provider=self.imu.hub.navigation_feedback)
        self.connection.start()

    def send(self, body):
        if body in ("STOP", "NAV STOP", "CHASSIS HALT", "ARM 0"):
            self.navigation.cancel()
            self.lift.pending = self.rotation.pending = None
        elif self.navigation.uploading and body.startswith(("MOVE ", "WHEELS ", "CHASSIS MOVE ", "CHASSIS CONFIG ", "TURN MOVE ", "TURN ZERO")):
            self.status.set("正在下发巡航，请等待完成或先停止。")
            return False
        if body == "STOP":
            self.chassis.held = self.chassis.intent = None
        if not self.connected or not self.connection:
            self.status.set("请先连接模拟板或真实串口。")
            return False
        if not self.connection.submit(body):
            self.status.set("发送队列已满，请稍后再试。")
            return False
        return True

    def arm(self):
        if self.connection and self.connection.port != "SIM" and not self.motion_ack.get():
            messagebox.showinfo("尚未确认接线", "LED 与系统查询无需使能电机。\n运动测试前请勾选接线确认。")
            return
        self.send("ARM 1")

    def move_axis(self):
        try:
            axis, steps, sps, accel = [int(self.move_fields[k].get()) for k in ("axis", "steps", "sps", "accel")]
            if not (1 <= axis <= 7 and 0 < abs(steps) <= 10000000 and
                    1 <= sps <= (5000 if axis <= 4 else 3000) and 1 <= accel <= 20000):
                raise ValueError
        except ValueError:
            messagebox.showerror("参数超出范围", "轴 1–7；步数非零且绝对值 ≤10000000；\nM1–M4 ≤5000 step/s，M5–M7 ≤3000；加速度 1–20000。")
            return
        self.send(f"MOVE {axis} {steps} {sps} {accel}")

    def set_wheels(self):
        try:
            speeds = [int(var.get()) for var in self.wheel_fields]
            if any(abs(s) > 5000 for s in speeds):
                raise ValueError
        except ValueError:
            messagebox.showerror("轮速无效", "四轮速度必须是 -5000 到 5000 的整数。")
            return
        self.send("WHEELS " + " ".join(map(str, speeds)) + " 2000")

    def log(self, text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        if int(self.log_text.index("end-1c").split(".")[0]) > 1000:
            self.log_text.delete("1.0", "101.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def handle_reply(self, command, reply, tag=None):
        self.navigation.reply(command, reply, tag)
        self.chassis.reply(command, reply)
        self.servo.reply(command, reply)
        self.lift.reply(command, reply, tag)
        self.rotation.reply(command, reply, tag)
        self.flow.reply(command, reply)
        f = reply.fields
        if not reply.ok:
            self.status.set(f"命令被拒绝：{f.get('code', reply.body)}")
            self.log(f"{command} → {reply.body}")
            return
        cmd = f.get("cmd")
        if cmd not in ("PING", "STAT", "AXIS", "SERVO_STAT", "CHASSIS_JOG", "NAV_STAT", "NAV_IMU") and command not in ("CHASSIS STAT", "LIFT STAT", "AUX STAT 5", "AUX STAT 6", "FLOW STAT", "FLOW WORLD", "FLOW CONFIG"):
            self.log(f"{command} → {reply.body}")
        if cmd == "HELLO":
            self.values["firmware"].set(f"{f.get('fw')} / v{f.get('protocol')}")
            self.values["board"].set(f.get("board", "—"))
        elif cmd == "INFO":
            self.values["cpu"].set(f"{int(f['cpu_hz'])/1000000:g} MHz")
            self.values["uid"].set(f.get("uid", "—"))
            self.values["flash"].set(f"{f.get('flash_kb')} KB / FreeRTOS {f.get('rtos')}")
            self.values["reset"].set(f.get("reset", "—"))
        elif cmd == "STAT":
            self.values["uptime"].set(f"{int(f['uptime_ms'])/1000:,.1f} s")
            self.values["heap"].set(f"{int(f['heap_free'])/1024:.1f} KiB")
            self.values["tasks"].set(f"{f['tasks']} / {f['heap_min']} bytes")
            self.values["stacks"].set(f"{f['stack_comm']} / {f['stack_motion']} / {f['stack_super']} bytes")
            self.values["serial"].set(f"{f['rx']} / {f['rx_err']} / {f['tx_err']}")
            self.values["protocol"].set(f"{f['bad']} / {f['replay']}")
            self.values["faults"].set(f"{f['timeout']} / {f['motion_fault']}")
        elif cmd == "AXIS":
            i = int(f["axis"])
            if 1 <= i <= 7:
                state = "运行中" if f["active"] == "1" else ("已停止" if int(f["remaining"]) else "空闲 / 完成")
                self.axis_table.item(str(i), values=(f"M{i}", 5000 if i <= 4 else 3000,
                    f["pos"], f["sps"], f["remaining"], state, f["move_id"]))
        if "led" in f:
            mask = int(f["led"])
            self.led_canvas.itemconfigure(self.led_shapes[1], fill=LED_GREEN if mask & 1 else LED_OFF)
            self.led_canvas.itemconfigure(self.led_shapes[2], fill=LED_BLUE if mask & 2 else LED_OFF)
        if "armed" in f:
            self.values["armed"].set("已使能" if f["armed"] == "1" else "已关闭")

    def pump(self):
        self.imu.tick()
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.connect_button.configure(text="断开", state="normal")
                    self.status.set("连接成功 · 心跳 250 ms · 状态自动刷新")
                    self.send("STAT")
                    self.send("SERVO STAT")
                elif kind == "reply":
                    try:
                        self.handle_reply(*payload)
                    except (KeyError, ValueError) as exc:
                        self.log(f"响应字段无效：{exc}")
                elif kind in ("error", "warning"):
                    self.status.set(str(payload))
                    self.log(f"{kind.upper()} {payload}")
                elif kind == "disconnected":
                    self.connected = False
                    self.chassis.disconnected()
                    self.navigation.disconnected()
                    self.servo.disconnected()
                    self.lift.disconnected()
                    self.rotation.disconnected()
                    self.flow.disconnected()
                    self.connection = None
                    self.connect_button.configure(text="连接", state="normal")
                    self.port_box.configure(state="readonly")
                    self.values["armed"].set("离线 / 未知")
                    self.link_mode.set("离线 · 上次数据已停止刷新")
                    self.log("连接已关闭；真实板的心跳租约将在超时后失效。")
        except queue.Empty:
            pass
        now = time.monotonic()
        if not self.closing:
            self.chassis.tick(now)
            self.lift.tick(now)
            self.rotation.tick(now)
            self.navigation.tick(now)
            self.flow.tick(now)
        if self.connected and not self.closing and self.connection.commands.qsize() < 3:
            if now - self.last_stat >= 1:
                self.send("STAT")
                self.send("SERVO STAT")
                self.last_stat = now
            elif now - self.last_poll >= .2:
                # Two axes per tick: a full seven-axis sweep is ~0.7 s, not ~1.4 s.
                for _ in range(2):
                    self.send(f"AXIS {self.axis_poll}")
                    self.axis_poll = self.axis_poll % 7 + 1
                self.last_poll = now
        if self.closing and self.connection is None:
            self.pump_job = None
            self.root.destroy()
            return
        self.pump_job = self.root.after(50, self.pump)

    def close(self):
        self.closing = True
        self.imu.close()
        self.navigation.cancel()
        self.chassis.halt()
        self.prefs.put("geometry", self.root.winfo_geometry())
        self.prefs.put("port", self.port.get())
        self.prefs.save()
        if self.connection:
            # Keep the pump alive: it drains the worker's "disconnected" event
            # and performs the final destroy.
            self.connection.close()
            return
        if self.pump_job is not None:
            # Cancel the pending tick first: destroying a window with a live
            # "after" callback makes Tk print an invalid-command-name error.
            self.root.after_cancel(self.pump_job)
            self.pump_job = None
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)  #description=__doc__ 表示把当前模块的文档字符串（文件开头的说明）作为帮助信息的描述。
    parser.add_argument("--demo", action="store_true", help="connect the simulated board automatically")
    args = parser.parse_args()
    root = tk.Tk()
    app = Console(root)
    if args.demo:
        root.after(100, app.toggle_connection)
    root.mainloop()


if __name__ == "__main__":
    main()
