"""Tk view of MCU-owned optical flow vectors; never reconstruct totals from polls."""
import math
import tkinter as tk
from tkinter import ttk
from .optical_flow import FlowSettings, config_command


class FlowPanel:
    """Calibration and recording controls using the shared H743 serial worker."""
    def __init__(self, app, parent):
        self.app, self.fields, self.supported, self.last_poll = app, {}, False, 0.
        self.settings = FlowSettings()
        self.mode = tk.StringVar(value="固定轴向")
        self.status = tk.StringVar(value="未连接光流固件")
        self.total = tk.StringVar(value="—")
        self.relative = tk.StringVar(value="—")
        self.inputs = {k: tk.StringVar(value=str(v)) for k, v in self.settings.snapshot().items()}
        self.swap, self.neg_x, self.neg_y = (tk.BooleanVar(value=bool(self.settings.value['map'] & b)) for b in (4, 1, 2))
        ttk.Label(parent, text="PB11 接模块 TX，PB14 接模块 RX；模块 5V 供电、3.3V UART，共地。\n"
                  "上电累计保存在 H743；开始记录只建立新时间点。往返矢量相抵消，长度是净位移。", wraplength=850).pack(anchor="w")
        ttk.Combobox(parent, textvariable=self.mode, values=("固定轴向", "结合 H723 航向"), state="readonly").pack(anchor="w", pady=8)
        self.mode.trace_add("write", lambda *args: self.render())
        for label, var in (("上电累计 · mm", self.total), ("记录时段 · mm", self.relative), ("接收状态", self.status)):
            ttk.Label(parent, text=label).pack(anchor="w", pady=(8, 0))
            ttk.Label(parent, textvariable=var, wraplength=900).pack(anchor="w")
        self.canvas = tk.Canvas(parent, height=230, background="white", highlightthickness=0)
        self.canvas.pack(fill="x", pady=8); self.canvas.bind("<Configure>", lambda e: self.render())
        self.buttons = []
        self.applied = tk.StringVar(value="等待板端参数回读")
        row = ttk.Frame(parent); row.pack(anchor="w")
        for label, body in (("从现在开始记录", "FLOW START"), ("结束并保留记录", "FLOW STOP")):
            button = ttk.Button(row, text=label, command=lambda b=body: self.send(b), state="disabled")
            button.pack(side="left", padx=4); self.buttons.append(button)
        form = ttk.LabelFrame(parent, text="换算与安装方向 · 保存后明确应用"); form.pack(fill="x", pady=12)
        for i, (key, label) in enumerate((("height_mm", "高度 mm"), ("gain_x", "X 比例"), ("gain_y", "Y 比例"),
                                         ("angle_deg", "顺时针安装角 °"), ("min_quality", "最低置信度"))):
            ttk.Label(form, text=label).grid(row=i, column=0, sticky="w", padx=5, pady=3)
            ttk.Entry(form, textvariable=self.inputs[key], width=15).grid(row=i, column=1, sticky="w", padx=5)
        for i, (label, var) in enumerate((("交换 X/Y", self.swap), ("反转 X", self.neg_x), ("反转 Y", self.neg_y))):
            ttk.Checkbutton(form, text=label, variable=var).grid(row=i, column=2, sticky="w", padx=8)
        ttk.Button(form, text="保存参数", command=self.save).grid(row=5, column=0, pady=8)
        self.apply_button = ttk.Button(form, text="应用已保存参数", command=self.apply, state="disabled")
        self.apply_button.grid(row=5, column=1, pady=8)
        ttk.Label(parent, textvariable=self.applied, wraplength=900).pack(anchor="w")
        ttk.Label(parent, text="默认高度 30 mm 为待校准值。新参数只影响后续增量，历史累计不重算；建议改参数后重新开始记录。\n"
                  "航向模式缺失 H723 时暂停对应累计并报告缺口；H723 会话变化后锁定地面坐标，须重启 H743 建立新坐标。", wraplength=900).pack(anchor="w")

    def send(self, body):
        """Only enqueue supported sensor commands; no motor enable is required."""
        if not self.app.connected or not self.supported:
            self.status.set("请连接支持光流的 0.2.11 固件。")
            return
        self.app.send(body)

    def save(self):
        """Validate and persist a draft; saving alone does not change hardware."""
        try:
            c = {k: self.inputs[k].get() for k in ("height_mm", "gain_x", "gain_y", "angle_deg")}
            c.update(map=int(self.neg_x.get()) | int(self.neg_y.get()) << 1 | int(self.swap.get()) << 2,
                     min_quality=int(self.inputs["min_quality"].get()))
            self.settings.save(c); self.status.set("参数已保存，尚未应用。")
        except (ValueError, OSError) as e:
            self.status.set(str(e))

    def apply(self):
        """Apply the saved snapshot; live acknowledgement arrives through reply."""
        self.send(config_command(self.settings.snapshot()))

    def reply(self, command, reply):
        """Merge split board telemetry; configuration does not clear the totals."""
        if not reply.ok:
            if command.startswith("FLOW "):
                self.status.set("光流命令被拒绝：" + reply.body)
            return
        f = reply.fields
        if f.get("cmd") == "HELLO":
            self.supported = f.get("flow") == "1"
            self.fields = {}; self.render()
        elif f.get("cmd", "").startswith("FLOW_"):
            self.fields.update(f); self.render()

    def render(self):
        """Display net vectors, MCU timestamps, missing-data state and arrows."""
        f = self.fields; world = self.mode.get() != "固定轴向"
        keys = ("wx_um", "wy_um", "wrx_um", "wry_um") if world else ("x_um", "y_um", "rx_um", "ry_um")
        v = [int(f.get(k, 0))/1000 for k in keys]
        self.total.set("X {:+.3f} · Y {:+.3f} · 合成 {:.3f}".format(v[0], v[1], math.hypot(*v[:2]))
                       if all(k in f for k in keys[:2]) else "—")
        self.relative.set("未开始记录" if f.get("marked") != "1" else
                          "X {:+.3f} · Y {:+.3f} · 合成 {:.3f} · MCU起点 {:.3f}s · {}".format(
                              v[2], v[3], math.hypot(*v[2:]), int(f.get("start_ms", 0))/1000,
                              "记录中" if f.get("recording") == "1" else "已结束"))
        ready = self.app.connected and self.supported
        self.buttons[0].configure(state="normal" if ready else "disabled")
        self.buttons[1].configure(state="normal" if ready and f.get("recording") == "1" else "disabled")
        self.apply_button.configure(state="normal" if ready else "disabled")
        if "height_um" in f:
            self.applied.set("板端高度 {} mm · X/Y比例 {} / {} · 映射 {} · 安装角 {}° · 最低置信度 {}".format(
                int(f['height_um'])/1000, int(f['gx_ppm'])/1e6, int(f['gy_ppm'])/1e6,
                f['map'], int(f['angle_cd'])/100, f['minq']))
        warning = " · 航向缺失/会话锁定，地面累计不完整" if world and (f.get("heading") != "1" or int(f.get("gaps", 0))) else ""
        self.status.set("{} · 置信度 {} · 坏帧 {} / 过滤 {} / UART错误 {} · 参数版本 {}{}".format(
            "在线" if f.get("online") == "1" else "未收到新数据", f.get("valid", "—"),
            f.get("bad", 0), f.get("low", 0), f.get("uart", 0), f.get("rev", 0), warning))
        w, h = max(self.canvas.winfo_width(), 300), 230
        self.canvas.delete("all"); self.canvas.create_line(20, h/2, w-20, h/2, fill="#9aa3af")
        self.canvas.create_line(w/2, 15, w/2, h-15, fill="#9aa3af")
        scale = min(w/2-25, h/2-25)/max(1, *map(abs, v))
        for x, y, colour in ((v[0], v[1], "#147bba"), (v[2], v[3], "#159b68")):
            self.canvas.create_line(w/2, h/2, w/2+x*scale, h/2-y*scale, fill=colour, width=3, arrow="last")

    def tick(self, now):
        """Low-rate polling reads accumulated values rather than sample deltas."""
        if self.app.connected and self.supported and self.app.connection.commands.qsize() < 3 and now-self.last_poll >= .3:
            for body in ("FLOW STAT", "FLOW WORLD", "FLOW CONFIG"):
                self.app.send(body)
            self.last_poll = now

    def disconnected(self):
        """Freeze old data visibly until new board telemetry is received."""
        self.supported = False; self.render(); self.status.set("离线 · 上次位移已停止刷新")
