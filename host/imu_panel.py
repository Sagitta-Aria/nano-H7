"""Tk view of the same guarded H723 USB connection used by the Web console.

All drawing stays on Tk's main thread. The hub owns USB, handshake and samples;
this panel does not talk to H743 or estimate position by integrating IMU data.
"""
import tkinter as tk
from tkinter import ttk

from .imu import ImuHub
from .ui import CARD, INK, MUTED, FONT, MONO


class ImuPanel:
    """Independent connection/plots inside the existing local Tk notebook."""
    def __init__(self, app, parent):
        """Create UI once; app.pump must call tick and app.close must call close."""
        self.app, self.parent = app, parent
        self.hub = ImuHub()
        self.points = []
        self.generation = None
        self.frame = self.hub.snapshot()
        self.port = tk.StringVar()
        self.status = tk.StringVar(value="未连接 H723")
        self.identity = tk.StringVar(value="板型尚未确认")
        self.readouts = tk.StringVar(value="Roll — °    Pitch — °    Yaw — °    X — m/s²    Y — m/s²")
        self.mode = tk.StringVar(value="线性加速度（去重力）")
        bar = ttk.Frame(parent, padding=10)
        bar.pack(fill="x")
        self.select = ttk.Combobox(bar, textvariable=self.port, state="readonly", width=35)
        self.select.pack(side="left")
        ttk.Button(bar, text="刷新", command=self.refresh).pack(side="left", padx=5)
        self.connect_button = ttk.Button(bar, text="握手连接 H723", command=self.connect)
        self.connect_button.pack(side="left")
        ttk.Button(bar, text="断开 H723", command=self.hub.close).pack(side="left", padx=5)
        ttk.Label(parent, textvariable=self.status).pack(anchor="w", padx=12)
        ttk.Label(parent, textvariable=self.identity, font=(MONO, 9)).pack(anchor="w", padx=12)
        ttk.Label(parent, textvariable=self.readouts, font=(MONO, 12)).pack(anchor="w", padx=12, pady=8)
        modes = ttk.Combobox(parent, textvariable=self.mode, state="readonly",
                            values=["线性加速度（去重力）", "加速度计（含重力）"], width=27)
        modes.pack(anchor="w", padx=12)
        modes.bind("<<ComboboxSelected>>", lambda event: self.draw())
        plots = ttk.Frame(parent)
        plots.pack(fill="both", expand=True, padx=10, pady=6)
        plots.columnconfigure(0, weight=1); plots.columnconfigure(1, weight=1)
        self.charts = []
        for i in range(3):
            canvas = tk.Canvas(plots, bg=CARD, height=220, highlightthickness=0)
            canvas.grid(row=i//2, column=i%2, sticky="nsew", padx=4, pady=4)
            canvas.bind("<Configure>", lambda event: self.draw())
            self.charts.append(canvas)
        ttk.Label(plots, text="X 向前 · Y 向左 · Z 向上\nYaw 是相对航向，会漂移。\nXY 图为加速度矢量，不是位置。\n握手/失效时不显示有效实时数值。",
                  foreground=MUTED, justify="left").grid(row=1, column=1, sticky="nw", padx=12, pady=20)
        self.refresh()

    def refresh(self):
        """List metadata only; no arbitrary port is opened or auto-selected."""
        self.ports = {f"{p['id']} · {p['description']} · 未确认": p["id"] for p in self.hub.ports()["ports"]}
        self.select["values"] = list(self.ports)
        if self.port.get() not in self.ports:
            self.port.set("")
        if not self.ports and self.hub.state == "offline":
            self.status.set("未发现 ST USB CDC；接入达妙板数据线后刷新。")

    def connect(self):
        """Require an explicit candidate and reject an already-owned H743 port."""
        port = self.ports.get(self.port.get())
        if not port:
            self.status.set("请先选择 H723 候选端口。")
            return
        if self.app.connection and self.app.connection.port == port:
            self.status.set("该串口正在由 H743 占用。")
            return
        ok, message = self.hub.connect(port)
        self.status.set("正在核对 H723 身份…" if ok else message)

    def tick(self):
        """Consume bounded worker batches on the Tk thread, at most once per pump."""
        for frame in self.hub.drain():
            self.frame = frame
            if self.generation != frame["generation"]:
                self.points = []
                self.generation = frame["generation"]
            self.points.extend(frame["points"])
            self.points = self.points[-1500:]
            self.status.set(frame["message"])
            identity = frame["identity"]
            self.identity.set(f"{identity.get('board','—')} · {identity.get('mcu','—')} · UID {identity.get('uid','—')}")
            busy = frame["state"] not in ("offline", "error")
            self.connect_button.configure(state="disabled" if busy else "normal")
            self.select.configure(state="disabled" if busy else "readonly")
            self.draw()

    def axes(self, canvas, xmin, xmax, ymin, ymax, title, xlabel, ylabel):
        """Draw numeric coordinate axes and return data-to-canvas transforms."""
        canvas.delete("all")
        w, h = max(300, canvas.winfo_width()), max(200, canvas.winfo_height())
        left, right, top, bottom = 62, w-18, 32, h-40
        x = lambda v: left + (v-xmin)/(xmax-xmin)*(right-left)
        y = lambda v: bottom - (v-ymin)/(ymax-ymin)*(bottom-top)
        canvas.create_text(12, 12, text=f"{title} · {ylabel}", anchor="w", fill=INK, font=(FONT, 10))
        for i in range(5):
            xv, yv = xmin+(xmax-xmin)*i/4, ymin+(ymax-ymin)*i/4
            canvas.create_line(x(xv), top, x(xv), bottom, fill=MUTED, stipple="gray25")
            canvas.create_line(left, y(yv), right, y(yv), fill=MUTED, stipple="gray25")
            canvas.create_text(x(xv), bottom+12, text=f"{xv:.1f}", fill=MUTED)
            canvas.create_text(left-6, y(yv), text=f"{yv:.1f}", anchor="e", fill=MUTED)
        canvas.create_text((left+right)/2, h-9, text=xlabel, fill=MUTED)
        return x, y

    def draw(self):
        """Render valid angle/acceleration histories and an equal-scale XY vector."""
        if self.app.notebook.select() != str(self.parent):
            return
        field = "linear_accel_ms2" if self.mode.get().startswith("线性") else "accel_ms2"
        last = self.frame.get("latest")
        valid = self.frame["state"] == "streaming" and last and last["valid"]
        self.readouts.set((f"Roll {last['roll_deg']:.2f} °    Pitch {last['pitch_deg']:.2f} °    Yaw {last['yaw_deg']:.2f} °    X {last[field][0]:.3f}    Y {last[field][1]:.3f} m/s²")
                         if valid else "Roll — °    Pitch — °    Yaw — °    X — m/s²    Y — m/s²")
        end = max(15., self.points[-1]["time_s"] if self.points else 0.)
        points = [p for p in self.points if p["time_s"] >= end-15]
        series = [[lambda p: p["roll_deg"], lambda p: p["pitch_deg"], lambda p: p["yaw_deg"]],
                  [lambda p: p[field][0], lambda p: p[field][1]]]
        colors = ["#0b82b8", "#0a8065", "#6b5bd6"]
        for i, functions in enumerate(series):
            canvas = self.charts[i]
            bound = max([10. if i == 0 else 1.] + [abs(f(p))*1.1 for p in points if p["valid"] for f in functions])
            x, y = self.axes(canvas, end-15, end, -bound, bound, "Roll / Pitch / Yaw" if i == 0 else "X / Y 加速度", "时间 (s)", "°" if i == 0 else "m/s²")
            for f, color in zip(functions, colors):
                previous = None
                for p in points:
                    current = (x(p["time_s"]), y(f(p)), p["time_s"]) if p["valid"] else None
                    if current and previous and current[2]-previous[2] < .3:
                        canvas.create_line(*previous[:2], *current[:2], fill=color, width=1.5)
                    previous = current
        canvas = self.charts[2]
        ratio = (max(300, canvas.winfo_width())-80)/(max(200, canvas.winfo_height())-72)
        bound = max([1.] + [max(abs(p[field][0]),abs(p[field][1])*ratio)*1.2 for p in points[-200:] if p["valid"]])
        x, y = self.axes(canvas, -bound, bound, -bound/ratio, bound/ratio, "XY 加速度矢量", "X 加速度 (m/s²)", "Y (m/s²)")
        if valid:
            px, py = x(last[field][0]), y(last[field][1])
            canvas.create_line(x(0), y(0), px, py, fill=colors[1], width=2, arrow="last")
            canvas.create_oval(px-4, py-4, px+4, py+4, fill=colors[1], outline=colors[1])

    def close(self):
        """Release only the IMU channel when the containing app exits."""
        self.hub.close()
