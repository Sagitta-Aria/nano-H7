"""Tk standalone gyro turn controls. Mirrors the Web angle modes and protocol;
uses the existing serial worker and H723 bridge, never owns wheel scheduling.
"""
import tkinter as tk
from tkinter import ttk
from .turn import turn_command, gain_command, tuning_command
from .ui import MUTED


class TurnPanel:
    """Independent turn form within chassis; completion comes from TURN STAT."""
    def __init__(self, app, parent):
        self.app, self.supported, self.fields, self.last_poll = app, False, {}, 0.
        self.mode = tk.StringVar(value="相对当前方向转动")
        self.angle, self.sps, self.accel = (tk.StringVar(value=value) for value in ("90", "500", "2000"))
        self.status = tk.StringVar(value="未连接 · 需要支持闭环转角的固件")
        self.kp = tk.StringVar(value='20')
        self.tolerance = tk.StringVar(value='0.20')
        self.precision_tunable = False
        self.tunable = False
        frame = ttk.LabelFrame(parent, text="陀螺仪闭环转角")
        frame.pack(fill="x", pady=12)
        form = ttk.Frame(frame)
        form.pack(anchor="w", fill="x")
        ttk.Combobox(form, textvariable=self.mode, values=("相对当前方向转动", "转到零点角度"),
                     state="readonly", width=22).pack(side="left", padx=5)
        for label, variable in (("角度 °", self.angle), ("最高 step/s", self.sps), ("加速度 step/s²", self.accel)):
            ttk.Label(form, text=label).pack(side="left", padx=5)
            ttk.Entry(form, textvariable=variable, width=9).pack(side="left", padx=5)
        tuning = ttk.Frame(frame)
        tuning.pack(anchor="w", fill="x", pady=5)
        for label, variable in (("陀螺仪 P (step/s)/°", self.kp), ("角度容差 ±°", self.tolerance)):
            ttk.Label(tuning, text=label).pack(side="left", padx=5)
            ttk.Entry(tuning, textvariable=variable, width=9).pack(side="left", padx=5)
        buttons = ttk.Frame(frame)
        buttons.pack(anchor="w", pady=8)
        for label, action in (("当前方向设为零点", "zero"), ("执行转角", "start"), ("停止转角", "stop"), ("应用陀螺仪参数", "configure")):
            ttk.Button(buttons, text=label, command=lambda selected=action: self.action(selected)).pack(side="left", padx=5)
        ttk.Label(frame, textvariable=self.status, wraplength=900).pack(anchor="w", padx=5)
        ttk.Label(frame, text="顺时针为正，逆时针为负；−180～180°，最多两位小数。零点模式按最短路径转动。\n"
                  "误差在设定角度容差内稳定 300 ms 后完成；容差可调 0.01～10°，默认0.2°。P 同时用于直线纠偏和巡航，默认 20，无调参上限，非负数、两位小数，0关闭纠偏。直线容差为纠偏死区。H723 重连后须重设零点。",
                  foreground=MUTED, wraplength=900).pack(anchor="w", padx=5, pady=5)

    def action(self, action):
        """Validate before enqueueing; firmware rechecks fresh yaw and ownership."""
        if not self.app.connected or not self.supported:
            self.status.set("请连接支持闭环转角的 0.2.10 固件。")
            return
        if action == "stop":
            self.app.send("TURN STOP")
            return
        if self.app.navigation.uploading or self.app.chassis.intent or self.fields.get("state") == "turning" or \
                self.app.navigation.telemetry.get("state") in ("turning", "moving", "orienting", "waiting", "blocked"):
            self.status.set("请先停止底盘运动或坐标巡航。")
            return
        if action == 'configure':
            if not self.precision_tunable:
                self.status.set('可调陀螺仪 P 和角度容差需要 0.2.16 固件。')
                return
            try:
                self.app.send(tuning_command(self.kp.get(), self.tolerance.get()))
            except ValueError as error:
                self.status.set(str(error))
            return
        if self.app.connection.port != "SIM" and not self.app.imu.hub.navigation_feedback()["valid"]:
            self.status.set("请先连接 H723 并等待校准完成及新航向数据。")
            return
        if action == "zero":
            self.app.send("TURN ZERO")
            return
        if self.app.values["armed"].get() != "已使能":
            self.status.set("请先使能输出。")
            return
        mode = "ABS" if self.mode.get() == "转到零点角度" else "REL"
        if mode == "ABS" and self.fields.get("zero") != "1":
            self.status.set("请先将当前方向设为转角零点。")
            return
        try:
            body = turn_command(mode, self.angle.get(), int(self.sps.get()), int(self.accel.get()))
        except ValueError as error:
            self.status.set(str(error))
            return
        self.app.send(body)

    def reply(self, command, reply):
        """Render board-confirmed reference, task result and feedback freshness."""
        if not reply.ok:
            if command.startswith("TURN "):
                self.status.set("转角命令被拒绝：" + reply.fields.get("code", reply.body))
            return
        fields = reply.fields
        if fields.get("cmd") == "HELLO":
            self.supported = fields.get("turn") == "1"
            self.tunable = fields.get('turn_tuning') == '1'
            self.precision_tunable = fields.get('turn_precision') == '1'
        elif fields.get("cmd") == "TURN_STAT":
            self.fields = fields
            names = {"idle":"空闲", "turning":"闭环转动中", "done":"转角完成", "stopped":"已停止", "fault":"转角故障"}
            self.status.set("{} · 零点{} · 本次基准角 {:.2f}° / 目标 {:.2f}° / 误差 {:.2f}° · {} · {}".format(
                names.get(fields.get("state"), "待查询"), "已设置" if fields.get("zero") == "1" else "未设置",
                int(fields.get("current_cd", 0))/100, int(fields.get("target_cd", 0))/100,
                int(fields.get("error_cd", 0))/100, "航向有效" if fields.get("imu") == "1" else "航向过期",
                fields.get("fault", "none")))
            if fields.get('kp_centi'):
                self.status.set(self.status.get() + ' · 板端 P={} · 角度容差 ±{:.2f}°'.format(int(fields['kp_centi'])/100, int(fields.get('tolerance_cd',20))/100))

    def tick(self, now):
        if self.app.connected and self.supported and self.app.connection.commands.qsize() < 3 and now - self.last_poll >= .3:
            self.app.send("TURN STAT")
            self.last_poll = now

    def disconnected(self):
        self.supported, self.fields = False, {}
        self.status.set("离线 · 转角状态未知")
