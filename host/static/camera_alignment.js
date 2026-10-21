import { VisionTuningView } from "/vision_tuning.js";
/* Camera-one recognition/alignment controls. NX owns continuous raw-frame
   detection and motion. Browser photos are for viewing only. */
export class CameraAlignmentView {
  constructor(post, camera, onState) {
    this.post = post; this.camera = camera; this.onState = onState;
    this.role = "controller"; this.status = {}; this.config = null;
    this.saving = false; this.polling = false; this.imageRevision = null;
    this.roiBusy = false; this.roiRunning = false; this.roiRevision = 0;
    this.node = key => document.getElementById(`vision-${key}`);
    this.node("save").addEventListener("click", () => this.save());
    this.node("recognize").addEventListener("click", () => this.action("recognize"));
    this.node("start").addEventListener("click", () => this.action("start"));
    this.node("stop").addEventListener("click", () => this.action("stop"));
    this.node("kind").addEventListener("change", () => this.controls());
    this.tuning = new VisionTuningView(this);
    this.node('target').addEventListener('change', () => {
      try { this.tuning.collectColor(); this.tuning.color = this.node('target').value; this.tuning.renderColor(); }
      catch (error) { this.node('config-state').textContent = error.message; }
    });
    this.node('roi-preview').addEventListener('click', () => this.previewRoi());
    this.node('roi-live').addEventListener('click', () => this.liveRoi());
    this.node('roi-stop').addEventListener('click', () => this.stopRoi());
    this.node('roi-b').addEventListener('click', async () => {
      try {
        const response = await fetch('/api/b-pickup/config');
        if (!response.ok) throw new Error('读取B点识别框失败。');
        const {config} = await response.json();
        ['x','y','width','height'].forEach(key => { this.node(`roi-${key}`).value = config[`roi_${key}_pct`]; });
      } catch (error) { this.node('roi-state').textContent = error.message; }
    });
    document.addEventListener('visibilitychange', () => { if (document.hidden) this.stopRoi(); });
    document.querySelectorAll('[data-tab],[data-open-tab]').forEach(button => button.addEventListener('click', () => this.stopRoi()));
    this.initialise();
  }

  /* Metadata-only initialization; visiting this tab never opens a camera. */
  async initialise() {
    try {
      const data = await fetch("/api/camera/vision/state").then(r => r.json());
      this.adoptConfig(data.config); this.update(data.status);
    } catch (error) { this.node("config-state").textContent = `视觉设置读取失败：${error.message}`; }
  }

  setRole(role, taskActive = false) { this.role = role; this.taskActive = taskActive; this.controls(); }

  /* Telemetry never replaces the user's edited offset, speed or deadzone. */
  adoptConfig(config) {
    this.config = config;
    this.tuning.adopt(config);
    this.node("kind").value = config.kind; this.node("target").value = config.target;
    this.node("min-area").value = config.min_area; this.node("black-max").value = config.black_max;
    ["x", "y"].forEach((axis, i) => { this.node(`offset-${axis}`).value = config.target_offset_px[i]; });
    this.node("sps").value = config.tracking.sps;
    this.node("deadzone").value = config.tracking.deadzone_px;
    this.node("camera-yaw").value = config.tracking.camera_yaw_deg;
    for (const [id,key] of [["range-steps","range_steps"],["scan-step","scan_step"],["spacing-steps","spacing_steps"],["scan-sps","sps"],["angle-tolerance","angle_tolerance_deg"],["max-turn","max_turn_deg"]]) this.node(id).value = config.ring_sequence[key];
    for (const [id,key] of [["kp","kp"],["ki","ki"],["kd","kd"],["min-sps","min_sps"],["accel","accel"]]) this.node(id).value = config.tracking[key];
    for (const axis of ["x", "y"]) this.node(`reverse-${axis}`).checked = config.tracking[`reverse_${axis}`];
    this.controls();
  }

  read() {
    if (!this.config) throw new Error("视觉设置尚未加载。");
    const config = structuredClone(this.config);
    const value = id => { const text = this.node(id).value.trim(); if (!text || !Number.isFinite(Number(text))) throw new Error("请填写有效的目标偏移、速度和死区。"); return Number(text); };
    config.kind = this.node("kind").value; config.target = this.node("target").value.trim();
    config.min_area = value("min-area"); config.black_max = value("black-max");
    config.target_offset_px = [value("offset-x"), value("offset-y")];
    config.tracking.sps = value("sps"); config.tracking.deadzone_px = value("deadzone");
    config.tracking.camera_yaw_deg = value("camera-yaw");
    for (const [id,key] of [["range-steps","range_steps"],["scan-step","scan_step"],["spacing-steps","spacing_steps"],["scan-sps","sps"],["angle-tolerance","angle_tolerance_deg"],["max-turn","max_turn_deg"]]) config.ring_sequence[key] = value(id);
    for (const [id,key] of [["kp","kp"],["ki","ki"],["kd","kd"],["min-sps","min_sps"],["accel","accel"]]) config.tracking[key] = value(id);
    for (const axis of ["x", "y"]) config.tracking[`reverse_${axis}`] = this.node(`reverse-${axis}`).checked;
    config.detector = this.tuning.read();
    return config;
  }

  async save() {
    if (this.role === "readonly" || this.status.busy || this.saving || this.taskActive) return false;
    this.saving = true; this.controls();
    try {
      const result = await this.post("/api/camera/vision/config", {config: this.read()});
      if (!result.ok) return false;
      this.adoptConfig(result.config);
      this.node("config-state").textContent = this.config.kind === "ring"
        ? `设置已保存 · 点1视觉校准，点间距 ${this.config.ring_sequence.spacing_steps} 脉冲`
        : "对准设置已保存 · NX 视觉 PID 连续调速";
      return true;
    } catch (error) { this.node("config-state").textContent = error.message; return false; }
    finally { this.saving = false; this.controls(); }
  }

  /* Each explicit action pins current settings. Local photos cannot enter a
     physical run, whose acquisition stays on the service host. */
  async action(action) {
    this.stopRoi();
    if (this.role === "readonly" || (action !== "stop" && this.taskActive)) return;
    try {
      if (action !== "stop" && !await this.save()) return;
      const payload = {};
      if (!this.camera.nano && action !== "stop") throw new Error("识别和对准请选择 Nano 来源并打开 NX 摄像头一。本机照片仅供查看。");
      const result = await this.post(`/api/camera/vision/${action}`, payload);
      if (result.ok) this.update(result.status);
    } catch (error) { this.node("config-state").textContent = error.message; }
  }

  update(status) {
    if (!status) return;
    this.status = status;
    this.onState(status);
    const names = {idle:"未启动", previewing:"NX 正在识别 · 不移动", preview:"NX 识别完成 · 不移动", initializing:"读取底盘方向", claw_opening:"S1 打开至 170° · 等待识别", zero:"记录车身朝向", tracking:"视觉 PID 连续靠近", braking:"换向减速", settling:"中心死区停车确认", pickup_wait:"B点已下降 · 框内等待目标颜色", done:"对准完成", stopped:"已停止", fault:"对准停止"};
    this.node("state").textContent = `${names[status.state] || status.state}${status.error ? `：${status.error}` : ""}`;
    const sequence = status.ring_sequence;
    if (sequence) {
      const completed = sequence.identity_source === "remembered_color" ? "第二次D点：中间圆环校准完成" : sequence.returning ? "1 → 2 → 3 → 2 定位完成" : "1 → 2 → 3 定位完成";
      const phases = {transfer:"按统一点距返回点2", search:"前后搜索同帧双圆", outward:"向地图外沿补搜移动", recover:"丢圆后往返搜索，等待前进时找到双圆", advance:"已重获双圆，继续前进 500 脉冲", align:"按圆心连线摆正", realign_brake:"圆列角度超差，停车后重新摆正", seek:`寻找点 ${sequence.expected}`, approach:`点 ${sequence.expected} 视觉靠近校准`, spacing:`${sequence.returning ? "后退返回" : "前进到"}点 ${sequence.expected} · 固定 ${sequence.spacing_steps} 脉冲`, dwell:`已到达点 ${sequence.expected}，衔接下一段`, complete:completed};
      if (sequence.identity_source === "remembered_color" && sequence.expected == null) phases.seek = "寻找可由第一批颜色确认的圆环";
      this.node("state").textContent = `${status.active ? phases[sequence.phase] : (status.state === "done" ? completed : (names[status.state] || status.state))}${status.error ? `：${status.error}` : ""}`;
    }
    const v = status.vision;
    const speed = status.velocity;
    this.node("result").textContent = v ? (v.found ? `目标中心 (${v.center.join(", ")}) · 控制点 (${v.control_point.join(", ")}) · 画面中心 (${v.reference.join(", ")}) · X 偏差 ${v.error_dx} px · Y 偏差 ${v.error_dy} px · ${status.state === "done" ? "已进入死区，对准完成" : "闭环更新 " + status.iterations + " 次"}${speed ? ` · 右移 ${speed.right} / 前进 ${speed.forward} step/s` : ""}` : "未找到指定目标") : "等待识别";
    if (sequence) this.node("result").textContent = `已到达：${sequence.recognized.map(item => item.point ?? item.digit).join(" → ") || "无"} · ${sequence.recognized.length ? `相对点1 ${sequence.position_steps} 脉冲 · 点间距 ${sequence.spacing_steps} 脉冲` : `搜索位置 ${sequence.position_steps} / ±${sequence.range_steps} 脉冲`}${sequence.line ? ` · 连线转角误差 ${sequence.line.error_deg.toFixed(1)}°` : ""}${status.streaming && v?.digit ? ` · 当前数字 ${v.digit}` : ""}`;
    if (sequence?.identity_source === 'remembered_color') {
      this.node("result").textContent = `第一批颜色恢复点号 · 已确认 ${sequence.recognized.map(item => item.point).join(" → ") || "无"} · 统一点距：1→2 ${sequence.offsets[2]} / 1→3 ${sequence.offsets[3]} 脉冲${v?.anchor_point ? ` · 当前点 ${v.anchor_point}` : " · 等待可确认的颜色圆环"}`;
    }
    if (status.streaming && sequence?.visible_digits?.length) this.node("result").textContent += ` · 画面数字 ${sequence.visible_digits.map(item => item.point ?? item.digit).join("、")}`;
    if (sequence?.phase === "approach" && v?.found) this.node("result").textContent += ` · 靠近误差 X ${v.error_dx} / Y ${v.error_dy} px`;
    if (status.control === 'pickup_color_wait') {
      this.node("state").textContent = `${status.state === 'done' ? 'B点目标色已确认' : names[status.state] || status.state}${status.error ? `：${status.error}` : ''}`;
      this.node("result").textContent = `底盘保持停止 · 连续确认 ${status.stable_frames}/3 帧${v?.roi ? ` · 识别框 (${v.roi.join(', ')}) px` : ''}${v?.found ? ' · 框内发现目标色' : ' · 等待目标色'} `;
    }
    // Manual photos belong to CameraView. Recognition never fetches a JPEG or
    // replaces that photo; all movement decisions run on NX without this page.
    this.controls();
    if (status.busy && !this.polling) this.poll();
  }

  /* Poll only during an explicitly requested operation; leaving the page
     does not own the controller's progress or completion. */
  async poll() {
    this.polling = true;
    try {
      while (this.status.busy) {
        await new Promise(resolve => setTimeout(resolve, 250));
        const data = await fetch("/api/camera/vision/state", {cache:"no-store"}).then(r => r.json());
        this.update(data.status);
      }
    } catch (error) { this.node("config-state").textContent = `视觉状态连接中断：${error.message}`; }
    finally { this.polling = false; }
  }

  /* Diagnostic ROI requests consume real camera frames, never motion or saved ROI. */
  async previewRoi() {
    if (this.role === 'readonly' || this.taskActive || this.roiBusy || (this.status.busy && !this.status.tuning)) return false;
    this.roiBusy = true; this.controls();
    const revision = this.roiRevision;
    try {
      if (!this.camera.nano) throw new Error('请先打开NX摄像头1。');
      const config = this.read();
      if (config.kind !== 'block') throw new Error('请将上方识别类型切换为色块。');
      const roi = ['x','y','width','height'].map(key => {
        const input = this.node(`roi-${key}`), value = Number(input.value);
        if (!input.value.trim() || !Number.isInteger(value) || value < Number(input.min) || value > Number(input.max)) throw new Error('识别框须填写有效整数百分比。');
        return value;
      });
      if (roi[0]+roi[2]>100 || roi[1]+roi[3]>100) throw new Error('识别框不能超出画面。');
      const result = await this.post('/api/camera/vision/tune', {config, preview_roi_pct:roi});
      if (!result.ok) throw new Error(result.error || '调试预览失败。');
      if (revision !== this.roiRevision) return false;
      this.node('roi-image').src = result.image; this.node('roi-mask').src = result.mask;
      this.node('roi-images').hidden = false;
      const v = result.report, settings = v.color_settings;
      this.node('roi-state').textContent = `目标 ${v.target} · ${v.found ? `找到 (${v.center.join(', ')})` : '未找到目标'} · 框 (${v.roi.join(', ')}) px · HSV ${settings.ranges[0][0].join(',')} ～ ${settings.ranges[0][1].join(',')} · 最小面积 ${settings.min_area} px²`;
      return true;
    } catch (error) { this.node('roi-state').textContent = error.message; return false; }
    finally { this.roiBusy = false; this.controls(); }
  }

  /* Poll only during explicit debug use; late replies cannot revive a stopped loop. */
  async liveRoi() {
    if (this.roiRunning || this.roiBusy) return;
    this.roiRunning = true; this.controls();
    const revision = this.roiRevision;
    while (this.roiRunning && revision === this.roiRevision) {
      if (!await this.previewRoi()) { this.stopRoi(); break; }
      await new Promise(resolve => setTimeout(resolve, 500));
    }
  }

  stopRoi() { this.roiRunning = false; this.roiRevision++; this.controls(); }

  controls() {
    const locked = this.role === "readonly" || this.status.busy || this.saving || this.taskActive;
    document.querySelectorAll("#vision-editor input, #vision-editor select").forEach(node => { node.disabled = locked; });
    this.node("target").disabled = locked || this.node("kind").value === "ring";
    this.node("black-max").disabled = locked || this.node("kind").value !== "ring";
    for (const id of ["range-steps", "scan-step", "spacing-steps", "scan-sps", "angle-tolerance", "max-turn"]) this.node(id).disabled = locked || this.node("kind").value !== "ring";
    this.node("start").textContent = this.node("kind").value === "ring" ? "校准点1并定距移动到2、3" : "开始车辆对准";
    for (const id of ["save", "recognize", "start"]) this.node(id).disabled = locked || !this.config;
    this.node("stop").disabled = this.role === "readonly" || !this.status.busy;
    this.tuning?.controls();
    const roiLocked = this.role === 'readonly' || this.taskActive || (this.status.busy && !this.status.tuning) || !this.config;
    for (const id of ['roi-preview','roi-live','roi-b']) this.node(id).disabled = roiLocked || this.roiBusy || this.roiRunning;
    this.node('roi-stop').disabled = !this.roiRunning && !this.roiBusy;
  }
}
