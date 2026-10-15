/* Independent H723 view. It consumes verified telemetry only; it never sends
   robot commands or integrates acceleration into a fictitious position. */
export class ImuView {
  constructor(post) {
    this.post = post;
    this.points = [];
    this.frame = { state: "offline", message: "未连接 H723", identity: {} };
    this.generation = null;
    this.readonly = false;
    this.pendingDraw = false;
    this.lastKey = null;
    this.mode = "linear_accel_ms2";
    const byId = (id) => document.getElementById(id);
    byId("imu-refresh").addEventListener("click", () => this.refreshPorts());
    byId("imu-port").addEventListener("change", () => this.render());
    byId("imu-connect").addEventListener("click", () => this.connect());
    byId("imu-disconnect").addEventListener("click", () => this.post("/api/imu/disconnect", {}));
    byId("imu-clear").addEventListener("click", () => { this.points = []; this.drawSoon(); });
    byId("imu-accel-mode").addEventListener("change", (event) => {
      this.mode = event.target.value;
      this.render();
    });
    this.resize = new ResizeObserver(() => this.drawSoon());
    for (const canvas of document.querySelectorAll(".imu-chart")) this.resize.observe(canvas);
    this.themes = new MutationObserver(() => this.drawSoon());
    this.themes.observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
    this.refreshPorts();
    // USB hotplug and automatic links must become visible on an already-open page.
    this.portRefresh = setInterval(() => {
      if (!document.hidden && document.getElementById("tab-imu").classList.contains("active")) this.refreshPorts();
    }, 2000);
    fetch("/api/imu/state").then(r => r.json()).then(frame => this.update(frame)).catch(() => {});
    this.render();
  }

  /* Metadata listing does not open a device. No candidate is selected silently. */
  async refreshPorts() {
    if (this.refreshingPorts) return;
    this.refreshingPorts = true;
    const select = document.getElementById("imu-port");
    const chosen = select.value;
    try {
      const data = await fetch("/api/imu/ports").then(r => r.json());
      select.replaceChildren(new Option("请选择 ST USB CDC 候选端口", ""));
      for (const port of data.ports) {
        const verified = port.id === this.frame.port && this.frame.identity?.uid &&
          ["streaming", "calibrating", "invalid", "stale"].includes(this.frame.state);
        select.add(new Option(`${port.id} · ${port.description} · ${verified ? "已握手确认" : "未握手确认"}`, port.id));
      }
      const current = ["streaming", "calibrating", "invalid", "stale", "handshaking"].includes(this.frame.state)
        ? this.frame.port : chosen;
      if (data.ports.some(p => p.id === current)) select.value = current;
      document.getElementById("imu-port-note").textContent = data.ports.length
        ? "候选端口需通过 H723 专用握手才能显示数据。"
        : "未发现 ST USB CDC。接入达妙板的数据线后刷新。";
    } catch {
      document.getElementById("imu-port-note").textContent = "串口列表读取失败，请刷新。";
    } finally {
      this.refreshingPorts = false;
    }
    this.render();
  }

  /* An explicit selection is required; rejected handshakes remain visible. */
  async connect() {
    const port = document.getElementById("imu-port").value;
    if (!port) return;
    const result = await this.post("/api/imu/connect", { port });
    if (result.ok) this.update({ state: "handshaking", message: "正在核对 H723 身份…", port });
    else document.getElementById("imu-state").textContent = result.error || "连接被拒绝";
  }

  /* Server generation separates reconnections; points arrive in bounded batches. */
  update(frame) {
    if (frame.generation !== undefined && this.generation !== null && frame.generation < this.generation) return;
    if (frame.generation === this.generation && frame.revision !== undefined && frame.revision < (this.frame.revision || 0)) return;
    if (frame.generation !== undefined && this.generation !== frame.generation) {
      this.generation = frame.generation;
      this.points = [];
      this.lastKey = null;
    }
    this.frame = { ...this.frame, ...frame };
    if (frame.history) { this.points = []; this.lastKey = null; }
    for (const point of (frame.history || frame.points || [])) {
      const key = `${point.timestamp_ms}:${point.sample_count}:${point.time_s}`;
      if (key !== this.lastKey) { this.points.push(point); this.lastKey = key; }
    }
    this.points = this.points.slice(-1500);
    this.render();
  }

  /* Readonly pages can inspect data and clear their own chart, but cannot reopen ports. */
  setRole(role) { this.readonly = role === "readonly"; this.render(); }

  render() {
    const get = (id) => document.getElementById(id);
    const frame = this.frame;
    const connected = ["streaming", "calibrating", "invalid", "stale"].includes(frame.state);
    const working = connected || ["handshaking", "closing"].includes(frame.state);
    get("imu-state").textContent = frame.message;
    get("imu-state").dataset.state = frame.state;
    get("imu-connect").disabled = this.readonly || working || !get("imu-port").value;
    get("imu-disconnect").disabled = this.readonly || !working;
    get("imu-port").disabled = working;
    const identity = frame.identity || {};
    get("imu-identity").textContent = identity.uid
      ? `${identity.board} · ${identity.mcu} · ${identity.sensor} · 固件 ${identity.fw} · UID ${identity.uid}`
      : "板型尚未确认";
    const latest = frame.latest;
    const valid = frame.state === "streaming" && latest?.valid;
    const fixed = (value, digits = 3) => valid && Number.isFinite(value) ? value.toFixed(digits) : "—";
    get("imu-roll").textContent = fixed(latest?.roll_deg, 2);
    get("imu-pitch").textContent = fixed(latest?.pitch_deg, 2);
    get("imu-yaw").textContent = fixed(latest?.yaw_deg, 2);
    get("imu-ax").textContent = fixed(latest?.[this.mode]?.[0]);
    get("imu-ay").textContent = fixed(latest?.[this.mode]?.[1]);
    get("imu-sample-status").textContent = latest
      ? `校准 ${latest.calibration_count}/2000 · 温度 ${latest.temperature_c.toFixed(1)} °C · 采样 ${latest.sample_count} · 坏帧 ${frame.rejected || 0}`
      : "等待通过握手后的采样数据";
    this.drawSoon();
  }

  /* Repaint once per animation frame; hidden panels are redrawn when resized. */
  drawSoon() {
    if (this.pendingDraw) return;
    this.pendingDraw = true;
    requestAnimationFrame(() => { this.pendingDraw = false; this.draw(); });
  }

  /* Scale a canvas for HiDPI and return consistent CSS-coordinate drawing state. */
  surface(id) {
    const canvas = document.getElementById(id);
    const width = Math.max(240, canvas.clientWidth || 600);
    const height = canvas.clientHeight || 260;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
    const ctx = canvas.getContext("2d");
    ctx.scale(ratio, ratio);
    const css = getComputedStyle(document.documentElement);
    const color = name => css.getPropertyValue(name).trim();
    ctx.fillStyle = color("--card"); ctx.fillRect(0, 0, width, height);
    ctx.font = "12px Segoe UI, Microsoft YaHei, sans-serif";
    return { ctx, width, height, color };
  }

  /* Draw labeled numeric axes. Both time charts and XY projection share units. */
  axes(surface, xmin, xmax, ymin, ymax, xlabel, ylabel) {
    const { ctx, width, height, color } = surface;
    const box = { left: 66, right: width - 20, top: 27, bottom: height - 46 };
    const x = v => box.left + (v - xmin) / (xmax - xmin) * (box.right - box.left);
    const y = v => box.bottom - (v - ymin) / (ymax - ymin) * (box.bottom - box.top);
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const xv = xmin + (xmax - xmin) * i / 4, yv = ymin + (ymax - ymin) * i / 4;
      ctx.strokeStyle = color("--line-soft");
      ctx.beginPath(); ctx.moveTo(x(xv), box.top); ctx.lineTo(x(xv), box.bottom); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(box.left, y(yv)); ctx.lineTo(box.right, y(yv)); ctx.stroke();
      ctx.fillStyle = color("--muted");
      ctx.textAlign = "center"; ctx.fillText(xv.toFixed(1), x(xv), box.bottom + 19);
      ctx.textAlign = "right"; ctx.fillText(yv.toFixed(1), box.left - 8, y(yv) + 4);
    }
    ctx.strokeStyle = color("--tick");
    ctx.strokeRect(box.left, box.top, box.right - box.left, box.bottom - box.top);
    ctx.fillStyle = color("--ink-2"); ctx.textAlign = "left";
    ctx.fillText(ylabel, 12, 15); ctx.textAlign = "center";
    ctx.fillText(xlabel, (box.left + box.right) / 2, height - 8);
    return { x, y, box };
  }

  /* A 15-second rolling chart; invalid samples split the path instead of being zeroes. */
  timeChart(id, series, label, minimum) {
    const surface = this.surface(id), { ctx, color } = surface;
    const end = Math.max(15, this.points.at(-1)?.time_s || 0);
    const start = end - 15;
    const points = this.points.filter(p => p.time_s >= start);
    let bound = minimum;
    for (const p of points) if (p.valid) for (const s of series) bound = Math.max(bound, Math.abs(s.value(p)) * 1.1);
    const { x, y, box } = this.axes(surface, start, end, -bound, bound, "时间 (s)", label);
    ctx.save(); ctx.beginPath(); ctx.rect(box.left, box.top, box.right-box.left, box.bottom-box.top); ctx.clip();
    for (const s of series) {
      ctx.strokeStyle = color(s.color); ctx.lineWidth = 1.8; ctx.beginPath();
      let continuous = false, previousTime = null;
      for (const p of points) {
        const v = s.value(p);
        if (!p.valid || !Number.isFinite(v)) { continuous = false; continue; }
        if (previousTime !== null && p.time_s - previousTime > .3) continuous = false;
        if (continuous) ctx.lineTo(x(p.time_s), y(v)); else ctx.moveTo(x(p.time_s), y(v));
        continuous = true; previousTime = p.time_s;
      }
      ctx.stroke();
    }
    ctx.restore();
    if (!points.some(p => p.valid)) {
      ctx.fillStyle = color("--muted"); ctx.textAlign = "center";
      ctx.fillText("等待有效采样", (box.left+box.right)/2, (box.top+box.bottom)/2);
    }
  }

  /* Show XY acceleration vector and recent trail, never a displacement estimate. */
  xyChart() {
    const surface = this.surface("imu-xy-chart"), { ctx, color, width, height } = surface;
    const samples = this.points.filter(p => p.valid).slice(-200);
    let bound = 1;
    for (const p of samples) bound = Math.max(bound, Math.abs(p[this.mode][0])*1.2, Math.abs(p[this.mode][1])*1.2*(width-86)/(height-73));
    // Equal data scale per pixel preserves direction and magnitude in XY.
    const ybound = bound * (height - 73) / (width - 86);
    const { x, y } = this.axes(surface, -bound, bound, -ybound, ybound, "X 加速度 (m/s²)", "Y 加速度 (m/s²)");
    ctx.strokeStyle = color("--tick"); ctx.beginPath();
    ctx.moveTo(x(-bound), y(0)); ctx.lineTo(x(bound), y(0));
    ctx.moveTo(x(0), y(-ybound)); ctx.lineTo(x(0), y(ybound)); ctx.stroke();
    ctx.fillStyle = color("--muted");
    for (const p of samples) { ctx.beginPath(); ctx.arc(x(p[this.mode][0]), y(p[this.mode][1]), 1.5, 0, Math.PI*2); ctx.fill(); }
    const latest = this.frame.latest;
    if (this.frame.state === "streaming" && latest?.valid) {
      const px = x(latest[this.mode][0]), py = y(latest[this.mode][1]);
      ctx.strokeStyle = color("--teal"); ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(x(0), y(0)); ctx.lineTo(px, py); ctx.stroke();
      ctx.fillStyle = color("--teal"); ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI*2); ctx.fill();
    }
  }

  draw() {
    if (!document.getElementById("tab-imu").classList.contains("active")) return;
    this.timeChart("imu-attitude-chart", [
      { color: "--cyan", value: p => p.roll_deg },
      { color: "--teal", value: p => p.pitch_deg },
      { color: "--violet", value: p => p.yaw_deg }], "角度 (°)", 10);
    this.timeChart("imu-accel-chart", [
      { color: "--cyan", value: p => p[this.mode][0] },
      { color: "--teal", value: p => p[this.mode][1] }], "加速度 (m/s²)", 1);
    this.xyChart();
  }
}
