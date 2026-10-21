/* Manual single-frame capture. Nano uses server-side V4L2; local mode uses
   browser media. There are no idle timers, video streams or repeated photos. */
export class CameraView {
  constructor(post, prefix = "camera", api = "/api/camera") {
    this.post = post;
    this.api = api;
    this.qrNodes = prefix === 'camera2' ? {
      code: document.getElementById('camera2-qr-code'),
      status: document.getElementById('camera2-qr-status'),
      seen: document.getElementById('camera2-qr-seen'),
      plan: document.getElementById('camera2-task-plan'),
      validation: document.getElementById('camera2-task-validation'),
    } : null;
    this.nodes = {};
    for (const name of ["source", "device", "refresh", "start", "stop", "resolution", "apply",
                        "snapshot", "mirror", "actual", "state", "message", "video", "image", "placeholder", "bind", "identity"]) {
      this.nodes[name] = document.getElementById(`${prefix}-${name}`);
    }
    this.media = navigator.mediaDevices;
    this.stream = null;
    this.busy = false;
    this.request = 0;
    this.role = "controller";
    this.frame = {state: "idle"};
    this.devices = [];
    this.nanoSupported = false;
    this.refreshing = false;
    this.localResolutions = [...this.nodes.resolution.options].map(o => o.value);
    this.nodes.start.addEventListener("click", () => this.start());
    this.nodes.stop.addEventListener("click", () => this.stop());
    this.nodes.refresh.addEventListener("click", () => this.refreshDevices());
    this.nodes.bind.addEventListener("click", () => this.bind());
    this.nodes.apply.addEventListener("click", () => this.applyResolution());
    this.nodes.snapshot.addEventListener("click", () => this.capture());
    this.nodes.device.addEventListener("change", () => this.resolutions());
    this.nodes.source.addEventListener("change", () => {
      this.releaseLocal();
      ++this.request;
      this.busy = false;
      this.frame = {state: "idle"};
      this.nodes.device.replaceChildren(new Option("请选择摄像头", ""));
      this.message("请选择设备并打开摄像头。", "idle");
      this.refreshDevices();
      this.controls();
    });
    this.nodes.mirror.addEventListener("change", () => {
      this.nodes.image.classList.toggle("mirrored", this.nodes.mirror.checked);
    });
    const leave = () => { ++this.request; this.releaseLocal(); };
    window.addEventListener("pagehide", leave);
    window.addEventListener("beforeunload", leave);
    this.media?.addEventListener("devicechange", () => { if (!this.nano) this.refreshDevices(); });
    this.initialisation = this.initialise();
  }

  get nano() { return this.nodes.source.value === "nano"; }

  /* Linux defaults to the camera attached to the server. Inspecting the page
     never opens the device, requests camera permission or fetches a photo. */
  async initialise() {
    try {
      const result = await fetch(`${this.api}/devices`).then(r => r.json());
      this.nanoSupported = result.supported === true;
      if (this.nanoSupported) this.nodes.source.value = "nano";
    } catch { /* Older local servers still support browser-local capture. */ }
    this.controls();
    await this.refreshDevices();
  }

  setRole(role) { this.role = role; this.controls(); }

  /* Only metadata is fetched on a tab visit or Refresh; no background polling. */
  async refreshDevices() {
    if (this.refreshing) return;
    this.refreshing = true;
    const nano = this.nano;
    const chosen = this.nodes.device.value;
    try {
      if (nano) {
        const result = await fetch(`${this.api}/devices`).then(r => r.json());
        const frame = await fetch(`${this.api}/state`).then(r => r.json());
        if (this.nano !== nano) return;
        this.devices = result.devices || [];
        this.binding = result.binding;
        this.boundDevice = result.bound_device;
        this.nodes.identity.textContent = this.binding ? `已记住：${this.binding.label} · ${this.binding.id}${this.boundDevice ? "" : " · 未连接"}` : "尚未绑定 Nano 摄像头；选择设备后点击“记住此摄像头”。";
        this.frame = frame;
        this.nodes.device.replaceChildren();
        this.devices.forEach(d => this.nodes.device.add(new Option(`${d.label} · ${d.id}`, d.id)));
        if (!this.devices.length) this.nodes.device.add(new Option("未发现 Nano 摄像头（USB / CSI）", ""));
        const selected = this.devices.some(d => d.id === chosen) ? chosen : this.boundDevice || frame.device;
        if (this.devices.some(d => d.id === selected)) this.nodes.device.value = selected;
        this.resolutions();
        this.showFrame(frame);
        if (!this.devices.length) this.message("Nano 未发现摄像头，请接入 USB 摄像头后刷新设备。", "idle");
      } else {
        if (!window.isSecureContext || !this.media?.getUserMedia) {
          this.message("本机摄像头需要 localhost / 127.0.0.1 或 HTTPS；查看 Nano 摄像头可使用当前地址。", "error");
          return;
        }
        const devices = (await this.media.enumerateDevices()).filter(d => d.kind === "videoinput");
        if (this.nano !== nano) return;
        this.nodes.device.replaceChildren(new Option("默认摄像头", ""));
        this.nodes.identity.textContent = "当前使用本机摄像头；Nano 的设备绑定保留。";
        devices.forEach((d, i) => {
          if (d.deviceId) this.nodes.device.add(new Option(d.label || `摄像头 ${i + 1}`, d.deviceId));
        });
        if (devices.some(d => d.deviceId === chosen)) this.nodes.device.value = chosen;
        this.resolutions();
      }
    } catch (error) { this.message(`设备读取失败：${error.message}`, "error"); }
    finally { this.refreshing = false; this.controls(); }
  }

  /* Remember metadata only; opening and taking a photo remain explicit. */
  async bind() {
    if (!this.nano || this.busy || this.role === "readonly") return;
    this.busy = true;
    this.controls();
    try {
      const result = await this.post(`${this.api}/bind`, {device: this.nodes.device.value});
      if (!result.ok) throw new Error(result.error || "绑定失败。");
      await this.refreshDevices();
      this.message("已记住此摄像头，重启后按 设备标识识别。", "idle");
    } catch (error) { this.message(this.errorText(error), "error"); }
    finally { this.busy = false; this.controls(); }
  }

  resolutions() {
    const selected = this.nodes.resolution.value;
    const modes = this.nano ? this.devices.find(d => d.id === this.nodes.device.value)?.resolutions || [] : this.localResolutions;
    this.nodes.resolution.replaceChildren();
    modes.forEach(mode => this.nodes.resolution.add(new Option(mode.replace("x", " × "), mode)));
    if (modes.includes(selected)) this.nodes.resolution.value = selected;
    else if (modes.includes("1280x720")) this.nodes.resolution.value = "1280x720";
    this.controls();
  }

  dimensions() {
    const [width, height] = this.nodes.resolution.value.split("x").map(Number);
    const value = {width: {exact: width}, height: {exact: height}};
    if (this.media?.getSupportedConstraints?.().resizeMode) value.resizeMode = {exact: "none"};
    return value;
  }

  /* Startup polls metadata only for this action, and ends within ten seconds. */
  async waitReady(revision) {
    for (let i = 0; i < 50 && revision === this.request; i++) {
      const frame = await fetch(`${this.api}/state`).then(r => r.json());
      if (revision !== this.request) return;
      this.showFrame(frame);
      if (frame.state !== "opening") return;
      await new Promise(resolve => setTimeout(resolve, 200));
    }
    if (revision === this.request) this.message("打开设备超时，请关闭后重新打开。", "error");
  }

  /* Late browser permission results are released after Stop or source change. */
  async start() {
    if (this.busy || (this.nano && this.role === "readonly")) return;
    const revision = ++this.request;
    this.busy = true;
    this.controls();
    this.message("正在打开摄像头…", "opening");
    let acquired = null;
    try {
      if (this.nano) {
        this.pendingStart = this.post(`${this.api}/start`, {device: this.nodes.device.value, resolution: this.nodes.resolution.value});
        const result = await this.pendingStart;
        if (revision !== this.request) return;
        if (!result.ok) throw new Error(result.error || "摄像头打开失败。");
        this.showFrame(result.camera);
        await this.waitReady(revision);
      } else {
        this.releaseLocal();
        const device = this.nodes.device.value;
        const selection = device ? {deviceId: {exact: device}} : {};
        let fallback = false;
        try {
          acquired = await this.media.getUserMedia({audio: false, video: {...selection, ...this.dimensions()}});
        } catch (error) {
          if (revision !== this.request) return;
          if (error.name !== "OverconstrainedError" || error.constraint === "deviceId") throw error;
          acquired = await this.media.getUserMedia({audio: false, video: selection});
          fallback = true;
        }
        if (revision !== this.request) { acquired.getTracks().forEach(t => t.stop()); return; }
        this.stream = acquired;
        acquired.getVideoTracks()[0].addEventListener("ended", () => {
          if (this.stream !== acquired) return;
          this.releaseLocal();
          this.message("摄像头已断开或权限被撤销，请重新打开。", "error");
          this.controls();
        });
        this.nodes.video.srcObject = acquired;
        await this.nodes.video.play();
        if (revision !== this.request) return;
        this.showLocal();
        this.message(fallback ? "所选分辨率不受支持，已用默认分辨率打开。点击“拍照”获取一帧。" : "摄像头已打开，点击“拍照”获取一帧。", "ready");
        await this.refreshDevices();
      }
    } catch (error) {
      if (revision !== this.request) return;
      if (!this.nano) { acquired?.getTracks().forEach(t => t.stop()); this.releaseLocal(); }
      this.message(this.errorText(error), "error");
    } finally {
      if (revision === this.request) { this.busy = false; this.controls(); }
    }
  }

  async applyResolution() {
    if (this.nano) { await this.start(); return; }
    const track = this.stream?.getVideoTracks()[0];
    if (!track || this.busy) return;
    const revision = this.request;
    this.busy = true;
    this.controls();
    try {
      await track.applyConstraints(this.dimensions());
      if (revision !== this.request) return;
      this.showLocal();
      this.message("分辨率已应用；现有照片不变，再拍照获取新分辨率的一帧。", "ready");
    } catch (error) {
      if (revision === this.request) this.message(this.errorText(error), "ready");
    } finally {
      if (revision === this.request) { this.busy = false; this.controls(); }
    }
  }

  /* One click makes one image request. The displayed image stays frozen until
     the next shutter click, regardless of camera or board state changes. */
  async capture() {
    if (this.busy || (this.nano && this.role === "readonly")) return;
    const revision = this.request;
    this.busy = true;
    this.controls();
    this.message("正在拍照…", "ready");
    try {
      if (this.nano) {
        const result = await this.post(`${this.api}/snapshot`, {});
        if (revision !== this.request) return;
        if (!result.ok) throw new Error(result.error || "拍照失败。");
        this.showFrame(result.camera);
        this.nodes.image.src = `${this.api}/frame?sequence=${result.camera.sequence}&t=${Date.now()}`;
      } else {
        if (!this.stream || !this.nodes.video.videoWidth) throw new Error("摄像头尚未出图，请稍后拍照。");
        const canvas = document.createElement("canvas");
        canvas.width = this.nodes.video.videoWidth;
        canvas.height = this.nodes.video.videoHeight;
        canvas.getContext("2d").drawImage(this.nodes.video, 0, 0);
        this.nodes.image.src = canvas.toDataURL("image/jpeg", 0.85);
        this.showLocal();
      }
      this.nodes.image.classList.remove("hidden");
      this.nodes.placeholder.classList.add("hidden");
      this.message(`拍照完成 · ${new Date().toLocaleTimeString()} · 照片保持不变，点击“拍照”获取下一帧。`, "ready");
    } catch (error) {
      if (revision === this.request) this.message(this.errorText(error), "error");
    } finally {
      if (revision === this.request) { this.busy = false; this.controls(); }
    }
  }

  releaseLocal() {
    const stream = this.stream;
    this.stream = null;
    stream?.getTracks().forEach(t => t.stop());
    this.nodes.video.srcObject = null;
  }

  async stop() {
    if (this.nano && this.role === "readonly") return;
    const revision = ++this.request;
    this.busy = true;
    this.controls();
    if (this.nano) {
      try {
        // Finish the already submitted open action before closing: otherwise
        // its slower device enumeration could reopen after a fast Stop.
        await this.pendingStart?.catch(() => {});
        const result = await this.post(`${this.api}/stop`, {});
        if (revision !== this.request) return;
        if (!result.ok) throw new Error(result.error || "关闭失败。");
        this.showFrame(result.camera);
      } catch (error) { this.message(this.errorText(error), "error"); }
    } else {
      this.releaseLocal();
      this.frame = {state: "idle"};
      this.message("摄像头已关闭，设备已释放；最后一张照片保留供查看。", "idle");
    }
    if (revision === this.request) { this.busy = false; this.controls(); }
  }

  showLocal() {
    const settings = this.stream?.getVideoTracks()[0]?.getSettings();
    if (settings) this.nodes.actual.textContent = `实际采集：${settings.width} × ${settings.height}`;
  }

  showFrame(frame) {
    this.frame = frame;
    this.message(this.qrNodes && this.nano && frame.state === 'ready'
      ? 'NX 摄像头2正在持续识别二维码，任务码自动更新；拍照仅用于查看画面。'
      : frame.message || "Nano 摄像头未打开。", frame.state);
    this.nodes.actual.textContent = frame.width ? `实际采集：${frame.width} × ${frame.height}` : "实际采集：—";
    this.controls();
    this.showQR(frame);
  }

  /* Server events carry only QR metadata; photos and motor control stay independent. */
  updateRemote(frame) {
    if (this.nano) this.showFrame(frame);
  }

  /* A locked run plan outlives camera shutdown; previews never overwrite it. */
  setTask(task) {
    this.task = task;
    this.controls();
  }

  showQR(frame = this.frame) {
    if (!this.qrNodes) return;
    const qr = this.nano ? frame.qr : null;
    this.qrNodes.code.textContent = qr?.last_code || '—';
    const labels = {valid: '当前已识别有效任务码', searching: '正在寻找二维码',
      located: '已找到二维码，正在解码', invalid: '二维码内容不是有效任务码',
      decode_error: '本帧解码失败，正在重试', stale: '识别数据已过期'};
    this.qrNodes.status.textContent = !this.nano ? '任务码识别使用 NX 摄像头2，请选择 NX 来源。'
      : frame.state !== 'ready' ? (frame.state === 'error' ? '摄像头离线或识别停止' : '等待摄像头2打开')
      : labels[qr?.status] || '正在启动二维码识别';
    this.qrNodes.seen.textContent = qr?.last_seen_at
      ? `上次有效识别：${new Date(qr.last_seen_at * 1000).toLocaleTimeString()}${qr.status === 'valid' && frame.state === 'ready' ? '' : ' · 显示保留结果'}`
      : '尚未识别到有效任务码';
    const locked = this.task?.task_plan;
    const plan = locked || qr?.task_plan || qr?.last_plan;
    this.qrNodes.validation.textContent = locked ? `本次执行已锁定：${locked.code} · ${this.task.qr_status || ''}`
      : qr?.task_error || (plan ? `扫码预览（尚未锁定）：${plan.code}` : '等待四组比赛任务码：颜色1～6，放置位置为1、2、3的排列。');
    this.qrNodes.plan.textContent = plan ? plan.batches.map(batch =>
      `第${batch.batch}批：` + batch.items.map(item => `${item.angle_deg}° ${item.color_name} → 位置${item.target}`).join('；')
    ).join('\n') : '';
    if (locked) {
      this.qrNodes.code.textContent = locked.code;
      this.qrNodes.status.textContent = '本次运行任务码已锁定';
      this.qrNodes.seen.textContent = this.task.qr_status || '任务码保留至下次运行';
    }
  }

  controls() {
    const readonly = this.nano && (this.role === "readonly" || (this.task?.active && this.task?.uses_task_code));
    const available = this.nano ? this.nanoSupported && !!this.nodes.device.value : window.isSecureContext && !!this.media?.getUserMedia;
    const ready = this.nano ? this.frame.state === "ready" : !!this.stream;
    const wrongBinding = this.nano && this.binding && this.nodes.device.value !== this.boundDevice;
    this.nodes.start.disabled = this.busy || readonly || !available || !!wrongBinding;
    this.nodes.bind.disabled = !this.nano || this.busy || readonly || !available || ready || this.frame.state === "opening";
    this.nodes.stop.disabled = readonly || (!this.busy && !ready && this.frame.state !== "opening" && this.frame.state !== "error");
    this.nodes.snapshot.disabled = this.busy || readonly || !ready;
    this.nodes.apply.disabled = this.busy || readonly || !ready;
    this.nodes.source.disabled = this.busy;
    this.nodes.device.disabled = this.busy || readonly;
    this.nodes.resolution.disabled = this.busy || readonly;
    this.showQR();
  }

  message(text, state = "idle") {
    this.nodes.message.textContent = text;
    this.nodes.state.dataset.state = state;
    this.nodes.state.textContent = ({idle: "未打开", opening: "正在打开", ready: "已打开 · 按需拍照", error: "操作提示"})[state] || "未打开";
    if (this.qrNodes && this.nano && state === 'ready') this.nodes.state.textContent = '已打开 · 二维码识别';
  }

  errorText(error) {
    return ({NotAllowedError: "摄像头权限未获允许，请检查浏览器和 Windows 摄像头隐私设置。",
      NotFoundError: "摄像头未找到，请检查 摄像头连接并刷新设备。",
      NotReadableError: "摄像头无法读取，可能被其他程序占用。",
      OverconstrainedError: "此设备不支持所选分辨率，当前采集设置保持不变。",
    })[error.name] || error.message || "摄像头操作失败。";
  }
}
