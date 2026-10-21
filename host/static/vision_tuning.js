/* Detector tuning shares the saved alignment snapshot. Preview uses a fresh
   camera-worker frame and never starts motion. Only one request may be active. */
export class VisionTuningView {
  constructor(owner) {
    this.owner = owner; this.draft = null; this.color = 'red'; this.busy = false;
    this.running = false; this.revision = 0;
    this.node = id => document.getElementById(`tuning-${id}`);
    this.node('color').addEventListener('change', () => {
      try { this.collectColor(); this.color = this.node('color').value; this.renderColor(); this.owner.node('target').value = this.color; }
      catch (e) { this.node('color').value = this.color; this.message(e.message); }
    });
    this.node('save').addEventListener('click', () => this.save());
    this.node('preview').addEventListener('click', () => this.preview());
    this.node('live').addEventListener('click', () => this.live());
    this.node('stop').addEventListener('click', () => this.stop());
    this.node('reload').addEventListener('click', () => this.reload());
    this.node('defaults').addEventListener('click', () => this.defaults());
    document.addEventListener('visibilitychange', () => { if (document.hidden) this.stop(); });
  }

  message(text) { this.node('state').textContent = text; }

  adopt(config) {
    this.color = config.target || this.color;
    if (config.kind) this.node('kind').value = config.kind;
    this.draft = structuredClone(config.detector); this.renderColor(); this.renderRing(); this.controls();
  }

  renderColor() {
    const item = this.draft.colors[this.color];
    this.node('color').value = this.color;
    this.node('area').value = item.min_area;
    this.node('selection').textContent = this.color === 'black' ? '选择面积最大的有效色块' : '选择面积最小的有效色块';
    for (let b = 0; b < 2; b++) for (let c = 0; c < 3; c++)
      this.node(`0-${b}-${c}`).value = item.ranges[0][b][c];
  }

  renderRing() {
    for (const [key, value] of Object.entries(this.draft.ring)) this.node(key).value = value;
    for (const [key, value] of Object.entries(this.draft.ocr)) this.node(key).value = value;
  }

  number(id) {
    const raw = this.node(id).value.trim();
    if (!raw || !Number.isFinite(Number(raw))) throw new Error('请填写有效数值。');
    return Number(raw);
  }

  collectColor() {
    if (!this.draft) throw new Error('视觉阈值尚未加载。');
    const item = this.draft.colors[this.color];
    item.min_area = this.number('area');
    item.ranges = [item.ranges[0].map((bound, b) => bound.map((v, c) => this.number(`0-${b}-${c}`)))];
  }

  read() {
    this.collectColor();
    for (const key of Object.keys(this.draft.ring)) this.draft.ring[key] = this.number(key);
    for (const key of Object.keys(this.draft.ocr)) {
      const value = this.number(key);
      if (value < 0.01 || value > 1) throw new Error('数字置信度须为 0.01～1.00。');
      this.draft.ocr[key] = value;
    }
    return structuredClone(this.draft);
  }

  config() {
    const config = this.owner.read();
    config.detector = this.read();
    config.kind = this.node('kind').value;
    config.target = this.color;
    return config;
  }

  locked() { return this.owner.role === 'readonly' || this.owner.taskActive || (this.owner.status.busy && !this.owner.status.tuning); }

  controls() {
    const locked = this.locked() || !this.draft;
    document.querySelectorAll('#vision-tuning input, #vision-tuning select').forEach(n => { n.disabled = locked; });
    for (const id of ['save','reload','defaults','preview','live']) this.node(id).disabled = locked || this.busy || this.running;
    this.node('stop').disabled = !this.running;
    if (locked && this.running) this.stop();
  }

  async save() {
    if (this.locked() || this.busy) return;
    this.stop(); this.busy = true; this.controls();
    try {
      const result = await this.owner.post('/api/camera/vision/config', {config:this.config()});
      if (!result.ok) throw new Error(result.error || '保存失败。');
      this.owner.adoptConfig(result.config);
      this.message(`已保存：目标 ${result.config.target}、HSV、面积及数字置信度；NX色块预览按此目标检测，未找到不会换找其他颜色。`);
    } catch (e) { this.message(e.message); }
    finally { this.busy = false; this.controls(); }
  }

  async reload() {
    if (this.locked() || this.busy) return;
    this.stop(); this.busy = true; this.controls();
    try {
      const data = await fetch('/api/camera/vision/state', {cache:'no-store'}).then(r => r.json());
      this.owner.adoptConfig(data.config); this.message('已重新读取保存值。');
    } catch (e) { this.message(e.message); }
    finally { this.busy = false; this.controls(); }
  }

  async defaults() {
    if (this.locked() || this.busy) return;
    this.stop(); this.busy = true; this.controls();
    try {
      const data = await fetch('/api/camera/vision/defaults').then(r => r.json());
      this.adopt(data); this.message('已载入这批新版的原始参数，点击保存后生效。');
    } catch (e) { this.message(e.message); }
    finally { this.busy = false; this.controls(); }
  }

  stop() { this.running = false; this.revision++; this.controls(); }

  async live() {
    if (this.locked() || this.busy || this.running) return;
    this.running = true; this.controls();
    const revision = ++this.revision;
    while (this.running && revision === this.revision) {
      if (!await this.preview()) { this.stop(); break; }
      await new Promise(resolve => setTimeout(resolve, 500));
    }
  }

  async preview() {
    if (this.locked() || this.busy) return false;
    this.busy = true; this.controls();
    const revision = this.revision;
    try {
      if (!this.owner.camera.nano) throw new Error('请选择 Nano 来源并打开 NX 摄像头一。');
      const result = await this.owner.post('/api/camera/vision/tune', {config:this.config()});
      if (!result.ok) throw new Error(result.error || '预览失败。');
      if (revision !== this.revision) return false;
      if (result.status) this.owner.update(result.status);
      this.node('image').src = result.image; this.node('mask').src = result.mask;
      this.node('images').hidden = false;
      const v = result.report;
      this.message(`${v.width}×${v.height} · ${v.found ? `中心 (${v.center.join(', ')})，偏差 X ${v.error_dx} / Y ${v.error_dy} px` : '未找到目标'}${v.rings ? ` · ${v.rings.length} 个圆环` : ''} · 当前草稿预览，不移动；保存后用于任务`);
      return true;
    } catch (e) { this.message(e.message); return false; }
    finally { this.busy = false; this.controls(); }
  }
}
