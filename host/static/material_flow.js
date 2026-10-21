/* C/D distances for the material mission only; edits never start movement. */
export class MaterialFlowView {
  constructor(post, changed) {
    this.post = post; this.changed = changed;
    this.dirty = false; this.saving = false; this.env = {};
    this.inputs = Object.fromEntries(['point2_steps', 'point3_steps'].map(key => [key, this.node(key)]));
    for (const input of Object.values(this.inputs)) input.addEventListener('input', () => {
      this.dirty = true; this.preview();
      this.message('有未保存修改；保存后，下次启动全流程生效。');
    });
    this.node('save').addEventListener('click', () => this.save());
    this.node('reload').addEventListener('click', () => this.reload());
  }

  node(key) { return document.getElementById(`material-distance-${key}`); }
  message(text) { this.node('state').textContent = text; }

  /* Derived distance is a preview of the draft, never a third editable value. */
  preview() {
    const a = Number(this.inputs.point2_steps.value), b = Number(this.inputs.point3_steps.value);
    this.node('delta').textContent = Number.isInteger(a) && Number.isInteger(b) && a > 0 && b > a
      ? `点2 → 点3：${b-a} 脉冲${this.dirty ? '（未保存）' : ''}` : '点1→点3必须大于点1→点2，请填写整数脉冲。';
  }

  /* Live snapshots update saved values without overwriting an operator draft. */
  update(config, env = this.env, force = false) {
    this.env = env;
    if (config && (!this.dirty || force) && (force || JSON.stringify(config) !== this.saved)) {
      for (const [key, input] of Object.entries(this.inputs)) input.value = config[key];
      this.saved = JSON.stringify(config); this.loaded = true; this.dirty = false;
      this.preview(); this.message('已加载保存值；C、D两站及两轮搬运共用。');
    }
    const locked = env.role === 'readonly' || !!env.task?.active || !!env.competition?.active ||
      !!env.pickPlace?.active || !!env.navigationRunner?.active || !!env.cameraAlignment?.active || this.saving;
    for (const input of Object.values(this.inputs)) input.disabled = locked || !this.loaded;
    this.node('save').disabled = locked || !this.loaded;
    this.node('reload').disabled = locked;
  }

  /* Persist a complete pair through the guarded API; failures retain the draft. */
  async save() {
    this.saving = true; this.update(null);
    try {
      const config = {};
      for (const [key, input] of Object.entries(this.inputs)) {
        const value = Number(input.value);
        if (!input.value.trim() || !Number.isInteger(value) || value < 1 || value > 100000)
          throw new Error('点位距离必须是1～100000的整数脉冲。');
        config[key] = value;
      }
      if (config.point3_steps <= config.point2_steps) throw new Error('点1→点3必须大于点1→点2的距离。');
      const result = await this.post('/api/material-flow/config', {config});
      if (!result.ok) throw new Error(result.error || '保存失败。');
      this.update(result.config, this.env, true); this.changed(result.config);
      this.message('C/D点位距离已保存，下次启动全流程生效；重启后保留。');
      return true;
    } catch (error) { this.message(error.message); return false; }
    finally { this.saving = false; this.update(null); }
  }

  async reload() {
    try {
      const response = await fetch('/api/material-flow/config');
      if (!response.ok) throw new Error('读取C/D点位距离失败。');
      const result = await response.json();
      this.update(result.config, this.env, true); this.changed(result.config);
    } catch (error) { this.message(error.message); }
  }
}
