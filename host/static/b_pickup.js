/* B-station settings only. Saving never starts a mechanism or camera action. */
import { FIELDS } from '/pick_place.js';

export class BPickupView {
  constructor(post, changed) {
    this.post = post; this.changed = changed;
    this.inputs = {}; this.dirty = false; this.saving = false; this.env = {};
    const root = document.getElementById('b-pickup-fields');
    const roiFields = ['B点识别框（画面百分比）', [
      ['roi_x_pct', '框左边 X / %', 0, 99], ['roi_y_pct', '框上边 Y / %', 0, 99],
      ['roi_width_pct', '框宽 / %', 1, 100], ['roi_height_pct', '框高 / %', 1, 100],
    ]];
    const delayFields = ['识别后抓取等待', [
      ['recognition_delay_ms', '识别目标色后延迟 / ms（2000 = 2秒）', 0, 60000],
    ]];
    for (const [heading, fields] of [roiFields, delayFields, ...FIELDS]) {
      const selected = fields.filter(([key]) => !key.startsWith('place_') && !key.startsWith('s2_slot_') && key !== 'vision_timeout_ms' && key !== 'd_second_drop_down');
      if (!selected.length) continue;
      const title = document.createElement('h3'); title.textContent = heading;
      const group = document.createElement('div'); group.className = 'aux-fields';
      for (const [key, text, min, max] of selected) {
        const label = document.createElement('label');
        label.textContent = key === 'pick_rotation' ? '抓住后转向盘子 / 脉冲' : text;
        const input = document.createElement('input');
        input.id = `b-pickup-${key}`; input.type = 'number'; input.step = '1';
        input.min = min; input.max = max; label.htmlFor = input.id;
        input.addEventListener('input', () => { this.dirty = true; this.message('有未保存修改，请先保存B点取料参数再启动全流程。'); });
        this.inputs[key] = input; label.append(input); group.append(label);
      }
      root.append(title, group);
      if (fields === roiFields[1]) {
        const help = document.createElement('p'); help.className = 'note';
        help.textContent = '宽、高越大，识别范围越大。X、Y 是框左上角的位置，均按整个画面的百分比填写。快捷按钮仅填写参数，点击保存后生效。';
        const presets = document.createElement('div'); presets.className = 'row wrap';
        for (const size of [50, 70, 100]) {
          const button = document.createElement('button');
          button.type = 'button'; button.dataset.bRoiSize = size;
          button.textContent = size === 100 ? '全画面' : `居中 ${size}%`;
          // Presets edit the form only; saving uses the existing ownership guard.
          button.addEventListener('click', () => {
            const offset = (100-size)/2;
            for (const [key, value] of Object.entries({roi_x_pct:offset, roi_y_pct:offset, roi_width_pct:size, roi_height_pct:size})) this.inputs[key].value = value;
            this.dirty = true;
            this.message('识别框已填写，请点击保存识别框与B点参数。');
          });
          presets.append(button);
        }
        const save = document.createElement('button'); save.type = 'button';
        save.id = 'b-pickup-roi-save'; save.textContent = '保存识别框与B点参数';
        save.addEventListener('click', () => this.save()); presets.append(save);
        root.append(help, presets);
      }
    }
    this.node('save').addEventListener('click', () => this.save());
    this.node('reload').addEventListener('click', () => this.reload());
    document.getElementById('b-pickup-open-roi').addEventListener('click', () => {
      document.querySelector('[data-tab="auxiliary"]').click();
      this.node('card').scrollIntoView({block:'start'});
      this.inputs.roi_width_pct.focus({preventScroll:true});
    });
  }

  node(key) { return document.getElementById(`b-pickup-${key}`); }
  message(text) { this.node('state').textContent = text; }

  /* SSE snapshots may refresh saved data but must not erase an operator draft. */
  update(config, env = this.env, force = false) {
    this.env = env;
    if (config && (!this.dirty || force) && (force || JSON.stringify(config) !== this.saved)) {
      for (const [key, input] of Object.entries(this.inputs)) input.value = config[key];
      this.dirty = false; this.loaded = true;
      this.saved = JSON.stringify(config);
      this.message('B点专用参数已加载；修改后点击保存。');
    }
    const locked = env.role === 'readonly' || !!env.task?.active || !!env.competition?.active || !!env.pickPlace?.active || this.saving;
    for (const input of Object.values(this.inputs)) input.disabled = locked;
    this.node('save').disabled = locked || !this.loaded;
    this.node('reload').disabled = locked;
    this.node('roi-save').disabled = locked || !this.loaded;
    document.querySelectorAll('[data-b-roi-size]').forEach(button => { button.disabled = locked || !this.loaded; });
  }

  /* Save a complete B-only snapshot; the server validates ranges and ownership. */
  async save() {
    this.saving = true; this.update(null);
    try {
      const config = {};
      for (const [key, input] of Object.entries(this.inputs)) {
        const value = Number(input.value);
        if (!input.value.trim() || !Number.isInteger(value) || value < Number(input.min) || value > Number(input.max)) throw new Error('请填写有效的完整整数参数。');
        config[key] = value;
      }
      if (config.roi_x_pct + config.roi_width_pct > 100 || config.roi_y_pct + config.roi_height_pct > 100) throw new Error('识别框不能超出画面范围。');
      const result = await this.post('/api/b-pickup/config', {config});
      if (!result.ok) throw new Error(result.error || '保存失败。');
      this.update(result.config, this.env, true); this.changed(result.config);
      this.message('B点取料参数已保存，下次全流程在B点使用。');
      return true;
    } catch (error) { this.message(error.message); return false; }
    finally { this.saving = false; this.update(null); }
  }

  async reload() {
    try {
      const response = await fetch('/api/b-pickup/config');
      if (!response.ok) throw new Error('读取B点参数失败。');
      const result = await response.json();
      this.update(result.config, this.env, true); this.changed(result.config);
    } catch (error) { this.message(error.message); }
  }
}
