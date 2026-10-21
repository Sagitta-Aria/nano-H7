/* One-item mechanism controls; the server owns sequencing and vision gating. */
export const FIELDS = [
  ['抓取位置', [['pick_down','抓取下降 / 脉冲',0,1000000], ['pick_rotation','抓取旋转 / 脉冲',-1000000,1000000]]],
  ['放置位置', [['place_take_down','车上取料下降 / 脉冲',0,1000000], ['place_rotation','车上取料旋转 / 脉冲',-1000000,1000000], ['place_drop_down','地面放料下降 / 脉冲',0,1000000], ['d_second_drop_down','第二次D点放料下降 / 绝对脉冲（-1未标定）',-1,1000000]]],
  ['速度与加速度', [['lift_sps','升降速度 / step/s',1,3000], ['lift_accel','升降加速度 / step/s²',1,20000], ['rotation_sps','旋转速度 / step/s',1,3000], ['rotation_accel','旋转加速度 / step/s²',1,20000]]],
  ['舵机角度', [['s1_open','S1 张开 / °',0,180], ['s1_closed','S1 闭合 / °',0,180], ['s2_slot_1','S2 抓取第1次槽位 / °',0,270], ['s2_slot_2','S2 抓取第2次槽位 / °',0,270], ['s2_slot_3','S2 抓取第3次槽位 / °',0,270]]],
  ['等待与超时', [['s1_settle_ms','S1 到位等待 / ms',100,30000], ['s2_settle_ms','S2 到位等待 / ms',100,30000], ['step_timeout_ms','单步超时 / ms',1000,600000], ['ack_timeout_ms','控制板应答超时 / ms',1000,600000], ['vision_timeout_ms','视觉确认超时 / ms',1000,600000]]],
];

export class PickPlaceView {
  /* Create controls once; polling must preserve unsaved operator edits. */
  constructor(post, changed, prepareBPickup = async () => true) {
    this.post = post;
    this.changed = changed;
    this.prepareBPickup = prepareBPickup;
    this.config = null;
    this.status = {};
    this.env = {};
    this.dirty = false;
    this.submitting = false;
    this.actionError = '';
    this.inputs = {};
    const root = document.getElementById('pick-place-fields');
    for (const [heading, fields] of FIELDS) {
      const title = document.createElement('h3'); title.textContent = heading;
      const group = document.createElement('div'); group.className = 'aux-fields';
      for (const [key, text, min, max] of fields) {
        const label = document.createElement('label'); label.textContent = text;
        const input = document.createElement('input');
        input.id = `pick-place-${key}`; input.type = 'number'; input.step = '1';
        input.min = min; input.max = max; label.htmlFor = input.id;
        input.addEventListener('input', () => {
          this.dirty = true;
          this.node('config-state').textContent = '参数已修改；点击抓取或放置会自动保存并运行。';
          this.actionError = '';
          this.render();
        });
        this.inputs[key] = input; label.append(input); group.append(label);
      }
      root.append(title, group);
    }
    for (const action of ['pick', 'b-pick', 'place', 'stop']) {
      this.node(action).addEventListener('click', () => this.run(action));
    }
    this.node('save').addEventListener('click', () => this.save());
    this.node('reload').addEventListener('click', async () => {
      try {
        const response = await fetch('/api/pick-place/state');
        const result = await response.json();
        this.update(result.status, result.config, true); this.changed(result.status, result.config);
      } catch (error) { this.node('config-state').textContent = `读取失败：${error.message}`; }
    });
  }

  node(suffix) { return document.getElementById(`pick-place-${suffix}`); }

  /* Apply snapshots without overwriting dirty inputs during SSE refreshes. */
  update(status, config, force = false) {
    this.status = status || this.status;
    if (config && (!this.config || force || !this.dirty)) {
      this.config = {...config};
      for (const [key, input] of Object.entries(this.inputs)) input.value = config[key];
      if (force) this.dirty = false;
      this.node('config-state').textContent = '参数已加载；启动时自动保存修改、应用已保存范围并使能。';
    }
    this.render();
  }

  /* Display current ownership and stage; no browser timer drives motion. */
  render(env = this.env) {
    this.env = env;
    const s = this.status;
    const readonly = env.role === 'readonly';
    const busy = !!s.active;
    const locked = readonly || busy || this.submitting;
    const names = {idle:'未启动',camera:'等待摄像头1启动',vision:'等待外部视觉确认',pickup_wait:'B点等待目标颜色',pickup_delay:'B点识别成功，延迟后抓取',preflight:'自动准备',arming:'自动使能',running:'运行中',done:'本次完成',stopped:'已停止',fault:'动作故障'};
    this.node('state').textContent = names[s.state] || '未启动';
    const item = s.context?.item;
    const profile = s.context?.profile === 'b_pickup' ? 'B点' : s.context?.profile === 'd_second' ? '第二次D点叠放' : '';
    const detail = `${profile ? profile + ' · ' : ''}${s.context?.mode === 'place' ? '放置' : '抓取'} · 第${s.context?.batch}批第${item}次${s.context?.color_name ? ` · 目标${s.context.color_name}${s.context.color ? ` (${s.context.color})` : ''}` : ''} · ${s.step}/${s.total} · ${s.label || ''}`;
    const unavailable = readonly ? '当前页面为只读，请点击顶部“接管控制”。' : env.link !== 'connected' ? '控制板未连接。' : '';
    this.node('detail').textContent = this.actionError || unavailable || s.error || (item ? detail : '点击一次即可测试；自动应用已保存范围并使能，无需另点准备按钮。');
    this.node('request').textContent = s.request ? `${s.state === 'pickup_wait' ? 'B点等待框内目标颜色（连续3帧，无等待超时）' : '等待外部视觉确认'}：第${s.request.batch}批第${s.request.item}次 · ${s.request.color_name || '手动目标'}${s.request.mode === 'place' ? ` · 放置标记 ${s.request.target || '未指定'}` : ''}` : 'B点完整抓取自动打开摄像头1，使用B点参数下降，框内连续3帧确认目标色后，按B点参数延迟再合爪（默认2秒）；启动、断帧和等色均无等待超时。目标取所选二维码批次/序号，无任务码时取已保存的识别目标。普通一键抓放按通用机构参数执行。';
    if (this.config) {
      const c = this.config;
      this.node('order').textContent = `抓取：${c.s2_slot_1}° → ${c.s2_slot_2}° → ${c.s2_slot_3}°；第二次D点下降须单独标定，数值小于地面下降；放置从车上取出：${c.s2_slot_3}° → ${c.s2_slot_2}° → ${c.s2_slot_1}°。`;
    }
    for (const input of Object.values(this.inputs)) input.disabled = locked;
    for (const key of ['batch','item','code','save','reload']) this.node(key).disabled = locked;
    const blocked = locked || !this.config || env.link !== 'connected';
    this.node('pick').disabled = blocked; this.node('b-pick').disabled = blocked; this.node('place').disabled = blocked;
    this.node('stop').disabled = readonly || !busy;
  }

  /* Save all parameters atomically; invalid drafts stay visible for correction. */
  async save() {
    try {
      const config = {};
      for (const [key, input] of Object.entries(this.inputs)) {
        if (!input.value.trim() || !input.checkValidity() || !Number.isInteger(Number(input.value))) throw new Error('请填写有效的完整整数参数。');
        config[key] = Number(input.value);
      }
      const result = await this.post('/api/pick-place/config', {config});
      if (!result.ok) throw new Error(result.error || '参数保存失败。');
      this.update(null, result.config, true); this.changed(this.status, result.config);
      return true;
    } catch (error) {
      this.node('config-state').textContent = error.message;
      return false;
    }
  }

  /* Start precisely one selected item; an explicit click is required each time. */
  async run(action) {
    if (action !== 'stop' && this.submitting) return;
    this.submitting = true; this.actionError = ''; this.render();
    try {
      // B has its own saved mechanism profile. A draft in the generic form
      // must neither overwrite generic settings nor block a B-only test.
      if (action === 'b-pick') {
        if (!await this.prepareBPickup()) return;
      } else if (action !== 'stop' && this.dirty && !await this.save()) return;
      const result = await this.post('/api/pick-place/action', {
        action: action === 'b-pick' ? 'pick' : action,
        profile: action === 'b-pick' ? 'b_pickup' : 'default',
        batch:Number(this.node('batch').value), item:Number(this.node('item').value),
        task_code:this.node('code').value.trim() || null, require_vision:false,
      });
      if (!result.ok) this.actionError = result.error || result.detail || '未能启动动作。';
      if (result.status) { this.update(result.status); this.changed(result.status, this.config); }
    } catch (error) { this.actionError = `请求失败：${error.message}`; }
    finally { this.submitting = false; this.render(); }
  }
}
