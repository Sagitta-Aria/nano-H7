/* The server owns startup and travel; a reload only observes the same run. */
const token = 'race-' + (globalThis.crypto?.randomUUID?.() || Math.random().toString(36).slice(2));
const node = id => document.getElementById(id);
let readonly = false, submitting = false, lastFrame = null;

function render(frame) {
  lastFrame = frame;
  const race = frame.competition || {}, nav = frame.navigation || {};
  const running = race.active || frame.navigation_runner?.active || frame.task?.active;
  node('start').disabled = readonly || submitting || !!running;
  node('start').textContent = running ? '执行中' : '启动';
  document.querySelector('h1').textContent = running ? '正在执行比赛路线' : race.state === 'done' ? '路线已完成' : '准备出发';
  const labels = {idle:'等待启动', stopped:'已停止', connecting:'正在连接设备并等待航向', preparing:'正在恢复参数并建立起点', running:'路线执行中', done:'路线已完成', fault:'启动或执行失败'};
  node('status').textContent = readonly ? '另一个页面正在控制车辆' : race.error || frame.settings_error || labels[race.state] || '等待启动';
  node('board').textContent = frame.link === 'connected' ? '已连接 · ' + (frame.device?.fw || '') : '等待连接';
  node('heading').textContent = nav.imu === '1' ? (Number(nav.heading_cd)/100).toFixed(2) + '°' : '等待数据';
  node('position').textContent = nav.x != null ? `${nav.x}, ${nav.y}` : '—';
  if (frame.mission) {
    const start = frame.mission.stations?.[frame.mission.start_station ?? 0];
    node('route').textContent = `${start?.name || 'O'} → A扫码 → B抓取 → C放置并收回 → D放置 → B → C → D → O`;
  }
  const material = frame.task?.material;
  if (running && material && frame.task.state.startsWith('material_')) {
    node('status').textContent = `第${material.batch}批 · ${material.station}点` + (material.item ? ` · ${material.item.color_name} · 点${material.item.target}` : ' · 校准位置');
  }
  if (running && frame.imu_pause?.paused) {
    node('status').textContent = 'IMU信号超时 · 已停车保留任务 · 等待原信号恢复';
  }
}

node('start').addEventListener('click', async () => {
  if (submitting || node('start').disabled) return;
  submitting = true; node('start').disabled = true;
  try {
    const response = await fetch('/api/competition/start', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({token, full_flow:true})});
    const result = await response.json();
    if (!result.ok) throw new Error(result.error || '启动失败');
    render(await (await fetch('/api/state', {cache:'no-store'})).json());
  } catch (error) { node('status').textContent = error.message; }
  finally { submitting = false; if (lastFrame) node('start').disabled = readonly || !!lastFrame.competition?.active || !!lastFrame.navigation_runner?.active || !!lastFrame.task?.active; }
});

const events = new EventSource('/api/events?token=' + encodeURIComponent(token));
events.onmessage = event => {
  const value = JSON.parse(event.data);
  if (value.type === 'role') { readonly = value.role === 'readonly'; if (lastFrame) render(lastFrame); }
  if (value.type === 'hello') render(value);
  if (value.type === 'snapshot') render(value.frames);
};
let polling = false;
async function refresh() {
  if (polling) return;
  polling = true;
  try {
    const heartbeat = await fetch('/api/session/heartbeat', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token})});
    readonly = (await heartbeat.json()).role === 'readonly';
    const response = await fetch('/api/state', {cache:'no-store'});
    render(await response.json());
  } catch { node('status').textContent = '正在重新连接上位机…'; node('start').disabled = true; }
  finally { polling = false; }
}
const timer = setInterval(refresh, 1000);
refresh();
window.addEventListener('pagehide', () => { clearInterval(timer); events.close(); });
