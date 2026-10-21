import { ImuView } from "/imu.js";
import { CameraView } from "/camera.js";
import { CameraAlignmentView } from "/camera_alignment.js";
import { PickPlaceView } from "/pick_place.js";
import { BPickupView } from "/b_pickup.js";
import { MaterialFlowView } from "/material_flow.js";
/* H743 robot console — front end.
   No framework: one state object, one SSE stream, and explicit render passes.
   The page never talks to the board directly; every command is POSTed to the
   Python process that owns the serial port, which keeps the heartbeat, the jog
   lease and the STOP priority exactly where the firmware expects them. */

const TOKEN = (crypto.randomUUID && crypto.randomUUID()) ||
  `s${Date.now().toString(36)}${Math.random().toString(36).slice(2)}`;

const state = {
  link: "offline",          // offline | connected
  port: null,
  demo: false,
  armed: false,
  role: "controller",       // controller | readonly
  restarting: false,
  automatic: null,
  wiringAck: false,
  device: {},
  axes: {},
  servoFrames: {},          // channel -> {angle, us, on}
  servoChannels: 2,
  chassis: {},
  invertDraft: null,
  lift: {},
  auxiliary: {},
  auxSettings: null,
  liftRenderedSign: null,
  navigation: {},
  turn: {},
  motion: {},
  flow: {},
  flowSettings: null,
  mission: null,
  fieldDirty: false,
  task: {},
  competition: {},
  cameraAlignment: {},
  pickPlace: {},
  pickPlaceConfig: null,
  bPickupConfig: null,
  materialFlowConfig: null,
  travel: 270,
  ledMask: 0,
  logs: [],
  seenLog: new Set(),
  showPolls: false,
  jogBody: null,
  selected: null,           // direction chosen for the next finite move
};

const QUIET_REPLY = /^(PING|STAT|AXIS \d|SERVO STAT|CHASSIS STAT|CHASSIS JOG|NAV STAT|AUX STAT [56]) →/;

const $ = (id) => document.getElementById(id);
const manualReadonly = () => state.role === "readonly" || !!state.task?.active || !!state.cameraAlignment?.active || !!state.pickPlace?.active;
const num = (value, fallback = null) => {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : fallback;
};

/* ── transport ─────────────────────────────────────────────────────────── */

function status(text, kind = "info") {
  if (state.restarting) { text = "正在重启上位机，等待服务恢复后自动刷新…"; kind = "warn"; }
  const node = $("status");
  node.textContent = text;
  node.style.color = kind === "warn" ? "var(--amber)" : kind === "error" ? "var(--red)" : "var(--ink-2)";
}

async function post(path, payload) {
  if (state.restarting) return {ok: false};
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json", "X-Session": TOKEN },
    body: JSON.stringify({ token: TOKEN, ack: state.wiringAck, ...payload }),
  });
  let data = {};
  try { data = await response.json(); } catch { /* an empty body is not fatal */ }
  if (!data.ok) status(data.error || "命令未被接受", "warn");
  return data;
}

/* Restart the host process after confirmation, then wait for a new instance.
   Polling is read-only and never restores arming or a previous motion task. */
async function restartHost() {
  if (state.restarting || state.role === "readonly") return;
  if (state.armed) { status("请先停止运动并取消电机使能，再重启上位机。", "warn"); return; }
  if (!window.confirm("确定重启上位机？设备连接会暂时断开，服务恢复后页面自动刷新。请先保存已修改的参数。")) return;
  const button = $("btn-host-restart");
  button.disabled = true;
  try {
    const result = await post("/api/host/restart", {});
    if (!result.ok) return;
    state.restarting = true;
    status("");
    const deadline = Date.now() + 60000;
    while (Date.now() < deadline) {
      await new Promise((resolve) => setTimeout(resolve, 700));
      try {
        const response = await fetch("/api/host/state", {cache: "no-store", signal: AbortSignal.timeout(2500)});
        const host = await response.json();
        if (response.ok && host.instance && host.instance !== result.instance && !host.restarting) {
          window.location.reload();
          return;
        }
      } catch { /* The listener is briefly unavailable during the restart. */ }
    }
    state.restarting = false;
    status("重启等待超时，请刷新页面检查上位机连接。", "error");
  } catch {
    status("重启请求未收到确认，请刷新页面检查服务状态。", "error");
  } finally {
    button.disabled = state.restarting || state.role === "readonly";
  }
}

/* Local validation still runs first so a typo never becomes a round trip; the
   board stays the only authority on what is actually accepted. */
function command(body) {
  if (!body) return Promise.resolve({ ok: false });
  return post("/api/command", { body });
}

/* Held-direction intent: the worker expires it, so a dropped link stops the
   wheels instead of latching a direction. */
async function jog(body) {
  if (body === state.jogBody) return Promise.resolve({ ok: true });
  state.jogBody = body;
  syncPad();
  const result = await post("/api/jog", { body });
  if (!result.ok && state.jogBody === body) { state.jogBody = null; syncPad(); }
  return result;
}

function stopMotion() {
  state.jogBody = null;
  syncPad();
  return Promise.all([post("/api/jog", { body: null }), command("CHASSIS HALT")]);
}

/* ── SSE ───────────────────────────────────────────────────────────────── */

function listen() {
  const source = new EventSource(`/api/events?token=${encodeURIComponent(TOKEN)}`);
  // A page must acknowledge liveness; receiving SSE data does not renew its lease.
  let renewing = false;
  async function renew() {
    if (renewing || source.readyState !== EventSource.OPEN) return;
    renewing = true;
    try {
      const response = await fetch('/api/session/heartbeat', {
        method: 'POST', headers: {'Content-Type': 'application/json', 'X-Session': TOKEN},
        body: '{}', signal: AbortSignal.timeout(4000),
      });
      const result = await response.json();
      if (result.ok && result.role !== state.role) apply({type: 'role', role: result.role});
    } catch { /* EventSource retries; an unresponsive page loses its lease. */ }
    finally { renewing = false; }
  }
  source.onopen = renew;
  const lease = setInterval(renew, 2000);
  window.addEventListener('pagehide', () => { clearInterval(lease); source.close(); });
  source.onmessage = (event) => {
    let payload;
    try { payload = JSON.parse(event.data); } catch { return; }
    apply(payload);
  };
  source.onerror = () => {
    if (state.link === "connected") status("与本地服务端的推送连接中断，正在重连…", "warn");
  };
}

function apply(event) {
  switch (event.type) {
    case "imu":
      imuView.update(event);
      break;
    case "hello":
      // Identity snapshots follow the role event and must preserve its permission.
      adopt(event);
      renderAll();
      if (state.role !== "readonly") {
        status(state.link === "connected" ? "连接成功 · 心跳 250 ms · 状态自动刷新" : "未连接 · 等待设备");
      }
      break;
    case "snapshot":
      if (event.frames) { adopt(event.frames); renderAll(); }
      break;
    case "link":
      state.link = event.state === "connected" ? "connected" : "offline";
      state.port = event.port;
      state.demo = event.port === "SIM";
      if (state.link === "connected") state.flow = {};
      if (state.link === "offline") {
        state.armed = false;
        state.servoFrames = {};
        state.lift = {};
        state.chassis = {};
        state.turn = {};
        state.motion = {};
        state.invertDraft = null;
        state.liftRenderedSign = null;
        state.auxiliary = {};
        state.axes = {};
        state.ledMask = 0;
        state.selected = null;
        state.jogBody = null;
        syncPad();
      }
      status(state.link === "connected" ? "连接成功 · 心跳 250 ms · 状态自动刷新" : "已断开");
      renderAll();
      break;
    case "role":
      state.role = event.role;
      renderRole();
      renderFlow();
      renderServo();
      renderLift();
      renderAuxiliary();
      renderPickPlace();
      if (state.role === 'controller') status('当前页面可控制 · 设备连接保持不变');
      break;
    case "device":
      state.device = event.fields || {};
      state.travel = Number(state.device.servo_travel) / 10 || state.travel;
      if (state.device.servo) state.servoChannels = Number(state.device.servo);
      renderDevice();
      renderFlow();
      servoCards().forEach(drawGauge);
      renderServo();
      break;
    case "stat":
      renderStat(event.fields);
      break;
    case "axis":
      state.axes[event.fields.axis] = event.fields;
      renderAxes();
      break;
    case "servo":
      state.servoFrames = { ...state.servoFrames, ...parseServoFrames(event.raw) };
      if (event.travel) state.travel = Number(event.travel);
      if (event.channels) state.servoChannels = Number(event.channels);
      renderServo();
      break;
    case "chassis":
      chassisTick(event.fields);
      break;
    case "lift":
      state.lift = event.fields || {};
      renderLift();
      renderAuxiliary();
      break;
    case "auxiliary":
      state.auxiliary[event.fields.axis] = event.fields;
      renderAuxiliary();
      break;
    case "aux_settings":
      adoptAuxiliary(event.config);
      $("aux-config-state").textContent = "板端范围已确认 · 参数已保存";
      break;
    case "navigation_runner":
      state.navigationRunner = event.fields; renderNavigation(); break;
    case "navigation":
      state.navigation = event.fields || {};
      renderNavigation();
      renderRecovery();
      break;
    case "flow":
      state.flow = {...state.flow, ...event.fields};
      renderFlow();
      break;
    case "flow_settings":
      adoptFlowSettings(event.config);
      renderFlow();
      break;
    case "turn":
      state.turn = event.fields || {};
      renderTurn();
      renderChassis();
      renderNavigation();
      renderRecovery();
      break;
    case "mission":
      adoptMission(event.config);
      break;
    case "task_config":
      $("task-config").value = JSON.stringify(event.config, null, 2);
      break;
    case "led":
      state.ledMask = event.mask;
      renderLeds();
      break;
    case "arm":
      state.armed = event.armed;
      if (!state.armed) { state.jogBody = null; syncPad(); }
      renderArm();
      renderPickPlace();
      break;
    case "motion":
      state.motion = event.fields || {};
      if (state.motion.latched === "1") { state.jogBody = null; syncPad(); }
      renderRecovery();
      break;
    case "jog_cancel":
      state.jogBody = null; syncPad(); renderChassis();
      break;
    case "log":
      pushLog(event.entry);
      break;
    case "status":
      status(event.text, "warn");
      break;
    case "camera2":
      camera2View.updateRemote(event.frame);
      break;
    case "task_progress":
      state.task = event.task;
      renderTask();
      renderPickPlace();
      break;
    case "pick_place":
      state.pickPlace = event.status || {};
      renderAll();
      break;
    case "pick_place_config":
      state.pickPlaceConfig = event.config;
      renderPickPlace();
      break;
    case "b_pickup_config":
      state.bPickupConfig = event.config;
      renderPickPlace();
      break;
    case "material_flow_config":
      state.materialFlowConfig = event.config;
      renderTask();
      break;
    case "ping":
      break;
  }
}

function adopt(frame) {
  if (frame.control_settings?.turn && !state.controlSettingsLoaded) {
    state.controlSettingsLoaded = true;
    const [kp, tolerance, minimum] = frame.control_settings.turn;
    for (const prefix of ['turn','nav-turn','imu-turn']) {
      $(`${prefix}-kp`).value = String(kp / 100);
      $(`${prefix}-tolerance`).value = String(tolerance / 100);
      $(`${prefix}-min-sps`).value = String(minimum);
    }
  }
  if (frame.pick_place) state.pickPlace = frame.pick_place;
  if (frame.pick_place_config) state.pickPlaceConfig = frame.pick_place_config;
  if (frame.b_pickup_config) state.bPickupConfig = frame.b_pickup_config;
  if (frame.material_flow_config) state.materialFlowConfig = frame.material_flow_config;
  if (frame.task) state.task = frame.task;
  if (frame.competition) state.competition = frame.competition;
  if (frame.camera_alignment) { state.cameraAlignment = frame.camera_alignment; cameraAlignmentView.update(frame.camera_alignment); }
  if (frame.task_config && !$("task-config").value) $("task-config").value = JSON.stringify(frame.task_config, null, 2);
  if (frame.imu) imuView.update(frame.imu);
  if (frame.auto_devices !== undefined) state.automatic = frame.auto_devices;
  state.link = frame.link === "connected" ? "connected" : "offline";
  state.port = frame.port;
  state.demo = !!frame.demo;
  state.armed = !!frame.armed;
  if (!state.armed) state.jogBody = null;
  if (frame.motion) state.motion = frame.motion;
  if (typeof frame.wiring_ack === "boolean" && !state.wiringAck) {
    state.wiringAck = frame.wiring_ack;
    $("ack").checked = frame.wiring_ack;
  }
  if (frame.device) {
    state.device = frame.device;
    state.travel = Number(frame.travel) || Number(frame.device.servo_travel) / 10 || 270;
    if (frame.device.servo) state.servoChannels = Number(frame.device.servo);
  }
  if (Array.isArray(frame.servo_channels)) state.servoChannels = frame.servo_channels.length;
  else if (typeof frame.servo_channels === "number") state.servoChannels = frame.servo_channels;
  if (frame.axes) state.axes = frame.axes;
  if (frame.servo && frame.servo.raw) state.servoFrames = parseServoFrames(frame.servo.raw);
  if (frame.chassis) state.chassis = frame.chassis;
  if (frame.lift) state.lift = frame.lift;
  if (frame.auxiliary) state.auxiliary = frame.auxiliary;
  if (frame.aux_settings && !state.auxSettings) adoptAuxiliary(frame.aux_settings);
  if (frame.navigation) state.navigation = frame.navigation;
  if (frame.navigation_runner) state.navigationRunner = frame.navigation_runner;
  if (frame.turn) state.turn = frame.turn;
  if (frame.flow) state.flow = frame.flow;
  if (frame.flow_settings && !state.flowSettings) adoptFlowSettings(frame.flow_settings);
  if (frame.mission && !state.mission) adoptMission(frame.mission);
  if (Array.isArray(frame.log)) {
    state.logs = frame.log.slice(-400);
    state.seenLog = new Set(state.logs.map((entry) => entry.seq));
  }
}

/* ── render ────────────────────────────────────────────────────────────── */

function renderAll() {
  renderLink();
  renderRole();
  renderArm();
  renderDevice();
  renderAxes();
  renderServo();
  renderLift();
  renderAuxiliary();
  renderLeds();
  renderChassis();
  renderNavigation();
  renderFlow();
  renderLog();
  renderTask();
  renderPickPlace();
}

function renderPickPlace() {
  if (typeof pickPlaceView === 'undefined') return;
  pickPlaceView.update(state.pickPlace, state.pickPlaceConfig);
  pickPlaceView.render(state);
  bPickupView.update(state.bPickupConfig, state);
}

function renderTask() {
  materialFlowView.update(state.materialFlowConfig, state);
  const task = state.task || {};
  const labels = {idle:"未启动",uploading:"下发路线",travel:"前往站点",actions:"执行站点动作",vision:"等待视觉",qr_wait:"A点扫码与搜索",qr_switch:"任务相机切换",qr_return:"扫码完成，返回A点",nudge_wait:"视觉微调",aux_wait:"等待辅助轴",servo_wait:"等待舵机稳定",wait:"等待",departing:"离开站点",done:"任务完成",stopped:"已停止",fault:"任务故障"};
  const ringLabels = {ring_start:"交接视觉控制",ring_begin:"交接视觉控制",ring_vision:"圆列 1→2→3→2",ring_fix:"建立数字2基准",ring_exit_accept:"开始右移",ring_exit:"执行右移",ring_end:"确认右移完成",ring_replan:"重算剩余路线",ring_upload:"下发剩余路线"};
  $("task-state").textContent = labels[task.state] || ringLabels[task.state] || "未启动";
  $("task-detail").textContent = task.error || (task.active ? `第 ${task.leg || "—"} 段 · 动作 ${task.action}` : "请核对路线与任务配置后启动");
  if (task.active && task.material && task.state.startsWith("material_")) {
    const m = task.material;
    const phases = {suspend:"交接控制",camera:"打开摄像头1",align:"视觉校准",cycle:"抓取/放置",move:"点间移动",arrived:"确认到点",return:"返回通道"};
    $("task-state").textContent = "物料搬运";
    $("task-detail").textContent = `第${m.batch}批 · ${m.station}点 · ${phases[m.phase] || m.phase}` + (m.item ? ` · ${m.item.color_name} · 转盘${m.item.angle_deg}° → 点${m.item.target}` : "");
  }
  if (state.competition.active && !task.active) {
    $("task-state").textContent = "全流程准备中";
    $("task-detail").textContent = "正在连接设备、检查机构零位与行程并建立O点";
  } else if (state.competition.full_flow && state.competition.error) {
    $("task-state").textContent = "全流程已中止";
    $("task-detail").textContent = state.competition.error;
  }
  $("material-start").disabled = state.role === "readonly" || state.competition.active || task.active || !!state.navigationRunner?.active || state.pickPlace.active || state.cameraAlignment.active;
  $("task-request").textContent = task.request ? `等待识别：${task.request.kind} · ${task.request.target} · 站点 ${task.request.station}` : "没有视觉请求";
  $("task-start").disabled = state.role === "readonly" || task.active || !state.armed || state.link !== "connected";
  if (state.cameraAlignment.active || state.navigationRunner?.active || state.pickPlace.active) $("task-start").disabled = true;
  $("task-stop").disabled = state.role === "readonly" || (!task.active && !state.competition.active);
  $("task-save").disabled = state.role === "readonly" || task.active;
  $("task-config").disabled = state.role === "readonly" || task.active;
  camera2View.setTask(task);
  const plan = task.task_plan;
  $("task-code-status").textContent = task.qr_status || '在A点路线段选择“读取任务码并切换相机”后启用。';
  $("task-code-locked").textContent = plan?.code || '尚未锁定任务码';
  const search = task.qr_search;
  const searchPhases = {watch:'停车识别',suspend:'交接底盘',pose:'建立搜索起点',accept:'开始移动',moving:'移动中',verify:'确认回位',done:'已回到A点'};
  $("task-code-search").textContent = search
    ? `${search.returning ? '返回A点' : '持续前后搜索'} · ${searchPhases[search.phase] || search.phase} · 相对起点 ${search.position_steps} 脉冲 · 范围 ±${search.range_steps} · 步长 ${search.step} · 速度 ${search.speed} step/s`
    : 'A点未识别时持续前后搜索，直到识别成功或手动停止：速度2500 step/s，范围±3200脉冲，每步800脉冲。';
  const rows = plan ? plan.batches.flatMap(batch => batch.items.map(item =>
    [batch.batch, `${item.angle_deg}°`, `${item.color_name}（${item.color_id}）`, item.target])) : [];
  const table = $("task-code-rows");
  const signature = JSON.stringify(rows);
  if (table.dataset.plan !== signature) {
    table.dataset.plan = signature;
    table.replaceChildren(...rows.map(values => {
      const row = document.createElement('tr');
      values.forEach(value => { const cell = document.createElement('td'); cell.textContent = String(value); row.appendChild(cell); });
      return row;
    }));
  }
  $("nav-go").disabled = manualReadonly() || state.navigation.held !== "1";
}

function renderLink() {
  $("link-dot").className = "dot" + (state.link === "connected" ? (state.demo ? " demo" : " on") : "");
  $("link-text").textContent = state.link !== "connected"
    ? "未连接"
    : (state.demo ? "模拟板 · 演示数据" : `已连接 ${state.port || ""}`);
  $("btn-connect").textContent = state.link === "connected" ? "断开" : "连接";
}

function renderRole() {
  const readonly = manualReadonly();
  $("btn-host-restart").disabled = readonly || state.restarting;
  imuView.setRole(manualReadonly() ? "readonly" : state.role);
  cameraView.setRole(state.role);
  cameraAlignmentView.setRole(state.role, !!state.task.active);
  if (state.cameraAlignment.busy) {
    for (const id of ["camera-start", "camera-stop", "camera-apply", "camera-source", "camera-device", "camera-resolution", "camera-bind"]) $(id).disabled = true;
  }
  camera2View.setRole(state.role);
  $("auto-devices-bar").classList.toggle("hidden", !state.automatic);
  $("auto-devices-enabled").checked = !!state.automatic?.enabled;
  $("auto-devices-enabled").disabled = readonly;
  $("auto-devices-status").textContent = state.automatic?.message || "";
  $("role-badge").classList.toggle("hidden", !readonly);
  $("role-badge").textContent = state.navigationRunner?.active ? "坐标巡航正在控制" : state.cameraAlignment.active ? "摄像头对准正在控制" : state.task.active ? "自动任务正在控制" : "其他页面持有控制权";
  $("btn-take-control").classList.toggle('hidden', !readonly || !!state.task.active || !!state.cameraAlignment.active);
  /* Disconnect and the view switches (theme, notes) stay available so a second
     page is never trapped; only the commands that touch the board are locked
     out. btn-theme used to be in this list, which contradicted the comment. */
  for (const id of ["pulse-send", "ch-move",
                    "ax-move", "w-apply", "w-zero", "inv-apply"]) {
    const node = $(id);
    if (node) node.disabled = readonly;
  }
  document.querySelectorAll("[data-led], .pad, .presets button").forEach((node) => {
    node.disabled = readonly;
  });
  $("btn-stop").disabled = state.role === "readonly";
  $("btn-arm").disabled = state.role === "readonly" || (!!state.task.active && !state.armed);
  if (readonly && !state.task.active && !state.cameraAlignment.active) status("当前页面仅查看；点击“接管控制”即可操作。", "warn");
  renderNavigation();
  renderChassis();
  renderRecovery();
}

function renderArm() {
  const text = state.link !== "connected" ? "未连接" : (state.armed ? "已使能" : "已关闭");
  $("m-armed").textContent = text;
  $("btn-arm").textContent = state.armed ? "取消使能" : "使能输出";
  renderLift();
  renderAuxiliary();
  renderNavigation();
  renderChassis();
  renderRecovery();
}

/* Acknowledgement never enables outputs or resumes the canceled task.
   Live faults and historical reason are displayed separately. */
function renderRecovery() {
  const button = $("btn-recover"), note = $("motion-status");
  if (!button || !note) return;
  const f = state.motion || {}, supported = f.v === "1";
  button.disabled = state.role === "readonly" || state.link !== "connected" ||
    state.armed || !!state.task.active || !supported || f.active !== "0";
  const reasons = {step_deadline:"步进定时超期",position_overflow:"脉冲位置溢出"};
  const fault = f.latched === "1" ? reasons[f.reason] || f.reason :
    [state.chassis,state.navigation,state.turn].find(s => s.state === "fault")?.fault;
  const uncertain = [5,6].filter(axis => Number(f.uncertain || 0) & (1 << (axis-1)));
  note.textContent = state.link !== "connected" ? "" : fault ?
    `故障已停车：${fault}。停止全部 → 排除原因 → 故障复位。` : uncertain.length ?
    `请先人工恢复 M${uncertain.join("、M")} 位置参考，再建立 O 点。` :
    f.total !== undefined && Number(f.total) > 0 ? `历史运动故障 ${f.total} 次 · 当前已复位` : "";
  button.title = !supported ? "需要 H743 0.2.21 或更新固件" :
    "排除故障后复位；保持停止，随后需重新建立 O 点。";
}

/* Auxiliary presets and limits are operator settings. Only AUX_STAT supplies
   the live position; state polls must never overwrite inputs being edited. */
function auxCards() { return [...document.querySelectorAll(".aux-card")].filter(card => ["5", "6"].includes(card.dataset.axis)); }

function adoptAuxiliary(config) {
  const previous = state.auxSettings;
  state.auxSettings = config;
  auxCards().forEach(card => {
    const value = config[card.dataset.axis];
    card.querySelectorAll(".aux-setting").forEach(input => {
      const key = input.dataset.key;
      const old = previous?.[card.dataset.axis];
      const oldValue = old && (key.startsWith("p") ? old.presets[Number(key[1])] : old[key]);
      if (previous && input.value !== String(oldValue)) return;
      input.value = input.dataset.key.startsWith("p") ? value.presets[Number(input.dataset.key[1])] : value[input.dataset.key];
    });
  });
  $("aux-config-state").textContent = "已读取保存参数 · 重连后在零位参考有效、轴静止时自动恢复范围，无需重复应用。";
}

function readAuxiliary(selected = null) {
  const config = selected && state.auxSettings ? JSON.parse(JSON.stringify(state.auxSettings)) : {};
  auxCards().forEach(card => {
    if (selected && card.dataset.axis !== selected) return;
    const axis = card.dataset.axis, value = {presets: [0, 0]};
    card.querySelectorAll(".aux-setting").forEach(input => {
      const v = input.value.trim() === "" ? NaN : Number(input.value), key = input.dataset.key;
      if (!Number.isSafeInteger(v)) throw new Error(`M${axis} 参数必须为整数。`);
      if (key.startsWith("p")) value.presets[Number(key[1])] = v;
      else value[key] = v;
    });
    if (value.min > value.max || value.min < (axis === "5" ? 0 : -1000000) || value.max > 1000000 ||
        (axis === "5" && value.min === value.max) || value.sps < 1 || value.sps > 3000 ||
        value.accel < 1 || value.accel > 20000 || value.step < 1 || value.step > 2000000 ||
        value.presets.some(v => v < value.min || v > value.max)) throw new Error(`M${axis} 范围或参数无效；常用位置必须在范围内。`);
    config[axis] = value;
  });
  return config;
}

async function saveAuxiliary(axis = null) {
  if (state.role === "readonly") { status("当前页面只读。", "warn"); return null; }
  try {
    const config = readAuxiliary(axis);
    if (axis && state.link === "connected") {
      $("aux-config-state").textContent = `M${axis} 正在应用 · 等待板端确认，尚未保存`;
      const result = await post("/api/auxiliary/apply", {axis, config});
      if (!result.ok) $("aux-config-state").textContent = `M${axis} 应用失败，参数未保存`;
      return result.ok ? config : null;
    }
    const result = await post("/api/auxiliary/config", {config});
    if (!result.ok) return null;
    adoptAuxiliary(result.config);
    $("aux-config-state").textContent = "离线参数已保存 · 连接后在位置参考有效、轴静止时自动恢复范围。";
    status("离线保存完成，板端范围尚未修改。");
    return result.config;
  } catch (error) { status(error.message, "warn"); return null; }
}

/* Use board state to explain the same prerequisites that gate motion buttons.
   Only acknowledged board state can establish readiness. */
function auxiliaryBlockedReason(axis, f, sameBounds) {
  if (state.task.active) return "自动任务正在控制辅助轴。";
  if (state.role === "readonly") return "当前页面只读，请点击“接管控制”。";
  if (state.link !== "connected") return "尚未连接 H743。";
  if (state.device.aux_control !== "1") return "固件不支持辅助轴控制。";
  if (f.active !== "0") return "请等待该轴停止。";
  if (f.referenced !== "1") return axis === "5" ? "升降零位尚未建立，请确认机构处于最高点。" : "旋转零位尚未建立：解除使能，将机构放在选定零位后点击“当前位置设为零”。";
  if (axis === "5" && !["1", "-1"].includes(state.lift.down_sign)) return "升降方向未确认：先解除使能，在最高点选择向下的步数方向，再点击“应用方向”。";
  if (f.configured !== "1") return "板端范围尚未应用，请点击“保存并应用”本轴参数。";
  if (!sameBounds) return "输入范围尚未生效，请点击“保存并应用”本轴参数。";
  if (!state.armed) return "输出未使能，请点击“使能输出”。";
  return "";
}

function renderAuxiliary() {
  auxCards().forEach(card => {
    const axis = card.dataset.axis, f = state.auxiliary[axis] || {};
    const writable = !manualReadonly(), connected = writable && state.link === "connected" && state.device.aux_control === "1";
    const idle = f.active === "0", referenced = f.referenced === "1";
    const sameBounds = Number(card.querySelector('[data-key="min"]').value) === Number(f.min) && Number(card.querySelector('[data-key="max"]').value) === Number(f.max);
    const blockedReason = auxiliaryBlockedReason(axis, f, sameBounds);
    const ready = !blockedReason;
    card.querySelector('.aux-readiness').textContent = blockedReason ? `移动按钮不可用：${blockedReason}` : '零位、方向和范围已就绪，输出已使能。';
    card.querySelector(".aux-position-value").textContent = f.pos ?? "—";
    card.querySelector(".aux-live").textContent = state.device.aux_control !== "1" ? "请连接 v0.2.8 或更新固件。" :
      !referenced ? "请先建立零位。" : f.configured !== "1" ? "零位已建立，请设置并应用范围。" :
      `板端范围 ${f.min}～${f.max} · ${idle ? "静止" : "运动中"}${sameBounds ? " · 输入范围已生效" : " · 输入范围尚未生效"}`;
    card.querySelectorAll(".aux-setting").forEach(input => { input.disabled = !writable; });
    card.querySelectorAll("[data-action]").forEach(button => {
      const action = button.dataset.action;
      button.disabled = action === "save" ? !writable :
        action === "apply" ? !(connected && idle && referenced && (!state.armed || state.device.aux_live === "1")) :
        action === "zero" ? !(connected && idle && !state.armed && state.motion.latched !== "1" &&
          (axis === "6" || (state.motion.v === "1" && !referenced))) :
        action === "reference" ? !(connected && idle && !state.armed && state.motion.latched !== "1" && state.motion.v === "1" && !referenced) :
        action === "halt" ? !connected :
        action.startsWith("record-") ? !(connected && idle && referenced) :
        action === "home" ? !(ready && Number(f.min) <= 0 && Number(f.max) >= 0 && Number(f.pos) !== 0) : !ready;
      if (["negative", "positive", "home"].includes(action) || action.startsWith("goto-")) {
        button.title = button.disabled ? blockedReason || (Number(f.pos) === 0 ? '已在最高点。' : '最高点不在板端配置范围内。') : '';
      }
    });
  });
}

async function auxiliaryAction(card, action) {
  const axis = card.dataset.axis, f = state.auxiliary[axis] || {};
  if (state.role === "readonly") { status("当前页面只读。", "warn"); return; }
  if (action === "save") { await saveAuxiliary(axis); return; }
  if (state.link !== "connected" || state.device.aux_control !== "1") { status("请连接支持升降与旋转控制的固件。", "warn"); return; }
  if (action === "halt") { await command(`AUX HALT ${axis}`); return; }
  if (action === "apply") { await saveAuxiliary(axis); return; }
  if (action === "reference") {
    const input = card.querySelector('.aux-reference-position').value;
    const value = input.trim() === "" ? NaN : Number(input);
    if (state.armed || f.active !== "0" || !Number.isSafeInteger(value) || value < Number(f.min) || value > Number(f.max)) {
      status("请先排除故障并解除使能；人工确认的位置须在板端范围内。", "warn");return;
    }
    await command(`AUX REFERENCE ${axis} ${value}`);return;
  }
  if (action === "zero") {
    if (state.armed || f.active !== "0") { status("请先停止该轴并解除使能。", "warn"); return; }
    if (action === "zero") { await command(`AUX ZERO ${axis}`); return; }
    return;
  }
  if (f.active !== "0" || f.referenced !== "1") { status("请先建立零位并等待该轴停止。", "warn"); return; }
  if (action.startsWith("record-")) {
    card.querySelector(`[data-key="p${action.slice(-1)}"]`).value = f.pos;
    $("aux-config-state").textContent = "已记录当前位置，请保存参数。";
    return;
  }
  if (!state.armed || f.configured !== "1" || (axis === "5" && !["1", "-1"].includes(state.lift.down_sign))) {
    status("请先应用范围、确认方向并使能。", "warn"); return;
  }
  try {
    const p = readAuxiliary(axis)[axis];
    if (p.min !== Number(f.min) || p.max !== Number(f.max)) throw new Error(`M${axis} 输入范围尚未生效，请先保存并应用。`);
    const absolute = action === "home" || action.startsWith("goto-");
    const value = action === "home" ? 0 : absolute ? p.presets[Number(action.slice(-1))] : p.step * (action === "positive" ? 1 : -1);
    const target = absolute ? value : Number(f.pos) + value;
    if (target < Number(f.min) || target > Number(f.max)) throw new Error(`M${axis} 目标超出板端范围，请检查参数或应用新范围。`);
    await command(`AUX ${absolute ? "GOTO" : "STEP"} ${axis} ${value} ${p.sps} ${p.accel}`);
  } catch (error) { status(error.message, "warn"); }
}

/* Depth is supplied by firmware relative to its first top reference, never
   reconstructed/reset by a browser reconnect or by opening a second page. */
/* Flow totals are MCU-owned. Polling never adds displacement in the browser. */
function adoptFlowSettings(config) {
  if (!config) return;
  state.flowSettings = config;
  $("flow-height").value = config.height_mm;
  $("flow-gx").value = config.gain_x;
  $("flow-gy").value = config.gain_y;
  $("flow-angle").value = config.angle_deg;
  $("flow-quality").value = config.min_quality;
  $("flow-swap").checked = !!(config.map & 4);
  $("flow-negx").checked = !!(config.map & 1);
  $("flow-negy").checked = !!(config.map & 2);
}

/* Exact range/precision validation is repeated by Python and firmware. */
function readFlowSettings() {
  const value = id => {
    const text = $(id).value.trim();
    if (!/^-?\d+(\.\d+)?$/.test(text)) throw new Error("光流参数需要普通十进制数。");
    return text;
  };
  const c = {height_mm:value("flow-height"), gain_x:value("flow-gx"), gain_y:value("flow-gy"),
    angle_deg:value("flow-angle"), min_quality:Number(value("flow-quality")),
    map:($("flow-negx").checked ? 1 : 0) | ($("flow-negy").checked ? 2 : 0) | ($("flow-swap").checked ? 4 : 0)};
  if (!(Number(c.height_mm)>=1 && Number(c.height_mm)<=1000 && Number(c.gain_x)>=.01 && Number(c.gain_x)<=10 &&
        Number(c.gain_y)>=.01 && Number(c.gain_y)<=10 && Math.abs(Number(c.angle_deg))<=180 &&
        Number.isInteger(c.min_quality) && c.min_quality>=1 && c.min_quality<=255)) throw new Error("光流高度、比例、安装角或置信度超出范围。");
  for (const [key, digits] of [["height_mm",3],["gain_x",6],["gain_y",6],["angle_deg",2]]) {
    const fraction = (c[key].split(".")[1] || "").replace(/0+$/, "");
    if (fraction.length>digits) throw new Error("高度最多三位小数，比例最多六位小数，安装角最多两位小数。");
  }
  return c;
}

async function saveFlowSettings() {
  try {
    const result = await post("/api/flow/config", {config:readFlowSettings()});
    if (result.ok) { state.flowSettings=result.config; renderFlow(); status("光流参数已保存，尚未应用到板端。"); }
  } catch (error) { status(error.message, "warn"); }
}

/* Display mode selects already accumulated vectors; switching never resets. */
function renderFlow() {
  const f = state.flow || {}, world = $("flow-mode").value === "world";
  const keys = world ? ["wx_um","wy_um","wrx_um","wry_um"] : ["x_um","y_um","rx_um","ry_um"];
  const v = keys.map(key => f[key] === undefined ? null : Number(f[key])/1000);
  const format = value => value === null || !Number.isFinite(value) ? "—" : value.toFixed(3);
  $("flow-x").textContent = format(v[0]); $("flow-y").textContent = format(v[1]);
  $("flow-length").textContent = v[0]===null || v[1]===null ? "—" : format(Math.hypot(v[0],v[1]));
  $("flow-rx").textContent = f.marked === "1" ? format(v[2]) : "—";
  $("flow-ry").textContent = f.marked === "1" ? format(v[3]) : "—";
  $("flow-rlength").textContent = f.marked === "1" && v[2]!==null && v[3]!==null ? format(Math.hypot(v[2],v[3])) : "—";
  const elapsed = ((Number(f.recording==="1" ? f.now_ms : f.end_ms) - Number(f.start_ms)) >>> 0)/1000;
  $("flow-record-time").textContent = f.marked !== "1" ? "尚未建立记录起点。" :
    `MCU 起点 ${(Number(f.start_ms)/1000).toFixed(3)} s · 已记录 ${elapsed.toFixed(3)} s · ${f.recording==="1" ? "记录中" : "已结束，结果保留"}`;
  let warning = world ? ` · 地面坐标始于 MCU ${(Number(f.world_ms || 0)/1000).toFixed(3)} s` : "";
  if (world && (f.heading !== "1" || Number(f.gaps || 0))) warning += ` · 航向${f.locked==="1" ? "会话变化，坐标已锁定" : f.heading!=="1" ? "缺失" : "已恢复"} · 累计缺口 ${f.gaps || 0} 帧，记录缺口 ${f.rgaps || 0} 帧`;
  if (f.sat === "1") warning += " · 累计超出范围，已暂停";
  $("flow-live").textContent = state.link!=="connected" ? "离线 · 显示上次值，尚未刷新" :
    state.device.flow!=="1" ? "固件不支持光流，请使用 0.2.11 或更新版本。" :
    `${state.demo ? "模拟数据 · " : ""}${f.online==="1" ? "传感器在线" : "未收到新数据"} · 置信度 ${f.valid ?? "—"} · 周期 ${f.dt_us ?? "—"} µs · 接收 ${f.frames || 0} / 采用 ${f.accepted || 0} / 坏帧 ${f.bad || 0} / 过滤 ${f.low || 0} / UART错误 ${f.uart || 0}${warning}`;
  if (f.height_um !== undefined) $("flow-applied").textContent = `板端参数版本 ${f.rev || 0} · 高度 ${Number(f.height_um)/1000} mm · 比例 ${Number(f.gx_ppm)/1e6} / ${Number(f.gy_ppm)/1e6} · 映射 ${f.map} · 安装角 ${Number(f.angle_cd)/100}° · 最低置信度 ${f.minq}`;
  const writable = !manualReadonly(), connected = writable && state.link==="connected" && state.device.flow==="1";
  $("flow-start").disabled = !connected; $("flow-stop").disabled = !connected || f.recording!=="1";
  $("flow-apply").disabled = !connected || !state.flowSettings; $("flow-save").disabled = !writable;
  document.querySelectorAll(".flow-form input").forEach(input => { input.disabled = !writable; });
  drawFlowVector(v, f.marked === "1");
}

/* Arrows end at the vector sum. Plot scales both modes identically in mm. */
function drawFlowVector(values, marked) {
  const canvas = $("flow-vector"), ctx = canvas.getContext("2d");
  if (!ctx) return;
  const w=canvas.width, h=canvas.height, ox=w/2, oy=h/2;
  const styles=getComputedStyle(document.documentElement), colour=name=>styles.getPropertyValue(name).trim();
  ctx.clearRect(0,0,w,h); ctx.strokeStyle=colour("--muted"); ctx.fillStyle=colour("--muted");
  ctx.beginPath(); ctx.moveTo(20,oy); ctx.lineTo(w-20,oy); ctx.moveTo(ox,20); ctx.lineTo(ox,h-20); ctx.stroke();
  ctx.fillText("+X",w-35,oy-7); ctx.fillText("+Y",ox+7,24);
  const v=values.map(value=>Number.isFinite(value) ? value : 0), extent=Math.max(1,...v.map(Math.abs));
  const scale=Math.min(ox-45,oy-35)/extent;
  for (const [index,tone] of [[0,"--cyan"],...(marked ? [[2,"--teal"]] : [])]) {
    const x=ox+v[index]*scale,y=oy-v[index+1]*scale,a=Math.atan2(y-oy,x-ox);
    ctx.strokeStyle=colour(tone); ctx.lineWidth=3; ctx.beginPath(); ctx.moveTo(ox,oy);ctx.lineTo(x,y);
    if (Math.hypot(x-ox,y-oy)>3) {ctx.moveTo(x-10*Math.cos(a-.5),y-10*Math.sin(a-.5));ctx.lineTo(x,y);ctx.lineTo(x-10*Math.cos(a+.5),y-10*Math.sin(a+.5));}
    ctx.stroke();
  }
  ctx.fillStyle=colour("--muted");ctx.fillText(`显示范围 ±${extent.toFixed(3)} mm`,20,h-10);
}

function renderLift() {
  const f = state.lift || {};
  if (f.down_sign !== undefined && state.liftRenderedSign !== f.down_sign) {
    $("lift-sign").value = f.down_sign;
    state.liftRenderedSign = f.down_sign;
  }
  const supported = state.device.lift === "1";
  const connected = state.link === "connected" && supported;
  const writable = connected && !manualReadonly();
  const initialized = f.initialized === "1";
  const directionKnown = Number(f.down_sign) === 1 || Number(f.down_sign) === -1;
  const idle = f.active === "0";
  const ready = writable && initialized && directionKnown && idle && state.armed;
  const up = Number(f.up_room || 0), down = Number(f.down_room || 0);
  $("lift-state").textContent = !connected ? "未连接或固件不支持 M5 限位。"
    : !initialized ? "等待建立最高点零位。"
    : `当前位置 ${f.pos} / ${f.max} 脉冲 · 还能向上 ${up}、向下 ${down} · `
      + (!directionKnown ? "方向未确认" : !idle ? "运行中" : !state.armed ? "未使能" : "就绪")
      + (directionKnown ? ` · ${Number(f.down_sign) > 0 ? "正" : "负"}步数向下` : "");
  $("lift-dir").disabled = !writable || !initialized || !idle || state.armed || Number(f.pos) !== 0;
  $("lift-sign").disabled = $("lift-dir").disabled;
  $("lift-down").disabled = !ready || down <= 0;
  $("lift-up").disabled = !ready || up <= 0;
  $("lift-top").disabled = !ready || up <= 0;
  $("lift-stop").disabled = !writable;
}

/* Front-end checks improve feedback; firmware independently rejects every
   out-of-range target, including commands sent through the raw MOVE form. */
async function moveLift(direction, returnToTop = false) {
  const f = state.lift || {};
  if (state.link !== "connected" || state.role === "readonly" || !state.armed ||
      f.initialized !== "1" || ![1, -1].includes(Number(f.down_sign)) || f.active !== "0") {
    status("M5 需先连接、确认方向并使能，且必须停止后才能接受下一条运动。", "warn"); return;
  }
  const steps = returnToTop ? Number(f.up_room) : Number($("lift-steps").value);
  const sps = Number($("lift-sps").value), accel = Number($("lift-accel").value);
  const room = Number(direction === "DOWN" ? f.down_room : f.up_room);
  if (!Number.isInteger(steps) || steps < 1 || steps > room) {
    status(`本次${direction === "DOWN" ? "向下" : "向上"}最多 ${room} 个脉冲。`, "warn"); return;
  }
  if (!Number.isInteger(sps) || sps < 1 || sps > 3000 || !Number.isInteger(accel) || accel < 1 || accel > 20000) {
    status("速度范围 1–3000 step/s，加速度范围 1–20000 step/s²。", "warn"); return;
  }
  await command(`LIFT MOVE ${direction} ${steps} ${sps} ${accel}`);
  command("LIFT STAT");
}

function renderDevice() {
  const d = state.device;
  $("d-firmware").textContent = d.fw ? `${d.fw} / v${d.protocol}` : "—";
  $("d-board").textContent = d.board || "—";
  $("d-uid").textContent = d.uid || "—";
  $("d-flash").textContent = d.flash_kb ? `${d.flash_kb} KB / FreeRTOS ${d.rtos}` : "—";
  $("d-reset").textContent = d.reset || "—";
  if (d.cpu_hz) $("m-cpu").textContent = `${Math.round(Number(d.cpu_hz) / 1e6)} MHz`;
}

function renderStat(f) {
  $("m-uptime").textContent = `${(Number(f.uptime_ms) / 1000).toFixed(1)} s`;
  $("m-heap").textContent = `${(Number(f.heap_free) / 1024).toFixed(1)} KiB`;
  $("r-tasks").textContent = `${f.tasks} / ${f.heap_min} bytes`;
  $("r-protocol").textContent = `${f.bad} / ${f.replay}`;
  $("r-stacks").textContent = `${f.stack_comm} / ${f.stack_motion} / ${f.stack_super} bytes`;
  $("r-faults").textContent = `${f.timeout} / ${f.motion_fault}`;
  $("r-serial").textContent = `${f.rx} / ${f.rx_err} / ${f.tx_err}`;
  state.armed = f.armed === "1";
  if (!state.armed) { state.jogBody = null; syncPad(); }
  renderArm();
}

function renderAxes() {
  const body = $("axis-body");
  if (body.childElementCount !== 7) {
    body.innerHTML = "";
    for (let i = 1; i <= 7; i += 1) {
      const row = document.createElement("tr");
      row.id = `axis-row-${i}`;
      row.innerHTML = `<td class="name">M${i}</td><td>${i <= 4 ? 5000 : 3000}</td>`
        + "<td>—</td><td>—</td><td>—</td><td>未知</td><td>—</td>";
      body.append(row);
    }
  }
  for (const [key, f] of Object.entries(state.axes)) {
    const row = $(`axis-row-${key}`);
    if (!row) continue;
    const active = f.active === "1";
    const remaining = Number(f.remaining);
    const verdict = active ? ["运行中", "act"] : (remaining ? ["已停止", "stop"] : ["空闲 / 完成", "done"]);
    row.children[2].textContent = f.pos;
    row.children[3].textContent = f.sps;
    row.children[4].textContent = f.remaining;
    row.children[5].textContent = verdict[0];
    row.children[5].className = verdict[1];
    row.children[6].textContent = f.move_id;
  }
}

function renderLeds() {
  $("lamp-green").classList.toggle("on", !!(state.ledMask & 1));
  $("lamp-blue").classList.toggle("on", !!(state.ledMask & 2));
}

/* ── servo gauges ──────────────────────────────────────────────────────── */

const GAUGE = { cx: 130, cy: 140, r: 96, start: 210, sweep: 270 };

function pointAt(degrees, radius = GAUGE.r) {
  const radians = (degrees * Math.PI) / 180;
  return [GAUGE.cx + radius * Math.cos(radians), GAUGE.cy - radius * Math.sin(radians)];
}

function arcPath(from, to, radius = GAUGE.r) {
  const [x1, y1] = pointAt(from, radius);
  const [x2, y2] = pointAt(to, radius);
  const large = Math.abs(to - from) > 180 ? 1 : 0;
  return `M ${x1.toFixed(2)} ${y1.toFixed(2)} A ${radius} ${radius} 0 ${large} 0 ${x2.toFixed(2)} ${y2.toFixed(2)}`;
}

/* The firmware answers SERVO STAT with one frame per channel in a single body,
   so each channel is read out of the raw reply instead of the flat field map
   (which keeps only the last duplicate key). */
const SERVO_FRAME = /ch=(\d+) angle=([\d.]+) us=(\d+) on=(\d)/g;

function parseServoFrames(raw) {
  const frames = {};
  if (!raw) return frames;
  SERVO_FRAME.lastIndex = 0;
  let match = SERVO_FRAME.exec(raw);
  while (match) {
    frames[Number(match[1])] = { channel: Number(match[1]), angle: match[2], us: Number(match[3]), on: match[4] };
    match = SERVO_FRAME.exec(raw);
  }
  return frames;
}

function servoCards() {
  return [...document.querySelectorAll(".servo-card")];
}

function channelOf(card) {
  return Number(card.dataset.ch);
}

/* UI S1 moves to PH9/180 degrees; UI S2 stays on PE4/270 degrees.
   Firmware channel IDs differ from UI IDs; route commands and readback by pin. */
const SERVO_LAYOUT = {
  1: { pin: "PH9", timer: "TIM12_CH2", travel: 180, home: 120, wireTravel: 180, wire: 2 },
  2: { pin: "PE4", timer: "TIM15_CH1N", travel: 270, home: 0, wireTravel: 270, wire: 1 },
};

function servoProtocolChannel(channel) {
  // PH9 belonged to the 270-degree channel on firmware predating PE4.
  const hasPE4 = [1, 2].some(wire => state.device[`servo${wire}_pin`] === "PE4");
  const pins = hasPE4 ? (channel === 1 ? ["PH9", "PH6"] : ["PE4"]) : (channel === 1 ? ["PH6"] : ["PH9"]);
  return [1, 2].find(wire => pins.includes(state.device[`servo${wire}_pin`])) || SERVO_LAYOUT[channel].wire;
}

/* User-confirmed physical travel is independent of firmware angle units. */
function servoTravel(channel) {
  return SERVO_LAYOUT[channel]?.travel || state.travel || 270;
}

/* Convert using the resolved wire channel, with firmware half-degree rounding. */
function servoWireAngle(channel, degrees) {
  const wire = servoProtocolChannel(channel);
  const travel = Number(state.device[`servo${wire}_travel`]) / 10 || SERVO_LAYOUT[channel]?.wireTravel || state.travel;
  return String(Math.round(Number(degrees) * travel / servoTravel(channel) * 2) / 2);
}

/* Convert calibration input using that channel's positional mapping only. */
function pulseToAngle(channel, us) {
  return (((us - 500) * servoTravel(channel)) / 2000).toFixed(1);
}

/* Draw (or redraw) one gauge: called on boot, on every theme switch, and when
   the travel limit changes. Kept separate from the data pass so a theme switch
   never needs new telemetry. */
function drawGauge(card) {
  const travel = servoTravel(channelOf(card));
  card.dataset.travel = String(travel);
  const svg = card.querySelector(".gauge");
  const track = svg.querySelector(".arc-track");
  const live = svg.querySelector(".arc-live");
  const path = arcPath(GAUGE.start, GAUGE.start - GAUGE.sweep);
  track.setAttribute("d", path);
  if (live.getAttribute("d") !== path) live.setAttribute("d", path);

  const ticks = svg.querySelector(".ticks");
  ticks.innerHTML = "";
  for (let step = 0; step <= 6; step += 1) {
    const deg = (travel / 6) * step;
    const angle = GAUGE.start - (deg / travel) * GAUGE.sweep;
    const [x1, y1] = pointAt(angle, GAUGE.r - 11);
    const [x2, y2] = pointAt(angle, GAUGE.r - 18);
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttribute("class", "tick");
    line.setAttribute("x1", x1.toFixed(1));
    line.setAttribute("y1", y1.toFixed(1));
    line.setAttribute("x2", x2.toFixed(1));
    line.setAttribute("y2", y2.toFixed(1));
    ticks.append(line);
    if (step % 3 === 0) {
      const [lx, ly] = pointAt(angle, GAUGE.r - 33);
      const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
      label.setAttribute("class", "tick-label");
      label.setAttribute("x", lx.toFixed(1));
      label.setAttribute("y", (ly + 3).toFixed(1));
      label.textContent = `${Math.round(deg)}°`;
      ticks.append(label);
    }
  }
  const frame = state.servoFrames[servoProtocolChannel(channelOf(card))];
  renderServoCard(card, frame);
}

function renderServoCard(card, frame) {
  const travel = servoTravel(channelOf(card));
  card.querySelector("h2").textContent = `舵机 S${channelOf(card)} · ${travel}°`;
  const svg = card.querySelector(".gauge");
  const value = frame ? Number(pulseToAngle(channelOf(card), frame.us)) : null;
  const shown = value === null || Number.isNaN(value) ? "—" : value.toFixed(1);

  card.querySelector(".angle").textContent = shown;
  card.querySelector(".us").textContent = frame ? String(frame.us) : "—";
  card.querySelector(".servo-on").textContent = state.link !== "connected"
    ? "未连接" : (frame ? (frame.on === "1" ? "输出中" : "未确认") : "未知");
  svg.querySelector(".gauge-value").textContent = shown === "—" ? "—" : `${shown}°`;
  if (shown === "—") return;

  const fraction = Math.max(0, Math.min(1, value / travel));
  svg.querySelector(".arc-live").style.strokeDasharray =
    `${(fraction * Math.PI * GAUGE.r * (GAUGE.sweep / 180)).toFixed(1)} ${(2 * Math.PI * GAUGE.r).toFixed(1)}`;
  const angle = GAUGE.start - fraction * GAUGE.sweep;
  svg.querySelector(".needle").style.transform = `rotate(${(angle + 90).toFixed(2)}deg)`;
}

function renderServo() {
  const cards = servoCards();
  cards.forEach((card, index) => {
    const channel = channelOf(card);
    const wire = servoProtocolChannel(channel);
    const wired = wire <= (state.servoChannels || cards.length);
    card.classList.toggle("hidden", !wired);
    if (wired) {
      const layout = SERVO_LAYOUT[channel];
      const pin = layout?.pin;
      card.querySelector('.tag').textContent = `${pin} · ${layout?.timer || ""}`;
      if (Number(card.dataset.travel) !== servoTravel(channel)) drawGauge(card);
      else renderServoCard(card, state.servoFrames[wire]);
    }
  });
  const sendable = state.link === "connected" && !manualReadonly();
  $("pulse-send").disabled = !sendable;
  $("servo-diagnose").disabled = !sendable;
  cards.forEach((card) => {
    card.querySelector(".servo-send").disabled = !sendable;
    card.querySelectorAll("[data-deg]").forEach((button) => {
      button.disabled = !sendable || Number(button.dataset.deg) > servoTravel(channelOf(card));
    });
  });
  $("pulse-ch").max = String(state.servoChannels || 1);
}

/* Angle text: whole degrees or one decimal, exactly what the firmware accepts. */
const ANGLE_RE = /^\d{1,3}(\.\d)?$/;

/* Read both physical outputs twice to inspect live registers/CNT in the log.
   Queries only: never set angles, reconnect, arm, or take another page's role. */
async function diagnoseServo() {
  if (state.link !== "connected" || manualReadonly()) return;
  for (let sample = 0; sample < 2; sample += 1) {
    for (const channel of [1, 2]) {
      if (!(await command(`SERVO DIAG ${channel}`)).ok) return;
    }
    if (sample === 0) await new Promise(resolve => setTimeout(resolve, 130));
  }
  status("已请求两路 PWM 诊断，请在通信记录中查看 SERVO_DIAG 回复。");
}

async function sendAngle(channel, text) {
  const value = String(text).trim();
  if (!ANGLE_RE.test(value)) {
    status("角度格式：整度（135）或一位小数（13.5）。", "warn");
    return;
  }
  const travel = servoTravel(channel);
  const wire = servoProtocolChannel(channel);
  if (Number(value) > travel) {
    status(`角度超出 ${travel}° 行程，板端会拒绝。`, "warn");
    return;
  }
  const result = await command(`SERVO ${wire} ${servoWireAngle(channel, value)}`);
  if (result.ok) command("SERVO STAT");
}

/* ── chassis ───────────────────────────────────────────────────────────── */

const DIRECTIONS = {
  FWD:   { label: "前进", wheels: [1, 1, 1, 1] },
  BACK:  { label: "后退", wheels: [-1, -1, -1, -1] },
  LEFT:  { label: "左移", wheels: [1, -1, -1, 1] },
  RIGHT: { label: "右移", wheels: [-1, 1, 1, -1] },
  CCW:   { label: "左旋", wheels: [-1, 1, -1, 1] },
  CW:    { label: "右旋", wheels: [1, -1, 1, -1] },
};

function renderWheels(direction) {
  const spec = DIRECTIONS[direction];
  document.querySelectorAll("#wheelview .wheel").forEach((node, index) => {
    node.classList.remove("fwd", "back");
    const label = node.querySelector(".dir");
    if (!spec) { label.textContent = "—"; return; }
    const sign = spec.wheels[Number(node.dataset.wheel) - 1];
    node.classList.add(sign > 0 ? "fwd" : "back");
    label.textContent = sign > 0 ? "+" : "−";
  });
}

function syncPad() {
  document.querySelectorAll(".pad").forEach((node) => {
    const direction = node.dataset.dir;
    node.classList.toggle("live", !!state.jogBody && state.jogBody.includes(` ${direction} `));
    node.classList.toggle("selected", state.selected === direction);
  });
}

function renderChassis() {
  renderTurn();
  const f = state.chassis || {};
  const known = f.state !== undefined;
  const moving = ["moving", "jogging", "switching"].includes(f.state);
  $("chassis-state").textContent = !known ? "—"
    : moving ? `${DIRECTIONS[f.direction]?.label || f.direction} · 剩余 ${f.remaining}`
      : f.state === "done" ? "完成" : f.state === "stopped" ? "已停止" : "空闲";
  if (f.state === "fault") $("chassis-state").textContent = `直线已停车 · ${f.fault}`;
  const assisted = $("ch-assist").checked;
  const assistSupported = state.device.straight === "1";
  const assistReady = state.turn.imu === "1";
  $("ch-assist").disabled = manualReadonly();
  $("ch-assist-state").textContent = !assistSupported ? "辅助直线需要 H743 0.2.17 固件" :
    f.assist === "1" ? `锁定航向 ${(Number(f.target_cd)/100).toFixed(2)}° · 偏差 ${(Number(f.error_cd)/100).toFixed(2)}°` :
      assistReady ? "航向可用 · 起步时锁定当前方向" : "等待有效 H723 航向";
  if (f.assist === "1" && f.fault !== "none") $("ch-assist-state").textContent += ` · ${f.fault}`;
  const busy = moving || !!state.jogBody || state.turn.state === "turning" ||
    ["turning","moving","orienting","waiting","nudging","blocked","checkpoint"].includes(state.navigation.state);
  $("ch-move").disabled = manualReadonly() || state.link !== "connected" || !state.armed || busy ||
    (assisted && (!assistSupported || !assistReady || ["CW","CCW"].includes(state.selected)));
  if (f.invert !== undefined && f.invert !== null) {
    const confirmed = Number(f.invert);
    if (state.invertDraft === confirmed) state.invertDraft = null;
    if (state.invertDraft === null) {
      document.querySelectorAll(".inv").forEach((box) => {
        box.checked = !!(confirmed & (1 << Number(box.value)));
      });
    }
    $("inv-state").textContent = `板端掩码 ${confirmed}` + (state.invertDraft === null ? "" : ` · 待应用 ${state.invertDraft}`);
  } else {
    $("inv-state").textContent = "等待板端方向配置" + (state.invertDraft === null ? "" : ` · 待应用 ${state.invertDraft}`);
  }
  if (state.jogBody) renderWheels(state.jogBody.split(" ")[2]);
  else if (moving) renderWheels(f.direction);
  else if (state.selected) renderWheels(state.selected);
  else renderWheels(null);
  syncPad();
}

function chassisTick(f) {
  state.chassis = f;
  if (f.state === "fault") { state.jogBody = null; syncPad(); }
  renderChassis();
  renderRecovery();
}

function chassisMove() {
  const direction = state.selected;
  if (!direction) { status("先点选一个方向，再执行位移。", "warn"); return; }
  const steps = num($("ch-steps").value);
  const sps = num($("ch-sps").value);
  const accel = num($("ch-accel").value);
  if (!(steps > 0 && steps <= 10000000)) { status("步数范围 1–10000000。", "warn"); return; }
  if (!(sps >= 1 && sps <= 5000)) { status("速度范围 1–5000 step/s。", "warn"); return; }
  if (!(accel >= 1 && accel <= 20000)) { status("加速度范围 1–20000 step/s²。", "warn"); return; }
  if (![steps,sps,accel].every(Number.isInteger)) { status("步数、速度和加速度必须为整数。", "warn"); return; }
  if ($("ch-assist").checked) {
    if (state.device.straight !== '1') { status("辅助直线需要 H743 0.2.17 固件。", "warn"); return; }
    if (["CW","CCW"].includes(direction)) { status("辅助直线支持前后左右平移。原地旋转请取消勾选。", "warn"); return; }
    command(`CHASSIS STRAIGHT ${direction} ${steps} ${sps} ${accel}`);
  } else command(`CHASSIS MOVE ${direction} ${steps} ${sps} ${accel}`);
}

/* ── log ───────────────────────────────────────────────────────────────── */
/* Host edits are saved for the next mission; live position comes from the MCU. */
function adoptMission(config) {
  if (!config) return;
  state.mission = config;
  renderStations(config);
  renderRoute(config);
  $("nav-sps").value = config.sps;
  $("nav-accel").value = config.accel;
  $("nav-dwell").value = config.dwell_ms / 1000;
  $("nav-inset-enabled").checked = !!config.turn_inset_enabled;
  $("nav-inset-steps").value = config.turn_inset_steps ?? 1000;
  renderStartStation(config);
  for (const axis of ["x", "y"]) config.grid[axis].forEach((value, index) => { $(`nav-grid-${axis}${index}`).value = value; });
  syncFieldEditor(config.grid);
  $("nav-config-state").textContent = "参数已保存 · 田字格通道 · 下次巡航生效";
  renderNavigation();
}

/* Station rows carry their own name, so the table is the source of both the
   coordinates and the order the route indices refer to. */
function renderStations(config) {
  const body = document.querySelector("#nav-points tbody");
  body.innerHTML = "";
  config.stations.forEach((station, index) => {
    const row = document.createElement("tr");
    row.dataset.station = String(index);
    const name = document.createElement("td");
    const field = document.createElement("input");
    field.type = "text";
    field.maxLength = 8;
    field.value = station.name;
    field.setAttribute("aria-label", `站点 ${index + 1} 名称`);
    /* The origin is established by a physical RESET, so its name may change
       but its row may not be reordered away from index 0. */
    name.appendChild(field);
    row.appendChild(name);
    for (const [key, label] of [["x", "x"], ["y", "y"], ["heading", "到达姿态"]]) {
      const cell = document.createElement("td");
      const input = document.createElement("input");
      input.type = "number";
      if (key === "heading") { input.min = "-180"; input.max = "180"; input.step = "0.01"; }
      if (key === "heading" && index === 0) input.readOnly = true;
      input.value = station[key];
      input.setAttribute("aria-label", `${station.name} ${label}`);
      cell.appendChild(input);
      row.appendChild(cell);
    }
    const modeCell = document.createElement("td");
    const enabled = document.createElement("input"); enabled.type = "checkbox";
    enabled.className = "nav-arrival-enabled";
    enabled.checked = station.arrival_enabled ?? false;
    enabled.setAttribute("aria-label", `${station.name} 到点调姿`);
    modeCell.appendChild(enabled); row.appendChild(modeCell);
    body.appendChild(row);
  });
}

/* Starting pose is explicit operator input; this never resets or moves MCU. */
function renderStartStation(config) {
  const select = $("nav-start-station");
  select.replaceChildren();
  config.stations.forEach((station, index) => {
    if (index !== 0 && station.name !== 'X') return;
    const option = document.createElement('option');
    option.value = String(index); option.textContent = `${station.name} (${station.x}, ${station.y}) · ${station.heading}°`;
    select.appendChild(option);
  });
  select.value = String(config.start_station ?? 0);
  $("nav-origin").textContent = `当前位置设为 ${config.stations[Number(select.value)]?.name || 'O'}`;
}

function renderRoute(config) {
  const body = document.querySelector("#nav-route tbody");
  body.innerHTML = "";
  config.route.forEach((leg, index) => {
    const row = document.createElement("tr");
    row.dataset.leg = String(index);
    const order = document.createElement("td");
    order.textContent = String(index + 1);
    row.appendChild(order);
    const pick = document.createElement("td");
    const select = document.createElement("select");
    select.setAttribute("aria-label", `第 ${index + 1} 段目标站点`);
    config.stations.forEach((station, position) => {
      const option = document.createElement("option");
      option.value = String(position);
      option.textContent = station.name;
      if (position === leg.station) option.selected = true;
      select.appendChild(option);
    });
    pick.appendChild(select);
    row.appendChild(pick);
    const arrivalCell = document.createElement("td");
    const arrivalMode = document.createElement("select"); arrivalMode.className = "nav-arrival-mode";
    arrivalMode.setAttribute("aria-label", `第 ${index + 1} 段到点调姿`);
    [["inherit","跟随站点"],["off","不调姿"],["on","调到指定角度"]].forEach(([value,label]) => {
      const option = document.createElement("option"); option.value = value; option.textContent = label;
      option.selected = value === (leg.arrival_enabled === undefined ? "inherit" : leg.arrival_enabled ? "on" : "off");
      arrivalMode.appendChild(option);
    });
    const arrivalAngle = document.createElement("input"); arrivalAngle.type = "number";
    arrivalAngle.className = "nav-arrival-angle"; arrivalAngle.min = "-180"; arrivalAngle.max = "180"; arrivalAngle.step = "0.01";
    arrivalAngle.value = leg.heading ?? ""; arrivalAngle.placeholder = "留空使用站点角度";
    arrivalAngle.setAttribute("aria-label", `第 ${index + 1} 段到达角度`);
    arrivalCell.appendChild(arrivalMode); arrivalCell.appendChild(arrivalAngle); row.appendChild(arrivalCell);
    const holdCell = document.createElement("td");
    const hold = document.createElement("input");
    hold.type = "checkbox";
    hold.checked = Number(leg.hold) === 1;
    hold.setAttribute("aria-label", `第 ${index + 1} 段到点挂起`);
    holdCell.appendChild(hold);
    const task = document.createElement("select");
    task.className = "nav-subtask";
    task.setAttribute("aria-label", `第 ${index + 1} 段视觉子任务`);
    [[0,"手动放行 / 已配置动作"],[4,"A点：读取任务码并切换相机"],[1,"子任务1：圆列 1→2→3→2"],[2,"子任务2：预留动作"],[3,"子任务3：预留动作"]].forEach(([value,label]) => {
      const option = document.createElement("option"); option.value = String(value); option.textContent = label;
      option.selected = value === Number(leg.task || 0); task.appendChild(option);
    });
    holdCell.appendChild(task);
    const parameters = document.createElement("div");
    parameters.className = "nav-subtask-parameters";
    for (const [key,label,fallback] of [["outward_steps","补搜外移脉冲",400],["right_steps","完成右移脉冲",800]]) {
      const wrapper = document.createElement("label"); wrapper.textContent = label;
      const input = document.createElement("input"); input.type = "number"; input.min = "0"; input.max = "100000";
      input.dataset.parameter = key; input.value = String(leg[key] ?? fallback);
      input.setAttribute("aria-label", `第 ${index + 1} 段${label}`); wrapper.appendChild(input); parameters.appendChild(wrapper);
    }
    holdCell.appendChild(parameters);
    const refresh = () => { task.disabled = !hold.checked; parameters.hidden = !hold.checked || task.value !== "1"; };
    hold.addEventListener("change", refresh); task.addEventListener("change", refresh); refresh();
    row.appendChild(holdCell);
    body.appendChild(row);
  });
  const names = config.stations.map((s) => s.name);
  $("nav-route-tag").textContent = [names[config.start_station ?? 0] || "?"]
    .concat(config.route.map((leg) => names[leg.station] || "?")).join(" → ");
}

/* Convert field dimensions into the existing six absolute lane coordinates.
   No units are inferred: all values remain integer wheel pulses. */
function fieldGrid({x, y, width, height, middleX, middleY}) {
  const grid = {x: [x, x + middleX, x + width], y: [y, y + middleY, y + height]};
  if (![x, y, width, height, middleX, middleY].every(Number.isSafeInteger) ||
      !validGrid(grid)) throw new Error("宽、高须为正整数，中间通道须在两边界之间；绝对坐标范围 ±1000000 脉冲。");
  return grid;
}

/* Used by the field editor; backend validation remains authoritative. */
function validGrid(grid) {
  return ["x", "y"].every(axis => grid[axis].length === 3 && grid[axis].every(value => Number.isSafeInteger(value) && Math.abs(value) <= 1000000) &&
    grid[axis][0] < grid[axis][1] && grid[axis][1] < grid[axis][2]);
}

/* Remap each lane interval separately, preserving lane membership and route
   indices. This edits coordinates only; it never moves or resets the vehicle. */
function remapField(config, grid, remap) {
  if (!validGrid(config.grid) || !validGrid(grid)) throw new Error("请先填写有效的通道坐标。");
  const draft = structuredClone(config);
  if (remap) draft.stations.forEach(station => {
    for (const axis of ["x", "y"]) {
      const before = config.grid[axis], after = grid[axis], value = station[axis];
      const lane = before.indexOf(value);
      if (lane >= 0) station[axis] = after[lane];
      else {
        const i = value < before[1] ? 0 : 1;
        station[axis] = Math.round(after[i] + (value - before[i]) * (after[i + 1] - after[i]) / (before[i + 1] - before[i]));
      }
    }
  });
  draft.grid = structuredClone(grid);
  return draft;
}

/* Quick editor is refreshed only by loading/applying a grid, never by polling,
   so half-typed dimensions survive telemetry updates. */
function syncFieldEditor(grid) {
  state.fieldDirty = false;
  const values = {x: grid.x[0], y: grid.y[0], width: grid.x[2] - grid.x[0], height: grid.y[2] - grid.y[0],
    "middle-x": grid.x[1] - grid.x[0], "middle-y": grid.y[1] - grid.y[0]};
  for (const [key, value] of Object.entries(values)) $(`nav-field-${key}`).value = value;
  renderFieldSummary(grid);
}

/* Width is the difference between boundaries, including for negative origins. */
function renderFieldSummary(grid = state.mission?.grid) {
  if (!grid) return;
  const width = grid.x[2] - grid.x[0], height = grid.y[2] - grid.y[0];
  const mm = mmPerPulse() ? ` · 约 ${formatMm(width)} × ${formatMm(height)}` : "";
  $("nav-field-summary").textContent = `X：${grid.x[0]}～${grid.x[2]} · Y：${grid.y[0]}～${grid.y[2]} · 宽 × 高：${width} × ${height} 脉冲${mm}`;
}

/* Editing never plans a route or rewrites half-typed station coordinates. */
function navigationChanged() {
  $("nav-config-state").textContent = "参数已修改 · 尚未保存";
}

/* Width/height edits are consumed by 保存参数 without a separate apply step. */
function readFieldGrid() {
  const value = key => { const input = $(`nav-field-${key}`); return input.value.trim() ? Number(input.value) : NaN; };
  return fieldGrid({x: value("x"), y: value("y"), width: value("width"), height: value("height"), middleX: value("middle-x"), middleY: value("middle-y")});
}

/* Structural edits consume a pending field edit once, avoiding double remaps. */
function writeMissionGrid(grid) {
  for (const axis of ["x", "y"]) grid[axis].forEach((value, index) => { $(`nav-grid-${axis}${index}`).value = value; });
  syncFieldEditor(grid);
}

function readMission() {
  const stations = Array.from(document.querySelectorAll("#nav-points tbody tr"), (row) => {
    const inputs = row.querySelectorAll("input");
    const value = (input) => input.value.trim() === "" ? NaN : Number(input.value);
    return { name: inputs[0].value.trim(), x: value(inputs[1]), y: value(inputs[2]),
             heading: value(inputs[3]), arrival_enabled: row.querySelector(".nav-arrival-enabled")?.checked ?? false };
  });
  const route = Array.from(document.querySelectorAll("#nav-route tbody tr"), (row) => {
    const leg = {station: Number(row.querySelector("select").value), hold: row.querySelector("input[type=checkbox]").checked ? 1 : 0};
    const mode = row.querySelector(".nav-arrival-mode")?.value;
    if (mode && mode !== "inherit") leg.arrival_enabled = mode === "on";
    const angle = row.querySelector(".nav-arrival-angle")?.value;
    if (angle !== undefined && angle.trim() !== "") {
      leg.heading = Number(angle);
      if (!Number.isFinite(leg.heading) || Math.abs(leg.heading) > 180 || Math.abs(leg.heading*100-Math.round(leg.heading*100)) > 1e-7) throw new Error("路线段到达角度为 -180～180°，最多两位小数。");
    }
    const task = row.querySelector(".nav-subtask");
    if (task && leg.hold && Number(task.value)) {
      leg.task = Number(task.value);
      if (leg.task === 1) for (const key of ["outward_steps","right_steps"]) {
        const input = row.querySelector(`[data-parameter="${key}"]`);
        leg[key] = input.value.trim() === "" ? NaN : Number(input.value);
        if (!Number.isSafeInteger(leg[key]) || leg[key] < 0 || leg[key] > 100000) throw new Error("子任务位移必须为 0～100000 的整数脉冲。");
      }
    }
    return leg;
  });
  const points = Object.fromEntries(stations.map((s) => [s.name, [s.x, s.y]]));
  const headings = Object.fromEntries(stations.map((s) => [s.name, s.heading]));
  const grid = {};
  for (const axis of ["x", "y"]) grid[axis] = [0, 1, 2].map(index => {
    const input = $(`nav-grid-${axis}${index}`);
    return input.value.trim() === "" ? NaN : Number(input.value);
  });
  const config = { stations, route, grid, start_station: Number($("nav-start-station")?.value || 0), sps: Number($("nav-sps").value), accel: Number($("nav-accel").value),
    dwell_ms: Number($("nav-dwell").value) * 1000,
    turn_inset_enabled: $("nav-inset-enabled").checked,
    turn_inset_steps: $("nav-inset-steps").value.trim() === "" ? NaN : Number($("nav-inset-steps").value) };
  if (!Number.isSafeInteger(config.turn_inset_steps) || config.turn_inset_steps < 1 || config.turn_inset_steps > 100000) throw new Error("转前内移范围 1～100000 整数脉冲。");
  if (stations.length < 2) throw new Error("至少需要两个站点。");
  if (!route.length) throw new Error("路线至少需要一段。");
  if (stations.some((s) => !/^[A-Za-z0-9_]{1,8}$/.test(s.name))) throw new Error("站点名为 1～8 个字母、数字或下划线。");
  if (new Set(stations.map((s) => s.name)).size !== stations.length) throw new Error("站点名不能重复。");
  const integers = [...Object.values(points).flat(), ...Object.values(grid).flat(), config.sps, config.accel, config.dwell_ms];
  if (integers.some((value) => !Number.isSafeInteger(value))) throw new Error("坐标和运动参数必须为有效整数；停留精确到毫秒。");
  if (Object.values(headings).some((value) => !Number.isFinite(value) || Math.abs(value) > 180 || Math.abs(value * 100 - Math.round(value * 100)) > 1e-7)) throw new Error("到达姿态为 -180～180°，最多两位小数。");
  return state.fieldDirty ? remapField(config, readFieldGrid(), $("nav-field-remap").checked) : config;
}

async function saveMission() {
  try {
    const result = await post("/api/navigation/config", { config: readMission() });
    if (result.ok) adoptMission(result.config);
    return result.ok;
  } catch (error) {
    status(error.message, "warn");
    return false;
  }
}

function renderNavigation() {
  renderTurn();
  const f = state.navigation || {};
  const names = { checkpoint: "等待 NX 下令", idle: "就绪", turning: "转向并确认", moving: "向前行驶", nudging: "视觉微调", orienting: "调整到达姿态", waiting: "到点停留", suspended: "底盘已交给任务控制", done: "巡航完成", stopped: "已停止", blocked: "障碍暂停", fault: "巡航故障" };
  const writable = !manualReadonly();
  const connected = state.link === "connected" && state.device.nav_heading === "1" && state.device.nav_grid === "1" && state.device.nav_path === "1";
  const busy = ["turning", "moving", "orienting", "waiting", "nudging", "blocked", "suspended", "checkpoint"].includes(f.state) || state.turn.state === "turning";
  $("nav-state").textContent = state.link === "connected" && state.device.nav_path !== "1" ? "请更新 H743 至 0.2.14" : !connected ? "未连接或固件不支持" : f.ready !== "1" ? "待建立所选起点" : names[f.state] || "待查询";
  if (state.navigationRunner?.active) {
    const phases = { inward:"向中心内移", turn:"转向", return:"返回原通道", settle:"回位后纠偏", correct:"航向纠偏", travel:"沿路线行驶", arrival:"到点确认", leave:"离开站点" };
    $("nav-state").textContent += ` · NX：${phases[state.navigationRunner.phase] || state.navigationRunner.state}`;
  }
  $("nav-position").textContent = f.ready === "1" ? `(${f.x}, ${f.y})` : "—";
  /* The board reports a station index, not a name: names live only in the
     host config. Leg count comes from the board so the page cannot disagree
     with the route the board actually committed. */
  const stations = (state.mission && state.mission.stations) || [];
  const station = stations[Number(f.station)];
  const label = station ? station.name : f.station;
  const legs = Number(f.legs) > 0 ? f.legs : stations.length ? String(stations.length) : "?";
  $("nav-target").textContent = f.station != null && f.ready === "1" ?
    `${label} (${f.tx}, ${f.ty}) · ${f.leg}/${legs}${f.held === "1" ? " · 等待放行" : ""}` : "—";
  $("nav-waypoint").textContent = f.planner === "host_grid" && Number(f.segments) > 0 ? `(${f.wx}, ${f.wy}) · 小段 ${f.segment}/${f.segments}` : "—";
  $("nav-wait").textContent = f.state === "waiting" ? `${(Number(f.wait_ms) / 1000).toFixed(1)} s` : "—";
  $("nav-heading").textContent = f.imu === "1" ? `${(Number(f.heading_cd) / 100).toFixed(2)}° → ${(Number(f.target_cd) / 100).toFixed(2)}°` : "等待有效 H723 航向";
  $("nav-assist-state").textContent = state.device.straight === '1' ?
    '每段直线自动进行陀螺仪纠偏 · 使用共享 P 与角度容差' : '巡航直线纠偏需要 H743 0.2.17 固件';
  $("nav-arrival").textContent = f.arrival_cd != null ? `${(Number(f.arrival_cd) / 100).toFixed(2)}°` : "—";
  const faults = {off_grid:"当前位置或任务点不在通道线上", grid_segment:"通道小段无效", imu_stale:"航向数据过期", imu_invalid:"航向数据失效", imu_changed:"H723 重连或重启", turn_timeout:"转向超时", heading_deviation:"行驶航向偏差超过 5°", move_start:"前进启动失败", turn_start:"转向启动失败", turn_update:"转向更新失败", move_interrupted:"前进被中断"};
  $("nav-fault").textContent = faults[f.fault] || (f.fault && f.fault !== "none" ? f.fault : "—");
  $("nav-save").disabled = !writable;
  $("nav-origin").disabled = !writable || !connected || state.armed || busy;
  $("nav-start").disabled = !writable || !connected || !state.armed || f.ready !== "1" || busy;
  $("nav-stop").disabled = !writable || state.link !== "connected" || state.device.nav_heading !== "1";
  document.querySelectorAll("#tab-navigation input, #nav-route select, #nav-start-station").forEach((input) => { input.disabled = !writable; });
  $("nav-add-x").disabled = !writable;
  for (const id of ["nav-station-add", "nav-station-remove", "nav-leg-add", "nav-leg-remove"]) $(id).disabled = !writable;
  renderTask();
}

/* Standalone task status uses its own origin and never edits route parameters.
   MCU status controls completion; page input stays editable between polls. */
function renderTurn() {
  const f = state.turn || {};
  const connected = state.link === "connected" && state.device.turn === "1";
  const writable = !manualReadonly();
  const busy = f.state === "turning" || ["turning", "moving", "orienting", "waiting", "nudging", "blocked", "suspended", "checkpoint"].includes(state.navigation.state) ||
    ["moving", "jogging", "switching"].includes(state.chassis.state) || !!state.jogBody;
  const labels = {idle:"空闲", turning:"闭环转动中", done:"转角完成", stopped:"已停止", fault:"转角故障"};
  const faults = {imu_stale:"航向数据过期", imu_invalid:"航向数据失效", imu_changed:"H723 会话变化", turn_timeout:"转向超时", turn_start:"启动失败", turn_update:"更新失败"};
  $("turn-state").textContent = connected ? labels[f.state] || "待查询" : "未连接或固件不支持";
  $("turn-zero-state").textContent = f.zero === "1" ? "零点已设置" : "零点未设置";
  $("turn-live").textContent = f.imu === "1" ? `本次基准角 ${(Number(f.current_cd)/100).toFixed(2)}° · 目标 ${(Number(f.target_cd)/100).toFixed(2)}° · 误差 ${(Number(f.error_cd)/100).toFixed(2)}°` : "等待有效 H723 航向";
  if (f.fault && f.fault !== "none") $("turn-live").textContent += ` · ${faults[f.fault] || f.fault}`;
  if (f.kp_centi != null) $("turn-live").textContent += ` · 板端 P=${Number(f.kp_centi)/100} · 角度容差 ±${(Number(f.tolerance_cd)/100).toFixed(2)}°`;
  if (f.min_sps != null) $("turn-live").textContent += ` · 最低转向 ${f.min_sps} step/s`;
  $("turn-zero").disabled = !connected || !writable || busy || f.imu !== "1";
  $("turn-start").disabled = !connected || !writable || !state.armed || busy || f.imu !== "1" ||
    ($("turn-mode").value === "ABS" && f.zero !== "1");
  $("turn-stop").disabled = !connected || !writable;
  for (const prefix of ['turn','nav-turn','imu-turn']) {
    const apply = $(`${prefix}-configure`);
    if (!apply) continue;
    apply.disabled = !connected || !writable || busy || state.device.turn_precision !== '1' || f.min_sps == null;
    for (const key of ['kp','tolerance','min-sps']) $(`${prefix}-${key}`).disabled = !writable;
    const live = $(`${prefix}-live`);
    if (prefix !== 'turn' && live) {
      const supported = state.device.turn_precision === '1' && f.min_sps != null;
      const pending = Number($(`${prefix}-kp`).value)*100 !== Number(f.kp_centi) || Number($(`${prefix}-tolerance`).value)*100 !== Number(f.tolerance_cd) || Number($(`${prefix}-min-sps`).value) !== Number(f.min_sps);
      live.textContent = !supported ? '最低转向轮速需要新版固件及参数回读。' :
        `板端 P=${Number(f.kp_centi)/100} · 角度容差 ±${(Number(f.tolerance_cd)/100).toFixed(2)}° · 最低转向 ${f.min_sps} step/s · ${pending ? '输入尚未应用' : '输入已生效'}`;
    }
  }
  for (const id of ["turn-mode", "turn-angle", "turn-sps", "turn-accel", "turn-kp"]) $(id).disabled = !writable;
}

/* Validate UI degrees before HTTP submission; server and MCU validate again. */
async function turnAction(action, tuningPrefix = "turn") {
  const payload = {action};
  if (action === "configure") {
    const text = $(`${tuningPrefix}-kp`).value.trim(), kp = Number(text);
    if (!/^\d+(\.\d{1,2})?$/.test(text) || !Number.isFinite(kp) || kp < 0) {
      status("P 必须是非负数，最多两位小数。", "warn"); return;
    }
    if (kp > 21474836.47) {
      status("P 过大，超出协议整数容量（P×100 不得超过 2147483647）。", "warn"); return;
    }
    const tolerance = $(`${tuningPrefix}-tolerance`).value.trim();
    if (!/^\d+(\.\d{1,2})?$/.test(tolerance) || Number(tolerance) < .01 || Number(tolerance) > 10) {
      status('角度容差范围 0.01～10°，最多两位小数。', 'warn'); return;
    }
    payload.kp = text;
    payload.tolerance = tolerance;
    const minimum = $(`${tuningPrefix}-min-sps`).value.trim();
    if (!/^\d+$/.test(minimum) || Number(minimum) < 1 || Number(minimum) > 5000) {
      status('最低转向轮速范围 1～5000 step/s，必须是整数。', 'warn'); return;
    }
    payload.min_sps = Number(minimum);
  }
  if (action === "start") {
    const text = $("turn-angle").value.trim();
    const spsText = $("turn-sps").value.trim(), accelText = $("turn-accel").value.trim();
    if (!/^-?\d+(\.\d{1,2})?$/.test(text) || Math.abs(Number(text)) > 180) {
      status("角度范围 -180～180°，最多两位小数。", "warn"); return;
    }
    const sps = Number(spsText), accel = Number(accelText);
    if (!/^\d+$/.test(spsText) || !/^\d+$/.test(accelText) || !Number.isSafeInteger(sps) || sps < 1 || sps > 5000 ||
        !Number.isSafeInteger(accel) || accel < 1 || accel > 20000) {
      status("最高轮速范围 1～5000，加速度范围 1～20000。", "warn"); return;
    }
    Object.assign(payload, {mode:$("turn-mode").value, angle:text, sps, accel});
  }
  return post("/api/turn/action", payload);
}


function logVisible(entry) {
  return state.showPolls || !QUIET_REPLY.test(entry.text);
}

function pushLog(entry) {
  if (!entry || state.seenLog.has(entry.seq)) return;
  state.seenLog.add(entry.seq);
  state.logs.push(entry);
  if (state.logs.length > 400) {
    state.logs.splice(0, state.logs.length - 400).forEach((item) => state.seenLog.delete(item.seq));
  }
  if (logVisible(entry)) renderLog();
}

function renderLog() {
  const node = $("log");
  const rows = state.logs.filter(logVisible);
  node.innerHTML = "";
  rows.forEach((entry) => {
    const row = document.createElement("div");
    row.className = entry.kind || "info";
    const time = document.createElement("span");
    time.className = "t";
    time.textContent = entry.time;
    const text = document.createElement("span");
    text.className = entry.kind || "info";
    text.textContent = entry.text;
    row.append(time, document.createTextNode("  "), text);
    node.append(row);
  });
  node.scrollTop = node.scrollHeight;
  $("log-count").textContent = String(rows.length);
}

/* ── mm calibration ────────────────────────────────────────────────────────
   Display-only. The firmware speaks pulses and keeps speaking pulses; this
   just lets a pulse count carry an approximate millimetre reading so grid
   coordinates are imaginable. Stored per browser, never sent to the board. */

const MM_KEY = "h743-mm-per-pulse";

function mmPerPulse() {
  try {
    const saved = Number(localStorage.getItem(MM_KEY));
    return Number.isFinite(saved) && saved > 0 ? saved : 0;
  } catch (error) {
    return 0;
  }
}

function formatMm(pulses) {
  const scale = mmPerPulse();
  if (!scale || !Number.isFinite(pulses)) return "";
  const mm = Math.abs(pulses) * scale;
  if (mm >= 1000) return `${(mm / 1000).toFixed(2)} m`;
  return `${mm.toFixed(mm < 100 ? 1 : 0)} mm`;
}

/* Any <span class="mm" data-mm-for="some-input-id"> mirrors that input's value
   in millimetres, so adding a readout is markup-only. */
function renderMm() {
  const scale = mmPerPulse();
  document.querySelectorAll("[data-mm-for]").forEach((node) => {
    const source = $(node.dataset.mmFor);
    const text = source ? formatMm(Number(source.value)) : "";
    const prefix = node.dataset.mmPrefix;
    node.textContent = text ? (prefix ? `${prefix} ≈ ${text}` : `≈ ${text}`) : "";
  });
  const state = $("mm-state");
  if (state) {
    state.textContent = scale ? `1000 步 ≈ ${formatMm(1000)}` : "未标定";
    state.dataset.state = scale ? "set" : "unset";
  }
  renderFieldSummary();
}

function saveMmCalibration() {
  const pulses = Number($("mm-pulses").value);
  const measured = Number($("mm-measured").value);
  if (!Number.isFinite(pulses) || pulses <= 0 || !Number.isFinite(measured) || measured <= 0) {
    status("毫米标定需要正的步数和实测距离。", "warn");
    return;
  }
  try { localStorage.setItem(MM_KEY, String(measured / pulses)); } catch (error) { /* private mode */ }
  renderMm();
  status(`已标定：${pulses} 步 = ${measured} mm。`, "ok");
}

function clearMmCalibration() {
  try { localStorage.removeItem(MM_KEY); } catch (error) { /* private mode */ }
  $("mm-measured").value = "";
  renderMm();
  status("已清除毫米标定，界面恢复只显示脉冲。", "ok");
}

/* ── theme ─────────────────────────────────────────────────────────────── */

const THEME_COLOR = { light: "#f4f6f9", dark: "#0a0e13" };

function applyTheme(name) {
  document.documentElement.dataset.theme = name;
  $("btn-theme").textContent = name === "dark" ? "☀" : "☾";
  $("btn-theme").title = name === "dark" ? "切换到浅色主题" : "切换到深色主题";
  const meta = document.querySelector('meta[name="theme-color"]');
  if (meta) meta.setAttribute("content", THEME_COLOR[name]);
  /* The gauges draw their arcs and ticks from CSS variables, so redraw them. */
  servoCards().forEach(drawGauge);
}

function toggleTheme() {
  const next = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
  applyTheme(next);
  try { localStorage.setItem("h743-theme", next); } catch (error) { /* private mode */ }
}

function applyNotes(mode) {
  document.documentElement.dataset.notes = mode;
  const button = $("btn-notes");
  button.setAttribute("aria-pressed", mode === "on" ? "true" : "false");
  button.title = mode === "on" ? "隐藏页面上的说明文字" : "显示页面上的说明文字";
}

function toggleNotes() {
  const next = document.documentElement.dataset.notes === "on" ? "off" : "on";
  applyNotes(next);
  try { localStorage.setItem("h743-notes", next); } catch (error) { /* private mode */ }
}

/* ── wiring ────────────────────────────────────────────────────────────── */

function tab(name) {
  document.querySelectorAll(".tab").forEach((node) => node.classList.toggle("active", node.dataset.tab === name));
  document.querySelectorAll(".panel").forEach((node) => node.classList.toggle("active", node.id === `tab-${name}`));
  /* Leaving the chassis page must not leave a held direction running. The worker
     would expire it anyway; this is the immediate version. */
  if (name !== "chassis" && state.jogBody) stopMotion();
  if (name === "imu") { imuView.refreshPorts(); imuView.drawSoon(); }
  if (name === "camera") cameraView.refreshDevices();
  if (name !== "vision") cameraAlignmentView.tuning.stop();
  if (name === "camera2") camera2View.refreshDevices();
}

async function loadPorts() {
  const data = await fetch("/api/ports").then((r) => r.json()).catch(() => null);
  if (!data) return;
  const select = $("port");
  select.innerHTML = "";
  data.ports.forEach((port) => {
    const option = document.createElement("option");
    option.value = port.id;
    option.textContent = port.description ? `${port.label} · ${port.description}` : port.label;
    select.append(option);
  });
  if (data.current && data.ports.some((p) => p.id === data.current)) select.value = data.current;
}

const KEY_DIRECTION = {
  ArrowUp: "FWD", ArrowDown: "BACK", ArrowLeft: "LEFT", ArrowRight: "RIGHT",
  q: "CCW", Q: "CCW", e: "CW", E: "CW",
};

function typing(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target.isContentEditable;
}

function bind() {
  $("flow-start").addEventListener("click", () => post("/api/flow/action", {action:"start"}));
  $("flow-stop").addEventListener("click", () => post("/api/flow/action", {action:"stop"}));
  $("flow-save").addEventListener("click", saveFlowSettings);
  $("flow-apply").addEventListener("click", () => post("/api/flow/action", {action:"apply"}));
  $("flow-mode").addEventListener("change", () => {
    try { localStorage.setItem("h743-flow-frame", $("flow-mode").value); } catch {}
    renderFlow();
  });
  try { $("flow-mode").value = localStorage.getItem("h743-flow-frame") === "world" ? "world" : "body"; } catch {}
  $("btn-take-control").addEventListener('click', async () => {
    const result = await post('/api/session/claim', {});
    if (result.ok) apply({type: 'role', role: result.role});
  });
  $("nav-save").addEventListener("click", saveMission);
  document.querySelectorAll("#nav-field-editor input").forEach(input => input.addEventListener("input", () => {
    if (input.id !== "nav-field-remap") state.fieldDirty = true;
    navigationChanged();
  }));
  document.querySelectorAll('[id^="nav-grid-"]').forEach(input => input.addEventListener("input", () => {
    const grid = {};
    for (const axis of ["x", "y"]) grid[axis] = [0, 1, 2].map(i => { const text = $(`nav-grid-${axis}${i}`).value.trim(); return text ? Number(text) : NaN; });
    if (validGrid(grid)) syncFieldEditor(grid);
    navigationChanged();
  }));
  for (const id of ["nav-points", "nav-route"]) $(id).addEventListener("input", navigationChanged);
  /* Structural edits redraw from the in-page draft so an unsaved coordinate is
     not silently discarded by adding a row. Nothing is sent until 保存参数. */
  const redraw = (mutate) => () => {
    let draft;
    try { draft = readMission(); } catch (error) { status(error.message, "warn"); return; }
    if (!draft) return;
    try { mutate(draft); } catch (error) { status(error.message, "warn"); return; }
    renderStations(draft);
    renderStartStation(draft);
    renderRoute(draft);
    writeMissionGrid(draft.grid);
    navigationChanged();
    renderNavigation();
  };
  $("nav-station-add").addEventListener("click", redraw((draft) => {
    if (draft.stations.length >= 32) throw new Error("站点上限 32 个。");
    const used = new Set(draft.stations.map((s) => s.name));
    let name = "S" + draft.stations.length;
    while (used.has(name)) name += "_";
    draft.stations.push({ name, x: draft.stations[0].x, y: draft.stations[0].y, heading: 0, arrival_enabled: false });
  }));
  $("nav-add-x").addEventListener("click", () => {
    try {
      const draft = readMission();
      if (draft.stations.some(s => s.name === 'X')) throw new Error('X 点已存在。');
      if (draft.stations.length >= 32) throw new Error('站点上限 32 个。');
      draft.stations.push({name:'X',x:0,y:28800,heading:-180,arrival_enabled:true});
      if (!draft.grid.x.includes(0)) throw new Error('X 点需要 x=0 的竖向通道。');
      draft.grid.y[2] = Math.max(draft.grid.y[2], 28800);
      renderStations(draft); renderStartStation(draft); renderRoute(draft);
      writeMissionGrid(draft.grid); navigationChanged();
      $("nav-config-state").textContent = '已添加 X 点 · 尚未保存';
    } catch (error) { status(error.message, 'warn'); }
  });
  $("nav-start-station").addEventListener('change', () => {
    try {
      const draft = readMission();
      renderStations(draft); renderStartStation(draft); renderRoute(draft);
      writeMissionGrid(draft.grid); navigationChanged();
      $("nav-config-state").textContent = '起点已修改 · 尚未保存';
    } catch (error) { status(error.message, 'warn'); }
  });
  $("nav-station-remove").addEventListener("click", redraw((draft) => {
    if (draft.stations.length <= 2) return;
    const last = draft.stations.length - 1;
    if (draft.start_station === last) throw new Error('请先切换巡航起点，再删除此站点。');
    if (draft.route.some((leg) => leg.station === last)) throw new Error("请先修改引用末站点的路线段，再删除站点。");
    draft.stations.pop();
  }));
  $("nav-leg-add").addEventListener("click", redraw((draft) => {
    if (draft.route.length >= 32) return;
    draft.route.push({ station: 0, hold: 0 });
  }));
  $("nav-leg-remove").addEventListener("click", redraw((draft) => {
    if (draft.route.length > 1) draft.route.pop();
  }));
  $("nav-origin").addEventListener("click", async () => {
    if (await saveMission()) post("/api/navigation/action", { action: "origin" });
  });
  $("nav-start").addEventListener("click", async () => {
    if (await saveMission()) post("/api/navigation/action", { action: "start" });
  });
  $("nav-stop").addEventListener("click", () => post("/api/navigation/action", { action: "stop" }));
  $("nav-go").addEventListener("click", () => post("/api/navigation/action", { action: "go" }));
  $("task-save").addEventListener("click", async () => {
    try { await post("/api/mission/config", { config: JSON.parse($("task-config").value) }); }
    catch (error) { status(error.message, "warn"); }
  });
  $("task-start").addEventListener("click", () => post("/api/mission/action", { action: "start" }));
  $("material-start").addEventListener("click", async () => {
    if (materialFlowView.dirty && !await materialFlowView.save()) return;
    await post("/api/competition/start", { full_flow: true });
  });
  $("task-stop").addEventListener("click", () => post("/api/mission/action", { action: "stop" }));
  $("turn-zero").addEventListener("click", () => turnAction("zero"));
  $("turn-start").addEventListener("click", () => turnAction("start"));
  $("turn-stop").addEventListener("click", () => turnAction("stop"));
  // The three pages edit one shared draft. Polling only updates board readback.
  for (const prefix of ['turn','nav-turn','imu-turn']) {
    $(`${prefix}-configure`).addEventListener('click', () => turnAction('configure', prefix));
    for (const key of ['kp','tolerance','min-sps']) {
      const node = $(`${prefix}-${key}`);
      try { node.value = localStorage.getItem(`h743-turn-${key}`) || ({kp:'20',tolerance:'0.20','min-sps':'200'}[key]); } catch {}
      node.addEventListener('input', () => {
        for (const other of ['turn','nav-turn','imu-turn']) $(`${other}-${key}`).value = node.value;
        try { localStorage.setItem(`h743-turn-${key}`, node.value); } catch {}
        renderTurn();
      });
    }
  }
  $("turn-mode").addEventListener("change", renderTurn);
  document.querySelectorAll("#tab-navigation input:not([id^=\"nav-turn-\"])").forEach((input) => {
    input.addEventListener("input", () => { $("nav-config-state").textContent = "参数已修改 · 尚未保存"; });
  });
  document.querySelectorAll(".tab").forEach((node) => {
    node.addEventListener("click", () => tab(node.dataset.tab));
  });
  document.querySelectorAll("[data-open-tab]").forEach(node => {
    node.addEventListener("click", () => tab(node.dataset.openTab));
  });

  $("btn-ports").addEventListener("click", loadPorts);
  $("btn-host-restart").addEventListener("click", restartHost);
  $("btn-theme").addEventListener("click", toggleTheme);
  $("btn-notes").addEventListener("click", toggleNotes);
  $("mm-save").addEventListener("click", saveMmCalibration);
  $("mm-clear").addEventListener("click", clearMmCalibration);
  /* Keep the millimetre readouts live while the pulse fields are being typed. */
  document.querySelectorAll("[data-mm-for]").forEach((node) => {
    const source = $(node.dataset.mmFor);
    if (source) source.addEventListener("input", renderMm);
  });
  $("btn-connect").addEventListener("click", () => {
    if (state.link === "connected") post("/api/disconnect");
    else post("/api/connect", { port: $("port").value });
  });
  $("ack").addEventListener("change", (event) => { state.wiringAck = event.target.checked; });

  $("btn-arm").addEventListener("click", () => {
    if (state.armed) command("ARM 0");
    else if (!state.wiringAck && !state.demo) status("真实板使能前请先勾选“已核对驱动器接线”。", "warn");
    else command("ARM 1");
  });
  $("btn-stop").addEventListener("click", () => { state.jogBody = null; syncPad(); command("STOP"); });
  $("btn-recover").addEventListener("click", () => command("MOTION CLEAR"));

  /* One wiring pass per gauge card, so a third servo needs markup and nothing
     else here. */
  servoCards().forEach((card) => {
    const channel = channelOf(card);
    const input = card.querySelector(".angle-input");
    card.querySelector(".servo-send").addEventListener("click", () => sendAngle(channel, input.value));
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") sendAngle(channel, input.value);
    });
    card.querySelector(".presets").addEventListener("click", (event) => {
      const deg = event.target.dataset ? event.target.dataset.deg : undefined;
      if (deg === undefined) return;
      input.value = deg;
      sendAngle(channel, deg);
    });
  });
  $("servo-diagnose").addEventListener("click", diagnoseServo);
  $("pulse-send").addEventListener("click", () => {
    const us = num($("pulse-input").value);
    const channel = num($("pulse-ch").value, 1);
    if (us === null || us < 500 || us > 2500) { status("脉宽必须在 500–2500 µs。", "warn"); return; }
    if (!(channel >= 1 && channel <= (state.servoChannels || 1))) {
      status(`通道必须是 1–${state.servoChannels || 1}。`, "warn");
      return;
    }
    const degrees = pulseToAngle(channel, us);
    /* The firmware exposes no raw-pulse command, so this panel converts and
       sends through the angle path instead of inventing a command. */
    const card = servoCards()[channel - 1];
    if (card) card.querySelector(".angle-input").value = degrees;
    sendAngle(channel, degrees);
  });

  document.querySelectorAll(".pad").forEach((node) => {
    const direction = node.dataset.dir;
    if (direction === "HALT") {
      node.addEventListener("click", () => { state.selected = null; stopMotion(); });
      return;
    }
    node.addEventListener("click", () => {
      state.selected = direction;
      renderChassis();
    });
    const press = (event) => {
      if (event.button !== undefined && event.button !== 0) return;
      event.preventDefault();
      const sps = num($("ch-sps").value, 500);
      const accel = num($("ch-accel").value, 2000);
      jog(`CHASSIS JOG ${direction} ${sps} ${accel}`);
    };
    const release = () => { if (state.jogBody && state.jogBody.includes(` ${direction} `)) stopMotion(); };
    node.addEventListener("pointerdown", press);
    node.addEventListener("pointerup", release);
    node.addEventListener("pointerleave", release);
    node.addEventListener("pointercancel", release);
  });
  $("ch-move").addEventListener("click", chassisMove);
  $("ch-assist").addEventListener("change", renderChassis);

  document.querySelectorAll(".inv").forEach(box => {
    box.addEventListener("change", () => {
      // Polls must preserve unsent selections; only a matching board mask
      // confirms application. This also retains newer edits during a request.
      state.invertDraft = [...document.querySelectorAll(".inv")].reduce((mask, node) => mask | (node.checked ? 1 << Number(node.value) : 0), 0);
      renderChassis();
    });
  });
  $("inv-apply").addEventListener("click", () => {
    let mask = 0;
    document.querySelectorAll(".inv").forEach((box) => { if (box.checked) mask |= 1 << Number(box.value); });
    command(`CHASSIS CONFIG ${mask}`);
  });

  $("ax-move").addEventListener("click", () => {
    const axis = num($("ax-axis").value, 0);
    const steps = num($("ax-steps").value, 0);
    const sps = num($("ax-sps").value, 0);
    const accel = num($("ax-accel").value, 0);
    if (!(axis >= 1 && axis <= 7)) { status("轴编号 1–7。", "warn"); return; }
    if (!steps || Math.abs(steps) > 10000000) { status("步数非零且绝对值 ≤10000000。", "warn"); return; }
    const limit = axis <= 4 ? 5000 : 3000;
    if (!(sps >= 1 && sps <= limit)) { status(`M${axis} 速度上限 ${limit} step/s。`, "warn"); return; }
    if (!(accel >= 1 && accel <= 20000)) { status("加速度 1–20000 step/s²。", "warn"); return; }
    command(`MOVE ${axis} ${steps} ${sps} ${accel}`);
  });
  $("lift-dir").addEventListener("click", () => {
    const sign = Number($("lift-sign").value);
    if (![1, -1].includes(sign)) { status("请选择已经确认的向下方向；程序不会自动试转。", "warn"); return; }
    command(`LIFT DIR ${sign}`);
  });
  auxCards().forEach(card => {
    card.querySelectorAll("[data-action]").forEach(button => {
      button.addEventListener("click", () => auxiliaryAction(card, button.dataset.action));
    });
    card.querySelectorAll(".aux-setting").forEach(input => {
        input.addEventListener("input", () => { $("aux-config-state").textContent = "参数已修改，请保存；范围修改后需重新应用。"; renderAuxiliary(); });
    });
  });

  $("w-apply").addEventListener("click", () => {
    const speeds = ["w1", "w2", "w3", "w4"].map((id) => num($(id).value));
    if (speeds.some((value) => value === null || Math.abs(value) > 5000)) {
      status("四轮速度必须是 −5000 到 5000 的整数。", "warn");
      return;
    }
    command(`WHEELS ${speeds.join(" ")} 2000`);
  });
  $("w-zero").addEventListener("click", () => {
    ["w1", "w2", "w3", "w4"].forEach((id) => { $(id).value = "0"; });
    command("WHEELS 0 0 0 0 2000");
  });

  document.querySelectorAll("[data-led]").forEach((node) => {
    node.addEventListener("click", () => command(`LED ${node.dataset.led}`));
  });

  $("log-polls").addEventListener("change", (event) => { state.showPolls = event.target.checked; renderLog(); });
  $("log-clear").addEventListener("click", () => { state.logs = []; state.seenLog.clear(); renderLog(); });

  document.addEventListener("keydown", (event) => {
    if (event.repeat || typing(event.target)) return;
    if (!$("tab-chassis").classList.contains("active")) return;
    const direction = KEY_DIRECTION[event.key];
    if (!direction || state.link !== "connected" || state.role === "readonly") return;
    event.preventDefault();
    const sps = num($("ch-sps").value, 500);
    const accel = num($("ch-accel").value, 2000);
    jog(`CHASSIS JOG ${direction} ${sps} ${accel}`);
  });
  document.addEventListener("keyup", (event) => {
    if (!KEY_DIRECTION[event.key]) return;
    if (state.jogBody) stopMotion();
  });
  window.addEventListener("blur", () => { if (state.jogBody) stopMotion(); });
  window.addEventListener("beforeunload", () => { if (state.jogBody) post("/api/jog", { body: null }); });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden && state.jogBody) stopMotion();
  });
}

/* ── boot ──────────────────────────────────────────────────────────────── */

applyTheme(document.documentElement.dataset.theme === "dark" ? "dark" : "light");
applyNotes(document.documentElement.dataset.notes === "on" ? "on" : "off");
renderMm();
const imuView = new ImuView(post);
const cameraView = new CameraView(post);
const cameraAlignmentView = new CameraAlignmentView(post, cameraView, status => { state.cameraAlignment = status; });
const camera2View = new CameraView(post, "camera2", "/api/camera2");
const bPickupView = new BPickupView(post, config => { state.bPickupConfig = config; });
const materialFlowView = new MaterialFlowView(post, config => { state.materialFlowConfig = config; });
const pickPlaceView = new PickPlaceView(post, (progress, config) => {
  state.pickPlace = progress;
  if (config) state.pickPlaceConfig = config;
  renderAll();
}, async () => bPickupView.dirty ? bPickupView.save() : true);
$("auto-devices-enabled").addEventListener("change", async (event) => {
  const result = await post("/api/devices/automatic", { enabled: event.target.checked });
  if (result.ok) state.automatic = result.automatic;
  renderRole();
});
bind();
renderRole();
renderWheels(null);
renderServo();
loadPorts();
listen();
