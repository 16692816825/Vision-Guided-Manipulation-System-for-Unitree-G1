// ======================================================================
// G1 NavGrasp Web 控制面板 — 前端逻辑
// 模块：路由、状态轮询、日志、命令下发、各视图数据绑定、工作流可视化
// ======================================================================

// ======================================================================
// SVG 图标常量（Lucide 风格，无外部依赖）
// 所有 path 来自 lucide-icons (MIT License)
// ======================================================================
const ICONS = {
  // 导航
  'target':       '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/></svg>',
  'search':       '<svg viewBox="0 0 24 24"><circle cx="11" cy="11" r="8"/><path d="m21 21-4.3-4.3"/></svg>',
  'gamepad':      '<svg viewBox="0 0 24 24"><line x1="6" y1="11" x2="10" y2="11"/><line x1="8" y1="9" x2="8" y2="13"/><line x1="15" y1="12" x2="15.01" y2="12"/><line x1="18" y1="10" x2="18.01" y2="10"/><path d="M17.32 5H6.68a4 4 0 0 0-3.978 3.59c-.006.052-.01.101-.017.152C2.604 9.416 2 14.456 2 16a3 3 0 0 0 3 3c1 0 1.5-.5 2-1l1.414-1.414A2 2 0 0 1 9.828 16h4.344a2 2 0 0 1 1.414.586L17 18c.5.5 1 1 2 1a3 3 0 0 0 3-3c0-1.545-.604-6.584-.685-7.258-.007-.05-.011-.1-.017-.151A4 4 0 0 0 17.32 5z"/></svg>',
  'activity':     '<svg viewBox="0 0 24 24"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>',
  'settings':     '<svg viewBox="0 0 24 24"><path d="M12.22 2h-.44a2 2 0 0 0-2 2v.18a2 2 0 0 1-1 1.73l-.43.25a2 2 0 0 1-2 0l-.15-.08a2 2 0 0 0-2.73.73l-.22.38a2 2 0 0 0 .73 2.73l.15.1a2 2 0 0 1 1 1.72v.51a2 2 0 0 1-1 1.74l-.15.09a2 2 0 0 0-.73 2.73l.22.38a2 2 0 0 0 2.73.73l.15-.08a2 2 0 0 1 2 0l.43.25a2 2 0 0 1 1 1.73V20a2 2 0 0 0 2 2h.44a2 2 0 0 0 2-2v-.18a2 2 0 0 1 1-1.73l.43-.25a2 2 0 0 1 2 0l.15.08a2 2 0 0 0 2.73-.73l.22-.39a2 2 0 0 0-.73-2.73l-.15-.08a2 2 0 0 1-1-1.74v-.5a2 2 0 0 1 1-1.74l.15-.09a2 2 0 0 0 .73-2.73l-.22-.38a2 2 0 0 0-2.73-.73l-.15.08a2 2 0 0 1-2 0l-.43-.25a2 2 0 0 1-1-1.73V4a2 2 0 0 0-2-2z"/><circle cx="12" cy="12" r="3"/></svg>',
  'sparkles':     '<svg viewBox="0 0 24 24"><path d="m12 3-1.9 5.8a2 2 0 0 1-1.3 1.3L3 12l5.8 1.9a2 2 0 0 1 1.3 1.3L12 21l1.9-5.8a2 2 0 0 1 1.3-1.3L21 12l-5.8-1.9a2 2 0 0 1-1.3-1.3z"/><path d="M5 3v4"/><path d="M19 17v4"/><path d="M3 5h4"/><path d="M17 19h4"/></svg>',
  // 工作流步骤
  'crosshair':    '<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><line x1="22" y1="12" x2="18" y2="12"/><line x1="6" y1="12" x2="2" y2="12"/><line x1="12" y1="6" x2="12" y2="2"/><line x1="12" y1="22" x2="12" y2="18"/></svg>',
  'footprints':   '<svg viewBox="0 0 24 24"><path d="M4 16v-2.38C4 11.5 2.97 10.5 3 8c.03-2.72 1.49-6 4.5-6C9.37 2 10 3.8 10 5.5c0 3.11-2 5.66-2 8.68V16a2 2 0 1 1-4 0Z"/><path d="M20 20v-2.38c0-2.12 1.03-3.12 1-5.62-.03-2.72-1.49-6-4.5-6C14.63 6 14 7.8 14 9.5c0 3.11 2 5.66 2 8.68V20a2 2 0 1 0 4 0Z"/><path d="M16 17h4"/><path d="M4 13h4"/></svg>',
  'bot':          '<svg viewBox="0 0 24 24"><path d="M12 8V4H8"/><rect width="16" height="12" x="4" y="8" rx="2"/><path d="M2 14h2"/><path d="M20 14h2"/><path d="M15 13v2"/><path d="M9 13v2"/></svg>',
  'check':        '<svg viewBox="0 0 24 24"><polyline points="20 6 9 17 4 12"/></svg>',
  // 任务按钮
  'package':      '<svg viewBox="0 0 24 24"><path d="m7.5 4.27 9 5.15"/><path d="M21 8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16Z"/><path d="m3.3 7 8.7 5 8.7-5"/><path d="M12 22V12"/></svg>',
  'rotate-cw':    '<svg viewBox="0 0 24 24"><path d="M21 12a9 9 0 1 1-9-9c2.52 0 4.93 1 6.74 2.74L21 8"/><path d="M21 3v5h-5"/></svg>',
  'rotate-ccw':   '<svg viewBox="0 0 24 24"><path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8"/><path d="M3 3v5h5"/></svg>',
  'panel-left-close':  '<svg viewBox="0 0 24 24"><rect width="18" height="18" x="3" y="3" rx="2"/><path d="M9 3v18"/><path d="m16 15-3-3 3-3"/></svg>',
  'panel-left-open':   '<svg viewBox="0 0 24 24"><rect width="18" height="18" x="3" y="3" rx="2"/><path d="M9 3v18"/><path d="m14 9 3 3-3 3"/></svg>',
  'arrow-left':   '<svg viewBox="0 0 24 24"><path d="m12 19-7-7 7-7"/><path d="M19 12H5"/></svg>',
  'square':       '<svg viewBox="0 0 24 24"><rect width="14" height="14" x="5" y="5" rx="2"/></svg>',
  'trash-2':      '<svg viewBox="0 0 24 24"><path d="M3 6h18"/><path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6"/><path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/><line x1="10" x2="10" y1="11" y2="17"/><line x1="14" x2="14" y1="11" y2="17"/></svg>',
  // 方向 / Chevron
  'chevron-up':    '<svg viewBox="0 0 24 24"><polyline points="18 15 12 9 6 15"/></svg>',
  'chevron-down':  '<svg viewBox="0 0 24 24"><polyline points="6 9 12 15 18 9"/></svg>',
  'chevron-left':  '<svg viewBox="0 0 24 24"><polyline points="15 18 9 12 15 6"/></svg>',
  'chevron-right': '<svg viewBox="0 0 24 24"><polyline points="9 18 15 12 9 6"/></svg>',
  // 设置 + 其他
  'lightbulb':    '<svg viewBox="0 0 24 24"><path d="M15 14c.2-1 .7-1.7 1.5-2.5 1-.9 1.5-2.2 1.5-3.5A6 6 0 0 0 6 8c0 1 .2 2.2 1.5 3.5.7.7 1.3 1.5 1.5 2.5"/><path d="M9 18h6"/><path d="M10 22h4"/></svg>',
  'refresh-cw':   '<svg viewBox="0 0 24 24"><path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"/><path d="M21 3v5h-5"/><path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"/><path d="M3 21v-5h5"/></svg>',
  'palette':      '<svg viewBox="0 0 24 24"><circle cx="13.5" cy="6.5" r=".5" fill="currentColor"/><circle cx="17.5" cy="10.5" r=".5" fill="currentColor"/><circle cx="8.5" cy="7.5" r=".5" fill="currentColor"/><circle cx="6.5" cy="12.5" r=".5" fill="currentColor"/><path d="M12 2C6.5 2 2 6.5 2 12s4.5 10 10 10c.926 0 1.648-.746 1.648-1.688 0-.437-.18-.835-.437-1.125-.29-.289-.438-.652-.438-1.125a1.64 1.64 0 0 1 1.668-1.668h1.996c3.051 0 5.555-2.503 5.555-5.554C21.965 6.012 17.461 2 12 2z"/></svg>',
  // 敬请期待
  'mic':          '<svg viewBox="0 0 24 24"><rect width="6" height="12" x="9" y="2" rx="3"/><path d="M19 10v2a7 7 0 0 1-14 0v-2"/><line x1="12" x2="12" y1="19" y2="22"/></svg>',
  'map':          '<svg viewBox="0 0 24 24"><polygon points="3 6 9 3 15 6 21 3 21 18 15 21 9 18 3 21"/><line x1="9" x2="9" y1="3" y2="18"/><line x1="15" x2="15" y1="6" y2="21"/></svg>',
  'cloud':        '<svg viewBox="0 0 24 24"><path d="M17.5 19H9a7 7 0 1 1 6.71-9h1.79a4.5 4.5 0 1 1 0 9Z"/></svg>',
  'users':        '<svg viewBox="0 0 24 24"><path d="M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2"/><circle cx="9" cy="7" r="4"/><path d="M22 21v-2a4 4 0 0 0-3-3.87"/><path d="M16 3.13a4 4 0 0 1 0 7.75"/></svg>',
  'camera':       '<svg viewBox="0 0 24 24"><path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3l-2.5-3z"/><circle cx="12" cy="13" r="3"/></svg>',
  'sliders':      '<svg viewBox="0 0 24 24"><line x1="4" x2="4" y1="21" y2="14"/><line x1="4" x2="4" y1="10" y2="3"/><line x1="12" x2="12" y1="21" y2="12"/><line x1="12" x2="12" y1="8" y2="3"/><line x1="20" x2="20" y1="21" y2="16"/><line x1="20" x2="20" y1="12" y2="3"/><line x1="2" x2="6" y1="14" y2="14"/><line x1="10" x2="14" y1="8" y2="8"/><line x1="18" x2="22" y1="16" y2="16"/></svg>',
  'image':        '<svg viewBox="0 0 24 24"><rect width="18" height="18" x="3" y="3" rx="2" ry="2"/><circle cx="9" cy="9" r="2"/><path d="m21 15-3.086-3.086a2 2 0 0 0-2.828 0L6 21"/></svg>',
  'server':       '<svg viewBox="0 0 24 24"><rect width="20" height="8" x="2" y="2" rx="2" ry="2"/><rect width="20" height="8" x="2" y="14" rx="2" ry="2"/><line x1="6" x2="6.01" y1="6" y2="6"/><line x1="6" x2="6.01" y1="18" y2="18"/></svg>',
};

function renderIcons(root = document) {
  root.querySelectorAll('[data-icon]').forEach(el => {
    const name = el.dataset.icon;
    const svg = ICONS[name];
    if (!svg) return;
    // 避免重复注入（如已经有 svg 子元素则跳过）
    if (el.querySelector('svg')) return;
    el.innerHTML = svg;
  });
}

renderIcons();


// ======================================================================
// 侧边栏折叠 / 展开（持久化到 localStorage）
// ======================================================================
const SIDEBAR_KEY = 'g1.sidebar.collapsed';
function applySidebarState(collapsed) {
  const layout = document.querySelector('.app-layout');
  if (!layout) return;
  layout.classList.toggle('sidebar-collapsed', !!collapsed);
  // 切换按钮图标 + 标题
  const btn = document.getElementById('sidebar-toggle');
  if (btn) {
    const iconEl = btn.querySelector('.sidebar-toggle-icon');
    if (iconEl) {
      iconEl.dataset.icon = collapsed ? 'panel-left-open' : 'panel-left-close';
      iconEl.innerHTML = ICONS[iconEl.dataset.icon] || '';
    }
    btn.title = collapsed ? '展开侧边栏' : '收起侧边栏';
    btn.setAttribute('aria-label', btn.title);
  }
}
(function initSidebarToggle() {
  // 启动时读取本地状态
  const saved = localStorage.getItem(SIDEBAR_KEY) === '1';
  applySidebarState(saved);

  const btn = document.getElementById('sidebar-toggle');
  if (!btn) return;
  btn.addEventListener('click', () => {
    const layout = document.querySelector('.app-layout');
    const next = !layout.classList.contains('sidebar-collapsed');
    applySidebarState(next);
    try { localStorage.setItem(SIDEBAR_KEY, next ? '1' : '0'); } catch (_) {}
  });
})();


// ======================================================================
// 状态映射
// ======================================================================
const STATE_COLORS = {
  IDLE:     { dot: '#94A3B8', text: '空闲' },
  WORKING:  { dot: '#F97316', text: '执行中' },
  GRABBING: { dot: '#EF4444', text: '抓取中' },
  MENU:     { dot: '#10B981', text: '可放下' },
};

const VIEW_META = {
  grasp:    { title: '目标抓取',   subtitle: '完整抓取任务流程' },
  detect:   { title: '目标识别',   subtitle: '仅显示视觉检测，不触发运动' },
  control:  { title: '运动控制',   subtitle: '手动遥控机器人' },
  arm_debug:{ title: '上肢调试',   subtitle: '分组滑块 + 预设管理' },
  status:   { title: '系统状态',   subtitle: '话题健康与日志总览' },
  nodes:    { title: '节点管理',   subtitle: 'ROS2 子进程启动 / 停止 / 参数' },
  settings: { title: '系统设置',   subtitle: '运行时参数热更新 + 界面偏好' },
  coming:   { title: '更多模块',   subtitle: '敬请期待' },
};

// ======================================================================
// 视图路由（hash 路由）
// ======================================================================
function switchView(name) {
  if (!VIEW_META[name]) name = 'grasp';

  document.querySelectorAll('.view').forEach(el => {
    el.classList.toggle('hidden', el.dataset.view !== name);
  });
  document.querySelectorAll('.nav-item').forEach(el => {
    el.classList.toggle('active', el.dataset.view === name);
  });
  // body 加标记类，让顶栏 TOC 仅在 settings 视图显示
  document.body.classList.toggle('settings-active', name === 'settings');

  const meta = VIEW_META[name];
  document.getElementById('page-title').textContent = meta.title;
  document.getElementById('page-subtitle').textContent = meta.subtitle;

  if (location.hash !== '#' + name) {
    history.replaceState(null, '', '#' + name);
  }
}

document.querySelectorAll('.nav-item').forEach(el => {
  el.addEventListener('click', e => {
    e.preventDefault();
    switchView(el.dataset.view);
  });
});

window.addEventListener('hashchange', () => {
  switchView(location.hash.replace('#', '') || 'grasp');
});

switchView(location.hash.replace('#', '') || 'grasp');


// ======================================================================
// 速度滑块（运动控制）
// ======================================================================
const speedSlider = document.getElementById('c-speed-slider');
const speedValue = document.getElementById('c-speed-value');
if (speedSlider) {
  speedSlider.addEventListener('input',
    () => speedValue.textContent = (+speedSlider.value).toFixed(2));
}

function getSpeed() {
  return speedSlider ? parseFloat(speedSlider.value) : 0.2;
}


// ======================================================================
// 命令下发
// ======================================================================
async function postCmd(path, body = {}) {
  try {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await r.json();
    if (!r.ok || (data && data.ok === false)) {
      appendLog(data.error || 'error', 'error');
    }
  } catch (e) {
    appendLog('网络错误: ' + e.message, 'error');
  }
}

// 抓取/放下等任务按钮（data-cmd）
document.querySelectorAll('[data-cmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd;
    switch (cmd) {
      case 'search':       postCmd('/api/cmd/search'); break;
      case 'grab':         postCmd('/api/cmd/grab'); break;
      case 'putdown':      postCmd('/api/cmd/putdown'); break;
      case 'turn_putdown': postCmd('/api/cmd/turn_putdown'); break;
      case 'left_putdown': postCmd('/api/cmd/left_putdown'); break;
      case 'stop':         postCmd('/api/cmd/stop'); break;
    }
  });
});

// 方向盘按钮（data-dcmd）— 运动控制模块
//   forward/backward → vx (前后)
//   left/right       → vy (左/右平移，vy>0=左)
//   turn_left/turn_right → vyaw (原地左转/右转，vyaw>0=左转)
document.querySelectorAll('[data-dcmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.dcmd;
    const v = getSpeed();
    const vyaw = v * 2.0;
    let vx = 0, vy = 0, vyawOut = 0;
    switch (cmd) {
      case 'forward':    vx =  v; break;
      case 'backward':   vx = -v; break;
      case 'left':       vy =  v; break;
      case 'right':      vy = -v; break;
      case 'turn_left':  vyawOut =  vyaw; break;
      case 'turn_right': vyawOut = -vyaw; break;
      case 'stop':       postCmd('/api/cmd/stop'); return;
    }
    postCmd('/api/cmd/manual', { vx, vy, vyaw: vyawOut });
    // 立即更新速度映射显示（乐观更新）
    setTxt('c-vx',   vx.toFixed(2));
    setTxt('c-vy',   vy.toFixed(2));
    setTxt('c-vyaw', vyawOut.toFixed(2));
  });
});


// ======================================================================
// 日志面板
// ======================================================================
function appendLog(msg, level = 'info') {
  const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  const text = `[${ts}] ${msg}`;
  ['log-box', 's-log-box'].forEach(id => {
    const box = document.getElementById(id);
    if (!box) return;
    const line = document.createElement('div');
    line.className = `log-line log-${level}`;
    line.textContent = text;
    box.appendChild(line);
    if (box.children.length > 300) box.removeChild(box.firstChild);
    box.scrollTop = box.scrollHeight;
  });
}

function clearLog() {
  ['log-box', 's-log-box'].forEach(id => {
    const box = document.getElementById(id);
    if (box) box.innerHTML = '';
  });
}


// ======================================================================
// 工作流步骤条可视化
// ======================================================================
// 根据后端状态 + 距离/对齐状况推断当前步骤
function inferWorkflowStep(s) {
  if (s.state === 'IDLE')     return 'IDLE';
  if (s.state === 'GRABBING') return 'GRAB';
  if (s.state === 'MENU')     return 'DONE';

  if (s.state === 'WORKING') {
    // 没有检测到目标 → 搜索中
    if (s.target_u == null) return 'SEARCH';
    // u 偏离中心 > 0.1 → 对齐中
    if (Math.abs(s.target_u - 0.5) > 0.1) return 'ALIGN';
    // 居中但未到达 → 接近中
    return 'APPROACH';
  }
  return 'IDLE';
}

// 更新工作流步骤条（active + done 状态）
function updateWorkflow(currentStep) {
  const order = ['SEARCH', 'ALIGN', 'APPROACH', 'GRAB', 'DONE'];
  const idx = order.indexOf(currentStep);
  document.querySelectorAll('.wf-step').forEach(el => {
    const step = el.dataset.step;
    const i = order.indexOf(step);
    el.classList.remove('active', 'done');
    if (i < 0) return;
    if (currentStep === 'IDLE') return;
    if (i < idx) el.classList.add('done');
    else if (i === idx) el.classList.add('active');
  });
}


// ======================================================================
// 健康度环形可视化
// ======================================================================
const RING_CIRC = 2 * Math.PI * 40; // r=40

function updateHealthRing(percent) {
  const ring = document.getElementById('health-ring-fill');
  if (!ring) return;
  const clamped = Math.max(0, Math.min(100, percent));
  const offset = RING_CIRC * (1 - clamped / 100);
  ring.style.strokeDashoffset = offset;
  // 颜色分级
  let color = '#10B981';
  if (clamped < 40) color = '#EF4444';
  else if (clamped < 70) color = '#F59E0B';
  ring.style.stroke = color;
  setTxt('health-percent', Math.round(clamped) + '%');
}

// 简化健康度计算：FPS + 检测 + 任务 三因子
function computeHealth(s) {
  let score = 0;
  // FPS 占 40%
  const fps = s.fps || 0;
  score += Math.min(fps / 25, 1) * 40;
  // 检测可用 占 30%
  if (s.det_count != null) score += 30;
  // 任务状态非 None 占 30%
  if (s.state && s.state !== 'UNKNOWN') score += 30;
  return score;
}


// ======================================================================
// 工具函数
// ======================================================================
function fmtOrDash(v, digits = 2, unit = '') {
  return v != null ? v.toFixed(digits) + unit : '—';
}

function setTxt(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function setBarPct(id, pct) {
  const bar = document.getElementById(id);
  if (!bar) return;
  const clamped = Math.max(0, Math.min(100, pct));
  bar.style.width = clamped + '%';
}


// ======================================================================
// 状态轮询（500ms）
// ======================================================================
let lastLogIdx = 0;
let connFailCount = 0;

async function pollState() {
  try {
    const r = await fetch('/api/state?since=' + lastLogIdx);
    if (!r.ok) { connFailCount++; return; }
    connFailCount = 0;
    const s = await r.json();

    updateConnIndicator(true, s.mock === true);

    // ----- 顶部全局状态 -----
    const st = STATE_COLORS[s.state] || { dot: '#94A3B8', text: s.state };
    const stateDot = document.getElementById('state-dot');
    if (stateDot) stateDot.style.background = st.dot;
    setTxt('state-label', st.text);
    setTxt('fps-label', 'FPS ' + (s.fps || 0).toFixed(0));
    setTxt('det-label', '检测 ' + (s.det_count || 0));

    // ----- 视图 1：目标抓取 -----
    updateWorkflow(inferWorkflowStep(s));
    setTxt('g-state-pill', s.state);
    setTxt('g-info-target', s.target_class || '—');
    setTxt('g-info-u',      fmtOrDash(s.target_u, 3));
    setTxt('g-info-dist',   fmtOrDash(s.distance, 2));
    setTxt('g-info-bbox',   fmtOrDash(s.bbox_max, 2));
    setTxt('g-info-count',  s.det_count != null ? s.det_count : '—');
    setTxt('g-info-fps',    (s.fps || 0).toFixed(1));

    // ----- 视图 2：目标识别 -----
    setTxt('d-fps',    (s.fps || 0).toFixed(1));
    setTxt('d-target', s.target_class || '—');
    setTxt('d-count',  s.det_count != null ? s.det_count : '—');
    setTxt('d-u',      fmtOrDash(s.target_u, 3));
    setTxt('d-bbox',   fmtOrDash(s.bbox_max, 2));
    setTxt('d-dist',   fmtOrDash(s.distance, 2, ' m'));

    // ----- 视图 3：运动控制 -----
    setTxt('c-info-state', s.state);
    setTxt('c-info-mode', s.state === 'WORKING' ? '自动任务' : '手动遥控');
    setTxt('c-info-fps', (s.fps || 0).toFixed(1));

    // ----- 视图 4：系统状态仪表盘 -----
    setTxt('s-fps',        (s.fps || 0).toFixed(1));
    setTxt('s-det-count',  s.det_count != null ? s.det_count : '—');
    setTxt('s-target',     s.target_class || '—');
    setTxt('s-u',          fmtOrDash(s.target_u, 3));
    setTxt('s-bbox',       fmtOrDash(s.bbox_max, 2));
    setTxt('s-dist',       fmtOrDash(s.distance, 2, ' m'));
    setTxt('s-state',      s.state);

    // 进度条
    setBarPct('s-fps-bar', ((s.fps || 0) / 30) * 100);
    // 深度距离：0 ~ 2m 映射到 100% ~ 0%（近时满条）
    if (s.distance != null) {
      setBarPct('s-dist-bar', Math.max(0, 100 - (s.distance / 2) * 100));
    } else {
      setBarPct('s-dist-bar', 0);
    }
    // u 位置条：距中心越近越满
    if (s.target_u != null) {
      setBarPct('s-u-bar', Math.max(0, 100 - Math.abs(s.target_u - 0.5) * 200));
    } else {
      setBarPct('s-u-bar', 0);
    }

    // 健康度环
    updateHealthRing(computeHealth(s));

    // ----- 相机/YOLO/RGBD 进程状态（轻量字段，不带日志）-----
    if (s.camera) applyProcStatus('camera', s.camera);
    if (s.yolo)   applyProcStatus('yolo', s.yolo);
    if (s.rgbd)   applyProcStatus('rgbd', s.rgbd);

    // ----- 日志追加 -----
    if (s.logs && s.logs.length > 0) {
      for (const entry of s.logs) appendLog(entry.msg, entry.level);
      lastLogIdx = s.log_idx;
    }
  } catch (e) {
    connFailCount++;
    updateConnIndicator(false);
  }
}

// 连接指示器（侧边栏底部）
function updateConnIndicator(ok, isMock = false) {
  const label = document.getElementById('conn-label');
  const dot = document.querySelector('.footer-pulse .pulse-dot');
  if (!label) return;
  if (ok) {
    if (isMock) {
      label.textContent = '本地预览';
      label.style.color = '#D97706';
      if (dot) dot.style.background = '#F59E0B';
    } else {
      label.textContent = '已连接';
      label.style.color = '#059669';
      if (dot) dot.style.background = '#10B981';
    }
  } else if (connFailCount > 3) {
    label.textContent = '连接断开';
    label.style.color = '#DC2626';
    if (dot) dot.style.background = '#EF4444';
  }
}

setInterval(pollState, getLocalPref('poll_interval', 500));
pollState();
appendLog('Web 面板已连接', 'info');


// ======================================================================
// 系统设置模块
// ======================================================================
// 本地偏好（存 localStorage）默认值
const LOCAL_PREF_DEFAULTS = {
  default_view: 'grasp',
  log_toast: false,
  poll_interval: 500,
};

function getLocalPref(key, fallback) {
  try {
    const v = localStorage.getItem('g1_pref_' + key);
    if (v == null) return fallback != null ? fallback : LOCAL_PREF_DEFAULTS[key];
    return JSON.parse(v);
  } catch (e) {
    return fallback != null ? fallback : LOCAL_PREF_DEFAULTS[key];
  }
}

function setLocalPref(key, value) {
  localStorage.setItem('g1_pref_' + key, JSON.stringify(value));
}

function resetLocalPrefs() {
  Object.keys(LOCAL_PREF_DEFAULTS).forEach(k => localStorage.removeItem('g1_pref_' + k));
  showToast('已重置本地偏好，刷新后生效', 'info');
  // 立刻把表单值刷回默认
  document.querySelectorAll('[data-local="true"]').forEach(el => {
    applyValueToInput(el, LOCAL_PREF_DEFAULTS[el.dataset.key]);
  });
}

function applyValueToInput(el, val) {
  if (el.type === 'checkbox') el.checked = !!val;
  else el.value = val == null ? '' : val;
}

function readValueFromInput(el) {
  if (el.type === 'checkbox') return el.checked;
  if (el.type === 'number') return el.value === '' ? null : parseFloat(el.value);
  return el.value;
}

// 加载配置：从后端拉取 + 本地偏好填充
async function reloadSettings() {
  // 1. 本地偏好
  document.querySelectorAll('[data-local="true"]').forEach(el => {
    applyValueToInput(el, getLocalPref(el.dataset.key));
  });

  // 2. 后端参数
  try {
    const r = await fetch('/api/config');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const data = await r.json();
    const values = data.values || {};
    document.querySelectorAll('[data-key]').forEach(el => {
      if (el.dataset.local === 'true') return;
      const key = el.dataset.key;
      if (key in values && values[key] != null) {
        applyValueToInput(el, values[key]);
      }
    });
    showToast('配置已加载', 'info');
  } catch (e) {
    showToast('加载配置失败: ' + e.message, 'error');
  }
}

// 保存某一分组
async function saveSettingsGroup(groupName) {
  const form = document.querySelector(`[data-settings-group="${groupName}"]`);
  if (!form) return;

  const backendUpdates = {};
  let localUpdateCount = 0;

  form.querySelectorAll('[data-key]').forEach(el => {
    const key = el.dataset.key;
    const val = readValueFromInput(el);
    if (el.dataset.local === 'true') {
      setLocalPref(key, val);
      localUpdateCount++;
    } else if (val !== null && val !== '') {
      backendUpdates[key] = val;
    }
  });

  // 后端更新
  if (Object.keys(backendUpdates).length > 0) {
    try {
      const r = await fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(backendUpdates),
      });
      const data = await r.json();
      if (!r.ok || data.ok === false) throw new Error(data.error || 'save failed');
      const updated = data.updated || [];
      const skipped = data.skipped || [];
      if (skipped.length > 0) {
        showToast(`已保存 ${updated.length} 项，${skipped.length} 项跳过`, 'error');
        for (const s of skipped) appendLog(`[配置] ${s.key} 跳过: ${s.reason}`, 'warn');
      } else {
        showToast(`已保存 ${updated.length} 项配置`, 'info');
      }
    } catch (e) {
      showToast('保存失败: ' + e.message, 'error');
      return;
    }
  } else if (localUpdateCount > 0) {
    showToast(`已保存 ${localUpdateCount} 项本地偏好`, 'info');
  } else {
    showToast('没有需要保存的更改', 'info');
  }
}

// 保存按钮事件绑定
document.querySelectorAll('[data-save]').forEach(btn => {
  btn.addEventListener('click', () => saveSettingsGroup(btn.dataset.save));
});

// Toast 提示
let toastTimer = null;
function showToast(msg, level = 'info') {
  const el = document.getElementById('settings-toast');
  if (!el) return;
  el.textContent = msg;
  el.classList.remove('hidden', 'error');
  if (level === 'error') el.classList.add('error');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add('hidden'), 2500);
}

// 初次进入设置页时懒加载
let settingsLoaded = false;
const origSwitchView = switchView;
switchView = function(name) {
  origSwitchView(name);
  if (name === 'settings' && !settingsLoaded) {
    settingsLoaded = true;
    reloadSettings();
  }
};

// 如果 hash 初始就是 settings，立即加载
if (location.hash === '#settings') {
  settingsLoaded = true;
  reloadSettings();
}

// 应用启动视图偏好（若未指定 hash）
if (!location.hash) {
  const defaultView = getLocalPref('default_view', 'grasp');
  if (defaultView && defaultView !== 'grasp') switchView(defaultView);
}


// ======================================================================
// 通用进程启动器（相机 / YOLO / RGBD）
// ======================================================================
const PROC_NAMES = ['camera', 'yolo', 'rgbd'];
const PROC_LABELS = { camera: '相机', yolo: 'YOLO', rgbd: 'RGBD' };

function _procEl(procName, bindKey) {
  return document.querySelector(`.process-card[data-proc="${procName}"] [data-bind="${bindKey}"]`);
}

async function procStart(name) {
  const startBtn = _procEl(name, 'start-btn');
  if (startBtn) startBtn.disabled = true;
  try {
    const r = await fetch(`/api/process/${name}/start`, { method: 'POST' });
    const data = await r.json();
    if (!r.ok || data.ok === false) {
      appendLog(`[${PROC_LABELS[name] || name}] 启动失败: ` + (data.error || 'unknown'), 'error');
    } else {
      appendLog(`[${PROC_LABELS[name] || name}] ` + (data.msg || '已启动'), 'info');
    }
  } catch (e) {
    appendLog(`[${PROC_LABELS[name] || name}] 网络错误: ` + e.message, 'error');
  } finally {
    if (startBtn) startBtn.disabled = false;
    refreshProcStatus(name);
  }
}

async function procStop(name) {
  const stopBtn = _procEl(name, 'stop-btn');
  if (stopBtn) stopBtn.disabled = true;
  try {
    const r = await fetch(`/api/process/${name}/stop`, { method: 'POST' });
    const data = await r.json();
    if (!r.ok || data.ok === false) {
      appendLog(`[${PROC_LABELS[name] || name}] 停止失败: ` + (data.error || 'unknown'), 'error');
    } else {
      appendLog(`[${PROC_LABELS[name] || name}] ` + (data.msg || '已停止'), 'info');
    }
  } catch (e) {
    appendLog(`[${PROC_LABELS[name] || name}] 网络错误: ` + e.message, 'error');
  } finally {
    if (stopBtn) stopBtn.disabled = false;
    refreshProcStatus(name);
  }
}

// 由轮询数据驱动单个进程卡片状态
function applyProcStatus(name, data) {
  if (!data) return;
  const card = document.querySelector(`.process-card[data-proc="${name}"]`);
  if (!card) return;

  const pill = card.querySelector('[data-bind="state-pill"]');
  const label = card.querySelector('[data-bind="state-label"]');
  if (pill && label) {
    if (data.running) {
      pill.classList.remove('cam-pill-off');
      pill.classList.add('cam-pill-on');
      label.textContent = '运行中';
    } else {
      pill.classList.remove('cam-pill-on');
      pill.classList.add('cam-pill-off');
      label.textContent = '未启动';
    }
  }

  const startBtn = card.querySelector('[data-bind="start-btn"]');
  const stopBtn = card.querySelector('[data-bind="stop-btn"]');
  if (startBtn) startBtn.disabled = !!data.running;
  if (stopBtn)  stopBtn.disabled  = !data.running;

  const pidEl = card.querySelector('[data-bind="pid"]');
  if (pidEl) pidEl.textContent = data.pid != null ? String(data.pid) : '—';
  const upEl = card.querySelector('[data-bind="uptime"]');
  if (upEl) upEl.textContent = fmtUptime(data.uptime);

  // 命令行预览
  const cmdEl = card.querySelector('[data-bind="cmd"]');
  if (cmdEl && data.params) {
    const parts = data.mode === 'launch'
      ? ['ros2 launch', data.pkg || '?', data.target || '?']
      : ['ros2 run',    data.pkg || '?', data.target || '?'];
    if (data.mode === 'launch') {
      Object.entries(data.params).forEach(([k, v]) => parts.push(`${k}:=${v}`));
    } else {
      const entries = Object.entries(data.params);
      if (entries.length > 0) {
        parts.push('--ros-args');
        entries.forEach(([k, v]) => parts.push(`-p ${k}:=${v}`));
      }
    }
    cmdEl.textContent = parts.join(' ');
  }

  // 参数动态绑定（data-bind="param.xxx"）
  if (data.params) {
    card.querySelectorAll('[data-bind^="param."]').forEach(el => {
      const key = el.dataset.bind.slice(6);  // 去掉 "param." 前缀
      if (key in data.params) el.textContent = data.params[key];
    });
  }
}

// 拉取单个进程完整状态（含日志）
async function refreshProcStatus(name) {
  try {
    const r = await fetch(`/api/process/${name}/status`);
    if (!r.ok) return;
    const data = await r.json();
    applyProcStatus(name, data);
    const card = document.querySelector(`.process-card[data-proc="${name}"]`);
    const box = card && card.querySelector('[data-bind="log-box"]');
    if (box && Array.isArray(data.logs)) {
      box.innerHTML = '';
      for (const line of data.logs) {
        const el = document.createElement('div');
        el.className = 'log-line log-info';
        el.textContent = line;
        box.appendChild(el);
      }
      box.scrollTop = box.scrollHeight;
    }
  } catch (e) { /* ignore */ }
}

// 切到 nodes 视图时刷新所有进程状态 + 把参数写入表单
const _origSwitchViewForCam = switchView;
switchView = function(name) {
  _origSwitchViewForCam(name);
  if (name === 'nodes') {
    PROC_NAMES.forEach(n => {
      refreshProcStatus(n);
      loadProcParamsForm(n);
    });
  }
};

// 把当前进程参数填回表单（从 /api/process/<n>/status 拉取）
async function loadProcParamsForm(name) {
  try {
    const r = await fetch(`/api/process/${name}/status`);
    if (!r.ok) return;
    const data = await r.json();
    const params = data.params || {};
    const form = document.querySelector(`[data-proc-form="${name}"]`);
    if (!form) return;
    form.querySelectorAll('[data-pkey]').forEach(el => {
      const key = el.dataset.pkey;
      if (!(key in params)) return;
      const v = params[key];
      if (el.type === 'checkbox') el.checked = String(v).toLowerCase() === 'true';
      else el.value = v;
    });
  } catch (e) { /* ignore */ }
}

// 保存表单参数 → POST /api/process/<n>/params
async function saveProcParams(name) {
  const form = document.querySelector(`[data-proc-form="${name}"]`);
  if (!form) return;
  const updates = {};
  form.querySelectorAll('[data-pkey]').forEach(el => {
    const key = el.dataset.pkey;
    if (el.type === 'checkbox') updates[key] = el.checked ? 'true' : 'false';
    else if (el.value !== '') updates[key] = el.value;
  });
  try {
    const r = await fetch(`/api/process/${name}/params`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(updates),
    });
    const data = await r.json();
    if (!r.ok || data.ok === false) throw new Error(data.error || 'save failed');
    appendLog(`[${PROC_LABELS[name] || name}] 参数已保存（下次启动生效）`, 'info');
    // 刷新 cmd 预览
    refreshProcStatus(name);
  } catch (e) {
    appendLog(`[${PROC_LABELS[name] || name}] 保存失败: ` + e.message, 'error');
  }
}



// ======================================================================
// 兼容旧 cameraStart/cameraStop（如果模板还在用）
// ======================================================================
async function cameraStart() { return procStart('camera'); }
async function cameraStop()  { return procStop('camera'); }
async function refreshCameraStatus() { return refreshProcStatus('camera'); }
function applyCameraStatus(cam) { applyProcStatus('camera', cam); }


// ======================================================================
// 系统环境自动检测
// ======================================================================
async function detectEnv() {
  try {
    const r = await fetch('/api/env/detect');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const data = await r.json();
    // 把检测结果填入表单（仅填空字段，不覆盖用户已填的值）
    const envFields = ['network_interface', 'cyclonedds_home', 'sdk_python_path', 'arm_script_dir'];
    envFields.forEach(key => {
      const el = document.querySelector(`[data-settings-group="env"] [data-key="${key}"]`);
      if (!el) return;
      // 更新 placeholder 为检测值
      if (data[key]) {
        el.placeholder = '检测到: ' + data[key];
      }
      // 如果用户没填，把检测值填入
      if (!el.value && data[key]) {
        el.value = data[key];
      }
    });
    // 显示额外信息
    const info = [];
    if (data.python_executable) info.push('Python: ' + data.python_executable);
    if (data.virtual_env) info.push('VENV: ' + data.virtual_env);
    if (info.length) appendLog('[环境检测] ' + info.join(' | '), 'info');
    showToast('环境检测完成', 'info');
  } catch (e) {
    showToast('环境检测失败: ' + e.message, 'error');
  }
}


function fmtUptime(sec) {
  if (sec == null || sec <= 0) return '—';
  const s = Math.floor(sec);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const ss = s % 60;
  if (h > 0) return `${h}h ${m}m ${ss}s`;
  if (m > 0) return `${m}m ${ss}s`;
  return `${ss}s`;
}


// ======================================================================
// 设置面板：相机参数子表单（独立于 /api/config）
// ======================================================================
async function reloadCameraParams() {
  try {
    const r = await fetch('/api/camera/status');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const data = await r.json();
    const params = data.params || {};
    document.querySelectorAll('[data-camera-key]').forEach(el => {
      const key = el.dataset.cameraKey;
      if (!(key in params)) return;
      const v = params[key];
      if (el.type === 'checkbox') el.checked = String(v).toLowerCase() === 'true';
      else el.value = v;
    });
    showToast('相机参数已加载', 'info');
  } catch (e) {
    showToast('读取相机参数失败: ' + e.message, 'error');
  }
}

async function saveCameraParams() {
  const updates = {};
  document.querySelectorAll('[data-camera-key]').forEach(el => {
    const key = el.dataset.cameraKey;
    if (el.type === 'checkbox') updates[key] = el.checked ? 'true' : 'false';
    else if (el.value !== '') updates[key] = el.value;
  });
  try {
    const r = await fetch('/api/camera/params', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(updates),
    });
    const data = await r.json();
    if (!r.ok || data.ok === false) throw new Error(data.error || 'save failed');
    showToast('相机参数已保存（下次启动生效）', 'info');
  } catch (e) {
    showToast('保存失败: ' + e.message, 'error');
  }
}

// 初次进入 settings 视图时也加载相机参数
const _origReloadSettings = reloadSettings;
reloadSettings = async function() {
  await _origReloadSettings();
  await reloadCameraParams();
  loadBgFormFromPrefs();
  detectEnv();  // 自动检测系统环境并填入表单
};


// ======================================================================
// 动态背景管理
// ======================================================================
const BG_PRESETS = {
  default: { type: 'default', mask: 0,  blur: 0 },
  bing:    { type: 'bing',    mask: 25, blur: 0 },
  'unsplash-nature': { type: 'url', url: 'https://picsum.photos/seed/nature1/1920/1080', mask: 30, blur: 0 },
  'unsplash-tech':   { type: 'url', url: 'https://picsum.photos/seed/tech1/1920/1080',   mask: 35, blur: 0 },
  picsum:  { type: 'url', url: 'https://picsum.photos/1920/1080?random=' + Math.floor(Math.random() * 1000), mask: 30, blur: 0 },
};

const BG_DEFAULTS = { type: 'default', url: '', mask: 0, blur: 0 };

function getBgPref() {
  return {
    type: getLocalPref('bg_type', BG_DEFAULTS.type),
    url:  getLocalPref('bg_url',  BG_DEFAULTS.url),
    mask: getLocalPref('bg_mask', BG_DEFAULTS.mask),
    blur: getLocalPref('bg_blur', BG_DEFAULTS.blur),
  };
}

function setBgPref(pref) {
  setLocalPref('bg_type', pref.type);
  setLocalPref('bg_url',  pref.url || '');
  setLocalPref('bg_mask', pref.mask);
  setLocalPref('bg_blur', pref.blur);
}

// 解析背景类型 → 实际 URL
function resolveBgUrl(type, customUrl) {
  switch (type) {
    case 'url':      return customUrl || '';
    case 'bing':     return 'https://bing.biturl.top/?resolution=1920&format=image&index=0&mkt=zh-CN';
    case 'unsplash': return 'https://picsum.photos/1920/1080?random=' + Date.now();
    case 'picsum':   return 'https://picsum.photos/1920/1080?random=' + Date.now();
    default:         return '';
  }
}

// 应用到 DOM
function applyBackground(pref) {
  const canvas = document.getElementById('bg-canvas');
  const img = document.getElementById('bg-image');
  const mask = document.getElementById('bg-mask');
  if (!canvas || !img || !mask) return;

  const url = resolveBgUrl(pref.type, pref.url);

  if (url) {
    // 预加载图片，加载成功才切换
    const probe = new Image();
    probe.onload = () => {
      img.style.backgroundImage = `url("${url}")`;
      img.style.filter = pref.blur > 0 ? `blur(${pref.blur}px)` : '';
      canvas.classList.add('has-image');
      // 蒙版：0 = 透明显示原图，100 = 全白
      const alpha = Math.max(0, Math.min(100, pref.mask)) / 100;
      mask.style.background = `rgba(255, 255, 255, ${alpha.toFixed(3)})`;
    };
    probe.onerror = () => {
      appendLog('[背景] 图片加载失败，回退到默认: ' + url, 'warn');
      img.style.backgroundImage = '';
      canvas.classList.remove('has-image');
      mask.style.background = 'rgba(255, 255, 255, 0)';
    };
    probe.src = url;
  } else {
    // 无图片时：清空图片层 + 透明蒙版 + 移除 has-image，让默认渐变 + 光斑显示
    img.style.backgroundImage = '';
    img.style.filter = '';
    canvas.classList.remove('has-image');
    mask.style.background = 'rgba(255, 255, 255, 0)';
  }
}

// 启动时立即应用
applyBackground(getBgPref());

// 同步表单 ← 偏好
function loadBgFormFromPrefs() {
  const pref = getBgPref();
  const sel = document.getElementById('bg-type-select');
  const urlEl = document.getElementById('bg-url-input');
  const maskEl = document.getElementById('bg-mask-input');
  const blurEl = document.getElementById('bg-blur-input');
  if (sel) sel.value = pref.type;
  if (urlEl) urlEl.value = pref.url || '';
  if (maskEl) {
    maskEl.value = pref.mask;
    setTxt('bg-mask-value', pref.mask + '%');
  }
  if (blurEl) {
    blurEl.value = pref.blur;
    setTxt('bg-blur-value', pref.blur + ' px');
  }
}

// 读取表单 → 偏好对象
function readBgFromForm() {
  return {
    type: document.getElementById('bg-type-select')?.value || 'default',
    url:  document.getElementById('bg-url-input')?.value || '',
    mask: parseInt(document.getElementById('bg-mask-input')?.value || '55', 10),
    blur: parseInt(document.getElementById('bg-blur-input')?.value || '0', 10),
  };
}

function applyBgFromForm() {
  const pref = readBgFromForm();
  applyBackground(pref);
  setBgPref(pref);
  showToast('背景已应用并保存', 'info');
}

function previewBgFromForm() {
  const pref = readBgFromForm();
  applyBackground(pref);
  showToast('已预览（未保存）', 'info');
}

function resetBg() {
  setBgPref({ ...BG_DEFAULTS });
  applyBackground({ ...BG_DEFAULTS });
  loadBgFormFromPrefs();
  document.querySelectorAll('.bg-preset').forEach(b => b.classList.remove('active'));
  showToast('已恢复默认背景', 'info');
}

// 预设按钮 + 滑块实时更新
document.addEventListener('DOMContentLoaded', () => bindBgControls());
bindBgControls(); // 兜底（脚本可能在 DOMContentLoaded 之后才解析）

function bindBgControls() {
  // 滑块标签实时更新
  const maskEl = document.getElementById('bg-mask-input');
  if (maskEl && !maskEl._bound) {
    maskEl._bound = true;
    maskEl.addEventListener('input', () => setTxt('bg-mask-value', maskEl.value + '%'));
  }
  const blurEl = document.getElementById('bg-blur-input');
  if (blurEl && !blurEl._bound) {
    blurEl._bound = true;
    blurEl.addEventListener('input', () => setTxt('bg-blur-value', blurEl.value + ' px'));
  }
  // 预设按钮
  document.querySelectorAll('.bg-preset').forEach(btn => {
    if (btn._bound) return;
    btn._bound = true;
    btn.addEventListener('click', () => {
      const name = btn.dataset.preset;
      const preset = BG_PRESETS[name];
      if (!preset) return;
      const pref = { ...BG_DEFAULTS, ...preset };
      // 写表单
      const sel = document.getElementById('bg-type-select');
      const urlEl = document.getElementById('bg-url-input');
      if (sel)   sel.value = pref.type;
      if (urlEl) urlEl.value = pref.url || '';
      if (maskEl) { maskEl.value = pref.mask; setTxt('bg-mask-value', pref.mask + '%'); }
      if (blurEl) { blurEl.value = pref.blur; setTxt('bg-blur-value', pref.blur + ' px'); }
      // 应用 + 保存
      applyBackground(pref);
      setBgPref(pref);
      // 高亮当前预设
      document.querySelectorAll('.bg-preset').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      showToast(`已应用预设: ${btn.textContent.trim()}`, 'info');
    });
  });
}


// ======================================================================
// 设置页 TOC scrollspy（点击锚点 + 滚动高亮）
// ======================================================================
document.querySelectorAll('.toc-link').forEach(a => {
  a.addEventListener('click', e => {
    e.preventDefault();
    const id = a.getAttribute('href').slice(1);
    const target = document.getElementById(id);
    if (target) {
      // 用 scrollIntoView 平滑滚动
      target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      // 立即高亮
      document.querySelectorAll('.toc-link').forEach(x => x.classList.remove('active'));
      a.classList.add('active');
    }
  });
});

// 滚动监听（高亮当前可见分组）
function updateTocActive() {
  const groups = document.querySelectorAll('.settings-group[id]');
  if (!groups.length) return;
  const offset = 120;  // fixed topbar (~98) + 余量
  let active = null;
  for (const g of groups) {
    const rect = g.getBoundingClientRect();
    if (rect.top - offset <= 0) active = g.id;
  }
  if (!active) active = groups[0].id;
  document.querySelectorAll('.toc-link').forEach(a => {
    a.classList.toggle('active', a.dataset.toc === active);
  });
}
window.addEventListener('scroll', () => {
  if (location.hash === '#settings') updateTocActive();
}, { passive: true });


// =====================================================================
// 上肢调试模块（分组滑块 + 预设 CRUD）
// =====================================================================
const ARM_JOINT_NAMES = [
  // 左臂 5 个
  'L-ShoulderPitch', 'L-ShoulderRoll', 'L-ShoulderYaw',
  'L-Elbow', 'L-WristRoll',
  // 右臂 5 个
  'R-ShoulderPitch', 'R-ShoulderRoll', 'R-ShoulderYaw',
  'R-Elbow', 'R-WristRoll',
  // 腰部 3 个
  'WaistYaw', 'WaistRoll', 'WaistPitch',
];
const ARM_LIMITS = [
  [-2.5, 2.5], [-1.5, 2.0], [-1.5, 1.5], [-1.5, 2.0], [-1.5, 1.5],
  [-2.5, 2.5], [-2.0, 1.5], [-1.5, 1.5], [-1.5, 2.0], [-1.5, 1.5],
  [-1.5, 1.5], [-0.5, 0.5], [-0.5, 0.5],
];
const ARM_GROUPS = [
  { id: 'left',  name: '左臂', icon: 'chevron-left',  start: 0, count: 5 },
  { id: 'right', name: '右臂', icon: 'chevron-right', start: 5, count: 5 },
  { id: 'waist', name: '腰部', icon: 'rotate-cw',       start: 10, count: 3 },
];

let _armDebugRunning = false;

// ---------- 初始化滑块 ----------
function buildArmSliders() {
  ARM_GROUPS.forEach(g => {
    const container = document.getElementById('arm-group-' + g.id);
    if (!container) return;
    container.innerHTML = '';
    for (let i = g.start; i < g.start + g.count; i++) {
      const row = document.createElement('div');
      row.className = 'arm-slider-row';
      row.innerHTML = `
        <span class="arm-slider-label">${ARM_JOINT_NAMES[i]}</span>
        <input type="range" class="arm-slider-input" data-idx="${i}"
               min="${ARM_LIMITS[i][0]}" max="${ARM_LIMITS[i][1]}" step="0.01" value="0" />
        <input type="number" class="arm-slider-val" data-idx="${i}"
               min="${ARM_LIMITS[i][0]}" max="${ARM_LIMITS[i][1]}" step="0.01" value="0" />
      `;
      container.appendChild(row);
    }
  });
  bindArmSliderEvents();
}

function bindArmSliderEvents() {
  // range → number
  document.querySelectorAll('.arm-slider-input').forEach(inp => {
    if (inp._bound) return;
    inp._bound = true;
    inp.addEventListener('input', () => {
      const idx = parseInt(inp.dataset.idx);
      const val = parseFloat(inp.value);
      const num = document.querySelector(`.arm-slider-val[data-idx="${idx}"]`);
      if (num) num.value = val.toFixed(2);
      sendArmAngles();
    });
  });
  // number → range
  document.querySelectorAll('.arm-slider-val').forEach(inp => {
    if (inp._bound) return;
    inp._bound = true;
    inp.addEventListener('change', () => {
      const idx = parseInt(inp.dataset.idx);
      let val = parseFloat(inp.value) || 0;
      const lo = ARM_LIMITS[idx][0], hi = ARM_LIMITS[idx][1];
      val = Math.max(lo, Math.min(hi, val));
      inp.value = val.toFixed(2);
      const rng = document.querySelector(`.arm-slider-input[data-idx="${idx}"]`);
      if (rng) rng.value = val;
      sendArmAngles();
    });
  });
}

// ---------- 读取 / 发送角度 ----------
function readArmAngles() {
  const angles = [];
  for (let i = 0; i < ARM_JOINT_NAMES.length; i++) {
    const el = document.querySelector(`.arm-slider-val[data-idx="${i}"]`);
    angles.push(parseFloat(el?.value) || 0);
  }
  return angles;
}

function sendArmAngles() {
  if (!_armDebugRunning) return;
  const angles = readArmAngles();
  fetch('/api/arm_debug/send', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ angles }),
  }).catch(() => {});
}

// ---------- 预设 ----------
function loadArmPresets() {
  fetch('/api/arm_debug/presets')
    .then(r => r.json())
    .then(data => {
      const sel = document.getElementById('arm-preset-select');
      if (!sel) return;
      // 保留第一个 placeholder
      sel.innerHTML = '<option value="">— 加载预设姿势 —</option>';
      (data.presets || []).forEach(p => {
        const opt = document.createElement('option');
        opt.value = p.key;
        opt.textContent = p.name;
        sel.appendChild(opt);
      });
    })
    .catch(() => {});
}

function applyArmPreset(key) {
  fetch('/api/arm_debug/presets')
    .then(r => r.json())
    .then(data => {
      const p = (data.presets || []).find(x => x.key === key);
      if (!p || !p.angles) return;
      p.angles.forEach((a, i) => {
        const rng = document.querySelector(`.arm-slider-input[data-idx="${i}"]`);
        const num = document.querySelector(`.arm-slider-val[data-idx="${i}"]`);
        if (rng) rng.value = a;
        if (num) num.value = parseFloat(a).toFixed(2);
      });
      if (_armDebugRunning) sendArmAngles();
    })
    .catch(() => {});
}

// ---------- 开始 / 停止 ----------
function armDebugStart() {
  fetch('/api/arm_debug/start', { method: 'POST' })
    .then(r => r.json())
    .then(data => {
      if (!data.ok) { showToast(data.error || '启动失败', 'error'); return; }
      _armDebugRunning = true;
      updateArmStatusUI();
      showToast('上肢调试已启动', 'info');
      // 发送当前角度
      sendArmAngles();
    })
    .catch(() => showToast('启动失败', 'error'));
}

function armDebugStop() {
  fetch('/api/arm_debug/stop', { method: 'POST' })
    .then(r => r.json())
    .then(data => {
      _armDebugRunning = false;
      updateArmStatusUI();
      showToast('上肢调试已停止', 'info');
    })
    .catch(() => {});
}

function updateArmStatusUI() {
  const dot  = document.getElementById('arm-status-dot');
  const txt  = document.getElementById('arm-status-text');
  const btnS = document.getElementById('arm-start-btn');
  const btnT = document.getElementById('arm-stop-btn');
  if (dot) dot.classList.toggle('active', _armDebugRunning);
  if (txt) txt.textContent = _armDebugRunning ? '调试中' : '未启动';
  if (btnS) btnS.classList.toggle('hidden', _armDebugRunning);
  if (btnT) btnT.classList.toggle('hidden', !_armDebugRunning);
}

// ---------- 轮询状态 ----------
function pollArmStatus() {
  fetch('/api/arm_debug/status')
    .then(r => r.json())
    .then(data => {
      const was = _armDebugRunning;
      _armDebugRunning = !!data.running;
      if (was && !_armDebugRunning) {
        // 进程意外退出
        showToast('调试进程已退出', 'warn');
      }
      updateArmStatusUI();
    })
    .catch(() => {});
}

// ---------- 初始化绑定 ----------
(function initArmDebug() {
  buildArmSliders();
  loadArmPresets();

  const btnStart = document.getElementById('arm-start-btn');
  const btnStop  = document.getElementById('arm-stop-btn');
  const selPreset = document.getElementById('arm-preset-select');
  const btnSave  = document.getElementById('arm-preset-save-btn');

  if (btnStart) btnStart.addEventListener('click', armDebugStart);
  if (btnStop)  btnStop.addEventListener('click',  armDebugStop);
  if (selPreset) selPreset.addEventListener('change', () => {
    const key = selPreset.value;
    if (key) applyArmPreset(key);
  });
  if (btnSave) btnSave.addEventListener('click', () => {
    // 简单 prompt 保存当前角度到后端（后端暂不支持，提示即可）
    showToast('保存预设功能需后端支持，当前仅支持加载内置预设', 'warn');
  });

  // 每 3 秒轮询状态
  setInterval(pollArmStatus, 3000);
  updateArmStatusUI();
})();

