// 工作台：左文件树（只读）+ 中 3D 实时跟随 + 下参数/指标/诊断 + 右栏切换。
//
// 数据来自 runner 起的本机服务（sim/host/display.py）：
//   /scene     body 名序、策略名、频率
//   /stream    SSE 每帧 {t, pos, quat, stats}
//   /files /file?path=   只读文件树与内容
//   /command?forward=&yaw=   滑杆 → runner（下一步生效）
import * as THREE from 'three';
// 相对路径：面板里相对 /ducksir-static/client/viewer/，直开时相对 /。绝对 /static/… 在面板里是 404，
// 而 import 失败会让**整个 module 不执行**（整页只剩静态 HTML）。
import { OrbitControls } from './vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/three/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);

// 两种打开方式，端点前缀和静态资源前缀都不同：
//   1) 面板里：页面来自 /ducksir-static/…，实时数据走插件代理 /ducksir-live/…
//   2) `ducksir run` 直接打开：页面由 runner 的显示服务提供，端点在根路径
const BUILD = '2026-11-11 01:10';   // 改这个文件就 bump：诊断页签会显示，用来判断「页面是不是旧的」

const IN_PANEL = location.pathname.startsWith('/ducksir-static/');
const API = IN_PANEL ? '/ducksir-live' : '';
// 静态资源全部走**相对路径**（相对页面所在目录）：面板里是 /ducksir-static/client/viewer/，
// runner 直开时是 `/`（显示服务把 vendor/data/assets 挂在根下）。这样不需要探测、也不会拼错根。
const assets = (rel) => rel.replace(/^\.\//, '');
const glb = (rel) => rel;   // duck-hd1910.json 里的 mesh 本来就是 ../../assets/…

// 启动标记：页面脚本真的跑起来了就先把这行改掉。
// 如果顶栏停在初始文案，说明 module 没执行（importmap/路径问题），而不是 fetch 失败。
// 防呆：这段脚本也可能被别的页面动态 import（比如自检页），那里没有 #conn，
// 不能因为一个标记就把整页炸掉。
const bootConn = document.getElementById('conn');
if (bootConn) bootConn.textContent = '页面已加载';
const canvas = $('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d141c);
scene.add(new THREE.HemisphereLight(0xffffff, 0x404850, 1.05));
const key = new THREE.DirectionalLight(0xffffff, 1.5);
key.position.set(0.6, 1.2, 0.8);
scene.add(key);
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
const controls = new OrbitControls(camera, canvas);
camera.position.set(0.6, 0.4, 0.85);
controls.target.set(0, 0.1, 0);
const grid = new THREE.GridHelper(8, 80, 0x2a333f, 0x1b222a);
scene.add(grid);

const robotGroup = new THREE.Group();
robotGroup.rotation.x = -Math.PI / 2;
scene.add(robotGroup);

const bodyIndex = new Map();
const groups = new Map();
let frames = 0, hz = 0, hzSince = performance.now(), hzFrames = 0;
let pending = null;

function resize() {
  const w = canvas.clientWidth || 1, h = canvas.clientHeight || 1;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);

// ── 3D ────────────────────────────────────────────────────
async function buildRobot() {
  const data = await (await fetch(assets('data/duck-hd1910.json'))).json();
  const loader = new GLTFLoader();
  await Promise.all(data.links.flatMap((link) => (link.geoms || []).map(async (g) => {
    let group = groups.get(link.name);
    if (!group) {
      group = new THREE.Group();
      robotGroup.add(group);
      groups.set(link.name, group);
    }
    const url = glb(g.mesh);
    const gltf = await loader.loadAsync(url);
    const inst = gltf.scene.clone(true);
    inst.position.fromArray(g.xyz);
    const [qw, qx, qy, qz] = g.quat;
    inst.quaternion.set(qx, qy, qz, qw);
    inst.traverse((o) => {
      if (o.isMesh) {
        o.material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(g.color?.[0] ?? 0.7, g.color?.[1] ?? 0.7, g.color?.[2] ?? 0.7),
          metalness: 0.15, roughness: 0.62,
        });
      }
    });
    group.add(inst);
  })));
}

function applyFrame(f) {
  for (const [name, group] of groups) {
    const i = bodyIndex.get(name);
    if (i === undefined) continue;
    const p = i * 3, q = i * 4;
    group.position.set(f.pos[p], f.pos[p + 1], f.pos[p + 2]);
    group.quaternion.set(f.quat[q + 1], f.quat[q + 2], f.quat[q + 3], f.quat[q]);
  }
  frames += 1; hzFrames += 1;
  const now = performance.now();
  if (now - hzSince > 1000) { hz = (hzFrames * 1000) / (now - hzSince); hzSince = now; hzFrames = 0; }
  const s = f.stats || {};
  $('hud-line').innerHTML = `t=${f.t.toFixed(2)}s · 基座 ${fmt(s.base_height)} m · 直立 ${fmt(s.upright)}`
    + ` · ${hz.toFixed(0)} Hz`
    + `<br>速度 ${fmt(s.speed)} m/s · 路程 ${fmt(s.distance)} m`;
  updateMetrics(s);
  $('conn').textContent = '已连接';
}

function loop() {
  resize(); controls.update(); renderer.render(scene, camera); requestAnimationFrame(loop);
}

// ── 指标 / 诊断 ────────────────────────────────────────────
const METRIC_ROWS = [
  ['base_height', '基座高度', 'm'], ['upright', '直立度', ''],
  ['forward', '前进指令', 'm/s'], ['yaw', '转向指令', 'rad/s'],
  ['speed', '实际速度', 'm/s'], ['distance', '路程', 'm'],
  ['rtf', '实时倍率', '×'], ['step', '控制步', ''],
];
function fmt(v) { return typeof v === 'number' ? v.toFixed(3) : '—'; }
let lastStats = {};
function updateMetrics(s) {
  lastStats = s;
  const t = $('metrics');
  t.innerHTML = METRIC_ROWS.map(([k, label, unit]) =>
    `<tr><th>${label}</th><td>${fmt(s[k])}${unit ? ' ' + unit : ''}</td></tr>`).join('');
}

async function loadDiag() {
  // 后端自检（/diag）：页面版本、宿主 pid、PYTHONPATH、请求数、上次错误。
  // 「连不上」时先看这里，比猜快得多。
  try {
    const dg = await (await fetch(`${API}/diag`)).json();
    const rows = [
      ['页面版本', BUILD],
      ['后端 pid / 运行时长', `${dg.pid} · ${dg.uptime_s}s`],
      ['后端 python', dg.python],
      ['PYTHONPATH', (dg.pythonpath || '').split(':')[0] || '（空）'],
      ['已处理请求', dg.requests],
      ['上次错误', dg.last_error ? String(dg.last_error).slice(-160) : '（无）'],
    ];
    $('diag').innerHTML += rows.map(([k, v]) => `<tr><th>${k}</th><td>${v}</td></tr>`).join('');
  } catch (e) {
    $('diag').innerHTML += `<tr><th>后端自检</th><td class="bad">取不到 /diag：${e?.message || e}</td></tr>`;
  }
  const sc = await (await fetch(`${API}/scene`)).json();
  $('diag').innerHTML = [
    ['项目', sc.project || '—'],
    ['策略', sc.policy || '—'],
    ['物理模型', sc.model || '—'],
    ['控制频率', (sc.control_hz ?? '—') + ' Hz'],
    ['物理频率', (sc.physics_hz ?? '—') + ' Hz'],
    ['body 数', (sc.bodies || []).length],
  ].map(([k, v]) => `<tr><th>${k}</th><td>${v}</td></tr>`).join('');
}

// ── 参数滑杆 ─────────────────────────────────────────────
function wireParams() {
  const fw = $('forward'), yw = $('yaw');
  let timer = null;
  const push = () => {
    $('forward-v').textContent = `${Number(fw.value).toFixed(2)} m/s`;
    $('yaw-v').textContent = `${Number(yw.value).toFixed(2)} rad/s`;
    clearTimeout(timer);
    timer = setTimeout(() => {
      fetch(`${API}/command?forward=${fw.value}&yaw=${yw.value}`).catch(() => {});
    }, 60);
  };
  fw.addEventListener('input', push);
  yw.addEventListener('input', push);
  addEventListener('keydown', (e) => {
    const k = e.key.toLowerCase();
    if (k === 'w') { fw.value = Math.min(0.25, Number(fw.value) + 0.05); push(); }
    else if (k === 's') { fw.value = Math.max(-0.2, Number(fw.value) - 0.05); push(); }
    else if (k === 'a') { yw.value = Math.max(-1, Number(yw.value) - 0.1); push(); }
    else if (k === 'd') { yw.value = Math.min(1, Number(yw.value) + 0.1); push(); }
  });
}

// ── 文件树（只读）─────────────────────────────────────────
let fileList = [];          // 当前可展示的文件（顺序同左侧树）

/** 显示一个文件（默认第一个）。点右栏「代码」但没有选过文件时用它。 */
async function showFile(rel) {
  const code = $('code'), view = $('view');
  const target = rel || fileList[0];
  if (!target) {
    code.textContent = '（这个项目没有可展示的源文件）';
    code.classList.add('on');
    view.classList.add('off');
    return;
  }
  document.querySelectorAll('#files button').forEach((b) => b.classList.toggle('on', b.dataset.path === target));
  const res = await fetch(`${API}/file?path=` + encodeURIComponent(target));
  code.textContent = res.ok ? await res.text() : `（读不到 ${target}）`;
  code.classList.add('on');
  view.classList.add('off');
  code.scrollTop = 0;
  code.scrollLeft = 0;
}

async function wireFiles() {
  const box = $('files');
  const listing = await (await fetch(`${API}/files`)).json();
  fileList = listing.files || [];
  const code = $('code'), view = $('view');
  box.querySelectorAll('button').forEach((b) => b.remove());
  for (const rel of listing.files || []) {
    const btn = document.createElement('button');
    btn.dataset.path = rel;
    btn.innerHTML = `${rel.split('/').pop()}<span class="ext">${rel.split('.').pop()}</span>`;
    btn.onclick = async () => {
      await showFile(rel);       // 统一走 showFile：高亮、取内容、切视图、滚回顶部
      setRail('code');
    };
    box.appendChild(btn);
  }
}

// ── 右栏 / 标签页 ────────────────────────────────────────
function setRail(name) {
  document.querySelectorAll('#rail button').forEach((b) => b.classList.toggle('on', b.dataset.rail === name));
  if (name === 'code') {
    // 之前这里直接 return —— 点「代码」什么都不发生。现在切到代码视图，
    // 没选过文件就自动展示第一个（否则是一片空白，看着也像「没反应」）。
    if (!$('code').classList.contains('on')) showFile().catch(() => {});
    return;
  }
  $('view').classList.remove('off');
  $('code').classList.remove('on');
}
function wireTabs() {
  document.querySelectorAll('#tabs button').forEach((b) => {
    b.onclick = () => {
      document.querySelectorAll('#tabs button').forEach((x) => x.classList.toggle('on', x === b));
      document.querySelectorAll('.tabpane').forEach((p) => p.classList.toggle('on', p.id === 'pane-' + b.dataset.tab));
    };
  });
  document.querySelectorAll('#rail button').forEach((b) => {
    b.onclick = () => {
      setRail(b.dataset.rail);
      const tab = { metrics: 'metrics', diag: 'diag', view: 'params', code: 'params' }[b.dataset.rail];
      document.querySelector(`#tabs button[data-tab="${tab}"]`)?.click();
    };
  });
}

// 3D 区的消息层：出错时把原因写在画面上（手机看不到控制台）。
// 注意：它必须声明在**第一次使用之前**，否则会撞 TDZ（`Cannot access 'msg' before initialization`）。
function showMsg(text) {
  const el = $('view-msg');
  if (!el) return;
  el.textContent = text;
  el.style.display = text ? 'grid' : 'none';
}

// ── bundle 浏览器执行：把编译产物交给 play.html（wasm 核 + ORT Web + MuJoCo wasm）──
// 这条路径**不需要宿主进程**：Python 只在编译期出现，运行期只吃 bundle。
async function wireBundles() {
  const sel = $('bundle');
  const frame = $('run-frame');
  const canvas = $('c');
  let list = [];
  try {
    const res = await (await fetch(`${API}/bundles`)).json();
    list = res.bundles || [];
  } catch { /* 宿主没重启时端点不存在，忽略 */ }
  sel.innerHTML = '';
  if (!list.length) {
    sel.innerHTML = '<option value="">（还没有 bundle，先 ducksir build）</option>';
    return;
  }
  for (const b of list) {
    const opt = document.createElement('option');
    opt.value = b.base;
    opt.textContent = b.name;
    sel.appendChild(opt);
  }
  // 手机默认就用浏览器执行：不起子进程、不占宿主、画面最顺
  const isPhone = matchMedia('(max-width: 820px)').matches;
  if (isPhone && list.length) runInBrowser(list[0].base);
  $('btn-browser-run').onclick = () => runInBrowser(sel.value);

  async function runInBrowser(base) {
    if (!base) return;
    showMsg('');
    // 先探一次产物：bundle 目录能不能读、有没有 contract.json。
    // 失败就把原因写在画面上，而不是留一块黑屏（手机上看不到控制台）。
    try {
      const probe = await fetch(base + 'contract.json');
      if (!probe.ok) {
        showMsg(`bundle 读不到：${base}contract.json（HTTP ${probe.status}）`);
        $('conn').textContent = 'bundle 不可用';
        return;
      }
    } catch (e) {
      showMsg('bundle 探测失败：' + (e?.message || e));
      return;
    }
    frame.src = `/ducksir-static/client/viewer/play.html?bundle=${encodeURIComponent(base)}&embed=1`;
    frame.style.display = 'block';
    canvas.style.display = 'none';
    document.body.classList.add('browser-mode');   // 收起 host 专属控件
    $('conn').textContent = '浏览器内执行';

    // 起来之后把 iframe 里那页的自报状态（HUD 第一行）转述到顶栏，
    // 这样「跑没跑起来」不用猜。
    frame.onload = () => {
      setTimeout(() => {
        try {
          const doc = frame.contentDocument;
          const stat = doc && doc.getElementById('stat');
          const busy = doc && doc.getElementById('busy');
          if (busy && busy.style.display !== 'none' && busy.textContent) {
            $('conn').textContent = '载入中：' + busy.textContent.slice(0, 28);
          } else if (stat && stat.textContent) {
            $('conn').textContent = stat.textContent.slice(0, 32);
          }
        } catch { /* 跨源就跳过 */ }
      }, 2500);
    };
  }
  $('btn-back').onclick = () => {
    frame.style.display = 'none';
    canvas.style.display = 'block';
    document.body.classList.remove('browser-mode');
    $('conn').textContent = '未连接';
  };
}

// ── 启动 / 停止（面板里用插件代理；直开时调 runner 自己的根路径）──
async function refreshConn() {
  if (document.body.classList.contains('browser-mode')) return false;   // 浏览器模式不关心宿主动态
  try {
    const st = await (await fetch(`${API}/status`)).json();
    $('conn').textContent = st.running ? `运行中 :${st.port}` : '未运行';
    $('conn').className = 'pill ' + (st.running ? 'ok' : '');
    return st.running;
  } catch {
    $('conn').textContent = '连不上后端（点「启动仿真」或面板「重载」）';
    return false;
  }
}

function wireRunControls() {
  $('btn-start').onclick = async () => {
    const project = $('project-path').value.trim() || 'sim/test/fixtures/project_demo.py';
    $('conn').textContent = '启动中…';
    const res = await fetch(`${API}/start`, {
      method: 'POST', headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ project, seconds: 600 }),
    });
    const out = await res.json();
    if (!out.ok) { $('conn').textContent = '启动失败：' + (out.error || ''); return; }
    await refreshConn();
    startStream();
  };
  $('btn-stop').onclick = async () => {
    await fetch(`${API}/stop`, { method: 'POST' });
    await refreshConn();
  };
}

let streamSrc = null;
function startStream() {
  if (streamSrc) streamSrc.close();
  streamSrc = new EventSource(`${API}/stream`);
  streamSrc.onmessage = (ev) => { try { applyFrame(JSON.parse(ev.data)); } catch (e) { console.warn(e); } };
  streamSrc.onerror = () => { $('conn').textContent = '断开了'; };
}

async function main() {
  wireTabs();
  wireParams();
  wireRunControls();
  wireBundles();

  // 先起渲染循环：即使模型加载失败，网格和 HUD 也要出来，不能「黑屏 = 看起来没反应」
  loop();

  const fail = (label) => (e) => {
    $('conn').textContent = `${label}失败：${e?.message || e}`;
    console.warn(label, e);
  };

  try {
    const running = document.body.classList.contains('browser-mode') ? false : await refreshConn();
    const sc = await (await fetch(`${API}/scene`)).json();
    (sc.bodies || []).forEach((n, i) => bodyIndex.set(n, i));
    if (running) startStream();
    await loadDiag().catch(fail('诊断'));
  } catch (e) { fail('连接')(e); }

  // 模型与文件树各自兜底：一个坏了不影响另一个
  try {
    await buildRobot();
  } catch (e) { fail('模型加载')(e); }
  try {
    await wireFiles();
  } catch (e) { fail('文件树')(e); }
}

main();
