// 工作台：左文件树（只读）+ 中 3D 实时跟随 + 下参数/指标/诊断 + 右栏切换。
//
// 数据来自 runner 起的本机服务（sim/host/display.py）：
//   /scene     body 名序、策略名、频率
//   /stream    SSE 每帧 {t, pos, quat, stats}
//   /files /file?path=   只读文件树与内容
//   /command?forward=&yaw=   滑杆 → runner（下一步生效）
import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/three/controls/OrbitControls.js';
import { GLTFLoader } from '/static/vendor/three/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);

// 两种打开方式，端点前缀和静态资源前缀都不同：
//   1) 面板里：页面来自 /ducksir-static/…，实时数据走插件代理 /ducksir-live/…
//   2) `ducksir run` 直接打开：页面由 runner 的显示服务提供，端点在根路径
const IN_PANEL = location.pathname.startsWith('/ducksir-static/');
const API = IN_PANEL ? '/ducksir-live' : '';
const ASSET = IN_PANEL ? '/ducksir-static/client/viewer' : '/static';
const assets = (rel) => `${ASSET}/${rel.replace(/^\.\//, '')}`;
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
    const url = assets(g.mesh.replace('../../assets/', 'assets/').replace('../assets/', 'assets/'));
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
    + ` · ${hz.toFixed(0)} Hz`;
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
async function wireFiles() {
  const box = $('files');
  const listing = await (await fetch(`${API}/files`)).json();
  const code = $('code'), view = $('view');
  box.querySelectorAll('button').forEach((b) => b.remove());
  for (const rel of listing.files || []) {
    const btn = document.createElement('button');
    btn.dataset.path = rel;
    btn.innerHTML = `${rel.split('/').pop()}<span class="ext">${rel.split('.').pop()}</span>`;
    btn.onclick = async () => {
      box.querySelectorAll('button').forEach((b) => b.classList.remove('on'));
      btn.classList.add('on');
      const res = await fetch(`${API}/file?path=` + encodeURIComponent(rel));
      const text = await res.text();
      code.textContent = text;
      code.classList.add('on');
      view.classList.add('off');
      setRail('code');
    };
    box.appendChild(btn);
  }
}

// ── 右栏 / 标签页 ────────────────────────────────────────
function setRail(name) {
  document.querySelectorAll('#rail button').forEach((b) => b.classList.toggle('on', b.dataset.rail === name));
  if (name === 'code') return;
  if (name === 'view') {
    $('view').classList.remove('off');
    $('code').classList.remove('on');
  }
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

// ── 启动 / 停止（面板里用插件代理；直开时调 runner 自己的根路径）──
async function refreshConn() {
  try {
    const st = await (await fetch(`${API}/status`)).json();
    $('conn').textContent = st.running ? `运行中 :${st.port}` : '未运行';
    $('conn').className = 'pill ' + (st.running ? 'ok' : '');
    return st.running;
  } catch {
    $('conn').textContent = '连不上服务';
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
  try {
    const running = await refreshConn();
    const sc = await (await fetch(`${API}/scene`)).json();
    (sc.bodies || []).forEach((n, i) => bodyIndex.set(n, i));
    await buildRobot();
    await wireFiles();
    await loadDiag();
    loop();
    if (running) startStream();
  } catch (e) {
    $('conn').textContent = '连不上服务：' + (e?.message || e);
  }
}

main();
